import trimesh
import numpy as np
import os
import time
from CGAL.CGAL_Kernel import Point_3, Triangle_3
from CGAL.CGAL_AABB_tree import AABB_tree_Triangle_3_soup
from dataset import GraspWebDatasetReader  # Assuming this file exists

# =================================================================
# SCORING CONFIGURATION
# =================================================================
# --- Weights for the positive score calculation
W_NORMAL = 15.0  # How important it is for contact normals to oppose each other
W_DISTANCE = 5.0  # How important the distance between contact points is
W_CLEARANCE = 10.0  # How important clearance from the ground plane is

# --- Penalties for invalid grasps (large negative values)
PENALTY_COLLISION = -1000.0
PENALTY_CLEARANCE = -1000.0
PENALTY_UNSTABLE = -500.0  # For grasps that don't make proper contact

# --- Bonus for partial contact
SCORE_PARTIAL_CONTACT = 5.0 # A small, fixed bonus for making at least one contact

# --- Parameters for checks
GROUND_PLANE_Z = 0.0
OUTPUT_FILE = "grasp_ranking_live.txt"


# =================================================================
# HELPER FUNCTIONS
# =================================================================

def trimesh_to_cgal_triangles(mesh: trimesh.Trimesh) -> list[Triangle_3]:
    """Converts a Trimesh object into a list of CGAL Triangle_3 objects."""
    triangles = []
    for face in mesh.faces:
        p1_coords, p2_coords, p3_coords = mesh.vertices[face]
        p1 = Point_3(p1_coords[0], p1_coords[1], p1_coords[2])
        p2 = Point_3(p2_coords[0], p2_coords[1], p2_coords[2])
        p3 = Point_3(p3_coords[0], p3_coords[1], p3_coords[2])
        triangles.append(Triangle_3(p1, p2, p3))
    return triangles


def get_successful_grasps(dataset_path: str, gripper_name: str, object_uuid: str) -> list[np.ndarray]:
    """Reads grasp data and returns a list of successful grasp poses."""
    webdataset_reader = GraspWebDatasetReader(os.path.join(dataset_path, gripper_name))
    try:
        grasp_data = webdataset_reader.read_grasps_by_uuid(object_uuid)
        if grasp_data is None: return []
        grasp_poses = np.array(grasp_data["grasps"]["transforms"])
        grasp_mask = np.array(grasp_data["grasps"]["object_in_gripper"])
        return [grasp for grasp in grasp_poses[grasp_mask]]
    except Exception as e:
        print(f"Error reading grasps for {object_uuid}: {e}")
        return []


def update_output_file(ranked_grasps: list, num_to_display: int = 10):
    """Writes the current top grasps to the output file."""
    with open(OUTPUT_FILE, "w") as f:
        f.write(f"Grasp Ranking (Live Update @ {time.strftime('%H:%M:%S')})\n")
        f.write("-" * 50 + "\n")
        for i, grasp_data in enumerate(ranked_grasps[:num_to_display]):
            f.write(f"Rank {i + 1:02d}: Score = {grasp_data['score']:.4f} (Grasp Index: {grasp_data['id']})\n")


# =================================================================
# CORE SCORING LOGIC
# =================================================================

def calculate_grasp_score(
        grasp_pose: np.ndarray,
        gripper_mesh: trimesh.Trimesh,
        object_mesh: trimesh.Trimesh,
        object_tree: AABB_tree_Triangle_3_soup
) -> float:
    """Calculates a quality score for a given grasp pose using a penalty system."""
    total_score = 0.0
    gripper_at_pose = gripper_mesh.copy()
    gripper_at_pose.apply_transform(grasp_pose)

    # --- 1. Collision Check ---
    gripper_cgal_triangles = trimesh_to_cgal_triangles(gripper_at_pose)
    if any(object_tree.do_intersect(tri) for tri in gripper_cgal_triangles):
        total_score += PENALTY_COLLISION

    # --- 2. Clearance Check ---
    min_gripper_z = gripper_at_pose.bounds[0][2]
    if min_gripper_z < GROUND_PLANE_Z:
        total_score += PENALTY_CLEARANCE

    # If score is already heavily penalized, no need to check stability
    if total_score < -1:
        return total_score

    # --- 3. Stability Analysis (Contact Points & Normals) ---
    ray_origins_local = np.array([[0.0, 0.06, 0.0], [0.0, -0.06, 0.0]])
    ray_directions_local = np.array([[0.0, -1.0, 0.0], [0.0, 1.0, 0.0]])

    ray_origins_world = trimesh.transform_points(ray_origins_local, grasp_pose)
    ray_directions_world = trimesh.transform_points(ray_directions_local, grasp_pose, translate=False)

    locations, index_ray, index_tri = object_mesh.ray.intersects_location(
        ray_origins=ray_origins_world, ray_directions=ray_directions_world
    )

    # ==========================================================
    # === MODIFIED LOGIC: Grade the contact instead of pass/fail ===
    # ==========================================================
    if len(locations) == 2:
        # IDEAL CASE: Two contacts found, calculate a full, detailed score.
        contact_p1, contact_p2 = locations
        normal_p1 = object_mesh.face_normals[index_tri[0]]
        normal_p2 = object_mesh.face_normals[index_tri[1]]

        normal_score = max(0.0, -np.dot(normal_p1, normal_p2))
        distance_score = np.linalg.norm(contact_p1 - contact_p2)
        clearance_score = min_gripper_z

        positive_score = (W_NORMAL * normal_score) + (W_DISTANCE * distance_score) + (W_CLEARANCE * clearance_score)
        total_score += positive_score

    elif len(locations) == 1:
        # GOOD ENOUGH CASE: One contact found. Give a small, fixed bonus.
        total_score += SCORE_PARTIAL_CONTACT

    else:  # len(locations) == 0
        # WORST CASE: A complete miss. Apply the instability penalty.
        total_score += PENALTY_UNSTABLE
    # ==========================================================

    return total_score


# =================================================================
# MAIN EXECUTION
# =================================================================

if __name__ == "__main__":
    # --- Configuration ---
    OBJECT_UUID = "000074a334c541878360457c672b6c2e"
    GRIPPER_NAME = "robotiq_2f_140"
    BASE_PATH = "/home/amine-ki6"

    # --- Paths ---
    object_path = os.path.join(BASE_PATH, 'CGAL', f'{OBJECT_UUID}.glb')
    gripper_folder_path = os.path.join(BASE_PATH, 'CGAL', 'prev_gripper')
    dataset_path = os.path.join(BASE_PATH, 'grasp', 'grasp_data')

    # --- Load Assets ---
    print("Loading assets...")
    object_mesh = trimesh.load_mesh(object_path)
    stl_files = [f for f in os.listdir(gripper_folder_path) if f.endswith('.stl')]
    gripper_parts = [trimesh.load_mesh(os.path.join(gripper_folder_path, f)) for f in stl_files]
    combined_gripper = trimesh.util.concatenate(gripper_parts)
    combined_gripper.apply_scale(0.001)

    successful_grasp_list = get_successful_grasps(dataset_path, GRIPPER_NAME, OBJECT_UUID)
    if not successful_grasp_list:
        print("No successful grasps found. Exiting.")
        exit()
    print(f"Retrieved {len(successful_grasp_list)} candidate grasps.")

    # --- Prepare for Collision Checking & Ranking ---
    print("Building AABB tree...")
    object_cgal_triangles = trimesh_to_cgal_triangles(object_mesh)
    tree_object = AABB_tree_Triangle_3_soup(object_cgal_triangles)

    # --- Evaluate, Rank, and Continuously Output ---
    print(f"\nStarting grasp evaluation. Watch '{OUTPUT_FILE}' for live results.")
    ranked_grasps = []
    total_grasps = len(successful_grasp_list)

    # Initialize the output file
    update_output_file([])

    for i, grasp_pose in enumerate(successful_grasp_list):
        print(f"Processing grasp {i + 1}/{total_grasps}...", end='\r')
        score = calculate_grasp_score(
            grasp_pose=grasp_pose,
            gripper_mesh=combined_gripper,
            object_mesh=object_mesh,
            object_tree=tree_object
        )

        ranked_grasps.append({'id': i, 'pose': grasp_pose, 'score': score})

        # Sort the list and update the output file after each evaluation
        ranked_grasps.sort(key=lambda x: x['score'], reverse=True)
        update_output_file(ranked_grasps, num_to_display=30)

    print("\n\nEvaluation complete.")
    print(f"Final ranking has been saved to '{OUTPUT_FILE}'.")