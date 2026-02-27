import itertools
import os

from setuptools import find_packages, setup

package_name = "giskardpy_ros"

data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
]
for dirpath, _, filenames in itertools.chain(os.walk("data"), os.walk("launch")):
    full_paths = [os.path.join(dirpath, f) for f in filenames]
    install_path = os.path.join("share", package_name, dirpath)
    data_files.append((install_path, full_paths))

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Simon Stelter",
    maintainer_email="stelter@uni-bremen.de",
    description="TODO: Package description",
    license="LGPLv3",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "generic_giskard = giskardpy_ros.scripts.generic_giskard:main",
            "pr2_standalone = giskardpy_ros.scripts.iai_robots.pr2.pr2_standalone:main",
            "hsr_standalone = giskardpy_ros.scripts.iai_robots.hsr.hsr_standalone:main",
            "hsr_velocity = giskardpy_ros.scripts.iai_robots.hsr.iai_hsr_real_time:main",
            "hsr_trust_me_bro = giskardpy_ros.scripts.iai_hsr_real_time_trust_me_bro:main",
            "r6bot = giskardpy_ros.scripts.other_robots.test.r6bot:main",
            "generic_giskard_standalone = giskardpy_ros.scripts.generic_giskard_standalone:main",
            "interactive_marker = giskardpy_ros.scripts.tools.interactive_marker:main",
            "motion_statechart_inspector = giskardpy_ros.scripts.tools.motion_statechart_inspector:main",
            "tracy_standalone = giskardpy_ros.scripts.tracy_standalone:main",
            "tracy_velocity = giskardpy_ros.scripts.tracy_velocity:main",
            "tiago_velocity = scripts.tiago_velocity:main",
            "armar_velocity = scripts.armar7_velocity:main",
            "ur5_velocity = scripts.ur5_velocity:main",
        ],
    },
)
