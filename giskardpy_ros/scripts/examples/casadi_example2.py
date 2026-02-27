import semantic_digital_twin.spatial_types.spatial_types as cas

a_T_b = cas.TransformationMatrix()
b_T_c = cas.TransformationMatrix()

c_P_x = cas.Point3()
c_V_x = cas.Vector3()

a_T_c = a_T_b.dot(b_T_c)
a_P_x = a_T_c.dot(c_P_x)
a_V_x = a_T_c.dot(c_V_x)
