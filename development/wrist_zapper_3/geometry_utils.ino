/* 
 *  A set of utility functions for various 3D geometry operations
 *  Requires BasicLinearAlgebra.h and Quaternion.h
 */

/* 
 *  Returns the magnitude of a 3-dimensional vector
 */
float norm(BLA::Matrix<3> vec) {
  return sqrt(vec(0)*vec(0) + vec(1)*vec(1) + vec(2)*vec(2));
}

/* 
 *  Returns the normalized version of a specific column from rotation matrix
 */
BLA::Matrix<3> getNormalizedCol(BLA::Matrix<3,3> rotm, char col) {
  BLA::Matrix<3> ret;
  ret = {rotm(0,col), rotm(1,col), rotm(2,col)};
  ret /= norm(ret);
  return ret;
}

/* 
 *  Returns the cross product of two 3-dimensional vectors
 */
BLA::Matrix<3> cross(BLA::Matrix<3> a, BLA::Matrix<3> b)
{
    BLA::Matrix<3> ret;

    ret(0) = a(1) * b(2) - a(2) * b(1);
    ret(1) = a(2) * b(0) - a(0) * b(2);
    ret(2) = a(0) * b(1) - a(1) * b(0);

    return ret;
}

/* 
 *  Returns the dot product of two 3-dimensional vectors
 */
float dot(BLA::Matrix<3> a, BLA::Matrix<3> b)
{
    return a(0)*b(0) + a(1)*b(1) + a(2)*b(2);
}

/* 
 *  Calculates the extension angle between arm and wrist IMUs
 *  - Assumes mounting with x distal, y left, z dorsal
 *  1. Convert quaternions to rotation matrices (which are direction vectors) 
 *  2. Extract and normalise direction vectors
 *  3. #TODO finish this explanation
 */
float calcExtensionAngle(Quaternion q_arm, Quaternion q_hand) 
{

//// -- SETUP -- ////

    // convert arm quaternion to rotation matrix
    BLA::Matrix<3,3> arm_rot = q_arm.to_rotation_matrix();
//    Serial << "ret(2,0): " << arm_rot << '\n'; 

    // pull direction vectors from rotation matrix
    BLA::Matrix<3> arm_x = getNormalizedCol(arm_rot, 0);
    BLA::Matrix<3> arm_y = getNormalizedCol(arm_rot, 1);
    BLA::Matrix<3> arm_z = getNormalizedCol(arm_rot, 2);

    // convert hand quaternion to rotation matrix
    BLA::Matrix<3,3> hand_rot = q_hand.to_rotation_matrix();
//    Serial << "ret(2,0): " << arm_rot << '\n';

    // pull normalized direction vectors from rotation matrix
    BLA::Matrix<3> hand_x = getNormalizedCol(hand_rot, 0);
    BLA::Matrix<3> hand_y = getNormalizedCol(hand_rot, 1);
    BLA::Matrix<3> hand_z = getNormalizedCol(hand_rot, 2);

//// -- FIND PLANES DESCRIBING THE EXTENSION ANGLE -- ////
    
    // find normal of arm "vertical" plane
    BLA::Matrix<3> n_vec_1 = arm_y; // normal of arm plane

    // find normal of hand extension plane
    BLA::Matrix<3> n_vec_2 = cross(hand_x,arm_y); // find normal of hand extension plane
    n_vec_2 /= norm(n_vec_2); // normalise
    
    BLA::Matrix<3> v_intersection = cross(n_vec_1,n_vec_2); // find vector of intersection of planes
    v_intersection /= norm(v_intersection); // normalise

//// -- CALCULATE THE EXTENSION ANGLE -- ////

    float angle = acos(dot(arm_x, v_intersection))*360/(2*PI);

    // find sign
    float sign = 1;
    BLA::Matrix<3> cross_prod = cross(arm_x, v_intersection);
    if (dot(arm_y, cross_prod) > 0) { 
      sign = -1;
    }
    
    return sign * angle;  
}
