#ifndef ROBOT_MOTION_GENERATOR___ANGULAR_VELOCITY_H_
#define ROBOT_MOTION_GENERATOR___ANGULAR_VELOCITY_H_

#include <eigen3/Eigen/Dense>

namespace motion{

// -- DQ2W Converts Quaterion rates to angular velocity --//
template<typename _Scalar>
inline Eigen::Matrix<_Scalar,3,1> d2qw(const Eigen::Quaternion<_Scalar>&  q,const  Eigen::Quaternion<_Scalar>&  dq){
    Eigen::Matrix<_Scalar,3,1> w;
    Eigen::Matrix<_Scalar,3,4> W;
    Eigen::Matrix<_Scalar,4,1> dq_tmp;
    W << -q.x(),  q.w(), -q.z(),  q.y(),
         -q.y(),  q.z(),  q.w(), -q.x(),
         -q.z(), -q.y(),  q.x(),  q.w();


    dq_tmp(0) = dq.w();
    dq_tmp(1) = dq.x();
    dq_tmp(2) = dq.y();
    dq_tmp(3) = dq.z();

    w = 2 * W * dq_tmp;

    return w;
}

}

#endif
