#ifndef ARM_PLANNING_UTILS_H
#define ARM_PLANNING_UTILS_H

#include <iostream>
#include <chrono>
#include <random>
#include <cmath>
#include <memory>
#include <algorithm>
#include <assert.h>
#include <iomanip>
#include <complex>

#include <Eigen/Core>
#include <Eigen/Dense> 
#include <Eigen/Geometry>


typedef std::vector<Eigen::Vector3d, 
        Eigen::aligned_allocator<Eigen::Vector3d> > VecEigenVec3;


class Utils{
public:

// ------------------------------------------------------------
static Eigen::VectorXd stdVector2EigenVectorXd(
    const std::vector<double> vector){ 

    Eigen::VectorXd eigen(vector.size());
    for(int i = 0; i < vector.size(); i++){
        eigen(i) = vector[i];
    }
    return eigen;
}


// ------------------------------------------------------------
static std::vector<double> eigenVectorXd2StdVector(
    const Eigen::VectorXd &eigen)
{ 
    std::vector<double> vector(eigen.rows());
    for(int i = 0; i < vector.size(); i++){
        vector[i] = eigen(i);
    }
    return vector;
}


// ------------------------------------------------------------
static std::vector<double> eigenVector2StdVector(
    const Eigen::Vector3d &eigen){ 

    std::vector<double> vector(3);
    for(int i = 0; i < vector.size(); i++){
        vector[i] = eigen(i);
    }
    return vector;
}

// ------------------------------------------------------------
static std::vector<double> eigenVector2StdVector(
    const Eigen::Quaterniond &eigen){ 

    std::vector<double> vector(4);
    for(int i = 0; i < vector.size(); i++){
        vector[i] = eigen.coeffs()(i); // x, y, z, w
    }
    return vector;
}

// ------------------------------------------------------------
static std::vector<double> eigenMatrix2StdVector(
    const Eigen::MatrixXd &eigen){ 

    std::vector<double> vector(eigen.size());
    int k = 0;
    for(int i = 0; i < eigen.rows(); i++){
        for(int j = 0; j < eigen.cols(); j++){
            vector[k] = eigen(i, j);
            k++;
        }
    }
    return vector;
}

// ------------------------------------------------------------
static std::vector<float> stdVectorDouble2Float(
        const std::vector<double> &d)
{ 
    std::vector<float> f(d.begin(), d.end());
    return f;
}


// ------------------------------------------------------------
static VecEigenVec3 eigenVectorXd2StdVecEigenVec3(const Eigen::VectorXd &vec)
{
    assert(vec.size() % 3 == 0);
    int num_fps = vec.size() / 3;
    VecEigenVec3 new_vec(num_fps);
    for(int i = 0; i < num_fps; i++){
        new_vec[i] = vec.block<3, 1>(3 * i, 0);
    }
    return new_vec;
}


// ------------------------------------------------------------
static Eigen::VectorXd stdVecEigenVec3ToEigenVectorXd(const VecEigenVec3 &std_vec)
{
    int num_fps = std_vec.size();

    Eigen::VectorXd new_vec(num_fps * 3);
    for(int i = 0; i < num_fps; i++){
        new_vec.block<3, 1>(3 * i, 0) = std_vec[i];
    }
    return new_vec;
}


// ------------------------------------------------------------
static Eigen::Isometry3d stdPosQuatVec2Isometry3d(
        const std::vector<double> &pos,
        const std::vector<double> &quat // [x, y, z, w]
){
    return EigenPosQuatVec2Isometry3d(
                stdVector2EigenVectorXd(pos),
                stdVector2EigenVectorXd(quat));
}


// ------------------------------------------------------------
static Eigen::Isometry3d EigenPosQuatVec2Isometry3d(
        const Eigen::Vector3d &pos,
        const Eigen::Vector4d &quat // [x, y, z, w]
){
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    // normalize the quaternion to avoid invalid input quaternion
    Eigen::Quaternion<double> eigen_quat = Eigen::Quaternion<double>(quat).normalized(); // initialized from 4D vector [x, y, z, w]
    T.rotate(eigen_quat);
    T.pretranslate(pos);
    return T;
}


// ------------------------------------------------------------
static double twoVecAngle(
                const Eigen::Vector3d &vec0,
                const Eigen::Vector3d &vec1)
{
    return std::atan2(vec0.cross(vec1).norm(), vec0.dot(vec1));
}


// ------------------------------------------------------------
template <typename T>
static void coutStdVector(
        const std::vector<T> &vec)
{
    for(auto &e: vec){
        std::cout << e << " ";
    }
    std::cout << std::endl;
}


// ------------------------------------------------------------
static double meanStdVec(
        const std::vector<double> &vec
){
    double sum = 0.0;
    for(auto &e: vec){
        sum += e;
    }
    return sum / vec.size();
}


// ------------------------------------------------------------
static void coutList(
    const std::string &vec_name,
    const Eigen::VectorXd &vec
){
    std::cout << vec_name << " = [";
    for(int i = 0; i < vec.size(); i++){
        std::cout << std::setprecision(4) << vec(i) << ", ";
    }
    std::cout << "]" << std::endl;
}


// ------------------------------------------------------------
static void pushBackEigenVecToStdVec(
    std::vector<double> &vec,
    const Eigen::VectorXd &eigen_vec
){
    for(int j = 0; j < eigen_vec.size(); j++){
        vec.push_back(eigen_vec(j));
    }  
}


// ------------------------------------------------------------
static double getRandomDouble(
    double lb = 0.0, 
    double ub = 1.0, 
    int s = -1)
{
    if(s < 0){
        std::random_device rd;
        std::default_random_engine seed(rd());
        std::uniform_real_distribution<> uniform(lb, ub);
        return uniform(seed);
    }else{
        std::default_random_engine seed(s);
        std::uniform_real_distribution<> uniform(lb, ub);
        return uniform(seed);
    }
}


// ------------------------------------------------------------
static Eigen::MatrixXd pseudoInverse(
    const Eigen::MatrixXd& a, 
    double epsilon = std::numeric_limits<double>::epsilon()
){
    Eigen::JacobiSVD< Eigen::MatrixXd > svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(a.cols(), a.rows()) * svd.singularValues().array().abs()(0);
    return svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}


// ------------------------------------------------------------
static Eigen::MatrixXd getNullSpaceProjectionMatrix(
    const Eigen::MatrixXd &A
){
    return Eigen::MatrixXd::Identity(A.cols(), A.cols()) - pseudoInverse(A) * A;
}


/**
 * @brief rpy: X-Y-Z fixed-axis angle
*/
static Eigen::VectorXd isometryToPosAndRPYAngle(
    const Eigen::Isometry3d &pose
){
    Eigen::VectorXd pose_vec(6);
    pose_vec.block<3, 1>(0, 0) = pose.translation();
    
    Eigen::Matrix3d rot_mat = pose.rotation();
    pose_vec(3) = std::atan2(rot_mat(2, 1), rot_mat(2, 2));
    pose_vec(4) = -std::asin(rot_mat(2, 0));
    pose_vec(5) = std::atan2(rot_mat(1, 0), rot_mat(0, 0));

    return pose_vec;
}


/**
 * @brief rpy: X-Y-Z fixed-axis angle
*/
static Eigen::Matrix3d matrixRelateAngularVelToRPYVel(
    const Eigen::Vector3d &rpy_vec
){
    using namespace std;

    double r = rpy_vec(0);
    double p = rpy_vec(1);
    double y = rpy_vec(2);

    Eigen::Matrix3d matrix;
    matrix << cos(y) / cos(p),          sin(y) / cos(p),          0.0,
              -sin(y),                  cos(y),                   0.0,
              cos(y) * sin(p) / cos(p), sin(y) * sin(p) / cos(p), 1.0;

    return matrix;
}


// ------------------------------------------------------------
static double distanceBetweenTwoPose(
    const Eigen::Isometry3d &pose_0,
    const Eigen::Isometry3d &pose_1,
    const double pos_weight,
    const double rot_weight
){
    double pos_dist = (pose_1.translation() - pose_0.translation()).norm();
    Eigen::AngleAxisd rotation_vector(pose_1.rotation() * pose_0.rotation().transpose());
    double rot_angle = rotation_vector.angle();

    return pos_weight * pos_dist + rot_weight * rot_angle;
}
  





}; // end class

#endif