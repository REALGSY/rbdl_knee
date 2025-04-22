#include <iostream>
#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

int main (int argc, char* argv[]) {
    rbdl_check_api_version (RBDL_API_VERSION);
    //new一个新的model
    Model* model_urdf = new Model();

    // unsigned int body_a_id, body_b_id, body_c_id;
    // Body body_a, body_b, body_c;
    // Joint joint_a, joint_b, joint_c;

    // 加载 URDF 文件到 model_urdf 中
    bool success = Addons::URDFReadFromFile("../skyworth_human.urdf", model_urdf, false);
    if (!success) {
        std::cout << "Error loading model" << std::endl;
        return -1;
    }
    VectorNd Q = VectorNd::Zero(model_urdf->q_size);
    VectorNd QDot = VectorNd::Zero(model_urdf->qdot_size);
    VectorNd Tau = VectorNd::Zero(model_urdf->qdot_size);
    VectorNd QDDot = VectorNd::Zero(model_urdf->qdot_size);

    UpdateKinematicsCustom (*model_urdf, &Q, &QDot,&QDDot);
    unsigned int base_id = model_urdf->GetBodyId("Z6"); 
    Vector3d body_point = Vector3d(0., 0., 0.); // link自身原点
    Vector3d base_coords = CalcBodyToBaseCoordinates(*model_urdf, Q, base_id, body_point, false);
    std::cout << "BASE在基坐标系下的位置: " << base_coords.transpose() << std::endl;
    //CalcBodyToBaseCoordinates(*model_urdf, @Q,model_urdf->GetBodyId("base_link"), Vector3d(0., 0., 0.), false);
    // 设置模型的重力
    model_urdf -> gravity = Vector3d (0., -9.81, 0.);
    //model_urdf ->
    // // 添加连接体
    // body_a = Body(1., Vector3d(0.5, 0., 0.0), Vector3d(1., 1., 1.));
    // joint_a = Joint(JointTypeRevolute, Vector3d(0., 0., 1.));
    // body_a_id = model.AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a);

    // body_b = Body(1., Vector3d(0., 0.5, 0.), Vector3d(1., 1., 1.));
    // joint_b = Joint(JointTypeRevolute, Vector3d(0., 0., 1.));
    // body_b_id = model.AddBody(body_a_id, Xtrans(Vector3d(1., 0., 0.)), joint_b, body_b);

    // body_c = Body(0., Vector3d(0.5, 0., 0.), Vector3d(1., 1., 1.));
    // joint_c = Joint(JointTypeRevolute, Vector3d(0., 0., 1.));
    // body_c_id = model.AddBody(body_b_id, Xtrans(Vector3d(0., 1., 0.)), joint_c, body_c);
    // 调用前向动力学
    ForwardDynamics(*model_urdf, Q, QDot, Tau, QDDot);

    // 输出结果
    //std::cout << Tau.transpose() << std::endl;
    //std::cout << Utils::GetModelDOFOverview(*model_urdf);
    //std::cout << Utils::GetModelHierarchy(*model_urdf);
    // 清理
    delete model_urdf;

    return 0;
}
