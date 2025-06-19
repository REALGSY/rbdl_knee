#include <iostream>
#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

int main(int argc, char *argv[])
{
    rbdl_check_api_version(RBDL_API_VERSION);
    // 创建一个新的 Model 对象
    Model *model_urdf = new Model();

    // 加载 URDF 文件到 model_urdf 中
    bool success = Addons::URDFReadFromFile("../10Human(LBw).SLDASM_leftfoot_fixed/urdf/xijp10Human(LBw).SLDASM.urdf", model_urdf, false, true);
    if (!success)
    {
        std::cout << "Error loading model" << std::endl;
        return -1;
    }
    for (unsigned int i = 0; i < model_urdf->mJoints.size(); ++i) {
    if (model_urdf->mJoints[i].mDoFCount > 0) { // 过滤掉 fixed joint
        std::string joint_name = model_urdf->GetBodyName(i);
        std::cout << "Joint " << i << ": " << joint_name << std::endl;
    }
}

    // 打印模型的自由度数量
    std::cout << "Model DOF: " << model_urdf->q_size << std::endl;

    // 初始化关节角度、速度和加速度
    VectorNd Q = VectorNd::Zero(model_urdf->q_size);
    VectorNd QDot = VectorNd::Zero(model_urdf->qdot_size);
    VectorNd Tau = VectorNd::Zero(model_urdf->qdot_size);
    VectorNd QDDot = VectorNd::Zero(model_urdf->qdot_size);

    // 设置模型的重力
    model_urdf->gravity = Vector3d(0, 0, -9.81);

    // 定义时间步长和模拟时间
    double dt = 0.5;                      // 时间步长
    double T = 10.0;                      // 模拟时间
    int steps = static_cast<int>(T / dt); // 总步数

    // 定义初始和目标关节角度
    VectorNd Q_initial = VectorNd::Zero(model_urdf->q_size);
    //Q_initial[11] = 1.3f;
    VectorNd Q_target = VectorNd::Zero(model_urdf->q_size);

    // 初始状态（现在是目标状态）
    //Q_target[0] = -1.0f; // YJ2 的目标角度为 -1.1 弧度
    Q_target[0] = -1.0f;
    Q_target[1] = 0.1f; 
    Q_target[2] = 1.0f;
    Q_target[3] = 0.1f; 
    Q_target[4] = 0.1f; 
    Q_target[5] = 0.1f; 
    Q_target[6] = 0.1f; 
    Q_target[7] = 0.1f; 
    Q_target[8] = 0.1f; 
    Q_target[9] = 0.1f; 
    Q_target[10] = 0.1f; 
    Q_target[11] = 0.1f; 
    // QDot[2] = 1;
    // QDDot[2] = -1.0f; // 初始加速度为零
    Q_initial[0] = 1.0f; // YJ2 的目标角度为 -1.1 弧度
    Q_initial[1] = 0.1f; 
    Q_initial[2] = -1.0f;
    Q_initial[3] = 0.1f; 
    Q_initial[4] = 0.1f; 
    Q_initial[5] = 0.1f; 
    Q_initial[6] = 0.1f; 
    Q_initial[7] = 0.1f; 
    Q_initial[8] = 0.1f; 
    Q_initial[9] = 0.1f; 
    Q_initial[10] = 0.1f; 
    Q_initial[11] = 0.1f; 

    for (int i = 0; i <= steps; ++i)
    {
        // 使用线性插值更新关节角度（从 Q_target 回到 Q_initial）
        double t = static_cast<double>(i) / steps;
        Q = (1 - t) * Q_target + t * Q_initial; // 反向插值
        //std::cout << "Q: " << Q << " s" << std::endl;

        // 更新yunodngxue
        UpdateKinematics(*model_urdf, Q, QDot, QDDot);
        // UpdateKinematicsCustom(*model_urdf, &Q, &QDot, &QDDot);

        // 计算力矩
        InverseDynamics(*model_urdf, Q, QDot, QDDot, Tau);

        // 打印当前状态
        std::cout << "Time: " << i * dt << " s" << std::endl;
        std::cout << "Joint Angles (Q): " << Q.transpose() << std::endl;
        std::cout << "Computed Torques (Tau): " << Tau.transpose() << std::endl;
        std::cout << "-----------------------------" << std::endl;
    }

    delete model_urdf;

    return 0;
}