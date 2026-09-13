// 考虑末端位置和姿态约束的零空间控制（力矩接口）

#include "panda_controllers/null_space_controller.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/spatial/fwd.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/Constants.h>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <rclcpp/logging.hpp>


void panda_controllers::NullSpaceController::pose_null_space_controller_torque(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    // 读当前每个关节位置 q_
    for (std::size_t i = 0; i < joints_.size(); i ++)
    {
        q_[i] = state_interfaces_[2*i].get_value();
        dq_[i] = state_interfaces_[2*i+1].get_value();
    }

    // 计算惯性矩阵、科氏力矩阵、重力矩阵
    // 关节空间动力学：M(q)*ddq + C(q,dq)*dq + g(q) = tau_c + tau_ext
        // 惯性矩阵
        // M = M(q)
    pinocchio::crba(model_, *data_, q_);
    data_->M.triangularView<Eigen::StrictlyLower>() = data_->M.transpose().triangularView<Eigen::StrictlyLower>();
    Eigen::MatrixXd M = data_->M.block(0, 0, 7, 7);

        // 科氏力矩阵
        // 科氏力与离心力项：tau_coriolis = C(q,dq)*dq
    Eigen::MatrixXd C = pinocchio::computeCoriolisMatrix(model_, *data_, q_, dq_).block(0, 0, 7, 7);

        // 重力矩阵
        // 重力项：g = g(q)
    Eigen::VectorXd G = pinocchio::computeGeneralizedGravity(model_, *data_,q_ ).topRows(7);

    // RCLCPP_INFO(get_node()->get_logger(),"M：%zu×%zu\n C：%zu×%zu\n G：%zu×%zu\n", M.rows(), M.cols(),
    //                                                                               C.rows(), C.cols(),
    //                                                                               G.rows(), G.cols());



    // 计算局部雅可比 J_local、局部雅可比导数 dJ_local、操作空间惯性矩阵 Lambda、动力学一致广义逆 J_bar、零空间投影矩阵 N_tau
        // 局部雅可比 J_local
        // 末端局部速度：V_LOCAL = J_local(q)*dq
    Eigen::Matrix<double, 6, 9> J_full;
    pinocchio::computeFrameJacobian(model_, *data_, q_, ee_frame_id_, pinocchio::LOCAL, J_full);
    Eigen::Matrix<double, 6, 7> J_local = J_full.leftCols(7);

        // 局部雅可比导数 dJ_local
        // 末端局部加速度：dV_LOCAL = J_local*ddq + dJ_local*dq
    Eigen::Matrix<double, 6, 9> dJ_full;
    pinocchio::computeJointJacobiansTimeVariation(model_, *data_, q_, dq_);
    pinocchio::getFrameJacobianTimeVariation(model_, *data_, ee_frame_id_, pinocchio::LOCAL, dJ_full);
    Eigen::Matrix<double, 6, 7> dJ_local = dJ_full.leftCols(7);

        // 操作空间惯性矩阵 Lambda
        // Lambda = (J_local*M^{-1}*J_local^T)^{-1}
    Eigen::Matrix<double, 6, 6> Lambda = (J_local*M.inverse()*J_local.transpose()).inverse();

        // 动力学一致广义逆 J_bar
        // J_bar = M^{-1}*J_local^T*Lambda
    Eigen::Matrix<double, 7, 6> J_bar = M.inverse()*J_local.transpose()*Lambda;

        // 零空间投影矩阵 N_tau
        // N_tau = I - J_local^T*J_bar^T，并满足 J_local*M^{-1}*N_tau = 0
    Eigen::Matrix<double, 7, 7> N_tau = Eigen::Matrix<double, 7, 7>::Identity() - J_local.transpose()*J_bar.transpose();


    
    // 计算任务空间科氏力 C_x_tau、任务空间重力 G_x_tau
        // 任务空间科氏力 mu
        // mu = (J_bar^T*C - Lambda*dJ_local)*dq
    Eigen::Vector<double, 6> C_x_tau = (J_bar.transpose()*C - Lambda*dJ_local)*dq_.topRows(7);

        // 任务空间重力 p
        // p = J_bar^T*g
    Eigen::Vector<double, 6> G_x_tau = J_bar.transpose()*G;



    
    // 计算任务空间位姿误差 pose_error、任务空间旋量误差 dpose_error
        // 任务空间位姿误差 pose_error
        // T_error = T_current^{-1}*T_desired
        // e = Log6(T_error)^vee = Log6(T_current^{-1}*T_desired)^vee
    pinocchio::forwardKinematics(model_, *data_, q_);
    pinocchio::updateFramePlacement(model_, *data_, ee_frame_id_);
                    // pinocchio::framesForwardKinematics(model_, *data_, q_);
    const pinocchio::SE3 pose_current = data_->oMf[ee_frame_id_];
    const pinocchio::SE3 pose_error = pose_current.inverse()*pose_desired_;
    const pinocchio::Motion pose_error_se3_motion = pinocchio::log6(pose_error);
    const Eigen::Vector<double, 6> pose_error_se3 = pose_error_se3_motion.toVector();

        // 任务空间旋量误差 dpose_error
        // 固定期望位姿下的当前实现：de/dt ≈ -V_LOCAL = -J_local*dq
        // 注：严格的 SE(3) 对数误差导数还需考虑 Jlog6
    const Eigen::Vector<double, 6> dpose_error = -J_local*dq_.topRows(7);



    // 计算任务空间控制力 F_task、零空间稳定力矩 tau_0
        // 空间控制力 F_task
        // F_task = Lambda*(Kp*e + Kd*de/dt) + mu + p
    const Eigen::Vector<double, 6> F_task = Lambda*(Kp_*pose_error_se3 + Kd_*dpose_error) + C_x_tau + G_x_tau;

        // 零空间稳定力矩 tau_0
        // qd = A*sin(omega*t)*1_7
        // dqd = A*omega*cos(omega*t)*1_7
        // ddqd = -A*omega^2*sin(omega*t)*1_7
        // tau_0 = M*[ddqd + Kp_0*(qd-q) + Kd_0*(dqd-dq)] + C*dq + g
    // Eigen::Vector<double, 7> tau_0 = -Kd_0*M*dq_.topRows(7);
    Eigen::Vector<double, 7> tau_0;
    Eigen::Vector<double, 7> qd, dqd, ddqd;
    qd = Eigen::Vector<double, 7>::Ones()*A_*std::sin(omega_*time.seconds());
    dqd = Eigen::Vector<double, 7>::Ones()*A_*omega_*std::cos(omega_*time.seconds());
    ddqd = Eigen::Vector<double, 7>::Ones()*A_*omega_*omega_*(-std::sin(omega_*time.seconds()));
    tau_0 = M*(ddqd + Kp_0_*(qd-q_.topRows(7)) + Kd_0_*(dqd-dq_.topRows(7))) + C*dq_.topRows(7) + G;



    // 动量观测器估计外力
    // 广义动量：r = M(q)*dq
    // 动量恒等式：dr/dt = tau_c - g + C^T*dq + tau_ext
    // 离散观测器：r_hat[k] = r_hat[k-1] + dt*[
    //     tau_c[k-1] - g[k-1] + C[k-1]^T*dq[k-1] + tau_ext_hat[k-1]]
    // tau_ext_hat[k] = Ko*(r[k] - r_hat[k])
    // tau_ext_hat_lim = clip(tau_ext_hat, -max_tau_ext_hat, max_tau_ext_hat)
    C_k_1_ = pinocchio::computeCoriolisMatrix(model_, *data_, q_k_1_, dq_k_1_).block(0, 0, 7, 7);
    G_k_1_ = pinocchio::computeGeneralizedGravity(model_, *data_, q_k_1_).topRows(7);
    
    Eigen::Vector<double, 7> momentum_hat = momentum_hat_k_1_ + period.seconds()*(tau_k_1_ - G_k_1_ + C_k_1_.transpose()*dq_k_1_.topRows(7) + tau_ext_hat_k_1_);
    Eigen::Vector<double, 7> momentum = M*dq_.topRows(7);
    Eigen::Vector<double, 7> tau_ext_hat = Ko_*(momentum - momentum_hat);
    Eigen::Vector<double, 7> tau_ext_hat_lim = tau_ext_hat.cwiseMax(-max_tau_ext_hat_).cwiseMin(max_tau_ext_hat_);

    // RCLCPP_INFO(get_node()->get_logger(),"限制后的估计外力矩：[%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]", 
    //                                                         tau_ext_hat_lim[0],
    //                                                         tau_ext_hat_lim[1],
    //                                                         tau_ext_hat_lim[2],
    //                                                         tau_ext_hat_lim[3],
    //                                                         tau_ext_hat_lim[4],
    //                                                         tau_ext_hat_lim[5],
    //                                                         tau_ext_hat_lim[6]);



    // 计算总关节力矩 tau
    // 不启用观测器补偿：tau_c = J_local^T*F_task + N_tau*tau_0
    // 启用观测器补偿：tau_c = J_local^T*F_task + N_tau*tau_0
    //                                      - J_local^T*J_bar^T*tau_ext_hat_lim
    Eigen::Vector<double, 7> tau_;
    if (add_observer_)
    {
        tau_ = J_local.transpose()*F_task + N_tau*tau_0 - J_local.transpose()*J_bar.transpose()*tau_ext_hat_lim;
    }
    else 
    {
        tau_ = J_local.transpose()*F_task + N_tau*tau_0;
    }

    // RCLCPP_INFO(get_node()->get_logger(),"输入力矩：[%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]", 
    //                                                 tau_[0],
    //                                                 tau_[1],
    //                                                 tau_[2],
    //                                                 tau_[3],
    //                                                 tau_[4],
    //                                                 tau_[5],
    //                                                 tau_[6]);
    


    // 更新 k-1 时刻状态
    // 离散递推状态更新：x[k-1] <- x[k]
    momentum_hat_k_1_ = momentum_hat;
    tau_k_1_ = tau_;
    C_k_1_ = C;
    G_k_1_ = G;
    q_k_1_ = q_;
    dq_k_1_ = dq_;
    tau_ext_hat_k_1_ = tau_ext_hat_lim;



    for (std::size_t i = 0; i < joints_.size(); i++)
    {
        command_interfaces_[i].set_value(tau_[i]);
    }

        // 打印输出
    Eigen::Vector3d p_current = pose_current.translation();
    Eigen::Matrix3d R_current = pose_current.rotation();
    // RCLCPP_INFO(get_node()->get_logger(), 
    //             "末端位置：[%.4f, %.4f, %.4f]\n"
    //             "末端旋转矩阵：\n"
    //             "[%.4f, %.4f, %.4f]\n"
    //             "[%.4f, %.4f, %.4f]\n"
    //             "[%.4f, %.4f, %.4f]\n",
    //             p_current.x(), p_current.y(), p_current.z(),
    //             R_current(0,0), R_current(0,1), R_current(0,2),
    //             R_current(1,0), R_current(1,1), R_current(1,2),
    //             R_current(2,0), R_current(2,1), R_current(2,2));

}
