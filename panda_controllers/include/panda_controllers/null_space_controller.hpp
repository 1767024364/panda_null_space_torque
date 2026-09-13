#ifndef PANDA_CONTROLLERS__NULL_SPACES_CONTROLLER_HPP_
#define PANDA_CONTROLLERS__NULL_SPACES_CONTROLLER_HPP_

#include "controller_interface/controller_interface.hpp"

// #include <Eigen/src/Core/Matrix.h>
#include <string>
#include <vector>
#include <memory>
#include <Eigen/Core>
#include <cmath>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"


namespace panda_controllers
{

class NullSpaceController : public controller_interface::ControllerInterface
{
public:
    controller_interface::CallbackReturn on_init() override;
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    // .yaml 参数
    std::vector<std::string> joints_;
    double max_velocity_{3.0};
    std::string mjcf_path_{""};
    double Kp_{0.0};
    double Kd_{0.0};
    double Kp_0_{0.0};
    double Kd_0_{0.0};
    double A_{0.0};
    double omega_{0.0};
    double Ko_{0.0};
    bool add_observer_{false};
    double max_tau_ext_hat_{30.0};

    // pinocchio 参数
    pinocchio::Model model_;
    std::unique_ptr<pinocchio::Data> data_;
    std::vector<pinocchio::Index> q_indice_{};

    // 关节变量
    Eigen::VectorXd q_{};
    Eigen::VectorXd dq_{};
    Eigen::VectorXd q_initial_{};


    // 末端变量
    Eigen::VectorXd p_desired_{};           //末端期望位置
    pinocchio::SE3 pose_desired_{};         //末端期望位姿

    pinocchio::Index ee_frame_id_;

    // 动量观测器变量
    // Eigen::Vector<double, 7> momentum_hat_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    Eigen::Vector<double, 7> momentum_hat_k_1_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // Eigen::Vector<double, 7> tau_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    Eigen::Vector<double, 7> tau_k_1_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // Eigen::Vector<double, 7> tau_ext_hat_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    Eigen::Vector<double, 7> tau_ext_hat_k_1_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    Eigen::Vector<double, 9> q_k_1_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    Eigen::Vector<double, 9> dq_k_1_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    Eigen::Vector<double, 7> G_k_1_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    Eigen::Matrix<double, 7, 7> C_k_1_ = Eigen::Matrix<double, 7, 7>::Zero();


    void sine_velocity_controller_test(const rclcpp::Time & time);
    void position_null_space_controller(const rclcpp::Time & time);
    void pose_null_space_controller(const rclcpp::Time & time);
    void pose_null_space_controller_torque(const rclcpp::Time & time, const rclcpp::Duration & period);



};


} // namespace panda_controllers

#endif // PANDA_CONTROLLERS__NULL_SPACES_CONTROLLER_HPP_