#include "panda_controllers/null_space_controller.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/spatial/fwd.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pinocchio/parsers/mjcf.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <cstddef>
#include <iterator>
#include <string>
#include <vector>
#include <cmath>
#include <Eigen/Core>

namespace panda_controllers
{

controller_interface::CallbackReturn NullSpaceController::on_init()
{
    auto_declare<std::vector<std::string>>("joints",{"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"});
    auto_declare<double>("max_velocity", 3.0);
    auto_declare<std::string>("mjcf_path", "");
    auto_declare("Kd", 2000.0);
    auto_declare("Kp", 200.0);
    auto_declare("Kp_0", 200.0);
    auto_declare("Kd_0", 20.0);
    auto_declare("A", 2.0);
    auto_declare("omega", 1.0);
    auto_declare("Ko", 100.0);
    auto_declare("add_observer", false);
    auto_declare("max_tau_ext_hat", 30.0);

    return  controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::InterfaceConfiguration NullSpaceController::command_interface_configuration() const
{   // 请求指令接口
    std::vector<std::string> command_interfaces_names={};
    for (std::size_t index = 0; index < joints_.size(); ++index)
    {
        command_interfaces_names.push_back(joints_[index] + "/" + hardware_interface::HW_IF_EFFORT);
    }

    return {controller_interface::interface_configuration_type::INDIVIDUAL, {command_interfaces_names}};
}

controller_interface::InterfaceConfiguration NullSpaceController::state_interface_configuration() const
{   // 请求状态接口
    std::vector<std::string>state_interface_names={};
    for (std::size_t index = 0; index < joints_.size(); ++index)
    {
        state_interface_names.push_back(joints_[index] + "/" + hardware_interface::HW_IF_POSITION);
        state_interface_names.push_back(joints_[index] + "/" + hardware_interface::HW_IF_VELOCITY);
    }

    return {controller_interface::interface_configuration_type::INDIVIDUAL, {state_interface_names}};
}

controller_interface::CallbackReturn NullSpaceController::on_configure(const rclcpp_lifecycle::State &)
{   // 获取 .yaml 参数
    joints_ = get_node()->get_parameter("joints").as_string_array();
    max_velocity_ = get_node()->get_parameter("max_velocity").as_double();
    mjcf_path_ = get_node()->get_parameter("mjcf_path").as_string();
    Kp_ = get_node()->get_parameter("Kp").as_double();
    Kd_ = get_node()->get_parameter("Kd").as_double();
    Kp_0_ = get_node()->get_parameter("Kp_0").as_double();
    Kd_0_ = get_node()->get_parameter("Kd_0").as_double();
    A_ = get_node()->get_parameter("A").as_double();
    omega_ = get_node()->get_parameter("omega").as_double();
    Ko_ = get_node()->get_parameter("Ko").as_double();
    add_observer_ = get_node()->get_parameter("add_observer").as_bool();
    max_tau_ext_hat_ = get_node()->get_parameter("max_tau_ext_hat").as_double();

    // 建立 pinocchio 模型
    pinocchio::mjcf::buildModel(mjcf_path_, model_);
    data_ = std::make_unique<pinocchio::Data>(model_);

    // 获取 pinocchio 模型接口的关节排列顺序（按名字查找下标
    for (std::size_t index = 0; index < joints_.size(); ++index)
    {
        const pinocchio::Index joint_id = model_.getJointId(joints_[index]);
        q_indice_.push_back(model_.idx_qs[joint_id]);
        // RCLCPP_INFO(get_node()->get_logger(), "\033[32m q_indice_: %d \033[0m", q_indice_[index]);
    }
    
    // 创建关节变量
    q_ = pinocchio::neutral(model_);
    dq_ = pinocchio::neutral(model_);
    q_initial_ = pinocchio::neutral(model_);


    RCLCPP_INFO(get_node()->get_logger(), "\033[32m配置成功\033[0m");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn NullSpaceController::on_activate(const rclcpp_lifecycle::State &)
{   
    for (std::size_t index = 0; index < joints_.size(); ++index)
    {
        command_interfaces_[index].set_value(0.0);
        q_initial_[index] = state_interfaces_[2*index].get_value();
    }
    pinocchio::forwardKinematics(model_, *data_, q_initial_);
    pinocchio::updateFramePlacements(model_, *data_);
    ee_frame_id_ = model_.getFrameId("ee_center_body");
    p_desired_ = data_->oMf[ee_frame_id_].translation();
    pose_desired_ = data_->oMf[ee_frame_id_];

    RCLCPP_INFO(get_node()->get_logger(), "\033[32m接口激活成功\033[0m");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn NullSpaceController::on_deactivate(const rclcpp_lifecycle::State &)
{   
    for (std::size_t index = 0; index < joints_.size(); ++index)
    {
        command_interfaces_[index].set_value(0.0);
    }

    RCLCPP_INFO(get_node()->get_logger(), "\033[32m控制器已失活\033[0m");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type NullSpaceController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
{   



    pose_null_space_controller_torque(time, period);


    return controller_interface::return_type::OK;
}


} // namespace panda_controllers

PLUGINLIB_EXPORT_CLASS(panda_controllers::NullSpaceController, controller_interface::ControllerInterface)