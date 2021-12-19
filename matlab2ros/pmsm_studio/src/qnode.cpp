/*
 * qnode.cpp
 *      Author: junyoung kim / lgkimjy
 */

#include "qnode.hpp"

/* PMSM CLASS */
PMSM::PMSM(const rclcpp::NodeOptions & node_options)
: Node("pmsm_node", node_options)
{
    // data_subscriber_pmsm_ = this->create_subscription<std_msgs::msg::Float64>(
    //     "/topic_1",
    //     qos_profile,
    //     std::bind(&PMSM::subscribe_topic_message_pmsm, this, _1));
    // data_publisher_pmsm_ = this->create_publisher<std_msgs::msg::Float64>(
    //     "/topic_3",
    //     QOS_RKL10V);
    // timer_ = this->create_wall_timer(0.1s, std::bind(&PMSM::timerInterrupt, this));
}

void PMSM::subscribe_topic_message_pmsm(const std_msgs::msg::Float64::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received message : '%f'", msg->data);
}

void PMSM::timerInterrupt()
{
}

PMSM::~PMSM()
{
}

/* QNode CLASS */
QNode::QNode(int argc, char** argv) :
	init_argc(argc),
	init_argv(argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("PMSM GUI Node"), "Initialize the PMSM nodes");
    node = std::make_shared<PMSM>();

    /* variable to pub */
    ref_input = 0;

    /* plot variable intialize */
    Idse = Iqse = M_Idse = M_Iqse = M_Ias = M_Ibs = M_Ics = 0;
    Van_Ref = Vbn_Ref = Vcn_Ref = 0;
    Torque_Ref = Torque_Real = Torque_Load= 0;
    Ref_rpm = Cur_rpm = 0;
}
void QNode::init()
{
    int8_t qos_depth = 0;
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

    data_publisher_qnode_ = node->create_publisher<std_msgs::msg::Float64>(
        "/Reference",
        QOS_RKL10V);
    mode_publisher_qnode_ = node->create_publisher<std_msgs::msg::Int32>(
        "/mode_selection",
        QOS_RKL10V);
    current_mode_publisher_qnode_ = node->create_publisher<std_msgs::msg::Int32>(
        "/CurrentMode",
        QOS_RKL10V);
    velocity_mode_publisher_qnode_ = node->create_publisher<std_msgs::msg::Int32>(
        "/VelocityMode",
        QOS_RKL10V);
    position_mode_publisher_qnode_ = node->create_publisher<std_msgs::msg::Int32>(
        "/PositionMode",
        QOS_RKL10V);
    dob_mode_publisher_qnode_ = node->create_publisher<std_msgs::msg::Int32>(
        "/DOBoption",
        QOS_RKL10V);
    vol_subscriber_qnode_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/Voltage",
        qos_profile,
        std::bind(&QNode::subscribe_voltage_qnode, this, _1));
    amp_subscriber_qnode_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/Current",
        qos_profile,
        std::bind(&QNode::subscribe_ampere_qnode, this, _1));
    torque_subscriber_qnode_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/Torque",
        qos_profile,
        std::bind(&QNode::subscribe_torque_qnode, this, _1));
    rpm_subscriber_qnode_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/Velocity",
        qos_profile,
        std::bind(&QNode::subscribe_rpm_qnode, this, _1));

    start();
    return;
}

void QNode::run() {
  rclcpp::spin(node);
  rclcpp::shutdown();
}

QNode::~QNode()
{
}

void QNode::sendData()
{
    RCLCPP_INFO(rclcpp::get_logger("PMSM Node"), "None clicked");

    // std_msgs::msg::Float64 msg_test;
    msg_test.data += 1.0;
    data_publisher_qnode_->publish(msg_test);
}

void QNode::pubMode(int mode)
{
    msg_mode.data = mode;
    msg_data.data = 0.0;
    mode_publisher_qnode_->publish(msg_mode);
    data_publisher_qnode_->publish(msg_data);
}
void QNode::pubMode(vector<int> mode)
{
    msg_current_mode.data = mode[0];
    msg_velocity_mode.data = mode[1];
    msg_position_mode.data = mode[2];
    msg_dob_mode.data = mode[3];
    /* mode */
    current_mode_publisher_qnode_->publish(msg_current_mode);
    velocity_mode_publisher_qnode_->publish(msg_velocity_mode);
    position_mode_publisher_qnode_->publish(msg_position_mode);
    dob_mode_publisher_qnode_->publish(msg_dob_mode);
    /* initalize mode selection */
    msg_mode.data = 1;
    mode_publisher_qnode_->publish(msg_mode);
    /* data */
    msg_data.data = 0.0;
    data_publisher_qnode_->publish(msg_data);
}
void QNode::pubData(float data)
{
    msg_data.data = data;
    data_publisher_qnode_->publish(msg_data);
}

void QNode::subscribe_ampere_qnode(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    // 1:Id(Measured), 2:Iq(Measured), 3:Id(Real), 4:Iq(Real), 5:I_A-phase, 6:I_B-phase, 7:I_C-phase
    Idse = msg->data[0];
    Iqse = msg->data[1];
    M_Iqse = msg->data[2];
    M_Iqse = msg->data[3];
    M_Ias = msg->data[4];
    M_Ibs = msg->data[5];
    M_Ics = msg->data[6];
}
void QNode::subscribe_voltage_qnode(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    // 1:V_A-phase, 2:V_B-phase, 3:V_C-phase
    Van_Ref = msg->data[0];
    Vbn_Ref = msg->data[1];
    Vcn_Ref = msg->data[2];
}
void QNode::subscribe_torque_qnode(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    // 1:Reference Torque, 2:Real Torque, 3:Load Torque
    Torque_Ref = msg->data[0];
    Torque_Real = msg->data[1];
    Torque_Load = msg->data[2];
}
void QNode::subscribe_rpm_qnode(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    // 1:Ref Rotating Speed, 2:Current Rotataing Speed
    Ref_rpm = msg->data[0];
    Cur_rpm = msg->data[1];
}