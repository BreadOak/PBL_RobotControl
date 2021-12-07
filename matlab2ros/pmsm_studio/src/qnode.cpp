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
}
void QNode::init()
{
    int8_t qos_depth = 0;
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

    data_publisher_qnode_ = node->create_publisher<std_msgs::msg::Float64>(
        "/tuned_gain",
        QOS_RKL10V);
    vol_subscriber_qnode_ = node->create_subscription<std_msgs::msg::Float64>(
        "/voltages",
        qos_profile,
        std::bind(&QNode::subscribe_voltage_qnode, this, _1));
    amp_subscriber_qnode_ = node->create_subscription<std_msgs::msg::Float64>(
        "/ampere",
        qos_profile,
        std::bind(&QNode::subscribe_ampere_qnode, this, _1));
    gain_subscriber_qnode_ = node->create_subscription<std_msgs::msg::Float64>(
        "/gain",
        qos_profile,
        std::bind(&QNode::subscribe_gain_qnode, this, _1));

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

    std_msgs::msg::Float64 msg_test;
    data_publisher_qnode_->publish(msg_test);
}

void QNode::subscribe_voltage_qnode(const std_msgs::msg::Float64::SharedPtr msg)
{
    RCLCPP_INFO(rclcpp::get_logger("PMSM Node"), "Data callbacked");
    msg->data;
}
void QNode::subscribe_ampere_qnode(const std_msgs::msg::Float64::SharedPtr msg)
{
    RCLCPP_INFO(rclcpp::get_logger("PMSM Node"), "Data callbacked");
    msg->data;
}
void QNode::subscribe_gain_qnode(const std_msgs::msg::Float64::SharedPtr msg)
{
    RCLCPP_INFO(rclcpp::get_logger("PMSM Node"), "Data callbacked");
    msg->data;
}