/*
 * pmsm_node.cpp
 *      Author: junyoung kim / lgkimjy
 */

#include "pmsm_node.hpp"

PMSM::PMSM(const rclcpp::NodeOptions & node_options)
: Node("pmsm_node", node_options)
{
    int8_t qos_depth = 0;

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();
    data_subscriber1_ = this->create_subscription<std_msgs::msg::Float64>(
        "/topic_1",
        qos_profile,
        std::bind(&PMSM::subscribe_topic_message1, this, _1));
    data_subscriber2_ = this->create_subscription<std_msgs::msg::Float64>(
        "/topic_2",
        qos_profile,
        std::bind(&PMSM::subscribe_topic_message2, this, _1));
    data_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
        "/topic_3",
        QOS_RKL10V);

    timer_ = this->create_wall_timer(0.1s, std::bind(&PMSM::timerInterrupt, this));
}

void PMSM::subscribe_topic_message1(const std_msgs::msg::Float64::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received message : '%f'", msg->data);
    msg_.data = msg->data;
}
void PMSM::subscribe_topic_message2(const std_msgs::msg::Float64::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received message : '%f'", msg->data);
}

void PMSM::timerInterrupt()
{
    // std_msgs::msg::Float64 msg;
    // msg_.data += ;
    data_publisher_->publish(msg_);
}

PMSM::~PMSM()
{
}

void print_help()
{
  printf("For argument node:\n");
  printf("node_name [-h]\n");
  printf("Options:\n");
  printf("\t-h Help           : Print this help function.\n");
}

int main(int argc, char * argv[])
{
    if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
        print_help();
        return 0;
    }

    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("PMSM Node"), "Initialize the PMSM nodes");
    auto node = std::make_shared<PMSM>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}