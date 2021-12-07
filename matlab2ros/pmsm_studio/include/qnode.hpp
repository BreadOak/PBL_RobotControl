/*
 * qnode.hpp
 *      Author: junyoung kim / lgkimjy
 */

#include <iostream>
#include <vector>
#include <sstream>
#include <string>

#include <QThread>
#include <QStringListModel>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"

// custom interfaces declaration
#include "custom_interfaces/msg/references.hpp"
#include "custom_interfaces/msg/ref_voltage.hpp"

using namespace std;
using std::placeholders::_1;

/* PMSM CLASS */
class PMSM : public rclcpp::Node
{
public:
    using voltages = custom_interfaces::msg::RefVoltage;
    using references = custom_interfaces::msg::References;

    explicit PMSM(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
    virtual ~PMSM();

private:
    /* subscriber */
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr data_subscriber_pmsm_;
    /* publisher */
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr data_publisher_pmsm_;
    std_msgs::msg::Float64 msg_;
    
    /* Timer interrupt */
    rclcpp::TimerBase::SharedPtr timer_;
    void timerInterrupt();

    /* subscription callabck func */
    void subscribe_topic_message_pmsm(const std_msgs::msg::Float64::SharedPtr msg);
};


/* QNode CLASS */
class QNode : public QThread
{
    Q_OBJECT
public:
	explicit QNode(int argc, char** argv);
	virtual ~QNode();
	void init();
    void run();

    void sendData();
    void subscribe_voltage_qnode(const std_msgs::msg::Float64::SharedPtr msg);
    void subscribe_ampere_qnode(const std_msgs::msg::Float64::SharedPtr msg);
    void subscribe_gain_qnode(const std_msgs::msg::Float64::SharedPtr msg);

Q_SIGNALS:
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;
    
    std::shared_ptr<rclcpp::Node> node;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr data_publisher_qnode_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr vol_subscriber_qnode_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr amp_subscriber_qnode_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gain_subscriber_qnode_;
};
