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

class PMSM : public rclcpp::Node
{
public:
    using voltages = custom_interfaces::msg::RefVoltage;
    using references = custom_interfaces::msg::References;

    explicit PMSM(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
    virtual ~PMSM();

private:
    /* subscriber */
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr data_subscriber1_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr data_subscriber2_;
    rclcpp::Subscription<voltages>::SharedPtr voltage_subscriber_;
    
    /* publisher */
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr data_publisher_;
    
    /* Timer interrupt */
    rclcpp::TimerBase::SharedPtr timer_;

    void subscribe_topic_message1(const std_msgs::msg::Float64::SharedPtr msg);
    void subscribe_topic_message2(const std_msgs::msg::Float64::SharedPtr msg);

    void publish_topic_message();
    
    void timerInterrupt();

    std_msgs::msg::Float64 msg_;
};


class QNode : public QThread
{
    Q_OBJECT
public:
	explicit QNode(int argc, char** argv);
	virtual ~QNode();
	void init();
    void run();

Q_SIGNALS:
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;
    
    std::shared_ptr<rclcpp::Node> node;
};
