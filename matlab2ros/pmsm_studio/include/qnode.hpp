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
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

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
    void pubMode(int mode);
    void pubMode(vector<int> mode);
    void pubData(float data);
    void subscribe_voltage_qnode(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void subscribe_ampere_qnode(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void subscribe_torque_qnode(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void subscribe_rpm_qnode(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void subscribe_angle_qnode(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    std_msgs::msg::Float64 msg_test;
    std_msgs::msg::Int32 msg_mode;
    std_msgs::msg::Int32 msg_current_mode;
    std_msgs::msg::Int32 msg_velocity_mode;
    std_msgs::msg::Int32 msg_position_mode;
    std_msgs::msg::Int32 msg_dob_mode;
    std_msgs::msg::Float64 msg_data;

    /* variable to pub */
    float ref_input;

    /* variables to plot */
    float Idse, Iqse, M_Idse, M_Iqse, M_Ias, M_Ibs, M_Ics;
    float Van_Ref, Vbn_Ref, Vcn_Ref;
    float Torque_Ref, Torque_Real, Torque_Load;
    float Ref_rpm, Cur_rpm;
    float Ang_Ref, Ang_Real;
    float Sim_time;

Q_SIGNALS:
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;
    
    std::shared_ptr<rclcpp::Node> node;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr data_publisher_qnode_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mode_publisher_qnode_;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr current_mode_publisher_qnode_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr velocity_mode_publisher_qnode_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr position_mode_publisher_qnode_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr dob_mode_publisher_qnode_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr vol_subscriber_qnode_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr amp_subscriber_qnode_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr torque_subscriber_qnode_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rpm_subscriber_qnode_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr angle_subscriber_qnode_;
};
