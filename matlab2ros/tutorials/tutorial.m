clear all
clc

testNode1 = ros2node("/matlab_node", 0)   % node initialization
pause(2);

% subscirber
sub1 = ros2subscriber(testNode1, "/topic_3", "std_msgs/Float64", @callback_func);
global data1;
sub2 = ros2subscriber(testNode1, "/topic_4", "std_msgs/Float64", @callback_func2);
global data2;

field1 = 'label';  value1 = 'empty';
field2 = 'size';  value2 = uint32(0);
field3 = 'stride';  value3 = uint32(0);
s = struct(field1, value1, field2, value2, field3, value3)

% publisher
pub_1 = ros2publisher(testNode1,"/ampere","std_msgs/Float64MultiArray");
pubMsg_1 = ros2message("std_msgs/Float64MultiArray");
pubMsg_1.data = 0;
pubMsg_1.layout.dim = s;
pubMsg_1.layout.data_offset = uint32(0);
pubMsg_1.data = [1 2 3]; % ref_ampere, cur_ampere, real_ampere 

pub_2 = ros2publisher(testNode1,"/volts","std_msgs/Float64MultiArray");
pubMsg_2 = ros2message("std_msgs/Float64MultiArray");
pubMsg_2.layout.dim = s;
pubMsg_2.layout.data_offset = uint32(0);
pubMsg_2.data = [1 2 3]; % ref_voltage, cur_voltage, real_voltage

for x = 1:10000
    send(pub_1,pubMsg_1)
%     pubMsg_1.data = pubMsg_1.data + 1;
    send(pub_2,pubMsg_2)
% 	pubMsg_2.data = pubMsg_2.data - 1;
    disp('msg sent')
    disp(data1)
    disp(data2)
    pause(0.01)
end

clear all
clc