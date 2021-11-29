clear all
clc

testNode1 = ros2node("/matlab_node", 0)   % node initialization
pause(2);

% subscirber
sub1 = ros2subscriber(testNode1, "/topic_3", "std_msgs/Float64", @callback_func)
global data1;
sub2 = ros2subscriber(testNode1, "/topic_4", "std_msgs/Float64", @callback_func2)
global data2;

% publisher
pub_1 = ros2publisher(testNode1,"/topic_1","std_msgs/Float64");
pubMsg_1 = ros2message("std_msgs/Float64");
pubMsg_1.data = 0;

pub_2 = ros2publisher(testNode1,"/topic_2","std_msgs/Float64");
pubMsg_2 = ros2message("std_msgs/Float64");
pubMsg_2.data = 0;

for x = 1:10000
    send(pub_1,pubMsg_1)
    pubMsg_1.data = pubMsg_1.data + 1;
    send(pub_2,pubMsg_2)
	pubMsg_2.data = pubMsg_2.data - 1;
    disp('msg sent')
    disp(data1)
    disp(data2)
    pause(0.01)
end

clear all
clc