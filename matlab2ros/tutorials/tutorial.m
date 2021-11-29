clear all
clc

testNode1 = ros2node("/test1", 0)
% sub = ros2subscriber(testNode1,"topic_2")
% wait for 20 secs until message from Linux environment is received
% msg = receive(sub, 20)
pub = ros2publisher(testNode1,"/topic_1","std_msgs/Float64");
pubMsg = ros2message("std_msgs/Float64");
pubMsg.data = 1;

pub_1 = ros2publisher(testNode1,"/topic_2","std_msgs/Float64");
pubMsg_2 = ros2message("std_msgs/Float64");
pubMsg_2.data = 2;
% publish 'Hello' 10 times to the /topic_1
for x = 1:10000
    send(pub,pubMsg)
    pubMsg.data = pubMsg.data + 1;
    send(pub_1,pubMsg_2)
    disp('msg sent')
    pause(0.01)
end

clear all
clc