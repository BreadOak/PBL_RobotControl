clear all
clc

testNode1 = ros2node("/test1")
% sub = ros2subscriber(testNode1,"topic_2")
% wait for 20 secs until message from Linux environment is received
% msg = receive(sub, 20)
pub = ros2publisher(testNode1,"/topic_1","std_msgs/String");
pubMsg = ros2message("std_msgs/String");
pubMsg.data = 'Hello';
% publish 'Hello' 10 times to the /topic_1
for x = 1:10
    send(pub,pubMsg)
    disp('msg sent')
    pause(1)
end