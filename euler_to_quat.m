function [ quaternion ] = euler_to_quat( rpy )
% takes in roll, pitch and yaw and convers to quaternions
roll = rpy(1);
pitch = rpy(2);
yaw = rpy(3);

e0 = cos(yaw/2)*cos(pitch/2)*cos(roll/2) + sin(yaw/2)*sin(pitch/2)*sin(roll/2);
e1 = cos(yaw/2)*cos(pitch/2)*sin(roll/2) - sin(yaw/2)*sin(pitch/2)*cos(roll/2);
e2 = cos(yaw/2)*sin(pitch/2)*cos(roll/2) + sin(yaw/2)*cos(pitch/2)*sin(roll/2);
e3 = sin(yaw/2)*cos(pitch/2)*cos(roll/2) - cos(yaw/2)*sin(pitch/2)*sin(roll/2);

quaternion = [e0, e1, e2, e3];

end

