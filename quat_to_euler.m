function [ rpy ] = quat_to_euler( quaternion )
%takes in a quaternion and spits out roll pitch and yaw
% euler angles

e0 = quaternion(1);
e1 = quaternion(2);
e2 = quaternion(3);
e3 = quaternion(4);

roll = atan2(2*(e0*e1 + e2*e3), (e0^2 + e3^2 - e1^2 -e2^2 ));
pitch = asin(2*(e0*e2 - e1*e3 ));
yaw = atan2(2*(e0*e3 + e1*e2 ), (e0^2 + e1^2 - e2^2 - e3^2 ));


rpy = [roll, pitch, yaw];
end

