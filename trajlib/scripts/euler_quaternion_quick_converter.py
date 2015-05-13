import math
from math import cos, sin, asin, atan2

ratio = math.pi/180;

def euler2quat_converter(roll, pitch, yaw):
	roll = roll*ratio;
	pitch = pitch * ratio;
	yaw = yaw * ratio;
	
	C_1 = cos(yaw/2);
	S_1 = sin(yaw/2);
	
	C_2 = cos(pitch/2);
	S_2 = sin(pitch/2);
	
	C_3 = cos(roll/2);
	S_3 = sin(roll/2);
	
	quat = [];
	qw =  C_1*C_2*C_3 - S_1*S_2*S_3;
	qx =  S_1*S_2*C_3 + C_1*C_2*S_3;
	qy =  S_1*C_2*C_3 + C_1*S_2*S_3;
	qz =  C_1*S_2*C_3 - S_1*C_2*S_3;
	
	quat.append(qw);
	quat.append(qx);
	quat.append(qy);
	quat.append(qz);
	print "Current Quaternion:", "w:",qw,"x:",qx,"y:",qy,"z:",qz;
	return quat;

def quat2euler_convertor(qw,qx,qy,qz):
	euler_ang = [];

	yaw_ang = atan2(2*qy*qw-2*qx*qz , 1 - 2*qy*qy - 2*qz*qz);
	pitch_ang = asin(2*qx*qy + 2*qz*qw);
	roll_ang = atan2(2*qx*qw-2*qy*qz , 1 - 2*qx*qx - 2*qz*qz);
	
	print "Current Euler Angle:";
	print "Roll:",roll_ang/ratio, "Pitch:", pitch_ang/ratio, "Yaw:", yaw_ang/ratio;
	
	euler_ang.append(roll_ang);
	euler_ang.append(pitch_ang);
	euler_ang.append(yaw_ang);
	return euler_ang;

if __name__ == '__main__':
	
	test_roll = -179.933167083;
	test_pitch = -76.8560705717;
	test_yaw = 103.627487118;
	euler2quat_converter(test_roll, test_pitch, test_yaw);


	test_qw = -0.488244;
	test_qx = -0.484592;
	test_qy = 0.384602;
	test_qz = 0.615524;
	quat2euler_convertor(test_qw, test_qx, test_qy, test_qz);

