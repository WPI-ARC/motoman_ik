
import os;
import sys;
import copy;
import math;
from math import cos, sin, pi
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../scripts"))
from euler_quaternion_quick_converter import euler2quat_converter;
from motoman_configuration import arm_right_home, arm_left_home;

# from calibration import bin_x, bin_y, bin_z;

# Bin dimension (Unit m)

#	A		B		C
#	D		E		F
#	G		H		I
#	J		K		L
#		   Base

Bin_depth = 0.430;

LeftBin_width = 0.250;
MiddleBin_width = 0.300;
RightBin_width = 0.250;

WorkBase_Height = 0.8350;
BottomLay_Height = 0.225;
SecndLayer_Height = 0.225;
ThirdLayer_Height = 0.227;
TopLayer_Height = 0.263;

Level4 = WorkBase_Height + BottomLay_Height + SecndLayer_Height + ThirdLayer_Height;
Level3 = WorkBase_Height + BottomLay_Height + SecndLayer_Height;
Level2 = WorkBase_Height + BottomLay_Height;
Level1 = WorkBase_Height;

Safe_Gap = 0.05;
ratio = pi/180;

# Tool dimension
Tool_length = 0.5;
Wrist_length = 0.07 # Cylinder

# Tray angle quaternion calculation
scoop_angle = 30;
tray_roll_ang = 90 + scoop_angle;
tray_pitch_ang = 90;
tray_yaw_ang = -180;

class goal_pnt:
	def __init__(self):
		self.bin_num = 0;
		self.pnt_property = "test_pos";
		self.x = 0;
		self.y = 0;
		self.z = 0;
		self.qx = 0;
		self.qy = 0;
		self.qz = 0;        
		self.qw = 0;
		
start_pnt = goal_pnt();
start_pnt.pnt_property = "start_pos";
start_pnt.x = 0.20257;
start_pnt.y = -0.58015;
start_pnt.z = 0.75699;
start_pnt.qw = 0.50227;
start_pnt.qx = -0.50067;
start_pnt.qy = -0.50182;
start_pnt.qz =  0.4952;


def generate_key_positions(Bin_base_x, Bin_base_y, Bin_base_z):
	
	# Final offsets
	X_shiftvalue = Bin_base_x - Bin_depth - Safe_Gap - (Tool_length + Wrist_length)*cos(scoop_angle * ratio);
	Y_Left_horizontal_ShiftValue = MiddleBin_width/2 + LeftBin_width/2;
	Y_Right_horizontal_ShiftValue = MiddleBin_width/2 + RightBin_width/2;
	Z_shiftvalue = (Tool_length + Wrist_length)*sin(scoop_angle * ratio) - 0.09;
	
	key_point_set = [];
	
	level_4_bin_start_pnt = goal_pnt();
	level_4_bin_start_pnt.pnt_property = "level4_start";
	level_4_bin_start_pnt.x = X_shiftvalue;
	level_4_bin_start_pnt.y = -0.58015;
	level_4_bin_start_pnt.z = Z_shiftvalue + Level4;
	level_4_bin_start_pnt.qw = 0.50227;
	level_4_bin_start_pnt.qx = -0.50067;
	level_4_bin_start_pnt.qy = -0.50182;
	level_4_bin_start_pnt.qz = 0.4952;	
	key_point_set.append(level_4_bin_start_pnt);
	
	level_3_bin_start_pnt = goal_pnt();
	level_4_bin_start_pnt.pnt_property = "level3_start";
	level_3_bin_start_pnt.x = X_shiftvalue;
	level_3_bin_start_pnt.y = -0.58015;
	level_3_bin_start_pnt.z = Z_shiftvalue + Level3;
	level_3_bin_start_pnt.qw = 0.50227;
	level_3_bin_start_pnt.qx = -0.50067;
	level_3_bin_start_pnt.qy = -0.50182;
	level_3_bin_start_pnt.qz = 0.4952;	
	key_point_set.append(level_3_bin_start_pnt);
	
	level_2_bin_start_pnt = goal_pnt();
	level_4_bin_start_pnt.pnt_property = "level2_start";
	level_2_bin_start_pnt.x = X_shiftvalue;
	level_2_bin_start_pnt.y = -0.58015;
	level_2_bin_start_pnt.z = Z_shiftvalue + Level2;
	level_2_bin_start_pnt.qw = 0.50227;
	level_2_bin_start_pnt.qx = -0.50067;
	level_2_bin_start_pnt.qy = -0.50182;
	level_2_bin_start_pnt.qz = 0.4952;	
	key_point_set.append(level_2_bin_start_pnt);
	
	level_1_bin_start_pnt = goal_pnt();
	level_4_bin_start_pnt.pnt_property = "level1_start";
	level_1_bin_start_pnt.x = X_shiftvalue;
	level_1_bin_start_pnt.y = -0.58015;
	level_1_bin_start_pnt.z = Z_shiftvalue + Level1;
	level_1_bin_start_pnt.qw = 0.50227;
	level_1_bin_start_pnt.qx = -0.50067;
	level_1_bin_start_pnt.qy = -0.50182;
	level_1_bin_start_pnt.qz = 0.4952;	
	key_point_set.append(level_1_bin_start_pnt);
	
	return key_point_set;

def generate_frontface_positions(Bin_base_x, Bin_base_y, Bin_base_z):

	# Final offsets
	X_shiftvalue = Bin_base_x - Bin_depth - Safe_Gap - (Tool_length + Wrist_length)*cos(scoop_angle * ratio);
	Y_Left_horizontal_ShiftValue = MiddleBin_width/2 + LeftBin_width/2;
	Y_Right_horizontal_ShiftValue = MiddleBin_width/2 + RightBin_width/2;
	Z_shiftvalue = (Tool_length + Wrist_length)*sin(scoop_angle * ratio) - 0.09;
	
	goal_pos = []; 
	
	binA = goal_pnt();
	binA.x = X_shiftvalue;
	binA.y = Bin_base_y + Y_Left_horizontal_ShiftValue;
	binA.z = Bin_base_z + Level4 + Z_shiftvalue;
	binA.bin_num = "A";
	goal_pos.append(binA);

	binB = goal_pnt();
	binB.x = X_shiftvalue;
	binB.y = Bin_base_y;
	binB.z = Bin_base_z + Level4+ Z_shiftvalue;
	binB.bin_num = "B";
	goal_pos.append(binB);

	binC = goal_pnt();
	binC.x = X_shiftvalue;
	binC.y = Bin_base_y - Y_Right_horizontal_ShiftValue;
	binC.z = Bin_base_z + Level4 + Z_shiftvalue;
	binC.bin_num = "C";
	goal_pos.append(binC);

	binD = goal_pnt();
	binD.x = X_shiftvalue;
	binD.y = Bin_base_y + Y_Left_horizontal_ShiftValue;
	binD.z = Bin_base_z + Level3 + Z_shiftvalue;
	binD.bin_num = "D";
	goal_pos.append(binD);

	binE = goal_pnt();
	binE.x = X_shiftvalue;
	binE.y = Bin_base_y;
	binE.z = Bin_base_z + Level3 + Z_shiftvalue;
	binE.bin_num = "E";
	goal_pos.append(binE);

	binF = goal_pnt();
	binF.x = X_shiftvalue;
	binF.y = Bin_base_y - Y_Right_horizontal_ShiftValue;
	binF.z = Bin_base_z + Level3 + Z_shiftvalue;
	binF.bin_num = "F";
	goal_pos.append(binF);

	binG = goal_pnt();
	binG.x = X_shiftvalue;
	binG.y = Bin_base_y + Y_Left_horizontal_ShiftValue;
	binG.z = Bin_base_z + Level2 + Z_shiftvalue;
	binG.bin_num = "G";
	goal_pos.append(binG);

	binH = goal_pnt();
	binH.x = X_shiftvalue;
	binH.y = Bin_base_y;
	binH.z = Bin_base_z + Level2 + Z_shiftvalue;
	binH.bin_num = "H";
	goal_pos.append(binH);

	binI = goal_pnt();
	binI.x = X_shiftvalue;
	binI.y = Bin_base_y - Y_Right_horizontal_ShiftValue;
	binI.z = Bin_base_z + Level2 + Z_shiftvalue;
	binI.bin_num = "I";
	goal_pos.append(binI);

	binJ = goal_pnt();
	binJ.x = X_shiftvalue;
	binJ.y = Bin_base_y + Y_Left_horizontal_ShiftValue;
	binJ.z = Bin_base_z + Level1 + Z_shiftvalue;
	binJ.bin_num = "J";
	goal_pos.append(binJ);

	binK = goal_pnt();
	binK.x = X_shiftvalue;
	binK.y = Bin_base_y;
	binK.z = Bin_base_z + Level1 + Z_shiftvalue;
	binK.bin_num = "K";
	goal_pos.append(binK);

	binL = goal_pnt();
	binL.x = X_shiftvalue;
	binL.y = Bin_base_y - Y_Right_horizontal_ShiftValue;
	binL.z = Bin_base_z + Level1 + Z_shiftvalue;
	binL.bin_num = "L";
	goal_pos.append(binL);

	quat = euler2quat_converter(tray_roll_ang, tray_pitch_ang, tray_yaw_ang);
	
	for pos in goal_pos:	
		pos.qw = quat[0];
		pos.qx = quat[1];
		pos.qy = quat[2];
		pos.qz = quat[3];
	return goal_pos;

class config_goal:
	def __init__(self):
		self.bin_num = "";
		self.traj_property = "test_pos";
		self.jnt_val = [];

def seed_state_generator():
	seed_state_set = [];
	Goal1 = config_goal();
	Goal1.bin_num = "A";
	Goal1.jnt_val = [-2.494322617213551, 0.5236253858745525, -1.58927266388269, -1.5760277304868606, -0.8813624307206954, -0.6095330671746702, 1.8744956287775745, 2.176790229329603];
	seed_state_set.append(Goal1);

	Goal2 = config_goal();
	Goal2.bin_num = "B";
	Goal2.jnt_val = [-2.5886653285200394, -2.374620051506527, 1.8133726045423784, -1.8764388649633008, 0.9006129161380904, -0.7376122309261526, -1.8880190429049368, -1.4743091056932842];
	seed_state_set.append(Goal2);
	
	Goal3 = config_goal();
	Goal3.bin_num = "C";
	Goal3.jnt_val = [ 0.10225791436341165, 0.7835974930475644, -1.6773199945660098, 1.49905452509214, 1.3427193003920177, 3.1189284019155186, -1.4674104840922055, -0.9027630644540776];
	seed_state_set.append(Goal3);
	
	Goal4 = config_goal();
	Goal4.bin_num = "D";
	Goal4.jnt_val = [-2.9304159179992215, -1.188145482834881, 1.2413950394903648, 1.448796166496277, 2.1197289875770853, -0.2957380340693733, 0.22356084880954222, 2.779410685704981];
	seed_state_set.append(Goal4);
	
	Goal5 = config_goal();
	Goal5.bin_num = "E";
	Goal5.jnt_val = [-2.617078223150794, -1.1700249924714978, 1.8999579332309542, -1.098277334234239, -1.7992468515858204, 2.3379186721820004, 0.4868876973376934, 2.4699605634819988];
	seed_state_set.append(Goal5);
	
	Goal6 = config_goal();
	Goal6.bin_num = "F";
	Goal6.jnt_val = [0.24514562009995183, 0.6469494458388144, -1.0091301947299118, 0.9320973548089223, 1.992558226545299, 2.471672159247223, -1.5916865933053375, -0.1546854272462241];
	seed_state_set.append(Goal6);
	
	Goal7 = config_goal();
	Goal7.bin_num = "G";
	Goal7.jnt_val = [-2.849710887732425, -1.8455983324002612, 0.423630514960432, -0.9114784023155666, -2.217974920357377, 1.7475936923018869, -0.7608672375929674, -1.9079153926250387];
	seed_state_set.append(Goal7);
	
	Goal8 = config_goal();
	Goal8.bin_num = "H";
	Goal8.jnt_val = [-2.8232615277416735, 0.6046811180302917, -1.3199900812523193, -2.9213054464463113, -1.6166655544412436, 1.680257377517132, -1.54861721727236, -2.147159909325972];
	seed_state_set.append(Goal8);
	
	Goal9 = config_goal();
	Goal9.bin_num = "I";
	Goal9.jnt_val = [-0.2684631401271395, -0.5944761002154216, 1.025978429821326, 2.5126240735616863, 2.163912575069292, 1.741182533849802, -1.895858443577232, 0.38680147950133437];
	seed_state_set.append(Goal9);
	
	Goal10 = config_goal();
	Goal10.bin_num = "J";
	Goal10.jnt_val = [-1.8638487643791364, 0.03964241835102434, -1.3264222668862966, -0.42707390465510126, 1.5590911109617618, 1.6030578531357988, 0.1145294217592958, 2.0083975636970286];
	seed_state_set.append(Goal10);
	
	Goal11 = config_goal();
	Goal11.bin_num = "K";
	Goal11.jnt_val = [0.6461938727094164, -1.3830831244973643, 1.095973358496806, 0.45177256217465794, -2.2491703946246178, 2.1547659742086287, 1.5037163344320572, -2.16208327232433];
	seed_state_set.append(Goal11);
	
	Goal12 = config_goal();
	Goal12.bin_num = "L";
	Goal12.jnt_val = [-0.2858043091428526, -1.5302684719380377, 1.4227702644339293, -2.1745498037104967, 2.218658549291765, -0.6976578836771063, 0.46488220398513513, 2.908319709480955];
	seed_state_set.append(Goal12);
	
	return seed_state_set;
