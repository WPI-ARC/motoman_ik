import copy;
import math;
from math import cos, sin, pi
from euler_quaternion_quick_converter import euler2quat_converter;

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

Safe_Gap = 0.1;
ratio = pi/180;

# Tool dimension
Tool_length = 0.5;
Wrist_length = 0.07 # Cylinder

# Tray angle quaternion calculation
scoop_angle = 30;
tray_roll_ang = 90 + scoop_angle;
tray_pitch_ang = 90;
tray_yaw_ang = -180;
quat = euler2quat_converter(tray_roll_ang, tray_pitch_ang, tray_yaw_ang);


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
	Goal1.jnt_val = [2.8864212093942743, 1.8094905114205873, -1.4240144585880297, 1.3683058615954278, -0.7880391411174132, -2.9700129826089583, 1.5037064195914491, 1.6786340183842678];
	seed_state_set.append(Goal1);

	Goal2 = config_goal();
	Goal2.bin_num = "B";
	Goal2.jnt_val = [2.009953280502425, 2.2771155175097357, -1.8999774493173929, -1.2509014312714184, 1.491977670942615, 0.90449080668376, 1.6598357784944817, -2.4636074731050925];
	seed_state_set.append(Goal2);
	
	Goal3 = config_goal();
	Goal3.bin_num = "C";
	Goal3.jnt_val = [-0.5427706523464673, -1.8117662455317785, 1.055254460603305, -2.473356466541643, 1.7056115543972246, -0.7905867813890765, 0.6695403583282108, -0.956317933173878];
	seed_state_set.append(Goal3);
	
	Goal4 = config_goal();
	Goal4.bin_num = "D";
	Goal4.jnt_val = [-1.7650875307268996, -2.455446774335427, 1.428416576356059, 2.9343399686233242, 1.2117964637357608, -3.092816371787612, -0.4252958791498084, 2.4263627959188114];
	seed_state_set.append(Goal4);
	
	Goal5 = config_goal();
	Goal5.bin_num = "E";
	Goal5.jnt_val = [-1.7352914416665217, -1.9841055604712607, 1.4581507650178642, 2.9306196609295454, 1.735971735506078, -3.1156033782721018, 0.0061378895603485225, 1.980907213066154];
	seed_state_set.append(Goal5);
	
	Goal6 = config_goal();
	Goal6.bin_num = "F";
	Goal6.jnt_val = [-1.8567830064091049, -1.0643199421130811, 1.6825956923312042, 2.9499959853804905, 1.6734996850688426, -3.1299481163889866, -0.04142915753582445, 1.0418575308973759];
	seed_state_set.append(Goal6);
	
	Goal7 = config_goal();
	Goal7.bin_num = "G";
	Goal7.jnt_val = [-2.3633872553884783, -2.1463992701333474, 1.0279539043006716, 2.088049961029411, 2.073708445168062, -2.419629662005361, 0.4890631580447461, 1.729345313343855];
	seed_state_set.append(Goal7);
	
	Goal8 = config_goal();
	Goal8.bin_num = "H";
	Goal8.jnt_val = [-2.3633872553884783, -2.1463992701333474, 1.0279539043006716, 2.088049961029411, 2.073708445168062, -2.419629662005361, 0.4890631580447461, 1.729345313343855];
	seed_state_set.append(Goal8);
	
	Goal9 = config_goal();
	Goal9.bin_num = "I";
	Goal9.jnt_val = [1.2119937572247355, 2.325292049350415, 1.6630654350377505, 1.2735072728121566, -2.3599780236161547, -0.9730277581154009, -1.8617584118607073, 1.6210698281876255];
	seed_state_set.append(Goal9);
	
	Goal10 = config_goal();
	Goal10.bin_num = "J";
	Goal10.jnt_val = [1.6091060643563821, -0.42601474698429737, 1.8786813516173775, -1.9590250404127998, 2.143914460770546, 2.089120054792059, -1.5320915492899372, -1.3181846257921348];
	seed_state_set.append(Goal10);
	
	Goal11 = config_goal();
	Goal11.bin_num = "K";
	Goal11.jnt_val = [1.6091060643563821, -0.42601474698429737, 1.8786813516173775, -1.9590250404127998, 2.143914460770546, 2.089120054792059, -1.5320915492899372, -1.3181846257921348];
	seed_state_set.append(Goal11);
	
	Goal12 = config_goal();
	Goal12.bin_num = "L";
	Goal12.jnt_val = [1.2119937572247355, 2.325292049350415, 1.6630654350377505, 1.2735072728121566, -2.3599780236161547, -0.9730277581154009, -1.8617584118607073, 1.6210698281876255];
	seed_state_set.append(Goal12);
	
	return seed_state_set;
