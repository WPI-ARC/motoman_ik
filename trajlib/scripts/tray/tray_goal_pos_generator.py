
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
	# Horizontal
	#Goal1.jnt_val = [2.8286594932856604,1.4258209109299205, -0.1868867195525305, -2.949978169671912, -2.281652360113256, 1.7588712832519384, -1.8896306445671973, -1.8664820311376307];
	Goal1.jnt_val = [2.608074188232422, -0.29658669233322144, 0.8934586644172668, 1.7289633750915527, 1.573803424835205, 1.2867212295532227, 1.4699939489364624, -2.8265552520751953];
	#seed_state_set.append(Goal1);

	Goal2 = config_goal();
	Goal2.bin_num = "B";
	Goal2.jnt_val = [-2.5886653285200394, -2.374620051506527, 1.8133726045423784, -1.8764388649633008, 0.9006129161380904, -0.7376122309261526, -1.8880190429049368, -1.4743091056932842];
	#seed_state_set.append(Goal2);
	
	Goal3 = config_goal();
	Goal3.bin_num = "C";
	Goal3.jnt_val = [ 0.10225791436341165, 0.7835974930475644, -1.6773199945660098, 1.49905452509214, 1.3427193003920177, 3.1189284019155186, -1.4674104840922055, -0.9027630644540776];
	#seed_state_set.append(Goal3);
	
	
	Goal4 = config_goal();
	Goal4.bin_num = "D";
	# test:2.905459411335167, 1.9791786685536243, 0.44221823283736134, -0.15580747083376772, 2.2349300507549397, 1.8704137525544255, 1.8999704363459953, 2.0352516343894233
	Goal4.jnt_val = [2.905459411335167, 1.9791786685536243, 0.44221823283736134, -0.15580747083376772, 2.2349300507549397, 1.8704137525544255, 1.8999704363459953, -2.7326506516615354];
	#seed_state_set.append(Goal4);
	
	Goal5 = config_goal();
	Goal5.bin_num = "E";
	Goal5.jnt_val = [2.89667019844055176, 1.4224945306777954, 
					-0.7801656126976013, -0.2995363175868988, 
					 2.195582151412964, 1.864424467086792, 
					 1.6602683067321777, 2.2383474826812744];
	seed_state_set.append(Goal5);
	
	Goal6 = config_goal();
	Goal6.bin_num = "F";
	#Goal6.jnt_val = [0.24514562009995183, 0.6469494458388144, -1.0091301947299118, 0.9320973548089223, 1.992558226545299, 2.471672159247223, -1.5916865933053375, -0.1546854272462241];
	Goal6.jnt_val = [1.7063605500004102,1.1749238170939902, 
					-1.826817689242944, -1.1213039820084452, 
					 1.5413271651472102, -1.749239622770513,
					 -1.8439300754469097, 1.7140113060163809];
	#seed_state_set.append(Goal6);
	
	Goal7 = config_goal();
	Goal7.bin_num = "G";
	# Horizontal
	#Goal7.jnt_val = [2.776894014401466,-0.639702221407827, 1.4694694444791978, -0.8327041049818699, 2.18906750147502, 2.2708616336783396, -1.232302856727945, 1.7149700732880278];

	Goal7.jnt_val = [2.4718597530192796, 1.1048811085600885, 
					  1.8289698505492917, -2.1170583249526715, 
					  -2.089052808535928, -2.178255911290856, 
					  1.5745535013303766, -1.735037580794114];
	seed_state_set.append(Goal7);
	
	Goal8 = config_goal();
	Goal8.bin_num = "H";
	Goal8.jnt_val = [2.89966156482696533, 1.8770301342010498, 
					 1.2306787967681885, -0.586269199848175, 
					 2.2546935081481934, 1.669684886932373, 
					 1.7160991430282593, 0.7149554491043091];
	#seed_state_set.append(Goal8);
	
	Goal9 = config_goal();
	Goal9.bin_num = "I";
	#Goal9.jnt_val = [-0.2684631401271395,-0.5944761002154216, 
	#				  1.025978429821326, 2.5126240735616863, 
	#				  2.163912575069292, 1.741182533849802, 
	#				  -1.895858443577232, 0.38680147950133437];
	
	#Goal9.jnt_val = [1.5194591283798218, 1.241114845275879, 
	#				-1.8047455549240112, 2.234393606185913, 
	#				-1.9810069799423218, 1.1204286813735962,
	#				-1.817457070350647, 0.8016403913497925];

	#Goal9.jnt_val = [1.3418513542538393, -1.9163393148721648, 
	#				 1.8999111796476877, 1.9555683274308242, 
	#				 2.285973354202339, 0.8327696820366999, 
	#				 1.621983626079816, 0.9235781887349414]
	
	#seed_state_set.append(Goal9);
	
	Goal10 = config_goal();
	Goal10.bin_num = "J";
	Goal10.jnt_val = [2.608074188232422, 0.4578932821750641, 1.8810696601867676, -0.5525216460227966, 1.9467278718948364, 0.23977181315422058, 0.7547944784164429, -0.43715447187423706];
	#seed_state_set.append(Goal10);
	
	Goal11 = config_goal();
	Goal11.bin_num = "K";
	Goal11.jnt_val = [2.89667019844055176, -0.873210072517395, -0.5380352735519409, 2.7276151180267334, -2.2068514823913574, 1.085071086883545, 1.8169622421264648, 1.6070705652236938];
	#seed_state_set.append(Goal11);
	
	Goal12 = config_goal();
	Goal12.bin_num = "L";
	#Goal12.jnt_val = [-0.2858043091428526, -1.5302684719380377, 1.4227702644339293, -2.1745498037104967, 2.218658549291765, -0.6976578836771063, 0.46488220398513513, 2.908319709480955];
	#Goal12.jnt_val = [1.7551809549331665, 0.04665006324648857, 
	#				 -1.8453619480133057, 1.8693605661392212, 
	#				 -1.189427375793457, 1.5698546171188354, 
	#				 -1.871213436126709, 0.8811066150665283];
	Goal12.jnt_val = [0.9074255864220171,0.08074085792646817, -1.1945154197247605, -1.106070858741708, 1.9850889243853769, 1.1827696548230184, 1.8891019593508724, -3.124429110575666];
 
	#seed_state_set.append(Goal12);
	
	return seed_state_set;
