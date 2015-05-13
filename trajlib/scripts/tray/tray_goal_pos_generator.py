
import os;
import sys;
import copy;
import math;
from math import cos, sin, pi
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../scripts"))
from euler_quaternion_quick_converter import euler2quat_converter;
from motoman_configuration import arm_right_init, arm_left_init;

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
	Goal1.jnt_val = [-2.7414562000821596, 0.35908996927034464, -1.0338458209745651, 0.9155596477651657, 1.4354311690354857, 2.0361323271088922, 1.5042305571563939, 1.7114496138561022];
	#seed_state_set.append(Goal1);

	Goal2 = config_goal();
	Goal2.bin_num = "B";
	Goal2.jnt_val = [-0.081586918733629, 1.568263068581441, -1.5600065565278145, -0.12640853623635262, 1.3818779719788765, 1.8057370642492034, -1.6627886081928707, 0.33500689702609654];
	#seed_state_set.append(Goal2);
	
	Goal3 = config_goal();
	Goal3.bin_num = "C";
	Goal3.jnt_val = [-0.00017003233637660742, 0.9476867391620163, -1.0973046218304017, 0.8007416131485688, 1.6354262550391043, 2.375895961885445, -1.3475549255271067, -0.3187735148088192];
	#seed_state_set.append(Goal3);
	
	Goal4 = config_goal();
	Goal4.bin_num = "D";
	Goal4.jnt_val = [-2.9317089196761317, -1.3891042771603888, 1.5551417996298735, -1.795130814567031, -1.959660141829467, -1.1281119250820135, -0.42715715260501547, 0.2481895577719651];
	#seed_state_set.append(Goal4);
	
	Goal5 = config_goal();
	Goal5.bin_num = "E";
	Goal5.jnt_val = [2.9569502758554647, -2.359048008419167, 1.0030676671104295, -2.9133625875265268, 2.0038547415944508, -1.202117633059331, -1.8691892376098491, -2.4821946960131354];
	#seed_state_set.append(Goal5);
	
	Goal6 = config_goal();
	Goal6.bin_num = "F";
	Goal6.jnt_val = [-0.1354916601858345, -1.4024311652242984, 0.6585454937449481, -0.039352751542353334, -2.2773070429260507, 1.8020177560841422, 1.5190178837225605, -2.871546413266448];
	#seed_state_set.append(Goal6);
	
	Goal7 = config_goal();
	Goal7.bin_num = "G";
	Goal7.jnt_val = [-2.5798535224392536, 1.0697790242434855, -1.0839641149950956, -1.2201908276215852, 2.02966228699635, 0.5189577687608469, -0.3282565335080377, 2.2993732434898195];
	#seed_state_set.append(Goal7);
	
	Goal8 = config_goal();
	Goal8.bin_num = "H";
	Goal8.jnt_val = [-2.248761887096955, 1.3522287218551485, -1.7969524759519773, -0.6772191051315353, 2.1173732752930943, -0.044749839368110944, -0.5070960571918733, 2.1482518074388928];
	#seed_state_set.append(Goal8);
	
	Goal9 = config_goal();
	Goal9.bin_num = "I";
	Goal9.jnt_val = [1.786625748813978, -2.009663767489044, 1.6610497535117468, 2.3018214865330067, 1.9959024320028638, 1.092379101763545, 1.7286319777156305, -0.9795453621545195];
	#seed_state_set.append(Goal9);
	
	Goal10 = config_goal();
	Goal10.bin_num = "J";
	Goal10.jnt_val = [-2.9380471248618245, 1.5273729645602547, 1.0371530902300101, 1.5252995298122611, -2.0966592360817806, -0.44308936434507046, 0.8177707416074836, 1.9581798762937488];
	#seed_state_set.append(Goal10);
	
	Goal11 = config_goal();
	Goal11.bin_num = "K";
	Goal11.jnt_val = [1.6091060643563821, -0.42601474698429737, 1.8786813516173775, -1.9590250404127998, 2.143914460770546, 2.089120054792059, -1.5320915492899372, -1.3181846257921348];
	#seed_state_set.append(Goal11);
	
	Goal12 = config_goal();
	Goal12.bin_num = "L";
	Goal12.jnt_val = [2.956911493198829, -0.13811920555100166, -1.8429633508275745, -0.3930214380483685, 1.4840606133756926, -0.4428208584099841, -1.764811157007895, -3.125611904804895];
	seed_state_set.append(Goal12);
	
	return seed_state_set;
