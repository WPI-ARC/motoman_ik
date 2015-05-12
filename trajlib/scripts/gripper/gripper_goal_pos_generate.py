import os;
import sys;
import copy;

from math import cos, sin;

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../scripts"))
from euler_quaternion_quick_converter import euler2quat_converter, ratio;
from motoman_configuration import arm_right_init, arm_left_init;

# All using the same Init Pos now
left_arm_init_joint_value = [2.193856910816974, 1.1630675621113802, 0.852437058039672, 1.113211995331904, 0.8438088567310283, -1.0094747189949542, 0.24438780016629988];
right_arm_init_joint_value = [2.5794765930828296, 1.3620727097356629, 1.3831275005664025, 0.7845256389316293, -3.057076564078304, -1.7625990915019676, 1.3096307216010097];

left_arm_torso_init_joint_value = arm_left_init;
right_arm_torso_init_joint_value = arm_right_init;

gripper_pitch_ang = 90;
roll = -180;
yaw = 90;

# Modulate
Safe_distance = 0.3;
Z_shift_value = 0.07;

# Gripper dimension
GripperLength = 0.2;
FingerLength = 0.15;
# Bin dimension Unit m
Bin_depth = 0.43;
Modulate = 0.01;

LeftBin_width = 0.250;
MiddleBin_width = 0.300;
RightBin_width = 0.250;

WorkBase_Height = 0.8350;
BottomLay_Height = 0.225;
SecndLayer_Height = 0.225;
ThirdLayer_Height = 0.227;
TopLayer_Height = 0.263;

Left_horizontal_ShiftValue = MiddleBin_width/2 + LeftBin_width/2;
Right_horizontal_ShiftValue = MiddleBin_width/2 + RightBin_width/2;

Level4 = WorkBase_Height + BottomLay_Height + SecndLayer_Height + ThirdLayer_Height + TopLayer_Height/2 + Z_shift_value;
Level3 = WorkBase_Height + BottomLay_Height + SecndLayer_Height + ThirdLayer_Height/2 + Z_shift_value;
Level2 = WorkBase_Height + BottomLay_Height + SecndLayer_Height/2 + Z_shift_value;
Level1 = WorkBase_Height + BottomLay_Height/2 + Z_shift_value;

class testpnt:
    def __init__(self):
        self.bin_num = chr(ord('S'));
        self.pnt_property = "test_pos";
        self.x = 0;
        self.y = 0;
        self.z = 0;
        self.qx = 0;
        self.qy = 0;
        self.qz = 0;
        self.qw = 0;
        
def generate_goal_points(Bin_base_x, Bin_base_y, Bin_base_z, using_torso = False):

    
    default_orientation_w =  -0.488244
    default_orientation_x =  -0.484592
    default_orientation_y =  0.384602
    default_orientation_z =  0.615524

    Entry_X_shiftvalue = Bin_base_x - Bin_depth - Safe_distance - GripperLength - FingerLength*cos(gripper_pitch_ang*ratio);

    goal_pos = [];
    # Setting Configuration:
    #	A		B		C
    #	D		E		F
    #	G		H		I
    #	J		K		L
    #		   Base

    binA = testpnt();
    binA.bin_num = "A";
    binA.x = Entry_X_shiftvalue;
    binA.y = Bin_base_y + Left_horizontal_ShiftValue;
    binA.z = Bin_base_z + Level4;
    binA.qx = default_orientation_x;
    binA.qy = default_orientation_y;
    binA.qz = default_orientation_z;
    binA.qw = default_orientation_w;
    goal_pos.append(binA);

    binB = testpnt();
    binB.bin_num = "B";
    binB.x = Entry_X_shiftvalue;
    binB.y = Bin_base_y;
    binB.z = Bin_base_z + Level4;
    binB.qx = default_orientation_x;
    binB.qy = default_orientation_y;
    binB.qz = default_orientation_z;
    binB.qw = default_orientation_w;
    goal_pos.append(binB);

    binC = testpnt();
    binC.bin_num = "C";
    binC.x = Entry_X_shiftvalue;
    binC.y = Bin_base_y - Right_horizontal_ShiftValue;
    binC.z = Bin_base_z + Level4;
    binC.qx = default_orientation_x;
    binC.qy = default_orientation_y;
    binC.qz = default_orientation_z;
    binC.qw = default_orientation_w;
    if using_torso:
		goal_pos.append(binC);

    binD = testpnt();
    binD.bin_num = "D";
    binD.x = Entry_X_shiftvalue;
    binD.y = Bin_base_y + Left_horizontal_ShiftValue;
    binD.z = Bin_base_z + Level3;
    binD.qx = default_orientation_x;
    binD.qy = default_orientation_y;
    binD.qz = default_orientation_z;
    binD.qw = default_orientation_w;
    goal_pos.append(binD);

    binE = testpnt();
    binE.bin_num = "E";
    binE.x = Entry_X_shiftvalue;
    binE.y = Bin_base_y;
    binE.z = Bin_base_z + Level3;
    binE.qx = default_orientation_x;
    binE.qy = default_orientation_y;
    binE.qz = default_orientation_z;
    binE.qw = default_orientation_w;
    goal_pos.append(binE);

    binF = testpnt();
    binF.bin_num = "F";
    binF.x = Entry_X_shiftvalue;
    binF.y = Bin_base_y - Right_horizontal_ShiftValue;
    binF.z = Bin_base_z + Level3;
    binF.qx = default_orientation_x;
    binF.qy = default_orientation_y;
    binF.qz = default_orientation_z;
    binF.qw = default_orientation_w;
    if using_torso:
		goal_pos.append(binF);

    binG = testpnt();
    binG.bin_num ="G";
    binG.x = Entry_X_shiftvalue;
    binG.y = Bin_base_y + Left_horizontal_ShiftValue;
    binG.z = Bin_base_z + Level2;
    binG.qx = default_orientation_x;
    binG.qy = default_orientation_y;
    binG.qz = default_orientation_z;
    binG.qw = default_orientation_w;
    goal_pos.append(binG);

    binH = testpnt();
    binH.bin_num = "H";
    binH.x = Entry_X_shiftvalue + 3*Modulate;
    binH.y = Bin_base_y;
    binH.z = Bin_base_z + Level2;
    binH.qx = default_orientation_x;
    binH.qy = default_orientation_y;
    binH.qz = default_orientation_z;
    binH.qw = default_orientation_w;
    goal_pos.append(binH);

    #print "Bin H, Pos_X: ",binH.x," Pos_Y:", binH.y, "Pos_Z:", binH.z;
    binI = testpnt();
    binI.bin_num = "I";
    binI.x = Entry_X_shiftvalue;
    binI.y = Bin_base_y - Right_horizontal_ShiftValue;
    binI.z = Bin_base_z + Level2;
    binI.qx = default_orientation_x;
    binI.qy = default_orientation_y;
    binI.qz = default_orientation_z;
    binI.qw = default_orientation_w;
    if using_torso:
		goal_pos.append(binI);

    binJ = testpnt();
    binJ.bin_num = "J";
    binJ.x = Entry_X_shiftvalue;
    binJ.y = Bin_base_y + Left_horizontal_ShiftValue;
    binJ.z = Bin_base_z + Level1;
    binJ.qx = default_orientation_x;
    binJ.qy = default_orientation_y;
    binJ.qz = default_orientation_z;
    binJ.qw = default_orientation_w;
    goal_pos.append(binJ);

    binK = testpnt();
    binK.bin_num = "K";
    binK.x = Entry_X_shiftvalue;
    binK.y = Bin_base_y;
    binK.z = Bin_base_z + Level1;
    binK.qx = default_orientation_x;
    binK.qy = default_orientation_y;
    binK.qz = default_orientation_z;
    binK.qw = default_orientation_w;
    goal_pos.append(binK);

    binL = testpnt();
    binL.bin_num = "L";
    binL.x = Entry_X_shiftvalue;
    binL.y = Bin_base_y - Right_horizontal_ShiftValue;
    binL.z = Bin_base_z + Level1;
    binL.qx = default_orientation_x;
    binL.qy = default_orientation_y;
    binL.qz = default_orientation_z;
    binL.qw = default_orientation_w;
    if using_torso:
		goal_pos.append(binL);

    return goal_pos;

class Jnt_state_goal:
    def __init__(self):
        self.bin_num = chr(ord('S'));
        self.pos_property = "test_pos";
        self.jnt_val = [];

def generate_left_arm_seed_state():

    seed_state = [];

    Goal1 = Jnt_state_goal();
    Goal1.bin_num = chr(ord('A'));
    Goal1.jnt_val = [ 1.2946151012717504, -1.363382168130896, -1.6617924058050058, -1.695796534346829, -0.779241703517125, -0.23640897614768297, 1.8915492395648426];
    seed_state.append(Goal1);

    Goal2 = Jnt_state_goal();
    Goal2.bin_num = chr(ord('A')+1);
    Goal2.jnt_val = [2.0470397118623382, -1.5900951732258286, 2.8176173676670175, -0.9284428878232793, 1.1380462986024937, 1.7034889656174173, -0.9567357683589586];
    seed_state.append(Goal2);



    Goal4 = Jnt_state_goal();
    Goal4.bin_num = chr(ord('A')+3);
    Goal4.jnt_val = [2.4580967147605235, -0.5446729671123386, 2.665470740770432, -2.1188655843905315, 1.2066158758731622, 1.6791622731613003, -0.841630540918266];
    seed_state.append(Goal4);

    Goal5 = Jnt_state_goal();
    Goal5.bin_num = chr(ord('A')+4);
    Goal5.jnt_val = [2.348465957508067, -1.40668371027442, 2.8710441151821295, -1.7654378467884415, 1.0416194682374236, 1.858653399311216, -0.3811025334024448];
    seed_state.append(Goal5);



    Goal7 = Jnt_state_goal();
    Goal7.bin_num = chr(ord('A')+6);
    Goal7.jnt_val = [-2.0370711246306104, -1.8295192457063296, -1.8451202902468806, 2.321145735151722, -3.0077142989456918, 1.2251922979462764, -2.2494549963774353];
    seed_state.append(Goal7);

    Goal8 = Jnt_state_goal();
    Goal8.bin_num = chr(ord('A')+7);
    Goal8.jnt_val = [-0.9916030724162687, 1.8642688181352713, -2.338997191523704, 2.0092924404247343, -2.3411044399524106, 1.3314934519202497, 0.7778852700524742];
    seed_state.append(Goal8);

    Goal10 = Jnt_state_goal();
    Goal10.bin_num = chr(ord('A')+9);
    Goal10.jnt_val = [-1.2948852785761256, -1.2403729592369999, -1.3933085254603284, 2.080249468059148, -0.8159017944741344, -0.37292349882999726, 2.351640933343906];
    seed_state.append(Goal10);

    Goal11 = Jnt_state_goal();
    Goal11.bin_num = chr(ord('A')+10);
    Goal11.jnt_val = [2.999197954039974, -1.8837653317964884, -2.198914825912091, -1.8150615006544348, 0.37585648711487446, 1.856511694456621, 0.7893266850603321];
    seed_state.append(Goal11);


    return seed_state;

def generate_left_arm_torso_seed_state():
	#
    seed_state = [];
    
    # torso + arm_joint

    Goal1 = Jnt_state_goal();
    Goal1.bin_num = chr(ord('A'));
    Goal1.jnt_val = [ 0.5341196238015118, 1.8156615932578113, -1.275265977286483, -2.609858966071363, -1.3034845539608502, 1.9796660131629278, 0.4239890905527271, -1.0985255476537754];
    seed_state.append(Goal1);
    
    Goal2 = Jnt_state_goal();
    Goal2.bin_num = chr(ord('A')+1);
    Goal2.jnt_val = [0.44444070735153696, 1.5931255200226575, -1.7214991803074458, -2.6669075131965516, -1.0965776606853834, 2.1108129990771083, 0.7574812090478747, -0.8348765029195171];
    seed_state.append(Goal2);    
    
    Goal3 = Jnt_state_goal();
    Goal3.bin_num = chr(ord('A')+2);
    Goal3.jnt_val = [-0.5712315987856218, 1.7310835851255508, -1.8270932042424857, -2.3749843887318813, -0.9268624667998937, 1.7546861558013398, 1.440749632864523, -0.6195602620210839];
    seed_state.append(Goal3);
    
    Goal4 = Jnt_state_goal();
    Goal4.bin_num = chr(ord('A')+3);
    Goal4.jnt_val = [0.17963051125465282, 1.6003692363633146, -0.68919946850517, -2.3665136202766965, -2.196021510260908, 1.1336085322639358, 0.7732147634900484, -0.4117695277695402];
    seed_state.append(Goal4);

    Goal5 = Jnt_state_goal();
    Goal5.bin_num = chr(ord('A')+4);
    Goal5.jnt_val = [-0.3436048682188014, 1.5510079630492097, -1.306013564008872, -2.594995589518679, -2.10634368384181, 1.463655797447624, 1.3880318290906768, 0.030162173127418777];
    seed_state.append(Goal5);
    
    Goal6 = Jnt_state_goal();
    Goal6.bin_num = chr(ord('A')+5);
    Goal6.jnt_val = [-0.8605064142164593, 1.8243318565668645, -1.6943853002581684, -2.33813482214666, -1.5990311110236555, 1.5698842214789754, 1.6154993982084753, 0.040729607620621826];
    seed_state.append(Goal6);
    
    Goal7 = Jnt_state_goal();
    Goal7.bin_num = chr(ord('A')+6);
    Goal7.jnt_val = [00.41731684212932757, 2.344646816300029, -0.9078876679077642, -2.513736672060711, -2.3308696636651045, 0.1973898972618137, 0.9693770329559368, 0.2312578571341544];
    seed_state.append(Goal7);
    
    Goal8 = Jnt_state_goal();
    Goal8.bin_num = chr(ord('A')+7);
    Goal8.jnt_val = [0.044055699180079705, -0.1681879334391681, 1.4226833054137182, 0.1327794592839025, -2.1506247197814794, -2.9154697502322597, -1.8999900136511538, 3.129939779097726];
    seed_state.append(Goal8);
    
    Goal9 = Jnt_state_goal();
    Goal9.bin_num = chr(ord('A')+8);
    Goal9.jnt_val = [-0.6844873753652071, -2.3058553081003335, -1.7088771512284773, 2.4105875129098844, -1.893805974678561, 2.024391222108095, -1.882680957194224, 2.070363154158572];
    seed_state.append(Goal9);
    
    Goal10 = Jnt_state_goal();
    Goal10.bin_num = chr(ord('A')+9);
    Goal10.jnt_val = [0.3375523245375861, -1.7505955792983339, -1.0792854934519172, 2.396496942110969, -2.0049487838514004, 2.4198581770138716, -0.6146628133824985, 2.5054592046864896];
    seed_state.append(Goal10);     
    
    Goal11 = Jnt_state_goal();
    Goal11.bin_num = chr(ord('A')+10);
    Goal11.jnt_val = [-0.5131183239739836, 1.3175085297094153, 0.9492773380110359, -0.26893080054061724, -2.219532349291596, -1.1195876267269027, 1.87585320864217, -0.3091419330856801];
    seed_state.append(Goal11);
    
    Goal12 = Jnt_state_goal();
    Goal12.bin_num = chr(ord('A')+11);
    Goal12.jnt_val = [-0.8107283536120312, 1.1885068501680074, 1.4407099293599144, -0.5506253027562428, -1.777021554647835, -1.1800589893491018, 1.8965499173550684, -0.505319212428926];
    seed_state.append(Goal12); 
    
    return seed_state;
    
def generate_key_joint_state(group_name):
	key_joint_state_set = [];
	
	Init_pos = Jnt_state_goal();
	Init_pos.bin_num = "Start";
	Init_pos.pos_property = "init_pos";
	drop_pos = Jnt_state_goal();
	drop_pos.bin_num = "End";
	drop_pos.pos_property = "drop_pos";
	
	if group_name == "arm_left":
		Init_pos.jnt_val = left_arm_init_joint_value;
		drop_pos.jnt_val = left_arm_init_joint_value;
	elif group_name == "arm_left_torso":
		Init_pos.jnt_val = left_arm_torso_init_joint_value;
		drop_pos.jnt_val = left_arm_torso_init_joint_value;
		
	elif group_name == "arm_right":
		Init_pos.jnt_val = right_arm_init_joint_value;
		drop_pos.jnt_val = right_arm_init_joint_value;
	elif group_name == "arm_right_torso":
		Init_pos.jnt_val = right_arm_torso_init_joint_value;
		drop_pos.jnt_val = right_arm_torso_init_joint_value;
		
	key_joint_state_set.append(Init_pos);
	key_joint_state_set.append(drop_pos);
	
	return key_joint_state_set;

