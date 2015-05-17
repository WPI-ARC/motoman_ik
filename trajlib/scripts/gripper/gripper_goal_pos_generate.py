
import os;
import sys;
import copy;

from math import cos, sin;

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../scripts"))
from euler_quaternion_quick_converter import euler2quat_converter, ratio;
from motoman_configuration import arm_right_home, arm_left_drop, arm_left_home;

# All using the same Init Pos now
left_arm_torso_init_joint_value = arm_left_home;
left_arm_torso_drop_joint_value = arm_left_drop;
right_arm_torso_init_joint_value = arm_right_home;

gripper_pitch_ang = 90;
roll = -180;
yaw = 90;

# Modulate

# 0.28 for bin A B C D E F G J K L
#Safe_distance = 0.23; 
Step_back = 0.1;
Z_shift_value = 0.04;

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
        
def generate_Scan_points(Bin_base_x, 
						 Bin_base_y, 
						 Bin_base_z, 
						 Extend_distance = 0.55,
						 default_orientation_w =  0.0,
						 default_orientation_x =  0.7,
						 default_orientation_y =  -0.7,
						 default_orientation_z =  0.0):
    
    scan_pnts = [];
    Entry_X_shiftvalue = Extend_distance;
    
# ----------------------- A ----------------------------
    binA_scan = testpnt();
    binA_scan.bin_num = "A";
    binA_scan.pnt_property = "scan_point";
    binA_scan.x = Entry_X_shiftvalue;
    binA_scan.y = Bin_base_y + Left_horizontal_ShiftValue;
    binA_scan.z = Bin_base_z + Level4 + Z_shift_value;
    binA_scan.qx = default_orientation_x;
    binA_scan.qy = default_orientation_y;
    binA_scan.qz = default_orientation_z;
    binA_scan.qw = default_orientation_w;
    scan_pnts.append(binA_scan);  

# ----------------------- B ----------------------------
    binB_scan = testpnt();
    binB_scan.bin_num = "B";
    binB_scan.pnt_property = "scan_point";
    binB_scan.x = Entry_X_shiftvalue;
    binB_scan.y = Bin_base_y;
    binB_scan.z = Bin_base_z + Level4 + Z_shift_value;
    binB_scan.qx = default_orientation_x;
    binB_scan.qy = default_orientation_y;
    binB_scan.qz = default_orientation_z;
    binB_scan.qw = default_orientation_w;
    scan_pnts.append(binB_scan);
    
# ----------------------- C ----------------------------
    binC_scan = testpnt();
    binC_scan.bin_num = "C";
    binC_scan.pnt_property = "scan_point";
    binC_scan.x = Entry_X_shiftvalue;
    binC_scan.y = Bin_base_y - Right_horizontal_ShiftValue;
    binC_scan.z = Bin_base_z + Level4 + Z_shift_value;
    binC_scan.qx = default_orientation_x;
    binC_scan.qy = default_orientation_y;
    binC_scan.qz = default_orientation_z;
    binC_scan.qw = default_orientation_w;
    scan_pnts.append(binC_scan); 

# ----------------------- D ----------------------------
    binD_scan = testpnt();
    binD_scan.bin_num = "D";
    binD_scan.pnt_property = "scan_point";
    binD_scan.x = Entry_X_shiftvalue;
    binD_scan.y = Bin_base_y + Left_horizontal_ShiftValue;
    binD_scan.z = Bin_base_z + Level3 + Z_shift_value;
    binD_scan.qx = default_orientation_x;
    binD_scan.qy = default_orientation_y;
    binD_scan.qz = default_orientation_z;
    binD_scan.qw = default_orientation_w;
    scan_pnts.append(binD_scan);

# ----------------------- E ----------------------------
    binE_scan = testpnt();
    binE_scan.bin_num = "E";
    binE_scan.pnt_property = "scan_point";
    binE_scan.x = Entry_X_shiftvalue;
    binE_scan.y = Bin_base_y;
    binE_scan.z = Bin_base_z + Level3 + Z_shift_value;
    binE_scan.qx = default_orientation_x;
    binE_scan.qy = default_orientation_y;
    binE_scan.qz = default_orientation_z;
    binE_scan.qw = default_orientation_w;
    scan_pnts.append(binE_scan);

# ----------------------- F ----------------------------
    binF_scan = testpnt();
    binF_scan.bin_num = "F";
    binF_scan.pnt_property = "scan_point";
    binF_scan.x = Entry_X_shiftvalue;
    binF_scan.y = Bin_base_y - Right_horizontal_ShiftValue;
    binF_scan.z = Bin_base_z + Level3 + Z_shift_value;
    binF_scan.qx = default_orientation_x;
    binF_scan.qy = default_orientation_y;
    binF_scan.qz = default_orientation_z;
    binF_scan.qw = default_orientation_w;
    scan_pnts.append(binF_scan); 

# ----------------------- G ----------------------------
    binG_scan = testpnt();
    binG_scan.bin_num = "G";
    binG_scan.pnt_property = "scan_point";
    binG_scan.x = Entry_X_shiftvalue;
    binG_scan.y = Bin_base_y + Left_horizontal_ShiftValue;
    binG_scan.z = Bin_base_z + Level2 + Z_shift_value;
    binG_scan.qx = default_orientation_x;
    binG_scan.qy = default_orientation_y;
    binG_scan.qz = default_orientation_z;
    binG_scan.qw = default_orientation_w;
    scan_pnts.append(binG_scan);      

# ----------------------- H ----------------------------
    binH_scan = testpnt();
    binH_scan.bin_num = "H";
    binH_scan.pnt_property = "scan_point";
    binH_scan.x = Entry_X_shiftvalue;
    binH_scan.y = Bin_base_y;
    binH_scan.z = Bin_base_z + Level2 + Z_shift_value;
    binH_scan.qx = default_orientation_x;
    binH_scan.qy = default_orientation_y;
    binH_scan.qz = default_orientation_z;
    binH_scan.qw = default_orientation_w;
    scan_pnts.append(binH_scan);

# ----------------------- I ----------------------------
    binI_scan = testpnt();
    binI_scan.bin_num = "I";
    binI_scan.pnt_property = "scan_point";
    binI_scan.x = Entry_X_shiftvalue;
    binI_scan.y = Bin_base_y - Right_horizontal_ShiftValue;
    binI_scan.z = Bin_base_z + Level2 + Z_shift_value;
    binI_scan.qx = default_orientation_x;
    binI_scan.qy = default_orientation_y;
    binI_scan.qz = default_orientation_z;
    binI_scan.qw = default_orientation_w;
    scan_pnts.append(binI_scan); 

# ----------------------- J ----------------------------
    binJ_scan = testpnt();
    binJ_scan.bin_num = "J";
    binJ_scan.pnt_property = "scan_point";
    binJ_scan.x = Entry_X_shiftvalue;
    binJ_scan.y = Bin_base_y + Left_horizontal_ShiftValue;
    binJ_scan.z = Bin_base_z + Level1 + Z_shift_value;
    binJ_scan.qx = default_orientation_x;
    binJ_scan.qy = default_orientation_y;
    binJ_scan.qz = default_orientation_z;
    binJ_scan.qw = default_orientation_w;
    scan_pnts.append(binJ_scan);      

# ----------------------- K ----------------------------
    binK_scan = testpnt();
    binK_scan.bin_num = "K";
    binK_scan.pnt_property = "scan_point";
    binK_scan.x = Entry_X_shiftvalue;
    binK_scan.y = Bin_base_y;
    binK_scan.z = Bin_base_z + Level1 + Z_shift_value;
    binK_scan.qx = default_orientation_x;
    binK_scan.qy = default_orientation_y;
    binK_scan.qz = default_orientation_z;
    binK_scan.qw = default_orientation_w;
    scan_pnts.append(binK_scan);

# ----------------------- L ----------------------------
    binL_scan = testpnt();
    binL_scan.bin_num = "L";
    binL_scan.pnt_property = "scan_point";
    binL_scan.x = Entry_X_shiftvalue;
    binL_scan.y = Bin_base_y - Right_horizontal_ShiftValue;
    binL_scan.z = Bin_base_z + Level1 + Z_shift_value;
    binL_scan.qx = default_orientation_x;
    binL_scan.qy = default_orientation_y;
    binL_scan.qz = default_orientation_z;
    binL_scan.qw = default_orientation_w;
    scan_pnts.append(binL_scan); 
    
    return scan_pnts;

def generate_Pick_points(Bin_base_x, 
						 Bin_base_y, 
						 Bin_base_z, 
						 Extend_distance = 0.42, 
						 pnt_property = 'PickPnt',
						 default_orientation_w =  -0.488244,
						 default_orientation_x =  -0.484592,
						 default_orientation_y =  0.384602,
						 default_orientation_z =  0.615524						 
						):
	
	#Entry_X_shiftvalue = Bin_base_x - Bin_depth - Safe_distance - GripperLength - FingerLength*cos(gripper_pitch_ang*ratio);
	Entry_X_shiftvalue = Extend_distance;
	print "Current X_shift value is:", Entry_X_shiftvalue;

	# Setting Configuration:
	#	A		B		C
	#	D		E		F
	#	G		H		I
	#	J		K		L
	#		   Base

	goal_pos = [];
	# ----------------------- A ----------------------------
	binA = testpnt();
	binA.bin_num = "A";
	binA.pnt_property = pnt_property;
	binA.x = Entry_X_shiftvalue;
	binA.y = Bin_base_y + Left_horizontal_ShiftValue;
	binA.z = Bin_base_z + Level4;
	binA.qx = default_orientation_x;
	binA.qy = default_orientation_y;
	binA.qz = default_orientation_z;
	binA.qw = default_orientation_w;
	goal_pos.append(binA);  

	# ----------------------- B ----------------------------
	binB_entrance = testpnt();
	binB_entrance.bin_num = "B";
	binB_entrance.pnt_property = pnt_property;
	binB_entrance.x = Entry_X_shiftvalue;
	binB_entrance.y = Bin_base_y;
	binB_entrance.z = Bin_base_z + Level4;
	binB_entrance.qx = default_orientation_x;
	binB_entrance.qy = default_orientation_y;
	binB_entrance.qz = default_orientation_z;
	binB_entrance.qw = default_orientation_w;
	goal_pos.append(binB_entrance);

	# ----------------------- C ----------------------------
	binC_entrance = testpnt();
	binC_entrance.bin_num = "C";
	binC_entrance.pnt_property = pnt_property;
	binC_entrance.x = Entry_X_shiftvalue;
	binC_entrance.y = Bin_base_y - Right_horizontal_ShiftValue;
	binC_entrance.z = Bin_base_z + Level4;
	binC_entrance.qx = default_orientation_x;
	binC_entrance.qy = default_orientation_y;
	binC_entrance.qz = default_orientation_z;
	binC_entrance.qw = default_orientation_w;
	goal_pos.append(binC_entrance); 

	# ----------------------- D ----------------------------
	binD_entrance = testpnt();
	binD_entrance.bin_num = "D";
	binD_entrance.pnt_property = pnt_property;
	binD_entrance.x = Entry_X_shiftvalue;
	binD_entrance.y = Bin_base_y + Left_horizontal_ShiftValue;
	binD_entrance.z = Bin_base_z + Level3;
	binD_entrance.qx = default_orientation_x;
	binD_entrance.qy = default_orientation_y;
	binD_entrance.qz = default_orientation_z;
	binD_entrance.qw = default_orientation_w;
	goal_pos.append(binD_entrance);      
	
	# ----------------------- E ----------------------------
	binE_entrance = testpnt();
	binE_entrance.bin_num = "E";
	binE_entrance.pnt_property = pnt_property;
	binE_entrance.x = Entry_X_shiftvalue;
	binE_entrance.y = Bin_base_y;
	binE_entrance.z = Bin_base_z + Level3 + Z_shift_value;
	binE_entrance.qx = default_orientation_x;
	binE_entrance.qy = default_orientation_y;
	binE_entrance.qz = default_orientation_z;
	binE_entrance.qw = default_orientation_w;
	goal_pos.append(binE_entrance);

	# ----------------------- F ----------------------------
	binF_entrance = testpnt();
	binF_entrance.bin_num = "F";
	binF_entrance.pnt_property = pnt_property;
	binF_entrance.x = Entry_X_shiftvalue;
	binF_entrance.y = Bin_base_y - Right_horizontal_ShiftValue;
	binF_entrance.z = Bin_base_z + Level3;
	binF_entrance.qx = default_orientation_x;
	binF_entrance.qy = default_orientation_y;
	binF_entrance.qz = default_orientation_z;
	binF_entrance.qw = default_orientation_w;
	goal_pos.append(binF_entrance); 

	# ----------------------- G ----------------------------
	binG_entrance = testpnt();
	binG_entrance.bin_num = "G";
	binG_entrance.pnt_property = pnt_property;
	binG_entrance.x = Entry_X_shiftvalue;
	binG_entrance.y = Bin_base_y + Left_horizontal_ShiftValue;
	binG_entrance.z = Bin_base_z + Level2;
	binG_entrance.qx = default_orientation_x;
	binG_entrance.qy = default_orientation_y;
	binG_entrance.qz = default_orientation_z;
	binG_entrance.qw = default_orientation_w;
	#goal_pos.append(binG_entrance);      

	# ----------------------- H ----------------------------
	binH_entrance = testpnt();
	binH_entrance.bin_num = "H";
	binH_entrance.pnt_property = pnt_property;
	binH_entrance.x = Entry_X_shiftvalue + 0.11;
	binH_entrance.y = Bin_base_y;
	binH_entrance.z = Bin_base_z + Level2;
	binH_entrance.qx = default_orientation_x;
	binH_entrance.qy = default_orientation_y;
	binH_entrance.qz = default_orientation_z;
	binH_entrance.qw = default_orientation_w;
	#goal_pos.append(binH_entrance);

	# ----------------------- I ----------------------------
	binI_entrance = testpnt();
	binI_entrance.bin_num = "I";
	binI_entrance.pnt_property = pnt_property;
	binI_entrance.x = Entry_X_shiftvalue;
	binI_entrance.y = Bin_base_y - Right_horizontal_ShiftValue;
	binI_entrance.z = Bin_base_z + Level2 + 0.03;
	binI_entrance.qx = default_orientation_x;
	binI_entrance.qy = default_orientation_y;
	binI_entrance.qz = default_orientation_z;
	binI_entrance.qw = default_orientation_w;
	#goal_pos.append(binI_entrance); 

	# ----------------------- J ----------------------------
	binJ_entrance = testpnt();
	binJ_entrance.bin_num = "J";
	binJ_entrance.pnt_property = pnt_property;
	binJ_entrance.x = Entry_X_shiftvalue;
	binJ_entrance.y = Bin_base_y + Left_horizontal_ShiftValue;
	binJ_entrance.z = Bin_base_z + Level1;
	binJ_entrance.qx = default_orientation_x;
	binJ_entrance.qy = default_orientation_y;
	binJ_entrance.qz = default_orientation_z;
	binJ_entrance.qw = default_orientation_w;
	#goal_pos.append(binJ_entrance);      

	# ----------------------- K ----------------------------
	binK_entrance = testpnt();
	binK_entrance.bin_num = "K";
	binK_entrance.pnt_property = pnt_property;
	binK_entrance.x = Entry_X_shiftvalue;
	binK_entrance.y = Bin_base_y;
	binK_entrance.z = Bin_base_z + Level1;
	binK_entrance.qx = default_orientation_x;
	binK_entrance.qy = default_orientation_y;
	binK_entrance.qz = default_orientation_z;
	binK_entrance.qw = default_orientation_w;
	#goal_pos.append(binK_entrance);

	# ----------------------- L ----------------------------
	binL_entrance = testpnt();
	binL_entrance.bin_num = "L";
	binL_entrance.pnt_property = pnt_property;
	binL_entrance.x = Entry_X_shiftvalue  + 0.05;
	binL_entrance.y = Bin_base_y - Right_horizontal_ShiftValue;
	binL_entrance.z = Bin_base_z + Level1;
	binL_entrance.qx = default_orientation_x;
	binL_entrance.qy = default_orientation_y;
	binL_entrance.qz = default_orientation_z;
	binL_entrance.qw = default_orientation_w;
	#goal_pos.append(binL_entrance); 
	
	return goal_pos;

def generate_goal_points(Bin_base_x, Bin_base_y, Bin_base_z, Extend_distance = 0.42):

    default_orientation_w =  -0.488244
    default_orientation_x =  -0.484592
    default_orientation_y =  0.384602
    default_orientation_z =  0.615524
    #Entry_X_shiftvalue = Bin_base_x - Bin_depth - Safe_distance - GripperLength - FingerLength*cos(gripper_pitch_ang*ratio);
    Entry_X_shiftvalue = Extend_distance;
    print "Current X_shift value is:", Entry_X_shiftvalue;

    # Setting Configuration:
    #	A		B		C
    #	D		E		F
    #	G		H		I
    #	J		K		L
    #		   Base

    goal_pos = [];
# ----------------------- A ----------------------------
    binA_entrance = testpnt();
    binA_entrance.bin_num = "A";
    binA_entrance.pnt_property = "Entrance_point";
    binA_entrance.x = Entry_X_shiftvalue;
    binA_entrance.y = Bin_base_y + Left_horizontal_ShiftValue;
    binA_entrance.z = Bin_base_z + Level4;
    binA_entrance.qx = default_orientation_x;
    binA_entrance.qy = default_orientation_y;
    binA_entrance.qz = default_orientation_z;
    binA_entrance.qw = default_orientation_w;
    goal_pos.append(binA_entrance);  
        
    binA_exit = testpnt();    
    binA_exit.bin_num = "A";
    binA_exit.pnt_property = "Exit_point";
    binA_exit.x = Entry_X_shiftvalue - Step_back;
    binA_exit.y = Bin_base_y + Left_horizontal_ShiftValue;
    binA_exit.z = Bin_base_z + Level4;
    binA_exit.qx = default_orientation_x;
    binA_exit.qy = default_orientation_y;
    binA_exit.qz = default_orientation_z;
    binA_exit.qw = default_orientation_w;
    goal_pos.append(binA_exit);

# ----------------------- B ----------------------------
    binB_entrance = testpnt();
    binB_entrance.bin_num = "B";
    binB_entrance.pnt_property = "Entrance_point";
    binB_entrance.x = Entry_X_shiftvalue;
    binB_entrance.y = Bin_base_y;
    binB_entrance.z = Bin_base_z + Level4;
    binB_entrance.qx = default_orientation_x;
    binB_entrance.qy = default_orientation_y;
    binB_entrance.qz = default_orientation_z;
    binB_entrance.qw = default_orientation_w;
    goal_pos.append(binB_entrance);
    
    binB_exit = testpnt();    
    binB_exit.bin_num = "B";
    binB_exit.pnt_property = "Exit_point";
    binB_exit.x = Entry_X_shiftvalue - Step_back;
    binB_exit.y = Bin_base_y;
    binB_exit.z = Bin_base_z + Level4;
    binB_exit.qx = default_orientation_x;
    binB_exit.qy = default_orientation_y;
    binB_exit.qz = default_orientation_z;
    binB_exit.qw = default_orientation_w;
    goal_pos.append(binB_exit);

# ----------------------- C ----------------------------
    binC_entrance = testpnt();
    binC_entrance.bin_num = "C";
    binC_entrance.pnt_property = "Entrance_point";
    binC_entrance.x = Entry_X_shiftvalue;
    binC_entrance.y = Bin_base_y - Right_horizontal_ShiftValue;
    binC_entrance.z = Bin_base_z + Level4;
    binC_entrance.qx = default_orientation_x;
    binC_entrance.qy = default_orientation_y;
    binC_entrance.qz = default_orientation_z;
    binC_entrance.qw = default_orientation_w;
    goal_pos.append(binC_entrance); 
       
    binC_exit = testpnt();    
    binC_exit.bin_num = "C";
    binC_exit.pnt_property = "Exit_point";
    binC_exit.x = Entry_X_shiftvalue - Step_back;
    binC_exit.y = Bin_base_y - Right_horizontal_ShiftValue;
    binC_exit.z = Bin_base_z + Level4;
    binC_exit.qx = default_orientation_x;
    binC_exit.qy = default_orientation_y;
    binC_exit.qz = default_orientation_z;
    binC_exit.qw = default_orientation_w;
    goal_pos.append(binC_exit);


# ----------------------- D ----------------------------
    binD_entrance = testpnt();
    binD_entrance.bin_num = "D";
    binD_entrance.pnt_property = "Entrance_point";
    binD_entrance.x = Entry_X_shiftvalue;
    binD_entrance.y = Bin_base_y + Left_horizontal_ShiftValue;
    binD_entrance.z = Bin_base_z + Level3;
    binD_entrance.qx = default_orientation_x;
    binD_entrance.qy = default_orientation_y;
    binD_entrance.qz = default_orientation_z;
    binD_entrance.qw = default_orientation_w;
    goal_pos.append(binD_entrance);      
    binD_exit = testpnt();    
    binD_exit.bin_num = "D";
    binD_exit.pnt_property = "Exit_point";
    binD_exit.x = Entry_X_shiftvalue - Step_back;
    binD_exit.y = Bin_base_y + Left_horizontal_ShiftValue;
    binD_exit.z = Bin_base_z + Level3;
    binD_exit.qx = default_orientation_x;
    binD_exit.qy = default_orientation_y;
    binD_exit.qz = default_orientation_z;
    binD_exit.qw = default_orientation_w;
    goal_pos.append(binD_exit);

# ----------------------- E ----------------------------
    binE_entrance = testpnt();
    binE_entrance.bin_num = "E";
    binE_entrance.pnt_property = "Entrance_point";
    binE_entrance.x = Entry_X_shiftvalue;
    binE_entrance.y = Bin_base_y;
    binE_entrance.z = Bin_base_z + Level3;
    binE_entrance.qx = default_orientation_x;
    binE_entrance.qy = default_orientation_y;
    binE_entrance.qz = default_orientation_z;
    binE_entrance.qw = default_orientation_w;
    goal_pos.append(binE_entrance);
    binE_exit = testpnt();    
    binE_exit.bin_num = "E";
    binE_exit.pnt_property = "Exit_point";
    binE_exit.x = Entry_X_shiftvalue - Step_back;
    binE_exit.y = Bin_base_y;
    binE_exit.z = Bin_base_z + Level3;
    binE_exit.qx = default_orientation_x;
    binE_exit.qy = default_orientation_y;
    binE_exit.qz = default_orientation_z;
    binE_exit.qw = default_orientation_w;
    goal_pos.append(binE_exit);

# ----------------------- F ----------------------------
    binF_entrance = testpnt();
    binF_entrance.bin_num = "F";
    binF_entrance.pnt_property = "Entrance_point";
    binF_entrance.x = Entry_X_shiftvalue;
    binF_entrance.y = Bin_base_y - Right_horizontal_ShiftValue;
    binF_entrance.z = Bin_base_z + Level3;
    binF_entrance.qx = default_orientation_x;
    binF_entrance.qy = default_orientation_y;
    binF_entrance.qz = default_orientation_z;
    binF_entrance.qw = default_orientation_w;
    goal_pos.append(binF_entrance); 
       
    binF_exit = testpnt();    
    binF_exit.bin_num = "F";
    binF_exit.pnt_property = "Exit_point";
    binF_exit.x = Entry_X_shiftvalue - Step_back;
    binF_exit.y = Bin_base_y - Right_horizontal_ShiftValue;
    binF_exit.z = Bin_base_z + Level3;
    binF_exit.qx = default_orientation_x;
    binF_exit.qy = default_orientation_y;
    binF_exit.qz = default_orientation_z;
    binF_exit.qw = default_orientation_w;
    goal_pos.append(binF_exit);

# ----------------------- G ----------------------------
    binG_entrance = testpnt();
    binG_entrance.bin_num = "G";
    binG_entrance.pnt_property = "Entrance_point";
    binG_entrance.x = Entry_X_shiftvalue;
    binG_entrance.y = Bin_base_y + Left_horizontal_ShiftValue;
    binG_entrance.z = Bin_base_z + Level2;
    binG_entrance.qx = default_orientation_x;
    binG_entrance.qy = default_orientation_y;
    binG_entrance.qz = default_orientation_z;
    binG_entrance.qw = default_orientation_w;
    goal_pos.append(binG_entrance);      
    binG_exit = testpnt();    
    binG_exit.bin_num = "G";
    binG_exit.pnt_property = "Exit_point";
    binG_exit.x = Entry_X_shiftvalue - Step_back;
    binG_exit.y = Bin_base_y + Left_horizontal_ShiftValue;
    binG_exit.z = Bin_base_z + Level2;
    binG_exit.qx = default_orientation_x;
    binG_exit.qy = default_orientation_y;
    binG_exit.qz = default_orientation_z;
    binG_exit.qw = default_orientation_w;
    goal_pos.append(binG_exit);

# ----------------------- H ----------------------------
    binH_entrance = testpnt();
    binH_entrance.bin_num = "H";
    binH_entrance.pnt_property = "Entrance_point";
    binH_entrance.x = Entry_X_shiftvalue;
    binH_entrance.y = Bin_base_y;
    binH_entrance.z = Bin_base_z + Level2;
    binH_entrance.qx = default_orientation_x;
    binH_entrance.qy = default_orientation_y;
    binH_entrance.qz = default_orientation_z;
    binH_entrance.qw = default_orientation_w;
    goal_pos.append(binH_entrance);
    binH_exit = testpnt();    
    binH_exit.bin_num = "H";
    binH_exit.pnt_property = "Exit_point";
    binH_exit.x = Entry_X_shiftvalue - Step_back;
    binH_exit.y = Bin_base_y;
    binH_exit.z = Bin_base_z + Level2;
    binH_exit.qx = default_orientation_x;
    binH_exit.qy = default_orientation_y;
    binH_exit.qz = default_orientation_z;
    binH_exit.qw = default_orientation_w;
    goal_pos.append(binH_exit);

# ----------------------- I ----------------------------
    binI_entrance = testpnt();
    binI_entrance.bin_num = "I";
    binI_entrance.pnt_property = "Entrance_point";
    binI_entrance.x = Entry_X_shiftvalue;
    binI_entrance.y = Bin_base_y - Right_horizontal_ShiftValue;
    binI_entrance.z = Bin_base_z + Level2;
    binI_entrance.qx = default_orientation_x;
    binI_entrance.qy = default_orientation_y;
    binI_entrance.qz = default_orientation_z;
    binI_entrance.qw = default_orientation_w;
    goal_pos.append(binI_entrance); 
       
    binI_exit = testpnt();    
    binI_exit.bin_num = "I";
    binI_exit.pnt_property = "Exit_point";
    binI_exit.x = Entry_X_shiftvalue - Step_back;
    binI_exit.y = Bin_base_y - Right_horizontal_ShiftValue;
    binI_exit.z = Bin_base_z + Level2;
    binI_exit.qx = default_orientation_x;
    binI_exit.qy = default_orientation_y;
    binI_exit.qz = default_orientation_z;
    binI_exit.qw = default_orientation_w;
    goal_pos.append(binI_exit);

# ----------------------- J ----------------------------
    binJ_entrance = testpnt();
    binJ_entrance.bin_num = "J";
    binJ_entrance.pnt_property = "Entrance_point";
    binJ_entrance.x = Entry_X_shiftvalue;
    binJ_entrance.y = Bin_base_y + Left_horizontal_ShiftValue;
    binJ_entrance.z = Bin_base_z + Level1;
    binJ_entrance.qx = default_orientation_x;
    binJ_entrance.qy = default_orientation_y;
    binJ_entrance.qz = default_orientation_z;
    binJ_entrance.qw = default_orientation_w;
    goal_pos.append(binJ_entrance);      
    binJ_exit = testpnt();    
    binJ_exit.bin_num = "J";
    binJ_exit.pnt_property = "Exit_point";
    binJ_exit.x = Entry_X_shiftvalue - Step_back;
    binJ_exit.y = Bin_base_y + Left_horizontal_ShiftValue;
    binJ_exit.z = Bin_base_z + Level1;
    binJ_exit.qx = default_orientation_x;
    binJ_exit.qy = default_orientation_y;
    binJ_exit.qz = default_orientation_z;
    binJ_exit.qw = default_orientation_w;
    goal_pos.append(binJ_exit);

# ----------------------- K ----------------------------
    binK_entrance = testpnt();
    binK_entrance.bin_num = "K";
    binK_entrance.pnt_property = "Entrance_point";
    binK_entrance.x = Entry_X_shiftvalue;
    binK_entrance.y = Bin_base_y;
    binK_entrance.z = Bin_base_z + Level1;
    binK_entrance.qx = default_orientation_x;
    binK_entrance.qy = default_orientation_y;
    binK_entrance.qz = default_orientation_z;
    binK_entrance.qw = default_orientation_w;
    goal_pos.append(binK_entrance);
    binK_exit = testpnt();    
    binK_exit.bin_num = "K";
    binK_exit.pnt_property = "Exit_point";
    binK_exit.x = Entry_X_shiftvalue - Step_back;
    binK_exit.y = Bin_base_y;
    binK_exit.z = Bin_base_z + Level1;
    binK_exit.qx = default_orientation_x;
    binK_exit.qy = default_orientation_y;
    binK_exit.qz = default_orientation_z;
    binK_exit.qw = default_orientation_w;
    goal_pos.append(binK_exit);

# ----------------------- L ----------------------------
    binL_entrance = testpnt();
    binL_entrance.bin_num = "L";
    binL_entrance.pnt_property = "Entrance_point";
    binL_entrance.x = Entry_X_shiftvalue;
    binL_entrance.y = Bin_base_y - Right_horizontal_ShiftValue;
    binL_entrance.z = Bin_base_z + Level1;
    binL_entrance.qx = default_orientation_x;
    binL_entrance.qy = default_orientation_y;
    binL_entrance.qz = default_orientation_z;
    binL_entrance.qw = default_orientation_w;
    goal_pos.append(binL_entrance); 
       
    binL_exit = testpnt();    
    binL_exit.bin_num = "L";
    binL_exit.pnt_property = "Exit_point";
    binL_exit.x = Entry_X_shiftvalue - Step_back;
    binL_exit.y = Bin_base_y - Right_horizontal_ShiftValue;
    binL_exit.z = Bin_base_z + Level1;
    binL_exit.qx = default_orientation_x;
    binL_exit.qy = default_orientation_y;
    binL_exit.qz = default_orientation_z;
    binL_exit.qw = default_orientation_w;
    goal_pos.append(binL_exit);
    return goal_pos;
    
class Jnt_state_goal:
    def __init__(self):
        self.bin_num = chr(ord('S'));
        self.pos_property = "test_pos";
        self.jnt_val = [];

def generate_left_arm_watch_config():
	watch_seed_state = [];

#---------------------------------- LEVEL 4 ----------------------------------------------
	GoalA = Jnt_state_goal();
	GoalA.bin_num = 'A';
	GoalA.pos_property = 'scan_pos';
	GoalA.jnt_val = [-0.39477163517091207, -1.4077955434713987, 0.016597422300892332, 0.48261441613885614, -1.8199331744395209, 1.9531320788400828, -0.6680145612267164, -2.400533715077793]
	watch_seed_state.append(GoalA);
	
	GoalB = Jnt_state_goal();
	GoalB.bin_num = 'B';
	GoalB.pos_property = 'scan_pos';
	GoalB.jnt_val = [-0.6520081913154009, -1.3518496957268928, 0.3071956756846225, 0.5301013664606998, -2.0353741179409406, 2.330295550278179, -1.0605288495864043, -3.0809613730275047]
	watch_seed_state.append(GoalB);
	
	GoalC = Jnt_state_goal();
	GoalC.bin_num = 'C';
	GoalC.pos_property = 'scan_pos';
	GoalC.jnt_val = [-0.8554662092290973, -1.2546435721887084, 0.9872819448489639, 0.8336318667356407, -1.7410502813833653, 2.539478526676659, -1.5853462878647213, -3.108681462366727]
	watch_seed_state.append(GoalC);
#---------------------------------- LEVEL 3 ----------------------------------------------
	GoalD = Jnt_state_goal();
	GoalD.bin_num = 'D';
	GoalD.pos_property = 'scan_pos';
	GoalD.jnt_val = [ -0.11689956351570152, -1.1841869905319233, 0.16731937070887654, 1.0781089516926308, -2.0462702782830284, 1.7779478425926578, -1.5248691400220453, -2.218266939588106]
	watch_seed_state.append(GoalD);
	
	GoalE = Jnt_state_goal();
	GoalE.bin_num = 'E';
	GoalE.pos_property = 'scan_pos';
	GoalE.jnt_val = [-0.5306548070997831, -1.1883422977785616, 0.3117524608514236, 1.037630557360579, -2.358607599753716, 1.905337154380018, -1.6789591857937636, -2.9885879356365246]
	watch_seed_state.append(GoalE);
	
	GoalF = Jnt_state_goal();
	GoalF.bin_num = 'F';
	GoalF.pos_property = 'scan_pos';
	GoalF.jnt_val = [-0.9721266550226851, -2.3823686623559834, -0.9540492131489646, 1.9036517961770574, -2.241875602377597, -1.6737967763491564, 0.6896186544551595, -0.5802294309583365]
	watch_seed_state.append(GoalF);
#---------------------------------- LEVEL 2 ----------------------------------------------
	GoalG = Jnt_state_goal();
	GoalG.bin_num = 'G';
	GoalG.pos_property = 'scan_pos';
	GoalG.jnt_val = [-0.06122219096813914, -1.6608391313874975, -0.798042800462775, 1.7924870070090773, -1.9945451144300332, -2.322112093391676, 1.3031523620494685, 1.0626014486439268]
	watch_seed_state.append(GoalG);

	GoalH = Jnt_state_goal();
	GoalH.bin_num = 'H';
	GoalH.pos_property = 'scan_pos';
	GoalH.jnt_val = [-0.38811636314364156, -1.6360910753613491, -1.038341589165388, 1.962749775823567, -2.182526011144044, -2.5431431366463, 1.1574689052480833, 0.5325639224977015]
	watch_seed_state.append(GoalH);

	GoalI = Jnt_state_goal();
	GoalI.bin_num = 'I';
	GoalI.pos_property = 'scan_pos';
	GoalI.jnt_val = [-0.8232119716585025, -1.7733433381364379, -1.2780521217468757, 1.994871501091125, -2.024166137948483, -2.7039831446770344, 1.073804093838212, 0.16879171196923723]
	watch_seed_state.append(GoalI);
#---------------------------------- LEVEL 1 ----------------------------------------------
	GoalJ = Jnt_state_goal();
	GoalJ.bin_num = 'J';
	GoalJ.pos_property = 'scan_pos';
	GoalJ.jnt_val = [-0.05591208449042534, -1.540142646599555, -1.029025928870858, 1.8583665232611106, -1.4436869434278028, -2.6028841562862257, 1.8214623022257945, 1.3491535752121777]
	watch_seed_state.append(GoalJ);

	GoalK = Jnt_state_goal();
	GoalK.bin_num = 'K';
	GoalK.pos_property = 'scan_pos';
	GoalK.jnt_val = [-0.3037673776575612, -1.5257706053327762, -1.372516536158061, 2.0303372636215054, -1.5452157225994279, -2.9578646581138264, 1.6965605084853133, 0.8374235452328564]
	watch_seed_state.append(GoalK);
	
	GoalL = Jnt_state_goal();
	GoalL.bin_num = 'L';
	GoalL.pos_property = 'scan_pos';
	GoalL.jnt_val = [-0.6286281250283243, -1.654018558131445, -1.5758725925311272, 2.009085958280058, -1.2795305187103037, -3.0784153728889168, 1.7655066199969274, 0.5587221533314563]
	watch_seed_state.append(GoalL);

	return watch_seed_state;

def generate_left_arm_torso_seed_state():
    seed_state = [];
    # torso + arm_joint

    Goal1 = Jnt_state_goal();
    Goal1.bin_num = 'A';
    Goal1.jnt_val = [ 0.5341196238015118, 1.8156615932578113, -1.275265977286483, -2.609858966071363, -1.3034845539608502, 1.9796660131629278, 0.4239890905527271, -1.0985255476537754];
    seed_state.append(Goal1);
    
    Goal2 = Jnt_state_goal();
    Goal2.bin_num = 'B';
    Goal2.jnt_val = [0.44444070735153696, 1.5931255200226575, -1.7214991803074458, -2.6669075131965516, -1.0965776606853834, 2.1108129990771083, 0.7574812090478747, -0.8348765029195171];
    seed_state.append(Goal2);    
    
    Goal3 = Jnt_state_goal();
    Goal3.bin_num = 'C';
    Goal3.jnt_val = [-0.5712315987856218, 1.7310835851255508, -1.8270932042424857, -2.3749843887318813, -0.9268624667998937, 1.7546861558013398, 1.440749632864523, -0.6195602620210839];
    seed_state.append(Goal3);
    
    Goal4 = Jnt_state_goal();
    Goal4.bin_num = 'D';
    Goal4.jnt_val = [0.17963051125465282, 1.6003692363633146, -0.68919946850517, -2.3665136202766965, -2.196021510260908, 1.1336085322639358, 0.7732147634900484, -0.4117695277695402];
    seed_state.append(Goal4);

    Goal5 = Jnt_state_goal();
    Goal5.bin_num = 'E';
    Goal5.jnt_val = [-0.3436048682188014, 1.5510079630492097, -1.306013564008872, -2.594995589518679, -2.10634368384181, 1.463655797447624, 1.3880318290906768, 0.030162173127418777];
    seed_state.append(Goal5);
    
    Goal6 = Jnt_state_goal();
    Goal6.bin_num = 'F';
    Goal6.jnt_val = [-0.8605064142164593, 1.8243318565668645, -1.6943853002581684, -2.33813482214666, -1.5990311110236555, 1.5698842214789754, 1.6154993982084753, 0.040729607620621826];
    seed_state.append(Goal6);
    
    Goal7 = Jnt_state_goal();
    Goal7.bin_num = 'G';
    Goal7.jnt_val = [00.41731684212932757, 2.344646816300029, -0.9078876679077642, -2.513736672060711, -2.3308696636651045, 0.1973898972618137, 0.9693770329559368, 0.2312578571341544];
    seed_state.append(Goal7);
    
    Goal8 = Jnt_state_goal();
    Goal8.bin_num = 'H';
    Goal8.jnt_val = [0.044055699180079705, -0.1681879334391681, 1.4226833054137182, 0.1327794592839025, -2.1506247197814794, -2.9154697502322597, -1.8999900136511538, 3.129939779097726];
    seed_state.append(Goal8);
    
    Goal9 = Jnt_state_goal();
    Goal9.bin_num = 'I';
    Goal9.jnt_val = [-0.6844873753652071, -2.3058553081003335, -1.7088771512284773, 2.4105875129098844, -1.893805974678561, 2.024391222108095, -1.882680957194224, 2.070363154158572];
    seed_state.append(Goal9);
    
    Goal10 = Jnt_state_goal();
    Goal10.bin_num = 'J';
    Goal10.jnt_val = [0.3375523245375861, -1.7505955792983339, -1.0792854934519172, 2.396496942110969, -2.0049487838514004, 2.4198581770138716, -0.6146628133824985, 2.5054592046864896];
    seed_state.append(Goal10);     
    
    Goal11 = Jnt_state_goal();
    Goal11.bin_num = 'K';
    Goal11.jnt_val = [-0.5131183239739836, 1.3175085297094153, 0.9492773380110359, -0.26893080054061724, -2.219532349291596, -1.1195876267269027, 1.87585320864217, -0.3091419330856801];
    seed_state.append(Goal11);
    
    Goal12 = Jnt_state_goal();
    Goal12.bin_num = 'L';
    Goal12.jnt_val = [-0.8107283536120312, 1.1885068501680074, 1.4407099293599144, -0.5506253027562428, -1.777021554647835, -1.1800589893491018, 1.8965499173550684, -0.505319212428926];
    seed_state.append(Goal12);
        
    return seed_state;
    
def generate_key_joint_state(group_name):
	key_joint_state_set = [];

	home_pos = Jnt_state_goal();
	home_pos.bin_num = "Start";
	home_pos.pos_property = "home_pos";
	drop_pos = Jnt_state_goal();
	drop_pos.bin_num = "End";
	drop_pos.pos_property = "drop_pos";
	
	if group_name == "arm_left_torso":
		home_pos.jnt_val = arm_left_home;
		drop_pos.jnt_val = arm_left_drop;
	elif group_name == "arm_right_torso":
		home_pos.jnt_val = arm_right_home;
		drop_pos.jnt_val = arm_right_home;
		
	key_joint_state_set.append(home_pos);
	key_joint_state_set.append(drop_pos);
	
	return key_joint_state_set;

