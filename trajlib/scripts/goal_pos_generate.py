import copy;

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

def generate_goal_points(Bin_base_x, Bin_base_y, Bin_base_z):

    # Gripper dimension
    GripperLength = 0.2;
    # Bin dimension Unit m
    Bin_depth = 0.4;
    Start_Gap = 0.100;

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

    Level4 = WorkBase_Height + BottomLay_Height + SecndLayer_Height + ThirdLayer_Height + TopLayer_Height/2;
    Level3 = WorkBase_Height + BottomLay_Height + SecndLayer_Height + ThirdLayer_Height/2;
    Level2 = WorkBase_Height + BottomLay_Height + SecndLayer_Height/2;
    Level1 = WorkBase_Height + BottomLay_Height/2;

    Entry_X_shiftvalue = Bin_base_x - Bin_depth - GripperLength;

    goal_pos = [];
    # Setting Configuration:
    #	A		B		C
    #	D		E		F
    #	G		H		I
    #	J		K		L
    #		   Base

    binA = testpnt();
    binA.bin_num = chr(ord("A"));
    binA.x = Entry_X_shiftvalue;
    binA.y = Bin_base_y + Left_horizontal_ShiftValue;
    binA.z = Bin_base_z + Level4;
    binA.qx = 0.5;
    binA.qy = -0.5;
    binA.qz = -0.5;
    binA.qw = 0.5;
    goal_pos.append(binA);

    binB = testpnt();
    binB.bin_num = chr(ord("A")+1);
    binB.x = Entry_X_shiftvalue;
    binB.y = Bin_base_y;
    binB.z = Bin_base_z + Level4;
    binB.qx = 0.5;
    binB.qy = -0.5;
    binB.qz = -0.5;
    binB.qw = 0.5;
    goal_pos.append(binB);

    binC = testpnt();
    binC.bin_num = chr(ord("A")+2);
    binC.x = Entry_X_shiftvalue;
    binC.y = Bin_base_y - Right_horizontal_ShiftValue;
    binC.z = Bin_base_z + Level4;
    binC.qx = 0.5;
    binC.qy = -0.5;
    binC.qz = -0.5;
    binC.qw = 0.5;
    #goal_pos.append(binC);

    binD = testpnt();
    binD.bin_num = chr(ord("A")+3);
    binD.x = Entry_X_shiftvalue;
    binD.y = Bin_base_y + Left_horizontal_ShiftValue;
    binD.z = Bin_base_z + Level3;
    binD.qx = 0.5;
    binD.qy = -0.5;
    binD.qz = -0.5;
    binD.qw = 0.5;
    goal_pos.append(binD);

    binE = testpnt();
    binE.bin_num = chr(ord("A")+4);
    binE.x = Entry_X_shiftvalue;
    binE.y = Bin_base_y;
    binE.z = Bin_base_z + Level3;
    binE.qx = 0.5;
    binE.qy = -0.5;
    binE.qz = -0.5;
    binE.qw = 0.5;
    goal_pos.append(binE);

    binF = testpnt();
    binF.bin_num = chr(ord("A")+5);
    binF.x = Entry_X_shiftvalue;
    binF.y = Bin_base_y - Right_horizontal_ShiftValue;
    binF.z = Bin_base_z + Level3;
    binF.qx = 0.5;
    binF.qy = -0.5;
    binF.qz = -0.5;
    binF.qw = 0.5;
    #goal_pos.append(binF);

    binG = testpnt();
    binG.bin_num = chr(ord("A")+6);
    binG.x = Entry_X_shiftvalue;
    binG.y = Bin_base_y + Left_horizontal_ShiftValue;
    binG.z = Bin_base_z + Level2;
    binG.qx = 0.5;
    binG.qy = -0.5;
    binG.qz = -0.5;
    binG.qw = 0.5;
    goal_pos.append(binG);

    # 0.63906; 0.0087358; 1.2154
    binH = testpnt();
    binH.bin_num = chr(ord("A")+7);
    binH.x = Entry_X_shiftvalue;
    binH.y = Bin_base_y;
    binH.z = Bin_base_z + Level2;
    binH.qx = 0.5;
    binH.qy = -0.5;
    binH.qz = -0.5;
    binH.qw = 0.5;
    goal_pos.append(binH);

    #print "Bin H, Pos_X: ",binH.x," Pos_Y:", binH.y, "Pos_Z:", binH.z;
    binI = testpnt();
    binI.bin_num = chr(ord("A")+8);
    binI.x = Entry_X_shiftvalue;
    binI.y = Bin_base_y - Right_horizontal_ShiftValue;
    binI.z = Bin_base_z + Level2;
    binI.qx = 0.5;
    binI.qy = -0.5;
    binI.qz = -0.5;
    binI.qw = 0.5;
    #goal_pos.append(binI);

    binJ = testpnt();
    binJ.bin_num = chr(ord("A")+9);
    binJ.x = Entry_X_shiftvalue;
    binJ.y = Bin_base_y + Left_horizontal_ShiftValue;
    binJ.z = Bin_base_z + Level1;
    binJ.qx = 0.5;
    binJ.qy = -0.5;
    binJ.qz = -0.5;
    binJ.qw = 0.5;
    goal_pos.append(binJ);

    binK = testpnt();
    binK.bin_num = chr(ord("A")+10);
    binK.x = Entry_X_shiftvalue;
    binK.y = Bin_base_y;
    binK.z = Bin_base_z + Level1;
    binK.qx = 0.5;
    binK.qy = -0.5;
    binK.qz = -0.5;
    binK.qw = 0.5;
    goal_pos.append(binK);

    binL = testpnt();
    binL.bin_num = chr(ord("A")+11);
    binL.x = Entry_X_shiftvalue;
    binL.y = Bin_base_y - Right_horizontal_ShiftValue;
    binL.z = Bin_base_z + Level1;
    binL.qx = 0.5;
    binL.qy = -0.5;
    binL.qz = -0.5;
    binL.qw = 0.5;
    #goal_pos.append(binL);

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
    #Goal1.jnt_val = [ 1.2946151012717504, -1.363382168130896, -1.6617924058050058, -1.695796534346829, -0.779241703517125, -0.23640897614768297, 1.8915492395648426];
    Goal1.jnt_val = [-1.5442344278755054, 1.0688537862129166, 0.9596692482080396, -1.40904847193336, 1.7674462114208445, 0.6331007451974301, -1.031333210948469];
    seed_state.append(Goal1);

    Goal2 = Jnt_state_goal();
    Goal2.bin_num = chr(ord('A')+1);
    Goal2.jnt_val = [2.0470397118623382, -1.5900951732258286, 2.8176173676670175, -0.9284428878232793, 1.1380462986024937, 1.7034889656174173, -0.9567357683589586];
    seed_state.append(Goal2);

    # torso_angle = [-0.7600786552701615, -0.7600786552701615]
    Goal3 = Jnt_state_goal();
    Goal3.bin_num = chr(ord('A')+2);
    Goal3.jnt_val = [1.636146928216539, -1.7942154910938195, -1.9579820089934483, -1.065370555588695, -1.1977333227354903, -1.2629020548563172, 2.4920217797426374];
    #seed_state.append(Goal3);

    Goal4 = Jnt_state_goal();
    Goal4.bin_num = chr(ord('A')+3);
    Goal4.jnt_val = [2.4580967147605235, -0.5446729671123386, 2.665470740770432, -2.1188655843905315, 1.2066158758731622, 1.6791622731613003, -0.841630540918266];
    seed_state.append(Goal4);

    Goal5 = Jnt_state_goal();
    Goal5.bin_num = chr(ord('A')+4);
    Goal5.jnt_val = [2.348465957508067, -1.40668371027442, 2.8710441151821295, -1.7654378467884415, 1.0416194682374236, 1.858653399311216, -0.3811025334024448];
    seed_state.append(Goal5);

    # torso_angle = [-0.7600786552701615, -0.7600786552701615]
    Goal6 = Jnt_state_goal();
    Goal6.bin_num = chr(ord('A')+5);
    Goal6.jnt_val = [0.3211845040291911, 1.8722236118989442, 2.04186945586128, 1.7676554743270105, 2.0355538838504685, 1.8938641153978812, -1.5620907149387162];
    #seed_state.append(Goal6);

    Goal7 = Jnt_state_goal();
    Goal7.bin_num = chr(ord('A')+6);
    Goal7.jnt_val = [-2.0370711246306104, -1.8295192457063296, -1.8451202902468806, 2.321145735151722, -3.0077142989456918, 1.2251922979462764, -2.2494549963774353];
    seed_state.append(Goal7);

    Goal8 = Jnt_state_goal();
    Goal8.bin_num = chr(ord('A')+7);
    Goal8.jnt_val = [-0.9916030724162687, 1.8642688181352713, -2.338997191523704, 2.0092924404247343, -2.3411044399524106, 1.3314934519202497, 0.7778852700524742];
    seed_state.append(Goal8);

    # torso_angle = [-0.7600786552701615, -0.7600786552701615]
    Goal9 = Jnt_state_goal();
    Goal9.bin_num = chr(ord('A')+8);
    Goal9.jnt_val = [-0.8463435946810004, 1.7179036202320017, -2.1239910978200562, 1.865937710123034, 1.2017196314404206, -1.5632909661353163, -2.4543389965598013];
    #seed_state.append(Goal9);

    Goal10 = Jnt_state_goal();
    Goal10.bin_num = chr(ord('A')+9);
    Goal10.jnt_val = [-1.2948852785761256, -1.2403729592369999, -1.3933085254603284, 2.080249468059148, -0.8159017944741344, -0.37292349882999726, 2.351640933343906];
    seed_state.append(Goal10);

    Goal11 = Jnt_state_goal();
    Goal11.bin_num = chr(ord('A')+10);
    Goal11.jnt_val = [2.999197954039974, -1.8837653317964884, -2.198914825912091, -1.8150615006544348, 0.37585648711487446, 1.856511694456621, 0.7893266850603321];
    seed_state.append(Goal11);

    # torso_angle = [-0.7600786552701615, -0.7600786552701615]
    Goal12 = Jnt_state_goal();
    Goal12.bin_num = chr(ord('A')+11);
    Goal12.jnt_val = [-0.1366744823232535, 1.8814087405923567, -1.918213729532003, 1.556195610950019, 1.0630905526132743, -1.714707790834795, -2.1028431148646893];
    #seed_state.append(Goal12);

    return seed_state;

def generate_key_joint_state():
	key_joint_state_set = [];
	
	# Here is drop position
	#left_arm_init_joint_value = [2.193856910816974, 1.1630675621113802, 0.852437058039672, 1.113211995331904, 0.8438088567310283, -1.0094747189949542, 0.24438780016629988];
	#right_arm_init_joint_value = [2.5794765930828296, 1.3620727097356629, 1.3831275005664025, 0.7845256389316293, -3.057076564078304, -1.7625990915019676, 1.3096307216010097];

    Init_pos = Jnt_state_goal();
    Init_pos.bin_num = "Start";
    Init_pos.pos_property = "init_pos";
    Init_pos.jnt_val = [2.193856910816974, 1.1630675621113802, 0.852437058039672, 1.113211995331904, 0.8438088567310283, -1.0094747189949542, 0.24438780016629988];
    key_joint_state_set.append(Init_pos);

    drop_pos = Jnt_state_goal();
    drop_pos.bin_num = "End";
    drop_pos.pos_property = "drop_pos";
    #drop_pos.jnt_val = [-1.466582785572278, -1.0355327918495896, 1.5012144903767006, 0.840856815273938, -0.6411362241516871, -1.0226092747529063, -0.35127936003281013];
    drop_pos.jnt_val = [2.193856910816974, 1.1630675621113802, 0.852437058039672, 1.113211995331904, 0.8438088567310283, -1.0094747189949542, 0.24438780016629988];
    key_joint_state_set.append(drop_pos);
	
	return key_joint_state_set;

