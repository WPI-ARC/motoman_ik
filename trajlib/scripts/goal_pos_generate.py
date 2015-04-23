import copy;

class testpnt:
	def __init__(self):
		self.bin_num = 0;
		self.pnt_property = "init_pos";
		self.x = 0;
		self.y = 0;
		self.z = 0;
		self.qx = 0;
		self.qy = 0;
		self.qz = 0;        
		self.qw = 0;
		
def generate_goal_points(Bin_base_x, Bin_base_y, Bin_base_z):
	goal_pos = [];
	bin_values = [];
    
    # Gripper dimension
	GripperLength = 0.25;
	# Bin dimension Unit m
	Bin_depth = 0.430;
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

	Entry_X_shiftvalue = Bin_base_x - Bin_depth - Start_Gap - GripperLength;

	# Setting Configuration:
	
	#	1		2		3
	#	4		5		6
	#	7		8		9
	#	10		11		12
	#		   Base	
	
	start = testpnt();
	start.x = 0.0;
	start.y = 0.0;
	start.z = 0.0;
	end = testpnt();
	end.pnt_property = "drop_pos";
	end.x = 0.2574;
	end.y = 0.6674;
	end.z = 0.5926;
	end.qx = 0.0142;
	end.qy = 0;
	end.qz = 0;
	end.qw = 1.0	
	
	
	
	bin1 = testpnt();
	bin1.x = Entry_X_shiftvalue;
	bin1.y = Bin_base_y + Left_horizontal_ShiftValue;
	bin1.z = Bin_base_z + Level4;
	bin_values.append(bin1);

	bin2 = testpnt();
	bin2.x = Entry_X_shiftvalue;
	bin2.y = Bin_base_y;
	bin2.z = Bin_base_z + Level4;
	bin_values.append(bin2);

	bin3 = testpnt();
	bin3.x = Entry_X_shiftvalue;
	bin3.y = Bin_base_y - Right_horizontal_ShiftValue;
	bin3.z = Bin_base_z + Level4;
	bin_values.append(bin3);

	bin4 = testpnt();
	bin4.x = Entry_X_shiftvalue;
	bin4.y = Bin_base_y + Left_horizontal_ShiftValue;
	bin4.z = Bin_base_z + Level3;
	bin_values.append(bin4);

	bin5 = testpnt();
	bin5.x = Entry_X_shiftvalue;
	bin5.y = Bin_base_y;
	bin5.z = Bin_base_z + Level3;
	bin_values.append(bin5);

	bin6 = testpnt();
	bin6.x = Entry_X_shiftvalue;
	bin6.y = Bin_base_y - Right_horizontal_ShiftValue;
	bin6.z = Bin_base_z + Level3;
	bin_values.append(bin6);

	bin7 = testpnt();
	bin7.x = Entry_X_shiftvalue;
	bin7.y = Bin_base_y + Left_horizontal_ShiftValue;
	bin7.z = Bin_base_z + Level2;
	bin_values.append(bin7);

	bin8 = testpnt();
	bin8.x = Entry_X_shiftvalue;
	bin8.y = Bin_base_y;
	bin8.z = Bin_base_z + Level2;
	bin_values.append(bin8);

	bin9 = testpnt();
	bin9.x = Entry_X_shiftvalue;
	bin9.y = Bin_base_y - Right_horizontal_ShiftValue;
	bin9.z = Bin_base_z + Level2;
	bin_values.append(bin9);

	bin10 = testpnt();
	bin10.x = Entry_X_shiftvalue;
	bin10.y = Bin_base_y + Left_horizontal_ShiftValue;
	bin10.z = Bin_base_z + Level1;
	bin_values.append(bin10);

	bin11 = testpnt();
	bin11.x = Entry_X_shiftvalue;
	bin11.y = Bin_base_y;
	bin11.z = Bin_base_z + Level1;
	bin_values.append(bin11);

	bin12 = testpnt();
	bin12.x = Entry_X_shiftvalue;
	bin12.y = Bin_base_y - Right_horizontal_ShiftValue;
	bin12.z = Bin_base_z + Level1;
	bin_values.append(bin12);
	
	for bin_num in range(1,13):	
		mid = bin_values[bin_num - 1];
		
		pnt2f = copy.deepcopy(mid);
		
		pnt2f.bin_num = bin_num;		
		pnt2f.pnt_property = "test_pos";		
		pnt2f.qx = 0.5;
		pnt2f.qy = -0.5;
		pnt2f.qz = -0.5;
		pnt2f.qw = 0.5;
		
		start.bin_num = bin_num;
		end.bin_num = bin_num;
		goal_pos.append(copy.deepcopy(start));		
		goal_pos.append(pnt2f);
		goal_pos.append(copy.deepcopy(end));
		
		# Here I set start point x,y,z = 0,0,0 to be a marker, so robot will use init position assigned in IK_solver as goal position

 
	return goal_pos;
	
class Jnt_goal:
	def __init__(self):
		self.bin_num = 0;
		self.traj_property = "test_pos";
		self.jnt_val = [];
	
def joint_value_goal_setting():
	
	goal_joint = [];
	
	Init_pos = Jnt_goal();
	Init_pos.bin_num = 0;
	Init_pos.traj_property = "init_pos";
	Init_pos.jnt_val = [-0.3772088970063918, -1.3143374665077188, 1.3055896989571494, -1.2431863366129394, -2.9914519460193256, 1.3527307468837322, 1.6504463451922884];

	drop_pos = Jnt_goal();
	drop_pos.bin_num = 0;
	drop_pos.traj_property = "drop_pos";
	
	#drop_pos.jnt_val = [-1.466582785572278, -1.0355327918495896, 1.5012144903767006, 0.840856815273938, -0.6411362241516871, -1.0226092747529063, -0.35127936003281013];
	drop_pos.jnt_val = [2.193856910816974, 1.1630675621113802, 0.852437058039672, 1.113211995331904, 0.8438088567310283, -1.0094747189949542, 0.24438780016629988];
	
	Goal1 = Jnt_goal();
	Goal1.bin_num = 1;
	#Goal1.jnt_val = [ 1.2946151012717504, -1.363382168130896, -1.6617924058050058, -1.695796534346829, -0.779241703517125, -0.23640897614768297, 1.8915492395648426];
	Goal1.jnt_val = [ -1.5442344278755054, 1.0688537862129166, 0.9596692482080396, -1.40904847193336, 1.7674462114208445, 0.6331007451974301, -1.031333210948469];
	
	goal_joint.append(Goal1);
	drop_pos1 = copy.deepcopy(drop_pos);
	drop_pos1.bin_num = 1;
	goal_joint.append(drop_pos1);
	
	Goal2 = Jnt_goal();
	Goal2.bin_num = 2;
	#Goal2.jnt_val = [1.8421781632099408, -1.4149261761742273, -2.854863976296227, -1.482880179281118, -1.8001793596638498, -1.251628050878441, 2.700367173210477];

	Goal2.jnt_val = [2.0470397118623382, -1.5900951732258286, 2.8176173676670175, -0.9284428878232793, 1.1380462986024937, 1.7034889656174173, -0.9567357683589586];
	#Goal2.jnt_val = [2.1447293646677834, -1.4817658196617631, 2.632472958692524, -1.3097692146019357, 0.9939809258268164, 1.8789563826186628, -0.8178301313617673];
	goal_joint.append(Goal2);	
	drop_pos2 = copy.deepcopy(drop_pos);
	drop_pos2.bin_num = 2;
	goal_joint.append(drop_pos2);
	
	# torso_angle = [-0.7600786552701615, -0.7600786552701615]
	Goal3 = Jnt_goal();
	Goal3.bin_num = 3;
	Goal3.jnt_val = [1.636146928216539, -1.7942154910938195, -1.9579820089934483, -1.065370555588695, -1.1977333227354903, -1.2629020548563172, 2.4920217797426374];
	goal_joint.append(Goal3);	
	drop_pos3 = copy.deepcopy(drop_pos);
	drop_pos3.bin_num = 3;
	goal_joint.append(drop_pos3);
	
	Goal4 = Jnt_goal();
	Goal4.bin_num = 4;
	Goal4.jnt_val = [2.4580967147605235, -0.5446729671123386, 2.665470740770432, -2.1188655843905315, 1.2066158758731622, 1.6791622731613003, -0.841630540918266];
	goal_joint.append(Goal4);	
	drop_pos4 = copy.deepcopy(drop_pos);
	drop_pos4.bin_num = 4;
	goal_joint.append(drop_pos4);
	
	Goal5 = Jnt_goal();
	Goal5.bin_num = 5;
	Goal5.jnt_val = [2.348465957508067, -1.40668371027442, 2.8710441151821295, -1.7654378467884415, 1.0416194682374236, 1.858653399311216, -0.3811025334024448];
	goal_joint.append(Goal5);	
	drop_pos5 = copy.deepcopy(drop_pos);
	drop_pos5.bin_num = 5;
	goal_joint.append(drop_pos5);
	
	# torso_angle = [-0.7600786552701615, -0.7600786552701615]
	Goal6 = Jnt_goal();
	Goal6.bin_num = 6;
	Goal6.jnt_val = [0.3211845040291911, 1.8722236118989442, 2.04186945586128, 1.7676554743270105, 2.0355538838504685, 1.8938641153978812, -1.5620907149387162];
	goal_joint.append(Goal6);	
	drop_pos6 = copy.deepcopy(drop_pos);
	drop_pos6.bin_num = 6;
	goal_joint.append(drop_pos6);
	
	Goal7 = Jnt_goal();
	Goal7.bin_num = 7;
	Goal7.jnt_val = [-2.0370711246306104, -1.8295192457063296, -1.8451202902468806, 2.321145735151722, -3.0077142989456918, 1.2251922979462764, -2.2494549963774353];
	goal_joint.append(Goal7);	
	drop_pos7 = copy.deepcopy(drop_pos);
	drop_pos7.bin_num = 7;
	goal_joint.append(drop_pos7);
	
	Goal8 = Jnt_goal();
	Goal8.bin_num = 8;
	Goal8.jnt_val = [-0.9916030724162687, 1.8642688181352713, -2.338997191523704, 2.0092924404247343, -2.3411044399524106, 1.3314934519202497, 0.7778852700524742];
	goal_joint.append(Goal8);	
	drop_pos8 = copy.deepcopy(drop_pos);
	drop_pos8.bin_num = 8;
	goal_joint.append(drop_pos8);
	
	# torso_angle = [-0.7600786552701615, -0.7600786552701615]
	Goal9 = Jnt_goal();
	Goal9.bin_num = 9;
	Goal9.jnt_val = [-0.8463435946810004, 1.7179036202320017, -2.1239910978200562, 1.865937710123034, 1.2017196314404206, -1.5632909661353163, -2.4543389965598013];
	goal_joint.append(Goal9);	
	drop_pos9 = copy.deepcopy(drop_pos);
	drop_pos9.bin_num = 9;
	goal_joint.append(drop_pos9);
	
	Goal10 = Jnt_goal();
	Goal10.bin_num = 10;
	Goal10.jnt_val = [-1.2948852785761256, -1.2403729592369999, -1.3933085254603284, 2.080249468059148, -0.8159017944741344, -0.37292349882999726, 2.351640933343906];
	goal_joint.append(Goal10);	
	drop_pos10 = copy.deepcopy(drop_pos);
	drop_pos10.bin_num = 10;
	goal_joint.append(drop_pos10);
	
	Goal11 = Jnt_goal();
	Goal11.bin_num = 11;
	Goal11.jnt_val = [2.999197954039974, -1.8837653317964884, -2.198914825912091, -1.8150615006544348, 0.37585648711487446, 1.856511694456621, 0.7893266850603321];
	goal_joint.append(Goal11);	
	drop_pos11 = copy.deepcopy(drop_pos);
	drop_pos11.bin_num = 11;
	goal_joint.append(drop_pos11);
	
	# torso_angle = [-0.7600786552701615, -0.7600786552701615]
	Goal12 = Jnt_goal();
	Goal12.bin_num = 12;
	Goal12.jnt_val = [-0.1366744823232535, 1.8814087405923567, -1.918213729532003, 1.556195610950019, 1.0630905526132743, -1.714707790834795, -2.1028431148646893];	
	goal_joint.append(Goal12);	
	drop_pos12 = copy.deepcopy(drop_pos);
	drop_pos12.bin_num = 12;
	goal_joint.append(drop_pos12);

    
	#goal_joint.append(drop_pos);
	#goal_joint.append(Init_pos);
	
	return goal_joint;

	
	
