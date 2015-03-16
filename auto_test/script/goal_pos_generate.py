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
		
	Goal1 = Jnt_goal();
	Goal1.bin_num = 1;
	Goal1.jnt_val = [ 1.2946151012717504, -1.363382168130896, -1.6617924058050058, -1.695796534346829, -0.779241703517125, -0.23640897614768297, 1.8915492395648426];
	goal_joint.append(Goal1);
	Init1 = copy.deepcopy(Init_pos);
	Init1.bin_num = 1;
	goal_joint.append(Init1);
	
	Goal2 = Jnt_goal();
	Goal2.bin_num = 2;
	Goal2.jnt_val = [1.8421781632099408, -1.4149261761742273, -2.854863976296227, -1.482880179281118, -1.8001793596638498, -1.251628050878441, 2.700367173210477];
	goal_joint.append(Goal2);	
	Init2 = copy.deepcopy(Init_pos);
	Init2.bin_num = 2;
	goal_joint.append(Init2);
	
	Goal3 = Jnt_goal();
	Goal3.bin_num = 3;
	Goal3.jnt_val = [];
	goal_joint.append(Goal3);	
	Init3 = copy.deepcopy(Init_pos);
	Init3.bin_num = 3;
	goal_joint.append(Init3);
	
	Goal4 = Jnt_goal();
	Goal4.bin_num = 4;
	Goal4.jnt_val = [1.9292449800537304, -0.430457441805311, -2.5403530143741544, -2.2929449706914227, -2.2637663317971226, -1.0855995393304494, 2.6715887318059974];
	goal_joint.append(Goal4);	
	Init4 = copy.deepcopy(Init_pos);
	Init4.bin_num = 4;
	goal_joint.append(Init4);
	
	Goal5 = Jnt_goal();
	Goal5.bin_num = 5;
	Goal5.jnt_val = [-1.6819154448186682, 1.8999636216979705, 0.9869621276251267, -1.9394283807079662, 1.3460971434355216, 0.5229151715316868, 0.4533038334910315];
	goal_joint.append(Goal5);	
	Init5 = copy.deepcopy(Init_pos);
	Init5.bin_num = 5;
	goal_joint.append(Init5);
	
	Goal6 = Jnt_goal();
	Goal6.bin_num = 6;
	Goal6.jnt_val = [];
	goal_joint.append(Goal6);	
	Init6 = copy.deepcopy(Init_pos);
	Init6.bin_num = 6;
	goal_joint.append(Init6);
	
	Goal7 = Jnt_goal();
	Goal7.bin_num = 7;
	Goal7.jnt_val = [-0.43134787090232624, 0.3259961022873073, 0.5852777288739619, -2.354710961748468, -0.02251247221778719, 1.1160518898409235, 0.007577890682513895];
	goal_joint.append(Goal7);	
	Init7 = copy.deepcopy(Init_pos);
	Init7.bin_num = 7;
	goal_joint.append(Init7);
	
	Goal8 = Jnt_goal();
	Goal8.bin_num = 8;
	Goal8.jnt_val = [ 0.6778446642423175, 1.3395984930498206, -0.3238526288710371, -2.0046636595537364, -0.5712584774538879, 1.5680131569666471, -0.625209196358681];
	goal_joint.append(Goal8);	
	Init8 = copy.deepcopy(Init_pos);
	Init8.bin_num = 8;
	goal_joint.append(Init8);
	
	Goal9 = Jnt_goal();
	Goal9.bin_num = 9;
	Goal9.jnt_val = [];
	goal_joint.append(Goal9);	
	Init9 = copy.deepcopy(Init_pos);
	Init9.bin_num = 9;
	goal_joint.append(Init9);
	
	Goal10 = Jnt_goal();
	Goal10.bin_num = 10;
	Goal10.jnt_val = [3.0591996672764634, -1.3982062338643588, -1.561834956140967, -1.915704640390788, 2.9937701175306124, -1.7399412093172637, -1.770419828879024];
	goal_joint.append(Goal10);	
	Init10 = copy.deepcopy(Init_pos);
	Init10.bin_num = 10;
	goal_joint.append(Init10);
	
	Goal11 = Jnt_goal();
	Goal11.bin_num = 11;
	Goal11.jnt_val = [-0.012623040035776644, 1.8589150445443539, 1.0871880310036417, -1.6449842057138226, -2.814004566548758, -1.6925707741162284, -2.160274609811549];
	goal_joint.append(Goal11);	
	Init11 = copy.deepcopy(Init_pos);
	Init11.bin_num = 11;
	goal_joint.append(Init11);
	
	Goal12 = Jnt_goal();
	Goal12.bin_num = 12;
	Goal12.jnt_val = [];	
	goal_joint.append(Goal12);	
	Init12 = copy.deepcopy(Init_pos);
	Init12.bin_num = 12;
	goal_joint.append(Init12);
	
	return goal_joint;

	
	
