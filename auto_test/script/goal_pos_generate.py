import copy;

class testpnt:
	def __init__(self):
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
	
	for bin_num in range(1,12):	
		mid = bin_values[bin_num - 1];
		pnt2f = copy.deepcopy(mid);
		pnt2f.pnt_property = "test_pos";
		pnt2f.qx = 0.5;
		pnt2f.qy = -0.5;
		pnt2f.qz = -0.5;
		pnt2f.qw = 0.5;
		goal_pos.append(pnt2f);
		goal_pos.append(end);		
		# Here I set start point x,y,z = 0,0,0 to be a marker, so robot will use init position assigned in IK_solver as goal position
		goal_pos.append(start);
 
	return goal_pos;

	
	
