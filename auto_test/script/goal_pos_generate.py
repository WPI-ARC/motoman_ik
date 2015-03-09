import copy;

class testpnt:
	def __init__(self):
		self.x = 0;
		self.y = 0;
		self.z = 0;
		""" Function doc """
		
def generate_goal_points(Bin_base_x, Bin_base_y, Bin_base_z, Test_Depth = 0.3):

	goal_pos = [];    
    # Gripper dimension
	GripperLength = 0.2;
	# Bin dimension Unit m
	Bin_depth = 0.430;
	Start_Gap = 0.100; 
	
	LeftBin_width = 0.240;
	MiddleBin_width = 0.300;
	RightBin_width = 0.240;
	
	WorkBase_Height = 0.820;
	BottomLay_Height = 0.230;
	SecndLayer_Height = 0.230;
	ThirdLayer_Height = 0.220;
	TopLayer_Height = 0.260;
		
	Left_horizontal_ShiftValue = MiddleBin_width/2 + LeftBin_width/2;
	Right_horizontal_ShiftValue = MiddleBin_width/2 + RightBin_width/2;
	
	TopLayer_vertical_shiftvalue = WorkBase_Height + BottomLay_Height + SecndLayer_Height + ThirdLayer_Height + TopLayer_Height/2;
	ThirdLayer_vertical_shiftvalue = WorkBase_Height + BottomLay_Height + SecndLayer_Height + ThirdLayer_Height/2;
	SecndLayer_vertical_shiftvalue = WorkBase_Height + BottomLay_Height + SecndLayer_Height/2;
	BottomLayer_vertical_shiftvalue = WorkBase_Height + BottomLay_Height/2;

	Entry_X_shiftvalue = Bin_base_x - Bin_depth - Start_Gap - GripperLength;

	# Setting Configuration:
	
	#	1		2		3
	#	4		5		6
	#	7		8		9
	#	10		11		12
	#		   Base	
	
	# Up Layer
	# Bin 1 
	# Start Pos
	pnt1 = testpnt();
	pnt1.x = Entry_X_shiftvalue;
	pnt1.y = Bin_base_y + Left_horizontal_ShiftValue;
	pnt1.z = Bin_base_z + TopLayer_vertical_shiftvalue;
	goal_pos.append(pnt1);
	# Extended
	pnt = copy.deepcopy(pnt1);
	pnt.x = Entry_X_shiftvalue + Test_Depth;
	goal_pos.append(pnt);
	# Exit
	pnt = copy.deepcopy(pnt1);
	goal_pos.append(pnt);
	
	# Bin 2 
	# Start Pos
	pnt2 = testpnt();
	pnt2.x = Entry_X_shiftvalue;
	pnt2.y = Bin_base_y;
	pnt2.z = Bin_base_z + TopLayer_vertical_shiftvalue;
	goal_pos.append(pnt2);
	# Extended
	pnt = copy.deepcopy(pnt2);
	pnt.x = Entry_X_shiftvalue + Test_Depth;
	goal_pos.append(pnt);
	# Exit
	pnt = copy.deepcopy(pnt2);
	goal_pos.append(pnt);
	
	# Bin 3 
	# Start Pos
	pnt3 = testpnt();
	pnt3.x = Entry_X_shiftvalue;
	pnt3.y = Bin_base_y - Right_horizontal_ShiftValue;
	pnt3.z = Bin_base_z + TopLayer_vertical_shiftvalue;
	goal_pos.append(pnt3);
	# Extended
	pnt = copy.deepcopy(pnt3);
	pnt.x = Entry_X_shiftvalue + Test_Depth;
	goal_pos.append(pnt);
	# Exit
	pnt = copy.deepcopy(pnt3);
	goal_pos.append(pnt);
	
	# *************  Third Layer **********
	# Bin 4 
	# Start Pos
	pnt4 = testpnt();
	pnt4.x = Entry_X_shiftvalue;
	pnt4.y = Bin_base_y + Left_horizontal_ShiftValue;
	pnt4.z = Bin_base_z + ThirdLayer_vertical_shiftvalue;
	goal_pos.append(pnt4);
	# Extended
	pnt = copy.deepcopy(pnt4);
	pnt.x = Entry_X_shiftvalue + Test_Depth;
	goal_pos.append(pnt);
	# Exit
	pnt = copy.deepcopy(pnt4);
	goal_pos.append(pnt);
	
	# Bin 5 
	# Start Pos
	pnt5 = testpnt();
	pnt5.x = Entry_X_shiftvalue;
	pnt5.y = Bin_base_y;
	pnt5.z = Bin_base_z + ThirdLayer_vertical_shiftvalue;
	goal_pos.append(pnt5);
	# Extended
	pnt = copy.deepcopy(pnt5);
	pnt.x = Entry_X_shiftvalue + Test_Depth;
	goal_pos.append(pnt);
	# Exit
	pnt = copy.deepcopy(pnt5);
	goal_pos.append(pnt);
	
	# Bin 6 
	# Start Pos
	pnt6 = testpnt();
	pnt6.x = Entry_X_shiftvalue;
	pnt6.y = Bin_base_y - Right_horizontal_ShiftValue;
	pnt6.z = Bin_base_z + ThirdLayer_vertical_shiftvalue;
	goal_pos.append(pnt6);
	# Extended
	pnt = copy.deepcopy(pnt6);
	pnt.x = Entry_X_shiftvalue + Test_Depth;
	goal_pos.append(pnt);
	# Exit
	pnt = copy.deepcopy(pnt6);
	goal_pos.append(pnt);
	
	# *************  Second Layer **********
	# Bin 7 
	# Start Pos
	pnt7 = testpnt();
	pnt7.x = Entry_X_shiftvalue;
	pnt7.y = Bin_base_y + Left_horizontal_ShiftValue;
	pnt7.z = Bin_base_z + SecndLayer_vertical_shiftvalue;
	goal_pos.append(pnt7);
	# Extended
	pnt = copy.deepcopy(pnt7);
	pnt.x = Entry_X_shiftvalue + Test_Depth;
	goal_pos.append(pnt);
	# Exit
	pnt = copy.deepcopy(pnt7);
	goal_pos.append(pnt);
	
	# Bin 8 
	# Start Pos
	pnt8 = testpnt();
	pnt8.x = Entry_X_shiftvalue;
	pnt8.y = Bin_base_y;
	pnt8.z = Bin_base_z + SecndLayer_vertical_shiftvalue;
	goal_pos.append(pnt8);
	# Extended
	pnt = copy.deepcopy(pnt8);
	pnt.x = Entry_X_shiftvalue + Test_Depth;
	goal_pos.append(pnt);
	# Exit
	pnt = copy.deepcopy(pnt8);
	goal_pos.append(pnt);
	
	# Bin 9 
	# Start Pos
	pnt9 = testpnt();
	pnt9.x = Entry_X_shiftvalue;
	pnt9.y = Bin_base_y - Right_horizontal_ShiftValue;
	pnt9.z = Bin_base_z + SecndLayer_vertical_shiftvalue;
	goal_pos.append(pnt9);
	# Extended
	pnt = copy.deepcopy(pnt9);
	pnt.x = Entry_X_shiftvalue + Test_Depth;
	goal_pos.append(pnt);
	# Exit
	pnt = copy.deepcopy(pnt9);
	goal_pos.append(pnt);
	
	# *************  Bottom Layer **********
	# Bin 10 
	# Start Pos
	pnt10 = testpnt();
	pnt10.x = Entry_X_shiftvalue;
	pnt10.y = Bin_base_y + Left_horizontal_ShiftValue;
	pnt10.z = Bin_base_z + BottomLayer_vertical_shiftvalue;
	goal_pos.append(pnt10);
	# Extended
	pnt = copy.deepcopy(pnt10);
	pnt.x = Entry_X_shiftvalue + Test_Depth;
	goal_pos.append(pnt);
	# Exit
	pnt = copy.deepcopy(pnt10);
	goal_pos.append(pnt);
	
	# Bin 11 
	# Start Pos
	pnt11 = testpnt();
	pnt11.x = Entry_X_shiftvalue;
	pnt11.y = Bin_base_y;
	pnt11.z = Bin_base_z + BottomLayer_vertical_shiftvalue;
	goal_pos.append(pnt11);
	# Extended
	pnt = copy.deepcopy(pnt11);
	pnt.x = Entry_X_shiftvalue + Test_Depth;
	goal_pos.append(pnt);
	# Exit
	pnt = copy.deepcopy(pnt11);
	goal_pos.append(pnt);
	
	# Bin 12 
	# Start Pos
	pnt12 = testpnt();
	pnt12.x = Entry_X_shiftvalue;
	pnt12.y = Bin_base_y - Right_horizontal_ShiftValue;
	pnt12.z = Bin_base_z + BottomLayer_vertical_shiftvalue;
	goal_pos.append(pnt12);
	# Extended
	pnt = copy.deepcopy(pnt12);
	pnt.x = Entry_X_shiftvalue + Test_Depth;
	goal_pos.append(pnt);
	# Exit
	pnt = copy.deepcopy(pnt12);
	goal_pos.append(pnt);
	
	return goal_pos;

	
	
