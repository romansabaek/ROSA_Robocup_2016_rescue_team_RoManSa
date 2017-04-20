int _getch();
int MOT_InitUART();
void MOT_InitDXLTorque();
void MOT_Write(int id, int Portnum, int Data);
int MOT_Read(int id, int Portnum);
void MOT_Syncwrite(int *id, int NUM_ACTUATOR, int* GoalPos);
void Start_End_Motion();
void MOT_InitDXLMode();
