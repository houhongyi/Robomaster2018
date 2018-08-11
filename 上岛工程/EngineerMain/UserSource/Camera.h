#ifndef __Camera_H
#define __Camera_H

#define LS_DATA_LEN    6
typedef struct
{
	unsigned short X;
	unsigned short Y;
}Mid_Pose;

typedef enum
{
	POSE_Command = 0x00,

	CALL_Command = 0x19,
	OK_Command = 0x20,
}Command;

extern Mid_Pose mid_pose;

void Camera_UART_Send_Buff(unsigned char*s,Command command, unsigned long data_input);
char Camera_UART_Receive_Buff(unsigned char *arrRC_Buf);

#endif

