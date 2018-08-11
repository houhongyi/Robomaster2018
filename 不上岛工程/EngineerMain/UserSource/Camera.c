#include "Camera.h"

Mid_Pose mid_pose={0};
void ISO14443AAppendCRCA(void* Buffer, unsigned short ByteCount) {
	unsigned short Checksum = 0x6363;
	unsigned char* DataPtr = (unsigned char*)Buffer;

	while (ByteCount--) {
		unsigned char Byte = *DataPtr++;

		Byte ^= (unsigned char)(Checksum & 0x00FF);
		Byte ^= Byte << 4;

		Checksum = (Checksum >> 8) ^ ((unsigned short)Byte << 8) ^
			((unsigned short)Byte << 3) ^ ((unsigned short)Byte >> 4);
	}

	*DataPtr++ = (Checksum >> 0) & 0x00FF;
	*DataPtr = (Checksum >> 8) & 0x00FF;
}

unsigned char ISO14443ACheckCRCA(void* Buffer, unsigned short ByteCount)
{
	unsigned short Checksum = 0x6363;
	unsigned char* DataPtr = (unsigned char*)Buffer;

	while (ByteCount--) {
		unsigned char Byte = *DataPtr++;

		Byte ^= (unsigned char)(Checksum & 0x00FF);
		Byte ^= Byte << 4;

		Checksum = (Checksum >> 8) ^ ((unsigned short)Byte << 8) ^
			((unsigned short)Byte << 3) ^ ((unsigned short)Byte >> 4);
	}

	return (DataPtr[0] == ((Checksum >> 0) & 0xFF)) && (DataPtr[1] == ((Checksum >> 8) & 0xFF));
}


void Camera_UART_Send_Buff(unsigned char*s,Command command, unsigned long data_input)
{
	s[0] = 0x55;
	s[1] = command;
	*(unsigned long*)&s[2] = data_input;
	ISO14443AAppendCRCA(s, LS_DATA_LEN);
}

//UART_Send_Buff(CALL_Command,0);

char Camera_UART_Receive_Buff(unsigned char *arrRC_Buf)
{
	unsigned long data_input;
	if (arrRC_Buf[0] == 0x55)
	{
		if (ISO14443ACheckCRCA(arrRC_Buf, LS_DATA_LEN))
		{
			switch (arrRC_Buf[1])
			{
			case OK_Command:
				break;
			case CALL_Command:
				break;
			case POSE_Command:
				data_input = *(unsigned long*)&arrRC_Buf[2];
				mid_pose.X = data_input >> 16;
				mid_pose.Y = data_input;
				//cout << mid_pose.Y << "rev_ok" << mid_pose.X << endl;
				break;
			default:
				return 0;
			}
		}
		else
		{
			return 0;
		}
	}
	else
	{
		return 0;
	}
	return 1;
}

