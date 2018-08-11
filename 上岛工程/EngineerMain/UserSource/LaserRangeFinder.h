#ifndef __LaserRF_H__
#define __LaserRF_H__


#define str_LRF_SetRange_5M "\xFA\x04\x09\x05\xF4" //��������5m
#define str_LRF_SetRange_10M "\xFA\x04\x09\x0A\xEF" //��������10m
#define str_LRF_SetRange_30M "\xFA\x04\x09\x1E\xDB" //��������30m

#define str_LRF_SetRevolution_1mm "\xFA\x04\x0C\x01\xF5" //���÷ֱ��� 1mm
#define str_LRF_SetRevolution_01mm "\xFA\x04\x0C\x02\xF4" //���÷ֱ���0.1mm

#define str_LRF_SetFrequence_5HZ "\xFA\x04\x0A\x05\xF3"  //����Ƶ��
#define str_LRF_SetFrequence_10HZ "\xFA\x04\x0A\x0A\xEE" 
#define str_LRF_SetFrequence_20HZ "\xFA\x04\x0A\x14\xE4"

#define str_LRF_SetPoweONStart2Mesure "\xFA\x04\x0D\x01\xF4" //�趨�ϵ�Ͳ�
#define str_LRF_SetPoweONStart2Mesure_NOT "\xFA\x04\x0D\x00\xF5" //�趨�ϵ�Ͳ�

#define str_LRF_StarMesure_Once "\x80\x06\x02\x78" //���β���
#define str_LRF_StarMesure "\x80\x06\x03\x77" //��������
#define str_LRF_PowerOFF "\x80\x04\x02\x7A" //�ػ�����


typedef enum
{
	LRF_DateCheck_Fail=0,
	LRF_DateCheck_OK,
}Date_CheckDef;


typedef enum
{
	LRF_Inite=0,//��ʼֵ
	LRF_Ready,//����״̬
	LRF_BUSY, //æ
	
	LRF_SetRange_OK,//�������̳ɹ�
	LRF_SetRange_Fail,
	
	LRF_SetFrequency_OK,//����Ƶ�ʳɹ�
	LRF_SetFrequency_Fail,
	
	LRF_SetRevolution_OK,//���÷ֱ��ʳɹ�
	LRF_SetRevolution_Fail,
	
	LRF_Mesure_OK,//�������
	LRF_DataCheck_Fail,//��������У��ʧ��
	
	LRF_SetOff_OK,//���ùػ��ɹ�
	LRF_SetOff_Fail,
}LaserRFStateDef;

typedef struct 
{
	LaserRFStateDef State;
	float Dist;
}LaserRFDef;

extern LaserRFDef LaserRF;

char LaserRFInite(void);//��ʼ��
void StartTOMeasure(void);//��ʼ����
void StopMeasure(void);//ֹͣ����

LaserRFStateDef LRF_Analy(unsigned char*s);//���ݽ���  ֻд������������ȡ

#endif
