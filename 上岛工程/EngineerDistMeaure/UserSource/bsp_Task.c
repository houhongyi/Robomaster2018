#include "bsp_Task.h"
 
 
#define NULL_ctr 0 //������
#define SPD_ctr 1  //�ٶȿ���
#define LOC_ctr 2  //λ�ÿ���

 TaskComDef TaskCom;
 xQueueHandle TaskComD_QueHandle;//˳��ִ�е���������
 //           [��������][���Ʋ���][�����]
 short CatchTask[6][2][11]=
 {
		{//         Motor1   ,Motor2   ,Motor3   ,Motor4   ,MotorA   ,MotorB   ,MotorC   ,MotorD   ,MotorE   ,MotorF   ,MOS      
		 /*TYPE*/   NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,
		 /*Data*/   0        ,0        ,0        ,0        ,0        ,0        ,0        ,0        ,0        ,0        ,0        
		},
		
		{//         Motor1   ,Motor2   ,Motor3   ,Motor4   ,MotorA   ,MotorB   ,MotorC   ,MotorD   ,MotorE   ,MotorF   ,MOS      
		 /*TYPE*/   NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,
		 /*Data*/   0        ,0        ,0        ,0        ,0        ,0        ,0        ,0        ,0        ,0        ,0        
		},
		
		{//         Motor1   ,Motor2   ,Motor3   ,Motor4   ,MotorA   ,MotorB   ,MotorC   ,MotorD   ,MotorE   ,MotorF   ,MOS      
		 /*TYPE*/   NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,
		 /*Data*/   0        ,0        ,0        ,0        ,0        ,0        ,0        ,0        ,0        ,0        ,0        
		},
		
		{//         Motor1   ,Motor2   ,Motor3   ,Motor4   ,MotorA   ,MotorB   ,MotorC   ,MotorD   ,MotorE   ,MotorF   ,MOS      
		 /*TYPE*/   NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,
		 /*Data*/   0        ,0        ,0        ,0        ,0        ,0        ,0        ,0        ,0        ,0        ,0        
		},
		
		{//         Motor1   ,Motor2   ,Motor3   ,Motor4   ,MotorA   ,MotorB   ,MotorC   ,MotorD   ,MotorE   ,MotorF   ,MOS      
		 /*TYPE*/   NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,
		 /*Data*/   0        ,0        ,0        ,0        ,0        ,0        ,0        ,0        ,0        ,0        ,0        
		},
		
		{//         Motor1   ,Motor2   ,Motor3   ,Motor4   ,MotorA   ,MotorB   ,MotorC   ,MotorD   ,MotorE   ,MotorF   ,MOS      
		 /*TYPE*/   NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,NULL_ctr ,
		 /*Data*/   0        ,0        ,0        ,0        ,0        ,0        ,0        ,0        ,0        ,0        ,0        
		}
 };
 
 //��ʼ��
 void bsp_Task_Inite()
 {
	 int len=sizeof(TaskComDef);//��ȡ TaskComDef ����
	 TaskComD_QueHandle=xQueueCreate(20,len);//��Ϣ���� ���Ϊ20 ÿ����Ԫ12�ֽ�
	 InsertTask(CatchTask,&TaskCom);
 }
 
 
 //�������������������������ṹ��������
 void InsertTask(short TaskMatrix[][2][11] ,TaskComDef* TaskCom)
 {
		int i=0;
	 for(i=0;i<6;i++){
	  TaskCom->Motor1.Ctr_Type= TaskMatrix[i][0][0];
		TaskCom->Motor1.Data    = TaskMatrix[i][1][0];
		
		TaskCom->Motor2.Ctr_Type= TaskMatrix[i][0][2];
		TaskCom->Motor2.Data    = TaskMatrix[i][1][2];
		 
		TaskCom->Motor3.Ctr_Type= TaskMatrix[i][0][3];
		TaskCom->Motor3.Data    = TaskMatrix[i][1][3];
		 
		TaskCom->Motor4.Ctr_Type= TaskMatrix[i][0][4];
		TaskCom->Motor4.Data    = TaskMatrix[i][1][4];
		 
		TaskCom->MotorA.Ctr_Type= TaskMatrix[i][0][5];
		TaskCom->MotorA.Data    = TaskMatrix[i][1][5];
		 
		TaskCom->MotorB.Ctr_Type= TaskMatrix[i][0][6];
		TaskCom->MotorB.Data    = TaskMatrix[i][1][6];
		 
		TaskCom->MotorC.Ctr_Type= TaskMatrix[i][0][7];
		TaskCom->MotorC.Data    = TaskMatrix[i][1][7];
		 
		TaskCom->MotorD.Ctr_Type= TaskMatrix[i][0][8];
		TaskCom->MotorD.Data    = TaskMatrix[i][1][8];
		
		TaskCom->MotorE.Ctr_Type= TaskMatrix[i][0][9];
		TaskCom->MotorE.Data    = TaskMatrix[i][1][9];
		
		TaskCom->MotorF.Ctr_Type= TaskMatrix[i][0][10];
		TaskCom->MotorF.Data    = TaskMatrix[i][1][10];
		
		//sent TO Que
		xQueueSendToBack(TaskComD_QueHandle,TaskCom,0);
	 }
 }


/*�жϵ���Ƿ�ﵽԤ��
 ����>> ��������ṹ	������Ʒ�ʽ  
 ����>> ���״̬ 0��δ�ﵽԤ�� 1���ﵽԤ��

*/
 short IsMotorReady(MotorXDef* Mot,short CtrType)
{
		switch(CtrType)
		{
			case NULL_ctr:
				return 1;
			case SPD_ctr:
					if(GetMotorSpdNear(Mot->State.State))
						return 1;
					else
						return 0;
			case LOC_ctr:
					if(GetMotorLocNear(Mot->State.State))
						return 1;
					else
						return 0;	
		}
		return 0;
}

/*���õ������
 ����>> ��������ṹ	������Ʒ�ʽ  ��������
 ����>> void

*/
void SetMotor(MotorXDef* Mot,short CtrType,float data)
{
		switch(CtrType)
		{
			case NULL_ctr:
				return ;
			case SPD_ctr:
				Mot->Com.Spd=data;
				return ;
			case LOC_ctr:
				Mot->Com.Loc=data;
				return ;	
		}
}
 
void TaskExcute(xQueueHandle Que,TaskComDef* tem_TaskCom)
{
	xQueueReceive(Que,tem_TaskCom,portMAX_DELAY);
		//�жϸ�����Ƿ����
	while(1){
	if( IsMotorReady(&Motor1,tem_TaskCom->Motor1.Ctr_Type)&&IsMotorReady(&Motor2,tem_TaskCom->Motor2.Ctr_Type)
		&&IsMotorReady(&Motor3,tem_TaskCom->Motor3.Ctr_Type)&&IsMotorReady(&Motor4,tem_TaskCom->Motor4.Ctr_Type)
	  &&IsMotorReady(&MotorA,tem_TaskCom->MotorA.Ctr_Type)&&IsMotorReady(&MotorB,tem_TaskCom->MotorB.Ctr_Type)
		&&IsMotorReady(&MotorC,tem_TaskCom->MotorC.Ctr_Type)&&IsMotorReady(&MotorD,tem_TaskCom->MotorD.Ctr_Type)
	  &&IsMotorReady(&MotorE,tem_TaskCom->MotorE.Ctr_Type)&&IsMotorReady(&MotorF,tem_TaskCom->MotorF.Ctr_Type)
	)break;
	vTaskDelay(100 / portTICK_RATE_MS);//�ӳٹ���
	}
	
	SetMotor(&Motor1,tem_TaskCom->Motor1.Ctr_Type,tem_TaskCom->Motor1.Data);
	SetMotor(&Motor2,tem_TaskCom->Motor2.Ctr_Type,tem_TaskCom->Motor2.Data);
	SetMotor(&Motor3,tem_TaskCom->Motor3.Ctr_Type,tem_TaskCom->Motor3.Data);
	SetMotor(&Motor4,tem_TaskCom->Motor4.Ctr_Type,tem_TaskCom->Motor4.Data);
	SetMotor(&MotorA,tem_TaskCom->MotorA.Ctr_Type,tem_TaskCom->MotorA.Data);
	SetMotor(&MotorB,tem_TaskCom->MotorB.Ctr_Type,tem_TaskCom->MotorB.Data);
	SetMotor(&MotorC,tem_TaskCom->MotorC.Ctr_Type,tem_TaskCom->MotorC.Data);
	SetMotor(&MotorD,tem_TaskCom->MotorD.Ctr_Type,tem_TaskCom->MotorD.Data);
	SetMotor(&MotorE,tem_TaskCom->MotorE.Ctr_Type,tem_TaskCom->MotorE.Data);
	SetMotor(&MotorF,tem_TaskCom->MotorF.Ctr_Type,tem_TaskCom->MotorF.Data);
		//������
}
