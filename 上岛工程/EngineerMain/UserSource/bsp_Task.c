#include "bsp_Task.h"
 
 
#define NULL_ctr 0 //不控制
#define SPD_ctr 1  //速度控制
#define LOC_ctr 2  //位置控制

 TaskComDef TaskCom;
 xQueueHandle TaskComD_QueHandle;//顺序执行电机任务队列
 //           [任务条数][控制参数][电机号]
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
 
 //初始化
 void bsp_Task_Inite()
 {
	 int len=sizeof(TaskComDef);//获取 TaskComDef 长度
	 TaskComD_QueHandle=xQueueCreate(20,len);//信息队列 深度为20 每个单元12字节
	 InsertTask(CatchTask,&TaskCom);
 }
 
 
 //根据任务序列数组对任务命令结构体进行填充
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


/*判断电机是否达到预期
 传入>> 电机参数结构	电机控制方式  
 传出>> 电机状态 0：未达到预期 1：达到预期

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

/*设置电机参数
 传入>> 电机参数结构	电机控制方式  控制数据
 传出>> void

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
		//判断各电机是否完成
	while(1){
	if( IsMotorReady(&Motor1,tem_TaskCom->Motor1.Ctr_Type)&&IsMotorReady(&Motor2,tem_TaskCom->Motor2.Ctr_Type)
		&&IsMotorReady(&Motor3,tem_TaskCom->Motor3.Ctr_Type)&&IsMotorReady(&Motor4,tem_TaskCom->Motor4.Ctr_Type)
	  &&IsMotorReady(&MotorA,tem_TaskCom->MotorA.Ctr_Type)&&IsMotorReady(&MotorB,tem_TaskCom->MotorB.Ctr_Type)
		&&IsMotorReady(&MotorC,tem_TaskCom->MotorC.Ctr_Type)&&IsMotorReady(&MotorD,tem_TaskCom->MotorD.Ctr_Type)
	  &&IsMotorReady(&MotorE,tem_TaskCom->MotorE.Ctr_Type)&&IsMotorReady(&MotorF,tem_TaskCom->MotorF.Ctr_Type)
	)break;
	vTaskDelay(100 / portTICK_RATE_MS);//延迟挂起
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
		//发命令
}
