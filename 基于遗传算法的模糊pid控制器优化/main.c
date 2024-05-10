#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>


/*
�����Ŵ��㷨��ģ���������Ż���Ƶ���ϸ�������£�

ȷ��ģ���������ṹ��
	+����ģ��������������������������ȷ�����ǵ�ģ����������
	Ϊÿ��ģ�������������Ⱥ�����membership functions����
	���ģ�����ƹ�����Щ����������δ����������ģ����ӳ�䵽���������ģ������

����ģ��������������
	��ģ���������Ĳ������������Ⱥ����Ĳ�����ģ�����ƹ���ȣ�����Ϊ�Ŵ��㷨�е�Ⱦɫ�壨���壩��ÿ������������һ�����򣬶������������򹹳�һ��Ⱦɫ�塣

������Ӧ�Ⱥ�����
	���һ����Ӧ�Ⱥ���������ÿ��Ⱦɫ�壨��ģ���������������������ܡ��������ͨ������ģ���������ڿ���ϵͳ�ϵı��֣������ƽ���ͣ�SSE���������ME������������ָ�ꡣ

��ʼ����Ⱥ��
	�������һ��Ⱦɫ�壨��ģ������������������Ϊ�Ŵ��㷨�ĳ�ʼ��Ⱥ����Ⱥ�е�ÿ�����嶼����һ�����ܵ�ģ����������������

�Ŵ��㷨���̣�
	ѡ��Selection����������Ӧ�Ⱥ���ֵ���ӵ�ǰ��Ⱥ��ѡ��һ���ָ�����Ϊ������ͨ��ʹ�����̶�ѡ�񡢽�����ѡ��ȷ�����
	���棨Crossover�������ѡ�������������壬������һ���Ľ�����ʺͽ��淽ʽ���絥�㽻�桢��㽻��ȣ��������ֻ��������µ��Ӵ����塣
	���죨Mutation�����������ɵ��Ӵ����壬����һ���ı�����ʺͱ��췽ʽ����������졢�Ǿ��ȱ���ȣ��ı䲿�ֻ�����������Ⱥ�Ķ����ԡ�

������Ӧ�ȣ����������ɵ��Ӵ��������Ӧ�Ⱥ���ֵ��
	��Ⱥ���£��������ɵ��Ӵ������븸������ϲ����γ��µ���Ⱥ��������Ӧ�Ⱥ���ֵ����Ⱥ�еĸ����������ѡ����Ӧ��ֵ�ϸߵĸ�����Ϊ��һ���ĸ�����

��������ֹ������
	�ظ�ִ���Ŵ��㷨��ѡ�񡢽��桢���������������ֱ��������ֹ��������ֹ���������ǴﵽԤ��ĵ�����������Ӧ��ֵ����һ��������Ҫ��ȡ�

������Ž⣺
	���Ŵ��㷨��ֹʱ��ѡ����Ӧ�Ⱥ���ֵ��ߵ�Ⱦɫ�壨��ģ������������������Ϊ���Ž������

��֤���Ż���
	���õ�������ģ��������������Ӧ�õ�ʵ�ʿ���ϵͳ�У�������֤���Ż�������ϵͳ��ʵ�ʱ��ֵ�����Ӧ�Ⱥ������Ŵ��㷨�����ȣ��Խ�һ�����ģ�������������ܡ�
	���ϲ�������Ŵ��㷨��ģ�������������Ż���ƣ������ģ���������ۺ��Ŵ��㷨��ȫ��������������ʵ��Ӧ���У����ܻ���Ҫ���ݾ����������������ʵ��ĵ�������չ��

*/



/*˼·
	����ģ������pid����һ��Ŀ�������ͨ��pid�㷨ʹ�ý���ӳ�ʼ״̬�𲽴ﵽĿ����������г�ʼ״̬�ϴ�ʱ��
	�仯�죬������Ŀ�����ʱ���仯����������Ҫ�������ı�pid������������������θı�kp��ki��kd��������
	ģ��������������ģ�����Ƽ���������ģ������ģ�����õ������pid�����������У�ģ�����Ƽ���εõ�
	����״̬���ɲ����Ŵ��㷨�Ż���ÿһ��ģ�����Ƽ�Ϊһ��Ⱦɫ�壬ͨ��ϵ���㷨�����ų����տ��Ƽ�������
	ģ���������У������ٵõ�����pid������ʹ����Ӧ�ﵽ���٣��ȶ�����ȷ�Ľ����
*/




typedef struct PID {
	float err_last;
	float err_last_last;
	float sum_err;
}PID;

float target_num=0;
float actual = 500;
float pid_num[3] = {0.9,0,0};
float err ,duty;
int index=1;
PID num_pid;
float pid_realize(PID* sprt,float target_num,float actual_num) {

	err = target_num - actual_num;
	//printf("%f\n",err);
	/*if (abs(err) > 200)
	{
		index = 0;
	}
	else {
		index = 1;*/
		sprt->sum_err += err;
	//}
	duty = pid_num[0] * err +index*pid_num[1]*sprt->sum_err + pid_num[2] * (err - sprt->err_last);
	sprt->err_last = err;
//	printf("%f\n", duty);
	actual = duty;
	return actual;

}

int count = 0;
float speed=0;




/**********************ģ��pid******************/

//��ģ�����ָ������̵����Ҵ�ģ�����ʼ�����޹أ���ע���̼��ﵽ��̬��Ŀ��ֵ���Ľ�������е���
// �����仯�ʣ������仯��pid������Ϊģ���������Ŵ��㷨���Ż�����

/*
* ģ������������
* ȷ��ģ���������ṹ��
	+����ģ��������������������������ȷ�����ǵ�ģ����������
	Ϊÿ��ģ�������������Ⱥ�����membership functions����
	���ģ�����ƹ�����Щ����������δ����������ģ����ӳ�䵽���������ģ������

1. ģ����

2. ģ������

3. ȥģ����
*/
float det_Kp;
float det_Ki;
float det_Kd;                    //12, 1, 28,180  MAX16  ˳
int Turn_Ctl_fuzzy(int point)
{
  float turn_err,turn_det;
  static float turn_err_last;
  int turn;

  turn_err=point-93;
  turn_det=turn_err-turn_err_last;
  turn_err_last=turn_err;


//  turn_err=-14.5;
  turn_det=-3;

  int i,j;
  /**********������*******/
  int NB=-6;
  int NM=-4;
  int NS=-2;
  int ZO=0;
  int PS=2;
  int PM=4;
  int PB=6;




//  float  eRule[13]={-6.0,-5.0,-4.0,-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0,4.0,5.0,6.0};   //���E��ģ������
//  float  ecRule[7]={-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0}; //���仯��EC��ģ������

  float e=turn_err;
  float ec=turn_det;
  float es[7];
  float ecs[7];

  float menbKp;
  float menbKi;
  float menbKd;

  float formKp[7][7],formKi[7][7],formKd[7][7];                  //(��7X7)�������ȱ�
  float sumKp = 0;                         //�������ķ����н�ģ��,sumΪ��ĸ
  float sumKi = 0;
  float sumKd = 0;
  //ģ�������
  float FuzzyRuleKp[7][7] = {{PB,PB,PM,PM,PS,ZO,ZO},
                              {PB,PB,PM,PS,PS,ZO,NS},
                              {PB,PM,PM,PS,ZO,NS,NS},
                              {PM,PM,PS,ZO,NS,NM,NM},
                              {PS,PS,ZO,NS,NS,NM,NB},
                              {PS,ZO,NS,NM,NM,NM,NB},
                              {ZO,ZO,NM,NM,NM,NB,NB}};

  /******************����det_Kp*****************/

  //��ƫ��ӻ�������ת������Ӧ��ģ��������
  if(turn_err>=0)
  {
    e=turn_err/(93-2)*3;
  }
  else
  {
    e=turn_err/(184-93)*3;
  }
  //ƫ��仯�ʴӻ�������ת������Ӧ��ģ��������
  //turn_det_show=turn_det;
  ec=turn_det/20*3;
  /********�����ȼ���********************/
  es[NB] = ufl(e, -3, -1);                       //����1��ƫ��E
  es[NM] = uf(e, -3, -2, 0);
  es[NS] = uf(e, -3, -1, 1);
  es[ZO] = uf(e, -2, 0, 2);
  es[PS] = uf(e, -1, 1, 3);
  es[PM] = uf(e, 0, 2, 3);
  es[PB] = ufr(e, 1, 3);

  ecs[NB] = ufl(ec, -3, -1);                    //����2��ƫ��仯��Ec
  ecs[NM] = uf(ec, -3, -2, 0);
  ecs[NS] = uf(ec, -3, -1, 1);
  ecs[ZO] = uf(ec, -2, 0, 2);
  ecs[PS] = uf(ec, -1, 1, 3);
  ecs[PM] = uf(ec, 0, 2, 3);
  ecs[PB] = ufr(ec, 1, 3);
  /******ģ������ ����Ŀ��Ŷ�ͨ��ȡС����õ�*****/
  for (i = 0; i < 7; i++)
  {
      float w,h,r;
      for (j = 0; j < 7; j++)
      {
          h = es[i];
          r = ecs[j];
          w = fand(h, r);
          formKp[i][j] = w;
          sumKp += w;
       }
  }
  /***************************��ģ���������ķ������¼������**********************************/
    for (i = 0; i < 7; i++)
    {
        for (j = 0; j < 7; j++)
        {
            if (FuzzyRuleKp[i][j] == NB) {menbKp += (formKp[i][j] * cufl(formKp[i][j], -6, -2));}
            else if(FuzzyRuleKp[i][j] == NM) {menbKp += (formKp[i][j] * cuf(formKp[i][j], -6, -4, 0));}
            else if(FuzzyRuleKp[i][j] == NS) {menbKp += (formKp[i][j] * cuf(formKp[i][j], -6, -2, 2));}
            else if(FuzzyRuleKp[i][j] == ZO) {menbKp += (formKp[i][j] * cuf(formKp[i][j], -4, 0, 4));}
            else if(FuzzyRuleKp[i][j] == PS) {menbKp += (formKp[i][j] * cuf(formKp[i][j], -2, 2, 6));}
            else if(FuzzyRuleKp[i][j] == PM) {menbKp += (formKp[i][j] * cuf(formKp[i][j], 0, 4, 6));}
            else if(FuzzyRuleKp[i][j] == PB) {menbKp += (formKp[i][j] * cufr(formKp[i][j], 2, 6));}
        }
    }
    det_Kp = (menbKp / sumKp);


    float FuzzyRuleKd[7][7] = {{PS,NS,NB,NB,NB,NM,PS},
                              {PS,NS,NB,NM,NM,NS,ZO},
                              {ZO,NS,NM,NM,NS,NS,ZO},
                              {ZO,ZO,NS,NS,NS,NS,ZO},
                              {ZO,ZO,ZO,ZO,ZO,ZO,ZO},
                              {PB,NS,PS,PS,PS,PS,PB},
                              {PB,PM,PM,PM,PS,PS,PB}};
/******ģ������ ����Ŀ��Ŷ�ͨ��ȡС����õ�*****/
    for (i = 0; i < 7; i++)
    {
        float w,h,r;
        for (j = 0; j < 7; j++)
        {
            h = es[i];
            r = ecs[j];
            w = fand(h, r);
            formKd[i][j] = w;
        sumKd += w;
         }
    }
/***************************��ģ���������ķ������¼������**********************************/
    for (i = 0; i < 7; i++)
    {
        for (j = 0; j < 7; j++)
            {
                    if (FuzzyRuleKd[i][j] == NB) {menbKd += (formKd[i][j] * cufl(formKd[i][j], -6, -2));}
                    else if(FuzzyRuleKd[i][j] == NM) {menbKd += (formKd[i][j] * cuf(formKd[i][j], -6, -4, 0));}
                    else if(FuzzyRuleKd[i][j] == NS) {menbKd += (formKd[i][j] * cuf(formKd[i][j], -6, -2, 2));}
                    else if(FuzzyRuleKd[i][j] == ZO) {menbKd += (formKd[i][j] * cuf(formKd[i][j], -4, 0, 4));}
                    else if(FuzzyRuleKd[i][j] == PS) {menbKd += (formKd[i][j] * cuf(formKd[i][j], -2, 2, 6));}
                    else if(FuzzyRuleKd[i][j] == PM) {menbKd += (formKd[i][j] * cuf(formKd[i][j], 0, 4, 6));}
                    else if(FuzzyRuleKd[i][j] == PB) {menbKd += (formKd[i][j] * cufr(formKd[i][j], 2, 6));}
            }
    }

    det_Kd = (menbKd / sumKd);

    turn=stree_center+(int)(turn_err*(S_D5[Set][KP]+det_Kp*10.0f)+turn_det*(S_D5[Set][KD]+det_Kd*30.0f));

                    if(turn>=stree_max)   turn= stree_max;
                    if(turn<=stree_min)   turn = stree_min;
   // turn_p_show=turn_p+det_Kp;
   // turn_d_show=turn_d+det_Kd;
    return turn;
}

int main(void) {

	scanf("%f",&target_num);
	for (count = 0; count < 1000; count++) {
		speed = pid_realize(&num_pid, target_num, actual);
		printf("��%d��ֵ: %f\n",count, speed);

	}
	return 0;
}