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



/*
typedef struct PID {
	float err_last;
	float err_last_last;
	float sum_err;
}PID;

float target_num=0;
float actual=0;
float pid_num[3] = {0.2,0.015,0.2};

PID num_pid;
float pid_realize(PID* sprt,float target_num,float actual_num) {
	float err ,duty;
	err = target_num - actual_num;
	sprt->sum_err += err;
	duty = pid_num[0] * err +pid_num[1]*sprt->sum_err + pid_num[2] * (err - sprt->err_last);
	sprt->err_last = err;
	actual = duty;
	return duty;

}

int count = 0;
float speed=0;
int main(vpid) {
	scanf("%f",&target_num);
	for (count = 0; count < 100; count++) {
		speed = pid_realize(pid_num, target_num, actual);
		printf("%.2f\n", speed);
		count++;
	}
	return 0;
}*/


struct _pid {
	float SetSpeed; //�����趨ֵ
	float ActualSpeed; //����ʵ��ֵ
	float err; //����ƫ��ֵ
	float err_last; //������һ��ƫ��ֵ
	float Kp, Ki, Kd; //������������֡�΢��ϵ��
	float duty; 
	float integral; //�������ֵ
}pid;

void PID_init() {
	printf("PID_init begin \n");
	pid.SetSpeed = 0.0;
	pid.ActualSpeed = 0.0;
	pid.err = 0.0;
	pid.err_last = 0.0;
	pid.duty = 0.0;
	pid.integral = 0.0;
	pid.Kp = 0.2;
	pid.Ki = 0.015;
	pid.Kd = 0.2;
	printf("PID_init end \n");
}

float PID_realize(float speed) {
	pid.SetSpeed = speed;
	pid.err = pid.SetSpeed - pid.ActualSpeed;
	pid.integral += pid.err;
	pid.duty = pid.Kp * pid.err + pid.Ki * pid.integral + pid.Kd * (pid.err - pid.err_last);
	pid.err_last = pid.err;
	pid.ActualSpeed = pid.duty * 1.0;
	return pid.ActualSpeed;
}


int main() {
	printf("System begin \n");
	PID_init();
	int count = 0;
	while (count < 1000)
	{
		float speed = PID_realize(200.0);
		printf("%f\n", speed);
		count++;
	}
	return 0;
}