#pragma once





//�Ŵ��㷨

//������Ⱥ��С������Χ���ֵ�ĸ�����
#define population_num  20
//�����������
#define reproduc_num   100
//���彻�����
#define cross_probability_set 0.56

//����������
#define var_probability_set 0.05
 
//������Ⱥ��ά����������û�б�ʾһ������(Ⱦɫ�壬����Χ���ֵ,1<x<301��

int chromosome[20][10];//20�У���20�����壬ÿ��/ÿ������9λ���룬10Ϊ��Ӧ��
int chromosome_copy[20][10];
int max_chromosome[11];//ÿ������Ⱦɫ��
int fin_max_chromosome[11];//ȫ������Ⱦɫ��

int choose_chromosome[20];


void find_Optimal(float chromosome[20][10], int Offspring);
void heredity_optimize(void);
//�������

//����Ⱦɫ�壨����Ψһָ������һ��Ⱦɫ�������������һ����

//������Ⱥ��Ⱦɫ����Ŀ�����𰸵ļ���



//����Ⱥ��ϵ�д������ţ����Ŵ��㷨


//��Ӧ�ȣ����𰸵ĺ���/�����̶ȣ��ù�ʽ���


//ѡ����Ӧ��Խ�󣬱�ѡ��ĸ���Խ��
// ���棬����𰸵ķ�Χ
// ���죬