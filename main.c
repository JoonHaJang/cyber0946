#include <stdio.h>
#include <stdlib.h>
#include <windows.h>
#define MAXSIZE 1000
//------����� �ڷ� ���� ����-------------------------------------- 
typedef struct _process {
	int pro_num, cpu_time, arr_t, pri, wait_t, ta_t, rem_t;
}
process;
	// ���μ��� �ĺ���ȣ
	// ���μ����� CPU�� ����� �ð�
	// ���μ��� �켱����
	//���μ��� ���ð�
	//���μ��� ��ü �ð� , WT+BT
	//���� ���� �ð�

//------------�����ٸ� �˰��� �Լ�--------------------------------
int process_fcfs(process *pro, int n);
int process_srt(process *pro, int n);
int process_pri(process *pro, int n);
int process_rr(processs *pro, int n, int Q);
// ���� �κ��� Ÿ�� ���ҵ� ���� �Ű������� �޴´�. 
int process_sjf(process *pro, int n);
int process_generate(process *pro, int n);
// -----------���μ����� ���� �Լ� --------------------------------- 
void at_sort(process *pro, int n);
void resort(process *pro, int n);


//===========================================================================================
int main() {
	int i =0;
	int n =0;
	int Q =0;
	int index=0;
	float tat;
	FILE *fp;
	fp=fopen("proc.txt","r");
	process ready_queue[MAXSIZE];
	printf("�����б� ����\n");
	while(!feof(fp)) {
		fscanf(fp, "%d", &ready_queue[i].pro_num);
		fscanf(fp, "%d", &ready_queue[i].cpu_time);
		fscanf(fp, "%d", &ready_queue[i].arr_t);
		fscanf(fp, "%d", &ready_queue[i].pri);
		ready_queue[i].rem_t=ready_queue[i].cpu_time;
		index=index+1;
		printf("%d\n",index);
		i++;
	}
	fclose(fp);
	printf("�����б� ��\n");
	n=index-1;
	printf("%d\n",n);
	printf("\n=================Main Menu====================\n\n1. Read processes from proc.txt\n2. Generate random processes\n3. First come first Serve (FCFS)\n4. Shortest Job First (SJF)\n5. Shortest Remaining time First (SRTF)\n6. Priority\n7. Round Robin (RR)\n8. Exit\n==============================================\n");
	while(1){
			 int ch = 0;
			 printf("�޴��� �������ּ���.\n"); 
			 scanf("%d", &ch);	 
			 float tat=0.0;
			 switch(ch){
			 				  case 1: 
                                  printf("1. Read processes from proc.txt\n====================PROC======================\n");
                                  printf("P#	BT	AT	Pri\n");
                                  for(i=0; i<n; i++) {
                                           printf("%d	%d	%d	%d\n",ready_queue[i].pro_num, ready_queue[i].cpu_time, ready_queue[i].arr_t, ready_queue[i].pri);
                                  }
                                  printf("==============================================\n");
                                  continue;
                                  break;
                             case 2:
                                  printf("2. Generate random processes\n");
                                  process_generate(ready_queue, n);
                                  printf("\n\n==================�����Ϸ�===================\n");
                                  n++;
                                  continue;
                                  break;
                             case 3:		//fcfs
                                  at_sort(ready_queue, n);
                                  process_fcfs(ready_queue, n);
                                  resort(ready_queue, n);
                                  printf("3. First come first Serve (FCFS)\n====================FCFS======================\n");
		                          printf("P#	BT	AT	Pri	WT	TAT\n");
		                          for(i=0; i<n; i++) {
                                           tat=tat+ready_queue[i].ta_t;
			                               printf("%d	%d	%d	%d	%d	%d\n",ready_queue[i].pro_num, ready_queue[i].cpu_time, ready_queue[i].arr_t, ready_queue[i].pri, ready_queue[i].wait_t, ready_queue[i].ta_t);
                                  }
		                          printf("average turnaround time: %.2f\n",tat/n);
		                          printf("==============================================\n");
		                          continue;
		                          break;
                            case 4:		//sjf
                                 at_sort(ready_queue, n);
                                 printf("1");
                                 process_sjf(ready_queue, n);
                                 printf("2");
								 resort(ready_queue, n);
								 printf("3");
                                 printf("4. Shortest Job First (SJF)\n====================SJF=======================\n");
                                 printf("P#	BT	AT	Pri	WT	TAT\n");
                                 for(i=0; i<n; i++) {
                                          tat=tat+ready_queue[i].ta_t;
                                          printf("%d	%d	%d	%d	%d	%d\n",ready_queue[i].pro_num, ready_queue[i].cpu_time, ready_queue[i].arr_t, ready_queue[i].pri, ready_queue[i].wait_t, ready_queue[i].ta_t);
                                 }
                                 printf("average turnaround time: %.2f\n",tat/n);
		                         printf("==============================================\n");
		                         continue;
		                         break;
                           case 5:		//srt
		                         at_sort(ready_queue, n);
		                         process_srt(ready_queue, n);
		                         resort(ready_queue, n);
		
	 	                         printf("5. Shortest Remaining time First (SRTF)\n====================SRTF======================\n");
		                         printf("P#	BT	AT	Pri	WT	TAT\n");
		                         for(i=0; i<n; i++) {
                                          tat=tat+ready_queue[i].ta_t;
                                          printf("%d	%d	%d	%d	%d	%d\n",ready_queue[i].pro_num, ready_queue[i].cpu_time, ready_queue[i].arr_t, ready_queue[i].pri, ready_queue[i].wait_t, ready_queue[i].ta_t);
                                 }
                                 printf("average turnaround time: %.2f\n",tat/n);
		                         printf("==============================================\n");
		                         continue;
		                         break;
                           case 6:		//pri
                                at_sort(ready_queue, n);
                                process_pri(ready_queue, n);
		                        resort(ready_queue, n);
		
                                printf("6. Priority\n====================PRI=======================\n");
		                        printf("P#	BT	AT	Pri	WT	TAT\n");
		                        for(i=0; i<n; i++) {
                                         tat=tat+ready_queue[i].ta_t;
                                         printf("%d	%d	%d	%d	%d	%d\n",ready_queue[i].pro_num, ready_queue[i].cpu_time, ready_queue[i].arr_t, ready_queue[i].pri, ready_queue[i].wait_t, ready_queue[i].ta_t);
                                }
		                        printf("average turnaround time: %.2f\n",tat/n);
		                        printf("==============================================\n");
		                        continue;
		                        break;
                            case 7:		//RR
                                 printf("���� �Է�\n");
                                 scanf("%d",&Q);
                                 at_sort(ready_queue, n);
                                 process_rr(ready_queue, n, Q);
                                 resort(ready_queue, n);
                                 printf("7. Round Robin (RR)\n====================RR========================\n");
                                 printf("P#	BT	AT	Pri	WT	TAT\n");
                                 for(i=0; i<n; i++) {
                                          tat=tat+ready_queue[i].ta_t;
                                          printf("%d	%d	%d	%d	%d	%d\n",ready_queue[i].pro_num, ready_queue[i].cpu_time, ready_queue[i].arr_t, ready_queue[i].pri, ready_queue[i].wait_t, ready_queue[i].ta_t);
                                 }
                                 printf("average turnaround time: %.2f\n",tat/n);
                                 printf("==============================================\n\n");
                                 ent_key=0;
                                 continue;
                                 break;
                            case 8:
		                         printf("\n\n====================EXIT======================\n");
		                         exit(0);
		                         break;
                                 }
                           }
  }	
	
	
	
void at_sort(process *pro, int n) {
// arival time ������ ���μ����� ���� ���� �ش�.  
	process temp;
	int i,j;
	for (i=n-1; i>0;i--) {
		for (j=0;j<i;j++) {
			// i �� n-1���� �����ؼ� 0 ���� n ���� process�� ���ϰ�, �״���  i�� n-2 �� �Ǹ� �� �տ��� �ΰ� �� ���� ���� ���������� �� �Ѵ�.  
			if(pro[j].arr_t>pro[j+1].arr_t) {
				temp=pro[j+1];
				pro[j+1]=pro[j];
				pro[j]=temp;
			} else if(pro[j].arr_t==pro[j+1].arr_t&&pro[j].pro_num>pro[j+1].pro_num) {
				//���ÿ� ������ ���μ��� ��ȣ ������ �����Ѵ�. ���� ��ȣ�� ���� ���´�.  
				temp=pro[j+1];
				pro[j+1]=pro[j];
				pro[j]=temp;
			}
		}
	}
}

void resort(process *pro, int n) {
//���μ��� ����ü
// i �� n-1���� �����ؼ� 0 ���� n ���� process�� ���ϰ�, �״���  i�� n-2 �� �Ǹ� �� �տ��� �ΰ� �� ���� ���� ���������� �� �Ѵ�.  
//������ ������� ���ĵ� ���μ��� ����ü�� �����ͼ� CPU_time�� ���ð� WT�� ���� �ش�. 
	process temp;
	int i,j;
	for (i=n-1; i>0;i--) {
		for (j=0;j<i;j++) {
			if(pro[j].pro_num>pro[j+1].pro_num) {
				temp=pro[j+1];
				pro[j+1]=pro[j];
				pro[j]=temp;
			}
		}
	}
}

int process_fcfs(process *pro, int n) {
//i�� 0����,1, 2, 3, 4, 5, n-1 ���� 
// j�� i���� ����(i,j)�� ǥ�� �ϸ� 
// (0,0) (1,0) (2,(0,1)), (3,(0,1,2)),
//...(n-1, (0,1,2,3,..,n-2))�̷�������
//process�� cpu_time�� temp�� �����Ѵ�. 
//���� ù��° �͸� ����������, ù��° ���μ����� cpu_time
//�� �ڷ�  
// ���ð��� = ��ü CPU ��� �ð�����, k�� ���� �����ϱ�,
// i�� ������ �ð��� ����, �ű⿡ k�� �����ð��� ���ϸ�, ��ٸ� �ð��̴�. 
// ��� �ð��� ���� ��� �ٷ� �����ϱ� ������ 
// ���� ���� �ð��� �Žð� ����Ͽ� ���� ª�� ���� ������ ���� ���μ��� ���� �ϸ�, 
//���� ������ �� ��
	int temp;
	int wt=0;
	int i,j,k=0;
	for (i=0;i<n;i++) {
		temp=0;
		for (j=k;j<i;j++) {
			temp=temp+pro[j].cpu_time;
		}
		wt=temp-pro[i].arr_t+pro[k].arr_t;
		if(wt<=0) {
			k=i;
			pro[i].wait_t=0;
			pro[i].ta_t=pro[i].cpu_time + pro[i].wait_t;
		} else {
			pro[i].wait_t=wt;
			pro[i].ta_t=pro[i].cpu_time + pro[i].wait_t;
		}
	}
}
int process_srt(process *pro, int n) {
//���� ���μ��� ��
// ���� �ð�
// for������ ���� �Ǵ� ���� process�� ���� �ð��� now���ų� �۰�
// i�� n���� ���� �� ����, ������Ű�鼭 
//�ش� ���μ����� ���� �ð��� �ּҰ� ���� �۰�, ����̸� 
// ���� ������ ���μ����� i�� ��´�.  
//�׸��� ���� ���μ����� ���� �ð��� �ּ� ������ ��ϴ�. 
// �ݺ����� ���鼭 �� ���� �ּҰ��� �ȴ�.  
// ���� �ð����� ���� now_p���� ���� ���� ��,  
//�ð��� ������Ų��. 
//temp[0]�� ���� process[0] cpu_time �ð��� �� ��,  
// temp���ٰ� n�� ��ŭ�� ���� �ð��� ä�� �ִ´�.  
// �ð��� ���� ���� ���� ����ϰ� �ִ� ���μ����� 
// ���� ���� �ð� ī��Ʈ�� ���� ��Ų��.  
// ���� ���۽ð��� 0�� �� ���ο� ���μ����� �缱�� �ϴ� ����
// ���� ���μ����� ���ҽ����ش�.  
// �����ٸ� �Ϸ��� ���� temp�� ������ index�� ��� �־��µ� �̸� �ٽ� 
//process�� remain_t���� ������ �ش�.  
// �켱������� �����층 ���� ���� �켱���� ������ 
	int remain, min, now_p, i, temp[150];
	int now, wt=0;
	remain=n;
	now=pro[0].arr_t;
	while(remain>0) {
		min=5000;
		for (i=0; pro[i].arr_t<=now && i<n; i++) {
			if(pro[i].rem_t<min && pro[i].rem_t>0) {
				now_p=i;
				min=pro[i].rem_t;
			}
		}
		now++;
		if(temp[0] !=pro[0].cpu_time) {
			for (i=0; i<n; i++) {
				temp[i]=pro[i].rem_t;
			}
		}
		pro[now_p].rem_t--;
		if(pro[now_p].rem_t==0) {
			pro[now_p].wait_t=now-pro[now_p].arr_t-pro[now_p].cpu_time;
			pro[now_p].ta_t=pro[now_p].cpu_time + pro[now_p].wait_t;
			wt=wt+pro[now_p].wait_t;
			remain--;
		}
	}
	for (i=0; i<n; i++) {
		pro[i].rem_t=temp[i];
	}
}
int process_pri(process *pro, int n) {
// �켱 ���� �ð��� �� �ش� process�� cpu_time���� ä���ش�. 
//���� �ð��� process[0]�� ������ �ð����� ��´�.  
//���� ���μ��� �� �� 
// temp���� process���� ���� �ð��� ä���ش�. 
// ���� ������ �ֵ�� ����
//���� ���μ����� �ƴѵ� ���� min���� �켱�������� ���� ���μ����� 
//�� ���μ��� ��ȣ�� num�� �ǰ�  
// �� ���μ����� �켱������ �ּ� ���� �Ǹ� 
//flag�� 1�� �ٲ��ش�. 
//�ð��� ���� ��Ų��.  
//pro[num]�� �� �����ϰ� �ð����ٰ� �߰����ִ� ���̴�.
	int flag = 0;
	int i,time,remain,num, min, temp[150];
	for (i=0; i<n; i++) {
		pro[i].rem_t = pro[i].cpu_time;
	}
	time = pro[0].arr_t;
	remain = n;
	if(temp[0] !=pro[0].cpu_time) {
		for (i=0; i<n; i++) {
			temp[i]=pro[i].rem_t;
		}
	}
	while(remain>0) {
		min = 9999;
		for (i=0; pro[i].arr_t <= time && i<n; i++) {
			if(pro[i].rem_t != 0 && pro[i].pri<min && pro[i].cpu_time>0) {
				num = i;
				min = pro[i].pri;
				flag = 1;
			}
		}
		if(flag ==1) {
			pro[num].rem_t = 0;
			pro[num].wait_t = time - pro[num].arr_t;
			remain--;
			time += pro[num].cpu_time;
		} else {
			time ++;
		}
		flag = 0;
	}
	for (i=0; i<n; i++) {
		pro[i].ta_t = pro[i].wait_t + pro[i].cpu_time;
	}
	for (i=0; i<n; i++) {
		pro[i].rem_t=temp[i];
	}
}
int process_sjf(process *pro, int n) {
	int flag = 0;
	int i,time,remain,num, min, temp[150];
	for (i=0; i<n; i++) {
		pro[i].rem_t = pro[i].cpu_time;
	}
	time = pro[0].arr_t;
	remain = n;
	if(temp[0] !=pro[0].cpu_time) {
		for (i=0; i<n; i++) {
			temp[i]=pro[i].rem_t;
		}
	}
	while(remain>0) {
		min = 9999;
		for (i=0; pro[i].arr_t <= time && i<n; i++) {
			if(pro[i].rem_t != 0 && pro[i].cpu_time<min && pro[i].cpu_time>0) {
				num = i;
				min = pro[i].cpu_time;
				flag = 1;
			}
		}
		if(flag ==1) {
			pro[num].rem_t = 0;
			pro[num].wait_t = time - pro[num].arr_t;
			remain--;
			time += pro[num].cpu_time;
		} else {
			time ++;
		}
		flag = 0;
	}
	for (i=0; i<n; i++) {
		pro[i].ta_t = pro[i].wait_t + pro[i].cpu_time;
	}
	for (i=0; i<n; i++) {
		pro[i].rem_t=temp[i];
	}
}
int process_rr(process *pro, int n, int Q) {
// temp���� ������ temp2���ٴ� process���� Ư�� �� ���� 
//int i , count �׸��� totaltime 
//�ʱ�ȭ ���� 
// process���� ���� �ð��� temp2 �迭�� �־��ش�. 
//���� ���� 
// ���� �ӽ� ���� quantom 
// ���� process i�� ���� �ð��� ���ٸ�,  
// count�� ���� ��Ű��,  
//i�� ���� ���� �ٸ� process�� ����.  
// ���� ���� �ð��� ���Һ��� ũ�ٸ�, �� ���� ���ش��� �ٽ� ���� 
// �����ð��� 0�̰ų� ���Һ��� ũ�ٸ�, 
// temp���ٰ� ���� �ð� �������ְ�,
// ���������ϱ�, pro[i].remain_t�� 0���� 
// ��ü �ð��� ������ �ð�, �ֳ��ϸ� ������
//���Һ��� ���� ��, �� ���� total time�� �־� �־���.
//process[i]�� ��ü �ð��� tt
// �� ������ ��,
//while�� ���� ����
// ���� �׵��� temp�� ��Ҵ��� reamin_t�� �־��ش�. 
// wait �ð��� ��ü �ð� - cpu�ð�
	int temp, temp2[150];
	int i, count;
	int tt =0;
	//tt�� ������ ���� ���� �ʵȴ�. 
	if(temp2[0] !=pro[0].cpu_time) {
		for (i=0; i<n; i++) {
			temp2[i]=pro[i].rem_t;
		}
	}
	while(1) {
		for (i=0,count=0;i<n;i++) {
			temp=Q;
			if(pro[i].rem_t==0) {
				count++;
				continue;
			}
			if(pro[i].rem_t>Q)
							pro[i].rem_t=pro[i].rem_t-Q; else if(pro[i].rem_t>=0) {
				temp=pro[i].rem_t;
				pro[i].rem_t=0;
			}
			tt=tt+temp;
			pro[i].ta_t=tt;
		}
		if(n==count)
					break;
	}
	for (i=0;i<n;i++) {
		pro[i].rem_t=temp2[i];
		pro[i].wait_t=pro[i].ta_t-pro[i].cpu_time;
	}
}
int process_generate(process *pro, int n) {
	// ���� ���μ��� ����,  
	FILE *fp2;
	int j, found;
	fp2=fopen("proc.txt","a+");
	int i=n;
	//���μ��� ����, ���� ������ ���μ����� ����� �ľ��ϴ� �� ���ȴ�. 
	int bt =(rand()%25)+1;
	pro[i].pro_num=i+1;
	pro[i].cpu_time=bt;
	while(1) {
		pro[i].pri=rand()%50;
		found=0;
		for (j=0;j<i;++j) {
			if(pro[j].pri==pro[i].pri) {
				found=1;
				break;
			}
		}
		if(!found) break;
	}
	while(1) {
		pro[i].arr_t=rand()%50;
		found=0;
		for (j=0;j<i;++j) {
			if(pro[j].arr_t==pro[i].arr_t) {
				found=1;
				break;
			}
		}
		if(!found) break;
	}
	fprintf(fp2,"\r\n%d %d %d %d",pro[i].pro_num, pro[i].cpu_time, pro[i].arr_t, pro[i].pri);
	fclose(fp2);
}
