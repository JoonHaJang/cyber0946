/*-----------------------------------------------------------------------------
 * Name: 040_CheckPoint
 * DESC: NumberRun(숫자 달리기) Ver0.0.1
 * 화면에 보여지는 것을 제어
-----------------------------------------------------------------------------*/
#include <iostream>
#include <Windows.h>
#include <time.h>
using namespace std;

int main()
{
	init:
    //value mapping  0,   1,  2,   3,   4,   5,    6,   7
    char tile[] = { ' ', '-', '|', '1' ,'2', '3', '4', '5' };
	char exitCh;
    int map[7][25] =
    {
      //col  0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24   row
            {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, //0
            {3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0}, //1
            {4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0}, //2
            {5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0}, //3
            {6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0}, //4
            {7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0}, //5
            {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, //6
    };

    int arrIndexX[5] = { 0, 0, 0, 0, 0 }; //{ };
    bool isFinish = false;  //루프를 제어..

    while(true)
    {
        system("cls");
		//초기값 설정
        for(int i = 1; i < 6; i++) //1 ~ 5
        {
            //0,1,2,3,4,5
			int index = i - 1;
            //0, 0, 0, 0, 0
			int indexValue = arrIndexX[index];
			//map[1,0] = 3
            int temp = map[i][indexValue];
			//한칸 이동
            map[i][indexValue + 1] = temp;
			//이전 칸을 다시 0으로 
            map[i][indexValue] = 0;
        }
		
		//랜덤 타임 
        srand(time(0));
        int selectIndex = rand() % 5; //0 ~ 4
        arrIndexX[selectIndex]++;
		for (int i = 0; i < 5; i++) {
			cout << i+1 << "select conunter : " << (arrIndexX[i]) << endl;
		}
		//

		// 1~5 숫자가 존재하는 Row
        int selectRow = selectIndex + 1;
        int indexCol = arrIndexX[selectIndex];

        int temp = map[selectRow][indexCol];
        map[selectRow][indexCol + 1] = temp;
        map[selectRow][indexCol] = 0;

		//tile 개수 만큼
        for(int i = 0; i < 7; i++)
        {
			//column 개수 만큼
            for(int j = 0; j < 25; j++)
            {
             /*   int tileIndex = map[i][j];
                cout << tile[tileIndex];*/
                cout << tile[map[i][j]]; 
            }
            cout << endl;
        }
		for (int i = 0; i < 5; i++)
		{
			if (arrIndexX[i] >= 22) //0 -> 1, 1 -> 2
			{
				cout << (i + 1) << "번이 1등" << endl;
				cin.clear();
				cin.ignore();
				cout << "한 번 더 수행할까요??" << endl;
				exitCh = cin.get();

				if (exitCh == 'Y' || exitCh == 'y') {
					isFinish = false; //for문에 break;
					goto init;
				}
				else {
					isFinish = true; //for문에 break;
					goto exit;
				}
			}
		}

		for (int i = 0; i < 5; i++)
			arrIndexX[i]++;

		Sleep(100); //#include <Windows.h>
	}
	exit:
	cout << "Enjoy your game?? Please visit again!!!!" << endl;
	return 0;
}

