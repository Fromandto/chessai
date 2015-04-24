#include <cstdio>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <time.h>

using namespace std;
using namespace cv;

const int size = 3;
const int maxState = 20000;
const int initState = 3 * 3 * 3 * 3 * 3 * 3 * 3 * 3 * 3 - 1;

int verbose = 1;
int resultArray[9] = {0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0};
int chuchan = 100;
int yingshe[9] = {3 , 6 , 9 , 2, 5 , 8 , 1 , 4 , 7};
int step = 0;
ros::Publisher pub;
int adapatFrame = 100; //在这么多帧内让摄像头适应光线
int location[9][2] = {169 ,  326,
   169  , 204,
   174  ,  77,
   293  , 332,
   296   ,207,
   296  ,  78,
   422 ,  333,
   423 ,  207,
   425  ,  81};

Scalar colorvector[9][2] = {159 ,  316,
   159  , 194,
   164  ,  67,
   283  , 322,
   286   ,197,
   286  ,  68,
   412 ,  323,
   413 ,  197,
   415  ,  71};

int colorCalibrated = 0;
float threshold1 = 0.0;
float threshold2 = 0.0;
int putAction = 0;
int actionTimer = 0;

enum BoardState {
    ZERO = 0,
    ONE = 1,
    UNOCCUPIED = 2
};

enum WinState {
    ZEROWIN = -1,
    ONEWIN = -2,
    TIE = -3,
    UNFINISH = -4
};

WinState stateChange[maxState][size*size];

WinState findStateChange(int boardState, BoardState parity);
int putPiece(int boardState, int row, int col, BoardState player);
BoardState getState(int boardState, int row, int col);
WinState judgeWin(int boardState);
int perceptState(Mat frame);
float getThrehold(float a , float b , float c);
void commandSend(int i);
int decideAction(int *resultArray);

int main(int argc , char ** argv)
{
    for(int i = 0; i < 3; i++)
        cout << endl;
    findStateChange(initState, ZERO);
    
	/*for(int i = 18864; i < maxState; i++){
        for(int k = 0; k < size * size; k++)
            cout << getState(i, k / size, k % size);
        cout << ": ";
        for(int j = 0; j < size * size; j++)
            cout << stateChange[i][j] << " ";
        cout << endl;
    }*/
	ros::init(argc , argv , "chessai");
	ros::NodeHandle n;
	pub = n.advertise<std_msgs::Int16>("/command" , 1);
	VideoCapture cap(1);
	namedWindow("chess" , 1);
	Mat chessframe;
	while(1)
	{
		ros::spinOnce();
		cap >> chessframe;
		imshow("chess" , chessframe);
		char c = waitKey(1);
		if(c == 32)
			break;

		if(adapatFrame != 0)
		{
			adapatFrame --;
			continue;
		}
		int result = perceptState(chessframe);
		if(verbose == 1)
		{
			for(int i = 0 ; i < 9 ; i ++)
			{
				cout << resultArray[i];
			}
			cout << endl;
		
			for(int i = 0 ; i < 9 ; i ++)
			{
				cout << stateChange[result][i] << " ";
			}
			cout << endl;
		}
		if(c == 114)
		{
			step = 0;
		}
		if(decideAction(resultArray) == 1 && putAction == 0)
		{
			actionTimer ++;
			if(actionTimer > 180)
			{
				putAction = 1;
			}
		}
		cout << "decideResult:" << decideAction(resultArray) << " putAction :" << putAction << " actionTimer :" << actionTimer << "\n" << endl;
		if((c == 97 && chuchan == 0) || putAction == 1)
		{
			cout << "a detected "<<endl;
			chuchan = 100;
			int winStep = 0;
			for(int i = 0 ; i < 9 ; i ++)
			{
				if(stateChange[result][i] == -1 && (winStep != 1))
				{
					cout << "bisheng : " << i << endl;
					commandSend(i);
					winStep = 1;
					putAction = 0;
					actionTimer = 0;
				}
			}
			cout << "before while" << endl;
			while(winStep != 1)
			{
				cout << "in while" << endl;
				srand(time(NULL));
				int tempr = rand() % 9;
				cout << result << " " << tempr << " " << stateChange[result][tempr] << endl;
				if(stateChange[result][tempr] == -3 || result == 19682)
				{
					cout << "bubisheng : " << tempr << endl;
					commandSend(tempr);
					winStep = 1;
					putAction = 0;
					actionTimer = 0;
				}
			}
			cout << "a detected finally" << endl;
		}
		if(chuchan > 0)
			chuchan --;
	}
    return 0;
}

int decideAction(int *resultArray)
{
	int onecount = 0;
	int zerocount = 0;
	for(int i = 0 ; i < 9 ; i ++)
	{
		if(resultArray[i] == 1)
		{
			onecount ++;
		}
		if(resultArray[i] == 0)
		{
			zerocount ++;
		}
	}
	if(onecount == zerocount)
	{
		return 1;
	}
	return 0;
}

int perceptState(Mat frame)
{
	vector<Mat> channels;
	split(frame , channels);
	for(int i = 0 ; i < 9 ; i ++)
	{
		Mat patchr = channels[2](Rect(location[i][0] , location[i][1] , 80 , 80));
		Mat patchg = channels[1](Rect(location[i][0] , location[i][1] , 80 , 80));
		Scalar meantemp;
		Scalar stdtemp;
		meanStdDev(patchr , meantemp , stdtemp);
		colorvector[i][0] = meantemp;
		meanStdDev(patchg , meantemp , stdtemp);
		colorvector[i][1] = meantemp;
	}
	vector<int> labels;
	float colorvector2[9][2] = {{colorvector[0][0].val[0] , colorvector[0][1].val[0]} , {colorvector[1][0].val[0] , colorvector[1][1].val[0]} , {colorvector[2][0].val[0] , colorvector[2][1].val[0]} , {colorvector[3][0].val[0] , colorvector[3][1].val[0]} , {colorvector[4][0].val[0] , colorvector[4][1].val[0]} , {colorvector[5][0].val[0] , colorvector[5][1].val[0]} , {colorvector[6][0].val[0] , colorvector[6][1].val[0]} , {colorvector[7][0].val[0] , colorvector[7][1].val[0]} , {colorvector[8][0].val[0] , colorvector[8][1].val[0]}};
	Mat centers(3 , 2 , CV_32F);
	Mat colorvector3(9 , 2 , CV_32F , colorvector2);

	if(colorCalibrated == 0)
	{
		//颜色校准pattern：（机械臂在（真实世界人的）视野左边）灰白绿，白绿白，灰绿灰
		//标号：机械臂用灰色标号0，人用绿色标号1，未填充标号2
		kmeans(colorvector3 , 3 , labels , TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0) , 3 , KMEANS_RANDOM_CENTERS , centers);
		threshold1 = getThrehold(centers.at<float>(0 , 0) , centers.at<float>(1 , 0) , centers.at<float>(2 , 0));
		threshold2 = getThrehold(centers.at<float>(0 , 1) , centers.at<float>(1 , 1) , centers.at<float>(2 , 1));
		colorCalibrated = 1;
	}
	else if(colorCalibrated == 1)
	{
		if(verbose == 1)
		{
			cout << colorvector3 << endl;
			cout << threshold1 << " " << threshold2 << endl;
		}
		for(int i = 0 ; i < 9 ; i ++)
		{
			if(colorvector2[i][1] < threshold2 && colorvector2[i][0] > threshold1)
			{
				resultArray[i] = 0;
				continue;
			}
			else
			{
				if(colorvector2[i][0] < threshold1)
				{
					resultArray[i] = 1;
					continue;
				}
				resultArray[i] = 2;
			}
		}
	}
	if(verbose == 1)
	{
		for(int i = 0 ; i < 9 ; i ++)
		{
			cout << resultArray[i];
		}
		cout << endl;
	}
	int result = 0;
	for(int i = 0 ; i < 9 ; i ++)
	{
		result += resultArray[i] * (pow(3 , i));
	}
	if(verbose == 1)
	{
		cout << result << endl;
	}
	return result;
}

void commandSend(int i )
{
	i = yingshe[i];
	step ++;
	int j = step * 10 + i;
	std_msgs::Int16 msg;
	msg.data = j;
	cout << "before publish" << endl;
	pub.publish(msg);
	cout << "after publish" << endl;
}

float getThrehold(float a , float b , float c)
{
	float t;
	if(a > b)
	{
		t = a;
		a = b;
		b = t;
	}
	if(b > c)
	{
		t = b;
		b = c;
		c = t;
	}
	if(a > b)
	{
		t = a;
		a = b;
		b = t;
	}
	return (a + c) / 2;
}

WinState findStateChange(int boardState, BoardState parity){
    for(int row = 0; row < size; row++)
        for(int col = 0; col < size; col++)
            if(getState(boardState, row, col) == UNOCCUPIED){
                int newState = putPiece(boardState, row, col, parity);
                WinState winState = judgeWin(newState);
                if(winState != UNFINISH){
                    stateChange[boardState][size * row + col] = winState;
                    return winState;
                }
                else{
                    if(parity == ONE)
                        stateChange[boardState][size * row + col] = findStateChange(newState, ZERO);
                    else
                        stateChange[boardState][size * row + col] = findStateChange(newState, ONE);
                }
            }

    if(parity == ZERO){
        WinState bestPosible = ONEWIN;
        for(int row = 0; row < size; row++)
            for(int col = 0; col < size; col++){
                if(stateChange[boardState][size * row + col] == ZEROWIN){
                    bestPosible = ZEROWIN;
                    return ZEROWIN;
                }
                else if(stateChange[boardState][size * row + col] == TIE)
                    bestPosible = TIE;
            }
        return bestPosible;
    }
    else{
        WinState bestPosible = ZEROWIN;
        for(int row = 0; row < size; row++)
            for(int col = 0; col < size; col++){
                if(stateChange[boardState][size * row + col] == ONEWIN){
                    bestPosible = ONEWIN;
                    return ONEWIN;
                }
                else if(stateChange[boardState][size * row + col] == TIE)
                    bestPosible = TIE;
            }
        return bestPosible;
    }
}

int putPiece(int boardState, int row, int col, BoardState player){
    int power = 1;
    for(int i = 0; i < size * row + col; i++)
        power *= size;
    if(player == ZERO)
        return boardState - 2*power;
    else
        return boardState - 2*power+ power;
}

BoardState getState(int boardState, int row, int col){
    int power = 1;
    for(int i = 0; i < size * row + col; i++)
        power *= size;
    switch((boardState / power) % size){
        case 0:
            return ZERO;
        case 1:
            return ONE;
        case 2:
            return UNOCCUPIED;
    }
}

WinState judgeWin(int boardState){
    int board[size][size];
    for(int row = 0; row < size; row++)
        for(int col = 0; col < size; col++)
            board[row][col] = getState(boardState, row, col);

    for(int row = 0; row < size; row++)
        if(board[row][0] == ONE && board[row][1] == ONE && board[row][2] == ONE)
            return ONEWIN;
    for(int col = 0; col < size; col++)
        if(board[0][col] == ONE && board[1][col] == ONE && board[2][col] == ONE)
            return ONEWIN;
    if(board[0][0] == ONE && board[1][1] == ONE && board[2][2] == ONE)
        return ONEWIN;
    if(board[0][2] == ONE && board[1][1] == ONE && board[2][0] == ONE)
        return ONEWIN;

    for(int row = 0; row < size; row++)
        if(board[row][0] == ZERO && board[row][1] == ZERO && board[row][2] == ZERO)
            return ZEROWIN;
    for(int col = 0; col < size; col++)
        if(board[0][col] == ZERO && board[1][col] == ZERO && board[2][col] == ZERO)
            return ZEROWIN;
    if(board[0][0] == ZERO && board[1][1] == ZERO && board[2][2] == ZERO)
        return ZEROWIN;
    if(board[0][2] == ZERO && board[1][1] == ZERO && board[2][0] == ZERO)
        return ZEROWIN;

    for(int row = 0; row < size; row++)
        for(int col = 0; col < size; col++)
            if(board[row][col] == UNOCCUPIED)
                return UNFINISH;
    return TIE;
}
