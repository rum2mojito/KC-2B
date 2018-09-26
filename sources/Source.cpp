#include <opencv/cv.h>
#include<opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <stdio.h>
#include <stdlib.h>
#include <windows.h>
#include <iostream>
#include <cstring>

using namespace std;

HANDLE hComm;
OVERLAPPED m_ov;
COMSTAT comstat;
DWORD  m_dwCommEvents;
int zs = -1;

bool openport(char *portname)
{
	WCHAR wszClassName[256];
	memset(wszClassName, 0, sizeof(wszClassName));
	MultiByteToWideChar(CP_ACP, 0, portname, strlen(portname) + 1, wszClassName,
		sizeof(wszClassName) / sizeof(wszClassName[0]));
	hComm = CreateFile(wszClassName, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, 0);
	if (hComm == INVALID_HANDLE_VALUE) return false;
	else      return true;
}

bool setupdcb(int rate_arg)
{
	DCB dcb;
	int rate = rate_arg;
	memset(&dcb, 0, sizeof(dcb));
	if (!GetCommState(hComm, &dcb))
	{
		return FALSE;
	}

	dcb.DCBlength = sizeof(dcb);
	dcb.BaudRate = rate;
	dcb.Parity = NOPARITY;
	dcb.fParity = 0;
	dcb.StopBits = ONESTOPBIT;
	dcb.ByteSize = 8;
	dcb.fOutxCtsFlow = 0;
	dcb.fOutxDsrFlow = 0;
	dcb.fDtrControl = DTR_CONTROL_DISABLE;
	dcb.fDsrSensitivity = 0;
	dcb.fRtsControl = RTS_CONTROL_DISABLE;
	dcb.fOutX = 0;
	dcb.fInX = 0;
	dcb.fErrorChar = 0;
	dcb.fBinary = 1;
	dcb.fNull = 0;
	dcb.fAbortOnError = 0;
	dcb.wReserved = 0;
	dcb.XonLim = 2;
	dcb.XoffLim = 4;
	dcb.XonChar = 0x13;
	dcb.XoffChar = 0x19;
	dcb.EvtChar = 0;
	if (!SetCommState(hComm, &dcb))
	{
		return false;
	}
	else
		return true;
}

bool setuptimeout(DWORD ReadInterval, DWORD ReadTotalMultiplier, DWORD ReadTotalconstant, DWORD WriteTotalMultiplier, DWORD WriteTotalconstant)
{
	COMMTIMEOUTS timeouts;
	timeouts.ReadIntervalTimeout = ReadInterval;
	timeouts.ReadTotalTimeoutConstant = ReadTotalconstant;
	timeouts.ReadTotalTimeoutMultiplier = ReadTotalMultiplier;
	timeouts.WriteTotalTimeoutConstant = WriteTotalconstant;
	timeouts.WriteTotalTimeoutMultiplier = WriteTotalMultiplier;
	if (!SetCommTimeouts(hComm, &timeouts))
	{
		return false;
	}
	else
		return true;
}

void ReceiveChar()
{
	BOOL bRead = TRUE;
	BOOL bResult = TRUE;
	DWORD dwError = 0;
	DWORD BytesRead = 0;
	char RXBuff;
	for (;;)
	{
		bResult = ClearCommError(hComm, &dwError, &comstat);
		if (comstat.cbInQue == 0)
			continue;
		if (bRead)
		{
			bResult = ReadFile(hComm, &RXBuff, 1, &BytesRead, &m_ov);
			printf("%c", RXBuff);
			if (!bResult)
			{
				switch (dwError = GetLastError())
				{
				case ERROR_IO_PENDING:
				{
					bRead = FALSE; break;
				}
				default:
				{break;
				}
				}
			}
			else
			{
				bRead = TRUE;
			}
		}
		if (!bRead)
		{
			bRead = TRUE;
			bResult = GetOverlappedResult(hComm, &m_ov, &BytesRead, TRUE);
		}
	}
}


bool WriteChar(char* m_szWriteBuffer, DWORD m_nToSend)
{
	BOOL bWrite = TRUE;
	BOOL bResult = TRUE;
	DWORD BytesSent = 0;
	HANDLE m_hWriteEvent = NULL;
	ResetEvent(m_hWriteEvent);
	if (bWrite)
	{
		m_ov.Offset = 0;
		m_ov.OffsetHigh = 0;
		bResult = WriteFile(hComm, m_szWriteBuffer, m_nToSend, &BytesSent, &m_ov);
		if (!bResult)
		{
			DWORD dwError = GetLastError();
			switch (dwError)
			{
			case ERROR_IO_PENDING: {
				BytesSent = 0;
				bWrite = FALSE;
				break; }
			default:break;
			}
		}
	}
	if (!bWrite)
	{
		bWrite = TRUE;
		bResult = GetOverlappedResult(hComm, &m_ov, &BytesSent, TRUE);
		if (!bResult)
		{
			printf("GetOverlappedResults() in WriteFile()");
		}
	}
	if (BytesSent != m_nToSend)
	{
		printf("WARNING: WriteFile() error.. Bytes Sent: %d; Message Length: %d\n", BytesSent, strlen((char*)m_szWriteBuffer));
	}
	return true;
}

char date = '0';


template<class T> class Image
{
private:
	IplImage* imgp;
public:
	Image(IplImage* img = 0) { imgp = img; }
	~Image() { imgp = 0; }
	void operator=(IplImage* img) { imgp = img; }
	inline T* operator[](const int rowIndx) {
		return ((T *)(imgp->imageData + rowIndx*imgp->widthStep));
	}
};

typedef struct {
	unsigned char b, g, r;
} RgbPixel;

typedef struct {
	float b, g, r;
} RgbPixelFloat;

typedef Image<RgbPixel>       RgbImage;
typedef Image<RgbPixelFloat>  RgbImageFloat;
typedef Image<unsigned char>  BwImage;
typedef Image<float>          BwImageFloat;

IplImage* img;
IplImage* transimg = NULL;
IplImage* lineImg = NULL;
IplImage* tmp;
CvCapture *pCapture;
int step = 1;
int RGB = 70;
int times = 15;
int mycount = 0;
bool visited[100];
int lastPoint = -1;

CvPoint carPos_head;
CvPoint carPos_tail;
CvPoint2D32f corners[100];
CvPoint2D32f originPoints[4];
CvPoint2D32f newPoints[4];
CvMat *transMat = cvCreateMat(3, 3, CV_32FC1);

void cvThin(IplImage* src, IplImage* dst, int iterations = 1);
void mouseEvent(int mouseevent, int x, int y, int flags, void* param);
void mouseMoveEvent(int mouseevent, int x, int y, int flags, void* param);
void findLine();
void exchange(IplImage *img);
void getTrack();
double calculateAngle(CvPoint2D32f roadPoint);
void destory();

void hough(IplImage *img);
void swap(int &a, int &b);
bool getdis(int a[4], int b[4]);
void getline(CvSeq *pcvSeqLines, int &num, int a[1000][4]);
bool getPoint_dis(CvPoint p1, CvPoint p2);
bool getangle(int a[4], int b[4]);
void instruction(int DirFlag, int Angel);

void instruction(int DirFlag, int Angle)
{
	if (Angle<15 && Angle>-15)
	{
		date = 'A';
		cout << "A";
		cvWaitKey(25);

	}
	else if (Angle < -15)
	{
		date = 'L';
		cout << "L";
		cvWaitKey(25);
		//car.move_f();
		//cvWaitKey(100);
	}
	else
	{
		date = 'R';
		cout << "R";
		cvWaitKey(25);
		//car.move_f();
		//cvWaitKey(100);
	}
}

void mouseEvent(int mouseevent, int x, int y, int flags, void *param)
{

	CvFont font;
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 1, CV_AA);
	if (mouseevent == CV_EVENT_LBUTTONDOWN)
	{
		CvPoint pt = cvPoint(x, y);
		char temp[16];
		sprintf(temp, "(%d,%d)", pt.x, pt.y);
		cvPutText(img, temp, pt, &font, cvScalar(255, 255, 255, 0));
		cvCircle(img, pt, 2, cvScalar(255, 0, 0, 0), CV_FILLED, CV_AA, 0);
		cvShowImage("colorfulImg", img);
	}
	if (step == 1 && mouseevent == CV_EVENT_LBUTTONDOWN)
	{
		originPoints[0] = cvPoint2D32f(x, y);
		++step;
	}
	else if (step == 2 && mouseevent == CV_EVENT_LBUTTONDOWN)
	{
		originPoints[1] = cvPoint2D32f(x, y);
		++step;
	}
	else if (step == 3 && mouseevent == CV_EVENT_LBUTTONDOWN)
	{
		originPoints[2] = cvPoint2D32f(x, y);
		++step;
	}
	else if (step == 4 && mouseevent == CV_EVENT_LBUTTONDOWN)
	{
		originPoints[3] = cvPoint2D32f(x, y);
		++step;
	}
}

void findLine()
{
	//cvCvtColor(colorfulImg, lineImg, CV_BGR2GRAY);
	RgbImage  imgA(transimg);
	BwImage   imgB(lineImg);

	for (int i = 0; i<transimg->height; i++)
	{
		for (int j = 0; j<transimg->width; j++)
		{
			if (imgA[i][j].b<RGB && imgA[i][j].g<RGB && imgA[i][j].r<RGB)
			{
				imgB[i][j] = 255;
			}
			else
			{
				imgB[i][j] = 0;
			}
		}
	}

	cvThin(lineImg, lineImg, times);
	//hough(lineImg);
}

void cvThin(IplImage* src, IplImage* dst, int iterations)
{
	cvCopy(src, dst);
	BwImage dstdat(dst);
	IplImage* t_image = cvCloneImage(src);
	BwImage t_dat(t_image);
	for (int n = 0; n < iterations; n++)
		for (int s = 0; s <= 1; s++) {
			cvCopy(dst, t_image);
			for (int i = 0; i < src->height; i++)
				for (int j = 0; j < src->width; j++)
					if (t_dat[i][j]) {
						int a = 0, b = 0;
						int d[8][2] = { { -1, 0 },{ -1, 1 },{ 0, 1 },{ 1, 1 },
						{ 1, 0 },{ 1, -1 },{ 0, -1 },{ -1, -1 } };
						int p[8];
						p[0] = (i == 0) ? 0 : t_dat[i - 1][j];
						for (int k = 1; k <= 8; k++) {
							if (i + d[k % 8][0] < 0 || i + d[k % 8][0] >= src->height ||
								j + d[k % 8][1] < 0 || j + d[k % 8][1] >= src->width)
								p[k % 8] = 0;
							else p[k % 8] = t_dat[i + d[k % 8][0]][j + d[k % 8][1]];
							if (p[k % 8]) {
								b++;
								if (!p[k - 1]) a++;
							}
						}
						if (b >= 2 && b <= 6 && a == 1)
						{
							if (!s && !(p[2] && p[4] && (p[0] || p[6])))
								dstdat[i][j] = 0;
							else if (s && !(p[0] && p[6] && (p[2] || p[4])))
								dstdat[i][j] = 0;
						}
					}
		}
	cvReleaseImage(&t_image);
}

void hough(IplImage *img)
{
	CvMemStorage *pcvMStorage = cvCreateMemStorage();
	double fRho = 1;
	double fTheta = CV_PI / 180;
	int nMaxLineNumber = 30;   
	double fMinLineLen = 60;   
	double fMinLineGap = 80; 
	CvSeq *pcvSeqLines = cvHoughLines2(img, pcvMStorage, CV_HOUGH_PROBABILISTIC, fRho, fTheta, nMaxLineNumber, fMinLineLen, fMinLineGap);
	cvSetZero(img);
	int i;
	//cout<<pcvSeqLines->total<<endl;
	int a[1000][4];
	int n;
	getline(pcvSeqLines, n, a);
	//cout<<n;
	IplImage* img1 = cvCloneImage(img);
	CvPoint cor_ini[100];
	int sum = 0;
	for (int i = 0; i < pcvSeqLines->total; i++) {
		if (a[i][0]) {
			cvLine(img1, cvPoint(a[i][0], a[i][1]), cvPoint(a[i][2], a[i][3]), cvScalar(255));
			cor_ini[sum] = cvPoint(a[i][0], a[i][1]);
			sum++;
			cor_ini[sum] = cvPoint(a[i][2], a[i][3]);
			sum++;
		}
	}

	for (int i = 0; i < sum - 1; i++) {
		for (int j = i + 1; j < sum; j++)
		{
			if (cor_ini[i].x && cor_ini[j].x && getPoint_dis(cor_ini[i], cor_ini[j])) {
				cor_ini[i].x = (cor_ini[j].x < cor_ini[i].x) ? cor_ini[i].x : cor_ini[j].x;
				cor_ini[i].y = (cor_ini[j].x < cor_ini[i].x) ? cor_ini[i].y : cor_ini[j].y;
				cor_ini[j].x = 0;
			}
		}
	}
	for (int i = 0; i < sum; i++) {
		if (cor_ini[i].x) {
			corners[mycount].x = cor_ini[i].x;
			corners[mycount].y = cor_ini[i].y;
			mycount++;
		}
	}

}


bool getdis(int a[4], int b[4]) {
	double x1 = abs(a[2] - a[0]), y1 = abs(a[3] - a[1]);
	double carLength = sqrt(pow(x1, 2.0) + pow(y1, 2.0));
	double x2 = abs(b[0] - a[0]), y2 = abs(b[1] - a[1]);
	double x3 = abs(b[0] - a[2]), y3 = abs(b[1] - a[3]);
	double dis = (x2*y3 + x3*y2) / carLength;
	return dis<20;
}

void getline(CvSeq *pcvSeqLines, int &num, int a[1000][4]) {
	int tmp = pcvSeqLines->total;
	for (int i = 0; i < tmp; i++) {
		CvPoint* line = (CvPoint*)cvGetSeqElem(pcvSeqLines, i);
		a[i][0] = line[0].x;
		a[i][1] = line[0].y;
		a[i][2] = line[1].x;
		a[i][3] = line[1].y;
	}
	for (int i = 0; i < tmp - 1; i++) {
		for (int j = i + 1; j < tmp; j++) {
			if (a[i][0] && a[j][0] && getangle(a[i], a[j]) && getdis(a[i], a[j])) {
				if (abs(a[i][0] - a[i][2])>abs(a[i][1] - a[i][3])) {
					if (a[i][0]>a[i][2]) {
						swap(a[i][0], a[i][2]);
						swap(a[i][1], a[i][3]);
					}
					if (a[j][0]>a[j][2]) {
						swap(a[j][0], a[j][2]);
						swap(a[j][1], a[j][3]);
					}
					if (a[i][0] > a[j][0]) {
						a[i][0] = a[j][0];
						a[i][1] = a[j][1];
					}
					if (a[i][2] < a[j][2]) {
						a[i][2] = a[j][2];
						a[i][3] = a[j][3];
					}
				}
				else {
					if (a[i][1]>a[i][3]) {
						swap(a[i][0], a[i][2]);
						swap(a[i][1], a[i][3]);
					}
					if (a[j][1]>a[j][3]) {
						swap(a[j][0], a[j][2]);
						swap(a[j][1], a[j][3]);
					}
					if (a[i][1] > a[j][1]) {
						a[i][0] = a[j][0];
						a[i][1] = a[j][1];
					}
					if (a[i][3] < a[j][3]) {
						a[i][2] = a[j][2];
						a[i][3] = a[j][3];
					}
				}
				a[j][0] = 0;
				a[j][1] = 0;
				a[j][2] = 0;
				a[j][3] = 0;
			}
		}
	}
	num = 0;
	for (int i = 0; i < tmp; i++)
		if (a[i][0])
			num++;
}

double getDistance(CvPoint2D32f x, CvPoint2D32f y)       
{
	return sqrt(pow(x.x - y.x, 2) + pow(x.y - y.y, 2));
}

void swap(int &a, int &b) {
	int tmp = a;
	a = b;
	b = a;
}

bool getPoint_dis(CvPoint p1, CvPoint p2) {
	double x1 = abs(p1.x - p2.x), y1 = abs(p1.y - p2.y);
	return sqrt(pow(x1, 2.0) + pow(y1, 2.0)) < 30;
}

bool getangle(int a[4], int b[4]) {
	double x1 = abs(a[2] - a[0]), y1 = abs(a[3] - a[1]);
	double x2 = abs(b[2] - b[0]), y2 = abs(b[3] - b[1]);
	double carLength = sqrt(pow(x1, 2.0) + pow(y1, 2.0)),
		roadLength = sqrt(pow(x2, 2.0) + pow(y2, 2.0)),
		cosAngle = (x1 * x2 + y1 * y2) / (carLength * roadLength);
	return (acos(cosAngle) * 180 / 3.14)<30;
}

bool carPosition(IplImage* img, CvPoint& carPos_head, CvPoint& carPos_tail) {
	IplImage  * Color_frame_Logitech;
	Color_frame_Logitech = img;
	int bChannel = 0, gChannel = 1, rChannel = 2;
	int height, width, step, channels;
	uchar *data;
	cvSaveImage("1.png", img);
	/* get image's properties */
	height = Color_frame_Logitech->height;
	width = Color_frame_Logitech->width;
	step = Color_frame_Logitech->widthStep;
	channels = Color_frame_Logitech->nChannels;
	data = (uchar *)Color_frame_Logitech->imageData;
	//outfile<<"the properties of the left image are: width="<<width<<", height="<<height<<", nChannel="<<channels<<", widthStep="<<img_left->widthStep<<", dataOrder="<<img_left->dataOrder<<endl;

	int red_x = 0, red_y = 0;
	int sumHeight = 0;
	int sumWidth = 0;
	int nRedPoint = 0;
	for (int i = 0; i<height; i++)
	{
		for (int j = 0; j<width; j++)
		{
			if ((data[i*step + j*channels + rChannel] - data[i*step + j*channels + gChannel])>50 && (data[i*step + j*channels + rChannel] - data[i*step + j*channels + bChannel])>40)
			{
				nRedPoint++;
				sumHeight = sumHeight + i;
				sumWidth = sumWidth + j;
			}
		}
	}
	if (nRedPoint)
	{
		sumHeight = sumHeight / nRedPoint;
		sumWidth = sumWidth / nRedPoint;
		red_x = sumHeight;
		red_y = sumWidth;
		cvCircle(Color_frame_Logitech, cvPoint(sumWidth, sumHeight), 10, cvScalar(0, 0, 255), 2);
		carPos_head = cvPoint(sumWidth, sumHeight + 16);
														//cvShowImage("lala",Color_frame_Logitech);
	}
	sumHeight = 0;
	sumWidth =		0;
	int nGreenPoint = 0;
	for (int i = 0; i<height; i++)
	{
		for (int j = 0; j<width; j++)
		{//45 40 100
			if ((data[i*step + j*channels + gChannel] - data[i*step + j*channels + rChannel])>25 && (data[i*step + j*channels + gChannel] - data[i*step + j*channels + rChannel])>20 && sqrt(pow(i - red_x, 2) + pow(j - red_y, 2))<150)
			{
				nGreenPoint++;
				sumHeight = sumHeight + i;
				sumWidth = sumWidth + j;
			}
		}
	}
	if (nGreenPoint)
	{
		sumHeight = sumHeight / nGreenPoint;
		sumWidth = sumWidth / nGreenPoint;
		cvCircle(Color_frame_Logitech, cvPoint(sumWidth, sumHeight), 10, cvScalar(0, 255, 0), 2);
		carPos_tail = cvPoint(sumWidth, sumHeight + 16);
		//cvShowImage("lala",Color_frame_Logitech);
		return true;
	}


	return false;
}

double calculateAngle(CvPoint2D32f roadPoint)
{
	CvPoint carVector = cvPoint(carPos_head.x - carPos_tail.x, carPos_head.y - carPos_tail.y),
		roadVector = cvPoint(roadPoint.x - (carPos_head.x + carPos_tail.x) / 2, roadPoint.y - (carPos_tail.y + carPos_head.y) / 2);
	double carLength = sqrt(pow(carVector.x, 2.0) + pow(carVector.y, 2.0)),
		roadLength = sqrt(pow(roadVector.x, 2.0) + pow(roadVector.y, 2.0)),
		sinAngle = (carVector.x * roadVector.y - carVector.y * roadVector.x) / (carLength * roadLength);
	return (asin(sinAngle) * 180 / 3.14);
}

void getTrack()
{
	CvPoint2D32f centre = cvPoint2D32f((carPos_head.x + carPos_tail.x) / 2, (carPos_head.y + carPos_tail.y) / 2);
	CvPoint2D32f head = cvPoint2D32f(carPos_head.x, carPos_head.y);
	if (lastPoint == -1)
	{
		for (int i = 0; i < mycount; ++i)
		{
			if (lastPoint == -1)
			{
				lastPoint = i;
			}
			else if (getDistance(centre, corners[i]) < getDistance(centre, corners[lastPoint]))
			{
				lastPoint = i;
			}
		}
		visited[lastPoint] = true;
		return;
	}

	int Angle, minDis = -1;
	for (int i = 0; i < mycount; ++i)
	{
		if (!visited[i] && i != lastPoint)
		{
			if (minDis == -1)
			{
				minDis = i;
			}
			else if (getDistance(corners[i], corners[lastPoint]) < getDistance(corners[minDis], corners[lastPoint]))
			{
				minDis = i;
			}
		}
	}

	if (minDis == -1)
	{
		++step;
	}
	else
	{
		Angle = int(calculateAngle(corners[minDis]));
		CvPoint2D32f head, tail;
		head.x = carPos_head.x;
		head.y = carPos_head.y;
		tail.x = carPos_tail.x;
		tail.y = carPos_tail.y;
		if (getDistance(corners[lastPoint], head) > getDistance(corners[lastPoint], tail))
			visited[lastPoint] = true;
		if (getDistance(corners[minDis], head)<50)
			lastPoint = minDis;
		zs = minDis;

		instruction(0, Angle);
	}
}



int main()
{
	char com[10] = "COM4";
	int bbb = 9600;
	int i;



	if (openport(com))
		printf("HEY\n");
	else
		printf("NOPE\n");

	if (setupdcb(bbb))
		printf("DONE\n");

	if (setuptimeout(0, 0, 0, 0, 0))

		PurgeComm(hComm, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);

	bool flag = false;
	bool flag1 = false;
	cv::VideoCapture cap;
	// open the default camera, use something different from 0 otherwise;
	// Check VideoCapture documentation.
	if (!cap.open(1))return 0;
	//pCapture = cvCaptureFromCAM(0);
	//cout << pCapture;
	cvNamedWindow("colorfulImg", 1);
	cvMoveWindow("colorfulImg", 0, 0);

	/*for (int i = 0; i < 10; i++) {
		img = cvQueryFrame(pCapture);
	}*/

	
	int counter = 0;
	while (true)
	{
		
		counter++;
		if (counter % 5 != 0) {
			
			continue;
		}
		//cv::imshow("colorfulImg", img);
		//cvShowImage("colorfulImg", img);
		cv::Mat framein;
		cap >> framein;
		cv::imshow("colorfulImg", framein);
		img = new IplImage(framein);
		//img = cvLoadImage("test2.jpg");
		cvShowImage("colorfulImg",	img);
		cvSetMouseCallback("colorfulImg", mouseEvent);

		if (step == 5)
		{
			transimg = cvCloneImage(img);
			newPoints[0] = cvPoint2D32f(0, 0);
			newPoints[1] = cvPoint2D32f(img->width, 0);
			newPoints[2] = cvPoint2D32f(0, img->height);
			newPoints[3] = cvPoint2D32f(img->width, img->height);

			cvGetPerspectiveTransform(originPoints, newPoints, transMat);
			cvWarpPerspective(img, transimg, transMat);
			cvWarpPerspective(img, img, transMat);

			if (!flag)
			{
				lineImg = cvCreateImage(cvSize(transimg->width, transimg->height),
					IPL_DEPTH_8U, 1);

				flag = 1;
			}
			findLine();
			cvNamedWindow("win2");
			mycount = 0;

		}
		else if (step > 5)
		{
			cvWarpPerspective(img, img, transMat);

			if (step == 6)
			{
				if (!flag1)
				{
					tmp = cvCloneImage(lineImg);
					hough(lineImg);
					for (int i = 0; i < mycount; ++i)
					{
						visited[i] = false;
						cvRectangle(img, cvPoint(corners[i].x, corners[i].y), cvPoint(corners[i].x + 10, corners[i].y + 10), CV_RGB(255, 0, 0), 1, CV_AA, 0);
					}



				}
				cvShowImage("colorfulImg", img);
			}
			if (step == 7)
			{
				for (int i = 0; i < mycount; ++i)
				{
					visited[i] = false;
					cvRectangle(img, cvPoint(corners[i].x, corners[i].y), cvPoint(corners[i].x + 10, corners[i].y + 10), CV_RGB(255, 0, 0), 1, CV_AA, 0);
				}
				cvShowImage("colorfulImg", img);
			}
			if (step == 8)
			{
				carPosition(img, carPos_head, carPos_tail);

				for (int i = 0; i < mycount; ++i)
				{

					cvRectangle(img, cvPoint(corners[i].x, corners[i].y), cvPoint(corners[i].x + 10, corners[i].y + 10), CV_RGB(255, 0, 0), 1, CV_AA, 0);
				}
				cvCircle(img, cvPoint(corners[zs].x, corners[zs].y), 10, cvScalar(255, 0, 0), 2);
				cvShowImage("colorfulImg", img);
				getTrack();

				WriteChar(&date, 1);

			}
			if (step == 9)
			{
				printf("SHIT");
				date = 'P';
				WriteChar(&date, 1);
			}
		}
		cvShowImage("win2", lineImg); 

		cvCreateTrackbar("Step", "colorfulImg", &step, 9, NULL);
		cvCreateTrackbar("RGB", "colorfulImg", &RGB, 255, NULL);
		cvWaitKey(5);
	}






	return 0;
}
