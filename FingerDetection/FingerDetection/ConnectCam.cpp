#include "TransImage.h"

int main(int argc, char** argv)
{
	VideoCapture vc(0); // 0 : 노트북 내장 웹캠
	if (!vc.isOpened()) return 0; // 연결실패
	vc.set(CV_CAP_PROP_FOURCC, CV_FOURCC('D', 'I', 'V', '3'));
	Mat img;
	int Count = -1, newCount1 = -1, newCount2 = -1;
	while (1) {
		vc >> img; // 웹캠에서 받은 데이터 대입
		if (img.empty()) break; // 받은거 없으면 종료
		imshow("Test", img);
		TransImage resultImg = TransImage(img, img.cols, img.rows);
		resultImg.MakeReverse();
		resultImg.DetectSkinColor();
		resultImg.Contours();

		imshow("Test", resultImg.getPicture());  // 화면에 띄우기
		if (waitKey(10) == 27) break; // ESC키 눌리면 종료
	}
	destroyAllWindows();
	return 0;
}
