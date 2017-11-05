#include "TransImage.h"

int main(int argc, char** argv)
{
	VideoCapture vc(0); // 0 : ��Ʈ�� ���� ��ķ
	if (!vc.isOpened()) return 0; // �������
	vc.set(CV_CAP_PROP_FOURCC, CV_FOURCC('D', 'I', 'V', '3'));
	Mat img;
	int Count = -1, newCount1 = -1, newCount2 = -1;
	while (1) {
		vc >> img; // ��ķ���� ���� ������ ����
		if (img.empty()) break; // ������ ������ ����
		imshow("Test", img);
		TransImage resultImg = TransImage(img, img.cols, img.rows);
		resultImg.MakeReverse();
		resultImg.DetectSkinColor();
		resultImg.Contours();

		imshow("Test", resultImg.getPicture());  // ȭ�鿡 ����
		if (waitKey(10) == 27) break; // ESCŰ ������ ����
	}
	destroyAllWindows();
	return 0;
}
