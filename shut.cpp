#include <opencv2/opencv.hpp>
#include <string>
//照相程序
using namespace cv;
using namespace std;
int main()
{
	VideoCapture cap(1);
	bool width1=cap.set(CAP_PROP_FRAME_WIDTH,1280);//CV_CAP_PROP_FRAME_WIDTH——宽度
	bool height1=cap.set(CAP_PROP_FRAME_HEIGHT,480);//CV_CAP_PROP_FRAME_HEIGHT——高度
	Mat src;
	int i=1;
	while(1)
	{
		cap>>src;
		imshow("cap",src);
		if (waitKey(10)=='r')
		{
			ostringstream oss;
			oss<<i<<".jpg";
			cout<<oss.str()<<endl;
			string c=oss.str();
			imwrite(c,src);
			i++;
		}
		if (waitKey(40)==27)
			return 0;
	}
	return 0;

}
