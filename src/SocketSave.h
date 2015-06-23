#ifndef CAM2IMAGE_H
#define CAM2IMAGE_H

#include <opencv2/opencv.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

class SocketSave {
	private:
		VideoCapture cap;
		Mat img;
		int sockfd;
	public:
		SocketSave() {};
		~SocketSave() {};
		void startServer(int portno);
		bool isCapturing();
		void setCapturing(bool cap);
		bool openCapture();
		void error(const char* msg);
		void processCapture (int sock);
		void getDepthMap(Mat &depth);
		void getPointCloud(Mat &pcl);
		void getBlackWhite(Mat &bw);
		void getColour(Mat &col);
		void getImage(Mat (&src), int channel);
		float getDepth(Mat &src, double x, double y, bool isDepth);
};

#endif // CAM2IMAGE
