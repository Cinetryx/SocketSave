/**
  SocketSave.cpp
Purpose: Simple interface to OpenNi cameras

@author Chris Gwilliams (encima)
@version 0.1 14/04/2015
*/

#include "SocketSave.h"
#include <fstream>
#include <stdio.h>
#include <time.h>
#include <ctime>
#include <string>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <sstream>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <opencv2/opencv.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

#define DTFMT "%Y_%m_%d_%H%M%S"
bool capturing  = true;
//destructor

void SocketSave::error(const char *msg) {
    perror(msg);
	close(sockfd);
    exit(1);
}

void SocketSave::startServer(int portno) {
    int newsockfd, pid;
    socklen_t clilen;
    struct sockaddr_in serv_addr, cli_addr;

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
       error("ERROR opening socket");
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portno);
    if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) 
        error("ERROR on binding");
    listen(sockfd,5);
    clilen = sizeof(cli_addr);
	cout << "Server listening on port: " << portno << endl;
    while (capturing == true) {
		cout << "wh" << capturing << endl;
        newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
        if (newsockfd < 0) 
            error("ERROR on accept");
        pid = fork();
        if (pid < 0)
            error("ERROR on fork");
        if (pid == 0)  {
            close(sockfd);
            processCapture(newsockfd);
            exit(0);
        }
        else close(newsockfd);
	//	capturing = false;
    } /* end of while */
	cout << "Closing server socket" << endl;
    close(sockfd);
}

void SocketSave::processCapture (int sock) {
   int n;
   char buffer[256];
   string msg;   
   bzero(buffer,256);
   n = read(sock,buffer,255);
   if (n < 0) error("ERROR reading from socket");
   printf("Recv: %s\n", buffer);
   if(strcmp("capture", buffer) == 0) {
	    msg = "point clouds saved";
		time_t curTime = time(NULL);
		struct tm *aTime = localtime(&curTime);
		stringstream ss;
		char timeBuff[21];
		strftime(timeBuff, 21, DTFMT, aTime);
		ss << "../saved/";
		ss << "colour_" << timeBuff;
		ss << ".png";
		//examples for getting feeds from cam and depth
		Mat col;
		getColour(col);
		imwrite(ss.str(), col);
		cout << "Saved colour image" << endl;
		ss.str("");

		Mat pcl;
		getPointCloud(pcl);
		cout << getDepth(pcl, 100, 100, false) << endl;
		//imwrite("../saved/cloud.png", pcl);
		ss << "../saved/";
		ss << "pcl_" << timeBuff;
		ss<< ".ply";
		cout << "saving point cloud" << endl;
		ofstream outFile(ss.str().c_str());

		if ( !outFile ) {
			cerr << "Error opening output file: cloud.ply!" << endl;
			exit(1);
		}

		outFile << "ply" << endl;
		outFile << "format ascii 1.0" << endl;
		outFile << "element vertex 307200" << endl;
		outFile << "property float x" << endl;
		outFile << "property float y" << endl;
		outFile << "property float z" << endl;
		outFile << "end_header" << endl;

		for(int x = 0; x < pcl.cols; x++) {
			for(int y = 0; y < pcl.rows; y++) {
				cv::Vec3f point = pcl.at<cv::Vec3f>(y, x);
				outFile << std::setprecision(8) << point[0] << ' ' << point[1] << ' ' << point[2] << endl;
			}
		}
		outFile << endl;
		cout << "pointint cloud saved" << endl;
	   if (n < 0) error("ERROR reading from socket");
	   printf("Here is the message: %s\n",buffer);
	   n = write(sock,"point clouds saved",18);
	   if (n < 0) error("ERROR writing to socket");
   }else if(strcmp("quit", buffer) == 0) {
	   cout << capturing << endl;
	   capturing = false;
	   cout << capturing << endl;
   }
   n = write(sock, msg.c_str(), 18);
   if (n < 0) error("ERROR writing to socket");
   close(sock);
}

bool SocketSave::openCapture() {
	//open capture and set mode, error if it fails
	cap.open(CAP_OPENNI_ASUS);
	if(!cap.isOpened()) {
		cout << "***Could not initialize capturing...***\n" << endl;
		cout << "Current parameter's value: \n" << 0 << endl;
		return false;
	}
	cout << "Capture Opened" << endl;
	return cap.set( CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CAP_OPENNI_VGA_30HZ );
}

void SocketSave::getDepthMap(Mat &depth) {
	getImage(depth, CAP_OPENNI_DEPTH_MAP);
}

void SocketSave::getPointCloud(Mat &pcl) {
	getImage(pcl, CAP_OPENNI_POINT_CLOUD_MAP);
}

void SocketSave::getBlackWhite(Mat &bw) {
	getImage(bw, CAP_OPENNI_GRAY_IMAGE);
}

void SocketSave::getColour(Mat &col) {
	getImage(col, CAP_OPENNI_BGR_IMAGE);
}

/**
  Root method to get an image from the OpenNI source
  @param src the image to save the output to
  @param channel the type of stream to retrieve

*/
void SocketSave::getImage(Mat &src, int channel) {
	cap.grab();
	cap.retrieve(src, channel);
}

/**
  Get depth from depth map or point clouds
  @param src the source image
  @param x x coordinate
  @param y y coordinate
  @param isDepth determines how to calculate depth
  */
float SocketSave::getDepth(Mat &src, double x, double y, bool isDepth) {
	if(isDepth) {
		//in mm
		return src.at<unsigned short>(y, x);
	}else{
		Vec3f pt3D = src.at<Vec3f>(y, x);
		//in m
		return pt3D[2];
	}
}

int main(int argc, char const *argv[]) {

    if (argc < 2) {
        fprintf(stderr,"ERROR, no port provided\n");
        exit(1);
    }

	//instantiate and open capture
	SocketSave ci;
	ci.openCapture();
	ci.startServer(atoi(argv[1]));

    return 1;
}



