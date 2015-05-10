// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#if defined(_MSC_VER) || defined(WIN32)  || defined(_WIN32) || defined(__WIN32__) \
    || defined(WIN64)    || defined(_WIN64) || defined(__WIN64__)
# include <io.h>
# include <fcntl.h>
# define SET_BINARY_MODE(handle) setmode(handle, O_BINARY)
#else
# define SET_BINARY_MODE(handle) ((void)0)
#endif

#include <svo/config.h>
#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <svo/frame.h>
#include <vector>
#include <string>
#include <vikit/math_utils.h>
#include <vikit/vision.h>
#include <vikit/abstract_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/pinhole_camera.h>
#include <opencv2/opencv.hpp>
#include <sophus/se3.h>
#include <iostream>
//#include "test_utils.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define BUFSIZE 10240

using namespace cv;


namespace svo {

class StdinAdaptor
{
  vk::AbstractCamera* cam_;
  svo::FrameHandlerMono* vo_;

public:
  StdinAdaptor();
  ~StdinAdaptor();
  void runFromStdin();
  void runFromFolder();
  int streamFromStdin();
};

StdinAdaptor::StdinAdaptor()
{
  cam_ = new vk::PinholeCamera(640, 480, 315.5, 315.5, 376.0, 240.0);
  vo_ = new svo::FrameHandlerMono(cam_);
  vo_->start();
}

StdinAdaptor::~StdinAdaptor()
{
  delete vo_;
  delete cam_;
}

void StdinAdaptor::runFromStdin()
{

  int imgCounter = 0;
  SET_BINARY_MODE(fileno(stdin));
  std::vector<char> data;
  bool skip=true;
  bool imgready=false;
  bool ff=false;
  int readbytes=-1;

  while (1)
  {   
    char ca[BUFSIZE];
    uchar c;
    if (readbytes!=0)
    {
      readbytes=read(fileno(stdin),ca,BUFSIZE);
      for(int i=0;i<readbytes;i++)
      {
         c=ca[i];
         if(ff && c==(uchar)0xd8)
         {
            skip=false;
            data.push_back((uchar)0xff);
         }
         if(ff && c==0xd9)
         {
            imgready=true;
            data.push_back((uchar)0xd9);
            skip=true;
         }
         ff=c==0xff;
         if(!skip)
         {
            data.push_back(c);
         }
         if(imgready)
         {
            if(data.size()!=0)
            {
               cv::Mat data_mat(data);
               cv::Mat frame(imdecode(data_mat, CV_LOAD_IMAGE_GRAYSCALE));
               //assert(!frame.empty());
               imgCounter++;

               // process frame
               vo_->addImage(frame, 0.01*imgCounter);

               // display tracking quality
               if(vo_->lastFrame() != NULL)
               {
                  std::cout << "Frame-Id: " << vo_->lastFrame()->id_ << " \t" << "#Features: " << vo_->lastNumObservations() << " \t" << "Proc. Time: " << vo_->lastProcessingTime()*1000 << " \t" << "Pose: " << vo_->lastFrame()->T_f_w_ << "ms \n";

                 // access the pose of the camera via vo_->lastFrame()->T_f_w_.


               }

               imshow("frame",frame);
               waitKey(1);
            } else
            {
               printf("warning");
            }
            imgready=false;
            skip=true;
            data.clear();
         }
      }
    } else
    {
      throw std::string("zero byte read");
    }
  }
}

/*
void StdinAdaptor::runFromFolder()
{
  for(int img_id = 2; img_id < 188; ++img_id)
  {
    // load image
    std::stringstream ss;
    ss << svo::test_utils::getDatasetDir() << "/sin2_tex2_h1_v8_d/img/frame_"
       << std::setw( 6 ) << std::setfill( '0' ) << img_id << "_0.png";
    if(img_id == 2)
      std::cout << "reading image " << ss.str() << std::endl;
    cv::Mat img(cv::imread(ss.str().c_str(), 0));
    assert(!img.empty());

    // process frame
    vo_->addImage(img, 0.01*img_id);

    // display tracking quality
    if(vo_->lastFrame() != NULL)
    {
    	std::cout << "Frame-Id: " << vo_->lastFrame()->id_ << " \t"
                  << "#Features: " << vo_->lastNumObservations() << " \t"
                  << "Proc. Time: " << vo_->lastProcessingTime()*1000 << "ms \n"
		  << "Pose: " << vo_->lastFrame()->T_f_w_ << "\n";
    	// access the pose of the camera via vo_->lastFrame()->T_f_w_.
    }
  }
}*/

int StdinAdaptor::streamFromStdin() {
   int frameCounter = 0;
   Mat frame, frameGreyscale, channels[3];

   VideoCapture cap("/dev/stdin");

    if(!cap.isOpened()) {
        std::cout << "Can't read from stdin.";
        return -1;
    }

    cap.set(CV_CAP_PROP_FPS, 60);

   // namedWindow("edges",1);

    for(;;)
    {
        cap >> frame; // get a new frame from camera
    	assert(!frame.empty());
	split(frame, channels);
	frameGreyscale = channels[0];
/*
	std::cout << "width: " << cap.get(CV_CAP_PROP_FRAME_WIDTH)
		<< " height: " << cap.get(CV_CAP_PROP_FRAME_HEIGHT)
		<< " Frame Index: " << cap.get(CV_CAP_PROP_POS_FRAMES)
		<< " FPS: " << cap.get(CV_CAP_PROP_FPS) << "\n";
*/
	vo_->addImage(frameGreyscale, 0.01*frameCounter);

        if(vo_->lastFrame() != NULL) {
         	std::cout << "Frame-Id: " << vo_->lastFrame()->id_ << " \t"
                << "#Features: " << vo_->lastNumObservations() << " \t"
                << "Proc. Time: " << vo_->lastProcessingTime()*1000 << "ms \n"
		<< "Pose: " << vo_->lastFrame()->T_f_w_ << "\n";

        }

       //imshow("edges", frameGreyscale);
       // if(waitKey(30) >= 0) break;
        frameCounter++;
    }
   return 0;
}

} // namespace svo


int main(int argc, char** argv)
{
  {
    svo::StdinAdaptor adaptor;
    //adaptor.runFromStdin();
    adaptor.streamFromStdin();
  }
  printf("Stdin Adaptor closed.\n");
  return 0;
}
