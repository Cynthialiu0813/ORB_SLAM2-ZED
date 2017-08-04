#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>
#include <iostream>
#include "System.h"

using namespace std;
using namespace sl;

void printHelp();
void printCameraInformation(sl::Camera &zed);

int main( int argc, char** argv )
{
    if (argc != 3)
    {
        cout<<"Usage: zed_run path_to_vocabulary path_to_config_file"<<endl;
        return 0;
    }

    ///////// Create a ZED camera //////////////////////////
    Camera zed;
    ///////// Set InitParameters  //////////////////////////
    InitParameters initParameters;
    initParameters.camera_resolution = sl::RESOLUTION_HD720;
    initParameters.depth_mode = sl::DEPTH_MODE_PERFORMANCE; //need quite a powerful graphic card in QUALITY
    RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = SENSING_MODE_STANDARD;

    ///////// Initialize and open the camera ///////////////
    ERROR_CODE err; // error state for all ZED SDK functions

    // Open the camera
    err = zed.open(initParameters);

    if (err != SUCCESS) {
        std::cout << errorCode2str(err) << std::endl;
        zed.close();
        return EXIT_FAILURE; // quit if an error occurred
    }

    // Print help in console
    printHelp();

    // Print camera information
    printCameraInformation(zed);

    cv::Mat depth;
    cv::Mat color;
    Mat left_image;
    Mat depth_image;
    //cv::Mat confidencemap(height, width, CV_8UC4);

    int confidence = 100;
    // init orb slam
    ORB_SLAM2::System orbslam( argv[1], argv[2], ORB_SLAM2::System::RGBD, true );

    cout<<"start to grab images, press q to quit."<<endl;
    double index = 0;
    char key = ' ';
    
    while (key != 'q') {
      
        zed.setConfidenceThreshold( confidence );
        err = zed.grab( runtime_parameters );
	
        if (err == SUCCESS)
        {
            zed.retrieveImage(left_image, VIEW_LEFT);
	    zed.retrieveMeasure(depth_image, sl::MEASURE_DEPTH);
	    color = cv::Mat(left_image.getHeight(), left_image.getWidth(), CV_8UC4, left_image.getPtr<sl::uchar1>(sl::MEM_CPU));
	    depth = cv::Mat(depth_image.getHeight(), depth_image.getWidth(), CV_8UC4, depth_image.getPtr<sl::uchar1>(sl::MEM_CPU));

            orbslam.TrackRGBD( color, depth, index );

        }

        key = cv::waitKey(5);
        index += 0.1;
    }

    orbslam.Shutdown();
    zed.close();
    return 0;
}

/**
 *  This function displays ZED camera information
 **/
void printCameraInformation(sl::Camera &zed) {
    printf("ZED Serial Number         : %d\n", zed.getCameraInformation().serial_number);
    printf("ZED Firmware              : %d\n", zed.getCameraInformation().firmware_version);
    printf("ZED Camera Resolution     : %dx%d\n", zed.getResolution().width, zed.getResolution().height);
    printf("ZED Camera FPS            : %d\n", (int) zed.getCameraFPS());
}

/**
 *  This function displays help
 **/
void printHelp() {
    cout << endl;
    cout << endl;
    cout << "Camera controls hotkeys: " << endl;
    cout << "  Increase camera settings value:            '+'" << endl;
    cout << "  Decrease camera settings value:            '-'" << endl;
    cout << "  Switch camera settings:                    's'" << endl;
    cout << "  Reset all parameters:                      'r'" << endl;
    cout << endl;
    cout << "Exit : 'q'" << endl;
    cout << endl;
    cout << endl;
    cout << endl;
}
