#include <math.h>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

/*
	Distorting and Undistorting an Image using Opencv 
	
	1. The Image is read using Opencv.
	2. A Distorted Map is created with the formula given in the link: https://docs.opencv.org/master/dc/dbb/tutorial_py_calibration.html
	3. Image is distorted using remap function
	4. Distortion coefficients are inverted and a undistortion map is created.
	5. Distorted Image is taken as input and is undistorted with the undistorted map.
*/

//Distort Image
void Distort(cv::Mat &K, cv::Mat &distortionCoef,
	double in_x, double in_y,
	/*out*/double &out_x, /*out*/double &out_y)
{
	double fx = K.at<double>(0, 0);
	double fy = K.at<double>(1, 1);
	double cx = K.at<double>(0, 2);
	double cy = K.at<double>(1, 2);
	double k1 = distortionCoef.at<double>(0, 0);
	double k2 = distortionCoef.at<double>(0, 1);
	double p1 = distortionCoef.at<double>(0, 2);
	double p2 = distortionCoef.at<double>(0
		, 3);
	double k3 = distortionCoef.at<double>(0, 4);
	
	// To relative coordinates <- this is the step you are missing.
	double x = (in_x - cx) / fx;
	double y = (in_y - cy) / fy;

	double r2 = x * x + y * y;
	double kc = (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
	
	// Radial distortion
	double xDistort = cx*(1 - kc) + in_x*kc;
	double yDistort = cy * (1 - kc) + in_y * kc;



	// Back to absolute coordinates.
	out_x = xDistort;
	out_y = yDistort;


}


/*
Distorting and Undistorting Image
input-> Input Image
dst -> Distorted Output
undistorted-> Undistorted Output
*/

void distortImage(cv::Mat& input, cv::Mat& dst, cv::Mat& undistorted)
{
	cv::Mat src = input.clone();
	CV_Assert(src.type() == CV_8UC1 || src.type() == CV_8UC3);

	int w = src.cols;
	int h = src.rows;

	dst = src.clone();
	cv::Mat mapx = cv::Mat(src.size(), CV_32FC1);
	cv::Mat mapy = cv::Mat(src.size(), CV_32FC1);
	//
	float xc = w/2;
	float yc = h/2;
	// Intrinsic parameters
	double fx = 1738.06409;
	double fy = 1736.96128;
	// Distortion coefficients
	double k1 = 0.2;
	double k2 = 0;
	double k3 = 0;
	double p1 = 0;
	double p2 = 0;
	cv::Mat cameraMatrix;
	cv::Mat distortionCoefficients;
	
	//Camera Matrix
	cameraMatrix = (cv::Mat1d(3, 3) << fx, 0.000000, xc, 0.000000, fy, yc, 0.000000, 0.000000, 1.000000);
	
	//Distortion Coefficients
	distortionCoefficients = (cv::Mat1d(1, 5) << k1, k2, p1, p2, k3);
	
	//Create distorted Map
	for (int y = 0; y < h; y++)
	{
		for (int x = 0; x < w; x++)
		{
			double o1 = 0, o2 = 0;
			Distort(cameraMatrix, distortionCoefficients,
				double(x), double(y),
				o1, o2);

			mapx.at<float>(y, x) = o1;
			mapy.at<float>(y, x) = o2;

		}

	}

	//Distort Image
	remap(src, dst, mapx, mapy, cv::INTER_LINEAR, cv::BORDER_CONSTANT);

	/*
		UNDISTORTION
	*/

	//Invert Distortion Coefficients
	distortionCoefficients = -distortionCoefficients;

	//Create Undistorted Map
	for (int y = 0; y < h; y++)
	{
		for (int x = 0; x < w; x++)
		{
			double o1 = 0, o2 = 0;
			Distort(cameraMatrix, distortionCoefficients,
				double(x), double(y),
				o1, o2);
			mapx.at<float>(y, x) = o1;
			mapy.at<float>(y, x) = o2;
		}

	}

	//Create Undistorted Image
	remap(dst, undistorted, mapx, mapy, cv::INTER_LINEAR, cv::BORDER_CONSTANT);

}

int main()
{
	cv::Mat input = cv::imread("grid.jpg", 0);

	if (!input.data || input.empty())
	{
		std::cout << "Problem loading image!!!" << std::endl;
		return 0;
	}
		

	cv::imshow("input", input);

	cv::Mat output1, output2;
	try {
  //Distort and Undistort
  
		distortImage(input, output1, output2);
   //Display Images
	cv::imshow("output1", output1);
	cv::imshow("output2", output2);
	
	//Wait for Key press
	cv::waitKey(0);
	}
	catch (...)
	{
		std::cout << "Failed to Distort Image"<<std::endl;
    
	}
	
//Destroy all Windows
	cv::destroyAllWindows();
	return 0;
}
