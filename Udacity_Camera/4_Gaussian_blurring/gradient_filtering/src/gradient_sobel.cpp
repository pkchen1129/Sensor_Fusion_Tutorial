#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;

void gradientSobel()
{
    // TODO: Based on the image gradients in both x and y, compute an image 
    // which contains the gradient magnitude according to the equation at the 
    // beginning of this section for every pixel position. Also, apply different 
    // levels of Gaussian blurring before applying the Sobel operator and compare the results.
    cv::Mat img;
    img = cv::imread("../images/img1gray.png");

    // create filter kernel
    float gauss_data[25] = {1, 4, 7, 4, 1,
                            4, 16, 26, 16, 4,
                            7, 26, 41, 26, 7,
                            4, 16, 26, 16, 4,
                            1, 4, 7, 4, 1};
    float sobel_data_x[9] = {-1, 0, 1,
                            -2, 0, 2,
                            -1, 0, 1};
    float sobel_data_y[9] = {-1, -2, -1,
                            0, 0, 0,
                            1, 2, 1};
    for(int i = 0 ; i < 25 ; i++){
        gauss_data[i] /= 273;
    }
    cv::Mat kernel = cv::Mat(5, 5, CV_32F, gauss_data);
    cv::Mat kernel_sobel_x = cv::Mat(3, 3, CV_32F, sobel_data_x);
    cv::Mat kernel_sobel_y = cv::Mat(3, 3, CV_32F, sobel_data_y);

    // apply filter
    cv::Mat result1;
    cv::Mat result2;
    cv::Mat result3;


    
    // Filter for Gaussian Blurring
    cv::filter2D(img, result1, -1, kernel, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);
    // Filter for Sobel operator
    cv::filter2D(result1, result2, -1 , kernel_sobel_x, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT); 
    cv::filter2D(result1, result3, -1 , kernel_sobel_y, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT); 
    
    string windowName1 = "Sobel Operator -x";
    string windowName2 = "Sobel Operator -y";

    cv::namedWindow(windowName1, 1); // create window
    cv::imshow(windowName1, result2);
    // cv::waitKey(0); // wait for keyboard input before continuing

    cv::namedWindow(windowName2, 1); // create window
    cv::imshow(windowName2, result3);
    cv::waitKey(0); // wait for keyboard input before continuing
}

int main()
{
    gradientSobel();
}