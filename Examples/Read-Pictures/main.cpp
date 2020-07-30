//
// Created by sph on 2020/7/26.
//

#include <opencv2/opencv.hpp>

int main()
{
    cv::Mat image, image_gray;
    image = cv::imread("/home/sph/Downloads/dataset-room4_512_16/mav0/cam0/data/1520531124150444163.png", cv::IMREAD_COLOR);
    cv::imshow("lenna", image);
    cv::cvtColor(image, image_gray, cv::IMREAD_GRAYSCALE);
    cv::imwrite("lenna_gray.jpg", image_gray);
    cv::waitKey(0);
    return 0;
}
