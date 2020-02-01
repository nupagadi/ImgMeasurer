#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <vector>

using namespace cv;

Mat MakeRoi(const Mat& aOrig)
{
    auto roi = Mat(aOrig.size(), aOrig.type(), Scalar::all(0));
    auto mask = Mat(aOrig.size(), CV_8UC1, Scalar(0));

    auto size = aOrig.size();
    std::vector<std::vector<Point>> points(1);
    points.front().emplace_back(0, size.height);
    points.front().emplace_back(size.width/2, size.height/2);
    points.front().emplace_back(size.width, size.height);

    drawContours(mask, points, -1, Scalar(255), cv::FILLED, 8);
    aOrig.copyTo(roi, mask);

    return std::move(roi);
}

int main(int argc, char** argv)
{
    String imageName("/home/ilya/Pic/road1.png");
    if (argc > 1)
    {
        imageName = argv[1];
    }

    Mat image;
    image = imread(samples::findFile(imageName), IMREAD_COLOR);
    if (image.empty())
    {
        std::cout << "Could not open or find the image" << std::endl;
        return -1;
    }

    //auto matSize = image.size();
    //std::cout << "w " << matSize.width << " h " << matSize.height << std::endl;

    auto roi = MakeRoi(image);

    namedWindow("Display window", WINDOW_NORMAL);
    imshow("Display window", roi);
    waitKey(0);

    return 0;
}

