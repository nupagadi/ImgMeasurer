#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>

using namespace cv;

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

    namedWindow("Display window", WINDOW_AUTOSIZE);
    imshow("Display window", image);
    waitKey(0);

    return 0;
}

