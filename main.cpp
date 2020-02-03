#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>
#include <vector>

using namespace cv;

bool LinesCrossPoint(const Vec4i& aLine1, const Vec4i& aLine2, Point* aCross)
{
    Point2f x  { (float)(aLine2[0] - aLine1[0]), (float)(aLine2[1] - aLine1[1]) };
    Point2f d1 { (float)(aLine1[2] - aLine1[0]), (float)(aLine1[3] - aLine1[1]) };
    Point2f d2 { (float)(aLine2[2] - aLine2[0]), (float)(aLine2[3] - aLine2[1]) };

    auto cr = d1.x*d2.y - d2.x*d1.y;
    if (!cr)
        return false;

    double t1 = (x.x*d2.y - x.y*d2.x) / cr;
    aCross->x = aLine1[0] + d1.x*t1;
    aCross->y = aLine1[1] + d1.y*t1;
    return true;
}

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

    // No deep copy here.
    return roi;
}

Mat BuildEdges(Mat& aOrig, int aBlurKernel, int aCannyThres1, int aCannyThres2)
{
    cvtColor(aOrig, aOrig, COLOR_BGR2GRAY);
    Mat edges;
    blur(aOrig, edges, Size(aBlurKernel, aBlurKernel));
    Canny(edges, edges, aCannyThres1, aCannyThres2);

    auto withEdges = Mat(aOrig.size(), aOrig.type(), Scalar::all(0));
    aOrig.copyTo(withEdges, edges);

    return withEdges;
}

std::vector<Vec4i> ExcludeLinesByTangent(const std::vector<Vec4i>& aHoughLines, float aMinTg, float aMaxTg, Mat* aLinesOnly)
{
    std::vector<Vec4i> result;

    for (size_t i = 0; i < aHoughLines.size(); i++)
    {
        double a = (aHoughLines[i][1] - aHoughLines[i][3]),
               b = (aHoughLines[i][0] - aHoughLines[i][2]);

        std::cout << "tangent " << a/b << std::endl;
        if (a-b == a || a-b == -b)
            continue;
        if (fabs(a/b) < aMinTg || aMaxTg < fabs(a/b))
            continue;
        result.push_back(aHoughLines[i]);

        if (aLinesOnly)
        {
            auto& hl = aHoughLines[i];
            line(*aLinesOnly, Point(hl[0], hl[1]), Point(hl[2], hl[3]),
                Scalar(0,0,255), 3, LINE_AA);
        }
    }

    return std::move(result);
}

std::pair<int,int> FindRoadLines(const std::vector<Vec4i>& aLines, int aWidht, int aHeight)
{
    std::vector<int> bottomCrosses;
    const Vec4i bottomLine {0, aHeight, aWidht, aHeight};
    for (const auto& l : aLines)
    {
        Point cross;
        if (!LinesCrossPoint(l, bottomLine, &cross))
            throw std::runtime_error("LinesCrossPoint: lines are parallel.");
        bottomCrosses.push_back(cross.x);
        std::cout << "bottom cross " << cross << std::endl;
    }

    auto comp = [hw = aWidht/2](auto lh, auto rh)
    {
        if (lh > hw && rh < hw || lh < hw && rh > hw)
            return lh > rh;
        return lh < rh;
    };
    auto it1 = std::max_element(bottomCrosses.cbegin(), bottomCrosses.cend(), comp);
    auto li = it1 - bottomCrosses.cbegin();
    auto it2 = std::min_element(bottomCrosses.cbegin(), bottomCrosses.cend(), comp);
    auto ri = it2 - bottomCrosses.cbegin();

    if (it1 == bottomCrosses.cend() || it2 == bottomCrosses.cend()
        || bottomCrosses[li] > aWidht/2 || bottomCrosses[ri] < aWidht/2)
    {
        return {-1, -1};
    }
    return {li, ri};
}

float CalcDistance(const Mat& aHomography,
        const Point2f& aBase1, const Point2f& aBase2, float aLaneWidth,
        const Point2f& aPoint1, const Point2f& aPoint2)
{
    std::vector<Point2f> src, dst(4);
    src.push_back(aBase1);
    src.push_back(aBase2);
    src.push_back(aPoint1);
    src.push_back(aPoint2);

    perspectiveTransform(src, dst, aHomography);

    for (auto p : dst)
        std::cout << p << std::endl;

    return fabs(dst[2].x - dst[3].x) / fabs(dst[0].x - dst[1].x) * aLaneWidth;
}

struct Config
{
    enum Mode {Error, Normal, Debug, Help};

    Config::Mode Mode;
    std::string FileName;
    float LaneWidth {}
    std::pair<int,int> Point1, Point2;

    bool IsGuiDebug = false;

    operator bool() const
    {
        return Mode != Error;
    }
};

Config MakeConfig(const std::vector<std::string>& aArgs)
{
    auto args = aArgs;
    Config cfg;
    cfg.Mode = Config::Error;

    auto it = std::find(args.cbegin(), args.cend(), "--help");
    if (it != args.cend())
    {
        cfg.Mode = Config::Help;
        return cfg;
    }

    it = std::find(args.cbegin(), args.cend(), "--debug");
    if (it != args.cend())
    {
        it = args.erase(it);
        cfg.Mode = Config::Debug;
        if (it != args.cend() && *it == "gui")
        {
            it = args.erase(it);
            cfg.IsGuiDebug = true;
        }
    }

    if (args.size() < 10)
        return cfg;

    it = std::find(args.cbegin(), args.cend(), "--file");
    if (it == args.cend() || ++it == args.cend())
        return cfg;
    cfg.FileName = *it;

    it = std::find(args.cbegin(), args.cend(), "--lane");
    if (it == args.cend() || ++it == args.cend())
        return cfg;
    cfg.LaneWidth = std::stof(*it);

    it = std::find(args.cbegin(), args.cend(), "--point1");
    if (it == args.cend() || it+2 == args.cend())
        return cfg;
    cfg.Point1.first = std::stoi(*++it);
    cfg.Point1.second = std::stoi(*++it);

    it = std::find(args.cbegin(), args.cend(), "--point2");
    if (it == args.cend() || it+2 == args.cend())
        return cfg;
    cfg.Point2.first = std::stoi(*++it);
    cfg.Point2.second = std::stoi(*++it);

    cfg.Mode = Config::Normal;

    return cfg;
}

void PrintHelp()
{
    std::cout << "Usage:\n";
    std::cout << "./ImgMeasurer --file /path/to/file.png --lane 3.75 "
       << "--point1 <x> <y> --point2 <x> <y>" << std::endl;
    std::cout << std::endl << "Optional parameters:\n";
    std::cout << "--debug [gui]" << std::endl;
    std::cout << "Show additional info. \"gui\" - for graphical representation." << std::endl;
}

void PrintConfig(const Config& aConfig)
{
    if (!aConfig)
    {
        std::cout << "Invalid config" << std::endl;
        PrintHelp();
        return;
    }
    if (aConfig.Mode == Config::Help)
    {
        PrintHelp();
        return;
    }

    std::cout << "Determine distance between ("
        << aConfig.Point1.first << ", " << aConfig.Point1.second << ") and ("
        << aConfig.Point2.first << ", " << aConfig.Point2.second << ")" << std::endl;
    std::cout << "Road pic: " << aConfig.FileName << std::endl;
    std::cout << "Lane width: " << aConfig.LaneWidth << " meters" << std::endl << std::endl;
}

int main(int argc, char** argv)
{
    std::vector<std::string> args(argv+1, argv+argc);
    auto config = MakeConfig(args);
    PrintConfig(config);
    if (!config)
    {
        return -1;
    }
    if (config.Mode == Config::Help)
    {
        return 0;
    }

    Mat image;
    try {
        image = imread(samples::findFile(config.FileName), IMREAD_COLOR);
    }
    catch (cv::Exception e)
    {
        std::cout << e.what() << std::endl;
    }
    if (image.empty())
    {
        std::cout << "Could not open or find the image" << std::endl;
        return -1;
    }

    auto roi = MakeRoi(image);
    auto withEdges = BuildEdges(roi, 3, 66, 150);

    namedWindow("Edges", WINDOW_NORMAL);
    imshow("Edges", withEdges);
    waitKey(0);

    std::vector<Vec4i> houghLines;
    HoughLinesP(withEdges, houghLines, 5, 1*CV_PI/180, 100, 300, 100);
    std::cout << "hough lines.size() " << houghLines.size() << std::endl;

    auto linesOnly = Mat(image.size(), image.type(), Scalar::all(0));
    auto lines = ExcludeLinesByTangent(houghLines, 1, 3, &linesOnly);

    int li, ri;
    auto w = image.size().width, h = image.size().height;
    std::tie(li, ri) = FindRoadLines(lines, w, h);
    if (li == -1 || ri == -1)
    {
        std::cout << "Can't find road lines." << std::endl;
        return -1;
    }

    namedWindow("Lines", WINDOW_NORMAL);
    imshow("Lines", linesOnly);
    waitKey(0);

    const Vec4i line34 {0, (int)(h*0.75), w, (int)(h*0.75)};
    Point l34cross, r34cross;
    LinesCrossPoint(lines[li], line34, &l34cross);
    LinesCrossPoint(lines[ri], line34, &r34cross);

    const Vec4i bottomLine {0, h, w, h};
    Point lBottomCross, rBottomCross;
    LinesCrossPoint(lines[li], bottomLine, &lBottomCross);
    LinesCrossPoint(lines[ri], bottomLine, &rBottomCross);

    std::vector<Point2f> ptsSrc, ptsDst;

    ptsSrc.emplace_back(lBottomCross.x, h);
    ptsSrc.emplace_back(l34cross.x, l34cross.y);
    ptsDst.emplace_back(l34cross.x, h);
    ptsDst.emplace_back(l34cross.x, l34cross.y);

    ptsSrc.emplace_back(rBottomCross.x, h);
    ptsSrc.emplace_back(r34cross.x, r34cross.y);
    ptsDst.emplace_back(r34cross.x, h);
    ptsDst.emplace_back(r34cross.x, r34cross.y);

    std::cout << "ptsSrc\n";
    for (auto e : ptsSrc)
        std::cout << " " << e;
    std::cout << std::endl;
    Mat homo = findHomography(ptsSrc, ptsDst);

    Point2f p1(config.Point1.first, config.Point1.second);
    Point2f p2(config.Point2.first, config.Point2.second);
    auto distance = CalcDistance(homo, ptsSrc[1], ptsSrc[3], config.LaneWidth, p1, p2);
    std::cout << "D " << distance << std::endl;

    Mat warped;
    warpPerspective(image, warped, homo, image.size());

    namedWindow("Warped", WINDOW_NORMAL);
    imshow("Warped", warped);
    waitKey(0);

    return 0;
}

