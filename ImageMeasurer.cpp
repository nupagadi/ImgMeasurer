#include <iostream>
#include "ImageMeasurer.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;

struct ImageMeasurer::Impl
{
    bool mIsDebug = false;
    bool mIsGuiDebug = false;

    ImageMeasurer::Parameters Parameters {};

    static bool LinesCrossPoint(const Vec4i& aLine1, const Vec4i& aLine2, Point* aCross);

    Mat MakeRoi(const Mat& aOrig);

    Mat BuildEdges(Mat& aOrig);

    std::vector<Vec4i> ExcludeLinesByTangent(
            const std::vector<Vec4i>& aHoughLines, Mat* aLinesOnly);

    std::pair<int,int> FindRoadLines(const std::vector<Vec4i>& aLines, int aWidht, int aHeight);

    float CalcDistance(const Mat& aHomography,
            const Point2f& aBase1, const Point2f& aBase2, float aLaneWidth,
            const Point2f& aPoint1, const Point2f& aPoint2);
};

ImageMeasurer::ImageMeasurer()
    : mImpl(std::make_unique<ImageMeasurer::Impl>())
{
}

void ImageMeasurer::Parameters::Print() const
{
    std::cout << "\tCannyBlurKernel " << CannyBlurKernel << std::endl;
    std::cout << "\tCannyThres1 " << CannyThres1 << std::endl;
    std::cout << "\tCannyThres2 " << CannyThres2 << std::endl;
    std::cout << std::endl;
    std::cout << "\tHoughRho " << HoughRho << std::endl;
    std::cout << "\tHoughTheta " << HoughTheta << std::endl;
    std::cout << "\tHoughThres " << HoughThres << std::endl;
    std::cout << "\tHoughMinLineLength " << HoughMinLineLength << std::endl;
    std::cout << "\tHoughMaxLineGap " << HoughMaxLineGap << std::endl;
    std::cout << std::endl;
    std::cout << "\tLinesMinTangent " << LinesMinTangent << std::endl;
    std::cout << "\tLinesMaxTangent " << LinesMaxTangent << std::endl;
    std::cout << std::endl;
}

ImageMeasurer::~ImageMeasurer() = default;

void ImageMeasurer::SetParameters(const Parameters& aParams)
{
    mImpl->Parameters = aParams;
}

bool ImageMeasurer::Impl::LinesCrossPoint(const Vec4i& aLine1, const Vec4i& aLine2, Point* aCross)
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

Mat ImageMeasurer::Impl::MakeRoi(const Mat& aOrig)
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

Mat ImageMeasurer::Impl::BuildEdges(Mat& aOrig)
{
    cvtColor(aOrig, aOrig, COLOR_BGR2GRAY);
    Mat edges;
    blur(aOrig, edges, Size(Parameters.CannyBlurKernel, Parameters.CannyBlurKernel));
    Canny(edges, edges, Parameters.CannyThres1, Parameters.CannyThres2);

    auto withEdges = Mat(aOrig.size(), aOrig.type(), Scalar::all(0));
    aOrig.copyTo(withEdges, edges);

    return withEdges;
}

std::vector<Vec4i> ImageMeasurer::Impl::ExcludeLinesByTangent(
        const std::vector<Vec4i>& aHoughLines, Mat* aLinesOnly)
{
    std::vector<Vec4i> result;

    for (size_t i = 0; i < aHoughLines.size(); i++)
    {
        double a = (aHoughLines[i][1] - aHoughLines[i][3]),
               b = (aHoughLines[i][0] - aHoughLines[i][2]);

        if (mIsDebug)
            std::cout << "tangent " << a/b << std::endl;

        if (a-b == a || a-b == -b)
            continue;
        if (fabs(a/b) < Parameters.LinesMinTangent || Parameters.LinesMaxTangent < fabs(a/b))
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

std::pair<int,int> ImageMeasurer::Impl::FindRoadLines(
        const std::vector<Vec4i>& aLines, int aWidht, int aHeight)
{
    std::vector<int> bottomCrosses;
    const Vec4i bottomLine {0, aHeight, aWidht, aHeight};
    for (const auto& l : aLines)
    {
        Point cross;
        if (!LinesCrossPoint(l, bottomLine, &cross))
            throw std::runtime_error("LinesCrossPoint: lines are parallel.");
        bottomCrosses.push_back(cross.x);
        if (mIsDebug)
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

float ImageMeasurer::Impl::CalcDistance(const Mat& aHomography,
        const Point2f& aBase1, const Point2f& aBase2, float aLaneWidth,
        const Point2f& aPoint1, const Point2f& aPoint2)
{
    std::vector<Point2f> src, dst(4);
    src.push_back(aBase1);
    src.push_back(aBase2);
    src.push_back(aPoint1);
    src.push_back(aPoint2);

    perspectiveTransform(src, dst, aHomography);

    if (mIsDebug)
        for (auto p : dst)
            std::cout << p << std::endl;

    return fabs(dst[2].x - dst[3].x) / fabs(dst[0].x - dst[1].x) * aLaneWidth;
}

float ImageMeasurer::Calc(const std::string& aFileName, float aLaneWidth,
        int aPoint1x, int aPoint1y, int aPoint2x, int aPoint2y)
{
    Mat image;
    try {
        image = imread(samples::findFile(aFileName), IMREAD_COLOR);
    }
    catch (cv::Exception e)
    {
        std::cout << e.what() << std::endl;
    }
    if (image.empty())
    {
        std::cout << "Could not open or find the image" << std::endl;
        throw std::exception{};
    }

    auto roi = mImpl->MakeRoi(image);
    auto withEdges = mImpl->BuildEdges(roi);

    if (mImpl->mIsDebug && mImpl->mIsGuiDebug)
    {
        namedWindow("Edges", WINDOW_NORMAL);
        imshow("Edges", withEdges);
        waitKey(0);
    }

    const auto& params = mImpl->Parameters;
    std::vector<Vec4i> houghLines;
    HoughLinesP(withEdges, houghLines,
            params.HoughRho, params.HoughTheta * CV_PI/180,
            params.HoughThres, params.HoughMinLineLength, params.HoughMaxLineGap);
    if (mImpl->mIsDebug)
        std::cout << "hough lines.size() " << houghLines.size() << std::endl;

    auto linesOnly = Mat(image.size(), image.type(), Scalar::all(0));
    auto lines = mImpl->ExcludeLinesByTangent(houghLines, &linesOnly);

    int li, ri;
    auto w = image.size().width, h = image.size().height;
    std::tie(li, ri) = mImpl->FindRoadLines(lines, w, h);
    if (li == -1 || ri == -1)
    {
        std::cout << "Can't find road lines." << std::endl;
        throw std::exception{};
    }

    if (mImpl->mIsDebug && mImpl->mIsGuiDebug)
    {
        namedWindow("Lines", WINDOW_NORMAL);
        imshow("Lines", linesOnly);
        waitKey(0);
    }

    const Vec4i line34 {0, (int)(h*0.75), w, (int)(h*0.75)};
    Point l34cross, r34cross;
    Impl::LinesCrossPoint(lines[li], line34, &l34cross);
    Impl::LinesCrossPoint(lines[ri], line34, &r34cross);

    const Vec4i bottomLine {0, h, w, h};
    Point lBottomCross, rBottomCross;
    Impl::LinesCrossPoint(lines[li], bottomLine, &lBottomCross);
    Impl::LinesCrossPoint(lines[ri], bottomLine, &rBottomCross);

    std::vector<Point2f> ptsSrc, ptsDst;

    ptsSrc.emplace_back(lBottomCross.x, h);
    ptsSrc.emplace_back(l34cross.x, l34cross.y);
    ptsDst.emplace_back(l34cross.x, h);
    ptsDst.emplace_back(l34cross.x, l34cross.y);

    ptsSrc.emplace_back(rBottomCross.x, h);
    ptsSrc.emplace_back(r34cross.x, r34cross.y);
    ptsDst.emplace_back(r34cross.x, h);
    ptsDst.emplace_back(r34cross.x, r34cross.y);

    if (mImpl->mIsDebug)
    {
        std::cout << "ptsSrc\n";
        for (auto e : ptsSrc)
            std::cout << " " << e;
        std::cout << std::endl;
    }
    Mat homo = findHomography(ptsSrc, ptsDst);

    Point2f p1(aPoint1x, aPoint1y);
    Point2f p2(aPoint2x, aPoint2y);
    auto distance = mImpl->CalcDistance(homo, ptsSrc[1], ptsSrc[3], aLaneWidth, p1, p2);

    Mat warped;
    warpPerspective(image, warped, homo, image.size());

    if (mImpl->mIsDebug && mImpl->mIsGuiDebug)
    {
        namedWindow("Warped", WINDOW_NORMAL);
        imshow("Warped", warped);
        waitKey(0);
    }

    return distance;
}

void ImageMeasurer::SetDebug(bool aIsGui)
{
    mImpl->mIsDebug = true;
    mImpl->mIsGuiDebug = aIsGui;
}

