#pragma once

#include <string>
#include <memory>

class ImageMeasurer
{
public:
    // Defaults.
    struct Parameters
    {
        int CannyBlurKernel = 3;
        int CannyThres1 = 66;
        int CannyThres2 = 150;

        float HoughRho = 5;
        float HoughTheta = 1;
        float HoughThres = 100;
        float HoughMinLineLength = 300;
        float HoughMaxLineGap = 100;

        float LinesMinTangent = 1;
        float LinesMaxTangent = 3;

        void Print() const;
    };

    ImageMeasurer();
    ~ImageMeasurer();

    float Calc(const std::string& aFileName, float aLaneWidth,
            int aPoint1x, int aPoint1y, int aPoint2x, int aPoint2y);

    void SetDebug(bool aIsGui);

    void SetParameters(const Parameters& aParams);

private:

    struct Impl;
    std::unique_ptr<Impl> mImpl;
};

