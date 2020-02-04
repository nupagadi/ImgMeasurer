#include <string>
#include <memory>

class ImageMeasurer
{
    // Defaults.
    static const int CannyBlurKernel = 3;
    static const int CannyThres1 = 66;
    static const int CannyThres2 = 150;

    static const int HoughRho = 5;
    static const int HoughTheta = 1;
    static const int HoughThres = 100;
    static const int HoughMinLineLength = 300;
    static const int HoughMaxLineGap = 100;

    static const int LinesMinTangent = 1;
    static const int LinesMaxTangent = 3;

public:

    ImageMeasurer();
    ~ImageMeasurer();

    float Calc(const std::string& aFileName, float aLaneWidth,
            int aPoint1x, int aPoint1y, int aPoint2x, int aPoint2y);

    void SetDebug(bool aIsGui);

private:

    struct Impl;
    std::unique_ptr<Impl> mImpl;
};

