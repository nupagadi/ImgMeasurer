#include <string>
#include <memory>

struct ImageMeasurer
{
    // Defaults.
    struct Parameters
    {
        int CannyBlurKernel = 3;
        int CannyThres1 = 66;
        int CannyThres2 = 150;

        int HoughRho = 5;
        int HoughTheta = 1;
        int HoughThres = 100;
        int HoughMinLineLength = 300;
        int HoughMaxLineGap = 100;

        int LinesMinTangent = 1;
        int LinesMaxTangent = 3;

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

