#include <string>
#include <memory>

class ImageMeasurer
{
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

