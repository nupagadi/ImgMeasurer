#include <string>

class ImageMeasurer
{
public:

float Calc(const std::string& aFileName, float aLaneWidth,
        int aPoint1x, int aPoint1y, int aPoint2x, int aPoint2y);

void SetDebug(bool aIsGui);


private:

    bool mIsDebug = false;
    bool mIsGuiDebug = false;
};

