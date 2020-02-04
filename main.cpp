#include <iostream>
#include <vector>
#include <algorithm>

#include "ImageMeasurer.h"

struct Config
{
    enum Mode {Error, Normal, Debug, Help, Defaults};

    Config::Mode Mode;
    std::string FileName;
    float LaneWidth {};
    std::pair<int,int> Point1, Point2;

    bool IsGuiDebug = false;

    int CannyBlurKernel {};
    int CannyThres1 {};
    int CannyThres2 {};
    int HoughRho {};
    int HoughTheta {};
    int HoughThres {};
    int HoughMinLineLength {};
    int HoughMaxLineGap {};
    int LinesMinTangent {};
    int LinesMaxTangent {};

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

    it = std::find(args.cbegin(), args.cend(), "--defaults");
    if (it != args.cend())
    {
        cfg.Mode = Config::Defaults;
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

    auto intParamProcessor = [&args](auto name, auto& member)
    {
        auto it = std::find(args.cbegin(), args.cend(), name);
        if (it != args.cend())
        {
            it = args.erase(it);
            if (it != args.cend())
            {
                member = std::stoi(*it);
                return true;
            }
        }
        return false;
    };

    intParamProcessor("--CannyBlurKernel", cfg.CannyBlurKernel);
    intParamProcessor("--CannyThres1", cfg.CannyThres1);
    intParamProcessor("--CannyThres2", cfg.CannyThres2);
    intParamProcessor("--HoughRho", cfg.HoughRho);
    intParamProcessor("--HoughTheta", cfg.HoughTheta);
    intParamProcessor("--HoughThres", cfg.HoughThres);
    intParamProcessor("--HoughMinLineLength", cfg.HoughMinLineLength);
    intParamProcessor("--HoughMaxLineGap", cfg.HoughMaxLineGap);
    intParamProcessor("--LinesMinTangent", cfg.LinesMinTangent);
    intParamProcessor("--LinesMaxTangent", cfg.LinesMaxTangent);

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

    if (!cfg)
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

    if (aConfig.Mode == Config::Defaults)
    {
        ImageMeasurer::PrintDefaults();
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
    switch (config.Mode)
    {
    case Config::Error:
        return -1;
    case Config::Help:
    case Config::Defaults:
        return 0;
    }

    ImageMeasurer im;
    if (config.Mode == Config::Debug)
    {
        im.SetDebug(config.IsGuiDebug);
    }

    auto distance = im.Calc(
            config.FileName, config.LaneWidth,
            config.Point1.first, config.Point1.second,
            config.Point2.first, config.Point2.second);

    std::cout << "The distance is " << distance << std::endl;

    return 0;
}

