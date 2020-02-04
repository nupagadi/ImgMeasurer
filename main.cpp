#include <cstring>
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

    ImageMeasurer::Parameters Parameters {};

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

    auto paramProcessor = [&args](auto name, auto& member)
    {
        auto it = std::find(args.cbegin(), args.cend(), name);
        if (it != args.cend())
        {
            it = args.erase(it);
            if (it != args.cend())
            {
                member = static_cast<typename std::remove_reference<decltype(member)>::type>(
                        std::stof(*it));
                return true;
            }
        }
        return false;
    };

    auto& params = cfg.Parameters;
    paramProcessor("--CannyBlurKernel", params.CannyBlurKernel);
    paramProcessor("--CannyThres1", params.CannyThres1);
    paramProcessor("--CannyThres2", params.CannyThres2);
    paramProcessor("--HoughRho", params.HoughRho);
    paramProcessor("--HoughTheta", params.HoughTheta);
    paramProcessor("--HoughThres", params.HoughThres);
    paramProcessor("--HoughMinLineLength", params.HoughMinLineLength);
    paramProcessor("--HoughMaxLineGap", params.HoughMaxLineGap);
    paramProcessor("--LinesMinTangent", params.LinesMinTangent);
    paramProcessor("--LinesMaxTangent", params.LinesMaxTangent);

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
    std::cout << "./ImgMeasurer --file <path-to-file> --lane <lane-width> "
       << "--point1 <x> <y> --point2 <x> <y>" << " [--debug [gui]]"
       << " [OTHER OPTIONS]" << std::endl;
    std::cout << "./ImgMeasurer --help" << std::endl;
    std::cout << std::endl << "Examples:\n";
    std::cout << "./ImgMeasurer --file /path/to/file.png --lane 3.75 "
       << "--point1 800 560 --point2 940 1021" << std::endl;
    std::cout << "./ImgMeasurer --debug --file /path/to/file.png --lane 3.75 "
       << "--point1 800 560 --point2 940 1021" << std::endl;
    std::cout << "./ImgMeasurer --file /path/to/file.png --lane 3.75 "
       << "--point1 800 560 --point2 940 1021" << " --HoughRho 3" << std::endl;
    std::cout << std::endl << "Optional parameters:\n";
    std::cout << "--debug [gui]" << std::endl;
    std::cout << "Show additional info. \"gui\" - for graphical representation." << std::endl;
    std::cout << std::endl << "Methods custom parameters:\n";
    std::cout << "--CannyBlurKernel <value>" << std::endl;
    std::cout << "--CannyThres1 <value>" << std::endl;
    std::cout << "--CannyThres2 <value>" << std::endl;
    std::cout << "--HoughRho <value>" << std::endl;
    std::cout << "--HoughTheta <value>" << std::endl;
    std::cout << "--HoughThres <value>" << std::endl;
    std::cout << "--HoughMinLineLength <value>" << std::endl;
    std::cout << "--HoughMaxLineGap <value>" << std::endl;
    std::cout << "--LinesMinTangent <value>" << std::endl;
    std::cout << "--LinesMaxTangent <value>" << std::endl;
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

    ImageMeasurer::Parameters d {};
    if (aConfig.Mode == Config::Defaults)
    {
        d.Print();
        return;
    }

    std::cout << "Determine distance between ("
        << aConfig.Point1.first << ", " << aConfig.Point1.second << ") and ("
        << aConfig.Point2.first << ", " << aConfig.Point2.second << ")" << std::endl;
    std::cout << "Road pic: " << aConfig.FileName << std::endl;
    std::cout << "Lane width: " << aConfig.LaneWidth << " meters" << std::endl << std::endl;

    if (memcmp(&d, &aConfig.Parameters, sizeof(ImageMeasurer::Parameters)))
    {
        std::cout << "Parameters have been updated\n";
        aConfig.Parameters.Print();
    }
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
    default:
        ;
    }

    ImageMeasurer im;
    if (config.Mode == Config::Debug)
    {
        im.SetDebug(config.IsGuiDebug);
    }
    im.SetParameters(config.Parameters);

    float distance;
    try {
        distance = im.Calc(
                config.FileName, config.LaneWidth,
                config.Point1.first, config.Point1.second,
                config.Point2.first, config.Point2.second);
    }
    catch (std::exception)
    {
        std::cout << "runtime error" << std::endl;
        return -1;
    }

    std::cout << "The distance is " << distance << std::endl;

    return 0;
}

