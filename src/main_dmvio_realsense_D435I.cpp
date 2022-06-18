/**
 * @file main_dmvio_realsense_D435i.cpp
 * @author Mario
 * @brief realsense d435i 的在线运行
 * @version 0.1
 * @date 2022-06-13
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <signal.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>
#include <vector>
#include <string>
#include <thread>


#include <condition_variable>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <librealsense2/rs.hpp>
#include "librealsense2/rsutil.h"

#include "IOWrapper/Output3DWrapper.h"
#include "IOWrapper/ImageDisplay.h"
#include <boost/thread.hpp>
#include "dso/util/settings.h"
#include "dso/util/globalFuncs.h"
#include "dso/util/D435IReader.h"
#include "dso/util/globalCalib.h"
#include "dso/util/NumType.h"

#include "util/TimeMeasurement.h"

#include "FullSystem/FullSystem.h"
#include "OptimizationBackend/MatrixAccumulators.h"
#include "FullSystem/PixelSelector2.h"

#include <util/SettingsUtil.h>

#include "IOWrapper/Pangolin/PangolinDSOViewer.h"
#include "IOWrapper/OutputWrapper/SampleOutputWrapper.h"


using namespace dso;
using namespace std;

// 标定的参数
std::string vignette = "";
std::string gammaCalib = "";
std::string calib = "";
std::string imuCalibFile = "";

int image_idx = 0;
int mode = 0; // TODO:

dmvio::IMUCalibration imuCalibration;
dmvio::IMUSettings imuSettings;

void settingsDefault(int preset)
{
    printf("\n=============== PRESET Settings: ===============\n");
    if(preset == 0 || preset == 1)
    {
        printf("DEFAULT settings:\n"
               "- %s real-time enforcing\n"
               "- 2000 active points\n"
               "- 5-7 active frames\n"
               "- 1-6 LM iteration each KF\n"
               "- original image resolution\n", preset == 0 ? "no " : "1x");

        preset == 1;
        setting_desiredImmatureDensity = 1500;
        // setting_desiredPointDensity = 2000;
        setting_desiredPointDensity = 1000;
        setting_minFrames = 5;
        setting_maxFrames = 7;
        setting_maxOptIterations = 6;
        setting_minOptIterations = 1;

        setting_logStuff = true;
    }

    if(preset == 2 || preset == 3)
    {
        printf("FAST settings:\n"
               "- %s real-time enforcing\n"
               "- 800 active points\n"
               "- 4-6 active frames\n"
               "- 1-4 LM iteration each KF\n"
               "- 424 x 320 image resolution\n", preset == 0 ? "no " : "5x");

        preset == 3;
        setting_desiredImmatureDensity = 600;
        setting_desiredPointDensity = 800;
        setting_minFrames = 4;
        setting_maxFrames = 6;
        setting_maxOptIterations = 4;
        setting_minOptIterations = 1;

        benchmarkSetting_width = 424;
        benchmarkSetting_height = 320;

        setting_logStuff = true;
    }

    printf("==============================================\n");
}

void parseArgument(char* arg, dmvio::SettingsUtil& settingsUtil)
{
    int option;
    float foption;
    char buf[1000];

    // --------------------------------------------------
    // These are mostly the original DSO commandline arguments which also work for DM-VIO.
    // The DM-VIO specific settings can also be set with commandline arguments (and also with the yaml settings file)
    // and have been registered in the main function. See IMUSettings and its members for details.
    // --------------------------------------------------

    if(1 == sscanf(arg, "quiet=%d", &option))
    {
        if(option == 1)
        {
            setting_debugout_runquiet = true;
            printf("QUIET MODE, I'll shut up!\n");
        }
        return;
    }

    if(1 == sscanf(arg, "preset=%d", &option))
    {
        settingsDefault(option);
        return;
    }


    if(1 == sscanf(arg, "nolog=%d", &option))
    {
        if(option == 1)
        {
            setting_logStuff = true;
            printf("DISABLE LOGGING!\n");
        }
        return;
    }
    if(1 == sscanf(arg, "nogui=%d", &option))
    {
        if(option == 1)
        {
            disableAllDisplay = true;
            printf("NO GUI!\n");
        }
        return;
    }
    if(1 == sscanf(arg, "nomt=%d", &option))
    {
        if(option == 1)
        {
            multiThreading = false;
            printf("NO MultiThreading!\n");
        }
        return;
    }
    if(1 == sscanf(arg, "calib=%s", buf))
    {
        calib = buf;
        printf("loading calibration from %s!\n", calib.c_str());
        return;
    }

    if(1 == sscanf(arg, "vignette=%s", buf))
    {
        vignette = buf;
        printf("loading vignette from %s!\n", vignette.c_str());
        return;
    }

    if(1 == sscanf(arg, "imuCalib=%s", buf))
    {
        imuCalibFile = buf;
        printf("Loading imu parameters from %s!\n", imuCalibFile.c_str());
        imuCalibration.loadFromFile(imuCalibFile);
        return;
    }

    if(1 == sscanf(arg, "useimu=%d", &option))
    {
        if(option == 0)
        {
            printf("Disabling IMU integration!\n");
            setting_useIMU = false;
        }else if(option == 1)
        {
            printf("Enabling IMU integration!\n");
            setting_useIMU = true;
        }
        return;
    }

    if(1 == sscanf(arg, "gamma=%s", buf))
    {
        gammaCalib = buf;
        printf("loading gammaCalib from %s!\n", gammaCalib.c_str());
        return;
    }

    if(1 == sscanf(arg, "save=%d", &option))
    {
        if(option == 1)
        {
            debugSaveImages = true;
            if(42 == system("rm -rf images_out"))
                printf("system call returned 42 - what are the odds?. This is only here to shut up the compiler.\n");
            if(42 == system("mkdir images_out"))
                printf("system call returned 42 - what are the odds?. This is only here to shut up the compiler.\n");
            if(42 == system("rm -rf images_out"))
                printf("system call returned 42 - what are the odds?. This is only here to shut up the compiler.\n");
            if(42 == system("mkdir images_out"))
                printf("system call returned 42 - what are the odds?. This is only here to shut up the compiler.\n");
            printf("SAVE IMAGES!\n");
        }
        return;
    }

    if(1 == sscanf(arg, "mode=%d", &option))
    {

        mode = option;
        if(option == 0)
        {
            printf("PHOTOMETRIC MODE WITH CALIBRATION!\n");
        }
        if(option == 1)
        {
            printf("PHOTOMETRIC MODE WITHOUT CALIBRATION!\n");
            setting_photometricCalibration = 0;
            setting_affineOptModeA = 0; //-1: fix. >=0: optimize (with prior, if > 0).
            setting_affineOptModeB = 0; //-1: fix. >=0: optimize (with prior, if > 0).
        }
        if(option == 2)
        {
            printf("PHOTOMETRIC MODE WITH PERFECT IMAGES!\n");
            setting_photometricCalibration = 0;
            setting_affineOptModeA = -1; //-1: fix. >=0: optimize (with prior, if > 0).
            setting_affineOptModeB = -1; //-1: fix. >=0: optimize (with prior, if > 0).
            setting_minGradHistAdd = 3;
        }
        return;
    }

    if(1 == sscanf(arg, "settingsFile=%s", buf))
    {
        YAML::Node settings = YAML::LoadFile(buf);
        settingsUtil.tryReadFromYaml(settings);
        printf("Loading settings from yaml file: %s!\n", imuCalibFile.c_str());
        return;
    }

    if(settingsUtil.tryReadFromCommandLine(arg))
    {
        return;
    }

    printf("could not parse argument \"%s\"!!!!\n", arg);
    assert(0);
}

void my_exit_handler(int s)
{
    printf("Caught signal %d\n", s);
    exit(1);
}

void exitThread()
{
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = my_exit_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    while(true) pause();
}

void run(IOWrap::PangolinDSOViewer* viewer)
{

    rs2::pipeline pipe; // 需要在主线程中保留
    D435IReader* reader = new D435IReader(calib, gammaCalib, vignette);
    reader->Initialize(pipe);
    reader->setGlobalCalibration();

    FullSystem* fullSystem = new FullSystem(false, imuCalibration, imuSettings);

    if(viewer != 0)
    {
        fullSystem->outputWrapper.push_back(viewer);
    }

    struct timeval tv_start;
    gettimeofday(&tv_start, NULL);
    clock_t started = clock();

    while(viewer != nullptr && !viewer->shouldQuit())
    {
        int image_idx = 0;
        if(!fullSystem->initialized)
        {
            gettimeofday(&tv_start, NULL);
            started = clock();
        }
        std::unique_ptr<dmvio::IMUData> imuData = std::make_unique<dmvio::IMUData>();
        ImageAndExposure* img = reader->getImageIMU(image_idx, imuData);
        if(imuData->size() != 0){
            fullSystem->addActiveFrame(img, image_idx, imuData.get(), 0);
            image_idx++;
        }
        std::cout << std::fixed << "Get Input Data and Sent it to system" << img->exposure_time << ", " << img->timestamp<< ", imu size: " << imuData->size() << std::endl;

        delete img;

        if(fullSystem->initFailed || setting_fullResetRequested)
        {
            if(image_idx < 250 || setting_fullResetRequested)
            {
                printf("RESETTING!\n");
                std::vector<IOWrap::Output3DWrapper*> wraps = fullSystem->outputWrapper;
                delete fullSystem;
                for(IOWrap::Output3DWrapper* ow : wraps) ow->reset();

                fullSystem = new FullSystem(false, imuCalibration, imuSettings);
                fullSystem->outputWrapper = wraps;
                setting_fullResetRequested = false;
            }
        }

        if(fullSystem->isLost)
        {
            printf("LOST!!\n");
            break;
        }
    }

    std::cout << "User closed window -> Quit!" << std::endl;

    fullSystem->blockUntilMappingIsFinished();
    clock_t ended = clock();
    struct timeval tv_end;
    gettimeofday(&tv_end, NULL);

    for(IOWrap::Output3DWrapper* ow : fullSystem->outputWrapper)
    {
        ow->join();
    }

    printf("DELETE FULLSYSTEM!\n");
    delete fullSystem;

    printf("EXIT NOW!\n");
}

int main(int argc, char **argv) {

    auto settingsUtil = std::make_shared<dmvio::SettingsUtil>();

    // Create Settings files.
    imuSettings.registerArgs(*settingsUtil);
    imuCalibration.registerArgs(*settingsUtil); // 从setting中拿数据
    settingsUtil->registerArg("setting_minOptIterations", setting_minOptIterations);
    settingsUtil->registerArg("setting_maxOptIterations", setting_maxOptIterations);
    settingsUtil->registerArg("setting_minIdepth", setting_minIdepth);
    settingsUtil->registerArg("setting_solverMode", setting_solverMode);
    settingsUtil->registerArg("setting_weightZeroPriorDSOInitY", setting_weightZeroPriorDSOInitY);
    settingsUtil->registerArg("setting_weightZeroPriorDSOInitX", setting_weightZeroPriorDSOInitX);
    settingsUtil->registerArg("setting_forceNoKFTranslationThresh", setting_forceNoKFTranslationThresh);
    settingsUtil->registerArg("setting_minFramesBetweenKeyframes", setting_minFramesBetweenKeyframes);

    for(int i = 1; i < argc; i++)
        parseArgument(argv[i], *settingsUtil);

    // // Print settings to commandline and file.
    // std::cout << "Settings:\n";
    // settingsUtil->printAllSettings(std::cout);
    // {
    //     std::ofstream settingsStream;
    //     settingsStream.open(imuSettings.resultsPrefix + "usedSettingsdso.txt");
    //     settingsUtil->printAllSettings(settingsStream);
    // }

    // hook crtl+C.
    boost::thread exThread = boost::thread(exitThread);

    // Pangolin Viewer
    IOWrap::PangolinDSOViewer* viewer = new IOWrap::PangolinDSOViewer(640, 480, false, settingsUtil);
    boost::thread runThread = boost::thread(boost::bind(run, viewer));
    viewer->run();
    delete viewer;
    runThread.join();

    cout << "System shutdown!\n";
    return 0;
}
