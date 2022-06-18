/**
 * @file main_dmvio_d435i.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-06-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <thread>
#include <locale.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "IOWrapper/Output3DWrapper.h"
#include "IOWrapper/ImageDisplay.h"

#include "util/Undistort.h"


#include <boost/thread.hpp>
#include "dso/util/settings.h"
#include "dso/util/globalFuncs.h"
#include "dso/util/globalCalib.h"
#include "util/TimeMeasurement.h"

#include "dso/util/NumType.h"
#include "FullSystem/FullSystem.h"
#include "OptimizationBackend/MatrixAccumulators.h"
#include "FullSystem/PixelSelector2.h"

#include <util/SettingsUtil.h>

#include "IOWrapper/Pangolin/PangolinDSOViewer.h"
#include "IOWrapper/OutputWrapper/SampleOutputWrapper.h"

#include "live/RealsenseD435I.h"
#include "util/MainSettings.h"
#include "live/FrameSkippingStrategy.h"

#include <boost/filesystem.hpp>

// If mainSettings.calib is set we use this instead of the factory calibration.
std::string calibSavePath = "./factoryCalibrationT265Camera.txt"; // Factory calibration will be saved here.
std::string camchainSavePath = ""; // Factory camchain will be saved here if set.

int start = 2;


using namespace dso;

dmvio::FrameContainer frameContainer;
dmvio::MainSettings mainSettings;
dmvio::IMUCalibration imuCalibration;
dmvio::IMUSettings imuSettings;
dmvio::FrameSkippingSettings frameSkippingSettings;
std::unique_ptr<dmvio::DatasetSaver> datasetSaver;
std::string saveDatasetPath = "";

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


void run(IOWrap::PangolinDSOViewer* viewer, Undistort* undistorter)
{
    bool linearizeOperation = false;
    std::cout << "Initialize FullSystem" << std::endl;
    auto fullSystem = std::make_unique<FullSystem>(linearizeOperation, imuCalibration, imuSettings);
    std::cout << "Finish Initialize FullSystem" << std::endl;

    if(setting_photometricCalibration > 0 && undistorter->photometricUndist == nullptr)
    {
        printf("ERROR: dont't have photometric calibation. Need to use commandline options mode=1 or mode=2 ");
        exit(1);
    }

    if(undistorter->photometricUndist != nullptr)
    {
        fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
    }

    if(viewer != 0)
    {
        fullSystem->outputWrapper.push_back(viewer);
    }

    dmvio::FrameSkippingStrategy frameSkipping(frameSkippingSettings);
    // frameSkipping registers as an outputWrapper to get notified of changes of the system status.
    fullSystem->outputWrapper.push_back(&frameSkipping);

    int ii = 0;
    int lastResetIndex = 0;

    std::cout << "Reaching loop" << std::endl;
    while(true)
    {
        // Skip the first few frames if the start variable is set.
        std::cout << "  frameContainergetIframeContainermageAndIMUData " << start << std::endl;

        if(start > 0 && ii < start)
        {
            auto pair = frameContainer.getImageAndIMUData();
            std::cout << "Skipping" << std::endl;
            ++ii;
            continue;
        }


        std::cout << "getImageAndIMUData " << start << std::endl;
        auto pair = frameContainer.getImageAndIMUData(frameSkipping.getMaxSkipFrames(frameContainer.getQueueSize()));

        std::cout << "Adding image and imu to fullsystem " << std::endl;
        fullSystem->addActiveFrame(pair.first.get(), ii, &(pair.second), nullptr);

        if(fullSystem->initFailed || setting_fullResetRequested)
        {
            if(ii - lastResetIndex < 250 || setting_fullResetRequested)
            {
                printf("RESETTING!\n");
                std::vector<IOWrap::Output3DWrapper*> wraps = fullSystem->outputWrapper;
                std::cout << "/* fullSystem.reset(); */" << std::endl;
                fullSystem.reset();
                std::cout << "/* ow->reset() */" << std::endl;
                for(IOWrap::Output3DWrapper* ow : wraps) ow->reset();
                std::cout << "/* fullSystem = std::make_unique */" << std::endl;
                fullSystem = std::make_unique<FullSystem>(linearizeOperation, imuCalibration, imuSettings);
                std::cout << "Initialize New FullSystem" << std::endl;
                if(undistorter->photometricUndist != nullptr)
                {
                    fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
                }
                fullSystem->outputWrapper = wraps;

                setting_fullResetRequested = false;
                lastResetIndex = ii;
            }
        }

        if(viewer != nullptr && viewer->shouldQuit())
        {
            std::cout << "User closed window -> Quit!" << std::endl;
            break;
        }

        if(fullSystem->isLost)
        {
            printf("LOST!!\n");
            break;
        }

        ++ii;

    }

    fullSystem->blockUntilMappingIsFinished();

    fullSystem->printResult(imuSettings.resultsPrefix + "result.txt", false, false, true);

    dmvio::TimeMeasurement::saveResults(imuSettings.resultsPrefix + "timings.txt");

    for(IOWrap::Output3DWrapper* ow : fullSystem->outputWrapper)
    {
        ow->join();
    }

    printf("DELETE FULLSYSTEM!\n");
    fullSystem.reset();

    if(datasetSaver) datasetSaver->end();


    printf("EXIT NOW!\n");
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "C");

#ifdef DEBUG
    std::cout << "DEBUG MODE!" << std::endl;
#endif

    auto settingsUtil = std::make_shared<dmvio::SettingsUtil>();

    // Create Settings files.
    imuSettings.registerArgs(*settingsUtil);
    imuCalibration.registerArgs(*settingsUtil);
    mainSettings.registerArgs(*settingsUtil);
    frameSkippingSettings.registerArgs(*settingsUtil);

    settingsUtil->registerArg("start", start);
    settingsUtil->registerArg("calibSavePath", calibSavePath);
    settingsUtil->registerArg("camchainSavePath", camchainSavePath);
    settingsUtil->registerArg("saveDatasetPath", saveDatasetPath);

    auto normalizeCamSize = std::make_shared<double>(0.0);
    settingsUtil->registerArg("normalizeCamSize", *normalizeCamSize, 0.0, 5.0);

    // This call will parse all commandline arguments and potentially also read a settings yaml file if passed.
    mainSettings.parseArguments(argc, argv, *settingsUtil);

    // Print settings to commandline and file.
    std::cout << "Settings:\n";
    settingsUtil->printAllSettings(std::cout);
    {
        std::ofstream settingsStream;
        settingsStream.open(imuSettings.resultsPrefix + "usedSettingsdso.txt");
        settingsUtil->printAllSettings(settingsStream);
    }

    // hook crtl+C.
    boost::thread exThread = boost::thread(exitThread);

    if(saveDatasetPath != "")
    {
        std::cout << "Saving Data to " << saveDatasetPath << std::endl;
        try
        {
            datasetSaver = std::make_unique<dmvio::DatasetSaver>(saveDatasetPath);
        } catch(const boost::filesystem::filesystem_error& err)
        {
            std::cout << "ERROR: Cannot save dataset: " << err.what() << std::endl;
        }
    }

    std::cout << "Saving camera calibration to " << calibSavePath << "\n";
    dmvio::RealsenseD435I realsense(frameContainer, calibSavePath, datasetSaver.get());
    realsense.start();
    std::cout << "Successfully start realsense" << std::endl;

    std::string usedCalib = calibSavePath;
    if(mainSettings.calib != "")
    {
        usedCalib = mainSettings.calib;
        std::cout << "Using custom camera calibration (instead of factory calibration): " << mainSettings.calib << "\n";
    }

    if(camchainSavePath != "")
    {
        std::cout << "Saving T_cam_imu to: " << camchainSavePath << std::endl;
        realsense.imuCalibration->saveToFile(camchainSavePath);
    }

    std::unique_ptr<Undistort> undistorter(
            Undistort::getUndistorterForFile(usedCalib, mainSettings.gammaCalib, mainSettings.vignette));
    realsense.setUndistorter(undistorter.get());

    std::cout << "Undistorter size is " << (int) undistorter->getSize()[0] << ", " << (int) undistorter->getSize()[1] << std::endl;
    setGlobalCalib(
            (int) undistorter->getSize()[0],
            (int) undistorter->getSize()[1],
            undistorter->getK().cast<float>());

    if(mainSettings.imuCalibFile != "")
    {
        imuCalibration.loadFromFile(mainSettings.imuCalibFile);
    }else
    {
        std::cout << "Using factory IMU calibration!" << std::endl;
        imuCalibration = *(realsense.imuCalibration);
    }
    std::cout << "Finish All Settings" << std::endl;

    if(!disableAllDisplay)
    {
        std::cout << "Pangolin View size is " << wG[0] << hG[0] << std::endl;
        IOWrap::PangolinDSOViewer* viewer = new IOWrap::PangolinDSOViewer(wG[0], hG[0], false, settingsUtil);

        boost::thread runThread = boost::thread(boost::bind(run, viewer, undistorter.get()));

        viewer->run();

        delete viewer;

        // Make sure that the destructor of FullSystem, etc. finishes, so all log files are properly flushed.
        runThread.join();
    }else
    {
        run(0, undistorter.get());
    }

    return 0;
}
