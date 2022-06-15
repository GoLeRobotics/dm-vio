/**
 * @file D435IReader.h
 * @author Mario
 * @brief 从D435I Realsense实时获取数据
 * @version 0.1
 * @date 2022-06-14
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include "util/settings.h"
#include "util/globalFuncs.h"
#include "util/globalCalib.h"
#include "util/Undistort.h"

#include "IMU/IMUTypes.h"
#include "IOWrapper/ImageRW.h"

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <condition_variable>

using namespace dso;

/**
 * @brief D435I 数据流的获取
 * 
 */
class D435IReader
{
public:
	D435IReader(std::string calibFile, std::string gammaFile, std::string vignetteFile):
        calibfile_(calibFile)
	{
		undistort_ = Undistort::getUndistorterForFile(calibFile, gammaFile, vignetteFile);
		width_img_=undistort_->getSize()[0];
		height_img_=undistort_->getSize()[1];

        // current_accel_timestamp_ = 0;
        image_timestamp_ = -1;
        image_ready_ = false;
        image_idx_ = -1;
        exposure_timestamp_ = 1;
        exposure_available_ = false;
        offset_ = 0;
        // prev_accel_timestamp_ = 0;
        pre_gyro_time_ = -1;
        std::cout << "D435I Reading CalibFile From:" << calibFile << ", " << width_img_ << ", " << height_img_ << std::endl;
	}

	~D435IReader()
	{
		delete undistort_;
	};

    /**
     * @brief realsense D435I 的初始化
     * 
     * @param pipe 
     * @return true 
     * @return false 
     */
    bool Initialize(rs2::pipeline& pipe){

        // realsense的回调函数
        auto realsense_callback = [&](const rs2::frame& frame)
        {
            std::unique_lock<std::mutex> lock(data_mutex_);

            if(rs2::frameset fs = frame.as<rs2::frameset>())
            {
                // 没遇到过，先注释了
                // double new_timestamp_image = fs.get_timestamp()*1e-3;
                // if(abs(image_timestamp_-new_timestamp_image)<0.001){
                //     std::cout << "Two frames with the same timeStamp!!!\n";
                //     return;
                // }

                rs2::video_frame color_frame = fs.get_infrared_frame();
                imCV_ = cv::Mat(cv::Size(width_img_, height_img_), CV_8U, (void*)(color_frame.get_data()), cv::Mat::AUTO_STEP);

                if(color_frame.supports_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE)){
                    exposure_available_ = true;
                    exposure_timestamp_ =
                      color_frame.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE) * 1e-6;
                }

                image_timestamp_ = fs.get_timestamp()*1e-3;
                image_ready_ = true;

                lock.unlock();
                cond_image_rec_.notify_all();
            }
            else if (rs2::motion_frame m_frame = frame.as<rs2::motion_frame>())
            {
                if (m_frame.get_profile().stream_name() == "Gyro")
                {
                    v_gyro_data_.push_back(m_frame.get_motion_data());
                    v_gyro_timestamp_.push_back((m_frame.get_timestamp()+offset_)*1e-3);
                    rs2_vector gyro_sample = m_frame.get_motion_data();
                    // std::cout << std::fixed << "+++++++++ Origin: Gyro:" << (m_frame.get_timestamp()+offset_)*1e-3 << ", " << gyro_sample.x << ", " << gyro_sample.y << ", " << gyro_sample.z << std::endl;
                }
                else if (m_frame.get_profile().stream_name() == "Accel")
                {
                    v_accel_data_.push_back(m_frame.get_motion_data());
                    v_accel_timestamp_.push_back((m_frame.get_timestamp()+offset_)*1e-3);
                    // std::cout << std::fixed << "+++++++++ Origin: Accel:" << v_accel_timestamp_.back()  << ", " << v_accel_data_.back().x << ", " << v_accel_data_.back().y << ", " << v_accel_data_.back().z << std::endl;
                }
            }
        };
    
        rs2::context ctx;
        rs2::device_list devices = ctx.query_devices();
        rs2::device selected_device;
        if (devices.size() == 0) {
            std::cerr << "No device connected, please connect a RealSense device" << std::endl;
            return 0;
        }
        else
            selected_device = devices[0];
        std::vector<rs2::sensor> sensors = selected_device.query_sensors();
        int index = 0;

        // 1 : Stereo Module
        // 2 : RGB Camera
        // 3 : Motion Module
        for (rs2::sensor sensor : sensors)
            if (sensor.supports(RS2_CAMERA_INFO_NAME)) {
                ++index;
                if (index == 1) {
                    sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
                    sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0); // switch off emitter
                }
                std::cout << "  " << index << " : " << sensor.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
                // get_sensor_option(sensor); // print detailed sensor option
                if (index == 3){
                    sensor.set_option(RS2_OPTION_ENABLE_MOTION_CORRECTION,0);
                }
            }

        // Declare RealSense pipeline, encapsulating the actual device and sensors
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30); // TODO:: 60hz
        cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
        cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

        rs2::pipeline_profile pipe_profile = pipe.start(cfg, realsense_callback);
        // rs2::pipeline_profile pipe_profile = pipe.start(cfg);
        rs2::stream_profile cam_stream = pipe_profile.get_stream(RS2_STREAM_INFRARED, 1);
        rs2::stream_profile imu_stream = pipe_profile.get_stream(RS2_STREAM_GYRO);
        float* Rbc = cam_stream.get_extrinsics_to(imu_stream).rotation;
        float* tbc = cam_stream.get_extrinsics_to(imu_stream).translation;
        std::cout << "Tbc = " << std::endl;
        for(int i = 0; i<3; i++){
            for(int j = 0; j<3; j++)
                std::cout << Rbc[i*3 + j] << ", ";
            std::cout << tbc[i] << "\n";
        }

        rs2_intrinsics intrinsics_cam = cam_stream.as<rs2::video_stream_profile>().get_intrinsics();
        std::cout << " width = " << intrinsics_cam.width << std::endl;
        std::cout << " height = " << intrinsics_cam.height << std::endl;
        std::cout << " fx = " << intrinsics_cam.fx << std::endl;
        std::cout << " fy = " << intrinsics_cam.fy << std::endl;
        std::cout << " cx = " << intrinsics_cam.ppx << std::endl;
        std::cout << " cy = " << intrinsics_cam.ppy << std::endl;
        std::cout << " height = " << intrinsics_cam.height << std::endl;
        std::cout << " width = " << intrinsics_cam.width << std::endl;
        std::cout << " Coeff = " << intrinsics_cam.coeffs[0] << ", " << intrinsics_cam.coeffs[1] << ", " <<
        intrinsics_cam.coeffs[2] << ", " << intrinsics_cam.coeffs[3] << ", " << intrinsics_cam.coeffs[4] << ", " << std::endl;
        std::cout << " Model = " << intrinsics_cam.model << std::endl;

        Eigen::Matrix3f K;
        K << intrinsics_cam.fx, 0, intrinsics_cam.ppx, 0, intrinsics_cam.fy, intrinsics_cam.ppy, 0, 0, 1;
	    setGlobalCalib(intrinsics_cam.width, intrinsics_cam.height, K);
        std::cout << "Finish Initialization" << std::endl;
        return true;
    }

	void getCalibMono(Eigen::Matrix3f &K, int &w, int &h)
	{
		K = undistort_->getK().cast<float>();
		w = undistort_->getSize()[0];
		h = undistort_->getSize()[1];
	}

	void setGlobalCalibration()
	{
		int w_out, h_out;
		Eigen::Matrix3f K;
		getCalibMono(K, w_out, h_out);
		setGlobalCalib(w_out, h_out, K);
	}
    
    /**
     * @brief 从相机获得image与IMU数据
     * 
     * @param idx 
     * @param imuData 
     * @return ImageAndExposure* 
     */
	ImageAndExposure* getImageIMU(int& idx, std::unique_ptr<dmvio::IMUData>& imuData)
	{

        std::unique_lock<std::mutex> lk(data_mutex_);
        if(!image_ready_)    // 等待image的数据
            cond_image_rec_.wait(lk);

        cv::Mat m = imCV_.clone();
        // cv::imwrite("/home/mario/tmp/infra.jpg", imCV_);

        assert(m.rows*m.cols!=0 && m.type() == CV_8U);

	    MinimalImageB* minimg = new MinimalImageB(m.cols, m.rows);
	    memcpy(minimg->data, m.data, m.rows*m.cols);
        ImageAndExposure* img = undistort_->undistort<unsigned char>(minimg, exposure_timestamp_, image_timestamp_);
        // ImageAndExposure* img = undistort_->undistort<unsigned char>(minimg);

        std::cout << "Current image timestamp is " << image_timestamp_ << std::endl;
        delete minimg;

        double old_accel_timestamp = -1;
        while(v_gyro_timestamp_.size() > 0 && v_accel_timestamp_.size() > 0 &&                                                  // 有值
            v_gyro_timestamp_.front() < image_timestamp_ &&                                                                     // 当前图片之前的数据，以gyro做基准
            v_accel_timestamp_.front() < v_gyro_timestamp_.front() && v_accel_timestamp_.back() > v_gyro_timestamp_.front()){   // accel有插值的timestamp数据

            rs2_vector gyro_data = v_gyro_data_.front();
            v_gyro_data_.pop_front();
            double gyro_time = v_gyro_timestamp_.front();
            v_gyro_timestamp_.pop_front();

            int acc_idx = 0;
            while(v_accel_timestamp_.at(acc_idx) < gyro_time)
                acc_idx++;
            rs2_vector interp_accel = interpolateMeasure(gyro_time, v_accel_data_.at(acc_idx), v_accel_timestamp_.at(acc_idx),  v_accel_data_.at(acc_idx-1), v_accel_timestamp_.at(acc_idx-1));

            Eigen::Vector3d accMeas, gyrMeas;
            accMeas << interp_accel.x, interp_accel.y, interp_accel.z;
            gyrMeas << gyro_data.x, gyro_data.y, gyro_data.z;
            // std::cout << "++++++++++ Adding IMU data" << gyro_time << accMeas.transpose() << ", " << gyrMeas.transpose() << std::endl;
            double integrationTime = pre_gyro_time_ < 0 ? 0 : ((double) (gyro_time - pre_gyro_time_) * 1e-6);  // The timestamps are in nanoseconds
            imuData->push_back(dmvio::IMUMeasurement(accMeas, gyrMeas, integrationTime));
            pre_gyro_time_ = gyro_time;
            old_accel_timestamp = (acc_idx >=2 ? v_accel_timestamp_.at(acc_idx-2):-1);
        }

        while(v_accel_timestamp_.front() < old_accel_timestamp){
            v_accel_data_.pop_front();
            v_accel_timestamp_.pop_front();
        }

        // std::cout << "-----------------------" << std::endl;
        // for(auto v:v_accel_timestamp_)
        //     std::cout << v << std::endl;
        // std::cout << "-----------------------" << std::endl;

        image_ready_ = false;

        idx = ++image_idx_;
        std::cout << "Get image idx " << image_idx_ << std::endl;

        return img;
	}

private:

    /**
     * @brief 加速度计的插值
     * 
     * @param target_time 
     * @param current_data 
     * @param current_time 
     * @param prev_data 
     * @param prev_time 
     * @return rs2_vector 
     */
    rs2_vector interpolateMeasure(const double target_time, const rs2_vector current_data, const double current_time, const rs2_vector prev_data, const double prev_time)
    {
        // If there are not previous information, the current data is propagated
        if(prev_time == 0)
        {
            return current_data;
        }

        rs2_vector increment;
        rs2_vector value_interp;

        if(target_time > current_time) {
            value_interp = current_data;
        }
        else if(target_time > prev_time)
        {
            increment.x = current_data.x - prev_data.x;
            increment.y = current_data.y - prev_data.y;
            increment.z = current_data.z - prev_data.z;

            double factor = (target_time - prev_time) / (current_time - prev_time);

            value_interp.x = prev_data.x + increment.x * factor;
            value_interp.y = prev_data.y + increment.y * factor;
            value_interp.z = prev_data.z + increment.z * factor;

            // zero interpolation
            value_interp = current_data;
        }
        else {
            value_interp = prev_data;
        }

        return value_interp;
    }

    /**
     * @brief 获取相机的配置支持参数
     * 
     * @param sensor 
     * @return rs2_option 
     */
    static rs2_option get_sensor_option(const rs2::sensor& sensor)
    {
        std::cout << "Sensor supports the following options:\n" << std::endl;
        for (int i = 0; i < static_cast<int>(RS2_OPTION_COUNT); i++)
        {
            rs2_option option_type = static_cast<rs2_option>(i);
            std::cout << "  " << i << ": " << option_type;
            if (sensor.supports(option_type))
            {
                std::cout << std::endl;
                const char* description = sensor.get_option_description(option_type);
                std::cout << "       Description   : " << description << std::endl;
                float current_value = sensor.get_option(option_type);
                std::cout << "       Current Value : " << current_value << std::endl;
            }
            else
            {
                std::cout << " is not supported" << std::endl;
            }
        }
        uint32_t selected_sensor_option = 0;
        return static_cast<rs2_option>(selected_sensor_option);
    }

	Undistort* undistort_;                          ///< 去畸变, undistorter. [0] always exists, [1-2] only when MT is enabled.
	std::string calibfile_;                         ///< 标定的文件
    bool exposure_available_;                       ///< 设备是否支持曝光时间

    std::condition_variable cond_image_rec_;        ///< 图像数据的线程变量
    std::mutex data_mutex_;                         ///< IMU的线程锁

    cv::Mat imCV_;                                  ///< 当前图片
    double exposure_timestamp_;                     ///< 当前图片的曝光时间
    double image_timestamp_;                        ///< 当前的图片时间戳
    double pre_image_timestamp_;                    ///< 前一帧图片时间戳
    int image_idx_;                                 ///< 图片的ID数
    int width_img_;                                 ///< 当前图片的宽，640
    int height_img_;                                ///< 当前图片的高，480
    bool image_ready_;                              ///< 收到图片的flag
    double offset_;                                 ///< IMU与图像之间的offset

    // double prev_accel_timestamp_;                   ///< 之前加速度计的时间戳
    // double current_accel_timestamp_;                ///< 当前加速度计的时间戳
    // rs2_vector prev_accel_data_;                    ///< 之前的加速度数据
    // rs2_vector current_accel_data_;                 ///< 当前的加速度数据

    double pre_gyro_time_;
    std::deque<double> v_gyro_timestamp_;          ///< 存储陀螺仪的时间戳
    std::deque<double> v_accel_timestamp_;    ///< 存储加速度的时间戳
    std::deque<rs2_vector> v_gyro_data_;           ///< 存储陀螺仪的原始数据
    std::deque<rs2_vector> v_accel_data_;     ///< 存储加速度计的同步数据

};

