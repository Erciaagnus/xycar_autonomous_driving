// Copyright (C) 2023 Grepp CO.
// All rights reserved.

/**
 * @file LaneKeepingSystem.cpp
 * @author JeongHyeok Lim (lrrghdrh@naver.com)
 * @brief Lane Keeping System Class source file
 * @version 1.2
 * @date 2024-03-28
 */
#include "LaneKeepingSystem/LaneKeepingSystem.hpp"

namespace Xycar {
template <typename PREC>
LaneKeepingSystem<PREC>::LaneKeepingSystem()
{
    std::string configPath;
    mNodeHandler.getParam("config_path", configPath);
    YAML::Node config = YAML::LoadFile(configPath);

    mPID = std::make_unique<PIDController<PREC>>(config["PID"]["P_GAIN"].as<PREC>(), config["PID"]["I_GAIN"].as<PREC>(), config["PID"]["D_GAIN"].as<PREC>());
    mMovingAverage = std::make_unique<MovingAverageFilter<PREC>>(config["MOVING_AVERAGE_FILTER"]["SAMPLE_SIZE"].as<uint32_t>());
    mHoughTransformLaneDetector = std::make_unique<HoughTransformLaneDetector<PREC>>(config);
    mVehicleModel = std::make_unique<VehicleModel<PREC>>(0,0,0);
    setParams(config);

    mPublisher = mNodeHandler.advertise<xycar_msgs::xycar_motor>(mPublishingTopicName, mQueueSize);
    mSubscriber = mNodeHandler.subscribe(mSubscribedTopicName, mQueueSize, &LaneKeepingSystem::imageCallback, this);
    //Added Lidar Sensor... Subscriber
    //mLidarSubscriber = mNodeHandler.subscribe("/scan", 1, &LaneKeepingSystem::liDARCallback, this);

    // Added Publisher to publish vehicle State
    mVehicleStatePublisher = mNodeHandler.advertise<std_msgs::Float32MultiArray>("/vehicle_state", 1);
    // Added Publisher to Position of the vehicle
    mLanePositionPublisher = mNodeHandler.advertise<std_msgs::Float32MultiArray>("/lane_position", 1);

    // Additionally added STANLEY CONTROLLER and BINARY FILTER
    mStanley = std::make_unique<StanleyController<PREC>>(mStanleyGain, mStanleyLookAheadDistance);
    mBinaryFilter = std::make_unique<BinaryFilter<PREC>>(mStopSampleSize, mStopProbability);
}

template <typename PREC>
void LaneKeepingSystem<PREC>::setParams(const YAML::Node& config)
{
    mPublishingTopicName = config["TOPIC"]["PUB_NAME"].as<std::string>();
    mSubscribedTopicName = config["TOPIC"]["SUB_NAME"].as<std::string>();
    mQueueSize = config["TOPIC"]["QUEUE_SIZE"].as<uint32_t>();
    mXycarSpeed = config["XYCAR"]["START_SPEED"].as<PREC>();
    mXycarMaxSpeed = config["XYCAR"]["MAX_SPEED"].as<PREC>();
    mXycarMinSpeed = config["XYCAR"]["MIN_SPEED"].as<PREC>();
    mXycarSpeedControlThreshold = config["XYCAR"]["SPEED_CONTROL_THRESHOLD"].as<PREC>();
    mAccelerationStep = config["XYCAR"]["ACCELERATION_STEP"].as<PREC>();
    mDecelerationStep = config["XYCAR"]["DECELERATION_STEP"].as<PREC>();
    // Added Stanley Gain. (&& YOU SHOULD ADDED THIS!!!!!!!)
    mStanleyGain = config["STANLEY"]["K_GAIN"].as<PREC>();
    mStanleyLookAheadDistance = config["STANLEY"]["LOOK_AHREAD_DISTANCE"].as<PREC>();
    mDebugging = config["DEBUG"].as<bool>();

    mLinearUnit = config["XYCAR"]["LINEAR_UNIT"].as<PREC>();
    mAngleUnit = config["XYCAR"]["ANGLE_UNIT"].as<PREC>();
    mStopSampleSize = config["STOP"]["SAMPLE_SIZE"].as<int32_t>();
    mStopProbability = config["STOP"]["PROBABILITY"].as<PREC>();

    // Since we don't use Traffic signal.
    //mSignSignalSecond = config["DETECTION"]["SIGN_LIFESECOND"].as<PREC>();

    mAvoidanceInput = std::make_pair(config["AVOIDANCE_INPUT"]["POSITION"].as<PREC>(), config["AVOIDANCE_INPUT"]["SLOPE"].as<PREC>());
    mRotateInput = std::make_pair(config["ROTATE_INPUT"]["POSITION"].as<PREC>(), config["ROTATE_INPUT"]["SLOPE"].as<PREC>());
    mSignInput = std::make_pair(config["SIGN_INPUT"]["POSITION"].as<PREC>(), config["SIGN_INPUT"]["SLOPE"].as<PREC>());

    mRotateThreshold = config["XYCAR"]["ROTATE_THRESHOLD"].as<PREC>();
    mMinBoundingboxArea = config["DETECTION"]["MIN_BOUNDINGBOX_AREA"].as<PREC>();
}

template <typename PREC>
void LaneKeepingSystem<PREC>::run()
{
    const PREC PI = std::atan(1) * 4.0;
    ros::Rate rate(kFrameRate);
    ros::Time currentTime, previousTime, pubTime, stopTime;
    ros::Time now = ros::Time::now();
    PREC previousSteeringAngle = 0.f;
    PREC steeringMaxRadian = kXycarSteeringAangleLimit * (PI / 180.f);

    std::cout << "max radian: " << steeringMaxRadian << std::endl;
    bool enableStopDetected = true;
    bool previousStopDetected = false;
    bool stopDetected = false;
    bool setStopTimer = true;
    bool isStop = false;
    bool runUpdate = true;

    std::string detectedTrafficSignLabel = "IGNORE";
    std::string prevDetectedTrafficSignLabel = "IGNORE";
    Eigen::Vector2d inputVector;
    Eigen::Vector2d zeroVector;

    currentTime = now;
    previousTime = now;
    pubTime = now;
    stopTime = now;

    inputVector << 0.f, 0.f;

    while (ros::ok())
    {
        ros::spinOnce();

        if (mFrame.empty())
            continue;

        mHoughTransformLaneDetector->copyDebugFrame(mFrame);

        if (!mDetectTrafficSigns.empty())
        {
            const auto [boundingBoxArea, box] = mDetectTrafficSigns.top();
            const auto [trafficSignLabel, trafficSignTime] = box;

            detectedTrafficSignLabel = mDetectionLabel[trafficSignLabel];
        } else {
            detectedTrafficSignLabel = "IGNORE";
        }

        int32_t leftPositionX = 0;
        int32_t rightPositionX = 640;

        PREC steeringAngle = 0.f;



            std::cout << "PREV DETECTED: " << prevDetectedTrafficSignLabel << std::endl;
            std::cout << "DETECTED: " << detectedTrafficSignLabel << std::endl;
            std::cout << "RUN UPDATE: " << runUpdate << std::endl;
            // std::cout << "input vector: " << inputVector(0) << std::endl;
            mHoughTransformLaneDetector->predictLanePosition(inputVector);
            auto [predictLeftPositionX, predictRightPositionX] = mHoughTransformLaneDetector->getLanePosition(mFrame, runUpdate, detectedTrafficSignLabel);

            leftPositionX = predictLeftPositionX;
            rightPositionX = predictRightPositionX;

            currentTime = ros::Time::now();
            stopDetected = mHoughTransformLaneDetector->getStopLineStatus();

            mBinaryFilter->addSample(stopDetected);
            PREC stopProbability = mBinaryFilter->getResult();
            stopDetected = stopProbability > 0.5;

            ros::Duration delta_t = currentTime - previousTime;
            mVehicleModel->update(mXycarSpeed / mLinearUnit, previousSteeringAngle * (M_PI / 180.f), static_cast<double>(delta_t.toNSec()) / 1000000.f / 1000.f);

            int32_t estimatedPositionX = static_cast<int32_t>((leftPositionX + rightPositionX) / 2);
            int32_t errorFromMid = estimatedPositionX - static_cast<int32_t>(mFrame.cols / 2);
            mStanley->calculateSteeringAngle(errorFromMid, 0, mXycarSpeed);

            PREC stanleyResult = mStanley->getResult();
            steeringAngle = std::max(static_cast<PREC>(-kXycarSteeringAangleLimit), std::min(static_cast<PREC>(stanleyResult), static_cast<PREC>(kXycarSteeringAangleLimit)));

            // std::cout << "error: " << errorFromMid << std::endl;
        }

        std::cout << "position: " << leftPositionX << ", " << rightPositionX << std::endl;
        if (enableStopDetected && stopDetected && !previousStopDetected)
        {
            stopTime = currentTime;
            setStopTimer = false;
            isStop = true;
            enableStopDetected = false;
        }

        int32_t estimatedPositionX = static_cast<int32_t>((leftPositionX + rightPositionX) / 2);

        speedControl(steeringAngle);
        drive(steeringAngle);

        previousSteeringAngle = steeringAngle;
        previousStopDetected = stopDetected;
        prevDetectedTrafficSignLabel = detectedTrafficSignLabel;

        if (mDebugging)
        {
            std_msgs::Float32MultiArray vehicleStateMsg;

            std::tuple<PREC, PREC, PREC> vehicleState = mVehicleModel->getResult();

            vehicleStateMsg.data.push_back(std::get<0>(vehicleState));
            vehicleStateMsg.data.push_back(std::get<1>(vehicleState));
            vehicleStateMsg.data.push_back(std::get<2>(vehicleState));
            vehicleStateMsg.data.push_back(leftPositionX);
            vehicleStateMsg.data.push_back(rightPositionX);

            ros::Duration pubDiff = ros::Time::now() - pubTime;

            if (pubDiff.toSec() > 0.1)
            {
                mVehicleStatePublisher.publish(vehicleStateMsg);
                pubTime = ros::Time::now();
            }

            mHoughTransformLaneDetector->drawRectangles(leftPositionX, rightPositionX, estimatedPositionX);
            cv::imshow("Debug", mHoughTransformLaneDetector->getDebugFrame());
            cv::waitKey(1);
        }

        previousTime = ros::Time::now();
    }
}

// 이미지 콜백함수
template <typename PREC>
void LaneKeepingSystem<PREC>::imageCallback(const sensor_msgs::Image& message)
{
    cv::Mat src = cv::Mat(message.height, message.width, CV_8UC3, const_cast<uint8_t*>(&message.data[0]), message.step);
    cv::cvtColor(src, mFrame, cv::COLOR_RGB2BGR);
}


// SPEED 컨트롤 콜백 함수 ( using Steering Angle )
template <typename PREC>
void LaneKeepingSystem<PREC>::speedControl(PREC steeringAngle)
{
    if (std::abs(steeringAngle) > mXycarSpeedControlThreshold)
    {
        mXycarSpeed -= mDecelerationStep;
        mXycarSpeed = std::max(mXycarSpeed, mXycarMinSpeed);
        return;
    }

    mXycarSpeed += mAccelerationStep;
    mXycarSpeed = std::min(mXycarSpeed, mXycarMaxSpeed);
}


// STOP 함수
template <typename PREC>
void LaneKeepingSystem<PREC>::stop(PREC steeringAngle)
{
    xycar_msgs::xycar_motor motorMessage;

    motorMessage.header.stamp = ros::Time::now();
    motorMessage.angle = steeringAngle;
    motorMessage.speed = 0;

    mPublisher.publish(motorMessage);
}


// 드라이브 함수
template <typename PREC>
void LaneKeepingSystem<PREC>::drive(PREC steeringAngle)
{
    xycar_msgs::xycar_motor motorMessage;

    motorMessage.header.stamp = ros::Time::now();
    motorMessage.angle = std::round(steeringAngle);
    motorMessage.speed = std::round(mXycarSpeed);
    mPublisher.publish(motorMessage);
}

template class LaneKeepingSystem<float>;
template class LaneKeepingSystem<double>;
} // namespace Xycar
