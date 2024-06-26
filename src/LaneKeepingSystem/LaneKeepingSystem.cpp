#include "LaneKeepingSystem/LaneKeepingSystem.hpp"

namespace Xycar {
template <typename PREC>
LaneKeepingSystem<PREC>::LaneKeepingSystem()
{
    std::string configPath;
    std::string calibrationPath;
    mNodeHandler.getParam("config_path", configPath);
    mNodeHandler.getParam("calibration_path", calibrationPath);
    YAML::Node config = YAML::LoadFile(configPath);
    YAML::Node calibration = YAML::LoadFile(calibrationPath);

    mPID = std::make_unique<PIDController<PREC>>(config["PID"]["P_GAIN"].as<PREC>(), config["PID"]["I_GAIN"].as<PREC>(), config["PID"]["D_GAIN"].as<PREC>());
    mStanley = std::make_unique<StanleyController<PREC>>(mStanleyGain, mStanleyLookAheadDistance);
    mMovingAverage = std::make_unique<MovingAverageFilter<PREC>>(config["MOVING_AVERAGE_FILTER"]["SAMPLE_SIZE"].as<uint32_t>());
    mImgPreProcessor = std::make_unique<IMGPreProcessor<PREC>>(config, calibration);
    mStopLineDetector = std::make_unique<StopLineDetector<PREC>>(config);
    mHoughTransformLaneDetector = std::make_unique<HoughTransformLaneDetector<PREC>>(config);
    setParams(config);

    mPublisher = mNodeHandler.advertise<xycar_msgs::xycar_motor>(mPublishingTopicName, mQueueSize);
    mSubscriber = mNodeHandler.subscribe(mSubscribedTopicName, mQueueSize, &LaneKeepingSystem::imageCallback, this);
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
    mStanleyGain = config["STANLEY"]["K_GAIN"].as<PREC>();
    mStanleyLookAheadDistance = config["STANLEY"]["LOOK_AHREAD_DISTANCE"].as<PREC>();
    mDebugging = config["DEBUG"].as<bool>();
}

template <typename PREC>
void LaneKeepingSystem<PREC>::run()
{
    // PI 다시 정리
    const PREC PI = std::atan(1)*4.0;
    ros::Rate rate(kFrameRate);
    // Define Steering Angle.
    PREC steeringMaxRadian = kXycarSteeringAngleLimit * (PI/180.f);
    std::cout << "max radian: " << steeringMaxRadian << std::endl;
    while (ros::ok())
    {
        ros::spinOnce();
        if (mFrame.empty())
            continue;

        mImgPreProcessor->preprocessImage(mFrame, mBlurredRoiImage, mEdgedRoiImage);

        // if (mStopLineDetector->detect(mBlurredRoiImage))
        // {
        //     finish();
        //     break;
        // }

        const auto [leftPosisionX, rightPositionX] = mHoughTransformLaneDetector->getLanePosition(mFrame, mEdgedRoiImage);

        mMovingAverage->addSample(static_cast<int32_t>((leftPosisionX + rightPositionX) / 2));

        // estimate position
        int32_t estimatedPositionX = static_cast<int32_t>(mMovingAverage->getResult());
        int32_t errorFromMid=estimatePositionX -static_cast<int32_t>(mframe.cols/2); // 여기서 ROI를 쓰는게 아니라 전체 FRAME을 씀.


        // Cross Track Error: 목표 위치와 현재 위치 사이의 거리
        float crossTrackError = static_cast<float>(errorFromMid);

        //TODO (1): Heading Error Calculation
        // 차선 중앙 경로 기울기 계산
        float x1_center = leftPositionX;
        float x2_center = rightPositionX;
        float y_center = mEdgedRoiImage.rows/2;
        // 화면 프레임 중앙값 좌표 계산.
        float slope_center = static_cast<float>(y_center) / (x2_center - x1_center);

        float angle_center=atan(slope_center);
        float headingAngle=angle_center;
        float headingAngleDegrees=headingAngle*(180.0f / M_PI);
        // 각도 계산

        mStanley->calculateSteeringAngle(crossTrackError, headingAngleDegrees, mXycarSpeed);

        // 조향각을 가져오거나 사용
        PREC steeringAngle = std::max(static_cast<PREC>(-kXycarSteeringAangleLimit), std::min(static_cast<PREC>(mStanley->getControlOutput()), static_cast<PREC>(kXycarSteeringAangleLimit)));

        // int32_t errorFromMid = estimatedPositionX - static_cast<int32_t>(mFrame.cols / 2);
        // PREC steeringAngle = std::max(static_cast<PREC>(-kXycarSteeringAangleLimit), std::min(static_cast<PREC>(mPID->getControlOutput(errorFromMid)), static_cast<PREC>(kXycarSteeringAangleLimit)));

        speedControl(steeringAngle);
        drive(steeringAngle);

        if (mDebugging)
        {
            std::cout << "imageWidth: " << imageWidth << ", imageHeight: " << imageHeight << ", imageCenterX: " << imageCenterX << ", A_y: " << A_y << ", B_y: " << B_y << ", CTE: " << crossTrackError << ", deltaY: " << deltaY << std::endl;
            // std::cout << "lpos: " << leftPosisionX << ", rpos: " << rightPositionX << ", mpos: " << estimatedPositionX << ", steeringAngle: " << steeringAngle << std::endl;
            mHoughTransformLaneDetector->drawRectangles(leftPosisionX, rightPositionX, estimatedPositionX);
            cv::imshow("Debug", mHoughTransformLaneDetector->getDebugFrame());
            cv::waitKey(1);
        }
    }
}

template <typename PREC>
void LaneKeepingSystem<PREC>::imageCallback(const sensor_msgs::Image& message)
{
    cv::Mat src = cv::Mat(message.height, message.width, CV_8UC3, const_cast<uint8_t*>(&message.data[0]), message.step);
    cv::cvtColor(src, mFrame, cv::COLOR_RGB2BGR);
}

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

template <typename PREC>
void LaneKeepingSystem<PREC>::drive(PREC steeringAngle)
{
    xycar_msgs::xycar_motor motorMessage;
    motorMessage.angle = std::round(steeringAngle);
    motorMessage.speed = std::round(mXycarSpeed);

    mPublisher.publish(motorMessage);
}

template <typename PREC>
void LaneKeepingSystem<PREC>::finish()
{
    ROS_INFO("STOP Detected - Starting Deceleration");
    PREC decelerationRate = 0.2; // 감속 비율 (예: 현재 속도의 20%만큼 감속)
    int decelerationInterval = 100; // 감속 간격 (ms)
    PREC minSpeed = static_cast<PREC>(0.1); // 이 속도 이하에서는 차량을 정지시킴

    while (mXycarSpeed > minSpeed && ros::ok())
    {
        mXycarSpeed *= (1 - decelerationRate); // 현재 속도에서 일정 비율 감소
        if (mXycarSpeed <= minSpeed)
        {
            mXycarSpeed = 0; // 최소 속도에 도달하면 완전히 정지
            ROS_INFO("Car Stopped");
        }

        // 차량 제어 메시지 업데이트 및 발행
        xycar_msgs::xycar_motor motorMessage;
        motorMessage.angle = 0; // 감속 중에는 핸들 각도를 0으로 유지
        motorMessage.speed = std::round(mXycarSpeed);
        mPublisher.publish(motorMessage);

        ros::Duration(decelerationInterval / 1000.0).sleep(); // 다음 감속까지 일정 시간 대기
    }
}

template class LaneKeepingSystem<float>;
template class LaneKeepingSystem<double>;
} // namespace Xycar