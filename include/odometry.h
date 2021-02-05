#include "main.h"

class ImuOdom : okapi::TwoEncoderOdometry {
 public:

  ImuOdom(const TimeUtil &itimeUtil,
          const std::shared_ptr<ReadOnlyChassisModel> &imodel,
          const ChassisScales &ichassisScales,
          const std::shared_ptr<Logger> &ilogger = Logger::getDefaultLogger());
  
  void step() override;

  OdomState ImuOdomMathStep(const std::valarray<std::int32_t> &itickDiff,
                         const QTime &ideltaT, QAngle imuDiff);

  void loop();

  void setState(const OdomState &istate,
                const StateMode &imode = StateMode::FRAME_TRANSFORMATION) override;

  OdomState getState(const StateMode &imode = StateMode::FRAME_TRANSFORMATION) const override;

  QAngle newImu, imuDiff, lastImu;
};

extern ImuOdom imu_odom;