#ifndef VRX_PATROL_AND_FOLOW_SCORING_PLUGIN_HH_
#define VRX_PATROL_AND_FOLOW_SCORING_PLUGIN_HH_

#include <gz/transport.hh>
#include "ScoringPlugin.hh"

#define PROCESS_SPEED_TASK_PERIOD 1.0
#define PROCESS_PUBLISH_TASK_PERIOD 1.0

using namespace gz;

enum PatrolAndFollowState
{
  Phase_Initial = 0,
  Phase_Rally = 1,
  Phase_Alert = 2,
  Phase_Alert_Follow = 3,
  Phase_Follow = 4,
  Phase_Task_Finished = 5
};

struct PatrolAndFollowDataStruct {
  std::chrono::duration<double> time = std::chrono::duration<double>::zero();
  PatrolAndFollowState phase = PatrolAndFollowState::Phase_Initial;
  math::Pose3d player = math::Pose3d::Zero;
  double playerSpeed = 0;
  double playerDistance = 0;
  math::Pose3d ennemy = math::Pose3d::Zero;
  double ennemySpeed = 0;
  double ennemyDistance = 0;
  std::vector<math::Pose3d> allies;
  std::vector<double> alliesSpeed;
  std::vector<double> alliesDistance;
  math::Vector3d buoy = math::Vector3d::Zero;
  double buoyDistance = 0;
  uint64_t collisions = 0;
  double lastAlertError = 0;
  uint alertRMSNumber = 0;
  double alertRMSErrorPow = 0;
  double alertRMSError = 0;
  math::Pose3d lastAlertLocalPose = math::Pose3d::Zero;
  math::Pose3d lastAlertGpsPose = math::Pose3d::Zero;
  double lastFollowError = 0;
  double followRMSNumber = 0;
  double followRMSErrorPow = 0;
  double followRMSError = 0;
};

namespace vrx
{
  /// \brief A plugin for computing the score of the patrol and folow task.
  ///  This plugin derives from the generic ScoringPlugin class. Refer to that
  /// plugin for an explanation of the four states defined (Initial, Ready,
  /// Running and Finished) as well as other required SDF elements.
  ///
  /// Patrol and folow task is a two part task. In the first part, the WAM-V
  /// must rally a patrol area as fast as possible. In the second part, the WAM-V must
  /// find and folow a moving boat.
  class PatrolAndFollowScoringPlugin : public ScoringPlugin
  {
    /// \brief Constructor.
    public: PatrolAndFollowScoringPlugin();

    /// \brief Destructor.
    public: ~PatrolAndFollowScoringPlugin() override = default;

    // Documentation inherited.
    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr) override;

    // Documentation inherited.
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &_ecm) override;

    // Documentation inherited.
    protected: void OnCollision() override;

    protected: void OnFinished() override;

    private: void OnRunning() override;

    // Log data csv
    private: 
    void OnLogDataCsv(PatrolAndFollowDataStruct& data);
    void OnSendTopicDebug(PatrolAndFollowDataStruct& data);
    void ProcessAlert(sim::EntityComponentManager &_ecm, PatrolAndFollowDataStruct& data);
    void ProcessFollow(sim::EntityComponentManager &_ecm, PatrolAndFollowDataStruct& data);

    /// \brief Private data pointer.
    GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };
}
#endif
