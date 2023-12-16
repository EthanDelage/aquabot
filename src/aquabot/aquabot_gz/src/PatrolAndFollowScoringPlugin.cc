#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/msgs/float.pb.h>
#include <gz/msgs/boolean.pb.h>

#include "PatrolAndFollowScoringPlugin.hh"
#include "WaypointMarkers.hh"

using namespace gz;
using namespace vrx;

/// \brief Private ScoringPlugin data class.
class PatrolAndFollowScoringPlugin::Implementation
{
  /// \brief The position of the pinger.
  public: math::Vector3d pingerPosition;

  /// \brief Name of the target used.
  public: std::string targetName;

  /// \brief Phase Rally : If the distance between the WAM-V and the pinger
  /// is within this tolerance we consider the task completed.
  public: double phaseRallyTolerance = 20;

  /// \brief Phase Alert : Period to process alert
  public: double phaseAlertProcessPeriod = 1.0; // 1 Hz

  /// \brief Phase Alert : Tolerance to consider alert
  public: double phaseAlertTolerance = 50.0;

  /// \brief Phase Follow : Period to process follow
  public: double phaseFollowProcessPeriod = 1.0; // 1 Hz

  /// \brief Phase Follow : Tolerance to start the follow
  public: double phaseFollowTolerance = 50;

  /// \brief Phase Follow : Target distance to ennemy
  public: double phaseFollowTargetDistance = 30;

  /// \brief Phase Follow : Time to follow the ennemy
  public: double phaseFollowEnnemyTime = 180.0; // 3 minutes

  /// \brief Minimum distance with collision objects.
  public: double collisionMinDistance = 10;

  /// \brief Finish if collision.
  public: bool collisionFinish = true;

  /// \brief Collision penality
  public: double collisionPenality = 10;

  /// \brief Others collisions objects
  public: std::vector<std::string> collisionsObjectsNames;

  /// \brief Ais allies period
  public: double aisAlliesPeriod = 1.0;

  /// \brief Ais ennemy period
  public: double aisEnnemyPeriod = 10.0;

  /// \brief Wait time before publishing ennemy ais
  public: double aisEnnemyWaitTime = 180.0; // 3 minutes


  /// \brief Transport node.
  public: transport::Node node{transport::NodeOptions()};

  /// \brief Entity of the vehicle used.
  public: sim::Entity playerEntity;

  /// \brief Entity of the target used.
  public: sim::Entity ennemyEntity;

  /// \brief List of allies entites
  public: std::vector<sim::Entity> alliesEntity;

  /// \brief Waypoint visualization markers
  public: WaypointMarkers waypointMarkers{"pinger_marker"};

  /// \brief Debug topic where 2D buoy pose error is published
  public: std::string topicBuoyPoseError = "/vrx/patrolandfollow/debug/buoy_pose_error";

  /// \brief Debug topic where 2D alert pose error is published
  public: std::string topicAlertPoseError = "/vrx/patrolandfollow/debug/alert_pose_error";

  /// \brief Debug topic where 2D follow pose error is published
  public: std::string topicFollowPoseError = "/vrx/patrolandfollow/debug/follow_pose_error";

  /// \brief Debug topic where 2D alert mean error is published
  public: std::string topicAlertMeanError = "/vrx/patrolandfollow/debug/alert_mean_error";

  /// \brief Debug topic where 2D follow mean error is published
  public: std::string topicFollowMeanError = "/vrx/patrolandfollow/debug/follow_mean_error";

  /// \brief Debug topic where ennemy distance is published
  public: std::string topicEnnemyDistance = "/vrx/patrolandfollow/debug/ennemy_distance";

  /// \brief Debug topic where allies distance are published
  public: std::string topicAlliesDistance = "/vrx/patrolandfollow/debug/allies_distance";


  /// \brief Topic where bool second task started is published
  public: std::string topicCurrentPhase = "/vrx/patrolandfollow/current_phase";

  /// \brief Topic where ennemy position is published
  public: std::string topicAisEnnemyPosition = "/wamv/ais_sensor/ennemy_position";

  /// \brief Topic where ennemy speed is published
  public: std::string topicAisEnnemySpeed = "/wamv/ais_sensor/ennemy_speed";

  /// \brief Topic where allies positions is published
  public: std::string topicAisAlliesPositions = "/wamv/ais_sensor/allies_positions";

  /// \brief Topic where allies speeds is published
  public: std::string topicAisAlliesSpeeds = "/wamv/ais_sensor/allies_speeds";


  /// \brief Topic where alert position is received
  public: std::string topicAlertPosition = "/vrx/patrolandfollow/alert_position";

  /// \brief Debug publisher for the combined 2D buoy pose error.
  public: transport::Node::Publisher buoyPoseErrorPub;

  /// \brief Debug publisher for the combined 2D alert pose error.
  public: transport::Node::Publisher alertPoseErrorPub;

  /// \brief Debug publisher for the combined 2D follow pose error.
  public: transport::Node::Publisher followPoseErrorPub;

  /// \brief Debug publisher for the combined 2D alert mean error.
  public: transport::Node::Publisher alertMeanErrorPub;

  /// \brief Debug publisher for the combined 2D follow mean error.
  public: transport::Node::Publisher followMeanErrorPub;

  /// \brief Debug publisher for the ennemy distance.
  public: transport::Node::Publisher ennemyDistancePub;

  /// \brief Debug publisher for the allies distances.
  public: transport::Node::Publisher alliesDistancePub;


  /// \brief Publisher for the bool second task started.
  public: transport::Node::Publisher topicCurrentPhasePub;
  
  /// \brief Publisher for the ais ennemy position.
  public: transport::Node::Publisher topicAisEnnemyPositionPub;

  /// \brief Publisher for the ais ennemy speed.
  public: transport::Node::Publisher topicAisEnnemySpeedPub;

  /// \brief Publisher for the ais allies positions.
  public: transport::Node::Publisher topicAisAlliesPositionsPub;

  /// \brief Publisher for the ais allies speeds.
  public: transport::Node::Publisher topicAisAlliesSpeedsPub;

  /// \brief Number of collisions
  public: int collisions = 0;

  /// \brief Time when phase rally finished.
  public: std::chrono::duration<double> timePhaseRally 
    = std::chrono::duration<double>::zero();

  /// \brief Time to finish alert phase
  public: std::chrono::duration<double> timePhaseAlert
    = std::chrono::duration<double>::zero();

  /// \brief Time to finish alert phase
  public: std::chrono::duration<double> timePhaseAlertFollow
    = std::chrono::duration<double>::zero();

  /// \brief Time to finish alert phase
  public: std::chrono::duration<double> timePhaseFollow
    = std::chrono::duration<double>::zero();

  /// \brief Time follow started
  public: std::chrono::duration<double> timeStartFollow
    = std::chrono::duration<double>::zero();

  /// \brief Last time process alert
  public: std::chrono::duration<double> lastTimeProcessAlert
    = std::chrono::duration<double>::zero();

  /// \brief Last time process follow
  public: std::chrono::duration<double> lastTimeProcessFollow
    = std::chrono::duration<double>::zero();

  /// \brief Last collision time
  public: std::chrono::duration<double> lastCollisionTime 
    = std::chrono::duration<double>::zero();

  /// \brief Last time process speed
  public: std::chrono::duration<double> lastTimeProcessSpeed
    = std::chrono::duration<double>::zero();

  /// \brief Last time publish AIS allies
  public: std::chrono::duration<double> lastTimePubAisAllies
    = std::chrono::duration<double>::zero();

  /// \brief Last time publish AIS ennemy
  public: std::chrono::duration<double> lastTimePubAisEnnemy
    = std::chrono::duration<double>::zero();

  /// \brief Last time process speed
  public: std::chrono::duration<double> lastTimeProcessPublish
    = std::chrono::duration<double>::zero();

  /// \brief Last player position for speed computation
  public: math::Pose3d lastPlayerPose;

  /// \brief Last ennemy position for speed computation
  public: math::Pose3d lastEnnemyPose;

  /// \brief Last allies positions for speed computation
  public: std::vector<math::Pose3d> lastAlliesPoses;

  /// \brief Last log csv data
  public: PatrolAndFollowDataStruct lastDataSaved;

  /// \brief Spherical coordinate conversions. 
  public: math::SphericalCoordinates sc; 

  /// \brief World sim
  public: sim::World world;

  /// \brief Log Info bool
  public: bool logCsvInfo = true;

  /// \brief Log CSV file
  public: std::ofstream logCsvFile;

  /// \brief Log TXT file
  public: std::ofstream logTxtFile;

  /// \brief Register a new alert received.
  /// \param[in] _msg The message containing a alert position.
  public: void OnAlertReceived(const msgs::Pose &_msg);

  /// \brief Current vector of perception requests to be processed.
  public: std::vector<msgs::Pose> requests;

  /// \brief Mutex to protect the requests.
  public: std::mutex mutex;
};

//////////////////////////////////////////////////
void PatrolAndFollowScoringPlugin::Implementation::OnAlertReceived(const msgs::Pose &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->requests.push_back(_msg);
}

/////////////////////////////////////////////////
PatrolAndFollowScoringPlugin::PatrolAndFollowScoringPlugin()
  : ScoringPlugin(),
  dataPtr(utils::MakeUniqueImpl<Implementation>())
{
  gzmsg << "PatrolAndFollowScoringPlugin scoring plugin loaded" << std::endl;
}

/////////////////////////////////////////////////
void PatrolAndFollowScoringPlugin::Configure(const sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  sim::EntityComponentManager &_ecm, sim::EventManager &_eventMgr)
{
  ScoringPlugin::Configure(_entity, _sdf, _ecm, _eventMgr);
  gzmsg << "Task [" << this->TaskName() << "]" << std::endl;

  // Optional log csv info.
  if (_sdf->HasElement("log_csv_info"))
    this->dataPtr->logCsvInfo = _sdf->Get<bool>("log_csv_info");

  // Initialize log files
  auto currentTime = std::chrono::system_clock::now();
  std::time_t currentTime_t = std::chrono::system_clock::to_time_t(currentTime);
  struct tm *localTime = std::localtime(&currentTime_t);
  // Format the date and time in the format YY-MM-DD_HH-mm-ss
  std::stringstream formattedTime;
  formattedTime << std::put_time(localTime, "%y-%m-%d_%H_%M_%S");
  std::string formattedTimeString = formattedTime.str();

  std::string colcon_install_path = std::getenv("COLCON_PREFIX_PATH");
  std::string csvFileName = colcon_install_path + std::string("/../log/patrolandfollow_") 
    + this->dataPtr->world.Name(_ecm).value_or("no_name") + "_" + formattedTimeString + ".csv";
  std::string txtFileName = colcon_install_path + std::string("/../log/patrolandfollow_") 
    + this->dataPtr->world.Name(_ecm).value_or("no_name") + "_" + formattedTimeString + ".txt";
  gzmsg << "PatrolAndFollowScoringPlugin log csv savec to " << csvFileName << std::endl;
  gzmsg << "PatrolAndFollowScoringPlugin log txt savec to " << txtFileName << std::endl;

  // Create log txt file
  this->dataPtr->logTxtFile.open(txtFileName);

  // Write txt header
  this->dataPtr->logTxtFile << "[INFO] Patrol and Follow Scoring Plugin" << std::endl;

  // Required pinger position.
  if (!_sdf->HasElement("pinger_position"))
  {
    gzerr << "Unable to find <pinger_position>" << std::endl;
    return;
  }

  // Required target name.
  if (!_sdf->HasElement("target_name"))
  {
    gzerr << "Unable to find <target_name>" << std::endl;
    return;
  }
  this->dataPtr->pingerPosition = _sdf->Get<math::Vector3d>("pinger_position");
  this->dataPtr->logTxtFile << "[CONF] pinger_position = (" + std::to_string(this->dataPtr->pingerPosition.X()) + "," 
            + std::to_string(this->dataPtr->pingerPosition.Y()) + "," 
            + std::to_string(this->dataPtr->pingerPosition.Z()) + ")" << std::endl;
  this->dataPtr->targetName = _sdf->Get<std::string>("target_name");
  this->dataPtr->logTxtFile << "[CONF] target_name = " + this->dataPtr->targetName << std::endl;
  
  // Optional pinger marker.
  if (_sdf->HasElement("markers"))
  {
    auto markersElement = _sdf->Clone()->GetElement("markers");

    this->dataPtr->waypointMarkers.Load(markersElement);
    if (!this->dataPtr->waypointMarkers.DrawMarker(0,
           this->dataPtr->pingerPosition.X(), this->dataPtr->pingerPosition.Y(),
           this->dataPtr->pingerPosition.Z()))
    {
      gzerr << "Error creating visual marker" << std::endl;
    }
  }

  // Optional tolerance.
  if (_sdf->HasElement("phase_rally_tolerance"))
    this->dataPtr->phaseRallyTolerance = _sdf->Get<double>("phase_rally_tolerance");
  this->dataPtr->logTxtFile << "[CONF] phase_rally_tolerance = " + std::to_string(this->dataPtr->phaseRallyTolerance) << std::endl;

  // Optional alert process period.
  if (_sdf->HasElement("phase_alert_process_period"))
    this->dataPtr->phaseAlertProcessPeriod = _sdf->Get<double>("phase_alert_process_period");
  this->dataPtr->logTxtFile << "[CONF] phase_alert_process_period = " + std::to_string(this->dataPtr->phaseAlertProcessPeriod) << std::endl;

  // Optional alert tolerance
  if (_sdf->HasElement("phase_alert_tolerance"))
    this->dataPtr->phaseAlertTolerance = _sdf->Get<double>("phase_alert_tolerance");
  this->dataPtr->logTxtFile << "[CONF] phase_alert_tolerance = " + std::to_string(this->dataPtr->phaseAlertTolerance) << std::endl;
  
  // Optional follow process period.
  if (_sdf->HasElement("phase_follow_process_period"))
    this->dataPtr->phaseFollowProcessPeriod = _sdf->Get<double>("phase_follow_process_period");
  this->dataPtr->logTxtFile << "[CONF] phase_follow_process_period = " + std::to_string(this->dataPtr->phaseFollowProcessPeriod) << std::endl;

  // Optional max distance.
  if (_sdf->HasElement("phase_follow_tolerance"))
    this->dataPtr->phaseFollowTolerance = _sdf->Get<double>("phase_follow_tolerance");
  this->dataPtr->logTxtFile << "[CONF] phase_follow_tolerance = " + std::to_string(this->dataPtr->phaseFollowTolerance) << std::endl;

  // Optional target distance.
  if (_sdf->HasElement("phase_follow_target_distance"))
    this->dataPtr->phaseFollowTargetDistance = _sdf->Get<double>("phase_follow_target_distance");
  this->dataPtr->logTxtFile << "[CONF] phase_follow_target_distance = " + std::to_string(this->dataPtr->phaseFollowTargetDistance) << std::endl;
  
  // Optional follow time
  if (_sdf->HasElement("phase_follow_ennemy_time"))
    this->dataPtr->phaseFollowEnnemyTime = _sdf->Get<double>("phase_follow_ennemy_time");
  this->dataPtr->logTxtFile << "[CONF] phase_follow_ennemy_time = " + std::to_string(this->dataPtr->phaseFollowEnnemyTime) << std::endl;

  // Optional min distance collision.
  if (_sdf->HasElement("collision_min_distance"))
    this->dataPtr->collisionMinDistance = _sdf->Get<double>("collision_min_distance");
  this->dataPtr->logTxtFile << "[CONF] collision_min_distance = " + std::to_string(this->dataPtr->collisionMinDistance) << std::endl;

  // Optional finish collision distance.
  if (_sdf->HasElement("collision_finish"))
    this->dataPtr->collisionFinish = _sdf->Get<bool>("collision_finish");
  this->dataPtr->logTxtFile << "[CONF] collision_finish = " + std::to_string(this->dataPtr->collisionFinish) << std::endl;

  // Optional collision penality.
  if (_sdf->HasElement("collision_penality"))
    this->dataPtr->collisionPenality = _sdf->Get<double>("collision_penality");
  this->dataPtr->logTxtFile << "[CONF] collision_penality = " + std::to_string(this->dataPtr->collisionPenality) << std::endl;

  // Optional no collisions objects.
  if (_sdf->HasElement("collision_objects"))
  {
    auto collisionsObjectsElement = _sdf->Clone()->GetElement("collision_objects");
    if(collisionsObjectsElement->HasElement("object"))
    {
      auto objectElem = collisionsObjectsElement->GetElement("object");
      while (objectElem)
      {
        auto objectName = objectElem->Get<std::string>("name");
        this->dataPtr->collisionsObjectsNames.push_back(objectName);
        this->dataPtr->logTxtFile << "[CONF] collision_objects = " + objectName << std::endl;
        objectElem = objectElem->GetNextElement("object");
      }
    }
  }

  // Optional ais allies period.
  if (_sdf->HasElement("ais_allies_period"))
    this->dataPtr->aisAlliesPeriod = _sdf->Get<double>("ais_allies_period");
  this->dataPtr->logTxtFile << "[CONF] ais_allies_period = " + std::to_string(this->dataPtr->aisAlliesPeriod) << std::endl;

  // Optional ais ennemy period.
  if (_sdf->HasElement("ais_ennemy_period"))
    this->dataPtr->aisEnnemyPeriod = _sdf->Get<double>("ais_ennemy_period");
  this->dataPtr->logTxtFile << "[CONF] ais_ennemy_period = " + std::to_string(this->dataPtr->aisEnnemyPeriod) << std::endl;

  // Optional ais ennemy wait time.
  if (_sdf->HasElement("ais_ennemy_wait_time"))
    this->dataPtr->aisEnnemyWaitTime = _sdf->Get<double>("ais_ennemy_wait_time");
  this->dataPtr->logTxtFile << "[CONF] ais_ennemy_wait_time = " + std::to_string(this->dataPtr->aisEnnemyWaitTime) << std::endl;

  // Declare publishers
  transport::AdvertiseMessageOptions opts;
  opts.SetMsgsPerSec(1u);
  // Debug topic buoy pose error : Distance between player and buoy (not published on competitive mode)
  this->dataPtr->buoyPoseErrorPub = this->dataPtr->node.Advertise<msgs::Float>(
    this->dataPtr->topicBuoyPoseError, opts);
    
  // Debug topic alert pose error : Distance between ennmy and alert (not published on competitive mode)
  this->dataPtr->alertPoseErrorPub = this->dataPtr->node.Advertise<msgs::Float>(
    this->dataPtr->topicAlertPoseError, opts);

  // Debug topic follow pose error : Distance between player and radius around ennmy (not published on competitive mode)
  this->dataPtr->followPoseErrorPub = this->dataPtr->node.Advertise<msgs::Float>(
    this->dataPtr->topicFollowPoseError, opts);

  // Debug topic alert mean error : Mean error of the alert error (not published on competitive mode)
  this->dataPtr->alertMeanErrorPub = this->dataPtr->node.Advertise<msgs::Float>(
    this->dataPtr->topicAlertMeanError, opts);

  // Debug topic follow mean error : Mean error of the ennemy error (not published on competitive mode)
  this->dataPtr->followMeanErrorPub = this->dataPtr->node.Advertise<msgs::Float>(
    this->dataPtr->topicFollowMeanError, opts);

  // Debug topic ennemy distance : Distance between player and ennemy (not published on competitive mode)
  this->dataPtr->ennemyDistancePub = this->dataPtr->node.Advertise<msgs::Float>(
    this->dataPtr->topicEnnemyDistance, opts);

  // Debug topic allies distances : Distances between player and allies (not published on competitive mode)
  this->dataPtr->alliesDistancePub = this->dataPtr->node.Advertise<msgs::Float_V>(
    this->dataPtr->topicAlliesDistance, opts);


  // Topic current phase : UInt32 with id of the current phase
  this->dataPtr->topicCurrentPhasePub = this->dataPtr->node.Advertise<msgs::UInt32>(
    this->dataPtr->topicCurrentPhase, opts);

  // Topic ennemy position : GPS Position of the target
  this->dataPtr->topicAisEnnemyPositionPub = this->dataPtr->node.Advertise<msgs::Pose>(
    this->dataPtr->topicAisEnnemyPosition, opts);

  // Topic ennemy speed : Float with speed of the target
  this->dataPtr->topicAisEnnemySpeedPub = this->dataPtr->node.Advertise<msgs::Float>(
    this->dataPtr->topicAisEnnemySpeed, opts);

  // Topic allies positions : GPS Positions of the target
  this->dataPtr->topicAisAlliesPositionsPub = this->dataPtr->node.Advertise<msgs::Pose_V>(
    this->dataPtr->topicAisAlliesPositions, opts);

  // Topic allies speeds : Floats with speeds of the target
  this->dataPtr->topicAisAlliesSpeedsPub = this->dataPtr->node.Advertise<msgs::Float_V>(
    this->dataPtr->topicAisAlliesSpeeds, opts);

  // Initialize spherical coordinates instance.
  auto worldEntity = _ecm.EntityByComponents(sim::components::World());
  sim::World world(worldEntity);
  this->dataPtr->world = world;
  this->dataPtr->sc = this->dataPtr->world.SphericalCoordinates(_ecm).value();

  gzmsg << "PatrolAndFollowScoringPlugin::Configured" << std::endl;

  // Create log csv file
  if (this->dataPtr->logCsvInfo)
  {
    this->dataPtr->logCsvFile.open(csvFileName);

    // Write csv header
    std::string line = "Time,Phase";
    line += ",Player X,Player Y,Player Yaw,Player Speed,Player Distance";
    line += ",Ennemy X,Ennemy Y,Ennemy Yaw,Ennemy Speed,Ennemy Distance";
    for(auto objectName : this->dataPtr->collisionsObjectsNames)
    {
      line += "," + objectName + " X," + objectName + " Y," + objectName + " Yaw," + objectName + " Speed," + objectName + " Distance";
    }
    line += ",Buoy X,Buoy Y,Buoy Distance";
    line += ",Last Alert Error,Alert RMS Error,Number Alerts";
    line += ",Alert Position X,Alert Position Y,Alert Latitude,Alert Longitude";
    line += ",Last Follow Error,Follow RMS Error,Number Follow";
    this->dataPtr->logCsvFile << line << std::endl;
  }
}

//////////////////////////////////////////////////
void PatrolAndFollowScoringPlugin::PreUpdate( const sim::UpdateInfo &_info, sim::EntityComponentManager &_ecm)
{
  // Don't update when paused
  if (_info.paused)
    return;

  ScoringPlugin::PreUpdate(_info, _ecm);

  if (this->ScoringPlugin::TaskState() != "running")
    return;

  PatrolAndFollowDataStruct currentData = this->dataPtr->lastDataSaved;
  currentData.time = this->ElapsedTime();

  // Update positions
  // The vehicles might not be ready yet, let's try to get them
  if (!this->dataPtr->playerEntity)
  {
    auto entity = _ecm.EntityByComponents(
      sim::components::Name(ScoringPlugin::VehicleName()));
    if (entity == sim::kNullEntity)
      return;

    this->dataPtr->playerEntity = entity;
  }
  currentData.player = _ecm.Component<sim::components::Pose>(
    this->dataPtr->playerEntity)->Data();

  // The target might not be ready yet, let's try to get it.
  if (!this->dataPtr->ennemyEntity)
  {
    auto entity = _ecm.EntityByComponents(
      sim::components::Name(this->dataPtr->targetName));
    if (entity == sim::kNullEntity)
      return;

    this->dataPtr->ennemyEntity = entity;
  }
  currentData.ennemy = _ecm.Component<sim::components::WorldPose>(
    this->dataPtr->ennemyEntity)->Data();

  std::vector<sim::Entity> entities;
  if(this->dataPtr->alliesEntity.size() == 0)
  {
    for(auto objectName : this->dataPtr->collisionsObjectsNames)
    {
      auto objectEntity = _ecm.EntityByComponents(
        sim::components::Name(objectName));
      if (objectEntity == sim::kNullEntity)
        return;
      entities.push_back(objectEntity);
      this->dataPtr->alliesEntity.push_back(objectEntity);
    }
    this->dataPtr->alliesEntity = entities;
  } else {
    currentData.allies.clear();
    for(auto entity : this->dataPtr->alliesEntity)
    {
      currentData.allies.push_back(_ecm.Component<sim::components::WorldPose>(
        entity)->Data());
    }
  }
  currentData.buoy = this->dataPtr->pingerPosition;

  // Update distances
  currentData.playerDistance = currentData.player.Pos().Distance(math::Vector3d());
  currentData.ennemyDistance = currentData.player.Pos().Distance(currentData.ennemy.Pos());
  currentData.alliesDistance.clear();
  for(auto collision : currentData.allies)
  {
    currentData.alliesDistance.push_back(currentData.player.Pos().Distance(collision.Pos()));
  }
  currentData.buoyDistance = currentData.player.Pos().Distance(this->dataPtr->pingerPosition);
  
  // Prevent speed to trigger errors at the beginning
  if(this->dataPtr->lastTimeProcessSpeed == std::chrono::duration<double>::zero())
  {
    this->dataPtr->lastTimeProcessSpeed = currentData.time;
    this->dataPtr->lastPlayerPose = currentData.player;
    this->dataPtr->lastEnnemyPose = currentData.ennemy;
    this->dataPtr->lastAlliesPoses = currentData.allies;
    for(auto collisionPose : currentData.allies)
    {
      currentData.alliesSpeed.push_back(0);
    }
  }

  // Process collisions with other objects
  bool collision = false;
  for(auto distance : currentData.alliesDistance)
  {
    if (distance < this->dataPtr->collisionMinDistance)
    {
      this->dataPtr->logTxtFile << "[EVENT] Collision, too close from ally : " 
        <<  "Time = " << std::to_string(currentData.time.count()) 
        << ", Distance = " << std::to_string(distance)
        << ", Max = " << std::to_string(this->dataPtr->collisionMinDistance) << std::endl;
      collision = true;
    }
  }
  if (currentData.ennemyDistance < this->dataPtr->collisionMinDistance)
  {
    this->dataPtr->logTxtFile << "[EVENT] Collision, too close from ennemy : " 
      << "Time = " << std::to_string(currentData.time.count()) 
      << ", Distance = " << std::to_string(currentData.ennemyDistance)
      << ", Max = " << std::to_string(this->dataPtr->collisionMinDistance) << std::endl;
    collision = true;
  }
  if(collision)
  {
    OnCollision();
  }
  currentData.collisions = this->dataPtr->collisions;

  // Process current phase
  if(currentData.phase == PatrolAndFollowState::Phase_Initial)
  {
    currentData.phase = PatrolAndFollowState::Phase_Rally;
    this->dataPtr->logTxtFile << "[PHASE] Phase initial finished : " 
      << "Time = " << std::to_string(currentData.time.count()) << std::endl;
  }
  else if(currentData.phase == PatrolAndFollowState::Phase_Rally)
  {
    // Check buoy distance to finish phase
    if (currentData.buoyDistance <= this->dataPtr->phaseRallyTolerance) 
    {
      currentData.phase = PatrolAndFollowState::Phase_Alert;
      this->dataPtr->timePhaseRally = currentData.time;
      this->dataPtr->logTxtFile << "[PHASE] Phase rally finished : "
        << "Time = " << std::to_string(this->dataPtr->timePhaseRally.count()) << std::endl;
    }
  } 
  else if(currentData.phase == PatrolAndFollowState::Phase_Alert)
  {
    if( (currentData.time - this->dataPtr->lastTimeProcessAlert).count() > this->dataPtr->phaseAlertProcessPeriod)
    {
      this->dataPtr->lastTimeProcessAlert = currentData.time;
      ProcessAlert(_ecm, currentData);
      // Check alert distance to finish phase
      if(currentData.alertRMSNumber > 0 && currentData.lastAlertError < this->dataPtr->phaseAlertTolerance)
      {
        currentData.phase = PatrolAndFollowState::Phase_Alert_Follow;
        this->dataPtr->timePhaseAlert = currentData.time - this->dataPtr->timePhaseRally;
        this->dataPtr->logTxtFile << "[PHASE] Phase alert finished : " 
        << "Time = " << std::to_string(this->dataPtr->timePhaseAlert.count()) 
        << ", Last error = " << std::to_string(currentData.lastAlertError)
        << ", RMS alerts = " << std::to_string(currentData.alertRMSError) 
        << ", RMS number = " << std::to_string(currentData.alertRMSNumber) << std::endl;
      }
      // Check time elapsed to skip alert phase
      if((this->ElapsedTime() - this->dataPtr->timePhaseRally).count() > this->dataPtr->aisEnnemyWaitTime)
      {
        currentData.phase = PatrolAndFollowState::Phase_Follow;
        this->dataPtr->timePhaseAlert = currentData.time - this->dataPtr->timePhaseRally;
        this->dataPtr->logTxtFile << "[PHASE] Phase alert skiped because position not found : "
        << "Time = " << std::to_string(this->dataPtr->timePhaseAlert.count()) 
        << ", Last error = " << std::to_string(currentData.lastAlertError)
        << ", RMS alerts = " << std::to_string(currentData.alertRMSError) 
        << ", RMS number = " << std::to_string(currentData.alertRMSNumber) << std::endl;
      }
    }
  }
  else if(currentData.phase == PatrolAndFollowState::Phase_Alert_Follow)
  {
    // This is a special phase in 2 phase : First wait the player to be in maximum range from the target, then start follow it
    if((currentData.time - this->dataPtr->lastTimeProcessAlert).count() > this->dataPtr->phaseAlertProcessPeriod)
    {
      this->dataPtr->lastTimeProcessAlert = currentData.time;
      ProcessAlert(_ecm, currentData);
    }
    if((currentData.time - this->dataPtr->lastTimeProcessFollow).count() > this->dataPtr->phaseFollowProcessPeriod)
    {
      this->dataPtr->lastTimeProcessFollow = this->ElapsedTime();
      ProcessFollow(_ecm, currentData);
    }
    // Check time elapsed to finish alert phase
    if((this->ElapsedTime() - this->dataPtr->timePhaseRally).count() > this->dataPtr->aisEnnemyWaitTime)
    {
      currentData.phase = PatrolAndFollowState::Phase_Follow;
      this->dataPtr->timePhaseAlertFollow = currentData.time - this->dataPtr->timePhaseRally - this->dataPtr->timePhaseAlert;
      this->dataPtr->logTxtFile << "[PHASE] Phase alert follow finished : "
        << "Time = " << std::to_string(this->dataPtr->timePhaseAlertFollow.count()) 
        << ", Last error = " << std::to_string(currentData.lastAlertError)
        << ", RMS alerts = " << std::to_string(currentData.alertRMSError) 
        << ", RMS number = " << std::to_string(currentData.alertRMSNumber) << std::endl;
    }
  }
  else if(currentData.phase == PatrolAndFollowState::Phase_Follow)
  {
    if((this->ElapsedTime() - this->dataPtr->lastTimeProcessFollow).count() > this->dataPtr->phaseFollowProcessPeriod)
    {
      this->dataPtr->lastTimeProcessFollow = this->ElapsedTime();
      ProcessFollow(_ecm, currentData);
    }
    // Check follow started and time elapsed to finish follow phase
    if(this->dataPtr->timeStartFollow != std::chrono::duration<double>::zero()
      && (this->ElapsedTime() - this->dataPtr->timeStartFollow).count() > this->dataPtr->phaseFollowEnnemyTime)
    {
      currentData.phase = PatrolAndFollowState::Phase_Task_Finished;
      this->dataPtr->timePhaseFollow = currentData.time - this->dataPtr->timePhaseRally - this->dataPtr->timePhaseAlert - this->dataPtr->timePhaseAlertFollow;
      this->dataPtr->logTxtFile << "[PHASE] Phase alert finished : "
      << "Time = " << std::to_string(this->dataPtr->timePhaseFollow.count()) 
      << ", Last error = " << std::to_string(currentData.lastFollowError) 
      << ", RMS follow = " << std::to_string(currentData.followRMSError) 
      << ", RMS number = " << std::to_string(currentData.followRMSNumber) << std::endl;
    }
  }
  else if(currentData.phase == PatrolAndFollowState::Phase_Task_Finished)
  {
    this->dataPtr->logTxtFile << "[PHASE] Task finished succesfully : "
    << "Total Time = " << std::to_string(this->ElapsedTime().count()) << std::endl;
    this->Finish();
  }

  // Process periodic tasks
  // Update speed only at 1 Hz
  if( (currentData.time - this->dataPtr->lastTimeProcessSpeed).count() > PROCESS_SPEED_TASK_PERIOD)
  {
    auto diffTime = (currentData.time - this->dataPtr->lastTimeProcessSpeed).count();
    currentData.playerSpeed = currentData.player.Pos().Distance(this->dataPtr->lastPlayerPose.Pos()) / diffTime;
    currentData.ennemySpeed = currentData.ennemy.Pos().Distance(this->dataPtr->lastEnnemyPose.Pos()) / diffTime;
    currentData.alliesSpeed.clear();
    for(auto collisionPose : currentData.allies)
    {
      currentData.alliesSpeed.push_back(collisionPose.Pos().Distance(this->dataPtr->lastAlliesPoses[0].Pos()) / diffTime);
    }
    this->dataPtr->lastTimeProcessSpeed = currentData.time;
    this->dataPtr->lastPlayerPose = currentData.player;
    this->dataPtr->lastEnnemyPose = currentData.ennemy;
    this->dataPtr->lastAlliesPoses = currentData.allies;
  }

  if( (currentData.time - this->dataPtr->lastTimePubAisAllies).count() > this->dataPtr->aisAlliesPeriod)
  {
    // Allies AIS
    msgs::Pose_V alliesPosesMsg;
    for(auto allyPose : currentData.allies)
    {
      math::Vector3d cartVec(-allyPose.Pos().X(), -allyPose.Pos().Y(), allyPose.Pos().Z());
      math::Vector3d latlon = this->dataPtr->sc.SphericalFromLocalPosition(cartVec);
      const math::Quaternion<double> orientation = allyPose.Rot();
      math::Pose3d pose(latlon, orientation);
      msgs::Set(alliesPosesMsg.add_pose(),pose);
    }
    this->dataPtr->topicAisAlliesPositionsPub.Publish(alliesPosesMsg);
    msgs::Float_V alliesSpeedsMsg;
    for(auto allySpeed : currentData.alliesSpeed)
    {
      alliesSpeedsMsg.add_data(allySpeed);
    }
    this->dataPtr->topicAisAlliesSpeedsPub.Publish(alliesSpeedsMsg);
    this->dataPtr->lastTimePubAisAllies = currentData.time;
  }

  // Publish ennemy AIS only during follow phase
  if(currentData.phase == PatrolAndFollowState::Phase_Follow &&
    (currentData.time - this->dataPtr->lastTimePubAisEnnemy).count() > this->dataPtr->aisEnnemyPeriod)
  {
    math::Vector3d cartVec(-currentData.ennemy.Pos().X(), -currentData.ennemy.Pos().Y(), currentData.ennemy.Pos().Z());
    math::Vector3d latlon = this->dataPtr->sc.SphericalFromLocalPosition(cartVec);
    const math::Quaternion<double> orientation = currentData.ennemy.Rot();
    msgs::Pose geoPoseMsg;
    geoPoseMsg.mutable_position()->set_x(latlon.X());
    geoPoseMsg.mutable_position()->set_y(latlon.Y());
    geoPoseMsg.mutable_position()->set_z(latlon.Z());
    geoPoseMsg.mutable_orientation()->set_x(orientation.X());
    geoPoseMsg.mutable_orientation()->set_y(orientation.Y());
    geoPoseMsg.mutable_orientation()->set_z(orientation.Z());
    geoPoseMsg.mutable_orientation()->set_w(orientation.W());
    this->dataPtr->topicAisEnnemyPositionPub.Publish(geoPoseMsg);
    msgs::Float speedMsg;
    speedMsg.set_data(currentData.ennemySpeed);
    this->dataPtr->topicAisEnnemySpeedPub.Publish(speedMsg);
    this->dataPtr->lastTimePubAisEnnemy = currentData.time;
  }

  // Publish debug topics, log csv and current phase topic
  if( (currentData.time - this->dataPtr->lastTimeProcessPublish).count() > PROCESS_PUBLISH_TASK_PERIOD)
  {
    msgs::UInt32 currentPhaseMsg;
    currentPhaseMsg.set_data(uint(currentData.phase));
    this->dataPtr->topicCurrentPhasePub.Publish(currentPhaseMsg);

    OnSendTopicDebug(currentData);
    OnLogDataCsv(currentData);

    this->dataPtr->lastTimeProcessPublish = currentData.time;
  }

  // Save data every loop
  this->dataPtr->lastDataSaved = currentData;
}

//////////////////////////////////////////////////
void PatrolAndFollowScoringPlugin::OnCollision()
{
  if((this->ElapsedTime() - this->dataPtr->lastCollisionTime).count() < 1)
  {
    this->dataPtr->lastCollisionTime = this->ElapsedTime();
  } else {
    this->dataPtr->lastCollisionTime = this->ElapsedTime();
    this->dataPtr->collisions++;
  }

  this->dataPtr->logTxtFile << "[EVENT] Collision detected : Number = " << std::to_string(this->dataPtr->collisions) << std::endl;

  if(this->dataPtr->collisionFinish)
  {
    this->Finish();
  }
}

//////////////////////////////////////////////////
void PatrolAndFollowScoringPlugin::OnFinished()
{
  // Print score details
  OnLogDataCsv(this->dataPtr->lastDataSaved);
  if(this->dataPtr->lastDataSaved.phase == PatrolAndFollowState::Phase_Task_Finished)
  {
    this->dataPtr->logTxtFile << "[FINISH] Task finished succesfully : Time = " << std::to_string(this->ElapsedTime().count()) << std::endl;
  } else {
    this->dataPtr->logTxtFile << "[FINISH]Task finished with error : Time = " << std::to_string(this->ElapsedTime().count()) << std::endl;
    this->dataPtr->logTxtFile << "[FINISH] GAME OVER, RETRY AGAIN !" << std::endl;
  }

  // Close log files
  this->dataPtr->logCsvFile.close();
  this->dataPtr->logTxtFile.close();

  // Call parent OnFinished
  ScoringPlugin::OnFinished();
}

//////////////////////////////////////////////////
void PatrolAndFollowScoringPlugin::OnRunning()
{
  this->dataPtr->node.Subscribe(this->dataPtr->topicAlertPosition,
    &PatrolAndFollowScoringPlugin::Implementation::OnAlertReceived, this->dataPtr.get());
  ScoringPlugin::OnRunning();
}

//////////////////////////////////////////////////
void PatrolAndFollowScoringPlugin::OnLogDataCsv(PatrolAndFollowDataStruct& data)
{
  if (this->dataPtr->logCsvInfo)
  {
    // Write csv header
    std::string line = std::to_string(data.time.count()) + "," + std::to_string(data.phase);
    line += "," + std::to_string(data.player.Pos().X()) + "," + std::to_string(data.player.Pos().Y()) 
      + "," + std::to_string(data.player.Rot().Yaw()) + "," + std::to_string(data.playerSpeed) 
      + "," + std::to_string(data.playerDistance);
    line += "," + std::to_string(data.ennemy.Pos().X()) + "," + std::to_string(data.ennemy.Pos().Y()) 
      + "," + std::to_string(data.ennemy.Rot().Yaw()) + "," + std::to_string(data.ennemySpeed) 
      + "," + std::to_string(data.ennemyDistance);
    if(data.allies.size() != data.alliesDistance.size() || data.allies.size() != data.alliesSpeed.size())
    {
      gzerr << "Error in allies data : Not the same size" << std::endl;
    }
    for(int i = 0; i < data.allies.size(); i++)
    {
      line += "," + std::to_string(data.allies[i].Pos().X()) + "," + std::to_string(data.allies[i].Pos().Y()) 
        + "," + std::to_string(data.allies[i].Rot().Yaw()) + "," + std::to_string(data.alliesSpeed[i]) 
        + "," + std::to_string(data.alliesDistance[i]);
    }
    line += "," + std::to_string(data.buoy.X()) + "," + std::to_string(data.buoy.Y()) + "," + std::to_string(data.buoyDistance);
    line += "," + std::to_string(data.lastAlertError) + "," + std::to_string(data.alertRMSError) + "," + std::to_string(data.alertRMSNumber) 
      + "," + std::to_string(data.lastAlertLocalPose.Pos().X()) + "," + std::to_string(data.lastAlertLocalPose.Pos().Y()) 
      + "," + std::to_string(data.lastAlertGpsPose.Pos().X()) + "," + std::to_string(data.lastAlertGpsPose.Pos().Y());
    line += "," + std::to_string(data.lastFollowError) + "," + std::to_string(data.followRMSError) + "," + std::to_string(data.followRMSNumber);
    this->dataPtr->logCsvFile << line << std::endl;
  }
}

//////////////////////////////////////////////////
void PatrolAndFollowScoringPlugin::OnSendTopicDebug(PatrolAndFollowDataStruct& data)
{
  msgs::Float buoyPoseErrorMsg;
  buoyPoseErrorMsg.set_data(data.buoyDistance);
  this->dataPtr->buoyPoseErrorPub.Publish(buoyPoseErrorMsg);

  msgs::Float alertPoseErrorMsg;
  alertPoseErrorMsg.set_data(data.lastAlertError);
  this->dataPtr->alertPoseErrorPub.Publish(alertPoseErrorMsg);

  msgs::Float followPoseErrorMsg;
  followPoseErrorMsg.set_data(data.lastFollowError);
  this->dataPtr->followPoseErrorPub.Publish(followPoseErrorMsg);

  msgs::Float alertMeanErrorMsg;
  alertMeanErrorMsg.set_data(data.alertRMSError);
  this->dataPtr->alertMeanErrorPub.Publish(alertMeanErrorMsg);

  msgs::Float followMeanErrorMsg;
  followMeanErrorMsg.set_data(data.followRMSError);
  this->dataPtr->followMeanErrorPub.Publish(followMeanErrorMsg);

  msgs::Float ennemyDistanceMsg;
  ennemyDistanceMsg.set_data(data.ennemyDistance);
  this->dataPtr->ennemyDistancePub.Publish(ennemyDistanceMsg);

  msgs::Float_V alliesDistanceMsg;
  for(auto distance : data.alliesDistance)
  {
    alliesDistanceMsg.add_data(distance);
  }
  this->dataPtr->alliesDistancePub.Publish(alliesDistanceMsg);
}

void PatrolAndFollowScoringPlugin::ProcessAlert(sim::EntityComponentManager &_ecm, PatrolAndFollowDataStruct& data)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  if(this->dataPtr->requests.size() == 0)
  {
    return;
  }
  // Get the last request
  auto request = this->dataPtr->requests.back();

  // Convert geo pose to Gazebo pose.
  math::Vector3d scVec(request.position().x(), request.position().y(), 0.0);
  math::Vector3d cartVec = this->dataPtr->sc.LocalFromSphericalPosition(scVec);

  data.lastAlertGpsPose.Pos().X() = request.position().x();
  data.lastAlertGpsPose.Pos().Y() = request.position().y();
  data.lastAlertLocalPose.Pos().X() = cartVec.X();
  data.lastAlertLocalPose.Pos().Y() = cartVec.Y();

  // 2D Error
  double error = sqrt(pow(cartVec.X() - data.ennemy.Pos().X(), 2) +
                      pow(cartVec.Y() - data.ennemy.Pos().Y(), 2));
  
  data.lastAlertError = error;
  data.alertRMSErrorPow = (data.alertRMSErrorPow * data.alertRMSNumber + pow(error, 2)) / (data.alertRMSNumber + 1);
  data.alertRMSError = sqrt(data.alertRMSErrorPow);
  data.alertRMSNumber++;

  this->dataPtr->logTxtFile << "[PROCESS] ProcessAlert : "
  << "Number = " << std::to_string(data.alertRMSNumber) 
  << ", Error = " << std::to_string(data.lastAlertError) 
  << ", Mean Error = " << std::to_string(data.alertRMSError) << std::endl;
  // Save last alert pose
  this->dataPtr->requests.clear();
  this->dataPtr->requests.push_back(request);
}

void PatrolAndFollowScoringPlugin::ProcessFollow(sim::EntityComponentManager &_ecm, PatrolAndFollowDataStruct& data)
{
  double distance = data.player.Pos().Distance(data.ennemy.Pos());

  if(this->dataPtr->timeStartFollow == std::chrono::duration<double>::zero())
  {
    if(distance < this->dataPtr->phaseFollowTolerance)
    {
      this->dataPtr->timeStartFollow = data.time;
    } else {
      this->dataPtr->logTxtFile << "[PROCESS] ProcessFollow to far from target : "
      << "Distance = " << std::to_string(distance) 
      << ", Max = " << std::to_string(this->dataPtr->phaseFollowTolerance) << std::endl;
    }
  }
  if(this->dataPtr->timeStartFollow != std::chrono::duration<double>::zero())
  {
    data.lastFollowError = abs(distance - this->dataPtr->phaseFollowTargetDistance);
    data.followRMSErrorPow = (data.followRMSErrorPow * data.followRMSNumber + pow(data.lastFollowError, 2)) / (data.followRMSNumber + 1);
    data.followRMSError = sqrt(data.followRMSErrorPow);
    data.followRMSNumber++;

    this->dataPtr->logTxtFile << "[PROCESS] ProcessFollow : "
    << "Number = " << std::to_string(data.followRMSNumber) 
    << ", Distance = " << std::to_string(distance)
    << ", Error = " << std::to_string(data.lastFollowError)
    << ", Mean Error = " << std::to_string(data.followRMSError) << std::endl;
  }
}

GZ_ADD_PLUGIN(vrx::PatrolAndFollowScoringPlugin,
              sim::System,
              vrx::PatrolAndFollowScoringPlugin::ISystemConfigure,
              vrx::PatrolAndFollowScoringPlugin::ISystemPreUpdate)
