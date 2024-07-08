/*
 * argos_ros_footbot.cpp
 *
 *  Created on: 20 Jun 2024
 *  Author: Sindiso Mkhatshwa
 *  Email: sindiso.mkhatshwa@uni-konstanz.de
 */

/* Include the controller definition */
#include "argos_ros_footbot.h"
using namespace std;
using namespace argos_interface;
using namespace argos_interface::msg;
using std::placeholders::_1;

/**
 * Initialize the node before creating the publishers
 * and subscribers. Otherwise we get a guard-error during
 * compilation if we initialize the node after.
 */
std::shared_ptr<rclcpp::Node> initNode() {
  int argc = 1;
  char *argv = (char *) "";
  rclcpp::init(argc, &argv);
  
  return std::make_shared<rclcpp::Node>("argos_ros_node");

}

std::shared_ptr<rclcpp::Node> ArgosRosFootbot::nodeHandle = initNode();

ArgosRosFootbot::ArgosRosFootbot() :
		m_pcWheels(NULL),
		m_pcOmniCam(NULL),
		m_pcProximity(NULL),
		stopWithoutSubscriberCount(10),
		stepsSinceCallback(0),
		leftSpeed(0),
		rightSpeed(0){}

ArgosRosFootbot::~ArgosRosFootbot(){}

void ArgosRosFootbot::Init(TConfigurationNode& t_node){
	// Create the topics to publish
	stringstream puckListTopic, proximityTopic;
	puckListTopic << "/puck_list";
	proximityTopic << "/proximity";
	puckListPublisher_ = ArgosRosFootbot::nodeHandle -> create_publisher<argos_interface::msg::PuckList>(puckListTopic.str(), 1);
	promixityPublisher_ = ArgosRosFootbot::nodeHandle -> create_publisher<argos_interface::msg::ProximityList>(proximityTopic.str(), 1);

	// Create the subscriber
	stringstream cmdVelTopic;
	cmdVelTopic << "/cmd_vel";
	cmdVelSubscriber_ = ArgosRosFootbot::nodeHandle -> create_subscription<geometry_msgs::msg::Twist>(
			cmdVelTopic.str(),
			1,
			std::bind(&ArgosRosFootbot::cmdVelCallback, this, _1)
			);

	// Get sensor/actuator handles
	m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
	m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
	m_pcOmniCam = GetSensor<CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");

	m_pcOmniCam->Enable();

	/*
	* Parse the configuration file
	*
	* The user defines this part. Here, the algorithm accepts three
	* parameters and it's nice to put them in the config file so we don't
	* have to recompile if we want to try other settings.
	*/
	GetNodeAttributeOrDefault(t_node, "stopWithoutSubscriberCount", stopWithoutSubscriberCount, stopWithoutSubscriberCount);


}

bool puckComparator(Puck a, Puck b) {
	return a.angle < b.angle;
}

void ArgosRosFootbot::ControlStep() {
	rclcpp::spin_some(ArgosRosFootbot::nodeHandle);
	const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& camReads = m_pcOmniCam->GetReadings();
	PuckList puckList;
	puckList.n = camReads.BlobList.size();
	for (size_t i = 0; i < puckList.n; ++i) {
		Puck puck;
		puck.type = (camReads.BlobList[i]->Color == CColor::RED);
		puck.range = camReads.BlobList[i]->Distance;
		// Make the angle of the puck in the range [-PI, PI].  This is useful for
		// tasks such as homing in on a puck using a simple controller based on
		// the sign of this angle.
		puck.angle = camReads.BlobList[i]->Angle.SignedNormalize().GetValue();
		puckList.pucks.push_back(puck);
	}

	// Sort the puck list by angle.  This is useful for the purposes of extracting meaning from
	// the local puck configuration (e.g. fitting a lines to the detected pucks).
	sort(puckList.pucks.begin(), puckList.pucks.end(), puckComparator);

	puckListPublisher_ -> publish(puckList);

	/* Get readings from proximity sensor */
	const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
	ProximityList proxList;
	proxList.n = tProxReads.size();
	for (size_t i = 0; i < proxList.n; ++i) {
		Proximity prox;
		prox.value = tProxReads[i].Value;
		prox.angle = tProxReads[i].Angle.GetValue();
		proxList.proximities.push_back(prox);

		//cout << GetId() << ": value: " << prox.value << ": angle: " << prox.angle << endl;
	}

	promixityPublisher_ -> publish(proxList);


	// If we haven't heard from the subscriber in a while, set the speed to zero.
	if (stepsSinceCallback > stopWithoutSubscriberCount) {
		leftSpeed = 0;
		rightSpeed = 0;
	} else {
		stepsSinceCallback++;
	}

	m_pcWheels->SetLinearVelocity(leftSpeed, rightSpeed);


}
void ArgosRosFootbot::cmdVelCallback(const geometry_msgs::msg::Twist& twist) {
	//cout << "cmdVelCallback: " << GetId() << endl;

	Real v = twist.linear.x;  // Forward speed
	Real w = twist.angular.z; // Rotational speed

	// Use the kinematics of a differential-drive robot to derive the left
	// and right wheel speeds.
	leftSpeed = (v - HALF_BASELINE * w) / WHEEL_RADIUS;
	rightSpeed = (v + HALF_BASELINE * w) / WHEEL_RADIUS;

	stepsSinceCallback = 0;
}

/*
* This statement notifies ARGoS of the existence of the controller.
* It binds the class passed as first argument to the string passed as
* second argument.
* The string is then usable in the configuration file to refer to this
* controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(ArgosRosFootbot, "argos_ros_bot_controller")
