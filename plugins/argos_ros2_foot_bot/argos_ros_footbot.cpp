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
using namespace collective_decision_making;
using namespace collective_decision_making::msg;
using namespace geometry_msgs::msg;
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
		m_pcLight(NULL),
		m_pcLEDs(NULL),
		m_pcCamera(NULL),
		m_pcProximity(NULL),
		m_pcPosition(NULL),
		m_pcRABA(NULL),
		m_pcRABS(NULL),
		stopWithoutSubscriberCount(10),
		stepsSinceCallback(0),
		leftSpeed(0),
		rightSpeed(0){}

ArgosRosFootbot::~ArgosRosFootbot(){}

void ArgosRosFootbot::Init(TConfigurationNode& t_node){
	/********************************
	 * Create the topics to publish
	 *******************************/
	stringstream lightTopic, blobTopic, proxTopic, positionTopic, rabTopic;
	lightTopic 		<< "/" << GetId() << "/light";
	blobTopic 		<< "/" << GetId() << "/blob";
	proxTopic 		<< "/" << GetId() << "/proximity";
	positionTopic 	<< "/" << GetId() << "/position";
	rabTopic 		<< "/" << GetId() << "/rab";
	lightListPublisher_ = ArgosRosFootbot::nodeHandle -> create_publisher<LightList>(lightTopic.str(), 1);
	blobPublisher_ 		= ArgosRosFootbot::nodeHandle -> create_publisher<BlobList>(blobTopic.str(), 1);
	promixityPublisher_ = ArgosRosFootbot::nodeHandle -> create_publisher<ProximityList>(proxTopic.str(), 1);
	positionPublisher_ 	= ArgosRosFootbot::nodeHandle -> create_publisher<Position>(positionTopic.str(), 1);
	rabPublisher_ 		= ArgosRosFootbot::nodeHandle -> create_publisher<PacketList>(rabTopic.str(), 1);

	/*********************************
	 * Create subscribers
	 ********************************/
	stringstream cmdVelTopic, cmdRabTopic, cmdLedTopic;
	cmdVelTopic 	<< "/" << GetId() << "/cmd_vel";
	cmdRabTopic		<< "/" << GetId() << "/cmd_rab";
	cmdLedTopic		<< "/" << GetId() << "/cmd_led";
	cmdVelSubscriber_ = ArgosRosFootbot::nodeHandle -> create_subscription<Twist>(
						cmdVelTopic.str(),
						1,
						std::bind(&ArgosRosFootbot::cmdVelCallback, this, _1)
						);
	cmdRabSubscriber_ = ArgosRosFootbot::nodeHandle -> create_subscription<Packet>(
						cmdRabTopic.str(),
						1,
						std::bind(&ArgosRosFootbot::cmdRabCallback, this, _1)
						);
	cmdLedSubscriber_ = ArgosRosFootbot::nodeHandle -> create_subscription<Led>(
						cmdLedTopic.str(),
						1,
						std::bind(&ArgosRosFootbot::cmdLedCallback, this, _1)
						);
	/********************************
	 * Get sensor/actuator handles
	 ********************************/
	m_pcLight  		= GetSensor < CCI_FootBotLightSensor					>("footbot_light");
	m_pcProximity 	= GetSensor < CCI_FootBotProximitySensor				>("footbot_proximity");
	m_pcLEDs   		= GetActuator<CCI_LEDsActuator                          >("leds");
	m_pcCamera 		= GetSensor < CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
	m_pcPosition 	= GetSensor < CCI_PositioningSensor						>("positioning");
	m_pcRABS 		= GetSensor < CCI_RangeAndBearingSensor					>("range_and_bearing" );

	/********************************
	 * Get actuator handles
	 ********************************/
	m_pcWheels = GetActuator< CCI_DifferentialSteeringActuator >("differential_steering");
	m_pcRABA = GetActuator< CCI_RangeAndBearingActuator        >("range_and_bearing" );

	/*
	* Other init stuff
	*/
	Reset();
	/*
	* Parse the configuration file
	*
	* The user defines this part. Here, the algorithm accepts three
	* parameters and it's nice to put them in the config file so we don't
	* have to recompile if we want to try other settings.
	*/
	GetNodeAttributeOrDefault(t_node, "stopWithoutSubscriberCount", stopWithoutSubscriberCount, stopWithoutSubscriberCount);

}

bool blobComparator(Blob a, Blob b) {
	return a.angle < b.angle;
}

void ArgosRosFootbot::ControlStep() {

	rclcpp::spin_some(ArgosRosFootbot::nodeHandle);

	/*********************************
	 * Get readings from light sensor
	 *********************************/
	const CCI_FootBotLightSensor::TReadings& tLightReads = m_pcLight->GetReadings();
	LightList lightList;
	lightList.n = tLightReads.size();
	for (size_t i = 0; i < lightList.n; ++i) {
		Light light;
		light.value = tLightReads[i].Value;
		light.angle = tLightReads[i].Angle.GetValue();
		lightList.lights.push_back(light);

		//cout << GetId() << ": value: " << light.value << ": angle: " << light.angle << endl;
	}

	lightListPublisher_ -> publish(lightList);

	/***********************************
	 * Get readings from proximity sensor
	 ***********************************/
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

	/**************************************************************
	 * Get readings from Colored Blob Omnidirectional Camera Sensor
	 *************************************************************/
	const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& camReads = m_pcCamera->GetReadings();
	BlobList blobList;
	blobList.n = camReads.BlobList.size();
	Blob blob;
	for (size_t i = 0; i < blobList.n; ++i) {
		//Blob blob;
		stringstream ss;
		ss << camReads.BlobList[i]->Color;
		blob.color = ss.str();
		blob.distance = camReads.BlobList[i]->Distance;

		// Make the angle of the puck in the range [-PI, PI].  This is useful for
		// tasks such as homing in on a puck using a simple controller based on
		// the sign of this angle.
		blob.angle = camReads.BlobList[i]->Angle.GetValue();//.SignedNormalize().GetValue();
		blobList.blobs.push_back(blob);
		//std::cout << "value: " << blob.distance << ": angle: " << blob.angle << " color: " << blob.color << std::endl;

	}

	// Sort the puck list by angle.  This is useful for the purposes of extracting meaning from
	// the local puck configuration (e.g. fitting a lines to the detected pucks).
	sort(blobList.blobs.begin(), blobList.blobs.end(), blobComparator);

	blobPublisher_ -> publish(blobList);

	/*********************************************************************
	 * Get readings from Positioning sensor
	 * TODO: Find an elegant way to make assignment
	 * Problem: can't directly assign argos::CVector3 to geometry::Vector3
	 * Same with the Quaternion
	 **********************************************************************/
	const CCI_PositioningSensor::SReading& tPosReads = m_pcPosition->GetReading();
	Position position;

	position.position.x = tPosReads.Position.GetX();
	position.position.y = tPosReads.Position.GetY();
	position.position.z = tPosReads.Position.GetZ();

	position.orientation.w = tPosReads.Orientation.GetW();
	position.orientation.x = tPosReads.Orientation.GetX();
	position.orientation.y = tPosReads.Orientation.GetY();
	position.orientation.z = tPosReads.Orientation.GetZ();

	positionPublisher_ -> publish(position);

	/**CVector3 pos(
				position.position.x,
				position.position.y,
				position.position.z
				);
	float r = float(blob.distance/100.0f);
	float x = r * cos(blob.angle);
	float y = r * sin(blob.angle);
	float dist = std::sqrt(std::pow((2 - pos.GetX()), 2) + std::pow((18 - pos.GetY()), 2));

	std::cout 	<< "calculated distance: " << dist << " measured in cm: "
				<< blob.distance << " in meters: "
				<< float(blob.distance/100.0f)
				<< " angle: " << blob.angle << std::endl;

	CVector2 toTarget(float(pos.GetX() + x), float(pos.GetY() + y));
	CQuaternion toTarget3(0, x, y, 0);
	CQuaternion prod1 = toTarget3 * tPosReads.Orientation;
	prod1 = prod1 * tPosReads.Orientation.Conjugate();
	float x_g = prod1.GetX() + tPosReads.Position.GetX();
	float y_g = prod1.GetY() + tPosReads.Position.GetY();
	std::cout 	<< "relative target- x: " << x << " y: " << y
				<< " current position: " << pos
				<< " Orientation: " << tPosReads.Orientation << std::endl;
	std::cout << "global target: " << x_g << ", " << y_g << std::endl;*/

	/*********************************************
	 * Get readings from Range-And-Bearing-Sensor
	 *********************************************/
	const CCI_RangeAndBearingSensor::TReadings& tRabReads = m_pcRABS->GetReadings();
	PacketList packetList;
	packetList.n = tRabReads.size();
	for (size_t i = 0; i < packetList.n; ++i) {
		Packet packet;
		packet.range = tRabReads[i].Range;
		packet.h_bearing = tRabReads[i].HorizontalBearing.GetValue();
		packet.v_bearing = tRabReads[i].VerticalBearing.GetValue();

		// Read out coordinates, keeping tract of signs
		/**float x = tRabReads[i].Data[0];
		float y = tRabReads[i].Data[1];
		float id = tRabReads[i].Data[4];

		x = (tRabReads[i].Data[2] > 0) ? (-x) : x;
		y = (tRabReads[i].Data[3] > 0) ? (-y) : y;


		packet.data.push_back(x);
		packet.data.push_back(y);*/
		if (tRabReads[i].Data[0] > 0){
			cout << GetId() << ": light-source-id: " << tRabReads[i].Data[0] << endl;
			packet.data.push_back(tRabReads[i].Data[0]);
			packet.data.push_back(tRabReads[i].Data[1]);

			packetList.packets.push_back(packet);
		}
		//cout << GetId() << ": id: " << tRabReads[i].Data[1] << endl;
	}

	rabPublisher_ -> publish(packetList);
	m_pcRABA -> ClearData();
	/* Get the camera readings */
	/**const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sReadings = m_pcCamera->GetReadings();
	BlobList blobList;
	blobList.n = sReadings.BlobList.size();
	for (size_t i = 0; i < blobList.n; ++i) {
		Blob blob;
		blob.distance = sReadings.BlobList[i]->Distance;
		blob.angle = sReadings.BlobList[i]->Angle.GetValue();
		blob.color = (sReadings.BlobList[i]->Color == CColor::RED)? "red" : "";

		blobList.blobs.push_back(blob);

		//cout << GetId() << ": value: " << prox.value << ": angle: " << prox.angle << endl;
	}

	blobPublisher_ -> publish(blobList);*/


	// If we haven't heard from the subscriber in a while, set the speed to zero.
	if (stepsSinceCallback > stopWithoutSubscriberCount) {
		leftSpeed = 0;
		rightSpeed = 0;
	} else {
		stepsSinceCallback++;
	}
	//std::cout << GetId() << ": left-wheel: " << leftSpeed << ": right-wheel: " << rightSpeed << std::endl;
	m_pcWheels->SetLinearVelocity(leftSpeed, rightSpeed);


}

void ArgosRosFootbot::Reset() {
   /* Enable camera filtering */
   m_pcCamera->Enable();
   /** Set the LED intensity first */
   m_pcLEDs -> SetAllIntensities( 100 );
   /* Set beacon color to all red to be visible for other robots */
   m_pcLEDs->SetAllColors(CColor::RED);
}

void ArgosRosFootbot::cmdVelCallback(const Twist& twist) {
	//cout << "cmdVelCallback: " << GetId() << endl;

	Real v = twist.linear.x;  // Forward speed
	Real w = twist.angular.z; // Rotational speed

	// Use the kinematics of a differential-drive robot to derive the left
	// and right wheel speeds.
	if (twist.linear.z == 1.0){
		leftSpeed = (v - HALF_BASELINE * w) / WHEEL_RADIUS;
		rightSpeed = (v + HALF_BASELINE * w) / WHEEL_RADIUS;
	}
	else{ // Drive to target
		leftSpeed = twist.linear.x;
		rightSpeed = twist.linear.y;
	}



	stepsSinceCallback = 0;
}

void ArgosRosFootbot::cmdRabCallback(const Packet& packet){
	cout << GetId() << " Packet data as received: " << packet.data[0] <<endl;
	//m_pcRABA -> SetData(0, 1); // validity flag
	m_pcRABA -> SetData(0, packet.data[0]);
	m_pcRABA -> SetData(1, std::stoi( packet.id ));

	/**if (isSigned(packet.data[0])){
		m_pcRABA -> SetData(0, -packet.data[0]);
		m_pcRABA -> SetData(2, 1);
	}
	else{
		m_pcRABA -> SetData(0, packet.data[0]);
		m_pcRABA -> SetData(2, 0);
	}
	if (isSigned(packet.data[1])){
		m_pcRABA -> SetData(1, -packet.data[1]);
		m_pcRABA -> SetData(3, 1);
	}
	else{
		m_pcRABA -> SetData(3, 0);
		m_pcRABA -> SetData(1, packet.data[1]);
	}
	if (isSigned(packet.data[2])){
		m_pcRABA -> SetData(4, -packet.data[2]);
		m_pcRABA -> SetData(5, 1);
	}
	else{
		m_pcRABA -> SetData(4, packet.data[2]);
		m_pcRABA -> SetData(5, 0);
	}*/
}
void ArgosRosFootbot::cmdLedCallback(const Led& ledColor){
	cout << " Received the following color: " << ledColor.color << std::endl;
	if ( ledColor.color == "yellow" ){
		m_pcLEDs->SetAllColors(CColor::ORANGE);
		cout << GetId() << " setting leds to yellow." << std::endl;
	}
	else if ( ledColor.color == "green" ){
		m_pcLEDs->SetAllColors(CColor::CYAN);
		cout << GetId() << " setting leds to green." << std::endl;
	}


}
bool ArgosRosFootbot::isSigned(float num)
{
	return num<0; //std::is_signed<decltype(num)>::value ;
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
