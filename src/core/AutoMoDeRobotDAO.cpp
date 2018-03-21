/*
 * @file <src/core/AutoMoDeRobotDAO.cpp>
 *
 * @author Antoine Ligot - <aligot@ulb.ac.be>
 *
 * @package ARGoS3-AutoMoDe
 *
 * @license MIT License
 */

#include "AutoMoDeRobotDAO.h"

namespace argos {

	/****************************************/
	/****************************************/

	AutoMoDeRobotDAO::AutoMoDeRobotDAO() {
		m_pcRng = CRandom::CreateRNG("argos");
		m_pcRabMessageBuffer = AutoMoDeRabBuffer();
		m_pcRabMessageBuffer.SetTimeLife(10);
		m_fMaxVelocity = 10;
		m_fLeftWheelVelocity = 0;
		m_fRightWheelVelocity = 0;
	}

	/****************************************/
	/****************************************/

	AutoMoDeRobotDAO::~AutoMoDeRobotDAO() {}

	/****************************************/
	/****************************************/

	CCI_EPuckProximitySensor::TReadings AutoMoDeRobotDAO::GetProximityInput() const {
		return m_sProximityInput;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeRobotDAO::SetProximityInput(CCI_EPuckProximitySensor::TReadings s_prox_input) {
		m_sProximityInput = s_prox_input;
	}

	/****************************************/
	/****************************************/

	CCI_EPuckLightSensor::TReadings AutoMoDeRobotDAO::GetLightInput() const{
		return m_sLightInput;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeRobotDAO::SetLightInput(CCI_EPuckLightSensor::TReadings s_light_input) {
		m_sLightInput = s_light_input;
	}

	/****************************************/
	/****************************************/

	CCI_EPuckGroundSensor::SReadings AutoMoDeRobotDAO::GetGroundInput() {
		std::deque<CCI_EPuckGroundSensor::SReadings>::iterator it;
	 	//float sum[3] = {0.0,0.0,0.0};
		int counter[2] = {0,0};
		for (it = m_deqGroundInput.begin(); it != m_deqGroundInput.end(); it++) {
			if (it->Left < 0.001) {
				counter[0] += 1;
			}
			else if (it->Left > 0.92) {
				counter[1] += 1;
			}
			if (it->Center < 0.001) {
				counter[0] +=1;
			}
			else if (it->Center > 0.92) {
				counter[1] += 1;
			}
			if (it->Right < 0.001) {
				counter[0] +=1;
			}
			else if (it->Right > 0.92) {
				counter[1] += 1;
			}
		}

		CCI_EPuckGroundSensor::SReadings readings;
		if (counter[0] > 10) {
			readings.Left = 0;
			readings.Center = 0;
			readings.Right = 0;
		}
		else if (counter[1] > 10) {
			readings.Left = 1;
			readings.Center = 1;
			readings.Right = 1;
		}
		else {
			readings.Left = 0.5;
			readings.Center = 0.5;
			readings.Right = 0.5;
		}

		return readings;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeRobotDAO::SetGroundInput(CCI_EPuckGroundSensor::SReadings s_ground_input) {
		m_deqGroundInput.push_back(s_ground_input);
		if (m_deqGroundInput.size() > 5) {
			m_deqGroundInput.pop_front();
		}
		//m_sGroundInput = s_ground_input;
	}

	/****************************************/
	/****************************************/

	const UInt8& AutoMoDeRobotDAO::GetNumberNeighbors() const {
		return m_unNumberNeighbors;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeRobotDAO::SetNumberNeighbors(const UInt8& un_number_neighbors){
		m_unNumberNeighbors = un_number_neighbors;
	}

	/****************************************/
	/****************************************/

	std::vector<CCI_EPuckRangeAndBearingSensor::SReceivedPacket*> AutoMoDeRobotDAO::GetRangeAndBearingMessages() {
		return m_pcRabMessageBuffer.GetMessages();
	}

	/****************************************/
	/****************************************/

	void AutoMoDeRobotDAO::SetRangeAndBearingMessages(CCI_EPuckRangeAndBearingSensor::TPackets s_packets) {
		std::map<UInt32, CCI_EPuckRangeAndBearingSensor::SReceivedPacket*> mapRemainingMessages;
		std::map<UInt32, CCI_EPuckRangeAndBearingSensor::SReceivedPacket*>::iterator mapIt;
		CCI_EPuckRangeAndBearingSensor::TPackets::iterator it;
		m_unNumberNeighbors = 0;
		for (it = s_packets.begin(); it < s_packets.end(); ++it) {
			if ((*it)->Data[0] != m_unRobotIdentifier) {
				if (mapRemainingMessages.find((*it)->Data[0]) != mapRemainingMessages.end()) {  // If ID not in map, add message.
					mapRemainingMessages[(*it)->Data[0]] = (*it);
				} else if ((*it)->Bearing != CRadians::ZERO){  // If ID there, overwrite only if the message is valid (correct range and bearing information)
					mapRemainingMessages[(*it)->Data[0]] = (*it);
				}
			}
		}
		for (mapIt = mapRemainingMessages.begin(); mapIt != mapRemainingMessages.end(); ++mapIt) {
			m_pcRabMessageBuffer.AddMessage((*mapIt).second);
			m_unNumberNeighbors += 1;
		}
		m_pcRabMessageBuffer.Update();
	}

	/****************************************/
	/****************************************/

	void AutoMoDeRobotDAO::SetRangeAndBearingMessageToSend(UInt8 un_message) {
		m_unMessageToSend = un_message;
	}

	/****************************************/
	/****************************************/

	UInt8 AutoMoDeRobotDAO::GetNumberMessagingNeighbors(UInt8 un_message) {
		CCI_EPuckRangeAndBearingSensor::TPackets sLastPackets = GetRangeAndBearingMessages();
		CCI_EPuckRangeAndBearingSensor::TPackets::iterator it;
		UInt8 unNumberMessagingNeighbors = 0;

		for (it = sLastPackets.begin(); it != sLastPackets.end(); it++) {
			if ( ((*it)->Data[0] != (UInt8) GetRobotIdentifier()) && ((*it)->Data[1] == (UInt8) un_message) ) {
				unNumberMessagingNeighbors+=1;
			}
		}

		return unNumberMessagingNeighbors;
	}

	/****************************************/
	/****************************************/

	const UInt8& AutoMoDeRobotDAO::GetMessageToSend() const {
		return m_unMessageToSend;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeRobotDAO::SetWheelsVelocity(const Real& un_left_velocity, const Real& un_right_velocity) {
		m_fLeftWheelVelocity = un_left_velocity;
		m_fRightWheelVelocity = un_right_velocity;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeRobotDAO::SetWheelsVelocity(const CVector2& c_velocity_vector) {
		m_fLeftWheelVelocity = c_velocity_vector.GetX();
		m_fRightWheelVelocity = c_velocity_vector.GetY();
	}

	/****************************************/
	/****************************************/

	const Real& AutoMoDeRobotDAO::GetRightWheelVelocity() const {
		return m_fRightWheelVelocity;
	}

	/****************************************/
	/****************************************/

	const Real& AutoMoDeRobotDAO::GetLeftWheelVelocity() const {
		return m_fLeftWheelVelocity;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeRobotDAO::Reset() {
		m_fLeftWheelVelocity = 0;
		m_fRightWheelVelocity = 0;
		m_pcRabMessageBuffer.Reset();
	}

	/****************************************/
	/****************************************/

	void AutoMoDeRobotDAO::SetRobotIdentifier(const UInt32& un_robot_id) {
		m_unRobotIdentifier = un_robot_id;
	}

	/****************************************/
	/****************************************/

	const UInt32& AutoMoDeRobotDAO::GetRobotIdentifier() const{
		return m_unRobotIdentifier;
	}

	/****************************************/
	/****************************************/

	CRandom::CRNG* AutoMoDeRobotDAO::GetRandomNumberGenerator() const {
		return m_pcRng;
	}

	/****************************************/
	/****************************************/

	const Real& AutoMoDeRobotDAO::GetMaxVelocity() const{
		return m_fMaxVelocity;
	}
}
