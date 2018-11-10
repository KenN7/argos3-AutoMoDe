/*
 * File: AutoMoDeBehaviourRepulsion.cpp
 *
 */

#include "AutoMoDeBehaviourRepulsionToMessage.h"


namespace argos {

	/****************************************/
	/****************************************/

	AutoMoDeBehaviourRepulsionToMessage::AutoMoDeBehaviourRepulsionToMessage() {
		m_strLabel = "RepulsionToMessage";
	}

	/****************************************/
	/****************************************/

	AutoMoDeBehaviourRepulsionToMessage::AutoMoDeBehaviourRepulsionToMessage(AutoMoDeBehaviourRepulsionToMessage* pc_behaviour) {
		m_strLabel = pc_behaviour->GetLabel();
		m_bLocked = pc_behaviour->IsLocked();
		m_bOperational = pc_behaviour->IsOperational();
		m_unIndex = pc_behaviour->GetIndex();
		m_unIdentifier = pc_behaviour->GetIdentifier();
		m_mapParameters = pc_behaviour->GetParameters();
		Init();
	}

	/****************************************/
	/****************************************/

	AutoMoDeBehaviourRepulsionToMessage::~AutoMoDeBehaviourRepulsionToMessage() {}

	/****************************************/
	/****************************************/

	AutoMoDeBehaviourRepulsionToMessage* AutoMoDeBehaviourRepulsionToMessage::Clone() {
		return new AutoMoDeBehaviourRepulsionToMessage(this);
	}

	/****************************************/
	/****************************************/

	void AutoMoDeBehaviourRepulsionToMessage::ControlStep() {

		m_pcRobotDAO->SetRangeAndBearingMessageToSend(0);
		CVector2 sRabVector(0,CRadians::ZERO);
		CVector2 sProxVector(0,CRadians::ZERO);
		CVector2 sResultVector(0,CRadians::ZERO);
		CCI_EPuckRangeAndBearingSensor::SReceivedPacket cRabReading = m_pcRobotDAO->GetAttractionVectorToMessagingNeighbors(m_unRepulsionParameter,m_unRepulsiveMessage);

		if (cRabReading.Range > 0.0f) {
			sRabVector = CVector2(cRabReading.Range, cRabReading.Bearing);
		}
		sProxVector = CVector2(m_pcRobotDAO->GetProximityReading().Value, m_pcRobotDAO->GetProximityReading().Angle);
		sResultVector = -sRabVector - 6*sProxVector;

		if (sResultVector.Length() < 0.1) {
			sResultVector = CVector2(1, CRadians::ZERO);
		}

		m_pcRobotDAO->SetWheelsVelocity(ComputeWheelsVelocityFromVector(sResultVector));

		m_bLocked = false;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeBehaviourRepulsionToMessage::Init() {
		std::map<std::string, Real>::iterator it = m_mapParameters.find("rep");
		if ( it != m_mapParameters.end() ) {
			m_unRepulsionParameter = it->second;
		} else {
			LOGERR << "[FATAL] Missing parameter for the following behaviour:" << m_strLabel << std::endl;
			THROW_ARGOSEXCEPTION("Missing Parameter");
		}
		m_unRepulsiveMessage = 85;

		// std::map<std::string, Real>::iterator itMes = m_mapParameters.find("brd");
		// if (itMes != m_mapParameters.end()) {
		// 	m_bBroadcastState = itMes->second;
		// } else {
		// 	LOGERR << "[FATAL] Missing parameter for the following behaviour:" << m_strLabel << std::endl;
		// 	THROW_ARGOSEXCEPTION("Missing Parameter");
		// }
	}

	/****************************************/
	/****************************************/

	void AutoMoDeBehaviourRepulsionToMessage::Reset() {
		m_bOperational = false;
		ResumeStep();
	}

	/****************************************/
	/****************************************/

	void AutoMoDeBehaviourRepulsionToMessage::ResumeStep() {
		m_bOperational = true;
	}
}
