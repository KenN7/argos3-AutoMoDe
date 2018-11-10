/*
 * File: AutoMoDeBehaviourAttractionToMessage.cpp
 *
 */

#include "AutoMoDeBehaviourAttractionToMessage.h"


namespace argos {

	/****************************************/
	/****************************************/

	AutoMoDeBehaviourAttractionToMessage::AutoMoDeBehaviourAttractionToMessage() {
		m_strLabel = "AttractionToMessage";
	}

	/****************************************/
	/****************************************/

	AutoMoDeBehaviourAttractionToMessage::AutoMoDeBehaviourAttractionToMessage(AutoMoDeBehaviourAttractionToMessage* pc_behaviour) {
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

	AutoMoDeBehaviourAttractionToMessage::~AutoMoDeBehaviourAttractionToMessage() {}

	/****************************************/
	/****************************************/

	AutoMoDeBehaviourAttractionToMessage* AutoMoDeBehaviourAttractionToMessage::Clone() {
		return new AutoMoDeBehaviourAttractionToMessage(this);   // todo: check without *
	}

	/****************************************/
	/****************************************/

	void AutoMoDeBehaviourAttractionToMessage::ControlStep() {
		m_pcRobotDAO->SetRangeAndBearingMessageToSend(0);
		CVector2 sRabVector(0,CRadians::ZERO);
		CVector2 sProxVector(0,CRadians::ZERO);
		CVector2 sResultVector(0,CRadians::ZERO);
		CCI_EPuckRangeAndBearingSensor::SReceivedPacket cRabReading = m_pcRobotDAO->GetAttractionVectorToMessagingNeighbors(m_unAttractionParameter,m_unAttractiveMessage);

		if (cRabReading.Range > 0.0f) {
			sRabVector = CVector2(cRabReading.Range, cRabReading.Bearing);
		}
		sProxVector = CVector2(m_pcRobotDAO->GetProximityReading().Value, m_pcRobotDAO->GetProximityReading().Angle);
		sResultVector = sRabVector - 6*sProxVector;

		if (sResultVector.Length() < 0.1) {
			sResultVector = CVector2(1, CRadians::ZERO);
		}

		m_pcRobotDAO->SetWheelsVelocity(ComputeWheelsVelocityFromVector(sResultVector));

		m_bLocked = false;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeBehaviourAttractionToMessage::Init() {
		std::map<std::string, Real>::iterator it = m_mapParameters.find("att");
		if ( it != m_mapParameters.end() ) {
			m_unAttractionParameter = it->second;
		} else {
			LOGERR << "[FATAL] Missing parameter for the following behaviour:" << m_strLabel << std::endl;
			THROW_ARGOSEXCEPTION("Missing Parameter");
		}
		m_unAttractiveMessage = 85;

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

	void AutoMoDeBehaviourAttractionToMessage::Reset() {
		m_bOperational = false;
		ResumeStep();
	}

	/****************************************/
	/****************************************/

	void AutoMoDeBehaviourAttractionToMessage::ResumeStep() {
		m_bOperational = true;
	}
}
