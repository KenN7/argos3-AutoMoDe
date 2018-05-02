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

		CCI_EPuckRangeAndBearingSensor::TPackets sLastPackets = m_pcRobotDAO->GetRangeAndBearingMessages();
		CCI_EPuckRangeAndBearingSensor::TPackets::iterator it;
		CVector2 sRabVectorSum(0,CRadians::ZERO);
		CVector2 sProxVectorSum(0,CRadians::ZERO);
		CVector2 sResultVector(0,CRadians::ZERO);

		for (it = sLastPackets.begin(); it != sLastPackets.end(); it++) {
			if ( ((*it)->Data[0] != (UInt8) m_pcRobotDAO->GetRobotIdentifier()) && ((*it)->Data[1] == (UInt8) m_unRepulsiveMessage) ) {
				sRabVectorSum += CVector2(m_unRepulsionParameter / ((*it)->Range + 1),(*it)->Bearing.SignedNormalize());
			}
		}
		sProxVectorSum = SumProximityReadings(m_pcRobotDAO->GetProximityInput());
		sResultVector = -sRabVectorSum - 5*sProxVectorSum;
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
        std::map<std::string, Real>::iterator itMes = m_mapParameters.find("m");
		if ( (it != m_mapParameters.end()) && (itMes != m_mapParameters.end()) ) {
			m_unRepulsionParameter = it->second;
            m_unRepulsiveMessage = itMes->second;
		} else {
			LOGERR << "[FATAL] Missing parameter for the following behaviour:" << m_strLabel << std::endl;
			THROW_ARGOSEXCEPTION("Missing Parameter");
		}
		//m_unRepulsiveMessage = 85; 

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
