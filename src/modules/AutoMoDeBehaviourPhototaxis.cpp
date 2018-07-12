/**
  * @file <src/modules/AutoMoDeBehaviourPhototaxis.cpp>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @package ARGoS3-AutoMoDe
  *
  * @license MIT License
  */

#include "AutoMoDeBehaviourPhototaxis.h"


namespace argos {

	/****************************************/
	/****************************************/

	AutoMoDeBehaviourPhototaxis::AutoMoDeBehaviourPhototaxis() {
		m_strLabel = "Phototaxis";
	}

	/****************************************/
	/****************************************/

	AutoMoDeBehaviourPhototaxis::AutoMoDeBehaviourPhototaxis(AutoMoDeBehaviourPhototaxis* pc_behaviour) {
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

	AutoMoDeBehaviourPhototaxis::~AutoMoDeBehaviourPhototaxis() {}

	/****************************************/
	/****************************************/

	AutoMoDeBehaviourPhototaxis* AutoMoDeBehaviourPhototaxis::Clone() {
		return new AutoMoDeBehaviourPhototaxis(this);
	}

	/****************************************/
	/****************************************/

	void AutoMoDeBehaviourPhototaxis::ControlStep() {
		m_pcRobotDAO->SetRangeAndBearingMessageToSend(m_bBroadcastStateAndMessage);
		CVector2 sResultVector(0,CRadians::ZERO);
		CVector2 sLightVector(0,CRadians::ZERO);
		CVector2 sProxVector(0,CRadians::ZERO);

		CCI_EPuckLightSensor::SReading cLightReading = m_pcRobotDAO->GetLightReading();
		sLightVector = CVector2(cLightReading.Value, cLightReading.Angle);

		sProxVector = CVector2(m_pcRobotDAO->GetProximityReading().Value, m_pcRobotDAO->GetProximityReading().Angle);
		sResultVector = sLightVector - 5*sProxVector;

		if (sResultVector.Length() < 0.1) {
			sResultVector = CVector2(1, CRadians::ZERO);
		}
		
		m_pcRobotDAO->SetWheelsVelocity(ComputeWheelsVelocityFromVector(sResultVector));

		m_bLocked = false;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeBehaviourPhototaxis::Init() {
		std::map<std::string, Real>::iterator itMes = m_mapParameters.find("brd");
		if (itMes != m_mapParameters.end()) {
			m_bBroadcastStateAndMessage = itMes->second;
			// if (m_bBroadcastStateAndMessage == 1) {
			// 	m_bBroadcastStateAndMessage = 85;
			// }
		} else {
			LOGERR << "[FATAL] Missing parameter for the following behaviour:" << m_strLabel << std::endl;
			THROW_ARGOSEXCEPTION("Missing Parameter");
		}
	}

	/****************************************/
	/****************************************/

	void AutoMoDeBehaviourPhototaxis::Reset() {
		m_bOperational = false;
		ResumeStep();
	}

	/****************************************/
	/****************************************/

	void AutoMoDeBehaviourPhototaxis::ResumeStep() {
		m_bOperational = true;
	}
}
