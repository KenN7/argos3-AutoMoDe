/*
 * File: AutoMoDeConditionMessageCount.cpp
 *
 */

 #include "AutoMoDeConditionMessageCount.h"

 namespace argos {

	/****************************************/
	/****************************************/

	AutoMoDeConditionMessageCount::AutoMoDeConditionMessageCount() {
		m_strLabel = "MessageCount";
	}

	/****************************************/
	/****************************************/

	AutoMoDeConditionMessageCount::~AutoMoDeConditionMessageCount() {}

	/****************************************/
	/****************************************/

	AutoMoDeConditionMessageCount::AutoMoDeConditionMessageCount(AutoMoDeConditionMessageCount* pc_condition) {
		m_strLabel = pc_condition->GetLabel();
		m_unIndex = pc_condition->GetIndex();
		m_unIdentifier = pc_condition->GetIndex();
		m_unFromBehaviourIndex = pc_condition->GetOrigin();
		m_unToBehaviourIndex = pc_condition->GetExtremity();
		m_mapParameters = pc_condition->GetParameters();
    Init();
	}

	/****************************************/
	/****************************************/

	AutoMoDeConditionMessageCount* AutoMoDeConditionMessageCount::Clone() {
		return new AutoMoDeConditionMessageCount(this);
	}

	/****************************************/
	/****************************************/

	bool AutoMoDeConditionMessageCount::Verify() {
		UInt32 unNumberNeighbors = m_pcRobotDAO->GetNumberMessagingNeighbors(1); //TODO add message to param, for now 1 is hardcoded for gianduja
        // if (unNumberNeighbors > 1) {
        //     LOG << "neighbos sending messages : " << unNumberNeighbors << std::endl;
        // }
		Real fProbability = (1/(1 + exp(m_fParameterEta * (m_unParameterXi - unNumberNeighbors))));
		return EvaluateBernoulliProbability(fProbability);
	}

	/****************************************/
	/****************************************/

	void AutoMoDeConditionMessageCount::Reset() {

	}

	/****************************************/
	/****************************************/

	void AutoMoDeConditionMessageCount::Init() {
		std::map<std::string, Real>::iterator itEta = m_mapParameters.find("w");
		std::map<std::string, Real>::iterator itXi = m_mapParameters.find("p");
		if ((itEta != m_mapParameters.end()) && (itXi != m_mapParameters.end())) {
			m_fParameterEta = itEta->second;
			m_unParameterXi = itXi->second;
		} else {
			LOGERR << "[FATAL] Missing parameter for the following condition:" << m_strLabel << std::endl;
			THROW_ARGOSEXCEPTION("Missing Parameter");
		}
	}

 }
