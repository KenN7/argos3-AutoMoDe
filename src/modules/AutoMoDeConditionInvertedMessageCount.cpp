/*
 * File: AutoMoDeConditionInvertedMessageCount.cpp
 *
 */

 #include "AutoMoDeConditionInvertedMessageCount.h"

 namespace argos {

	/****************************************/
	/****************************************/

	AutoMoDeConditionInvertedMessageCount::AutoMoDeConditionInvertedMessageCount() {
		m_strLabel = "InvertedMessageCount";
	}

	/****************************************/
	/****************************************/

	AutoMoDeConditionInvertedMessageCount::~AutoMoDeConditionInvertedMessageCount() {}

	/****************************************/
	/****************************************/

	AutoMoDeConditionInvertedMessageCount::AutoMoDeConditionInvertedMessageCount(AutoMoDeConditionInvertedMessageCount* pc_condition) {
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

	AutoMoDeConditionInvertedMessageCount* AutoMoDeConditionInvertedMessageCount::Clone() {
		return new AutoMoDeConditionInvertedMessageCount(this);
	}

	/****************************************/
	/****************************************/

	bool AutoMoDeConditionInvertedMessageCount::Verify() {
		// SInt32 unNumberNeighbors = m_pcRobotDAO->GetNumberMessagingNeighbors(m_unParameterMes);
		// Real fProbability = 1 - (1/(1 + exp(m_fParameterEta * (m_unParameterXi - unNumberNeighbors))));
		// return EvaluateBernoulliProbability(fProbability);
        SInt8 unNumberNeighbors = 0;
        if (m_unParameterMes > 0) {
            unNumberNeighbors = m_pcRobotDAO->GetNumberMessagingNeighbors(m_unParameterMes);
        }


        //Real fProbability = (1/(1 + exp(m_fParameterEta * (m_unParameterXi - unNumberNeighbors))));

        if (unNumberNeighbors < m_fParameterEta) {
            return 1;
        }
        else {
            return 0;
        }

	}

	/****************************************/
	/****************************************/

	void AutoMoDeConditionInvertedMessageCount::Reset() {

	}

	/****************************************/
	/****************************************/

	void AutoMoDeConditionInvertedMessageCount::Init() {
		std::map<std::string, Real>::iterator itEta = m_mapParameters.find("w");
        //std::map<std::string, Real>::iterator itXi = m_mapParameters.find("p");
        std::map<std::string, Real>::iterator itMes = m_mapParameters.find("m");
		if ( (itEta != m_mapParameters.end()) && (itMes != m_mapParameters.end()) ) { //&& (itXi != m_mapParameters.end())
			m_fParameterEta = itEta->second;
			//m_unParameterXi = itXi->second;
            m_unParameterMes = itMes->second;
		} else {
			LOGERR << "[FATAL] Missing parameter for the following condition:" << m_strLabel << std::endl;
			THROW_ARGOSEXCEPTION("Missing Parameter");
		}
	}

 }
