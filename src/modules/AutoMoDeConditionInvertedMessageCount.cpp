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
        m_strMethod = pc_condition->GetMethod();
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
        SInt32 unNumberNeighbors = m_pcRobotDAO->GetNumberMessagingNeighbors(m_unParameterMes);
            if (m_strMethod == "2") {
                if (unNumberNeighbors < m_fParameterEta && m_unParameterMes > 0) {
                    return 1;
                }
                else {
                    return 0;
                }
            }
            else if (m_strMethod == "1" || m_strMethod == "1X") {
                Real fProbability = 1 - (1/(1 + exp(m_fParameterEta * (m_unParameterXi - unNumberNeighbors))));
                return EvaluateBernoulliProbability(fProbability);
            }
        return 0;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeConditionInvertedMessageCount::Reset() {

	}

	/****************************************/
	/****************************************/

	void AutoMoDeConditionInvertedMessageCount::Init() {
        AutoMoDeCondition::Init();
		std::map<std::string, Real>::iterator itEta = m_mapParameters.find("w");
		if ( (itEta != m_mapParameters.end()) ) {
			m_fParameterEta = itEta->second;
		} else {
			LOGERR << "[FATAL] Missing parameter for the following condition:" << m_strLabel << std::endl;
			THROW_ARGOSEXCEPTION("Missing Parameter");
		}

        if (m_strMethod == "1" || m_strMethod == "1X" || m_strMethod == "2") {
            std::map<std::string, Real>::iterator itXi = m_mapParameters.find("p");
            if ( itXi != m_mapParameters.end() ) {
                m_unParameterXi = itXi->second;
            }
            else {
                LOGERR << "[FATAL] Missing p parameter for the following behaviour:" << m_strLabel << std::endl;
                THROW_ARGOSEXCEPTION("Missing Parameter");
            }
        }

        m_unParameterMes = 85; //default value for 1bit 
        if (m_strMethod == "2") {
            std::map<std::string, Real>::iterator itMes = m_mapParameters.find("m");
            if ( itMes != m_mapParameters.end() ) {
                m_unParameterMes = itMes->second;
            }
            else {
                LOGERR << "[FATAL] Missing m parameter for the following behaviour:" << m_strLabel << std::endl;
                THROW_ARGOSEXCEPTION("Missing Parameter");
            }
        }
	}

 }
