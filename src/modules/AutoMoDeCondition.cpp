/**
  * @file <src/modules/AutoMoDeCondition.cpp>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @package ARGoS3-AutoMoDe
  *
  * @license MIT License
  */

#include "AutoMoDeCondition.h"

namespace argos {

	/****************************************/
	/****************************************/

	const std::string AutoMoDeCondition::GetDOTDescription() {
		std::stringstream ss;
		ss << m_strLabel;
		if (!m_mapParameters.empty()) {
			std::map<std::string, Real>::iterator it;
			for (it = m_mapParameters.begin(); it != m_mapParameters.end(); it++) {
				ss << "\\n" << it->first << "=" << it->second ;
			}
		}
		return ss.str();
	}

	/****************************************/
	/****************************************/

	void AutoMoDeCondition::Init() {
		m_unMessage = 0;
		if (m_strMethod == "2E" || m_strMethod == "1E" || m_strMethod == "1EX" || m_strMethod == "2") {
			std::map<std::string, Real>::iterator itMes = m_mapParameters.find("m");
		    if (itMes != m_mapParameters.end()) {
		      m_unMessage = itMes->second;
		    } else {
		      LOGERR << "[FATAL] Missing parameter m for the following condition:" << m_strLabel << std::endl;
		      THROW_ARGOSEXCEPTION("Missing Parameter");
			}
		}
		if (m_strMethod == "2E" || m_strMethod == "1E" || m_strMethod == "1EX") {
			std::map<std::string, Real>::iterator itThres = m_mapParameters.find("t");
			if (itThres != m_mapParameters.end()) {
			  m_unThreshold = itThres->second;
			} else {
			  LOGERR << "[FATAL] Missing parameter t for the following condition:" << m_strLabel << std::endl;
			  THROW_ARGOSEXCEPTION("Missing Parameter");
			}
		}

	}

	/****************************************/
	/****************************************/

	void AutoMoDeCondition::AddParameter(const std::string& str_identifier, const Real& f_value) {
		m_mapParameters.insert(std::pair<std::string, Real>(str_identifier, f_value));
	}

	/****************************************/
	/****************************************/

	const UInt32& AutoMoDeCondition::GetOrigin() const {
		return m_unFromBehaviourIndex;
	}

	/****************************************/
	/****************************************/

	const UInt32& AutoMoDeCondition::GetExtremity() const {
		return m_unToBehaviourIndex;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeCondition::SetOrigin(const UInt32& un_from) {
		m_unFromBehaviourIndex = un_from;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeCondition::SetExtremity(const UInt32& un_to) {
		m_unToBehaviourIndex = un_to;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeCondition::SetOriginAndExtremity(const UInt32& un_from, const UInt32& un_to) {
		m_unFromBehaviourIndex = un_from;
		m_unToBehaviourIndex = un_to;
	}

	/****************************************/
	/****************************************/

	const std::string& AutoMoDeCondition::GetLabel() const {
		return m_strLabel;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeCondition::SetIndex(const UInt32& un_index) {
		m_unIndex = un_index;
	}

	/****************************************/
	/****************************************/

	const UInt32& AutoMoDeCondition::GetIndex() const {
		return m_unIndex;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeCondition::SetMethod(const std::string& str_method) {
		m_strMethod = str_method;
	}

	/****************************************/
	/****************************************/

	const std::string& AutoMoDeCondition::GetMethod() const {
		return m_strMethod;
	}


	/****************************************/
	/****************************************/

	void AutoMoDeCondition::SetIdentifier(const UInt32& un_id) {
		m_unIdentifier = un_id;
	}

	/****************************************/
	/****************************************/

	const UInt32& AutoMoDeCondition::GetIdentifier() const {
		return m_unIdentifier;
	}

	/****************************************/
	/****************************************/

	std::map<std::string, Real> AutoMoDeCondition::GetParameters() const {
		return m_mapParameters;
	}

  /****************************************/
	/****************************************/

    void AutoMoDeCondition::SetRobotDAO(EpuckDAO* pc_robot_dao) {
        m_pcRobotDAO = pc_robot_dao;
    }

	/****************************************/
	/****************************************/

	bool AutoMoDeCondition::EvaluateBernoulliProbability(Real f_probability) {
		if (m_unMessage > 0) {
		  	SInt32 unNumberNeighbors = m_pcRobotDAO->GetNumberMessagingNeighbors(m_unMessage);
			//LOG << "mes:" << m_unMessage << " : "<< m_unThreshold << " :: " << unNumberNeighbors << std::endl;
			if (unNumberNeighbors > m_unThreshold) {
  				return m_pcRobotDAO->GetRandomNumberGenerator()->Bernoulli(f_probability);
			}
		}
		else if (m_unMessage == 0) {
		  	return m_pcRobotDAO->GetRandomNumberGenerator()->Bernoulli(f_probability);
		}
		return 0;
	}

}
