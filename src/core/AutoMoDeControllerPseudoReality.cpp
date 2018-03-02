/*
 * @file <src/core/AutoMoDeController.cpp>
 *
 * @author Antoine Ligot - <aligot@ulb.ac.be>
 *
 * @package ARGoS3-AutoMoDe
 *
 * @license MIT License
 */

#include "AutoMoDeControllerPseudoReality.h"

namespace argos {

	/****************************************/
	/****************************************/

	AutoMoDeControllerPseudoReality::AutoMoDeControllerPseudoReality() {
		m_pcRobotState = new AutoMoDeRobotDAO();
		m_unTimeStep = 0;
		m_strFsmConfiguration = "";
		m_bMaintainHistory = false;
		m_bPrintReadableFsm = false;
		m_strHistoryFolder = "./";
		m_bFiniteStateMachineGiven = false;
		m_bPseudoRealitySet = false;
		m_fLossProbability = 0.85;
	}

	/****************************************/
	/****************************************/

	AutoMoDeControllerPseudoReality::~AutoMoDeControllerPseudoReality() {
		delete m_pcRobotState;
		delete m_pcFsmBuilder;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeControllerPseudoReality::Init(TConfigurationNode& t_node) {
		// Parsing parameters
		try {
			GetNodeAttributeOrDefault(t_node, "fsm-config", m_strFsmConfiguration, m_strFsmConfiguration);
			GetNodeAttributeOrDefault(t_node, "history", m_bMaintainHistory, m_bMaintainHistory);
			GetNodeAttributeOrDefault(t_node, "hist-folder", m_strHistoryFolder, m_strHistoryFolder);
			GetNodeAttributeOrDefault(t_node, "readable", m_bPrintReadableFsm, m_bPrintReadableFsm);
			GetNodeAttributeOrDefault(t_node, "pseudo_reality_description", m_strPseudoRealityDescriptionFile, m_strPseudoRealityDescriptionFile);
		} catch (CARGoSException& ex) {
			THROW_ARGOSEXCEPTION_NESTED("Error parsing <params>", ex);
		}

		LoadPseudoReality();

		m_unRobotID = atoi(GetId().substr(5, 6).c_str());
		m_pcRobotState->SetRobotIdentifier(m_unRobotID);

		/*
		 * If a FSM configuration is given as parameter of the experiment file, create a FSM from it
		 */
		if (m_strFsmConfiguration.compare("") != 0 && !m_bFiniteStateMachineGiven) {
			m_pcFsmBuilder = new AutoMoDeFsmBuilder();
			SetFiniteStateMachine(m_pcFsmBuilder->BuildFiniteStateMachine(m_strFsmConfiguration));
			if (m_bMaintainHistory) {
				m_pcFiniteStateMachine->SetHistoryFolder(m_strHistoryFolder);
				m_pcFiniteStateMachine->MaintainHistory();
			}
			if (m_bPrintReadableFsm) {
				std::cout << "Finite State Machine description: " << std::endl;
				std::cout << m_pcFiniteStateMachine->GetReadableFormat() << std::endl;
			}
		} else {
			LOGERR << "Warning: No finite state machine configuration found in .argos" << std::endl;
		}



		/*
		 *  Initializing sensors and actuators
		 */
		try{
			m_pcProximitySensor = GetSensor<CCI_EPuckProximitySensor>("epuck_proximity");
			m_pcLightSensor = GetSensor<CCI_EPuckLightSensor>("epuck_light");
			m_pcGroundSensor = GetSensor<CCI_EPuckGroundSensor>("epuck_ground");
			 m_pcRabSensor = GetSensor<CCI_EPuckRangeAndBearingSensor>("epuck_range_and_bearing");
			 m_pcCameraSensor = GetSensor<CCI_EPuckOmnidirectionalCameraSensor>("epuck_omnidirectional_camera");
		} catch (CARGoSException ex) {
			LOGERR<<"Error while initializing a Sensor!\n";
		}

		try{
			m_pcWheelsActuator = GetActuator<CCI_EPuckWheelsActuator>("epuck_wheels");
			m_pcRabActuator = GetActuator<CCI_EPuckRangeAndBearingActuator>("epuck_range_and_bearing");
			m_pcLEDsActuator = GetActuator<CCI_EPuckRGBLEDsActuator>("epuck_rgb_leds");
		} catch (CARGoSException ex) {
			LOGERR<<"Error while initializing an Actuator!\n";
		}

		/*
		 * Constantly send range-and-bearing messages containing the robot integer identifier.
		 */
		if (m_pcRabActuator != NULL) {
			UInt8 data[4];
			data[0] = m_unRobotID;
			data[1] = 0;
			data[2] = 0;
			data[3] = 0;
			m_pcRabActuator->SetData(data);
		}

		argos::CRandom::CreateCategory("pseudo-reality", 0);
    m_pcRngPseudoReality = argos::CRandom::CreateRNG("pseudo-reality");
	}

	/****************************************/
	/****************************************/

	void AutoMoDeControllerPseudoReality::ControlStep() {
		/*
		 * Setting up the pseudo reality.
		 */
		if (!m_bPseudoRealitySet) {
			Real fRandomFactor = m_pcRngPseudoReality->Uniform(CRange<Real>(0, 1.0));
		//	LOG << fRandomFactor << std::endl;
			//Range and Bearing range
	    m_unRabRangeTransfoFunction = m_pcRngPseudoReality->Bernoulli(0.5);
	    if (m_fRangeDistributionPower != 0)
	      m_fAlphaRabRange = pow(fRandomFactor, m_fRangeDistributionPower);
	    else
	      m_fAlphaRabRange = 0;
	    //LOG << " rab range " << m_fAlphaRabRange << std::endl;

			//Range and Bearing bearing
	    m_cRabBearingTruncNormDist = TruncNormalDistr(m_fBearingTruncDistrMean, m_fBearingTruncDistrSd, -m_fBearingTruncDistrLimit, m_fBearingTruncDistrLimit);

			// Range and Bearing loss probability
	    Real fLoosProbabilityIncrement = m_pcRngPseudoReality->Uniform(CRange<Real>(m_fLossProbaLowerBound, m_fLossProbaUpperBound));
	    m_fLossProbability = m_fLossProbability + fLoosProbabilityIncrement;
	  //  LOG << " loss proba " << m_fLossProbability << std::endl;

			// Wheels transformation
			m_unLeftWheelTransfoFunction = m_pcRngPseudoReality->Bernoulli(0.5);
			m_unRightWheelTransfoFunction = m_pcRngPseudoReality->Bernoulli(0.5);

			m_fAlphaWheels = pow(fRandomFactor, m_fWheelsDistributionPower);
			m_fAlphaWheels = m_fAlphaWheels * m_fWheelsMaxAlpha;
			//LOG << " wheels " << m_fAlphaWheels << std::endl;

			//LOG << " prox ";
			for(size_t i=0; i<8; i++) {
				m_unProxiTransfo[i] = m_pcRngPseudoReality->Bernoulli(0.5);
				if (m_fProxiDistributionPower != 0)
					m_fAlphaProxi[i] = pow(m_pcRngPseudoReality->Uniform(CRange<Real>(0,1.0)), m_fProxiDistributionPower);
				else
					m_fAlphaProxi[i] = 0;
				//LOG << m_fAlphaProxi[i] << " ";
			}
			//LOG << std::endl;

			m_bPseudoRealitySet = true;
		}

		/*
		 * 1. Update RobotDAO
		 */
		if(m_pcRabSensor != NULL){
			const CCI_EPuckRangeAndBearingSensor::TPackets& packets = m_pcRabSensor->GetPackets();
			CCI_EPuckRangeAndBearingSensor::TPackets newPackets = m_pcRabSensor->GetPackets();
			UInt32 i = 0;
			while (i < newPackets.size()) {
				if (!m_pcRobotState->GetRandomNumberGenerator()->Bernoulli(m_fLossProbability)) {
					newPackets[i]->Range = TransformationFunction(packets[i]->Range, m_unRabRangeTransfoFunction, 100, m_fAlphaRabRange);
					newPackets[i]->Bearing = packets[i]->Bearing + CRadians((m_cRabBearingTruncNormDist()*3.1415)/180);
					i++;
				} else {
					newPackets.erase(newPackets.begin() + i);
				}
			}
			m_pcRobotState->SetRangeAndBearingMessages(newPackets);
		}
		if (m_pcGroundSensor != NULL) {
			const CCI_EPuckGroundSensor::SReadings& readings = m_pcGroundSensor->GetReadings();
			m_pcRobotState->SetGroundInput(readings);
		}
		if (m_pcLightSensor != NULL) {
			const CCI_EPuckLightSensor::TReadings& readings = m_pcLightSensor->GetReadings();
			m_pcRobotState->SetLightInput(readings);
		}
		if (m_pcProximitySensor != NULL) {
			const CCI_EPuckProximitySensor::TReadings& readings = m_pcProximitySensor->GetReadings();
			CCI_EPuckProximitySensor::TReadings newReadings = m_pcProximitySensor->GetReadings();
			for (size_t i=0; i<8; i++) {
				newReadings[i].Value = TransformationFunction(readings[i].Value, m_unProxiTransfo[i], 1, m_fAlphaProxi[i]);
			}
			m_pcRobotState->SetProximityInput(newReadings);
		}

		/*
		 * 2. Execute step of FSM
		 */
		m_pcFiniteStateMachine->ControlStep();

		/*
		 * 3. Update Actuators
		 */
		if (m_pcWheelsActuator != NULL) {
			Real signLeft = 1.0, signRight = 1.0;
			Real fLeftWheelSpeed = m_pcRobotState->GetLeftWheelVelocity();
			Real fRightWheelSpeed = m_pcRobotState->GetRightWheelVelocity();
      if (fLeftWheelSpeed < 0.0) {
        signLeft = -1.0;
      }
      if (fRightWheelSpeed < 0.0) {
        signRight = -1.0;
      }
			m_pcWheelsActuator->SetLinearVelocity(signLeft * TransformationFunction(fabs(fLeftWheelSpeed), m_unLeftWheelTransfoFunction, 16, m_fAlphaWheels), signRight * TransformationFunction(fabs(fRightWheelSpeed), m_unRightWheelTransfoFunction, 16, m_fAlphaWheels));
		}

		/*
		 * 4. Update variables and sensors
		 */
		if (m_pcRabSensor != NULL) {
			m_pcRabSensor->ClearPackets();
		}
		m_unTimeStep++;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeControllerPseudoReality::Destroy() {}

	/****************************************/
	/****************************************/

	void AutoMoDeControllerPseudoReality::Reset() {
		m_pcFiniteStateMachine->Reset();
		m_pcRobotState->Reset();
	}

	/****************************************/
	/****************************************/

	void AutoMoDeControllerPseudoReality::SetFiniteStateMachine(AutoMoDeFiniteStateMachine* pc_finite_state_machine) {
		m_pcFiniteStateMachine = pc_finite_state_machine;
		m_pcFiniteStateMachine->SetRobotDAO(m_pcRobotState);
		m_pcFiniteStateMachine->Init();
		m_bFiniteStateMachineGiven = true;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeControllerPseudoReality::LoadPseudoReality() {
	  std::ifstream iFile(m_strPseudoRealityDescriptionFile.c_str(),std::ios::in);
	  std::string strParameter;
		if (iFile.good()) {
		  while (std::getline(iFile, strParameter)) {
		    std::vector<std::string> vecTokens;
		    std::istringstream iss(strParameter);
		    std::string token;
		    while(std::getline(iss, token, ' '))   // but we can specify a different one
		      vecTokens.push_back(token);

		    // Parsing parameters
		    if (vecTokens.at(0).compare("--rangeDistributionPower") == 0) {
		      m_fRangeDistributionPower = strtof(vecTokens.at(1).c_str(), 0);
		    } else if (vecTokens.at(0).compare("--bearingTruncDistrMean") == 0) {
		      m_fBearingTruncDistrMean = strtof(vecTokens.at(1).c_str(), 0);
		    } else if (vecTokens.at(0).compare("--bearingTruncDistrSd") == 0) {
		      m_fBearingTruncDistrSd = strtof(vecTokens.at(1).c_str(), 0);
		    } else if (vecTokens.at(0).compare("--bearingTruncDistrLimit") == 0) {
		      m_fBearingTruncDistrLimit = strtof(vecTokens.at(1).c_str(), 0);
		    } else if (vecTokens.at(0).compare("--lossProbaLowerBound") == 0) {
		      m_fLossProbaLowerBound = strtof(vecTokens.at(1).c_str(), 0);
		    } else if (vecTokens.at(0).compare("--lossProbaUpperBound") == 0) {
		      m_fLossProbaUpperBound = strtof(vecTokens.at(1).c_str(), 0);
		    } else if (vecTokens.at(0).compare("--wheelsDistributionPower") == 0) {
		      m_fWheelsDistributionPower = strtof(vecTokens.at(1).c_str(), 0);
		    } else if (vecTokens.at(0).compare("--wheelsMaxAlpha") == 0) {
		      m_fWheelsMaxAlpha = strtof(vecTokens.at(1).c_str(), 0);
		    } else if (vecTokens.at(0).compare("--proxiDistributionPower") == 0) {
		      m_fProxiDistributionPower = strtof(vecTokens.at(1).c_str(), 0);
		    } else {
		      THROW_ARGOSEXCEPTION("Cannot parse pseudo reality parameter '" << vecTokens.at(0) << "'");
		    }
			}
		} else {
			THROW_ARGOSEXCEPTION("Cannot open pseudo reality description file '" << m_strPseudoRealityDescriptionFile << "'");
		}
	}

 /****************************************/
 /****************************************/

 Real AutoMoDeControllerPseudoReality::TransformationFunction(Real f_origin, UInt8 un_function, Real f_max, Real f_alpha) {
	  Real fNew;
		if (f_alpha == 0.0f) {
			return f_origin;
		} else {
		  if (un_function == 0) {
		    fNew =  ( ((1 - f_alpha) * (f_origin / f_max)) + ( f_alpha * (pow((f_origin/f_max),2)) ) ) * f_max;
		  }
		  else {
		    fNew =  ( ( -(1 - f_alpha) + sqrt( pow(1 - f_alpha, 2) + (4 * f_alpha * (f_origin/f_max))  ) ) / (2 * f_alpha) ) * f_max;
		  }
		}
	  return Max(0.0, fNew);
	}

	REGISTER_CONTROLLER(AutoMoDeControllerPseudoReality, "automode_controller_pseudo_reality");
}
