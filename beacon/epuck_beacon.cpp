/* Include the controller definition */
#include "epuck_beacon.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>

/****************************************/
/****************************************/

CEPuckBeacon::CEPuckBeacon() :
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_pcBaseLeds(NULL),
   m_pcLightSensors(NULL),
   m_pcGroundSensor(NULL),
   m_pcRabActuator(NULL),
   m_pcRabSensor(NULL),
   m_unTimeStep(0),
   m_unTBar(0),
   m_unTBarParam(0),
   m_unState(0),
   m_fWheelVelocity(2.5f),
   m_unMessageToSend(0)
       {}

/****************************************/
/****************************************/

void CEPuckBeacon::Init(TConfigurationNode& t_node) {
    /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the
    * XML tag of the device whose handle we want to have. For a list of
    * allowed values, type at the command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors
    * internally, on the basis of the lists provided the configuration
    * file at the <controllers><epuck_obstacleavoidance><actuators> and
    * <controllers><epuck_obstacleavoidance><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
    m_pcWheels = GetActuator<CCI_EPuckWheelsActuator>("epuck_wheels");
    m_pcBaseLeds = GetActuator<CCI_EPuckBaseLEDsActuator>("epuck_base_leds");

    m_pcProximity = GetSensor<CCI_EPuckProximitySensor>("epuck_proximity");
    m_pcLightSensors = GetSensor<CCI_EPuckLightSensor>("epuck_light");
    m_pcGroundSensor = GetSensor<CCI_EPuckGroundSensor>("epuck_ground");

    try {
        m_pcRabActuator = GetActuator<CCI_EPuckRangeAndBearingActuator>("epuck_range_and_bearing");
    } catch(CARGoSException& ex) {}

    try {
        m_pcRabSensor = GetSensor<CCI_EPuckRangeAndBearingSensor>("epuck_range_and_bearing");
    } catch(CARGoSException& ex) {}

    if(m_pcRabActuator != NULL) {
        UInt8 data[2];
        data[0] = getRobotId();
        data[1] = 0;
        m_pcRabActuator->SetData(data);
        m_pcRabActuator->Disable();
    }
    m_pcRng = CRandom::CreateRNG("argos");
    /*
    * Parse the configuration file
    *
    * The user defines this part. Here, the algorithm accepts three
    * parameters and it's nice to put them in the config file so we don't
    * have to recompile if we want to try other settings.
    */
    GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
    GetNodeAttributeOrDefault(t_node, "time", m_unTBarParam, (SInt32) 0 );
    if (m_unTBarParam == -1) {
        m_unTBar = m_pcRng->Uniform(CRange<UInt32>(400, 600));
    }
    else {
        m_unTBar = m_unTBarParam;
    }

    GetNodeAttributeOrDefault(t_node, "mes", m_unMesParam, (UInt8) 3 );
    //m_unMesParam = 1;
    if (m_unMesParam == 3) {
        m_unMessageToSend = m_pcRng->Uniform(CRange<UInt32>(0, 2))*150+10;
    }
    else if (m_unMesParam == 0) {
        m_unMessageToSend = 160;
    }
    else if (m_unMesParam == 1) {
        m_unMessageToSend = 10;
    }
}

void CEPuckBeacon::Reset() {
    m_unTimeStep = 0;
    m_unState = 0;

    if (m_unMesParam == 3) {
        m_unMessageToSend = m_pcRng->Uniform(CRange<UInt32>(0, 2))*150+10;
    }

    if (m_unTBarParam == -1) {
        m_unTBar = m_pcRng->Uniform(CRange<UInt32>(400, 600));
    }
    else {
        m_unTBar = m_unTBarParam;
    }
}

/****************************************/
/****************************************/
UInt32 CEPuckBeacon::getTBar() {
    return m_unTBar;
}


UInt32 CEPuckBeacon::getMessage() {
    return m_unMessageToSend;
}

void CEPuckBeacon::setMessage(UInt8 un_message_to_send) {
    m_unMessageToSend = un_message_to_send;
    m_unState = 0;
}

void CEPuckBeacon::DisableBeacon() {
    if(m_pcRabActuator != NULL) {
        m_pcRabActuator->Disable();
    }
    for (int i=0;i<8;i++) {
        //std::cout << m_pcLightSensors->GetReadings()[i].Value << std::endl;
        m_pcBaseLeds->SwitchLED(i,false);
    }
}


void CEPuckBeacon::SendBeacon(UInt8 un_message) {
    if(m_pcRabActuator != NULL) {
        UInt8 data[2];
        data[0] = getRobotId();
        data[1] = un_message;
        m_pcRabActuator->SetData(data);
    }
    for (int i=0;i<8;i++) {
        //std::cout << m_pcLightSensors->GetReadings()[i].Value << std::endl;
        m_pcBaseLeds->SwitchLED(i,true);
    }
}


void CEPuckBeacon::ControlStep() {
    /* Get the highest reading in front of the robot, which corresponds to the closest object */
    if (m_unTimeStep > m_unTBar && m_unState == 0) {
        SendBeacon(m_unMessageToSend);
        LOG << "becmes: " << m_unMessageToSend << std::endl;
        m_unState = 1;
    }
    // if (m_pcGroundSensor != NULL) {
    //     const CCI_EPuckGroundSensor::SReadings& readings = m_pcGroundSensor->GetReadings();
    //     LOG << readings.Left << "::" << readings.Center << "::" << readings.Right << std::endl;
    // }
    // Real decision;
    // decision = m_pcRng->Uniform(CRange<UInt32>(0,2));
    // if (decision == 2) {
    //   LOG<< "DECISION=2 !!! :" << decision << std::endl;
    // }
    m_unTimeStep++;
}

UInt32 CEPuckBeacon::getRobotId() {
    if (m_nId < 0) {
        std::string strId = GetId();
        std::string::size_type pos = strId.find_first_of("0123456789");
        std::string numero = strId.substr(pos, strId.size());
        if (!(std::istringstream(numero) >> m_nId)) {
            m_nId = 0;
        }
    }
    return m_nId;
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CEPuckBeacon, "epuck_beacon")
