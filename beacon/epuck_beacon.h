/*
 * AUTHOR: Ken Hasselmann <khasselm@ulb.ac.be>
 *
 */

#ifndef EPUCK_BEACON_H
#define EPUCK_BEACON_H

/*
 * Include some necessary headers.
 */
 #include <argos3/core/control_interface/ci_controller.h>
 #include <argos3/core/utility/math/range.h>
 #include <argos3/core/utility/math/vector2.h>
 #include <argos3/core/utility/math/rng.h>

 #include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_wheels_actuator.h>
 #include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_range_and_bearing_sensor.h>
 #include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_range_and_bearing_actuator.h>
 #include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_rgb_leds_actuator.h>
 #include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_proximity_sensor.h>
 #include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_light_sensor.h>
 #include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_ground_sensor.h>
 #include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_omnidirectional_camera_sensor.h>
 #include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_base_leds_actuator.h>




/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CEPuckBeacon : public CCI_Controller {

public:

   /* Class constructor. */
   CEPuckBeacon();

   /* Class destructor. */
   virtual ~CEPuckBeacon() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><epuck_obstacleavoidance_controller> section.
    */
   virtual void Init(TConfigurationNode& t_node);

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Reset();

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Destroy() {}

   /*
   * Func to disable sending messages with the RAB actuator
   */
   void DisableBeacon();

   void SendBeacon(UInt8 message);

   void setMessage(UInt8 un_message_to_send);

   UInt32 getRobotId();

   UInt32 getTBar();

   UInt32 getMessage();

private:

   /* Pointer to the differential steering actuator */
   CCI_EPuckWheelsActuator* m_pcWheels;
   /* Pointer to the e-puck proximity sensor */
   CCI_EPuckProximitySensor* m_pcProximity;
   CCI_EPuckBaseLEDsActuator* m_pcBaseLeds;
   CCI_EPuckLightSensor* m_pcLightSensors;
   CCI_EPuckGroundSensor* m_pcGroundSensor;
   /*
    * Pointer to the robot range-and-bearing actuator.
    */
   CCI_EPuckRangeAndBearingActuator* m_pcRabActuator;
   /*
    * Pointer to the robot range-and-bearing sensor.
    */
   CCI_EPuckRangeAndBearingSensor* m_pcRabSensor;
   CRandom::CRNG* m_pcRng;
   /*
    * The following variables are used as parameters for the
    * algorithm. You can set their value in the <parameters> section
    * of the XML configuration file, under the
    * <controllers><epuck_obstacleavoidance_controller> section.
    */
   /* Wheel speed. */

   UInt32 m_unTimeStep;
   UInt32 m_unTBar;
   SInt32 m_unTBarParam;
   UInt8 m_unState;
   Real m_fWheelVelocity;
   SInt32 m_nId;
   UInt8 m_unMessageToSend;
   UInt8 m_unMesParam;

};

#endif
