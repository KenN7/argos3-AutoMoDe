/*
 * File: AutoMoDeBehaviourAttractionToMessage.h
 *
 */

#ifndef AUTOMODE_BEHAVIOUR_ATTRACTION_TO_MESSAGE_H
#define AUTOMODE_BEHAVIOUR_ATTRACTION_TO_MESSAGE_H

#include "AutoMoDeBehaviour.h"

namespace argos {
	class AutoMoDeBehaviourAttractionToMessage: public AutoMoDeBehaviour {
		public:
			AutoMoDeBehaviourAttractionToMessage();
			AutoMoDeBehaviourAttractionToMessage(AutoMoDeBehaviourAttractionToMessage* pc_behaviour);
			virtual ~AutoMoDeBehaviourAttractionToMessage();

			virtual void ControlStep();
			virtual void Reset();
			virtual void ResumeStep();
			virtual void Init();

			virtual AutoMoDeBehaviourAttractionToMessage* Clone();

		private:
			UInt8 m_unAttractionParameter;
			UInt8 m_unAttractiveMessage;
	};
}

#endif
