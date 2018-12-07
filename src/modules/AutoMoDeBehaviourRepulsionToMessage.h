/*
 * File: AutoMoDeBehaviourRepulsionToMessage.h
 *
 */

#ifndef AUTOMODE_BEHAVIOUR_REPULSION_TO_MESSAGE_H
#define AUTOMODE_BEHAVIOUR_REPULSION_TO_MESSAGE_H

#include "AutoMoDeBehaviour.h"

namespace argos {
	class AutoMoDeBehaviourRepulsionToMessage: public AutoMoDeBehaviour {
		public:
			AutoMoDeBehaviourRepulsionToMessage();
			AutoMoDeBehaviourRepulsionToMessage(AutoMoDeBehaviourRepulsionToMessage* pc_behaviour);
			virtual ~AutoMoDeBehaviourRepulsionToMessage();

			virtual void ControlStep();
			virtual void Reset();
			virtual void ResumeStep();
			virtual void Init();

			virtual AutoMoDeBehaviourRepulsionToMessage* Clone();

		private:
			UInt8 m_unRepulsionParameter;
			UInt8 m_unRepulsiveMessage;
	};
}


#endif
