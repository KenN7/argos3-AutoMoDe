/*
 * File: AutoMoDeConditionInvertedMessageCount.h
 *
 */

#ifndef AUTOMODE_CONDITION_INVERTED_MESSAGE_COUNT_H
#define AUTOMODE_CONDITION_INVERTED_MESSAGE_COUNT_H

#include "AutoMoDeCondition.h"

namespace argos {
	class AutoMoDeConditionInvertedMessageCount: public AutoMoDeCondition {
		public:
			AutoMoDeConditionInvertedMessageCount();
			virtual ~AutoMoDeConditionInvertedMessageCount();

			AutoMoDeConditionInvertedMessageCount(AutoMoDeConditionInvertedMessageCount* pc_condition);
			virtual AutoMoDeConditionInvertedMessageCount* Clone();

			virtual bool Verify();
			virtual void Reset();
			virtual void Init();

		private:
			Real m_fParameterEta;
			SInt8 m_unParameterXi;
            SInt8 m_unParameterMes;
	};
}

#endif
