#ifndef STATES_H_
#define STATES_H_

namespace States
{
  enum State
  {
     Follow,
     TrackBack,
     ReturnHome,
	 Hold
  };
}
typedef States::State State;

#endif
