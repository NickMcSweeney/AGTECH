#ifndef STATES_H_
#define STATES_H_

namespace States
{
  enum State
  {
    Follow,
    TrackBack,
    ReturnHome,
    Hold,
    Collect,
    Manual,
    Drop,
    Next
  };
}
typedef States::State State;

#endif
