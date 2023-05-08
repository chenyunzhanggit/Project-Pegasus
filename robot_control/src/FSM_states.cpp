#include "FSM_states.hpp"

using namespace std;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;


FSM_STATES::FSM_STATES()
{
    
    //state = PASSIVE;
    state.data = PASSIVE;
    
    

}


FSM_STATES::~FSM_STATES()
{
    state.data = PASSIVE;
}

