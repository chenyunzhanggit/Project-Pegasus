#include "leg_states.hpp"



using namespace std;

LEG_STATES legs_states;

LEG_STATES::LEG_STATES()
{
    Leg1.leg_states = DAMPING; 
    Leg2.leg_states = DAMPING; 
    Leg3.leg_states = DAMPING; 
    Leg4.leg_states = DAMPING; 


    // Other attributes may not be initialized.
    //Legs.Leg1.wheel_deflection = 0.0;
}



LEG_STATES::~LEG_STATES()
{
    Leg1.leg_states = DAMPING; 
    Leg2.leg_states = DAMPING; 
    Leg3.leg_states = DAMPING; 
    Leg4.leg_states = DAMPING; 

}