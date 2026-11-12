#pragma once
#include "Icon.hpp"

/*
    WarningLevel
    ---------------
    NONE -> no problem
    CAUTION  -> Something minor is out of whack or a sensor read an abnormal reading, *Proceed with Caution*
    WARNING  -> Something is wrong or a sensor reads a dangerous reading and DaVE's ability to complete the challenge is in question *Stop and Evaluate*
    CRITICAL -> Something is broken and the safety of the rover and nearby people might be threatened. *Seriously Consider Shuting Down*
*/
enum class WarningLevel { NONE, CAUTION, WARNING, CRITICAL };

/*
    Warning Light

    This is an extension of Icon
    They will be flashing lights that are used to warn the pilot of some failure or danger

    They can be clicked to turn off the alarm
*/
class WarningLight : public Icon
{

private: 
    // Add sound effects here

    float _flashTimer; // how long since the last flash
    float _flashSpeed; // how long between flashes

    WarningLevel _state;

public:

    WarningLight();

    void Caution();  // Set state to "CAUTION"
    void Warning();  // Set state to "WARNING"
    void Critical(); // Set state to "CRITICAL"

    void Draw();
    void Update(float dt);

};