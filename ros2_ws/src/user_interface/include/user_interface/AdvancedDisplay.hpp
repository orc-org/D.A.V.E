#pragma once

#include "SimpleDisplay.hpp"

/*
    AdvancedDisplay
    --------------------
    Base Display for use in MFDs.
    Unlike simple displys, these define functions that are specific to the the display (i.e. Create waypoint for the NAV display)

    Extend this for each individual display
*/
class AdvancedDisplay : SimpleDisplay
{
protected:

public:

    virtual void Init();

    virtual void Draw();

    virtual void Update(float dt);
};