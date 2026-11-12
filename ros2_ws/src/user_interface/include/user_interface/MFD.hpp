#pragma once

#include "SimpleMFD.hpp"
#include "AdvancedDisplay.hpp"

/*
    MFD
    -----------
    A Multi-Function-Display
    like the simple mfd, these can be switched to display different things at different times

    Unlike a simple mfd, full MFDs can interact with the displays and as such can call functions or services through them
*/
class MFD : public SimpleMFD
{
private:

    list<AdvancedDisplay> _displays;

    void rebindButtons();
    void claimSOI();

public: 

    virtual void Init();

    virtual void Draw() const;

    virtual void Update(float dt);
};