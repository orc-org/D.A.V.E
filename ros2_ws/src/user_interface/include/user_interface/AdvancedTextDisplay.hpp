#pragma once

#include "TextDisplay.hpp"

/*
    AdvancedTextDisplay
    ------------------------
    A Text display that you can also write to
    example use case: Linux like terminal
*/
class AdvancedTextDisplay : TextDisplay
{
protected:

    void claimSOI();
    
public:
    
    virtual void Draw();

    virtual void Update(float dt);
};