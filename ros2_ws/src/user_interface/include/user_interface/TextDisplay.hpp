#pragma once

#include "Instrument.hpp"

/*
    TestDisplay
    --------------------------------
    A simple read only text display
*/
class TextDisplay : public Instrument
{
protected:

    string label;

public:

    virtual void Draw();

    virtual void Update(float dt);

};