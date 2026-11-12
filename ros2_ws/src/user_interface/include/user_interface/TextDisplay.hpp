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

    virtual void Draw() const;

    virtual void Update(float dt);

};