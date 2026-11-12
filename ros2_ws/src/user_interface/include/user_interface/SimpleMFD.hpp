#pragma once

#include "Instrument.hpp"
#include "Button.hpp"
#include <list>

using std::list;

class SimpleDisplay;

/*
    ButtonLayout
    -------------
    Defines where the bank of buttoms for MFDs are located
*/
enum class ButtonLayout { PERIMETER, TOP, BOTTOM, LEFT, RIGHT, TOP_LEFT, TOP_RIGHT, BOTTOM_LEFT, BOTTOM_RIGHT };

/*
    SimpleMFD
    ---------
    A special display that can be toggled to display different information
*/
class SimpleMFD : public Instrument
{
private:

    list<SimpleDisplay> _displays;

protected:

    int _buttonSpacing; // distance between buttons
    
    Rectangle _displayRectangle; // Used to tell the simple display where is is allowed to draw to (it's the destination rectangle for the display)
    
    list<Button> _buttons;

public:

    SimpleMFD();

    virtual void Init();

    virtual void Draw();

    virtual void Update(float dt);

};