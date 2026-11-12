#pragma once

#include <string>
#include <raylib.h>
#include <raymath.h>

using std::string;

/*
    SimpleDisplay
    ----------------
    Base display object for use in simple MFDs

    Extend for each individual display
*/
class SimpleDisplay
{
protected:

    string _name;

    Rectangle* _destinationRectangle;

    RenderTexture2D _canvas; // the "surface" that the draw method will

public:

    virtual void Init();

    virtual void Draw();

    virtual void Update(float dt);

};