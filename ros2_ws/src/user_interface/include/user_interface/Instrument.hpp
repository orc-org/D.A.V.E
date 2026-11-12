#pragma once

#include <string>
#include <raylib.h>
#include <raymath.h>

using std::string;

/*
    Instrument
    --------------
    Parent object for each instrument for our User interface
*/
class Instrument
{
protected:

    bool _sensorOfInterest;
    static Instrument* _currentSOI;

    Vector2 _position;
    Vector2 _size;

    RenderTexture2D _canvas;

public:
    Instrument();

    virtual void Init(Vector2 position, Vector2 size);

    virtual void Draw() const;

    virtual void Update(float dt);
};