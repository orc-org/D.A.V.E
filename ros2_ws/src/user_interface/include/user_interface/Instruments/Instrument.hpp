#pragma once

#include <string>
#include <raylib.h>
#include <raymath.h>

using std::string;

/*
    Instrument
    --------------
    Abstract Parent object for each instrument for our User interface
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
    Instrument() = default;
    ~Instrument() = default;

    inline bool isClicked(Vector2 mousePosition)
    {
        if (mousePosition.x >= _position.x && 
            mousePosition.x <= _position.x + _size.x && 
            mousePosition.y >= _position.y && 
            mousePosition.y <= _position.y + _size.y)
            return true;
        else
            return false;
        
    }

    virtual void Init(Vector2 position, Vector2 size) = 0;

    virtual void Draw() const = 0;

    virtual void Update(float dt) = 0;
};