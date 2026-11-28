#pragma once
#include <raylib.h>
#include <string>

using std::string;

enum class IconState { OFF, ILLUMINATED };

/*
    Icon

    A small symbol that can be displayed anywhere on screen.
    Icons are meant to be little images that can be used to communicate information to the pilot.

    It will have 2 textures that it can display (so that we can have a flashing icon by switching between them)
    Flashing could be used as an indicator (ex: excessive g force) or to show that a the icon has been pressed (ex: a button)

    Example use cases:
    indicator lights -> not an urgent warning, but can notify you of something in the system (ex: battery is below %30)

*/
class Icon 
{

protected:

    Vector2 _iconPosition;
    Vector2 _iconSize;
    float _scale;

    Vector2 _labelPosition;
    string _label;
    int _fontSize;

    IconState _state;

    Texture2D* _icon;
    Texture2D* _illuminatedIcon;

    Color _hue; // might not need this

public:

    Icon();

    /*
        Init(Vector2 position, Vector2 size, Texture2D &icon, Texture2D &illuminatedIcon, Color hue = WHITE)
        -----------------------------------------------------------------------------------------------------
        Function used to setup the Icon
    */
    virtual void Init(Vector2 position, Vector2 size, Texture2D &icon, Texture2D &illuminatedIcon, Color hue = WHITE);

    /*
        Init( ... , std::string label, Vector2 labelPosition, int fontSize = 15, Color hue = WHITE)
        -------------------------------------------------------------------------------------------
        Function used to setup the Icon with an additional text label
    */
    virtual void Init(Vector2 position, Vector2 size, Texture2D &icon, Texture2D &illuminatedIcon, 
                        std::string label, Vector2 labelPosition, int fontSize = 15, Color hue = WHITE);


    virtual void Draw() const;
    virtual void Update(float dt);

    void TurnON();   // swtich icon state to "ILLUMINATED"
    void TurnOff();  // switch icon state to "OFF"

};

