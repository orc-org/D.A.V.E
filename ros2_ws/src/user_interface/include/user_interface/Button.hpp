#pragma once
#include "Icon.hpp"

/*
    Button

    Buttons are essentially just icons that have a function binding
    pressing a button will activate a function in the display it self.

    Since MFDs can switch between displays, Buttons will be able to be updated to activate different functions

    This is an approach for if we want to implement our own buttons, 
    RAYGUI also exists and might be worth looking into
    It is an imediate mode GUI addon for raylib (it is already included in this package, use if you wish)
*/
class Button : public Icon 
{

private:
    int _id;
    // function pointer or some other way to call a method in the display
    // sound effect for when the button is pressed.

    /*
        MouseIsOverButton()
        -----------------------------------------
        returns true is the mouse is hovering over top of the button
    */
    bool MouseIsOverButton(Vector2 mousePosition);

public:

    Button();

    /*
        ChangeButtonFunction()
        --------------------------
        method used to change the behaviour of a button.
        this is for when we swap displays on MFDs, 
        the buttons should have new icons, labels and functions to work with the nedisplay

        TODO: Also pass the new fucntion binding
    */
    void ChangeButtonFunction(Texture2D &icon, Texture2D &illuminatedIcon); 
    void Draw(); 
    void Update(float dt); 

};