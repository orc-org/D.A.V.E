#pragma once

#include <list>
#include "Instrument.hpp"
#include "Button.hpp"

using std::list;

/*
    LaunchMenu
    -------------------------
    A menu that can be used to launch/kill ros2 nodes
*/
class LaunchMenu : Instrument
{
private:

    list<Button> _nodes; //1 button for each launchable node, if a button is pressed when a node is active, kill the node

public:

    void Init();

    void Draw() const;
    
    void Update(float dt);

};