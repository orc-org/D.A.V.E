#pragma once

#include <list>
#include "Instrument.hpp"
#include "WarningLight.hpp"

using std::list;

/*
    WarningPanel
    -------------------------------
    Display that shows a bunch of indicator lights to warn the operator of any problems with DaVE
*/
class WarningPanel : public Instrument
{
private:

    list<WarningLight> _warningLights;

public:

    void Init();

    void Draw();

    void Update(float dt);

};