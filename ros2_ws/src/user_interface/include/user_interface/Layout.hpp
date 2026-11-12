#pragma once

#include <vector>
#include "AdvancedTextDisplay.hpp"
#include "Instrument.hpp"
#include "LaunchMenu.hpp"
#include "MFD.hpp"
#include "SimpleMFD.hpp"
#include "TextDisplay.hpp"
#include "WarningPanel.hpp"

using std::vector;

/*
    Layout
    -----------------
    An object to allow us to define multiple layouts for the different displays and switch between them
*/
class Layout
{
private:

    vector<Instrument> _instruments;

    void addInstrument();

    void removeInstrument();

    void loadLayout();

    void saveLayout();

    void editLayout();

public:

    void Init();

    void Draw();

    void Update(float dt);

};