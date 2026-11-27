#pragma once

#include <vector>
#include "Instruments/AdvancedTextDisplay.hpp"
#include "Instruments/Instrument.hpp"
#include "Instruments/LaunchMenu.hpp"
#include "Instruments/MFD.hpp"
#include "Instruments/SimpleMFD.hpp"
#include "Instruments/TextDisplay.hpp"
#include "Instruments/WarningPanel.hpp"
#include "UIConfig.hpp"

using std::vector;

enum class InstrumentPanelState { DISPLAY_MODE, EDITING_MODE };

/*
    InstrumentPanel
    -----------------
    An object to allow us to define multiple layouts for the different displays and switch between them
*/
class InstrumentPanel
{
private:

    vector<Instrument*> _instruments;
    InstrumentPanelState _state;

    void addInstrument(Instrument* instrument);

    void removeInstrument(Vector2 mousePosition);

    void loadLayout();

    void saveLayout();

    void editLayout();

    void drawEditingOverlay() const;

    void handleInput();

public:

    void Draw(RenderTexture2D& canvas) const;

    void Update(float dt);

};