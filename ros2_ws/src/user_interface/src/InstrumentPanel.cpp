#include "user_interface/InstrumentPanel.hpp"


void InstrumentPanel::Draw(RenderTexture2D& canvas) const
{
    BeginTextureMode(canvas);
        ClearBackground(BLACK);

        for(Instrument* instrument : _instruments) instrument->Draw();

        if (_state == InstrumentPanelState::EDITING_MODE) drawEditingOverlay();
         
        const char* title = "Hello World :)";
        int titleW = MeasureText(title, 60); // We use a raylib function to get the width of our text on screen
        DrawText(title, (UIConfig::BASE_W - titleW) / 4, (UIConfig::BASE_H - 60) / 4, 60, RAYWHITE);

    EndTextureMode();
}

void InstrumentPanel::Update(float dt)
{
    for(Instrument* instrument : _instruments)
    {
        instrument->Update(dt);
    }

    handleInput();
}

void InstrumentPanel::drawEditingOverlay() const
{
    DrawRectangle(0 ,0 ,UIConfig::BASE_W, UIConfig::BASE_H, ColorAlpha(PURPLE, 0.6));

    // TODO: Add logic to allow for the user to add/remove/alter instruments on screen
}

void InstrumentPanel::handleInput()
{
    // Replace these with buttons 
    if (IsKeyPressed(KEY_E)) _state = InstrumentPanelState::EDITING_MODE;
    if (IsKeyPressed(KEY_D)) _state = InstrumentPanelState::DISPLAY_MODE;
}