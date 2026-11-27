#include "user_interface/LaunchNode.hpp"

LaunchNode::LaunchNode() : rclcpp::Node("LaunchNode")
{
	std::cout << "Great Success" << std::endl;

    DisplayGUI(); // DELETE THIS AND MAKE IT RUN ON IT'S OWN THREAD
}

void LaunchNode::DisplayGUI()
{
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(UIConfig::BASE_W, UIConfig::BASE_H, "DaVE GUI");
    SetTargetFPS(60);
    SetExitKey(KEY_NULL);

    /*
        some quality of life stuff
        we can resize/reposition the window in the code if we want
    */
    // Vector2 pos = GetMonitorPosition(0);
    // SetWindowPosition(pos.x + 320, pos.y + 180);
    // SetWindowSize(1920, 1080);

    /* 
        create render texture to change how our window resizing works
        without this, the size of the map will change (in stead of zooming in/out, we'd be changing the dimensions of the world)
    */
    RenderTexture2D canvas = LoadRenderTexture(UIConfig::BASE_W, UIConfig::BASE_H);

    /*
        use a filter to smooth out the image so the pixels don't stand out when zoomed in 
        there are several different filters available for use, we will be using the bilinear filter
    */
    SetTextureFilter(canvas.texture, TEXTURE_FILTER_BILINEAR);

    InstrumentPanel instrumentPanel;

    while (!WindowShouldClose())
    {
        instrumentPanel.Update(GetFrameTime());
        instrumentPanel.Draw(canvas); // draw onto render texture

        // math related to window resizing
        // how much we must scale our render texture to fill the window
        float scale = std::min(
            (float)GetScreenWidth()  / UIConfig::BASE_W,
            (float)GetScreenHeight() / UIConfig::BASE_H
        ); 

        // How much of an offset we need to keep our instrumentPanel centered in the window (needed if aspect ratio changes)
        float offsetX = (GetScreenWidth()  - UIConfig::BASE_W * scale) * 0.5f;
        float offsetY = (GetScreenHeight() - UIConfig::BASE_H * scale) * 0.5f;

        // setting up the source and destination rectangle for drawing the render texture
        Rectangle src  = { 0, 0, (float)UIConfig::BASE_W, -(float)UIConfig::BASE_H };
        Rectangle dest = { offsetX, offsetY, UIConfig::BASE_W * scale, UIConfig::BASE_H * scale };

        BeginDrawing();
            ClearBackground(BLACK);
            DrawTexturePro(canvas.texture, src, dest, {0, 0}, 0.0f, WHITE);
        EndDrawing();
    }

    UnloadRenderTexture(canvas);
    RM::get().Unload();
    CloseWindow();
}

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaunchNode>());
    rclcpp::shutdown();
    return 0;
}