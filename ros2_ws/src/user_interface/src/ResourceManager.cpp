#include "user_interface/ResourceManager.hpp"
#include "user_interface/ResourceKeys.hpp"
#include <stdexcept>

void ResourceManager::Unload()
{
    // Unload Textures from vram
    for (auto& [name, tex] : _textures)
        UnloadTexture(tex);

    _textures.clear();

    // unload images from RAM
    for (auto& [name, img] : _images)
        UnloadImage(img);

    _images.clear();

    TraceLog(LOG_INFO, "ResourceManager: all resources unloaded");
}

void ResourceManager::Load()
{
    ChangeDirectory(TextFormat("%s/../assets/images", GetApplicationDirectory()));

    loadTexture(RK::BACKGROUND, "Floor.png");

    loadTexture(RK::ORTHOGRAPHIC_DAVE, "Orthographic_DaVE.png");
    loadTexture(RK::ISOMETRIC_DAVE,  "Isometric_DaVE.png");

    TraceLog(LOG_INFO, "ResourceManager: loaded %d textures", (int)_textures.size());
    TraceLog(LOG_INFO, "ResourceManager: loaded %d images", (int)_images.size());
}

void ResourceManager::loadTexture(const std::string& name, const std::string& path)
{
    Texture2D tex = LoadTexture(path.c_str());

    // check if we can find the texture
    if(tex.id == 0)
        throw std::runtime_error("Failed to load texture: " + path);

    // insert the texture into our hash map
    _textures.emplace(name, std::move(tex));
}

void ResourceManager::loadImage(const std::string& name, const std::string& path)
{
    Image img = LoadImage(path.c_str());

    // check if we can find the texture
    if(img.data == 0)
        throw std::runtime_error("Failed to load image: " + path);

    // insert the texture into our hash map
    _images.emplace(name, std::move(img));
}

const Texture2D& ResourceManager::GetTexture(const std::string& name) const
{
    //we are using auto because the actual type is stupidly long to type (like in ROS2 code)
    auto it = _textures.find(name);

    // ensure that we find a texture
    if (it == _textures.end())
        throw std::runtime_error("Texture not Found: '" + name + "'");

    return it->second; // return the second value (it's a key-value pair)
}

const Image& ResourceManager::GetImage(const std::string& name) const
{
    //we are using auto because the actual type is stupidly long to type (like in ROS2 code)
    auto it = _images.find(name);

    // ensure that we find a texture
    if (it == _images.end())
        throw std::runtime_error("Image not Found: '" + name + "'");

    return it->second; // return the second value (it's a key-value pair)
}