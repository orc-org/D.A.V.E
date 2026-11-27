#pragma once
#include <raylib.h>
#include <unordered_map>
#include <string>

/*
    This is a singleton that will manage all of our resources for our game

    It will make use of a hashMap that will be keyed on the keys defined in the ResourceKeys namespace
*/

class ResourceManager
{
public:

    // returns a reference to the singleton (creates this instance when it is first called)
    static ResourceManager& instance() 
    {
        static ResourceManager inst;
        return inst;
    }

    ResourceManager(const ResourceManager&) = delete; // disable the copy constructor
    ResourceManager& operator=(const ResourceManager&) = delete; // disable the assignment operator

    void Load(); // load everything
    void Unload(); // unload everything

    const Texture2D& GetTexture(const std::string& name) const;
    const Image& GetImage(const std::string& name) const;

private: 
    ResourceManager() = default; //make the default constructor private to prevent instanciation

    void loadTexture(const std::string& name, const std::string& path);
    void loadImage  (const std::string& name, const std::string& path);

    std::unordered_map<std::string, Texture2D> _textures;
    std::unordered_map<std::string, Image> _images;

};

namespace RM
{
    inline ResourceManager& get() { return ResourceManager::instance(); }
}