#pragma once

#include <string>
#include <navigation/known_map.hpp>

namespace mujoco_backend
{

class KnownMapWorld
{
public:
    KnownMapWorld(const std::string& robot_model, const navigation::KnownMap& scenario);
    ~KnownMapWorld();
    KnownMapWorld(const KnownMapWorld&) = delete;
    KnownMapWorld& operator=(const KnownMapWorld&) = delete;
    const std::string& path() const;

private:
    std::string path_;
};

}
