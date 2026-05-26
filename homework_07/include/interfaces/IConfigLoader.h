#pragma once

class IConfigLoader {

public:
    virtual int load(int source) = 0;
    virtual int getConfig() = 0;
    virtual int getAmmoParams() = 0;
    virtual ~IConfigLoader() {}
};