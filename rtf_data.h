#pragma once

class ExtendedVehicleData
{
public:
    int framesDelayCountSearchBump;
    float freeLineTimer;
    struct
    {
        bool bPhysics : 1;
        bool bRunGroundStuckFix : 1;
        bool bFoundBump : 1;
    } flags;

    inline void Reset()
    {
        framesDelayCountSearchBump = 0;
        freeLineTimer = 0.0f;
        flags.bPhysics = false;
        flags.bRunGroundStuckFix = true;
        flags.bFoundBump = false;
    }
};