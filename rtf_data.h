#pragma once

class ExtendedVehicleData
{
public:
    int framesDelayCountSearchBump;
    float freeLineTimer;
    struct
    {
        bool bPhysics;
        bool bRunGroundStuckFix;
        bool bFoundBump;
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