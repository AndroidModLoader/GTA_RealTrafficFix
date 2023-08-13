#include <mod/amlmod.h>
#include <mod/logger.h>
#include <mod/config.h>
#include <ctime>

#ifdef AML32
    #include "GTASA_STRUCTS.h"
    #define BYVER(__for32, __for64) (__for32)
#else
    #include "GTASA_STRUCTS_210.h"
    #define BYVER(__for32, __for64) (__for64)
#endif

#include "rtf_data.h"

MYMODCFG(net.juniordjjr.rusjj.rtf, RealTrafficFix, 1.1, JuniorDjjr & RusJJ)
BEGIN_DEPLIST()
    ADD_DEPENDENCY_VER(net.rusjj.aml, 1.0.2.1)
END_DEPLIST()

uintptr_t pGTASA;
void* hGTASA;

// GTA Vars
#ifdef AML32
CPool<CVehicle, CHeli> **ms_pVehiclePool;
#else
CPool<CVehicle, CPlane> **ms_pVehiclePool;
#endif
CPool<CObject, CCutsceneObject> **ms_pObjectPool;
CCamera *TheCamera;
float *ms_fTimeStep;
CEntity **pIgnoreEntity;

// GTA Funcs
float (*FindGroundZFor3DCoord)(float, float, float, bool*, CEntity**);
void (*PlaceAutomobileOnRoadProperly)(CVehicle*); // Originally uses CAutomobile*
CPlayerPed* (*FindPlayerPed)(int);
CColModel* (*GetColModel)(CEntity*);
void (*SwitchVehicleToRealPhysics)(CVehicle*);
bool (*IsLawEnforcementVehicle)(CVehicle*);
bool (*HasCarStoppedBecauseOfLight)(CVehicle*); // Originally uses CAutomobile*
bool (*ProcessLineOfSight)(CVector const&, CVector const&, CColPoint &, CEntity *&, bool, bool, bool, bool, bool, bool, bool, bool);

// OWN Vars
static constexpr float fMagic = 50.0f / 30.0f;
CPool<ExtendedVehicleData> *ms_pVehicleExtendedPool = NULL;

// OWN Configs
int HornsThreshold;
float CruiseSpeed, CruiseMinSpeed, CruiseMaxSpeed, TurningSpeedDecrease, FrontMultDist, HeightDiffLimit, CheckGroundHeight, FinalGroundHeight, SidesSpeedOffsetDiv, ObstacleSpeedDecrease, Acceleration, DeAcceleration, BicycleSpeedMult, BicycleCruiseMaxSpeed;
bool DoFixGroundStuck, UseBikeLogicOnCars, UseBikeLogicOnBicycles, BicyclesDontStopForRed, OnlyInNormal, CheckObjects;

// OWN Funcs
inline ExtendedVehicleData* GetExtData(CVehicle* veh)
{
    if(!ms_pVehicleExtendedPool)
    {
        if(!*ms_pVehiclePool) return NULL;
        auto size = (*ms_pVehiclePool)->m_nSize;
        ms_pVehicleExtendedPool = new CPool<ExtendedVehicleData>(size, "ExtendedVehicleData");
        ms_pVehicleExtendedPool->m_nFirstFree = size;
        for(int i = 0; i < size; ++i) ms_pVehicleExtendedPool->m_byteMap[i].bEmpty = false;
    }
    return ms_pVehicleExtendedPool->GetAt((*ms_pVehiclePool)->GetIndex(veh));
}
inline void Clamp(int& val, int min, int max)
{
    if(val > max) val = max;
    else if(val < min) val = min;
}
inline void Clamp(float& val, float min, float max)
{
    if(val > max) val = max;
    else if(val < min) val = min;
}
inline int RandomInt(int min, int max)
{
    int r = max - min + 1;
    return min + rand() % r;
}
inline float RandomFloat(float min, float max)
{
    float r = (float)rand() / (float)RAND_MAX;
    return min + r * (max - min);
}
inline CVector GetWorldCoordWithOffset(CEntity *entity, CVector& v)
{
    auto& m = *entity->m_matrix;
    return entity->m_matrix->pos + CVector
    {
        m.GetRight().x * v.x + m.GetForward().x * v.y + m.GetUp().x * v.z,
        m.GetRight().y * v.x + m.GetForward().y * v.y + m.GetUp().y * v.z,
        m.GetRight().z * v.x + m.GetForward().z * v.y + m.GetUp().z * v.z,
    };
}
inline bool ReposVehicle(CVehicle* vehicle)
{
    float randomDiff;
    bool foundGround;
    float newZ;
    CVector coord = vehicle->GetPosition();
    for (int i = 0; i < 2; i++)
    {
        randomDiff = RandomFloat(-0.5f, 0.5f);
        
        coord.x += randomDiff;
        coord.y += randomDiff;
        coord.z += 2.0f;
        newZ = FindGroundZFor3DCoord(coord.x, coord.y, coord.z, &foundGround, NULL);
        if (foundGround)
        {
            coord.z = newZ + 0.5f;
            vehicle->Teleport(coord);
            PlaceAutomobileOnRoadProperly(vehicle);
            return true;
        }
    }
    return false;
}
inline bool ThereIsObjectHere(CVector coord, float radius, int modelStart, int modelEnd)
{
    auto pool = *ms_pObjectPool;
    auto poolSize = pool->m_nSize;
    for (int index = 0; index < poolSize; ++index)
    {
        if (auto obj = pool->GetAt(index))
        {
            if (obj->m_nModelIndex >= modelStart && obj->m_nModelIndex <= modelEnd && (obj->GetPosition() - coord).Magnitude() <= radius) return true;
        }
    }
    return false;
}
inline bool FixGroundStuck_IsValid(CVector coord, float maxZ)
{
    bool foundGround;
    coord.z += maxZ;
    float groundZ = FindGroundZFor3DCoord(coord.x, coord.y, coord.z, &foundGround, NULL);
    if (foundGround) return true;
    return false;
}
inline bool FixGroundStuck(CVehicle *vehicle, CVector *modelMin, CVector *modelMax)
{
    CVector offset;
    CVector coord;
    float groundZ;
    bool foundGround;
    offset = { 0.0, (modelMin->y * 0.9f), 0.0f };
    coord = GetWorldCoordWithOffset(vehicle, offset);
    groundZ = FindGroundZFor3DCoord(coord.x, coord.y, coord.z, &foundGround, NULL);
    if (!foundGround && FixGroundStuck_IsValid(coord, modelMax->z))
    {
        if (ReposVehicle(vehicle)) return true;
        return false;
    }
    offset = { 0.0, (modelMax->y * 0.9f), 0.0f };
    coord = GetWorldCoordWithOffset(vehicle, offset);
    groundZ = FindGroundZFor3DCoord(coord.x, coord.y, coord.z, &foundGround, NULL);
    return (!foundGround && FixGroundStuck_IsValid(coord, modelMax->z) && ReposVehicle(vehicle));
}
inline void GetEntityDimensions(CEntity *entity, CVector *outCornerA, CVector *outCornerB)
{
    CColModel *colModel = GetColModel(entity);
    outCornerA->x = colModel->m_boxBound.m_vecMin.x;
    outCornerA->y = colModel->m_boxBound.m_vecMin.y;
    outCornerA->z = colModel->m_boxBound.m_vecMin.z;
    outCornerB->x = colModel->m_boxBound.m_vecMax.x;
    outCornerB->y = colModel->m_boxBound.m_vecMax.y;
    outCornerB->z = colModel->m_boxBound.m_vecMax.z;
}
inline float NormalizeCruiseSpeed(float newSpeed)
{
    if (newSpeed < CruiseMinSpeed) return CruiseMinSpeed;
    else if (newSpeed > CruiseMaxSpeed) return CruiseMaxSpeed;
    return newSpeed;
}
inline void ProcessRTFVehicle(CVehicle* vehicle)
{
    if (!vehicle->m_pDriver || vehicle->m_nCreateBy == eVehicleCreatedBy::MISSION_VEHICLE || vehicle->m_pDriver->IsPlayer()) return;

    auto subClass = vehicle->m_nVehicleSubType;
    bool isBike = (subClass == VEHICLE_TYPE_BIKE || subClass == VEHICLE_TYPE_BMX);
    if (subClass != VEHICLE_TYPE_AUTOMOBILE &&
        subClass != VEHICLE_TYPE_MTRUCK &&
        subClass != VEHICLE_TYPE_QUAD &&
        !isBike) return;

    auto& xdata = *GetExtData(vehicle);
    float curZOffset = 0.0f;
    float fNewCruiseSpeed = CruiseSpeed;

    // Cruise default speed
    vehicle->m_AutoPilot.CruiseSpeed = (uint8_t)fNewCruiseSpeed;

    // Use real physics
    if (xdata.flags.bPhysics == false)
    {
        if(subClass == VEHICLE_TYPE_AUTOMOBILE ||
           subClass == VEHICLE_TYPE_BMX ||
           subClass == VEHICLE_TYPE_MTRUCK)
        {
            SwitchVehicleToRealPhysics(vehicle);
            xdata.flags.bPhysics = true;
        }
    }

    CVector modelMin;
    CVector modelMax;
    GetEntityDimensions(vehicle, &modelMin, &modelMax);

    // Ground stuck
    if (xdata.flags.bRunGroundStuckFix == true && DoFixGroundStuck)
    {
        if (((vehicle->m_fMovingSpeed * (*ms_fTimeStep * fMagic)) < 0.5f) && FixGroundStuck(vehicle, &modelMin, &modelMax))
        {
            // There was nothing :p
        }
    }

    // Is valid driving
    if(!((isBike || vehicle->GetNumContactWheels() > 2) && ((vehicle->m_fMovingSpeed * (*ms_fTimeStep * fMagic)) > 0.01f)))
    {
        xdata.freeLineTimer = 0.0f;
        return;
    }

    // I can wait
    if(!(vehicle->m_AutoPilot.pTargetEntity == NULL && (!vehicle->vehicleFlags.bSirenOrAlarm || vehicle->m_nModelIndex == 423))) return;

    float speed = vehicle->m_vecMoveSpeed.Magnitude() * 50.0f * 3.6f;
    float distanceToCam = DistanceBetweenPoints(TheCamera->GetPosition(), vehicle->GetPosition());

    // Looks like it isn't stucked on the ground
    if (speed > 10.0f && distanceToCam < 30.0f) xdata.flags.bRunGroundStuckFix = false;

    // Disable exagerated driving style
    if (vehicle->m_AutoPilot.DrivingMode == DRIVING_STYLE_PLOUGH_THROUGH && !IsLawEnforcementVehicle(vehicle))
    {
        vehicle->m_AutoPilot.DrivingMode = DRIVING_STYLE_AVOID_CARS;
    }

    // Make use of driving style "6" for cars and bicycles
    if (vehicle->m_AutoPilot.DrivingMode <= DRIVING_STYLE_AVOID_CARS ||
        vehicle->m_AutoPilot.DrivingMode == DRIVING_STYLE_STOP_FOR_CARS_IGNORE_LIGHTS)
    {
        if (isBike)
        {
            if (subClass == VEHICLE_TYPE_BMX)
            {
                if (UseBikeLogicOnBicycles) vehicle->m_AutoPilot.DrivingMode = DRIVING_STYLE_AVOID_CARS_STOP_FOR_PEDS_OBEY_LIGHTS;
                if (BicyclesDontStopForRed) vehicle->m_AutoPilot.DrivingMode = DRIVING_STYLE_PLOUGH_THROUGH;
            }
        }
        else
        {
            if (UseBikeLogicOnCars)
            {
                if (HasCarStoppedBecauseOfLight(vehicle)) vehicle->m_AutoPilot.DrivingMode = DRIVING_STYLE_STOP_FOR_CARS;
                else vehicle->m_AutoPilot.DrivingMode = DRIVING_STYLE_AVOID_CARS_STOP_FOR_PEDS_OBEY_LIGHTS;
            }
        }
    }

    // Only in normal behaviour
    if (OnlyInNormal && vehicle->m_AutoPilot.DrivingMode != DRIVING_STYLE_STOP_FOR_CARS) return;

    // Check reverse gear
    if (vehicle->m_nCurrentGear <= 0) return;

    // Decrease speed turning
    float absSteerAngle = fabsf(vehicle->m_fSteerAngle);
    if (absSteerAngle > 0.001)
    {
        float turningDecrease = TurningSpeedDecrease * absSteerAngle;
        fNewCruiseSpeed = NormalizeCruiseSpeed(fNewCruiseSpeed - turningDecrease);
    }

    float speedFactor = speed / 40.0f;
    if (speedFactor > 1.0f) speedFactor = 1.0f;

    float dist = vehicle->m_vecMoveSpeed.Magnitude() * FrontMultDist;
    if (dist < 1.0f) dist = 1.0f;

    bool foundGround = false, forceObstacle = false;
    float newZ, steerBackOffset;
    CVector offsetA[3], coordA[3], coordB[3], offset;
    float steerOffset = vehicle->m_fSteerAngle * (-20.0f * speedFactor);

    float frontHeight = (modelMin.z * 0.4f);
    if (frontHeight > 1.0f) frontHeight = 1.0f;
    else if (frontHeight < -1.0f) frontHeight = -1.0f;

    float frontHeightBonus = 0.15f;
    if (isBike) frontHeightBonus = 0.5f;

    float heightDiffLimit = HeightDiffLimit;
    if (isBike) heightDiffLimit *= 2.0f;

    offset = { 0.0, modelMax.y, (frontHeight + frontHeightBonus) };
    coordA[0] = GetWorldCoordWithOffset(vehicle, offset);
    offset = { steerOffset, (modelMax.y + dist), ((frontHeight / 1.5f) + (frontHeightBonus * 2.0f)) };
    coordB[0] = GetWorldCoordWithOffset(vehicle, offset);
    newZ = FindGroundZFor3DCoord(coordB[0].x, coordB[0].y, coordB[0].z + CheckGroundHeight, &foundGround, NULL);

    if (fabsf((newZ - coordB[0].z)) > heightDiffLimit) forceObstacle = true; else coordB[0].z = newZ + FinalGroundHeight;

    if (!isBike)
    {
        // L
        steerBackOffset = (modelMax.y * vehicle->m_fSteerAngle);
        if (steerBackOffset < 0.0f) steerBackOffset = 0.0f;
        offsetA[1] = { modelMin.x, modelMax.y - steerBackOffset, frontHeight + frontHeightBonus };
        coordA[1] = GetWorldCoordWithOffset(vehicle, offsetA[1]);

        offset = { modelMin.x - (speed / SidesSpeedOffsetDiv) + steerOffset, (modelMax.y + dist), (frontHeight / 1.5f) };
        coordB[1] = GetWorldCoordWithOffset(vehicle, offset);
        newZ = FindGroundZFor3DCoord(coordB[1].x, coordB[1].y, coordB[1].z + CheckGroundHeight, &foundGround, NULL);
        if (fabsf((newZ - coordB[1].z)) > HeightDiffLimit) forceObstacle = true; else coordB[1].z = newZ + FinalGroundHeight;

        // R
        steerBackOffset = (modelMax.y * vehicle->m_fSteerAngle);
        if (steerBackOffset > 0.0f) steerBackOffset = 0.0f;
        offset = { modelMax.x, modelMax.y + steerBackOffset, frontHeight + frontHeightBonus };
        coordA[2] = GetWorldCoordWithOffset(vehicle, offset);

        offset = { modelMax.x + (speed / SidesSpeedOffsetDiv) + steerOffset, (modelMax.y + dist), (frontHeight / 1.5f) };
        coordB[2] = GetWorldCoordWithOffset(vehicle, offset);
        newZ = FindGroundZFor3DCoord(coordB[2].x, coordB[2].y, coordB[2].z + CheckGroundHeight, &foundGround, NULL);
        if (fabsf((newZ - coordB[2].z)) > HeightDiffLimit) forceObstacle = true; else coordB[2].z = newZ + FinalGroundHeight;
    }

    float obstacleDistFactor;
    bool bObstacle = false;
    *pIgnoreEntity = vehicle;
    CColPoint outColPoint;
    CEntity *outEntityC = NULL, *outEntityL = NULL, *outEntityR = NULL;
    if (forceObstacle)
    {
        obstacleDistFactor = 1.0f;
        goto label_force_obstacle;
    }

    if (isBike)
    {
        if (ProcessLineOfSight(coordA[0], coordB[0], outColPoint, outEntityC, 1, 1, 1, CheckObjects, 0, 0, 0, 0)) goto label_obstacle;
        goto label_force_no_obstacle;
    }

    if (ProcessLineOfSight(coordA[0], coordB[0], outColPoint, outEntityC, 1, 1, 1, CheckObjects, 0, 0, 0, 0) ||
        ProcessLineOfSight(coordA[1], coordB[1], outColPoint, outEntityL, 1, 1, 1, CheckObjects, 0, 0, 0, 0) ||
        ProcessLineOfSight(coordA[2], coordB[2], outColPoint, outEntityR, 1, 1, 1, CheckObjects, 0, 0, 0, 0))
    {
      label_obstacle:
        obstacleDistFactor = DistanceBetweenPoints(vehicle->GetPosition(), outColPoint.m_vecPoint) / 20.0f;
        if (obstacleDistFactor > 1.0f) obstacleDistFactor = 1.0f;
        obstacleDistFactor = 1.0f - obstacleDistFactor;

      label_force_obstacle:
        xdata.freeLineTimer -= ((obstacleDistFactor * speedFactor) * (*ms_fTimeStep / fMagic));
        if (xdata.freeLineTimer < 0.0f) xdata.freeLineTimer = 0.0f;

        fNewCruiseSpeed = NormalizeCruiseSpeed((float)(vehicle->m_AutoPilot.CruiseSpeed) - ((ObstacleSpeedDecrease * obstacleDistFactor) * (*ms_fTimeStep / fMagic)));
    }
    else
    {
      label_force_no_obstacle:
        if(0)
        {
            // Bumps part here?
        }
        else
        {
            if (vehicle->m_fGasPedal >= 0.05f && !vehicle->vehicleFlags.bParking) // Accelerate
            {
                xdata.freeLineTimer += (Acceleration * (*ms_fTimeStep / fMagic));
                if (xdata.freeLineTimer > 1.0f) xdata.freeLineTimer = 1.0f;
                float freeLineSpeedIncrease = (CruiseMaxSpeed - CruiseSpeed) * xdata.freeLineTimer;
                fNewCruiseSpeed = NormalizeCruiseSpeed(fNewCruiseSpeed + freeLineSpeedIncrease);
            }
            else // I don't want to accelerate
            {
                xdata.freeLineTimer -= (DeAcceleration * (*ms_fTimeStep / fMagic));
                if (xdata.freeLineTimer < 0.0f) xdata.freeLineTimer = 0.0f;
            }
        }
    }

    if (subClass == VEHICLE_TYPE_BMX)
    {
        fNewCruiseSpeed *= BicycleSpeedMult;
        if (fNewCruiseSpeed > BicycleCruiseMaxSpeed) fNewCruiseSpeed = BicycleCruiseMaxSpeed;
    }
    vehicle->m_AutoPilot.CruiseSpeed = (uint8_t)fNewCruiseSpeed;
}

// Hooks
DECL_HOOKv(AutomobileHorn, CAutomobile* self)
{
    if(!RandomInt(0, HornsThreshold)) AutomobileHorn(self);
}
DECL_HOOKv(BikeHorn, CBike* self)
{
    if(!RandomInt(0, HornsThreshold)) BikeHorn(self);
}
DECL_HOOKv(VehicleRender, CVehicle* self)
{
    ProcessRTFVehicle(self);
    VehicleRender(self);
}
DECL_HOOKv(VehicleDestroy, CVehicle* self)
{
    GetExtData(self)->Reset();
    VehicleDestroy(self);
}

// Patch
uintptr_t VehicleChangesLane_BackTo, SetNewCarLane_BackTo;
extern "C" int VehicleChangesLane_Patch(CVehicle* vehicle)
{
    auto player = FindPlayerPed(-1);
    CVehicle* playerVehicle;
    if(player && (playerVehicle = player->m_pVehicle))
    {
        float playerSpeed = playerVehicle->m_vecMoveSpeed.Magnitude() * 50.0f * 3.6f;
        if (playerSpeed > 80.0f || (vehicle->GetPosition() - playerVehicle->GetPosition()).Magnitude() <= 20.0f)
        {
            return vehicle->m_AutoPilot.NewLane;
        }
    }
    
    auto curLine = vehicle->m_AutoPilot.NewLane;
    if(rand() % 1) --vehicle->m_AutoPilot.NewLane;
    else ++vehicle->m_AutoPilot.NewLane;

    return vehicle->m_AutoPilot.NewLane;
}
extern "C" void SetNewCarLane_Patch(CVehicle* vehicle, int magicValue)
{
    if((vehicle->vehicleFlags.bIsBig || vehicle->vehicleFlags.bIsBus || vehicle->m_pHandling->Transmission.m_fMaxVelocity < 0.62f) &&
       (!vehicle->vehicleFlags.bIsLawEnforcer && !vehicle->vehicleFlags.bIsAmbulanceOnDuty && !vehicle->vehicleFlags.bIsFireTruckOnDuty))
    {
        vehicle->m_AutoPilot.OldLane = 1;
        vehicle->m_AutoPilot.NewLane = 1;
        vehicle->m_AutoPilot.bAlwaysInSlowLane = true;
    }
    else
    {
        int __rand = rand();
        vehicle->m_AutoPilot.OldLane = __rand % magicValue;
        vehicle->m_AutoPilot.NewLane = __rand % magicValue;
    }
}
#ifdef AML32
__attribute__((optnone)) __attribute__((naked)) void VehicleChangesLane(void)
{
    asm volatile(
        "PUSH {R1-R11}\n"
        "LDR R0, [SP, #0x28]\n"
        "BL VehicleChangesLane_Patch\n"
        "PUSH {R0}\n"
    );

    asm volatile(
        "MOV R12, %0\n"
        "POP {R0}\n"
        "POP {R1-R11}\n"
        "BX R12\n"
    :: "r" (VehicleChangesLane_BackTo));
}
__attribute__((optnone)) __attribute__((naked)) void SetNewCarLane(void)
{
    asm volatile(
        "PUSH {R0-R11}\n"
        "LDR R0, [SP, #0x2C]\n"
        "LDR R1, [SP, #0x14]\n"
        "BL SetNewCarLane_Patch\n"
    );

    asm volatile(
        "MOV R12, %0\n"
        "POP {R0-R11}\n"
        "BX R12\n"
    :: "r" (SetNewCarLane_BackTo));
}
#else
__attribute__((optnone)) __attribute__((naked)) void VehicleChangesLane(void)
{
    asm volatile("MOV X0, X19\nBL VehicleChangesLane_Patch");
    asm volatile("MOV X16, %0\n" :: "r"(VehicleChangesLane_BackTo));
    asm("MOV W8, W0\nBR X16");
}
__attribute__((optnone)) __attribute__((naked)) void SetNewCarLane(void)
{
    asm volatile("MOV X0, X11\nMOV W1, W22\nBL SetNewCarLane_Patch");
    asm volatile("MOV X16, %0\n" :: "r"(SetNewCarLane_BackTo));
    asm("BR X16");
}
#endif


// Main
extern "C" void OnModLoad()
{
    logger->SetTag("RealTrafficFix");

    pGTASA = aml->GetLib("libGTASA.so");
    hGTASA = aml->GetLibHandle("libGTASA.so");
    
    HornsThreshold = cfg->GetInt("HornsThreshold", 2000, "Settings");
    Clamp(HornsThreshold, 0, 2000);

    CruiseSpeed = cfg->GetFloat("CruiseSpeed", 15.0f, "Settings");
    CruiseMinSpeed = cfg->GetFloat("CruiseMinSpeed", 3.0f, "Settings");
    CruiseMaxSpeed = cfg->GetFloat("CruiseMaxSpeed", 30.0f, "Settings");
    TurningSpeedDecrease = cfg->GetFloat("TurningSpeedDecrease", 15.0f, "Settings");
    FrontMultDist = cfg->GetFloat("FrontMultDist", 50.0f, "Settings");
    HeightDiffLimit = cfg->GetFloat("HeightDiffLimit", 4.0f, "Settings");
    CheckGroundHeight = cfg->GetFloat("CheckGroundHeight", 1.0f, "Settings");
    FinalGroundHeight = cfg->GetFloat("FinalGroundHeight", 0.7f, "Settings");
    SidesSpeedOffsetDiv = cfg->GetFloat("SidesSpeedOffsetDiv", 70.0f, "Settings");
    ObstacleSpeedDecrease = cfg->GetFloat("ObstacleSpeedDecrease", 50.0f, "Settings");
    Acceleration = cfg->GetFloat("Acceleration", 0.005f, "Settings");
    DeAcceleration = cfg->GetFloat("Desacceleration", 0.01f, "Settings");
    BicycleSpeedMult = cfg->GetFloat("BicycleSpeedMult", 0.4f, "Settings");
    BicycleCruiseMaxSpeed = cfg->GetFloat("BicycleCruiseMaxSpeed", 10.0f, "Settings");
    DoFixGroundStuck = cfg->GetBool("FixGroundStuck", true, "Settings");
    UseBikeLogicOnCars = cfg->GetBool("UseBikeLogicOnCars", false, "Settings");
    UseBikeLogicOnBicycles = cfg->GetBool("UseBikeLogicOnBicycles", true, "Settings");
    BicyclesDontStopForRed = cfg->GetBool("BicyclesDontStopForRed", false, "Settings");
    OnlyInNormal = cfg->GetBool("OnlyInNormal", false, "Settings");
    CheckObjects = cfg->GetBool("CheckObjects", false, "Settings");

    SET_TO(ms_pVehiclePool, aml->GetSym(hGTASA, "_ZN6CPools15ms_pVehiclePoolE"));
    SET_TO(ms_pObjectPool, aml->GetSym(hGTASA, "_ZN6CPools14ms_pObjectPoolE"));
    SET_TO(TheCamera, aml->GetSym(hGTASA, "TheCamera"));
    SET_TO(ms_fTimeStep, aml->GetSym(hGTASA, "_ZN6CTimer12ms_fTimeStepE"));
    SET_TO(pIgnoreEntity, aml->GetSym(hGTASA, "_ZN6CWorld13pIgnoreEntityE"));
    
    SET_TO(FindGroundZFor3DCoord, aml->GetSym(hGTASA, "_ZN6CWorld21FindGroundZFor3DCoordEfffPbPP7CEntity"));
    SET_TO(PlaceAutomobileOnRoadProperly, aml->GetSym(hGTASA, "_ZN11CAutomobile19PlaceOnRoadProperlyEv"));
    SET_TO(FindPlayerPed, aml->GetSym(hGTASA, "_Z13FindPlayerPedi"));
    SET_TO(GetColModel, aml->GetSym(hGTASA, "_ZN7CEntity11GetColModelEv"));
    SET_TO(SwitchVehicleToRealPhysics, aml->GetSym(hGTASA, "_ZN8CCarCtrl26SwitchVehicleToRealPhysicsEP8CVehicle"));
    SET_TO(IsLawEnforcementVehicle, aml->GetSym(hGTASA, "_ZNK8CVehicle23IsLawEnforcementVehicleEv"));
    SET_TO(HasCarStoppedBecauseOfLight, aml->GetSym(hGTASA, "_ZNK11CAutomobile27HasCarStoppedBecauseOfLightEv"));
    SET_TO(ProcessLineOfSight, aml->GetSym(hGTASA, "_ZN6CWorld18ProcessLineOfSightERK7CVectorS2_R9CColPointRP7CEntitybbbbbbbb"));

    HOOK(AutomobileHorn, aml->GetSym(hGTASA, "_ZN11CAutomobile11PlayCarHornEv"));
    HOOK(BikeHorn, aml->GetSym(hGTASA, "_ZN5CBike11PlayCarHornEv"));
    HOOKPLT(VehicleRender, pGTASA + BYVER(0x66E26C, 0x83D318)); // vtable
    HOOKPLT(VehicleRender, pGTASA + BYVER(0x66F3B4, 0x83F048));
    HOOK(VehicleDestroy, aml->GetSym(hGTASA, "_ZN8CVehicleD2Ev"));

    // CCarCtrl::PickNextNodeRandomly
    if(cfg->GetBool("AvoidVehiclesChangingLane", true, "Settings")) aml->Redirect(pGTASA + BYVER(0x2EB0F8 + 0x1, 0x3AED1C), (uintptr_t)VehicleChangesLane);
    VehicleChangesLane_BackTo = pGTASA + BYVER(0x2EB114 + 0x1, 0x3AED38);

    // CCarCtrl::GenerateOneRandomCar
    if(cfg->GetBool("SlowVehiclesSlowLane", true, "Settings")) aml->Redirect(pGTASA + BYVER(0x2E8CCC + 0x1, 0x3AC658), (uintptr_t)SetNewCarLane);
    SetNewCarLane_BackTo = pGTASA + BYVER(0x2E8CDE + 0x1, 0x3AC66C);
}