#pragma once
#include <windows.h>
#include "ac_structs.h"

class ACSharedMemory {
public:
    ACSharedMemory();
    ~ACSharedMemory();
    
    bool init();
    
    SPageFilePhysics* getPhysics();
    SPageFileGraphics* getGraphics();
    SPageFileStatic* getStatic();

private:
    HANDLE m_hMapPhysics;
    HANDLE m_hMapGraphics;
    HANDLE m_hMapStatic;
    
    SPageFilePhysics* m_physics;
    SPageFileGraphics* m_graphics;
    SPageFileStatic* m_static;
};
