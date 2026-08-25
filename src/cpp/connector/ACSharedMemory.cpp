#include "ACSharedMemory.h"
#include <iostream>

ACSharedMemory::ACSharedMemory() 
    : m_hMapPhysics(nullptr), m_hMapGraphics(nullptr), m_hMapStatic(nullptr),
      m_physics(nullptr), m_graphics(nullptr), m_static(nullptr) {}

ACSharedMemory::~ACSharedMemory() {
    if (m_physics) UnmapViewOfFile(m_physics);
    if (m_graphics) UnmapViewOfFile(m_graphics);
    if (m_static) UnmapViewOfFile(m_static);
    
    if (m_hMapPhysics) CloseHandle(m_hMapPhysics);
    if (m_hMapGraphics) CloseHandle(m_hMapGraphics);
    if (m_hMapStatic) CloseHandle(m_hMapStatic);
}

bool ACSharedMemory::init() {
    m_hMapPhysics = OpenFileMappingW(FILE_MAP_READ, FALSE, L"Local\\acpmf_physics");
    if (!m_hMapPhysics) {
        std::cerr << "Failed to open physics shared memory. Is Assetto Corsa running?\n";
        return false;
    }
    m_physics = (SPageFilePhysics*)MapViewOfFile(m_hMapPhysics, FILE_MAP_READ, 0, 0, sizeof(SPageFilePhysics));
    
    m_hMapGraphics = OpenFileMappingW(FILE_MAP_READ, FALSE, L"Local\\acpmf_graphics");
    if (!m_hMapGraphics) {
        std::cerr << "Failed to open graphics shared memory.\n";
        return false;
    }
    m_graphics = (SPageFileGraphics*)MapViewOfFile(m_hMapGraphics, FILE_MAP_READ, 0, 0, sizeof(SPageFileGraphics));
    
    m_hMapStatic = OpenFileMappingW(FILE_MAP_READ, FALSE, L"Local\\acpmf_static");
    if (!m_hMapStatic) {
        std::cerr << "Failed to open static shared memory.\n";
        return false;
    }
    m_static = (SPageFileStatic*)MapViewOfFile(m_hMapStatic, FILE_MAP_READ, 0, 0, sizeof(SPageFileStatic));
    
    return true;
}

SPageFilePhysics* ACSharedMemory::getPhysics() {
    return m_physics;
}

SPageFileGraphics* ACSharedMemory::getGraphics() {
    return m_graphics;
}

SPageFileStatic* ACSharedMemory::getStatic() {
    return m_static;
}
