#include "CSVLoader.h"
#include <fstream>
#include <sstream>
#include <iostream>

std::vector<TrackWaypoint> CSVLoader::loadTrack(const std::string& filepath) {
    std::vector<TrackWaypoint> track;
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Failed to open track file: " << filepath << "\n";
        return track;
    }

    std::string line;
    // skip header if exists, or assume no header.
    // Let's assume standard format: x, y, inner_bound, outer_bound
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        
        std::stringstream ss(line);
        std::string token;
        TrackWaypoint wp;
        
        try {
            std::getline(ss, token, ','); wp.x = std::stod(token);
            std::getline(ss, token, ','); wp.y = std::stod(token);
            std::getline(ss, token, ','); wp.inner_bound = std::stod(token);
            std::getline(ss, token, ','); wp.outer_bound = std::stod(token);
            track.push_back(wp);
        } catch (...) {
            // skip bad lines
        }
    }
    return track;
}
