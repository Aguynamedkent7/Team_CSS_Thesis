#pragma once
#include <vector>
#include <string>

struct TrackWaypoint {
    double x, y;
    double inner_bound, outer_bound; // Distance to track limits
};

class CSVLoader {
public:
    static std::vector<TrackWaypoint> loadTrack(const std::string& filepath);
};
