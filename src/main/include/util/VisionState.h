#pragma once

class VisionState
{
public:
    double m_xOffset;
    double m_distance;
    double m_timestamp;

    VisionState(double xOffset, double distance, double timestamp)
    {
        m_xOffset = xOffset;
        m_distance = distance;
        m_timestamp = timestamp;
    }
};