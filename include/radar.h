#pragma once
#include<trajectory.h>

class Radar{
public:
    Radar() : radar(radiusR, pointInCircle) {}
    void init(sf::Vector2f pos) {
        radar.setFillColor(sf::Color::White);
        radar.setOutlineColor(sf::Color::Black);
        radar.setOutlineThickness(2.0f);
        radar.setOrigin(radiusR, radiusR);
        radar.setPosition(pos);
    }

    void drawRadar(sf::RenderWindow &win) {   
        win.draw(radar, states);
    }

    bool isContains(sf::Vector2f point) {
        return Trajectory::calcDistance(point, radar.getPosition()) < radiusR;
    }

    void clearTrajectories(void) {
        trajectories.clear();
    }

    std::vector<Trajectory> & getTrajectories(void) { return trajectories; }

private:
    std::vector<Trajectory> trajectories;
    sf::CircleShape radar;
};
