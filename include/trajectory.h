#pragma once
#include<global.h>

class Trajectory {
public:
    Trajectory(sf::Vector2f firstPoint) : startPoint(3.f), endPoint(3.f) {
        startPoint.setOrigin(3.f, 3.f);
        endPoint.setOrigin(3.f, 3.f);
        startPoint.setFillColor(color);
        endPoint.setFillColor(color);
        startPoint.setPosition(firstPoint);
        endPoint.setPosition(firstPoint);
        trajectory.setPrimitiveType(sf::LineStrip);
        addPoint(firstPoint);
    }

    void resize(int size) { trajectory.resize(size); }

    void addPoint(sf::Vector2f point) {
        sf::Vertex newPoint(point);
        newPoint.color = color;
        trajectory.append(newPoint);
        endPoint.setPosition(point);
    }

    void drawTrajectory(sf::RenderWindow & win) {
        win.draw(trajectory,states);
        win.draw(startPoint);
        win.draw(endPoint);
    }

    void setColor(sf::Color col) {
        for (int i = 0; i < trajectory.getVertexCount(); i++) {
            trajectory[i].color = col;
        }
        startPoint.setFillColor(col);
        endPoint.setFillColor(col);
    }

    int getCountPoints(void) { return trajectory.getVertexCount(); }

    static float calcDistance(sf::Vector2f v1, sf::Vector2f v2) {
        return sqrt(pow(v1.x - v2.x, 2) + pow(v1.y - v2.y, 2));
    }

    static sf::Vector2f calcDirection(sf::Vector2f fromPoint, sf::Vector2f toPoint) {
        return (toPoint - fromPoint);
    }

    static sf::Vector2f calcOffset(sf::Vector2f fromPoint, sf::Vector2f toPoint, float v, float gravityCoef = 0.6) {
        sf::Vector2f direction = Trajectory::calcDirection(fromPoint, toPoint);
        float distance = Trajectory::calcDistance(fromPoint, toPoint);
        if (distance != 0)
            return (direction/distance)*(gravityCoef*v);
        else
            return sf::Vector2f(0,0);
    }

    sf::Vector2f calcOffset(int ind, int S) {
        float distance = calcDistance(directions[ind], sf::Vector2f(0,0));
        if (distance != 0)
            return (directions[ind]/distance)*(float)S;
        else 
            return sf::Vector2f(0,0);
    }

    void refreshDirections(void) {
        directions.resize(getCountPoints());
        for (int i = 0; (i < directions.size()) && (directions.size() > 1); i++) {
            if (i != (getCountPoints() - 1)) 
                directions[i] = calcDirection(trajectory[i].position,trajectory[i+1].position);
            else
                directions[i] = directions[i-1];
        }
    }

    std::vector<sf::Vector2f> & getDirections(void) { return directions; }

    sf::Vertex & getPoint(int ind) { return trajectory[ind]; }

    int & getSpeed(void) { return speed; }

    int & getRadarIndice(void) { return radIndice; }

    std::pair<int,int> & getStartTime(void) { return t0; }
private:
    sf::CircleShape startPoint; 
    sf::CircleShape endPoint;
    sf::VertexArray trajectory;
    std::vector<sf::Vector2f> directions = {sf::Vector2f(0,0)};
    sf::Color color = sf::Color::Blue;
    std::pair<int,int> t0 = std::make_pair(0,0);
    int speed = 1;
    int radIndice;
};
