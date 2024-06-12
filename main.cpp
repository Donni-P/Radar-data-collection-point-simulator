#include <imgui.h>
#include <imgui-SFML.h>
#include <math.h>
#include <vector>
#include <map>
#include <set>
#include <string>
#include <iostream>
#include <SFML/Graphics/Texture.hpp>
#include <SFML/Graphics/Sprite.hpp>
#include <SFML/Graphics/ConvexShape.hpp>
#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/System/Clock.hpp>
#include <SFML/Window/Event.hpp>
constexpr float radiusR = 200.f; 
constexpr float radiusCP = 120.f;
constexpr float centerCP = 450.f;
constexpr int sigma = 1;
constexpr float epsilon = 1.f;
constexpr float minDistanceToAttract = 2.5;
constexpr float gravityCoef = 0.5;
constexpr int T = 12;
constexpr int minPointsInTrajectory = 4;
constexpr int pointInCircle = 500;
    ImVec2 menuPos(0,0);
    ImVec2 configWinPos(0,70);
    ImVec2 simWinPos(400,0);
    static constexpr float fontScale = 1.3f;
    static constexpr float timerWidth = 25.f;
    static constexpr float sliderWidth = 80.f; 
    static constexpr ImVec2 separator(15,1);
sf::RenderStates states;
sf::Time simStep = sf::seconds(1.f);
sf::Time updateRate = sf::seconds((float)T);


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

    static sf::Vector2f calcOffset(sf::Vector2f fromPoint, sf::Vector2f toPoint, float v) {
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

    static bool arePointsClose(sf::Vector2f p1, sf::Vector2f p2, float tolerance = (float)T/2.0) {
        return (abs(p1.x - p2.x) < tolerance) && (abs(p1.y - p2.y) < tolerance);
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

    std::tuple<bool, bool, int> addIfContainsPoint(sf::Vector2f &point){
        bool isContains = Trajectory::calcDistance(point, radar.getPosition()) < radiusR;
        bool isDistance = false;
        int currentTrajectoryInd = -1;
        if (isContains) {
            if (!isAddingTrajectory) {
                isAddingTrajectory = true;
                trajectories.push_back(Trajectory(point));
            }
            sf::Vertex lastPoint = trajectories.back().getPoint(trajectories.back().getCountPoints() - 1);
            float distance = Trajectory::calcDistance(lastPoint.position, point);
            if (distance >= T) {
                isDistance = true;
                sf::Vector2f direction = point - lastPoint.position;
                direction /= distance;
                point = lastPoint.position + direction*((float)T);  
            } else { isDistance = false; }

            if (isDistance) {
                trajectories.back().addPoint(point);
            }
        } else  { 
            endAddingTrajectory(currentTrajectoryInd);
        }
        return {isContains, isDistance, currentTrajectoryInd};
    }

    void endAddingTrajectory(int & ind) {
        if (isAddingTrajectory) { 
            isAddingTrajectory = false;
            if (trajectories.back().getCountPoints() < minPointsInTrajectory) 
                trajectories.pop_back();
            else 
                ind = trajectories.size() - 1;
        }
    }

    void clearTrajectories(void) {
        trajectories.clear();
    }

    std::vector<Trajectory> & getTrajectories(void) { return trajectories; }

private:
    bool isAddingTrajectory = false;
    std::vector<Trajectory> trajectories;
    sf::CircleShape radar;
};


class CollectionPoint {
public:
    enum mode {
        stay,
        add,
        config,
        simulation,
        pause
    };

    static void init(void) {
        //===========preferences IMGUI======================
        auto & style = ImGui::GetStyle();
        style.Alpha = 1.0f;
        style.ItemSpacing.y = 6.f;
        //=================================================
        setRadarsPos(2);
        srand(time(nullptr));
    }

    static void setRadarsPos(int amountRadars) {
        radars.resize(amountRadars);
        for (int i = 1; i <= amountRadars; i++){
            float angleInRadians = ((360/amountRadars)*M_PI/180) * i;
            radars[i-1].init(sf::Vector2f(center + cosf(angleInRadians)*radiusCP, center - sinf(angleInRadians)*radiusCP));
        }
    }

    static void updateTrajectoryTime(int ind) {
        Trajectory & trajectory = definedTrajectories[ind].trajectory;
        std::vector<std::pair<int,int>> & subTrajectoriesRadInd = definedTrajectories[ind].subTrajectoriesRadInd;
        int & speed = trajectory.getSpeed();
        std::pair<int,int> trajectoryStartTime = trajectory.getStartTime();
        for (int i = 0; i < subTrajectoriesRadInd.size(); i++) {
            for (int j = 0; j < trajectory.getCountPoints(); j++) {
                int subRadInd = subTrajectoriesRadInd[i].first;
                int subTrajectoryInd = subTrajectoriesRadInd[i].second;
                if (Trajectory::arePointsClose(trajectory.getPoint(j).position,
                    radars[subRadInd].getTrajectories()[subTrajectoryInd].getPoint(0).position)) {
                    std::pair<int,int> & subStartTime = radars[subRadInd].getTrajectories()[subTrajectoryInd].getStartTime();
                    subStartTime = {(trajectoryStartTime.first + (T*j/speed) / 60),
                                    (trajectoryStartTime.second + (T*j/speed) % 60)};
                }
            }
        }
    }

    static bool isOverLimitTime(int ind) {
        std::pair<int,int> & trStartTime = definedTrajectories[ind].trajectory.getStartTime();
        return ((trStartTime.first > timer.first) 
                            || 
                ((trStartTime.first == timer.first) && (trStartTime.second >= (timer.second - T))));
    }

    static std::pair<bool, bool> radarsAddPoint(sf::Vector2f &point) {
        bool contains = false;
        bool enoughDistance = false;
        std::tuple<bool, bool, int> res;
        for (int i = 0; i < radars.size(); i++) {
            res = radars[i].addIfContainsPoint(point);
            contains = contains || std::get<0>(res);
            enoughDistance = enoughDistance || std::get<1>(res);
            if(std::get<2>(res) != -1) {
                int newSubTrajectoryInd = std::get<2>(res);
                definedTrajectories.back().subTrajectoriesRadInd.push_back({i, newSubTrajectoryInd});
                radars[i].getTrajectories()[newSubTrajectoryInd].getRadarIndice() = i;
            }
        }
        return {contains, enoughDistance};
    }

    static void endAddingProcess(void) {
        for (int i = 0; i < radars.size(); i++) {
            int ind = -1;
            radars[i].endAddingTrajectory(ind);
            if (ind != -1) {
                definedTrajectories.back().subTrajectoriesRadInd.push_back({i, ind});
                radars[i].getTrajectories()[ind].getRadarIndice() = i;
            }
        }
    }

    static void clearRadarsTrajectories(void) {
        for (int i = 0; i < radars.size(); i++) {
            radars[i].clearTrajectories();
        }
    }

    static void identificateExisting(void) {
        std::vector<std::map<std::pair<int,int>, std::pair<int,float>>> identificationVariants{};
        //содержит варианты отождествлений map(ключ:{индекс ЭОИ, индекс РЛС}, значение:{индекс ЭОП, дистанция между т.})
        std::vector<float> sumDistances;
        std::map<int, sf::Vector2f> newExistingPoints;
        std::map<int,int> identifiedPointsCount;
        std::map<int, sf::Vector2f> newDirections;
        std::map<int, int> newSpeed;
        int minDistanceVarInd = 0;
        for (int i = 0; i < incomingTrajectories.size(); i++) {
            std::vector<int> addedPointsInd = {-1};
            for (int j = 0; j < existingTrajectories.size(); j++) {
                sf::Vector2f existLastP = existingTrajectories[j].getPoint(existingTrajectories[j].getCountPoints()-1).position;
                sf::Vector2f incomLastP = incomingTrajectories[i].getPoint(incomingTrajectories[i].getCountPoints()-1).position;
                float distance = Trajectory::calcDistance(existLastP, incomLastP);
                int radInd = incomingTrajectories[i].getRadarIndice();
                if (distance < epsilon) {
                    std::vector<std::map<std::pair<int,int>, std::pair<int,float>>> newVariants{};
                    if (identificationVariants.empty())
                        identificationVariants.push_back(std::map<std::pair<int,int>, std::pair<int,float>>{});
                    for (int varInd = 0; varInd < identificationVariants.size(); varInd++) {
                        std::map<std::pair<int,int>, std::pair<int,float>> newVar(identificationVariants[varInd]);
                        if (addedPointsInd[varInd] != -1) {
                            newVar.erase({addedPointsInd[varInd],radInd});
                            newVar[{j,radInd}] = {i, distance};
                            addedPointsInd.push_back(j);
                            newVariants.push_back(newVar);
                        }else if (identificationVariants[varInd].count({j,radInd}) > 0) {
                            newVar[{j,radInd}] = {i, distance};
                            addedPointsInd.push_back(j);
                            newVariants.push_back(newVar);
                        } else {
                            identificationVariants[varInd][{j,radInd}] = {i, distance};
                            addedPointsInd[varInd] = j;
                        }
                    }
                    identificationVariants.insert(identificationVariants.end(), newVariants.begin(), newVariants.end());
                }
            }
        }

        for (int varInd = 0; varInd < identificationVariants.size(); varInd++) {
            std::cout<<"\n{";
            for (auto [key, value] : identificationVariants[varInd]) {
                std::cout<<"\nO|"<<key.first<<"-"<<value.first<<"-r"<<key.second<<"|П";
            }
            std::cout<<"\n}";
        }

        for (int i = 0; i < identificationVariants.size(); i++) {
            sumDistances.push_back(0.f);
            for (auto [key, value] : identificationVariants[i]) {
                sumDistances[i] = sumDistances[i] + value.second;
            }
            if (sumDistances[i] < sumDistances[minDistanceVarInd])
                minDistanceVarInd = i;
        }

        for (int i = 0; i < incomingTrajectories.size(); i++) {
            int newSize = incomingTrajectories[i].getCountPoints() - T/2;
            incomingTrajectories[i].resize(newSize);
            incomingTrajectories[i].getDirections().resize(newSize);
        }
        if (!identificationVariants.empty()) {
            for (auto [key,value] : identificationVariants[minDistanceVarInd]) {
                Trajectory & incomT = incomingTrajectories[value.first];
                if (newExistingPoints.count(key.first) == 0) {
                    newExistingPoints[key.first] = incomT.getPoint(incomT.getCountPoints()-1).position;
                    newDirections[key.first] = incomT.getDirections()[incomT.getCountPoints()-1];
                    newSpeed[key.first] = incomT.getSpeed();
                } else {
                    newExistingPoints[key.first] += incomT.getPoint(incomT.getCountPoints()-1).position;
                    newDirections[key.first] += incomT.getDirections()[incomT.getCountPoints()-1];
                    newSpeed[key.first] += incomT.getSpeed();
                }
                if (identifiedPointsCount.count(key.first) > 0) 
                    identifiedPointsCount[key.first] += 1;
                else 
                    identifiedPointsCount[key.first] = 1;
            } 

            std::vector<Trajectory> newIncomingTrajectories;
            for (int  i = 0; i < incomingTrajectories.size(); i++) {
                bool notIdentified = true;
                for (auto [key,value] : identificationVariants[minDistanceVarInd]) {
                    if (value.first == i)
                        notIdentified = false;
                }
                if (notIdentified)
                    newIncomingTrajectories.push_back(incomingTrajectories[i]);
            }
            incomingTrajectories = newIncomingTrajectories;
        }

        std::vector<int> trajectoriesToDropInd(existingTrajectories.size(),-1);
        for (int i = 0; i < existingTrajectories.size(); i++) {
            int newSize = existingTrajectories[i].getCountPoints() - T/2;
            existingTrajectories[i].resize(newSize);
            existingTrajectories[i].getDirections().resize(newSize);
            if (newExistingPoints.count(i) > 0) {
                trajectoriesToDropInd[i] = 1;
                existingTrajectories[i].addPoint(
                    (existingTrajectories[i].getPoint(existingTrajectories[i].getCountPoints()-1).position +
                     newExistingPoints[i]) /((float)identifiedPointsCount[i] + 1.f));
                existingTrajectories[i].getDirections().push_back(newDirections[i] /(float)identifiedPointsCount[i]);
                existingTrajectories[i].getSpeed() = (existingTrajectories[i].getSpeed() + newSpeed[i]) / 
                    ((float)identifiedPointsCount[i] + 1.f);
            } 
        }
        std::vector<Trajectory> newExistingTrajectories;
        for (int  i = 0; i < existingTrajectories.size(); i++) {
            if (trajectoriesToDropInd[i] != -1) {
                newExistingTrajectories.push_back(existingTrajectories[i]);
            } 
        }
        if (!existingTrajectories.empty())
            existingTrajectories = newExistingTrajectories;
    }

    static void printDef(void) {
        for (int i = 0; i < definedTrajectories.size(); i++) {
            Trajectory & trajectory = definedTrajectories[i].trajectory;
            std::vector<std::pair<int,int>> & subTrajectoriesRadInd = definedTrajectories[i].subTrajectoriesRadInd;
            std::cout<<"\n N subtrajectories: "<<subTrajectoriesRadInd.size();
            for (int i = 0; i < subTrajectoriesRadInd.size(); i++) {
                for (int j = 0; j < trajectory.getCountPoints(); j++) {
                    int subRadInd = subTrajectoriesRadInd[i].first;
                    int subTrajectoryInd = subTrajectoriesRadInd[i].second;
                    if (trajectory.getPoint(j).position == 
                        radars[subRadInd].getTrajectories()[subTrajectoryInd].getPoint(0).position) {
                            std::cout<<"\n+";
                    }
                }
            }
        }
    }

    static void identificateIncoming(void) {
        std::vector<std::vector<std::map<int,int>>> identificationVariants = 
            {{std::map<int,int>{{incomingTrajectories[0].getRadarIndice(),0}}}};
        std::vector<float> sumDistances;
        for (int curTrajectoryInd = 1; curTrajectoryInd < incomingTrajectories.size(); curTrajectoryInd++) {
            std::vector<std::vector<std::map<int,int>>> newVariants;
            int radInd = incomingTrajectories[curTrajectoryInd].getRadarIndice();
            for (int varInd = 0; varInd < identificationVariants.size(); varInd++) {
                std::pair<int,int> trajectoryAddedInd = {-1,-1};
                for (int groupInd = 0; groupInd < identificationVariants[varInd].size(); groupInd++) {
                    std::map<int,int> related;
                    for (std::pair<int, int> tInGroup : identificationVariants[varInd][groupInd]) {
                        float distance = Trajectory::calcDistance(
                        incomingTrajectories[tInGroup.second].getPoint(incomingTrajectories[tInGroup.second].getCountPoints()-1).position,
                        incomingTrajectories[curTrajectoryInd].getPoint(incomingTrajectories[curTrajectoryInd].getCountPoints()-1).position);
                        if (distance < epsilon) {
                            if (tInGroup.first != radInd) 
                                related[radInd] = tInGroup.second;
                        }
                    }
                    if (related.size() == identificationVariants[varInd][groupInd].size()) {
                        if (trajectoryAddedInd == std::make_pair(-1,-1)) {
                            identificationVariants[varInd][groupInd][radInd] = curTrajectoryInd;
                            trajectoryAddedInd = {groupInd,radInd};
                        } else {
                            std::vector<std::map<int,int>> copyVar = identificationVariants[varInd];
                            copyVar[trajectoryAddedInd.first].erase(trajectoryAddedInd.second);
                            copyVar[groupInd][radInd] = curTrajectoryInd;
                            newVariants.push_back(copyVar);
                        }
                    } else if (!related.empty()) {
                        std::vector<std::map<int,int>> copyVar = identificationVariants[varInd];
                        for (std::pair<int,int> tInGroup : copyVar[groupInd]) {
                            for (std::pair<int,int> relT : related) {
                                if (tInGroup == relT) {
                                    copyVar[groupInd].erase(tInGroup.first);
                                }
                            }
                        }
                        related[radInd] = curTrajectoryInd;
                        if (trajectoryAddedInd != std::make_pair(-1,-1))
                            copyVar[trajectoryAddedInd.first].erase(trajectoryAddedInd.second);
                        copyVar.push_back(related);
                        newVariants.push_back(copyVar);
                    }
                }
                if (trajectoryAddedInd == std::make_pair(-1,-1)) 
                    identificationVariants[varInd].push_back(std::map<int,int>{{radInd, curTrajectoryInd}});
                else
                    trajectoryAddedInd = {-1,-1};
            }
            identificationVariants.insert(identificationVariants.end(), newVariants.begin(), newVariants.end()); 
            newVariants.clear();
        }

        std::vector<float> averageDistances(identificationVariants.size(), 0.f);
        for (int varInd = 0; varInd < identificationVariants.size(); varInd++) {
            std::vector<sf::Vector2f> groupCentres(identificationVariants[varInd].size(), {0.f,0.f});
            for (int groupInd = 0; groupInd < identificationVariants[varInd].size(); groupInd++) {
                for (std::pair<int,int> tInGroup : identificationVariants[varInd][groupInd]) {
                    groupCentres[groupInd] += 
                        ((incomingTrajectories[tInGroup.second].getPoint(
                          incomingTrajectories[tInGroup.second].getCountPoints()-1).position)/
                          (float)identificationVariants[varInd][groupInd].size());
                }
            }
            int numDistances = (groupCentres.size()*(groupCentres.size() - 1))/2;
            for (int i = 0; i < groupCentres.size(); i++) {
                for (int j = i + 1; j < groupCentres.size(); j++) {
                    averageDistances[varInd] += (Trajectory::calcDistance(groupCentres[i], groupCentres[j])/numDistances);
                }
            }
        }

        int maxAvDistInd = 0;
        for (int i = 1; i < averageDistances.size(); i++) {
            if (averageDistances[i] > averageDistances[maxAvDistInd])
                maxAvDistInd = i;
        }

        for (int groupInd = 0; groupInd < identificationVariants[maxAvDistInd].size(); groupInd++) {
            sf::Vector2f newExistingPoint(0.f,0.f);
            sf::Vector2f newDirection(0.f,0.f);
            int newSpeed = 0;
            for (std::pair<int,int> tInGroup : identificationVariants[maxAvDistInd][groupInd]) {
                newExistingPoint += incomingTrajectories[tInGroup.second].getPoint(
                    incomingTrajectories[tInGroup.second].getCountPoints()-1).position /
                    (float)identificationVariants[maxAvDistInd][groupInd].size();
                newDirection += incomingTrajectories[tInGroup.second].getDirections()[
                    incomingTrajectories[tInGroup.second].getCountPoints()-1] /
                    (float)identificationVariants[maxAvDistInd][groupInd].size();
                newSpeed += incomingTrajectories[tInGroup.second].getSpeed() /
                    (float)identificationVariants[maxAvDistInd][groupInd].size();
            }
            existingTrajectories.push_back(Trajectory(newExistingPoint));
            existingTrajectories.back().getDirections()[0] = newDirection;
            existingTrajectories.back().getSpeed() = newSpeed;
        }
        incomingTrajectories.clear();
    }

    static void extrapolationStep(bool extrExisting) {
        std::vector<Trajectory> & trajectories = 
            (extrExisting) ? existingTrajectories : incomingTrajectories;
        std::vector<Trajectory> & labels = 
            (!extrExisting) ? existingTrajectories : incomingTrajectories;
        std::vector<sf::Vector2f> offsets(trajectories.size(), sf::Vector2f(0.f,0.f));
        std::vector<sf::Vector2f> directionsPointLast;
        for (int i = 0; i < trajectories.size(); i++) {
            directionsPointLast.push_back(trajectories[i].getDirections()[trajectories[i].getCountPoints()-1]);
            int speed = trajectories[i].getSpeed();
            for (int j = 0; j < trajectories.size(); j++) {
                bool fromSameRadars = !extrExisting && (trajectories[i].getRadarIndice() == trajectories[j].getRadarIndice());
                if ((j != i) && (extrExisting || fromSameRadars)){
                    float distance = Trajectory::calcDistance(trajectories[j].getPoint(trajectories[j].getCountPoints()-1).position,
                        trajectories[i].getPoint(trajectories[i].getCountPoints()-1).position);
                    if (distance <= minDistanceToAttract) {
                        if (extrExisting)
                            offsets[i] += Trajectory::calcOffset(
                                trajectories[j].getPoint(trajectories[j].getCountPoints()-1).position,
                                trajectories[i].getPoint(trajectories[i].getCountPoints()-1).position,
                                (float)speed
                            );
                        else
                            offsets[i] += Trajectory::calcOffset(
                                    trajectories[i].getPoint(trajectories[i].getCountPoints()-1).position,
                                    trajectories[j].getPoint(trajectories[j].getCountPoints()-1).position,
                                    (float)speed
                                );
                    }
                }
            }
            for (int j = 0; j < labels.size(); j++) {
                float distance = Trajectory::calcDistance(trajectories[i].getPoint(trajectories[i].getCountPoints()-1).position,
                    labels[j].getPoint(labels[j].getCountPoints()-1).position);
                if (distance <= minDistanceToAttract) {
                    offsets[i] += Trajectory::calcOffset(
                        trajectories[i].getPoint(trajectories[i].getCountPoints()-1).position,
                        labels[j].getPoint(labels[j].getCountPoints()-1).position,
                        (float)speed
                    );
                }
            }
            offsets[i] += trajectories[i].calcOffset(trajectories[i].getCountPoints()-1,speed);
        }
        for (int i = 0; i < trajectories.size(); i++) {
           // sf::Vector2f smoothedOffset = Trajectory::calcOffset(sf::Vector2f(0,0), offsets[i], trajectories[i].getSpeed());
            sf::Vector2f newPoint = trajectories[i].getPoint(trajectories[i].getCountPoints()-1).position + offsets[i];
            trajectories[i].addPoint(newPoint);
            trajectories[i].getDirections().push_back(directionsPointLast[i]);
        }
    }

    static void extrapolateExisting(void) {
        for (int i = 0; i < existingTrajectories.size(); i++) {
            existingTrajectories[i].addPoint(
                existingTrajectories[i].getPoint(existingTrajectories[i].getCountPoints()-1).position +
                existingTrajectories[i].calcOffset(existingTrajectories[i].getCountPoints()-1, T/existingTrajectories[i].getSpeed()));
            existingTrajectories[i].getDirections().push_back(
                existingTrajectories[i].getDirections()[existingTrajectories[i].getCountPoints() - 2]);
        }
    }

    static void getIncoming(int elapsedSecs) {
        for (int i = 0; i < radars.size(); i++) {
            std::vector<Trajectory> & trajectories = radars[i].getTrajectories();
            for (int j = 0; j < trajectories.size(); j++) {
                std::pair<int,int> & t0 = trajectories[j].getStartTime();
                int speed = trajectories[j].getSpeed();
                int startTimeSec = t0.first*60 + t0.second;
                int correctElapsedSecs = elapsedSecs - startTimeSec;
                if (correctElapsedSecs >= 0) {
                    int S = correctElapsedSecs*speed;
                    int ind = S/T;
                    int residueS = S%T;
                    if ((elapsedSecs >= startTimeSec) && (ind < trajectories[j].getCountPoints())){
                        incomingTrajectories.push_back(Trajectory(
                            trajectories[j].getPoint(ind).position + trajectories[j].calcOffset(ind, residueS)));
                        incomingTrajectories.back().getDirections()[0] = trajectories[j].getDirections()[ind];
                        incomingTrajectories.back().getSpeed() = trajectories[j].getSpeed();
                        incomingTrajectories.back().getRadarIndice() = i;
                    }
                }
            }
        }
    } 

    static void identificateTrajectories(void) {
        identificateExisting();
        if (!incomingTrajectories.empty()) {
            identificateIncoming();
        }
    }

    static void tick(sf::RenderWindow & win, sf::Clock & clk) {
        sf::Time elapsed = clk.restart();
        elapsed *= (float)simulationSpeed;
        ImGui::SFML::Update(win, elapsed);
        static int elapsedSecs = 0;
        static sf::Time stepTime = sf::Time::Zero;
        static sf::Time periodTime = sf::Time::Zero;
        static bool extrExisting = true;
        if (modeCP == mode::simulation) {
            stepTime += elapsed;
            periodTime += elapsed;
            if (stepTime >= simStep) {
                elapsedSecs++;
                stepTime -= simStep;
                if (timer.second == 0) {
                    if (timer.first == 0) {
                        modeCP = mode::pause;
                    } else {
                        timer.first--;
                        timer.second = 59;
                    }
                } else {
                    timer.second--;
                }
                extrapolationStep(extrExisting);
                extrExisting = !extrExisting;
                for (Trajectory & t : existingTrajectories)
                    t.setColor(sf::Color::Green);
            }
            if (periodTime >= updateRate) {
                periodTime -= updateRate;
                identificateTrajectories();
                extrapolateExisting();
                getIncoming(elapsedSecs);
            }
        } else if (modeCP != mode::pause) {
            elapsedSecs = 0;
            extrExisting = true;
            stepTime = sf::Time::Zero;
            periodTime = sf::Time::Zero;
            existingTrajectories.clear();
        }
    }

    static void drawRadars(sf::RenderWindow & win) {
        for (int i = 0; i < radars.size(); i++)
            radars[i].drawRadar(win);
    }

    static void drawTrajectories(sf::RenderWindow & win) {
        if ((modeCP == mode::simulation) || (modeCP == mode::pause)) {
            for (int i = 0; i < existingTrajectories.size(); i++) 
                existingTrajectories[i].drawTrajectory(win);
            for (int i = 0; i < incomingTrajectories.size(); i++) 
                incomingTrajectories[i].drawTrajectory(win);
        } else {
            for (int i = 0; i < definedTrajectories.size(); i++)
                definedTrajectories[i].trajectory.drawTrajectory(win);
        }
    }

    static void drawGUI(void) {
        if ((modeCP == mode::simulation) || (modeCP == mode::pause)) {
            ImGui::Begin("Simulation", nullptr, ImGuiWindowFlags_NoCollapse | 
                                                ImGuiWindowFlags_NoResize | 
                                                ImGuiWindowFlags_AlwaysAutoResize |
                                                ImGuiWindowFlags_NoMove);
            ImGui::SetWindowPos(menuPos);
            ImGui::SetWindowFontScale(fontScale);
            ImGui::SetNextItemWidth(timerWidth);
            ImGui::InputScalar("m", ImGuiDataType_U8, &timer.first, nullptr, 
                                nullptr, nullptr, ImGuiInputTextFlags_ReadOnly);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(timerWidth);
            ImGui::InputScalar("s", ImGuiDataType_U8, &timer.second, nullptr, 
                                nullptr, nullptr, ImGuiInputTextFlags_ReadOnly);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(sliderWidth);
            ImGui::SliderInt("Speed", &simulationSpeed, 1, 4);
            ImGui::SameLine();
            (modeCP == mode::pause) ? ImGui::PushStyleColor(ImGuiCol_Button, sf::Color::Red) 
                                    : ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_Button));
            if (ImGui::Button("Pause")) {
                if (modeCP == mode::simulation)
                    modeCP = mode::pause;
                else if (timer != std::make_pair(0,0))
                    modeCP = mode::simulation;
            }
            ImGui::PopStyleColor();
            ImGui::SameLine();
            if (ImGui::Button("Stop")) {
                modeCP = mode::stay;
                existingTrajectories.clear();
                incomingTrajectories.clear();
            }

            ImGui::PopStyleColor();
            ImGui::End();
        } else {
            static int curInd = 0;
            static std::pair<int,int> prevTimer = {59,59};
            static int amountRads = 2;
            ImGui::Begin("Menu", nullptr, ImGuiWindowFlags_NoCollapse | 
                                        ImGuiWindowFlags_NoResize | 
                                        ImGuiWindowFlags_AlwaysAutoResize |
                                        ImGuiWindowFlags_NoMove);
            ImGui::SetWindowPos(menuPos);
            ImGui::SetWindowFontScale(fontScale);
            (modeCP == mode::add) ? ImGui::PushStyleColor(ImGuiCol_Button, sf::Color::Red) 
                                  : ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_Button));
            if (ImGui::Button("Add Mode")) {
                if (modeCP == mode::stay) {
                    modeCP = mode::add;
                } else {
                    printDef();
                    for (int i = 0; i < definedTrajectories.size(); i++) {
                        updateTrajectoryTime(i);
                        for (std::pair<int,int> subT : definedTrajectories[i].subTrajectoriesRadInd) {
                            radars[subT.first].getTrajectories()[subT.second].refreshDirections();
                        }
                    }
                    modeCP = mode::stay;
                }
            }
            ImGui::PopStyleColor();
            ImGui::SameLine();
            ImGui::Dummy(separator);
            ImGui::SameLine();
            (modeCP == mode::config) ? ImGui::PushStyleColor(ImGuiCol_Button, sf::Color::Red) 
                                     : ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_Button));
            if (ImGui::Button("Config Mode")) {
                if (!definedTrajectories.empty() && (modeCP != mode::add)) {
                    definedTrajectories[curInd].trajectory.setColor(sf::Color::Blue);
                    if (modeCP == mode::stay)
                        modeCP = mode::config;
                    else
                        modeCP = mode::stay;
                }
            }
            ImGui::PopStyleColor();
            ImGui::SameLine();
            ImGui::Dummy(separator);
            ImGui::SameLine();
            if (ImGui::Button("Clear")) {
                if ((modeCP != mode::config) && (modeCP != mode::simulation)) {
                    definedTrajectories.clear();
                    clearRadarsTrajectories();
                }
            }
            ImGui::SameLine();
            ImGui::Dummy(separator);
            ImGui::SameLine();
            ImGui::Text("Timer");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(timerWidth);
            if (ImGui::InputScalar("m", ImGuiDataType_U8, &timer.first)) {
                if ((timer.first < 1) || (timer.first > 59)) {
                    timer.first = prevTimer.first;
                }
                for (int i = 0; i < definedTrajectories.size(); i++) {
                    if (isOverLimitTime(i)) {
                        definedTrajectories[i].trajectory.getStartTime() = {0,0};
                        updateTrajectoryTime(i);
                    }
                }

            }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(timerWidth);
            if (ImGui::InputScalar("s", ImGuiDataType_U8, &timer.second)) {
                if (timer.second > 59) timer.second = prevTimer.second;
                for (int i = 0; i < definedTrajectories.size(); i++) {
                    if (isOverLimitTime(i)) {
                        definedTrajectories[i].trajectory.getStartTime() = {0,0};
                        updateTrajectoryTime(i);
                    }
                }
            }
            prevTimer = timer;
            ImGui::SameLine();
            ImGui::Dummy(separator);
            ImGui::SameLine();
            ImGui::Text("Amount of radars");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(sliderWidth);
            if (ImGui::SliderInt("##Amount of RADARs", &amountRads, 2, 5))
                setRadarsPos(amountRads);
            ImGui::SameLine();
            ImGui::Dummy(separator);
            ImGui::SameLine();
            if (ImGui::Button("Start")) {
                if ((modeCP != mode::config) && (modeCP != mode::add)) {
                    modeCP = mode::simulation;
                    getIncoming(0);
                }
            }
            ImGui::End();
            if (modeCP == mode::config) {
                ImGui::Begin("Trajectory", nullptr, ImGuiWindowFlags_NoCollapse | 
                                                    ImGuiWindowFlags_NoResize | 
                                                    ImGuiWindowFlags_AlwaysAutoResize |
                                                    ImGuiWindowFlags_NoMove);
                definedTrajectories[curInd].trajectory.setColor(sf::Color::Red);
                ImGui::SetWindowPos(configWinPos);
                ImGui::Text("Speed");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(sliderWidth);
                if (ImGui::SliderInt("pix/sec", &definedTrajectories[curInd].trajectory.getSpeed(),1,4)) {
                    for (int i = 0; i < definedTrajectories[curInd].subTrajectoriesRadInd.size(); i++) {
                        int radInd = definedTrajectories[curInd].subTrajectoriesRadInd[i].first;
                        int trajectoryInd = definedTrajectories[curInd].subTrajectoriesRadInd[i].second;
                        radars[i].getTrajectories()[trajectoryInd].getSpeed() 
                                = 
                        definedTrajectories[curInd].trajectory.getSpeed();
                    }
                }
                ImGui::Text("t0");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(timerWidth);
                std::pair<int,int> & curStartTime = definedTrajectories[curInd].trajectory.getStartTime();
                if (ImGui::InputScalar("m", ImGuiDataType_U8, &curStartTime.first)) {
                    if (isOverLimitTime(curInd)) curStartTime = {0,0};
                    updateTrajectoryTime(curInd);
                }
                ImGui::SameLine();
                ImGui::SetNextItemWidth(timerWidth);
                if (ImGui::InputScalar("s", ImGuiDataType_U8, &curStartTime.second)) {
                    if (isOverLimitTime(curInd)) curStartTime = {0,0};
                    updateTrajectoryTime(curInd);
                }
                if (ImGui::Button("<")) {
                    definedTrajectories[curInd].trajectory.setColor(sf::Color::Blue);
                    if (curInd == 0)
                        curInd = definedTrajectories.size() - 1;
                    else 
                        curInd--;
                }
                ImGui::SameLine();
                if (ImGui::Button(">")) {
                    definedTrajectories[curInd].trajectory.setColor(sf::Color::Blue);
                    if (curInd == (definedTrajectories.size() - 1))
                        curInd = 0;
                    else
                        curInd++;
                }
                ImGui::End(); 
            }
        }
    }

    static void draw(sf::RenderWindow & win) {
        drawGUI();
        drawRadars(win);
        drawTrajectories(win);
    }

    static void eventsProcessing(sf::Event ev, sf::RenderWindow & win) {
        static bool mousePressed = false;
        static bool trajectoryCreated = false;
        if (modeCP == mode::add) {
            if (ev.type == sf::Event::MouseButtonPressed) {
                sf::Vector2f curMousePos(sf::Mouse::getPosition(win));
                std::pair<bool, bool> res = CollectionPoint::radarsAddPoint(curMousePos);
                if (res.first) {
                    definedTrajectories.push_back
                    (defTrajectory({Trajectory(curMousePos), std::vector<std::pair<int,int>>{}}));
                    trajectoryCreated = true;
                }
                mousePressed = true;
            } 
            if (mousePressed && (ev.type == sf::Event::MouseMoved)) {
                sf::Vector2f curMousePos(sf::Mouse::getPosition(win));
                std::pair<bool, bool> res = CollectionPoint::radarsAddPoint(curMousePos);
                if (res.first) {
                    if (!trajectoryCreated) {
                        definedTrajectories.push_back
                        (defTrajectory({Trajectory(curMousePos), std::vector<std::pair<int,int>>{}}));
                        trajectoryCreated = true;
                    } else if (res.second) {
                        definedTrajectories.back().trajectory.addPoint(curMousePos);
                    }
                } else { trajectoryCreated = false; }
            }
            if (ev.type == sf::Event::MouseButtonReleased) {
                CollectionPoint::endAddingProcess();
                if (!definedTrajectories.empty())
                    if (definedTrajectories.back().trajectory.getCountPoints() < minPointsInTrajectory)
                        definedTrajectories.pop_back();
                mousePressed = false;
                trajectoryCreated = false;
            }
        } else { mousePressed = false; }
    }

private:
    struct defTrajectory {
        Trajectory trajectory;
        std::vector<std::pair<int,int>> subTrajectoriesRadInd;
    };
    static inline std::pair<int,int> timer = {59,59};
    static inline mode modeCP = mode::stay;
    static inline std::vector<defTrajectory> definedTrajectories; 
    static inline std::vector<Trajectory> existingTrajectories;
    static inline std::vector<Trajectory> incomingTrajectories;
    static inline std::vector<Radar> radars;
    static inline float center = centerCP;
    static inline int simulationSpeed = 1;
};

int main() {
    states.blendMode = sf::BlendMultiply;
    sf::RenderWindow window(sf::VideoMode(900, 900), "RDCP Simulator");
    window.setFramerateLimit(60);
    ImGui::SFML::Init(window);


    CollectionPoint::init();
    sf::Clock deltaClock;
    //==================================================
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            ImGui::SFML::ProcessEvent(window, event);
            if (event.type == sf::Event::Closed) {
                window.close();
            }
            CollectionPoint::eventsProcessing(event, window);
        }

        CollectionPoint::tick(window, deltaClock);

        window.clear(sf::Color::White);
        CollectionPoint::draw(window);
        ImGui::SFML::Render(window);
        window.display();
    }

    ImGui::SFML::Shutdown();
}