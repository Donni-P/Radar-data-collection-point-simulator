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
constexpr float gravityCoef = 0.6;
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
    Trajectory(std::tuple<sf::Vertex,sf::Vector2f,int> initParams) : Trajectory(std::get<0>(initParams).position) {
        directions.push_back(std::get<1>(initParams));
        speed = std::get<2>(initParams);
    }

    void resize(int size) { trajectory.resize(size); }

    void addPoint(sf::Vector2f point) {
        sf::Vertex newPoint(point);
        newPoint.color = color;
        trajectory.append(newPoint);
        refreshDirections();
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
        return (direction*(gravityCoef*v))/distance;
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

    sf::Vector2f getDirection(int ind) { return directions[ind]; }

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
    std::pair<int,int> t0 = {0,0};
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

    std::tuple<bool, bool, Trajectory*> addIfContainsPoint(sf::Vector2f &point){
        bool isContains = Trajectory::calcDistance(point, radar.getPosition()) < radiusR;
        bool isDistance = false;
        Trajectory * currentTrajectory = nullptr;
        if (isContains) {
            if (!isAddingTrajectory) {
                isAddingTrajectory = true;
                trajectories.push_back(Trajectory(point));
            }
            currentTrajectory = &trajectories.back();
            sf::Vertex lastPoint = currentTrajectory->getPoint(currentTrajectory->getCountPoints() - 1);
            float distance = Trajectory::calcDistance(lastPoint.position, point);
            if (distance >= T) {
                isDistance = true;
                sf::Vector2f direction = point - lastPoint.position;
                direction /= distance;
                point = lastPoint.position + direction*((float)T);  
            } else { isDistance = false; }

            if (isDistance) {
                currentTrajectory->addPoint(point);
            }
            currentTrajectory = nullptr;
        } else if (isAddingTrajectory) { 
            isAddingTrajectory = false; 
            if (trajectories.back().getCountPoints() < minPointsInTrajectory) 
                trajectories.pop_back();
            else 
                currentTrajectory = &trajectories.back();
        }
        return {isContains, isDistance, currentTrajectory};
    }

    void endAddingTrajectory(void) {
        if (isAddingTrajectory) { 
            isAddingTrajectory = false;
            if (trajectories.back().getCountPoints() < minPointsInTrajectory) 
                trajectories.pop_back();
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
        std::vector<Trajectory*> & subtrajectories = definedTrajectories[ind].subTrajectories;
        int & speed = trajectory.getSpeed();
        std::pair<int,int> trajectoryStartTime = trajectory.getStartTime();
        for (int i = 0; i < subtrajectories.size(); i++) {
            for (int j = 0; j < trajectory.getCountPoints(); j++) {
                if (trajectory.getPoint(j).position == subtrajectories[i]->getPoint(0).position) {
                    std::pair<int,int> & subStartTime = subtrajectories[i]->getStartTime();
                    /*subStartTime.first = trajectoryStartTime.first + (T*j/speed) / 60;
                    subStartTime.second = trajectoryStartTime.second + (T*j/speed) % 60;*/
                    /*subStartTime = {(trajectoryStartTime.first + (T*j/speed) / 60),
                                    (trajectoryStartTime.second + (T*j/speed) % 60)};*/
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
        std::tuple<bool, bool, Trajectory*> res;
        for (int i = 0; i < radars.size(); i++) {
            res = radars[i].addIfContainsPoint(point);
            contains = contains || std::get<0>(res);
            enoughDistance = enoughDistance || std::get<1>(res);
            if(std::get<2>(res) != nullptr) {
                Trajectory * newSubTrajectory = std::get<2>(res);
                definedTrajectories.back().subTrajectories.push_back(newSubTrajectory);
                newSubTrajectory->getRadarIndice() = i;
            }
        }
        return {contains, enoughDistance};
    }

    static void endAddingProcess(void) {
        for (int i = 0; i < radars.size(); i++) {
            radars[i].endAddingTrajectory();
        }
    }

    static void clearRadarsTrajectories(void) {
        for (int i = 0; i < radars.size(); i++) {
            radars[i].clearTrajectories();
        }
    }

    static void extrapolateTrajectories(bool extrExisting) {
        std::vector<Trajectory> & trajectories = 
        (extrExisting) ? existingTrajectories : incomingTrajectories;
        std::vector<Trajectory> & labels = 
        (!extrExisting) ? existingTrajectories : incomingTrajectories;
        std::vector<sf::Vector2f> offsets(trajectories.size(), sf::Vector2f(0.f,0.f));
        for (int i = 0; i < trajectories.size(); i++) {
            int speed = trajectories[i].getSpeed();
            for (int j = 0; j < trajectories.size(); i++) {
                if (j != i) {
                    offsets[i] += Trajectory::calcOffset(
                        trajectories[j].getPoint(trajectories[j].getCountPoints()-1).position,
                        trajectories[i].getPoint(trajectories[i].getCountPoints()-1).position,
                        (float)speed
                    );
                }
            }
            for (int j = 0; j < labels.size(); j++) {
                offsets[i] += Trajectory::calcOffset(
                    trajectories[i].getPoint(trajectories[i].getCountPoints()-1).position,
                    labels[j].getPoint(labels[j].getCountPoints()-1).position,
                    (float)speed
                );
            }
            offsets[i] += (trajectories[i].getDirection(trajectories[i].getCountPoints()-1)
                          * ((float)speed)) / ((float)T);
        }
        for (int i = 0; i < trajectories.size(); i++) {
            sf::Vector2f newPoint = trajectories[i].getPoint(trajectories[i].getCountPoints()-1).position + offsets[i];
            trajectories[i].addPoint(newPoint);
        }
    }

    static void identificateTrajectories(void) {
        std::vector<std::map<std::pair<int,int>, std::pair<int,float>>> identificationVariants{{}};
        std::vector<float> sumDistances;
        std::map<int, sf::Vector2f> newExistingPoints;
        std::map<int,int> identifiedPointsCount;
        int minDistanceVarInd = 0;
        for (int i = 0; i < incomingTrajectories.size(); i++) {
            for (int j = 0; j < existingTrajectories.size(); j++) {
                sf::Vector2f existLastP = existingTrajectories[j].getPoint(existingTrajectories[j].getCountPoints()-1).position;
                sf::Vector2f incomLastP = incomingTrajectories[i].getPoint(incomingTrajectories[i].getCountPoints()-1).position;
                newExistingPoints[j] = existLastP;
                float distance = Trajectory::calcDistance(existLastP, incomLastP);
                int radInd = incomingTrajectories[i].getRadarIndice();
                if (distance < epsilon) {
                    for (auto var : identificationVariants) {
                        if (var.count({j,radInd}) > 0) {
                            std::map<std::pair<int,int>, std::pair<int,float>> newVar(var);
                            newVar[{j,radInd}] = {i, distance};
                            identificationVariants.push_back(newVar);
                        } else {
                            var[{j,radInd}] = {i, distance};
                        }
                    }
                }
            }
        }
        for (int i = 0; i < identificationVariants.size(); i++) {
            sumDistances.push_back(0.f);
            for (auto [key, value] : identificationVariants[i]) {
                sumDistances[i] = sumDistances[i] + value.second;
            }
            if (sumDistances[i] < sumDistances[minDistanceVarInd])
                minDistanceVarInd = i;
        }

        for (auto [key,value] : identificationVariants[minDistanceVarInd]) {
            Trajectory & incomT = incomingTrajectories[value.first];
            newExistingPoints[key.first] += incomT.getPoint(incomT.getCountPoints()-1).position;
            if (identifiedPointsCount.count(key.first) > 0) 
                identifiedPointsCount[key.first] += 1;
            else 
                identifiedPointsCount[key.first] = 1;
            incomingTrajectories.erase(incomingTrajectories.begin() + value.first);
        } 
        for (int i = 0; i < existingTrajectories.size(); i++) {
            existingTrajectories[i].resize(existingTrajectories[i].getCountPoints() - T/2);
            if (newExistingPoints.count(i) > 0) {
                existingTrajectories[i].addPoint(newExistingPoints[i]/((float)identifiedPointsCount[i]));
            } else {
                existingTrajectories[i].addPoint(
                    existingTrajectories[i].getPoint(existingTrajectories[i].getCountPoints()-1).position +
                    (existingTrajectories[i].getDirection(existingTrajectories[i].getCountPoints()-1)
                    * ((float)existingTrajectories[i].getSpeed())) / ((float)T));
            }
        }
        for (int i = 0; i < incomingTrajectories.size(); i++) {
            incomingTrajectories[i].resize(incomingTrajectories[i].getCountPoints() - T/2);
            existingTrajectories.push_back(incomingTrajectories[i]);
        }
    }

    static void tick(sf::RenderWindow & win, sf::Clock & clk) {
        sf::Time elapsed = clk.restart();
        ImGui::SFML::Update(win, elapsed);
        static int elapsedSecs = 0;
        static sf::Time stepTime = sf::Time::Zero;
        static sf::Time periodTime = sf::Time::Zero;
        if (modeCP == mode::simulation) {
            stepTime += elapsed;
            periodTime += elapsed;
            if (stepTime >= simStep) {
                static bool extrExisting = true;
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
                extrapolateTrajectories(extrExisting);
                extrExisting = !extrExisting;
            }
            if (periodTime >= updateRate) {
                periodTime -= updateRate;
                identificateTrajectories();
                initIncomingTrajectories(elapsedSecs);
            }
        } else if (modeCP != mode::pause) {
            elapsedSecs = 0;
            stepTime = sf::Time::Zero;
            periodTime = sf::Time::Zero;
        }
    }

    static void initIncomingTrajectories(int elapsedSecs) {
        for (int i = 0; i < radars.size(); i++) {
            std::vector<Trajectory> & trajectories = radars[i].getTrajectories();
            for (int j = 0; j < trajectories.size(); j++) {
                std::pair<int,int> & t0 = trajectories[j].getStartTime();
                int ind = elapsedSecs - (t0.first*60 + t0.second);
                if ((ind >= 0) && (ind < trajectories[j].getCountPoints())){
                    std::tuple<sf::Vertex, sf::Vector2f, int> initParams = {
                        trajectories[j].getPoint(ind),
                        trajectories[j].getDirection(ind),
                        trajectories[j].getSpeed()
                    };
                    incomingTrajectories.push_back(Trajectory(initParams));
                }
            }
        }
    }

    static void initExistingTrajectories(void) {
        if(existingTrajectories.empty()) {
            for (int i = 0; i < definedTrajectories.size(); i++) {
                if (definedTrajectories[i].trajectory.getStartTime() == std::make_pair(0,0)) {
                    std::tuple<sf::Vertex,sf::Vector2f,int> initParams = {
                        definedTrajectories[i].trajectory.getPoint(0),
                        definedTrajectories[i].trajectory.getDirection(0),
                        definedTrajectories[i].trajectory.getSpeed()
                    };
                    existingTrajectories.push_back(Trajectory(initParams));
                    existingTrajectories.back().setColor(sf::Color::Green);
                }
            }
        } 
    }

    static void drawRadars(sf::RenderWindow & win) {
        for (int i = 0; i < radars.size(); i++)
            radars[i].drawRadar(win);
    }

    static void drawTrajectories(sf::RenderWindow & win) {
        if (modeCP == mode::simulation) {
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
            ImGui::SetWindowPos(simWinPos);
            ImGui::SetWindowFontScale(fontScale);
            ImGui::SetNextItemWidth(timerWidth);
            ImGui::InputScalar("m", ImGuiDataType_U8, &timer.first, nullptr, 
                                nullptr, nullptr, ImGuiInputTextFlags_ReadOnly);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(timerWidth);
            ImGui::InputScalar("s", ImGuiDataType_U8, &timer.second, nullptr, 
                                nullptr, nullptr, ImGuiInputTextFlags_ReadOnly);
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
                    for (int i = 0; i < definedTrajectories.size(); i++)
                        updateTrajectoryTime(i);
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
                    initExistingTrajectories();
                    initIncomingTrajectories(0);
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
                    for (int i = 0; i < definedTrajectories[curInd].subTrajectories.size(); i++) {
                        definedTrajectories[curInd].subTrajectories[i]->getSpeed() 
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
        if (modeCP == mode::add) {
            static bool mousePressed = false;
            static bool trajectoryCreated = false;
            if (ev.type == sf::Event::MouseButtonPressed) {
                sf::Vector2f curMousePos(sf::Mouse::getPosition(win));
                std::pair<bool, bool> res = CollectionPoint::radarsAddPoint(curMousePos);
                if (res.first) {
                    definedTrajectories.push_back
                    (defTrajectory({Trajectory(curMousePos), std::vector<Trajectory*>{}}));
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
                        (defTrajectory({Trajectory(curMousePos), std::vector<Trajectory*>{}}));
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
        }
    }

private:
    struct defTrajectory {
        Trajectory trajectory;
        std::vector<Trajectory*> subTrajectories;
    };
    static inline std::pair<int,int> timer = {59,59};
    static inline mode modeCP = mode::stay;
    static inline bool isDefiningTrajectories = false;
    static inline std::vector<defTrajectory> definedTrajectories; 
    static inline std::vector<Trajectory> existingTrajectories;
    static inline std::vector<Trajectory> incomingTrajectories;
    static inline std::vector<Radar> radars;
    static inline float center = centerCP;
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