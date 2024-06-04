#include <imgui.h>
#include <imgui-SFML.h>
#include <math.h>
#include <array>
#include <vector>
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
constexpr float T = 10.f;
constexpr int minPointsInTrajectory = 2;
constexpr int pointInCircle = 500;
    ImVec2 menuPos(0,0);
    ImVec2 configWinPos(0,70);
    ImVec2 simWinPos(400,70);
    static constexpr float fontScale = 1.3f;
    static constexpr float timerWidth = 25.f;
    static constexpr float sliderWidth = 80.f; 
    static constexpr ImVec2 separator(15,1);
sf::RenderStates states;

class Trajectory;
namespace given {
    std::vector<Trajectory> trajectories;
    bool isProcessAdding = false;
} 

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
        for(int i = 0; i < trajectory.getVertexCount(); i++) {
            trajectory[i].color = col;
        }
        startPoint.setFillColor(col);
        endPoint.setFillColor(col);
    }

    sf::Vertex & getPoint(int ind) { return trajectory[ind]; }

    int getCountPoints(void) { return trajectory.getVertexCount(); }

    int * getSpeed(void) { return &speed; }

    int * getMin0(void) { return &min0; }

    int * getSec0(void) { return &sec0; }
private:
    sf::CircleShape startPoint; 
    sf::CircleShape endPoint;
    sf::VertexArray trajectory;
    sf::Color color = sf::Color::Blue;
    int min0 = 0;
    int sec0 = 0;
    int speed = 1;
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
    /*void drawTrajectories(sf::RenderWindow & win) {
        for(int i = 0; i < trajectories.size(); i++)
            trajectories[i].drawTrajectory(win);
    }*/
    bool addIfContainsPoint(sf::Vector2f &point){
        bool isContains = sqrt(pow((point.x - radar.getPosition().x), 2) + 
                               pow((point.y - radar.getPosition().y), 2)) < radiusR;
        bool isDistance = true;
        Trajectory * currentTrajectory = nullptr;
        if (isContains) {
            if (!isAddingTrajectory) {
                isAddingTrajectory = true;
                trajectories.push_back(Trajectory(point));
            }
            currentTrajectory = &trajectories.back();
            sf::Vertex lastPoint = currentTrajectory->getPoint(currentTrajectory->getCountPoints() - 1);
            float distance = sqrt(pow((lastPoint.position.x - point.x), 2) +
                                pow((lastPoint.position.y - point.y), 2));
            if (distance >= T) {
                isDistance = true;
                sf::Vector2f direction = point - lastPoint.position;
                direction /= distance;
                point = lastPoint.position + direction*T;  
            } else { isDistance = false; }

            if (isDistance) {
                currentTrajectory->addPoint(point);
            }
        } else if (isAddingTrajectory) { isAddingTrajectory = false; }

        return isContains;
    }
    void setAddingTrajectory(bool b) {
        isAddingTrajectory = b;
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
        simulation
    };

    static void init(void) {
        //===========preferences IMGUI======================
        auto & style = ImGui::GetStyle();
        style.Alpha = 1.0f;
        style.ItemSpacing.y = 6.f;
        //=================================================
        setRadarsPos(2);
    }

    static void setRadarsPos(int amountRadars) {
        radars.resize(amountRadars);
        for(int i = 1; i <= amountRadars; i++){
            float angleInRadians = ((360/amountRadars)*M_PI/180) * i;
            radars[i-1].init(sf::Vector2f(center + cosf(angleInRadians)*radiusCP, center - sinf(angleInRadians)*radiusCP));
        }
    }

    static void drawRadars(sf::RenderWindow & win) {
        for(int i = 0; i < radars.size(); i++)
            radars[i].drawRadar(win);
    }

    static void drawTrajectories(sf::RenderWindow & win) {
        for (int i = 0; i < definedTrajectories.size(); i++)
            definedTrajectories[i].drawTrajectory(win);
    }

    static bool radarsAddPoint(sf::Vector2f point) {
        bool res = false;
        for(int i = 0; i < radars.size(); i++) {
            res = res || radars[i].addIfContainsPoint(point);
        }
        return res;
    }

    static void setRadarsAddingState(bool b) {
        for(int i = 0; i < radars.size(); i++) {
            radars[i].setAddingTrajectory(b);
        }
    }

    static void clearRadarsTrajectories(void) {
        for(int i = 0; i < radars.size(); i++) {
            radars[i].clearTrajectories();
        }
    }

    static void drawGUI(void) {
        static int curInd = 0;
        static unsigned int mins = 59;
        static unsigned int secs = 59;
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
            if (modeCP == mode::stay)
                modeCP = mode::add;
            else
                modeCP = mode::stay;
        }
        ImGui::PopStyleColor();
        ImGui::SameLine();
        ImGui::Dummy(separator);
        ImGui::SameLine();
        (modeCP == mode::config) ? ImGui::PushStyleColor(ImGuiCol_Button, sf::Color::Red) 
                                 : ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_Button));
        if (ImGui::Button("Config Mode")) {
            if (!definedTrajectories.empty() && (modeCP != mode::add) && (modeCP != mode::simulation)) {
                definedTrajectories[curInd].setColor(sf::Color::Blue);
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
        if ((mins < 1) || (mins > 59)) mins = 59;
        ImGui::InputScalar("m", ImGuiDataType_U8, &mins, nullptr, nullptr, nullptr, 
        (modeCP == mode::simulation) ? ImGuiInputTextFlags_ReadOnly : ImGuiInputTextFlags_None);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(timerWidth);
        if (secs > 59) secs = 59;
        ImGui::InputScalar("s", ImGuiDataType_U8, &secs, nullptr, nullptr, nullptr, 
        (modeCP == mode::simulation) ? ImGuiInputTextFlags_ReadOnly : ImGuiInputTextFlags_None);
        ImGui::SameLine();
        ImGui::Dummy(separator);
        ImGui::SameLine();
        ImGui::Text("Amount of radars");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(sliderWidth);
        if (ImGui::SliderInt("##Amount of RADARs", &amountRads, 2, 5, "%d", 
        (modeCP == mode::simulation) ? ImGuiSliderFlags_NoInput : ImGuiSliderFlags_None))
            setRadarsPos(amountRads);
        ImGui::SameLine();
        ImGui::Dummy(separator);
        ImGui::SameLine();
        (modeCP == mode::simulation) ? ImGui::PushStyleColor(ImGuiCol_Button, sf::Color::Red) 
                                     : ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_Button));
        if (ImGui::Button("Start")) {
            if ((modeCP != mode::config) && (modeCP != mode::add)) {
                    if (modeCP == mode::stay)
                    modeCP = mode::add;
                else
                    modeCP = mode::stay;
            }
        }
        ImGui::PopStyleColor();
        ImGui::End();
        if (modeCP == mode::config) {
            ImGui::Begin("Trajectory", nullptr, ImGuiWindowFlags_NoCollapse | 
                                                ImGuiWindowFlags_NoResize | 
                                                ImGuiWindowFlags_AlwaysAutoResize |
                                                ImGuiWindowFlags_NoMove);
            definedTrajectories[curInd].setColor(sf::Color::Red);
            ImGui::SetWindowPos(configWinPos);
            ImGui::Text("Speed");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(sliderWidth);
            ImGui::SliderInt("pix/sec", definedTrajectories[curInd].getSpeed(),1,4);
            ImGui::Text("t0");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(timerWidth);
            int * min0 = definedTrajectories[curInd].getMin0();
            if (*min0 > mins) *min0 = 0;
            ImGui::InputScalar("m", ImGuiDataType_U8, min0);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(timerWidth);
            int * sec0 = definedTrajectories[curInd].getSec0();
            if ((*min0 == mins) && (*sec0 > secs)) *sec0 = 0;
            ImGui::InputScalar("s", ImGuiDataType_U8, sec0);
            if (ImGui::Button("<")) {
                definedTrajectories[curInd].setColor(sf::Color::Blue);
                if (curInd == 0)
                    curInd = definedTrajectories.size() - 1;
                else 
                    curInd--;
            }
            ImGui::SameLine();
            if (ImGui::Button(">")) {
                definedTrajectories[curInd].setColor(sf::Color::Blue);
                if (curInd == (definedTrajectories.size() - 1))
                    curInd = 0;
                else
                    curInd++;
            }
            ImGui::End(); 
        }
        if (modeCP == mode::simulation) {
            /*ImGui::Begin("Simulation", nullptr, ImGuiWindowFlags_NoCollapse | 
                                                ImGuiWindowFlags_NoResize | 
                                                ImGuiWindowFlags_AlwaysAutoResize |
                                                ImGuiWindowFlags_NoMove);
            ImGui::SetWindowPos(simWinPos);
            (isPause) ? ImGui::PushStyleColor(ImGuiCol_Button, sf::Color::Red) 
                      : ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_Button));
            if (ImGui::Button("Pause")) {
            }

            ImGui::PopStyleColor();
            ImGui::End();*/
        }
    }

    static void setDefiningTrajectories(bool b) { isDefiningTrajectories = b; }

    static std::vector<Radar> & getRadars(void) { return radars; }

    static mode getMode(void) { return modeCP; }

    static void setMode(mode newMode) { modeCP = newMode; }

    static void eventsProcessing(sf::Event ev, sf::RenderWindow & win) {
        switch(modeCP) {
            case mode::stay:
                break;
            case mode::add:
                static bool mousePressed = false;
                static bool trajectoryCreated = false;
                if (ev.type == sf::Event::MouseButtonPressed) {
                    sf::Vector2f curMousePos(sf::Mouse::getPosition(win));
                    if (CollectionPoint::radarsAddPoint(curMousePos)) {
                        definedTrajectories.push_back(Trajectory(curMousePos));
                        trajectoryCreated = true;
                    }
                    mousePressed = true;
                } 
                if (mousePressed && (ev.type == sf::Event::MouseMoved)) {
                    sf::Vector2f curMousePos(sf::Mouse::getPosition(win));
                    if (CollectionPoint::radarsAddPoint(curMousePos)) {
                        if (!trajectoryCreated) {
                            definedTrajectories.push_back(Trajectory(curMousePos));
                            trajectoryCreated = true;
                        } else {
                            definedTrajectories.back().addPoint(curMousePos);
                        }
                    } else { trajectoryCreated = false; }
                }
                if (ev.type == sf::Event::MouseButtonReleased) {
                    mousePressed = false;
                    trajectoryCreated = false;
                }
                break;
            case mode::config:
                break;
            case mode::simulation:
                break;
            default:
                break;
        }
    }

private:
    /*struct defTrajectory {
        Trajectory trajectory;
        std::vector<Trajectory&> subTrajectories;
    };*/
    static inline mode modeCP = mode::stay;
    static inline bool isDefiningTrajectories = false;
    static inline std::vector<Trajectory> definedTrajectories; 
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

        ImGui::SFML::Update(window, deltaClock.restart());
        CollectionPoint::drawGUI();

        window.clear(sf::Color::White);
        CollectionPoint::drawRadars(window);
        CollectionPoint::drawTrajectories(window);
        ImGui::SFML::Render(window);
        window.display();
    }

    ImGui::SFML::Shutdown();
}