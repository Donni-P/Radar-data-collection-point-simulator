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
constexpr float sigma = 2.f;
constexpr float epsilon = 2.f;
constexpr int minPointsInTrajectory = 20;
constexpr int pointInCircle = 500;
sf::RenderStates states;

class Trajectory;
namespace given {
    std::vector<Trajectory> trajectories;
    bool isProcessAdding = false;
} 

class Trajectory {
public:
    Trajectory(sf::Vector2f firstPoint) : startPoint(2.f), endPoint(2.f) {
        startPoint.setOrigin(2.f, 2.f);
        endPoint.setOrigin(2.f, 2.f);
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
        win.draw(trajectory);
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
        for(int i = 0; i < trajectories.size(); i++)
            trajectories[i].drawTrajectory(win);
    }
    void addIfContainsPoint(sf::Vector2f &point){
        bool isContains = sqrt(pow((point.x - radar.getPosition().x), 2) + 
                               pow((point.y - radar.getPosition().y), 2)) < radiusR;
        if (isContains && !isAddingTrajectory) {
            isAddingTrajectory = true;
            if(!trajectories.empty())
                if(trajectories.back().getCountPoints() < minPointsInTrajectory)
                    trajectories.pop_back();
            trajectories.push_back(Trajectory(point));
        } else if (isContains && isAddingTrajectory) {
            trajectories.back().addPoint(point);
        } else if (!isContains && isAddingTrajectory) {
            isAddingTrajectory = false;
        }
    }
    void clearTrajectories(void) {
        trajectories.clear();
    }

private:
    bool isAddingTrajectory = false;
    std::vector<Trajectory> trajectories;
    sf::CircleShape radar;
};


class CollectionPoint {
public:
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

    static void radarsAddPoint(sf::Vector2f point) {
        for(int i = 0; i < radars.size(); i++) {
            radars[i].addIfContainsPoint(point);
        }
    }

    static void clearRadarsTrajectories(void) {
        for(int i = 0; i < radars.size(); i++) {
            radars[i].clearTrajectories();
        }
    }

private:
    static inline std::vector<Radar> radars;
    static inline float center = centerCP;
};

int main() {
    states.blendMode = sf::BlendMultiply;
    sf::RenderWindow window(sf::VideoMode(900, 900), "RDCP Simulator");
    window.setFramerateLimit(60);
    ImGui::SFML::Init(window);

    //===========preferences IMGUI======================
    ImVec2 menuPos(0,0);
    ImVec2 configWinPos(0,70);
    ImVec2 simWinPos(400,70);
    static constexpr float fontScale = 1.3f;
    static constexpr float timerWidth = 25.f;
    static constexpr float sliderWidth = 80.f; 
    static constexpr ImVec2 separator(15,1);
    auto & style = ImGui::GetStyle();
    style.Alpha = 1.0f;
    style.ItemSpacing.y = 6.f;
    static bool addMode = false;
    static bool configMode = false;
    static bool simulationMode = false;
    static bool isPause = false;
    static int amountRads = 2;
    static unsigned int mins = 59;
    static unsigned int secs = 59;
    static int curConfigIndex = 0;
    //=================================================
    CollectionPoint::setRadarsPos(amountRads);
    sf::Clock deltaClock;
    //==================================================
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            ImGui::SFML::ProcessEvent(window, event);
            if (event.type == sf::Event::Closed) {
                window.close();
            }
            if (addMode && (event.type == sf::Event::MouseButtonPressed)) {
                sf::Vector2f curMousePos(sf::Mouse::getPosition(window));
                given::trajectories.push_back(Trajectory(curMousePos));
                CollectionPoint::radarsAddPoint(curMousePos);
                given::isProcessAdding = true;
            } 
            if (given::isProcessAdding && (event.type == sf::Event::MouseMoved)) {
                sf::Vector2f curMousePos(sf::Mouse::getPosition(window));
                CollectionPoint::radarsAddPoint(curMousePos);
                given::trajectories.back().addPoint(curMousePos);

            }
            if (addMode && (event.type == sf::Event::MouseButtonReleased)) {
                if(given::trajectories.back().getCountPoints() < minPointsInTrajectory)
                    given::trajectories.pop_back();
                given::isProcessAdding = false;
            }
        }

        ImGui::SFML::Update(window, deltaClock.restart());
        ImGui::Begin("Menu", nullptr, ImGuiWindowFlags_NoCollapse | 
                                      ImGuiWindowFlags_NoResize | 
                                      ImGuiWindowFlags_AlwaysAutoResize |
                                      ImGuiWindowFlags_NoMove);
        ImGui::SetWindowPos(menuPos);
        ImGui::SetWindowFontScale(fontScale);
        (addMode) ? ImGui::PushStyleColor(ImGuiCol_Button, sf::Color::Red) 
                  : ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_Button));
        if (ImGui::Button("Add Mode")) {
            if (!configMode && !simulationMode)
                addMode = !addMode;
        }
        ImGui::PopStyleColor();
        ImGui::SameLine();
        ImGui::Dummy(separator);
        ImGui::SameLine();
        (configMode) ? ImGui::PushStyleColor(ImGuiCol_Button, sf::Color::Red) 
                     : ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_Button));
        if (ImGui::Button("Config Mode")) {
            if (!given::trajectories.empty() && !addMode && !simulationMode) {
                given::trajectories[curConfigIndex].setColor(sf::Color::Blue);
                configMode = !configMode;
            }
        }
        ImGui::PopStyleColor();
        ImGui::SameLine();
        ImGui::Dummy(separator);
        ImGui::SameLine();
        if (ImGui::Button("Clear")) {
            if (!configMode && !simulationMode) {
                given::trajectories.clear();
                CollectionPoint::clearRadarsTrajectories();
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
        (simulationMode) ? ImGuiInputTextFlags_ReadOnly : ImGuiInputTextFlags_None);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(timerWidth);
        if (secs > 59) secs = 59;
        ImGui::InputScalar("s", ImGuiDataType_U8, &secs, nullptr, nullptr, nullptr, 
        (simulationMode) ? ImGuiInputTextFlags_ReadOnly : ImGuiInputTextFlags_None);
        ImGui::SameLine();
        ImGui::Dummy(separator);
        ImGui::SameLine();
        ImGui::Text("Amount of radars");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(sliderWidth);
        if (ImGui::SliderInt("##Amount of RADARs", &amountRads, 2, 5, "%d", 
        (simulationMode) ? ImGuiSliderFlags_NoInput : ImGuiSliderFlags_None))
            CollectionPoint::setRadarsPos(amountRads);
        ImGui::SameLine();
        ImGui::Dummy(separator);
        ImGui::SameLine();
        (simulationMode) ? ImGui::PushStyleColor(ImGuiCol_Button, sf::Color::Red) 
                         : ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_Button));
        if (ImGui::Button("Start")) {
            if (!configMode && !addMode)
                simulationMode = !simulationMode;
        }
        ImGui::PopStyleColor();
        ImGui::End();
        if (configMode) {
            ImGui::Begin("Trajectory", nullptr, ImGuiWindowFlags_NoCollapse | 
                                                ImGuiWindowFlags_NoResize | 
                                                ImGuiWindowFlags_AlwaysAutoResize |
                                                ImGuiWindowFlags_NoMove);
            given::trajectories[curConfigIndex].setColor(sf::Color::Red);
            ImGui::SetWindowPos(configWinPos);
            ImGui::Text("Speed");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(sliderWidth);
            ImGui::SliderInt("pix/sec",given::trajectories[curConfigIndex].getSpeed(),1,4);
            ImGui::Text("t0");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(timerWidth);
            int * min0 = given::trajectories[curConfigIndex].getMin0();
            if (*min0 > mins) *min0 = 0;
            ImGui::InputScalar("m", ImGuiDataType_U8, min0);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(timerWidth);
            int * sec0 = given::trajectories[curConfigIndex].getSec0();
            if ((*min0 == mins) && (*sec0 > secs)) *sec0 = 0;
            ImGui::InputScalar("s", ImGuiDataType_U8, sec0);
            if (ImGui::Button("<")) {
                given::trajectories[curConfigIndex].setColor(sf::Color::Blue);
                if (curConfigIndex == 0)
                    curConfigIndex = given::trajectories.size() - 1;
                else 
                    curConfigIndex--;
            }
            ImGui::SameLine();
            if (ImGui::Button(">")) {
                given::trajectories[curConfigIndex].setColor(sf::Color::Blue);
                if (curConfigIndex == (given::trajectories.size() - 1))
                    curConfigIndex = 0;
                else 
                    curConfigIndex++;
            }
            ImGui::End(); 
        }
        if (simulationMode) {
            ImGui::Begin("Simulation", nullptr, ImGuiWindowFlags_NoCollapse | 
                                                ImGuiWindowFlags_NoResize | 
                                                ImGuiWindowFlags_AlwaysAutoResize |
                                                ImGuiWindowFlags_NoMove);
            ImGui::SetWindowPos(simWinPos);
            (isPause) ? ImGui::PushStyleColor(ImGuiCol_Button, sf::Color::Red) 
                      : ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_Button));
            if (ImGui::Button("Pause")) {
            }

            ImGui::PopStyleColor();
            ImGui::End();
        }

        window.clear(sf::Color::White);
        CollectionPoint::drawRadars(window);
        /*for(int i = 0; i < given::trajectories.size(); i++)
            given::trajectories[i].drawTrajectory(window);*/
        ImGui::SFML::Render(window);
        window.display();
    }

    ImGui::SFML::Shutdown();
}