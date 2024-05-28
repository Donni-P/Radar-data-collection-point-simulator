#include <imgui.h>
#include <imgui-SFML.h>
#include <math.h>
#include <array>
#include <vector>
#include <SFML/Graphics/Texture.hpp>
#include <SFML/Graphics/Sprite.hpp>
#include <SFML/Graphics/ConvexShape.hpp>
#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/System/Clock.hpp>
#include <SFML/Window/Event.hpp>
constexpr float radiusR = 150.f; 
constexpr float radiusCP = 70.f;
constexpr float centerCP = 450.f;
constexpr float rangeBetweenVertexes = 2.f;
constexpr int pointInCircle = 500;
sf::RenderStates states;
std::vector<sf::VertexArray> addedTrajectories;

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

private:
    std::vector<sf::VertexArray> trajectories;
    sf::CircleShape radar;
};


template <int amountRadars>
class CollectionPoint {
public:
    static void setRadarsPos(void) {
        for(int i = 1; i <= amountRadars; i++){
            float angleInRadians = ((360/amountRadars)*M_PI/180) * i;
            radars[i-1].init(sf::Vector2f(center + cosf(angleInRadians)*radiusCP, center - sinf(angleInRadians)*radiusCP));
        }
    }

    static void drawRadars(sf::RenderWindow & win) {
        for(int i = 0; i < amountRadars; i++)
            radars[i].drawRadar(win);
    }
private:
    static inline std::array<Radar, amountRadars> radars;
    static inline float center = centerCP;
};


int main() {
    states.blendMode = sf::BlendMultiply;
    sf::RenderWindow window(sf::VideoMode(900, 900), "RDCP Simulator");
    window.setFramerateLimit(60);
    ImGui::SFML::Init(window);

    //===========preferences IMGUI======================
    ImVec2 menuPos(0,0);
    static constexpr float fontScale = 1.3f;
    static constexpr float timerWidth = 65.f;
    static constexpr float radSliderWidth = 80.f; 
    auto & style = ImGui::GetStyle();
    style.Alpha = 1.0f;
    static bool addMode = false;
    static char timer[10] = "01:00"; 
    static int amountRads = 2;
    //=================================================
    CollectionPoint<2>::setRadarsPos();
    CollectionPoint<3>::setRadarsPos();
    CollectionPoint<4>::setRadarsPos();
    CollectionPoint<5>::setRadarsPos();
    sf::Clock deltaClock;
    static int amountAddedTr = 0;
    sf::VertexArray TrajectoryToBeAdded;
    static bool isProcessAdding = false;
    //==================================================
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            ImGui::SFML::ProcessEvent(window, event);
            if (event.type == sf::Event::Closed) {
                window.close();
            }
            if((addMode) && (event.type == sf::Event::MouseButtonPressed)){
                sf::Vertex point(sf::Vector2f(sf::Mouse::getPosition(window)));
                point.color = sf::Color::Blue; 
                addedTrajectories.push_back(TrajectoryToBeAdded);
                addedTrajectories[amountAddedTr].setPrimitiveType(sf::LineStrip);
                addedTrajectories[amountAddedTr].append(point);
                isProcessAdding = true;
            }
            if(isProcessAdding && (event.type == sf::Event::MouseMoved)) {
                sf::Vertex point(sf::Vector2f(sf::Mouse::getPosition(window)));
                point.color = sf::Color::Blue; 
                addedTrajectories[amountAddedTr].append(point);
            }
            if((addMode) && (event.type == sf::Event::MouseButtonReleased)){
                amountAddedTr++;
                TrajectoryToBeAdded.clear();
                isProcessAdding = false;
            }
        }
        window.clear(sf::Color::White);

        ImGui::SFML::Update(window, deltaClock.restart());
        ImGui::Begin("Menu", nullptr, ImGuiWindowFlags_NoCollapse | 
                                      ImGuiWindowFlags_NoResize | 
                                      ImGuiWindowFlags_AlwaysAutoResize |
                                      ImGuiWindowFlags_NoMove);
        ImGui::SetWindowPos(menuPos);
        ImGui::SetWindowFontScale(fontScale);
        if (ImGui::Button("Add Mode")){
            addMode = !addMode;
            (addMode) ? ImGui::PushStyleColor(ImGuiCol_Button, sf::Color::Red) 
                      : ImGui::PopStyleColor();
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(timerWidth);
        ImGui::InputText("Timer", timer, IM_ARRAYSIZE(timer));
        ImGui::SameLine();
        ImGui::SetNextItemWidth(radSliderWidth);
        ImGui::SliderInt("Amount of RADARs", &amountRads, 2, 5);
        ImGui::End();
        switch(amountRads){
            case 2:
                CollectionPoint<2>::drawRadars(window);
                break;
            case 3:
                CollectionPoint<3>::drawRadars(window);
                break;
            case 4:
                CollectionPoint<4>::drawRadars(window);
                break;
            case 5:
                CollectionPoint<5>::drawRadars(window);
                break;   
            default:
                break;
        }

        for(int i = 0; (i < amountAddedTr) || ((i <= amountAddedTr) && (isProcessAdding)); i++)
            window.draw(addedTrajectories[i]);
        ImGui::SFML::Render(window);
        window.display();
    }

    ImGui::SFML::Shutdown();
}