#include <imgui.h>
#include <imgui-SFML.h>
#include <math.h>
#include <array>
#include <SFML/Graphics/ConvexShape.hpp>
#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/System/Clock.hpp>
#include <SFML/Window/Event.hpp>
constexpr float radiusR = 150.f; 
constexpr float radiusCP = 70.f;
constexpr float centerCP = 450.f;
constexpr int pointInCircle = 500;
sf::RenderStates states;

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
    ImVec2 pos(0,0);
    ImVec2 size(405,60);
    auto & style = ImGui::GetStyle();
    style.Alpha = 1.0f;
    CollectionPoint<2>::setRadarsPos();
    CollectionPoint<3>::setRadarsPos();
    CollectionPoint<4>::setRadarsPos();
    CollectionPoint<5>::setRadarsPos();
    sf::Clock deltaClock;
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            ImGui::SFML::ProcessEvent(window, event);
            if (event.type == sf::Event::Closed) {
                window.close();
            }
        }
        window.clear(sf::Color::White);

        ImGui::SFML::Update(window, deltaClock.restart());
        ImGui::SetNextWindowPos(pos);
        ImGui::SetNextWindowSize(size);

        ImGui::Begin("Menu", nullptr, ImGuiWindowFlags_NoCollapse);
        ImGui::PushItemWidth(50.0f);
        static bool b;
        ImGui::Checkbox("Add trajectory", &b);
        ImGui::SameLine();
        static char buffer[10] = "01:00"; 
        ImGui::InputText("Timer", buffer, IM_ARRAYSIZE(buffer));
        ImGui::SameLine();
        static int amountRads = 2;
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
        ImGui::SFML::Render(window);
        window.display();
    }

    ImGui::SFML::Shutdown();
}