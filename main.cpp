#include<radar.h>
#include<trajectory.h>
#include<collection_point.h>

int main() {
    states.blendMode = sf::BlendMultiply;
    sf::RenderWindow window(sf::VideoMode(900, 900), "RDCP Simulator");
    window.setFramerateLimit(60);
    ImGui::SFML::Init(window);


    CollectionPoint::init();
    sf::Clock deltaClock;
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