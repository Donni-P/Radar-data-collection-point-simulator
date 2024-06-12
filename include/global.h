#pragma once
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
constexpr float sigma = 1.f;
constexpr float epsilon = 1.5f;
constexpr float minDistanceToAttract = 2.5;
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