cmake_minimum_required(VERSION 3.10)
project(MyProject)

set(SFML_DIR SFML-2.6.1)
set(IMGUI_DIR imgui-1.90.6)
set(IMGUI_SFML_DIR imgui-sfml-2.6)

# Находим SFML
find_package(SFML COMPONENTS system window graphics REQUIRED)
include_directories(
  ${SFML_DIR}
  ${IMGUI_DIR} 
  ${IMGUI_SFML_DIR}
) 

set(OpenGL_GL_PREFERENCE "LEGACY")
find_package(OpenGL REQUIRED)

# Создаем статическую библиотеку ImGui
add_library(ImGui STATIC
  ${IMGUI_DIR}/imgui.cpp 
  ${IMGUI_DIR}/imgui_widgets.cpp 
  ${IMGUI_DIR}/imgui_draw.cpp 
  ${IMGUI_DIR}/imgui_tables.cpp
)

# Добавляем ImGui-SFML к ImGui
target_sources(ImGui PRIVATE
  ${IMGUI_SFML_DIR}/imgui-SFML.cpp
)

# Указываем зависимости ImGui от SFML 
target_link_libraries(ImGui 
  sfml-graphics 
  sfml-window 
  sfml-system 
)

# Создаем исполняемый файл
add_executable(MyProject main.cpp)

# Линкуем библиотеки
target_link_libraries(MyProject 
    ImGui
    ${OPENGL_LIBRARIES}
)