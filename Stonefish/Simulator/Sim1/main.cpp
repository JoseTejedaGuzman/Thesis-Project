#include <Stonefish/core/GraphicalSimulationApp.h>
#include "MySimulationManager.h"

int main(int argc, char **argv)
{
    //Using default settings
    sf::RenderSettings s;
    sf::HelperSettings h;

    MySimulationManager manager(500.0);
    sf::GraphicalSimulationApp app("Simple simulator", "Sim1/MySimulationManager.h", s, h, &manager);
    app.Run();

    return 0;
}
