#include <string>
#include <iostream>
#include <iomanip>
#include "SolarSystem.h"
#include "ConfigParser.h"
#include "FunctorHelpers.h"
#include "FileContentIO.h"
#include "OBJFileWriter.h"

SolarSystem solarSystem;
int idSun, idEarth, idMoon, idJupiter;

void record() {
    solarSystem.takeSnapshot();
}

void step() {
    const Seconds dt = 6 * 60.0;
    solarSystem.computeNextState(dt);
    solarSystem.swapStates();
}

void init() {
    {
        auto configString1 = FileContentIO("real_solar_system/gravity_model.cfg").getContent();
        auto configString2 = FileContentIO("real_solar_system/initial_state_jd_2433282_500000000.cfg").getContent();
        ConfigParser configParser;
        configParser.parse(configString1);
        configParser.parse(configString2);
        solarSystem.initializeFromConfig(configParser.getConfig());
    }
    {
        idSun = solarSystem.getBodyIndexByName("Sun");
        idEarth = solarSystem.getBodyIndexByName("Earth");
        idMoon = solarSystem.getBodyIndexByName("Moon");
        idJupiter = solarSystem.getBodyIndexByName("Jupiter");
    }
}

void finish() {
    OBJFileWriter obj{FileContentIO("/tmp/solarSystem.obj").writeStream()};
    ReferenceFrame frame;
    frame = solarSystem.getBodyFixedReferenceFrame(idJupiter);
    for (size_t i = 0; i < solarSystem.numBodies(); i++) {
        auto trajectory = solarSystem.getBodyTrajectory(i);
        frame.worldToLocal(trajectory);
        obj.addCurve(trajectory.positionHistory, 1.0 / 149597870.0);
    }
}

int main() {
    init();
    record();

    const size_t maxI = 1 * 365 * 4;
    for (size_t i = 0; i < maxI; i++) {
        const size_t maxJ = 60;
        for (size_t j = 0; j < maxJ; j++) {
            step();
        }
        record();
        std::cout << i << '/' << maxI << std::endl;
    }

    finish();
    return 0;
}
