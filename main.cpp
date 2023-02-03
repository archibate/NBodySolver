#include <string>
#include <iostream>
#include <iomanip>
#include "SolarSystem.h"
#include "ConfigParser.h"
#include "FunctorHelpers.h"
#include "FileContentIO.h"
#include "OBJFileWriter.h"
#include "ScopeProfiler.h"

SolarSystem solarSystem;
int idSun, idEarth, idMoon, idJupiter;

void record() {
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
    DefScopeProfiler;
    std::cout << "saving OBJ file\n";
    OBJFileWriter obj{FileContentIO("/tmp/solarSystem.obj").writeStream()};

    ReferenceFrame frame;
    //frame = solarSystem.getBodyInertialReferenceFrame(idJupiter);

    for (size_t i = 0; i < solarSystem.numBodies(); i++) {
        auto trajectory = solarSystem.getBodyTrajectory(i);
        trajectory.resampleDensity(1.0); // 1 day per OBJ point
        frame.worldToLocal(trajectory);
        obj.addCurve(trajectory.positionHistory, 1.0 / 149597870.0); // 1 AU -> 1 m in Blender
    }
}

int main() {
    DefScopeProfiler;

    init();
    solarSystem.takeSnapshot();
    std::cout << "solving " << solarSystem.numBodies() << " bodies\n";

    const JulianDays maxTime = 1.0 * 365.0; // simulate for 8 years
    const Seconds dt = 5 * 60.0; // 5 minutes per step
    const size_t numSubSteps = 3; // 15 minutes per snapshot
    const size_t maxI = (size_t)std::ceil(maxTime * 86400.0 / (numSubSteps * dt));
    for (size_t i = 0; i < maxI; i++) {
        solarSystem.evolveForTime(dt, numSubSteps);
        solarSystem.takeSnapshot();
        if (i % 1000 == 0) std::cout << '\r' << i << '/' << maxI << std::flush;
    }
    std::cout << "\rsimulation done\n";

    finish();
    return 0;
}
