#include <string>
#include <iostream>
#include <iomanip>
#include "SolarSystem.h"
#include "ConfigParser.h"
#include "FunctorHelpers.h"
#include "FileContentIO.h"
#include "OBJFileWriter.h"
#include "ScopeProfiler.h"
#include "GregorianTime.h"

SolarSystem solarSystem;

void init() {
    {
        auto configString1 = FileContentIO("real_solar_system/gravity_model.cfg").getContent();
        auto configString2 = FileContentIO("real_solar_system/initial_state_jd_2433282_500000000.cfg").getContent();
        ConfigParser configParser;
        configParser.parse(configString1);
        configParser.parse(configString2);
        solarSystem.initializeFromConfig(configParser.getConfig());
    }
}

void finish() {
    DefScopeProfiler;
    std::cout << "saving OBJ file\n";
    OBJFileWriter obj{FileContentIO("/tmp/solarSystem.obj").writeStream()};

    ReferenceFrame frame;
    //frame = solarSystem.getBodyInertialReferenceFrame(solarSystem.getBodyIndexByName("Earth"));
    //frame = solarSystem.getBodyFixedReferenceFrame(solarSystem.getBodyIndexByName("Earth"));
    frame = solarSystem.getBodyAlignedReferenceFrame(solarSystem.getBodyIndexByName("Earth"), solarSystem.getBodyIndexByName("Sun"));

    obj.addComment("SolarSystem 1:149597870700");
    for (size_t i = 0; i < solarSystem.numBodies(); i++) {
        if (1
            && i != solarSystem.getBodyIndexByName("Sun")
            //&& i != solarSystem.getBodyIndexByName("Moon")
            //&& i != solarSystem.getBodyIndexByName("Mercury")
            //&& i != solarSystem.getBodyIndexByName("Venus")
            //&& i != solarSystem.getBodyIndexByName("Mars")
            //&& i != solarSystem.getBodyIndexByName("Jupiter")
            //&& i != solarSystem.getBodyIndexByName("Saturn")
            ) continue;
        auto trajectory = solarSystem.getBodyTrajectory(i);
        trajectory.resampleDensity(24.0 / 24.0); // 24 hours per OBJ point
        frame.worldToLocal(trajectory);
        obj.addCurve(trajectory.positionHistory, 1.0 / 149597870.7); // 1 AU -> 1 m
    }
}

int main() {
    //Vector3<Real> v;
    //v = {std::sqrt(2), std::sqrt(2), 2};
    //std::cout << v.declination() << std::endl;
    //std::cout << v.rightAscension() << std::endl;
    //v.rotateByAxis({0, 0, 1}, 30.0);
    //std::cout << v.declination() << std::endl;
    //std::cout << v.rightAscension() << std::endl;
    //return 0;

    DefScopeProfiler;
    init();

    solarSystem.takeSnapshot();
    std::cout << "solving " << solarSystem.numBodies() << " bodies\n";
    const JulianDays maxTime = 1.0 * 365.0; // simulate for 1 year
    const Seconds dt = 5 * 60.0; // 5 minutes per step
    const size_t numSubSteps = 4; // 20 minutes per snapshot
    const size_t maxI = (size_t)std::ceil(maxTime * 86400.0 / (numSubSteps * dt));
    std::cout << "from " << GregorianTime::fromJulianDays(solarSystem.currentInstant).toString()
              << " to " << GregorianTime::fromJulianDays(solarSystem.currentInstant + maxTime).toString() << '\n';
    for (size_t i = 0; i < maxI; i++) {
        solarSystem.evolveForTime(dt, numSubSteps);
        solarSystem.takeSnapshot();
        if (i % 1000 == 0) std::cout << '\r' << i << '/' << maxI << std::flush;
    }
    std::cout << "\rsimulation done\n";

    finish();
    return 0;
}
