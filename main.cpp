#include <string>
#include <iostream>
#include "DebugHelper.h"
#include "SolarSystem.h"
#include "ConfigParser.h"
#include "FunctorHelpers.h"
#include "FileContentIO.h"
#include "OBJFileWriter.h"
#include "ScopeProfiler.h"
#include "GregorianTime.h"

SolarSystem solarSystem;

void init() {
    //auto configString1 = FileContentIO("real_solar_system/gravity_model.cfg").getContent();
    //auto configString2 = FileContentIO("real_solar_system/initial_state_jd_2433282_500000000.cfg").getContent();
    auto configString1 = FileContentIO("mini_solar_system/gravity_model.cfg").getContent(); // mini_solar_system: only Sun, Earth, Moon
    auto configString2 = FileContentIO("mini_solar_system/initial_state_jd_2433282_500000000.cfg").getContent();
    ConfigParser configParser;
    configParser.parse(configString1);
    configParser.parse(configString2);
    solarSystem.initializeFromConfig(configParser.getConfig());
}

void detectTransits(size_t earthId, size_t sunId, size_t moonId,
                    Degrees earthLatitude, Degrees earthLongitude, Kilometers earthAltitude) {

    auto earthRadius = solarSystem.gravityModel.getBodyGravityModel(earthId).referenceRadius;
    auto sunRadius = solarSystem.gravityModel.getBodyGravityModel(sunId).referenceRadius;
    auto moonRadius = solarSystem.gravityModel.getBodyGravityModel(moonId).referenceRadius;
    auto earthOffset = Vector3<Kilometers>().fromSpherical(earthAltitude + earthRadius, earthLatitude, -earthLongitude);

    //std::cout << earthOffset.x << ' ' << earthOffset.y << ' ' << earthOffset.z << '\n';
    //std::cout << earthOffset.length() << ' ' << earthOffset.declination() << ' ' << earthOffset.rightAscension() << '\n';

    auto earthFrame = solarSystem.getBodyFixedReferenceFrame(earthId);
    ReferenceFrame locationFrame{
        FrameRotation::fromZenithAndNorth(earthOffset, Vector3<Real>{0, 0, 1}),
        BodyTrajectory::fromConstantPosition(earthOffset)};

    auto sunPath = solarSystem.getBodyTrajectory(sunId);
    earthFrame.worldToLocal(sunPath);
    locationFrame.worldToLocal(sunPath);
    auto moonPath = solarSystem.getBodyTrajectory(moonId);
    earthFrame.worldToLocal(moonPath);
    locationFrame.worldToLocal(moonPath);

    //Vector3<Real> test{0, 0, 10000000.0};
    //locationFrame.worldToLocal(test, 2433713.388863);
    //std::cout << test.length() << ' ' << test.declination() << ' ' << test.rightAscension() - 180 << '\n';
    //return;

    for (size_t i = 0; i < sunPath.numHistoryCount(); i++) {
        auto sunPos = sunPath.positionHistory[i];
        auto instant = sunPath.historyInstants[i];
        auto moonPos = moonPath.positionAtInstant(instant);
        Degrees sunVisualRadius = std::asin(sunRadius * sunPos.lengthInversed()) / kDegrees;
        Degrees moonVisualRadius = std::asin(moonRadius * moonPos.lengthInversed()) / kDegrees;
        Degrees diffAngle = sunPos.angleBetween(moonPos);
        if (diffAngle >= sunVisualRadius + moonVisualRadius) continue;
        Real moonSunRatio = moonVisualRadius / sunVisualRadius;
        Real progressRate = 1.0 - diffAngle / (sunVisualRadius + moonVisualRadius);
        std::cout << "transit " << GregorianTime::fromJulianDays(instant).toString();
        std::cout << " JD" << std::to_string(instant);
        std::cout << " alt " << AngleInDegrees{sunPos.declination()}.toString();
        std::cout << " az " << AngleInDegrees{sunPos.rightAscension()}.toStringAs360();
        std::cout << " ratio " << moonSunRatio * 100 << "%";
        std::cout << " rate " << progressRate * 100 << "%";
        std::cout << '\n';
    }
}

void analysis() {
    DefScopeProfiler;
    std::cout << "analysising trajectory\n";
    auto earthId = solarSystem.getBodyIndexByName("Earth");
    auto sunId = solarSystem.getBodyIndexByName("Sun");
    auto moonId = solarSystem.getBodyIndexByName("Moon");
    detectTransits(earthId, sunId, moonId, 30.0, 120.0, 0.0); // Shanghai at 30°N 120°E
}

void finish() {
    DefScopeProfiler;
    std::cout << "saving OBJ file\n";
    OBJFileWriter obj{FileContentIO("/tmp/solarSystem.obj").writeStream()};

    ReferenceFrame frame;
    frame = solarSystem.getBodyInertialReferenceFrame(solarSystem.getBodyIndexByName("Earth"));
    //frame = solarSystem.getBodyFixedReferenceFrame(solarSystem.getBodyIndexByName("Earth"));
    //frame = solarSystem.getBodyAlignedReferenceFrame(solarSystem.getBodyIndexByName("Earth"), solarSystem.getBodyIndexByName("Sun"));

    obj.addComment("SolarSystem 1:149597870700");
    for (size_t i = 0; i < solarSystem.numBodies(); i++) {
        if (1
            && i != solarSystem.getBodyIndexByName("Sun")
            && i != solarSystem.getBodyIndexByName("Moon")
            //&& i != solarSystem.getBodyIndexByName("Mercury")
            //&& i != solarSystem.getBodyIndexByName("Venus")
            //&& i != solarSystem.getBodyIndexByName("Mars")
            //&& i != solarSystem.getBodyIndexByName("Jupiter")
            //&& i != solarSystem.getBodyIndexByName("Saturn")
            ) continue;
        auto trajectory = solarSystem.getBodyTrajectory(i);
        trajectory.resampleDensity(24.0 / 24.0); // 24 hours per OBJ point
        frame.worldToLocal(trajectory);
        trajectory.normalizePositions(149597870.7, true); // project to celestial sphere
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
    const JulianDays maxTime = 2.0 * 365.0; // simulate for 2 years
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

    analysis();
    //finish();
    return 0;
}
