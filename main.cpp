#include <string>
#include <iostream>
#include "DebugHelper.h"
#include "SolarSystem.h"
#include "ConfigParser.h"
#include "FunctorHelpers.h"
#include "FileContentIO.h"
#include "OBJFileWriter.h"
#include "GregorianTime.h"
#include "KeplerianOrbit.h"
#include "ScopeProfiler.h"
#include "ProgressBar.h"

SolarSystem solarSystem;

void init() {
    DefScopeProfiler;
    BakedAssocLaguerre::bake();
    ConfigParser configParser;
    //auto configString1 = FileContentIO("real_solar_system/gravity_model.cfg").getContent();
    //auto configString2 = FileContentIO("real_solar_system/initial_state_jd_2433282_500000000.cfg").getContent();
    auto configString1 = FileContentIO("mini_solar_system/gravity_model.cfg").getContent(); // mini_solar_system: only Sun, Earth, Moon
    auto configString2 = FileContentIO("mini_solar_system/initial_state_jd_2433282_500000000.cfg").getContent();
    configParser.parse(configString1);
    configParser.parse(configString2);
    solarSystem.initializeFromConfig(configParser.getConfig());
}

void addVesselOrbitAround(size_t earthId, KeplerianOrbit const &orbit) {
    auto instant = solarSystem.currentState.instant;
    auto [position, velocity] = orbit.getPositionAndVelocity(
        solarSystem.gravityModel.getBodyGravityModel(earthId).gravitationalParameter, instant);
    auto earthRotation = solarSystem.getBodyRotation(earthId);
    earthRotation.toInertial();
    earthRotation.worldToLocal(position, instant);
    earthRotation.worldToLocal(velocity, instant);
    position += solarSystem.currentState.positions[earthId];
    velocity += solarSystem.currentState.velocities[earthId];
    solarSystem.addCustomVessel(position, velocity, FrameRotation{});
}

void putgarbage() {
    size_t earthId = solarSystem.getBodyIndexByName("Earth");
    KeplerianOrbit satOrbit;
    satOrbit.semiMajorAxis = solarSystem.gravityModel.getBodyGravityModel(earthId).referenceRadius + 500.0;
    satOrbit.inclination = 50.0;
    addVesselOrbitAround(earthId, satOrbit);
}

void detectTransits(size_t earthId, size_t sunId, size_t moonId,
                    Degrees earthLatitude, Degrees earthLongitude, Kilometers earthAltitude) {
    DefScopeProfiler;

    auto earthRadius = solarSystem.gravityModel.getBodyGravityModel(earthId).referenceRadius;
    auto sunRadius = solarSystem.gravityModel.getBodyGravityModel(sunId).referenceRadius;
    auto moonRadius = solarSystem.gravityModel.getBodyGravityModel(moonId).referenceRadius;
    auto earthOffset = Vector3<Kilometers>().fromSpherical(earthAltitude + earthRadius, earthLatitude, -earthLongitude);

    //std::cout << earthOffset.x << ' ' << earthOffset.y << ' ' << earthOffset.z << '\n';
    //std::cout << earthOffset.length() << ' ' << earthOffset.declination() << ' ' << earthOffset.rightAscension() << '\n';

    auto earthFrame = solarSystem.getBodyFixedReferenceFrame(earthId);
    ReferenceFrame locationFrame{
        //FrameRotation::fromZenithAndNorth(earthOffset, Vector3<Real>{0, 0, 1}),
        FrameRotation::fromDirection({0, 0, 1}),
        BodyTrajectory::fromConstantPosition(earthOffset)};

    auto sunPath = solarSystem.getBodyTrajectory(sunId);
    earthFrame.worldToLocal(sunPath);
    locationFrame.worldToLocal(sunPath);
    auto moonPath = solarSystem.getBodyTrajectory(moonId);
    earthFrame.worldToLocal(moonPath);
    locationFrame.worldToLocal(moonPath);

    //Vector3<Real> test{10000000.0, 0, 0};
    //locationFrame.worldToLocal(test, 0);
    //std::cout << test.length() << ' ' << test.declination() << ' ' << test.rightAscension() << '\n';
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
        std::cout << " dec " << AngleInDegrees{sunPos.declination()}.toString();
        std::cout << " ra " << AngleInDegrees{sunPos.rightAscension() + earthFrame.rotation.angleAtInstant(instant)}.toStringAsHourAngle();
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

void dump() {
    DefScopeProfiler;
    std::cout << "saving OBJ file\n";
    OBJFileWriter obj{FileContentIO("/tmp/solarSystem.obj").writeStream()};

    ReferenceFrame frame;
    frame = solarSystem.getBodyInertialReferenceFrame(solarSystem.getBodyIndexByName("Earth"));
    //frame = solarSystem.getBodyFixedReferenceFrame(solarSystem.getBodyIndexByName("Earth"));
    //frame = solarSystem.getBodyAlignedReferenceFrame(solarSystem.getBodyIndexByName("Earth"), solarSystem.getBodyIndexByName("Sun"));

    obj.addComment("SolarSystem 1:149597870700");
    for (size_t i = 0; i < solarSystem.numBodies(); i++) {
        if (0
            //&& i != solarSystem.getBodyIndexByName("Sun")
            //&& i != solarSystem.getBodyIndexByName("Moon")
            //&& i != solarSystem.getBodyIndexByName("Mercury")
            //&& i != solarSystem.getBodyIndexByName("Venus")
            //&& i != solarSystem.getBodyIndexByName("Mars")
            //&& i != solarSystem.getBodyIndexByName("Jupiter")
            //&& i != solarSystem.getBodyIndexByName("Saturn")
            ) continue;
        auto trajectory = solarSystem.getBodyTrajectory(i);
        trajectory.resampleDensity(1.0 / 24.0 / 60.0); // 1 minutes per OBJ point
        frame.worldToLocal(trajectory);
        //trajectory.normalizePositions(149597870.7, true); // project to celestial sphere
        //obj.addCurve(trajectory.positionHistory, 1.0 / 149597870.7); // 1 AU -> 1 m
        obj.addCurve(trajectory.positionHistory, 1.0 / 6378.1363); // 1 earth radius -> 1 m
    }
}

void compute() {
    DefScopeProfiler;
    std::cout << "solving " << solarSystem.numBodies() << " bodies\n";
    const JulianDays maxTime = 30.0; // simulate for 30 days
    const Seconds dt = 2.0; // 2 seconds per step
    const size_t numSubSteps = 30; // 1 minute per snapshot
    const size_t maxI = (size_t)std::ceil(maxTime * 86400.0 / (numSubSteps * dt));
    std::cout << "from " << GregorianTime::fromJulianDays(solarSystem.currentState.instant).toString()
              << " to " << GregorianTime::fromJulianDays(solarSystem.currentState.instant + maxTime).toString() << '\n';
    ProgressBar progressBar;
    for (size_t i = 0; i < maxI; i++) {
        solarSystem.evolveForTime(dt, numSubSteps);
        solarSystem.takeSnapshot();
        if (i % 1000 == 0) progressBar(i, maxI);
    }
    std::cout << "\nsimulation done\n";
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
    init();
    putgarbage();
    compute();
    //analysis();
    dump();
    ScopeProfiler::printLog();
    return 0;
}
// rk4 dt=5min: transit 1966/5/20 10:59:29.186 JD2439265.957977 dec 20°02′16.6″ ra 3h49m01.80s ratio 97.8435% rate 70.7393%
// rk4 dt=1min: transit 1966/5/20 10:59:29.186 JD2439265.957977 dec 20°02′10.7″ ra 3h48m59.93s ratio 97.8431% rate 71.3207%
// stellarium:  transit 1966/5/20 10:59:29.186 JD2439265.957977 dec 20°02′07.4″ ra 3h48m58.32s ratio 99.4000% rate 43.4200%
