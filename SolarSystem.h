#pragma once

#include <array>
#include <string>
#include <vector>
#include <cassert>
#include "MathUtils.h"
#include "FunctorHelpers.h"
#include "ConfigParser.h"
#include "FunctorHelpers.h"

using BodyName = std::string;

struct BodyState {
    // 位置（千米）
    Vector3<Kilometers> position;
    // 速度（千米）
    Vector3<KilometersPerSecond> velocity;
};

struct GeoPotentialComponent {
    int degree = 0;
    int order = 0;
    // 以下分别为 C[degree, order] 和 S[degree, order] 的值
    // order 为 0 时，C[degree, 0] = J[degree]，S[degree, 0] = 0
    Real cosValue = 0.0;
    Real sinValue = 0.0;
};

struct FrameRotation {
    // 参考瞬间（儒略日数）
    JulianDays referenceInstant = 0.0;
    // 参考瞬间时0°经线位置（度）
    Degrees referenceAngle = 90.0;
    // 自转频率（度/日）
    DegreesPreDay angularFrequency = 0.0;
    // 北极点赤经（度）
    Degrees axisRightAscension = 0.0;
    // 北极点赤纬（度）
    Degrees axisDeclination = 0.0;

    void fixRotation() {
        referenceInstant = 0.0;
        referenceAngle = 0.0;
        angularFrequency = 0.0;
    }

    Degrees angleAtInstant(JulianDays instantQuery) const {
        JulianDays deltaTime = instantQuery - referenceInstant;
        return std::clamp(referenceAngle + angularFrequency * deltaTime, 0.0, 360.0);
    }

    void worldToLocal(Vector3<Kilometers> &position, JulianDays instant) const {
        {
            auto cosRA = std::cos(axisRightAscension * kDegrees);
            auto sinRA = -std::sin(axisRightAscension * kDegrees);
            auto newX = position.x * cosRA + position.y * sinRA;
            auto newY = position.y * cosRA - position.x * sinRA;
            position.x = newX;
            position.y = newY;
        }
        {
            auto cosDec = std::cos(axisDeclination * kDegrees);
            auto sinDec = -std::sin(axisDeclination * kDegrees);
            auto newZ = position.z * sinDec + position.y * cosDec;
            auto newY = position.y * sinDec - position.z * cosDec;
            position.z = newZ;
            position.y = newY;
        }
        {
            Degrees angle = angleAtInstant(instant);
            auto cosAngle = std::cos(angle * kDegrees);
            auto sinAngle = -std::sin(angle * kDegrees);
            auto newX = position.x * cosAngle + position.y * sinAngle;
            auto newY = position.y * cosAngle - position.x * sinAngle;
            position.x = newX;
            position.y = newY;
        }
    }

    void localToWorld(Vector3<Kilometers> &position, JulianDays instant) const {
        {
            Degrees angle = angleAtInstant(instant);
            auto cosAngle = std::cos(angle * kDegrees);
            auto sinAngle = std::sin(angle * kDegrees);
            auto newX = position.x * cosAngle + position.y * sinAngle;
            auto newY = position.y * cosAngle - position.x * sinAngle;
            position.x = newX;
            position.y = newY;
        }
        {
            auto cosDec = std::cos(axisDeclination * kDegrees);
            auto sinDec = std::sin(axisDeclination * kDegrees);
            auto newZ = position.z * sinDec + position.y * cosDec;
            auto newY = position.y * sinDec - position.z * cosDec;
            position.z = newZ;
            position.y = newY;
        }
        {
            auto cosRA = std::cos(axisRightAscension * kDegrees);
            auto sinRA = std::sin(axisRightAscension * kDegrees);
            auto newX = position.x * cosRA + position.y * sinRA;
            auto newY = position.y * cosRA - position.x * sinRA;
            position.x = newX;
            position.y = newY;
        }
    }

    void worldToLocalVelocity(Vector3<KilometersPerSecond> &velocity, JulianDays instant) const {
        worldToLocal(velocity, instant);
    }

    void localToWorldVelocity(Vector3<KilometersPerSecond> &velocity, JulianDays instant) const {
        localToWorld(velocity, instant);
    }
};

struct BodyTrajectory {
    std::vector<Vector3<Kilometers>> positionHistory;
    std::vector<Vector3<KilometersPerSecond>> velocityHistory;
    std::vector<JulianDays> historyInstants;

    size_t numHistoryCount() const {
        assert(positionHistory.size() == historyInstants.size());
        assert(velocityHistory.size() == historyInstants.size());
        return positionHistory.size();
    }

    BodyState stateAtInstant(JulianDays instant) const {
        Real index = binarySearch(historyInstants, instant);
        return {linearInterpolate(positionHistory, index), linearInterpolate(velocityHistory, index)};
    }
};

struct ReferenceFrame {
    // 旋转部分
    FrameRotation rotation;
    // 平移部分
    BodyTrajectory trajectory;

    void worldToLocal(Vector3<Kilometers> &position, JulianDays instant) const {
        auto bodyPosition = trajectory.stateAtInstant(instant).position;
        position.x -= bodyPosition.x;
        position.y -= bodyPosition.y;
        position.z -= bodyPosition.z;
        rotation.worldToLocal(position, instant);
    }

    void localToWorld(Vector3<Kilometers> &position, JulianDays instant) const {
        rotation.localToWorld(position, instant);
        auto bodyPosition = trajectory.stateAtInstant(instant).position;
        position.x += bodyPosition.x;
        position.y += bodyPosition.y;
        position.z += bodyPosition.z;
    }

    void worldToLocalVelocity(Vector3<KilometersPerSecond> &velocity, JulianDays instant) const {
        auto bodyVelocity = trajectory.stateAtInstant(instant).velocity;
        velocity.x -= bodyVelocity.x;
        velocity.y -= bodyVelocity.y;
        velocity.z -= bodyVelocity.z;
        rotation.worldToLocalVelocity(velocity, instant);
    }

    void localToWorldVelocity(Vector3<KilometersPerSecond> &velocity, JulianDays instant) const {
        rotation.localToWorldVelocity(velocity, instant);
        auto bodyVelocity = trajectory.stateAtInstant(instant).velocity;
        velocity.x += bodyVelocity.x;
        velocity.y += bodyVelocity.y;
        velocity.z += bodyVelocity.z;
    }

    void worldToLocal(BodyTrajectory &trajectory) const {
        for (size_t i = 0; i < trajectory.numHistoryCount(); i++) {
            worldToLocal(trajectory.positionHistory[i], trajectory.historyInstants[i]);
            worldToLocalVelocity(trajectory.velocityHistory[i], trajectory.historyInstants[i]);
        }
    }

    void localToWorld(BodyTrajectory &trajectory) const {
        for (size_t i = 0; i < trajectory.numHistoryCount(); i++) {
            localToWorld(trajectory.positionHistory[i], trajectory.historyInstants[i]);
            localToWorldVelocity(trajectory.velocityHistory[i], trajectory.historyInstants[i]);
        }
    }
};

struct BodyGravityModel {
    // 引力参数（GM）
    Kilometers3PerSeconds2 gravitationalParameter;
    // 参考半径（千米）
    Kilometers referenceRadius;
    // 引力摄动项
    std::vector<GeoPotentialComponent> geoPotentialComponents;
    // 自转参考系
    FrameRotation rotation;
    // 天体名称
    BodyName name;

    Vector3<KilometersPerSecond2> gravityAccelerationAtPosition(BodyState const &body, Vector3<Kilometers> const &positionQuery) const {
        Vector3<Kilometers> delta;
        delta.x = body.position.x - positionQuery.x;
        delta.y = body.position.y - positionQuery.y;
        delta.z = body.position.z - positionQuery.z;
        Real distanceSquared = delta.x * delta.x + delta.y * delta.y + delta.z * delta.z;
        Real referenceRadiusSquared = referenceRadius * referenceRadius;
        if (distanceSquared < referenceRadiusSquared) {
            distanceSquared = referenceRadiusSquared;
        }
        Real distance = std::sqrt(distanceSquared);
        Real distanceCubed = distanceSquared * distance;
        Real gravityScale = gravitationalParameter / distanceCubed;
        Vector3<KilometersPerSecond2> acceleration;
        acceleration.x = delta.x * gravityScale;
        acceleration.y = delta.y * gravityScale;
        acceleration.z = delta.z * gravityScale;
        return acceleration;
    }
};

struct SystemGravityModel {
    std::vector<BodyGravityModel> bodies;

    Optional<size_t> getBodyIndexByName(std::string_view nameQuery) const {
        size_t index = 0;
        for (auto const &body: bodies) {
            if (body.name == nameQuery) {
                return index;
            }
            index++;
        }
        return std::nullopt;
    }

    void initializeFromConfig(ConfigParser::Variant const &rootV) {
        auto modelV = rootV.dictEntry("principia_gravity_model:NEEDS[RealSolarSystem]");
        modelV.dictEntryForEach("body", [this] (auto, auto const &bodyV) {
            auto &bodyM = bodies.emplace_back();
            bodyM.name = bodyV.dictEntrySafe("name") & []F_LR(->getAtom()) | []F_L0("???");
            bodyM.gravitationalParameter = bodyV.dictEntrySafe("gravitational_parameter") & []F_LR(->getDouble()) | []F_L0(0.0);
            bodyM.rotation.referenceInstant = bodyV.dictEntrySafe("reference_instant") & []F_LR(->getDouble()) | []F_L0(0.0);
            bodyM.rotation.axisRightAscension = bodyV.dictEntrySafe("axis_right_ascension") & []F_LR(->getDouble()) | []F_L0(0.0);
            bodyM.rotation.axisDeclination = bodyV.dictEntrySafe("axis_declination") & []F_LR(->getDouble()) | []F_L0(0.0);
            bodyM.rotation.referenceAngle = bodyV.dictEntrySafe("reference_angle") & []F_LR(->getDouble()) | []F_L0(0.0);
            bodyM.rotation.angularFrequency = bodyV.dictEntrySafe("angular_frequency") & []F_LR(->getDouble()) | []F_L0(0.0);
            bodyM.referenceRadius = bodyV.dictEntrySafe("reference_radius") & []F_LR(->getDouble()) | []F_L0(1.0);
            bodyV.dictEntrySafe("j2") & []F_LR(->getDouble()) & [&bodyM]FF_L1(j2) {
                auto &componentM = bodyM.geoPotentialComponents.emplace_back();
                componentM.degree = 2;
                componentM.order = 0;
                componentM.cosValue = j2;
                componentM.sinValue = 0.0;
            };
            bodyV.dictEntryForEach("geopotential_row", [&bodyM] (auto, auto const &rowV) {
                int degree = rowV.dictEntry("degree").getInt();
                rowV.dictEntryForEach("geopotential_column", [&bodyM, degree] (auto, auto const &componentV) {
                    auto &componentM = bodyM.geoPotentialComponents.emplace_back();
                    componentM.degree = degree;
                    componentM.order = componentV.dictEntry("order").getInt();
                    componentM.cosValue = componentV.dictEntrySafe("cos") & []F_LR(->getDouble())
                        | [&componentV]F_L0(componentV.dictEntry("j").getDouble());
                    componentM.sinValue = componentV.dictEntry("sin").getDouble();
                });
            });
        });
    }
};

struct SystemState {
    std::vector<BodyState> bodies;
    JulianDays instant;

    void initializeFromConfig(ConfigParser::Variant const &rootV, SystemGravityModel const &gravityModel) {
        auto stateV = rootV.dictEntry("principia_initial_state:NEEDS[RealSolarSystem]");
        instant = stateV.dictEntry("solar_system_epoch").getDouble();
        bodies.resize(stateV.dictEntryCount("body"));
        stateV.dictEntryForEach("body", [this, &gravityModel] (auto, auto const &bodyV) {
            auto name = bodyV.dictEntry("name").getAtom();
            auto index = gravityModel.getBodyIndexByName(name) | []F_DIE(0);
            auto &bodyM = bodies[index];
            bodyM.position.x = bodyV.dictEntry("x").getDouble();
            bodyM.position.y = bodyV.dictEntry("y").getDouble();
            bodyM.position.z = bodyV.dictEntry("z").getDouble();
            bodyM.velocity.x = bodyV.dictEntry("vx").getDouble();
            bodyM.velocity.y = bodyV.dictEntry("vy").getDouble();
            bodyM.velocity.z = bodyV.dictEntry("vz").getDouble();
        });
    }
};

struct SolarSystem {
    SystemGravityModel gravityModel;
    SystemState currentState, nextState;
    std::vector<Vector3<KilometersPerSecond2>> bodyAccelerations;
    std::vector<BodyTrajectory> bodyTrajectories;

    void takeSnapshot() {
        for (size_t i = 0; i < bodyTrajectories.size(); i++) {
            auto const &body = currentState.bodies[i];
            bodyTrajectories[i].positionHistory.push_back(body.position);
            bodyTrajectories[i].velocityHistory.push_back(body.velocity);
            bodyTrajectories[i].historyInstants.push_back(currentState.instant);
        }
    }

    BodyTrajectory getBodyTrajectory(size_t bodyIndex) const {
        return bodyTrajectories[bodyIndex];
    }

    void initializeFromConfig(ConfigParser::Variant const &rootV) {
        gravityModel.initializeFromConfig(rootV);
        currentState.initializeFromConfig(rootV, gravityModel);
        nextState.bodies.resize(currentState.bodies.size());
        bodyAccelerations.resize(currentState.bodies.size());
        bodyTrajectories.resize(currentState.bodies.size());
    }

    size_t getBodyIndexByName(std::string_view nameQuery) const {
        return gravityModel.getBodyIndexByName(nameQuery) | []F_L0((size_t)-1);
    }

    ReferenceFrame getBodyRotationalReferenceFrame(size_t bodyIndex) {
        auto const &model = gravityModel.bodies[bodyIndex];
        auto const &trajectory = bodyTrajectories[bodyIndex];
        return {model.rotation, trajectory};
    }

    ReferenceFrame getBodyFixedReferenceFrame(size_t bodyIndex) {
        auto frame = getBodyRotationalReferenceFrame(bodyIndex);
        frame.rotation.fixRotation();
        return frame;
    }

    size_t numBodies() const {
        assert(currentState.bodies.size() == gravityModel.bodies.size());
        return currentState.bodies.size();
    }

    Vector3<KilometersPerSecond2> gravityAccelerationAtPosition(Vector3<Kilometers> positionQuery) const {
        Vector3<KilometersPerSecond2> acceleration = {0, 0, 0};
        for (size_t i = 0; i < numBodies(); i++) {
            auto const &bodyI = currentState.bodies[i];
            auto const &modelI = gravityModel.bodies[i];
            auto accelI = modelI.gravityAccelerationAtPosition(bodyI, positionQuery);
            acceleration.x += accelI.x;
            acceleration.y += accelI.y;
            acceleration.z += accelI.z;
        }
        return acceleration;
    }

    Vector3<KilometersPerSecond2> gravityAccelerationAtPosition(Vector3<Kilometers> positionQuery, size_t excludeIndex) const {
        Vector3<KilometersPerSecond2> acceleration = {0, 0, 0};
        for (size_t i = 0; i < numBodies(); i++) {
            if (i == excludeIndex) continue;
            auto const &bodyI = currentState.bodies[i];
            auto const &modelI = gravityModel.bodies[i];
            auto accelI = modelI.gravityAccelerationAtPosition(bodyI, positionQuery);
            acceleration.x += accelI.x;
            acceleration.y += accelI.y;
            acceleration.z += accelI.z;
        }
        return acceleration;
    }

    void computeNextStateRK1(Seconds dt) {
        nextState.instant = currentState.instant + dt / 86400.0;
#pragma omp parallel for
        for (size_t i = 0; i < numBodies(); i++) {
            auto const &bodyI = currentState.bodies[i];
            auto &nextBodyI = nextState.bodies[i];
            auto accelI = gravityAccelerationAtPosition(bodyI.position, i);
            nextBodyI.velocity = bodyI.velocity + accelI * dt;
            nextBodyI.position = bodyI.position + (nextBodyI.velocity + bodyI.velocity) * 0.5 * dt;
        }
    }

    void computeNextStateRK2(Seconds dt) {
        nextState.instant = currentState.instant + dt / 86400.0;
        // p_mid = p - 0.5 * dt * velocity(p)
        // p -= dt * velocity(p_mid)
#pragma omp parallel for
        for (size_t i = 0; i < numBodies(); i++) {
            auto const &bodyI = currentState.bodies[i];
            auto &nextBodyI = nextState.bodies[i];
            auto p0 = bodyI.position;
            auto v0 = bodyI.velocity;
            auto a0 = gravityAccelerationAtPosition(p0, i);
            auto p1 = p0 + 0.5 * dt * (v0 + 0.5 * dt * a0);
            auto v1 = v0 + 0.5 * dt * a0;
            auto a1 = gravityAccelerationAtPosition(p1, i);
            nextBodyI.position = p0 + dt * v1;
            nextBodyI.velocity = v0 + dt * a1;
        }
    }

    void computeNextStateRK3(Seconds dt) {
        nextState.instant = currentState.instant + dt / 86400.0;
        //v1 = velocity(p)
        //p1 = p - 0.5 * dt * v1
        //v2 = velocity(p1)
        //p2 = p - 0.75 * dt * v2
        //v3 = velocity(p2)
        //p -= dt * (2 / 9 * v1 + 1 / 3 * v2 + 4 / 9 * v3)
#pragma omp parallel for
        for (size_t i = 0; i < numBodies(); i++) {
            auto const &bodyI = currentState.bodies[i];
            auto &nextBodyI = nextState.bodies[i];
            auto p0 = bodyI.position;
            auto v0 = bodyI.velocity;
            auto a0 = gravityAccelerationAtPosition(p0, i);
            auto p1 = p0 + 0.5 * dt * (v0 + 0.5 * dt * a0);
            auto v1 = v0 + 0.5 * dt * a0;
            auto a1 = gravityAccelerationAtPosition(p1, i);
            auto p2 = p0 + 0.75 * dt * (v1 + 0.75 * dt * a1);
            auto v2 = v0 + 0.75 * dt * a1;
            auto a2 = gravityAccelerationAtPosition(p2, i);
            nextBodyI.position = p0 + dt * (2.0 / 9.0 * v0 + 1.0 / 3.0 * v1 + 4.0 / 9.0 * v2);
            nextBodyI.velocity = v0 + dt * (2.0 / 9.0 * a0 + 1.0 / 3.0 * a1 + 4.0 / 9.0 * a2);
        }
    }

    void evalGravityAccelerations() {
#pragma omp parallel for
        for (size_t i = 0; i < numBodies(); i++) {
            auto const &bodyI = currentState.bodies[i];
            bodyAccelerations[i] = gravityAccelerationAtPosition(bodyI.position, i);
        }
    }

    void computeNextStateRK4(Seconds dt) {
        nextState.instant = currentState.instant + dt / 86400.0;
        evalGravityAccelerations();
        // y' = f(y)   y(t0) = y0
        // g(r) = r / r^3
        // G(p) = \sum_j m_j g(p_j - p)
        // p_i' = v_i
        // v_i' = G(p_i)
        // y = (p, v)
        // y' = (v, G(p))
        // k1 = f(y) = (v, G(p))
        // y + h/2 k1 = (p + h/2 v, v + h/2 G(p))
        // k2 = f(y + h/2 k1) = (v + h/2 G(p), G(p + h/2 v))
        // y + h/2 k2 = (p + h/2 (v + h/2 G(p)), v + h/2 G(p + h/2 v))
        // k3 = f(y + h/2 k2) = (v + h/2 G(p + h/2 v), G(p + h/2 (v + h/2 G(p))))
        // y + h k3 = (p + h (v + h/2 G(p + h/2 v)), v + h G(p + h/2 (v + h/2 G(p))))
        // k4 = f(y + h k3) = (v + h G(p + h/2 (v + h/2 G(p))), G(p + h (v + h/2 G(p + h/2 v))))
        // k = (\delta_p, \delta_v) = (k1 + 2 k2 + 2 k3 + k4) / 6
        // \delta_p * 6 = v + 2 v + h G(p) + 2 v + h G(p + h/2 v) + v + h G(p + h/2 (v + h/2 G(p)))
        // \delta_v * 6 = G(p) + 2 G(p + h/2 v) + 2 G(p + h/2 (v + h/2 G(p))) + G(p + h (v + h/2 G(p + h/2 v)))
        // \delta_p = v + h/6 G(p) + h/6 G(p + h/2 v) + h/6 G(p + h/2 (v + h/2 G(p)))
        // \delta_v = 1/6 G(p) + 1/3 G(p + h/2 v) + 1/3 G(p + h/2 (v + h/2 G(p))) + 1/6 G(p + h (v + h/2 G(p + h/2 v)))
    }

    void computeNextState(Seconds dt) {
        computeNextStateRK3(dt);
    }

    void swapStates() {
        std::swap(nextState, currentState);
    }
};
