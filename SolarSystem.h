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

    void toInertial() {
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

    void resampleDensity(JulianDays newSampleDensity) {
        if (newSampleDensity <= 0 || historyInstants.empty()) return;
        auto minInstant = historyInstants.front();
        auto maxInstant = historyInstants.back();
        if (minInstant == maxInstant) return;
        size_t newSize = (size_t)std::ceil((maxInstant - minInstant) / newSampleDensity);
        std::vector<JulianDays> newHistoryInstants(newSize);
        std::vector<Vector3<Kilometers>> newPositionHistory(newSize);
        std::vector<Vector3<KilometersPerSecond>> newVelocityHistory(newSize);
        JulianDays newInstant = minInstant;
        for (size_t i = 0; i < newSize; i++) {
            Real index = binarySearch(historyInstants, newInstant);
            newHistoryInstants[i] = linearInterpolate(historyInstants, index);
            newPositionHistory[i] = linearInterpolate(positionHistory, index);
            newVelocityHistory[i] = linearInterpolate(velocityHistory, index);
            newInstant = std::min(maxInstant, newInstant + newSampleDensity);
        }
        historyInstants = std::move(newHistoryInstants);
        positionHistory = std::move(newPositionHistory);
        velocityHistory = std::move(newVelocityHistory);
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

    Vector3<KilometersPerSecond2> gravityAccelerationAtPosition(Vector3<Kilometers> const &bodyPosition, Vector3<Kilometers> const &positionQuery) const {
        Vector3<Kilometers> delta;
        delta.x = bodyPosition.x - positionQuery.x;
        delta.y = bodyPosition.y - positionQuery.y;
        delta.z = bodyPosition.z - positionQuery.z;
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

struct SystemGravityModel; 

struct SystemState {
    std::vector<Vector3<Kilometers>> positions;
    std::vector<Vector3<KilometersPerSecond>> velocities;

    size_t numBodies() const {
        assert(positions.size() == velocities.size());
        return positions.size();
    }
};

struct SystemGravityModel {
    std::vector<BodyGravityModel> bodyModels;

    Optional<size_t> getBodyIndexByName(std::string_view nameQuery) const {
        size_t index = 0;
        for (auto const &body: bodyModels) {
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
            auto &bodyM = bodyModels.emplace_back();
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

    Vector3<KilometersPerSecond2> gravityAccelerationAtPosition(Vector3<Kilometers> positionQuery,
                                                                std::vector<Vector3<Kilometers>> const &bodyPositions) const {
        Vector3<KilometersPerSecond2> acceleration = {0, 0, 0};
        for (size_t i = 0; i < bodyPositions.size(); i++) {
            auto accelI = bodyModels[i].gravityAccelerationAtPosition(bodyPositions[i], positionQuery);
            acceleration.x += accelI.x;
            acceleration.y += accelI.y;
            acceleration.z += accelI.z;
        }
        return acceleration;
    }

    Vector3<KilometersPerSecond2> gravityAccelerationAtBody(size_t indexQuery,
                                                            std::vector<Vector3<Kilometers>> const &bodyPositions) const {
        Vector3<KilometersPerSecond2> acceleration = {0, 0, 0};
        for (size_t i = 0; i < bodyPositions.size(); i++) {
            if (i == indexQuery) continue;
            auto accelI = bodyModels[i].gravityAccelerationAtPosition(bodyPositions[i], bodyPositions[indexQuery]);
            acceleration.x += accelI.x;
            acceleration.y += accelI.y;
            acceleration.z += accelI.z;
        }
        return acceleration;
    }

    void evaluateGravityAccelerations(std::vector<Vector3<KilometersPerSecond2>> const &positions,
                                      std::vector<Vector3<KilometersPerSecond2>> &accelerations) const {
#pragma omp for simd
        for (size_t i = 0; i < positions.size(); i++) {
            accelerations[i] = gravityAccelerationAtBody(i, positions);
        }
    }
};

struct RungeKuttaSolver {
    std::vector<Vector3<Kilometers>> pos2;
    std::vector<Vector3<Kilometers>> pos3;
    std::vector<Vector3<Kilometers>> pos4;
    std::vector<Vector3<KilometersPerSecond>> vel2;
    std::vector<Vector3<KilometersPerSecond>> vel3;
    std::vector<Vector3<KilometersPerSecond2>> acc1;
    std::vector<Vector3<KilometersPerSecond2>> acc2;
    std::vector<Vector3<KilometersPerSecond2>> acc3;
    std::vector<Vector3<KilometersPerSecond2>> acc4;

    void setNumBodies(size_t n) {
        pos2.resize(n);
        pos3.resize(n);
        pos4.resize(n);
        vel2.resize(n);
        vel3.resize(n);
        acc1.resize(n);
        acc2.resize(n);
        acc3.resize(n);
        acc4.resize(n);
    }

    // https://zh.wikipedia.org/wiki/%E9%BE%99%E6%A0%BC%EF%BC%8D%E5%BA%93%E5%A1%94%E6%B3%95
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

    void evolveForTime(SystemGravityModel const &model, SystemState &state, Seconds dt) {
        auto const &pos1 = state.positions; // p
        model.evaluateGravityAccelerations(pos1, acc1); // G(p)
        auto const &vel1 = state.velocities; // v
        freeEvolveForTime(pos2, pos1, vel1, dt / 2.0); // p + h/2 v
        model.evaluateGravityAccelerations(pos2, acc2); // G(p + h/2 v)
        freeEvolveForTime(vel2, vel1, acc1, dt / 2.0); // v + h/2 G(p)
        freeEvolveForTime(pos3, pos1, vel2, dt / 2.0); // p + h/2 (v + h/2 G(p))
        model.evaluateGravityAccelerations(pos3, acc3); // G(p + h/2 (v + h/2 G(p)))
        freeEvolveForTime(vel3, vel1, acc2, dt / 2.0); // v + h/2 G(p + h/2 v)
        freeEvolveForTime(pos4, pos1, vel2, dt); // p + h (v + h/2 G(p + h/2 v))
        model.evaluateGravityAccelerations(pos4, acc4); // G(p + h (v + h/2 G(p + h/2 v)))
        correctedEvolvePosition(state.positions, state.velocities, dt);
        correctedEvolveVelocity(state.velocities, dt);
    }

    void correctedEvolvePosition(std::vector<Vector3<Kilometers>> &positions,
                                 std::vector<Vector3<KilometersPerSecond>> &velocities, Real dt) const {
#pragma omp for simd
        for (size_t i = 0; i < positions.size(); i++) {
            Vector3<Real> correction = (1.0 / 6.0) * (acc1[i] + acc2[i] + acc3[i]);
            positions[i] = positions[i] + (velocities[i] + correction * dt) * dt;
        }
    }

    void correctedEvolveVelocity(std::vector<Vector3<KilometersPerSecond>> &velocities, Real dt) const {
#pragma omp for simd
        for (size_t i = 0; i < velocities.size(); i++) {
            Vector3<Real> acceleration = (1.0 / 6.0) * (acc1[i] + 2.0 * acc2[i] + 2.0 * acc3[i] + acc4[i]);
            velocities[i] = velocities[i] + acceleration * dt;
        }
    }

    static void freeEvolveForTime(std::vector<Vector3<Real>> &newPositions,
                                  std::vector<Vector3<Real>> const &positions,
                                  std::vector<Vector3<Real>> const &velocities,
                                  Seconds dt) {
#pragma omp for simd
        for (size_t i = 0; i < positions.size(); i++) {
            newPositions[i] = positions[i] + velocities[i] * dt;
        }
    }
};

struct SolarSystem {
    SystemGravityModel gravityModel;
    SystemState currentState;
    std::vector<BodyTrajectory> bodyTrajectories;
    JulianDays currentInstant;
    RungeKuttaSolver rungeKuttaSolver;

    void takeSnapshot() {
        for (size_t i = 0; i < bodyTrajectories.size(); i++) {
            bodyTrajectories[i].positionHistory.push_back(currentState.positions[i]);
            bodyTrajectories[i].velocityHistory.push_back(currentState.velocities[i]);
            bodyTrajectories[i].historyInstants.push_back(currentInstant);
        }
    }

    BodyTrajectory getBodyTrajectory(size_t bodyIndex) const {
        return bodyTrajectories[bodyIndex];
    }

    void initializeFromConfig(ConfigParser::Variant const &rootV) {
        gravityModel.initializeFromConfig(rootV);
        {
            auto stateV = rootV.dictEntry("principia_initial_state:NEEDS[RealSolarSystem]");
            currentInstant = stateV.dictEntry("solar_system_epoch").getDouble();
            size_t bodyCount = stateV.dictEntryCount("body");
            currentState.positions.resize(bodyCount);
            currentState.velocities.resize(bodyCount);
            stateV.dictEntryForEach("body", [this] (auto, auto const &bodyV) {
                auto name = bodyV.dictEntry("name").getAtom();
                auto index = gravityModel.getBodyIndexByName(name) | []F_DIE(0);
                auto &positionM = currentState.positions[index];
                auto &velocityM = currentState.velocities[index];
                positionM.x = bodyV.dictEntry("x").getDouble();
                positionM.y = bodyV.dictEntry("y").getDouble();
                positionM.z = bodyV.dictEntry("z").getDouble();
                velocityM.x = bodyV.dictEntry("vx").getDouble();
                velocityM.y = bodyV.dictEntry("vy").getDouble();
                velocityM.z = bodyV.dictEntry("vz").getDouble();
            });
        }
        assert(currentState.numBodies() == gravityModel.bodyModels.size());
        rungeKuttaSolver.setNumBodies(currentState.numBodies());
        bodyTrajectories.resize(currentState.numBodies());
    }

    size_t getBodyIndexByName(std::string_view nameQuery) const {
        return gravityModel.getBodyIndexByName(nameQuery) | []F_L0((size_t)-1);
    }

    ReferenceFrame getBodyFixedReferenceFrame(size_t bodyIndex) {
        auto const &model = gravityModel.bodyModels[bodyIndex];
        auto const &trajectory = bodyTrajectories[bodyIndex];
        return {model.rotation, trajectory};
    }

    ReferenceFrame getBodyInertialReferenceFrame(size_t bodyIndex) {
        auto frame = getBodyFixedReferenceFrame(bodyIndex);
        frame.rotation.toInertial();
        return frame;
    }

    size_t numBodies() const {
        return currentState.numBodies();
    }

    Vector3<KilometersPerSecond2> gravityAccelerationAtPosition(Vector3<Kilometers> positionQuery) const {
        return gravityModel.gravityAccelerationAtPosition(positionQuery, currentState.positions);
    }

    Vector3<KilometersPerSecond2> gravityAccelerationAtBody(size_t indexQuery) const {
        return gravityModel.gravityAccelerationAtBody(indexQuery, currentState.positions);
    }

    void evolveForTime(Seconds dt) {
        rungeKuttaSolver.evolveForTime(gravityModel, currentState, dt);
        currentInstant = currentInstant + dt / 86400.0;
    }

    void evolveForTime(Seconds dt, size_t numSubSteps) {
        for (size_t i = 0; i < numSubSteps; i++) {
            evolveForTime(dt);
        }
    }
};
