#pragma once

#include "simpleBA.h"

#include <nlohmann/json.hpp>

namespace SimpleBA
{
    void to_json(nlohmann::json& j, const SimpleBA::Camera& camera) 
    {
        j = nlohmann::json{
                {"id", camera.id},
                {"intrinsicsId", camera.intrinsicsId},
                {"camFromWorldQ", camera.camFromWorldQ},
                {"camFromWorldT", camera.camFromWorldT},
                {"isFixed", camera.isFixed}
            };
    };

    void from_json(const nlohmann::json& j, SimpleBA::Camera& camera) {
        j.at("id").get_to(camera.id);
        j.at("intrinsicsId").get_to(camera.intrinsicsId);
        j.at("camFromWorldQ").get_to(camera.camFromWorldQ);
        j.at("camFromWorldT").get_to(camera.camFromWorldT);
        j.at("isFixed").get_to(camera.isFixed);
    };

    void to_json(nlohmann::json& j, const SimpleBA::Intrinsics& intrinsics) 
    {
        j = nlohmann::json{
                {"id", intrinsics.id},
                {"cameraParameters", intrinsics.cameraParameters},
                {"isFixed", intrinsics.isFixed}
            };
    };

    void from_json(const nlohmann::json& j, SimpleBA::Intrinsics& intrinsics) {
        j.at("id").get_to(intrinsics.id);
        j.at("cameraParameters").get_to(intrinsics.cameraParameters);
        j.at("isFixed").get_to(intrinsics.isFixed);
    };

    void to_json(nlohmann::json& j, const SimpleBA::Observation& observation) 
    {
        j = nlohmann::json{
                {"id", observation.id},
                {"cameraId", observation.cameraId},
                {"pointId", observation.pointId},
                {"x", observation.x},
                {"y", observation.y}
            };
    };

    void from_json(const nlohmann::json& j, SimpleBA::Observation& observation) {
        j.at("id").get_to(observation.id);
        j.at("cameraId").get_to(observation.cameraId);
        j.at("pointId").get_to(observation.pointId);
        j.at("x").get_to(observation.x);
        j.at("y").get_to(observation.y);
    };

    void to_json(nlohmann::json& j, const SimpleBA::Point& point) 
    {
        j = nlohmann::json{
                {"id", point.id},
                {"point3d", point.point3d},
                {"isFixed", point.isFixed},
            };
    };

    void from_json(const nlohmann::json& j, SimpleBA::Point& point) {
        j.at("id").get_to(point.id);
        j.at("point3d").get_to(point.point3d);
        j.at("isFixed").get_to(point.isFixed);
    };

    void to_json(nlohmann::json& j, const SimpleBA::SyntheticSceneGenerator& scene)
    {
        j = nlohmann::json{
                {"intrinsics_map", scene.intrinsics_map},
                {"cameras_map", scene.cameras_map},
                {"points_map", scene.points_map},
                {"observations_map", scene.observations_map}
        };
    };

    void from_json(const nlohmann::json& j, SimpleBA::SyntheticSceneGenerator& scene) {
        j.at("intrinsics_map").get_to(scene.intrinsics_map);
        j.at("cameras_map").get_to(scene.cameras_map);
        j.at("points_map").get_to(scene.points_map);
        j.at("observations_map").get_to(scene.observations_map);
    };

}