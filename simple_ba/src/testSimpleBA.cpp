#include "simpleBA.h"
#include "simpleBASerialization.h"
#include <nlohmann/json.hpp>

#include <fstream>
#include <iostream>

using json = nlohmann::json;

int main(int argc, char** argv)
{
    std::string filepathIn = "C:/Users/dapot/code/simple_bundle_adjuster/simple_ba/src/test_data.json";
    std::string filepathOut = "C:/Users/dapot/code/simple_bundle_adjuster/simple_ba/src/test_data_out.json";

    if (argc > 2)
    {
        filepathIn = argv[1];
        filepathOut = argv[2];
    }

    std::ifstream f(filepathIn);

    json data = json::parse(f);
    SimpleBA::SyntheticSceneGenerator scene = data.template get<SimpleBA::SyntheticSceneGenerator>();

    SimpleBA::BundleAdjuster testBundleAdjuster;
    for (auto& id_point_pair : scene.points_map)
    {
        testBundleAdjuster.addPoint(id_point_pair.second);
    }

    for (auto& id_intrinsics_pair : scene.intrinsics_map)
    {
        testBundleAdjuster.addCameraIntrinsics(id_intrinsics_pair.second);
    }

    for (auto& id_camera_pair : scene.cameras_map)
    {
        testBundleAdjuster.addCamera(id_camera_pair.second);
    }

    for (auto& id_observation_pair : scene.observations_map)
    {
        testBundleAdjuster.addObservation(id_observation_pair.second);
    }

    std::cout << "running ba" << std::endl;
    bool result = testBundleAdjuster.run();
    std::cout << "result: " << result << std::endl;

    scene.cameras_map = testBundleAdjuster.cameras_map;
    scene.intrinsics_map = testBundleAdjuster.intrinsics_map;
    scene.observations_map = testBundleAdjuster.observations_map;
    scene.points_map = testBundleAdjuster.points_map;

    data = scene;

    std::ofstream o(filepathOut);
    o << std::setw(4) << data << std::endl;

    //std::cout << data << std::endl;
    //SimpleBA::SyntheticSceneGenerator sampleScene;
    //SimpleBA::Point point0(
    //    -0.7615716703048154,
    //    -0.35831161439408554,
    //    0.531807717972557,
    //    "0",
    //    false);
    //SimpleBA::Point point1(
    //    0.030494563339805332,
    //    0.8207993396837433,
    //    -0.564641810301221,
    //    "1",
    //    false);

    //SimpleBA::Intrinsics intrinsics0;
    //intrinsics0.id = "0";
    //intrinsics0.cameraParameters = { 1215.2946956483515, 1139.3387771703296, 800.0, 600.0 };
    //intrinsics0.isFixed = false;

    //SimpleBA::Camera camera0;
    //camera0.id = "0";
    //camera0.intrinsicsId = "0";
    //camera0.isFixed = false;
    //camera0.camFromWorldQ = { 0.854949917929487,
    //            0.08786918080315172,
    //            0.48742773147863383,
    //            0.15412284542837676 };
    //camera0.camFromWorldT = { 3.404500658784671e-17,
    //            -2.5705806900632676e-17,
    //            2.988181688867581 };
    //SimpleBA::Camera camera1;
    //camera1.id = "1";
    //camera1.intrinsicsId = "0";
    //camera1.isFixed = false;
    //camera1.camFromWorldQ = { -0.0015422136263664686,
    //            -0.49749054498137896,
    //            0.8674675768979869,
    //            0.0008844557628336459 };
    //camera1.camFromWorldT = { 1.271790386092001e-19,
    //            3.434628326701288e-16,
    //            2.9910184059570684 };

    //SimpleBA::Observation observation00(842.4606970586592, 607.0311254810002, "0", "0", "0");
    //SimpleBA::Observation observation01(639.8308178828597, 389.4509826555294, "1", "0", "1");
    //SimpleBA::Observation observation10(1058.5923549867068, 510.5186712567527, "2", "1", "0");
    //SimpleBA::Observation observation11(782.6673685545591, 641.603273550856, "3", "1", "1");

    //sampleScene.cameras_map["0"] = camera0;
    //sampleScene.cameras_map["1"] = camera1;

    //sampleScene.intrinsics_map["0"] = intrinsics0;

    //sampleScene.observations_map["0"] = observation00;
    //sampleScene.observations_map["1"] = observation01;
    //sampleScene.observations_map["2"] = observation10;
    //sampleScene.observations_map["3"] = observation11;

    //sampleScene.points_map["0"] = point0;
    //sampleScene.points_map["1"] = point1;

    //json j = sampleScene;
    //std::cout << std::setw(4) << j << std::endl;

}