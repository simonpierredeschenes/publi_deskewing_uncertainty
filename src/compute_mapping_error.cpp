#include <pointmatcher/PointMatcher.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>

typedef PointMatcher<float> PM;

// taken from here: https://stackoverflow.com/questions/62696455/is-there-a-way-to-find-the-median-value-of-coefficients-of-an-eigen-matrix
float computeMedian(PM::Matrix errors)
{
    std::sort(errors.row(0).begin(), errors.row(0).end());
    return errors.row(0).size() % 2 == 0 ? errors.row(0).segment((errors.row(0).size() - 2) / 2, 2).mean() : errors.row(0)(errors.row(0).size() / 2);
}

int main(int argc, char** argv)
{
    if(argc < 4 || argc > 7)
    {
        throw std::runtime_error(
                "Incorrect number of arguments. Argument 1 is the path to the map to evaluate. Argument 2 is the path to the ground truth map. Argument 3 is the ball radius for the nearest neighbor search. Argument 4 (default=0) is the prior yaw angle in degrees. Argument 5 (default=0) is the prior x translation. Argument 6 (default=0) is the prior y translation.");
    }
    PM::DataPoints map(PM::DataPoints::load(argv[1]));
    PM::DataPoints groundTruth(PM::DataPoints::load(argv[2]));
    float threshold = std::stof(argv[3]);

    float priorYaw = 0;
    float priorX = 0;
    float priorY = 0;
    if(argc > 4)
    {
        priorYaw = std::stof(argv[4]) * M_PI / 180.f;
    }
    if(argc > 5)
    {
        priorX = std::stof(argv[5]);
    }
    if(argc > 6)
    {
        priorY = std::stof(argv[6]);
    }
    PM::TransformationParameters prior = PM::Matrix::Identity(4, 4);
    prior(0, 0) = prior(1, 1) = std::cos(priorYaw);
    prior(0, 1) = -std::sin(priorYaw);
    prior(1, 0) = std::sin(priorYaw);
    prior(0, 3) = priorX;
    prior(1, 3) = priorY;

    PM::ICP icp;
    std::ifstream icpConfig;
    icpConfig.open("/home/sp/ros2_ws/src/publi_deskewing_uncertainty/params/mapping_error_icp_config.yaml");
    icp.loadFromYaml(icpConfig);
    icpConfig.close();
    icp.transformations.apply(map, prior);
    size_t fileExtensionPosition = std::string(argv[1]).rfind('.');
    std::string fileName = std::string(argv[1]).substr(0, fileExtensionPosition);
    std::string fileExtension = std::string(argv[1]).substr(fileExtensionPosition);
    std::string mapWithPriorFileName = fileName + "_prior" + fileExtension;
    map.save(mapWithPriorFileName);
    PM::TransformationParameters correction = icp(map, groundTruth);
    icp.transformations.apply(map, correction);

    PM::Parameters params;
    params["knn"] = "1";
    params["epsilon"] = "0";
    params["maxDist"] = "inf";
    std::shared_ptr<PM::Matcher> matcher = PM::get().MatcherRegistrar.create("KDTreeMatcher", params);

    matcher->init(groundTruth);
    PM::Matches matches = matcher->findClosests(map);
    PM::Matrix errors = matches.dists.array().sqrt();
    PM::Matrix belongingToMap = (errors.array() <= threshold).cast<float>();
    map.addDescriptor("error", errors);
    map.addDescriptor("belonging_to_map", belongingToMap);
    std::string registeredMapFileName = fileName + "_with_error" + fileExtension;
    map.save(registeredMapFileName);

    int nbPointsTotal = matches.ids.cols();
    int nbPointsUnderThreshold = 0;
    std::string errorFileName = fileName + "_errors.csv";
    std::ofstream errorFile(errorFileName);
    errorFile << "error" << std::endl;
    for(unsigned int i = 0; i < nbPointsTotal; ++i)
    {
        errorFile << errors(0, i) << std::endl;
        if(errors(0, i) <= threshold)
        {
            ++nbPointsUnderThreshold;
        }
    }
    errorFile.close();
    std::cout << "The mean mapping error is " << std::setprecision(2) << std::fixed << 100.f * errors.mean() << " cm." << std::endl;
    std::cout << "The median mapping error is " << std::setprecision(2) << std::fixed << 100.f * computeMedian(errors) << " cm." << std::endl;
    std::cout << "The overlap ratio is " << std::setprecision(2) << std::fixed << 100.f * (float(nbPointsUnderThreshold) / float(nbPointsTotal)) << " %." << std::endl;

    return 0;
}
