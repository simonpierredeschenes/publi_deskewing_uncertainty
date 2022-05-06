#include <pointmatcher/PointMatcher.h>
#include <iostream>
#include <iomanip>

typedef PointMatcher<float> PM;

int main(int argc, char** argv)
{
	if(argc != 4)
	{
		throw std::runtime_error("Incorrect number of arguments. Argument 1 is the path to the map to evaluate. Argument 2 is the path to the ground truth map. Argument 3 is the ball radius for the nearest neighbor search.");
	}

	PM::DataPoints map(PM::DataPoints::load(argv[1]));
	PM::DataPoints groundTruth(PM::DataPoints::load(argv[2]));
	float maxDist = std::stof(argv[3]);

	PM::ICP icp;
	icp.setDefault();
	PM::TransformationParameters correction = icp(map, groundTruth);
	icp.transformations.apply(map, correction);
	size_t fileExtensionPosition = std::string(argv[1]).rfind('.');
	std::string fileName = std::string(argv[1]).substr(0, fileExtensionPosition);
	std::string fileExtension = std::string(argv[1]).substr(fileExtensionPosition);
	std::string registeredMapFileName = fileName + "_registered_in_groundtruth" + fileExtension;
	map.save(registeredMapFileName);

	PM::Parameters params;
	params["knn"] = "1";
	params["epsilon"] = "0";
	params["maxDist"] = std::to_string(maxDist);
	std::shared_ptr<PM::Matcher> matcher = PM::get().MatcherRegistrar.create("KDTreeMatcher", params);

	matcher->init(groundTruth);
	PM::Matches matches = matcher->findClosests(map);

	int nbPointsOk = 0;
	for(unsigned int i = 0; i < matches.ids.cols(); ++i)
	{
		if(matches.ids(0, i) != PM::Matches::InvalidId)
		{
			++nbPointsOk;
		}
	}
	std::cout << "The overlap ratio is " << std::setprecision(2) << std::fixed << 100.f * (float(nbPointsOk) / float(matches.ids.cols())) << " %." << std::endl;

	return 0;
}
