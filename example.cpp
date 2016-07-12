


#include <vector>
#include "svData.h"
#include <random>
#include "svVoronoiCore.h"
#include <iostream>
#include <chrono>

std::vector<sv::Real3> generatePoints(size_t count) {
	std::vector<sv::Real3> points;

	std::random_device rd;
	std::mt19937 rng(rd());

	std::uniform_real_distribution<> heightDistribution(-1, 1);
	std::uniform_real_distribution<> angleDistribution(0, 2 * M_PI);

	for (size_t i = 0; i < count; i++) {
		double z = heightDistribution(rng);
		double phi = angleDistribution(rng);
		double theta = asin(z);
		double x = cos(theta) * cos(phi);
		double y = cos(theta) * sin(phi);

		points.push_back(sv::Real3(x, y, z));
	}

	return points;
}

int main(int argc, char* argv[]) {

	if (argc < 2) {
		std::cout << "Usage: " << argv[0] << " [Number of cells]\n";
		return 1;
	}

	int cellCount = std::stoi(argv[1]);
	auto points = generatePoints(cellCount);

	auto constructStart = std::chrono::high_resolution_clock::now();

	sv::SphericalVoronoiCore voronoiDiagram(points);

	auto constructEnd = std::chrono::high_resolution_clock::now();
	auto constructDur = constructEnd - constructStart;
	std::cout << "Construct took " << std::chrono::duration_cast<std::chrono::nanoseconds>(constructDur).count() << " ns\n";

	voronoiDiagram.solve();

	auto solveEnd = std::chrono::high_resolution_clock::now();

	/*voronoiDiagram.solve([](int value)
	{
		std::cout << "In callback: " << value << std::endl;
	});*/

	auto solveDur = solveEnd - constructEnd;

	std::cout << "Solve took " << std::chrono::duration_cast<std::chrono::nanoseconds>(solveDur).count() << " ns\n";

	system("pause");

	return 0;
}
