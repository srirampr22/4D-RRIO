#pragma once
#include<string>
#include<vector>
#include"hdbscanRunner.hpp"
#include"hdbscanParameters.hpp"
#include"hdbscanResult.hpp"
#include"outlierScore.hpp"

using namespace std;


class Hdbscan

{

private:

	string fileName;

	hdbscanResult result;

public:

	vector <vector<double>> dataset;

	std::vector<int> labels_;

	std::vector<int> normalizedLabels_;

	std::vector<outlierScore>outlierScores_;

	std::vector <double> membershipProbabilities_;

	uint32_t noisyPoints_;

	uint32_t numClusters_;



	Hdbscan(string readFileName) {

		fileName = readFileName;

	}

	Hdbscan() {};

	string getFileName();
			   
	int loadCsv(int numberOfValues, bool skipHeader=false);

	void execute(int minPoints, int minClusterSize, string distanceMetric);

	void displayResult();


};

