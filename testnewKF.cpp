#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "AprilFilter.h"

int main() {
    // Hardcoded file paths
    std::string inputFilePath = R"(C:\Users\Snoopy\Documents\TRT\SRAD_Avionics\Spaceport\23-24\Code\Teensy-Based Avionics\src\StateTest\fake_data.csv)";
    std::string outputFilePath = R"(C:\Users\Snoopy\Documents\TRT\SRAD_Avionics\Spaceport\23-24\Code\Teensy-Based Avionics\src\StateTest\test_results_windows.csv)";

    std::ifstream inputFile(inputFilePath);
    std::ofstream outputFile(outputFilePath);

    if (!inputFile.is_open() || !outputFile.is_open()) {
        std::cerr << "Error opening files!" << std::endl;
        return 1;
    }

    std::string line;
    std::vector<double> data(16); // Adjust the size based on the maximum index you need, e.g., 15

    // Create an instance of the filter
    LinearKalmanFilter* kf = initializeFilter();

    // Skip the header line
    std::getline(inputFile, line);
    double lastTime = 1838.66;
    // Process each line of the file
    while (std::getline(inputFile, line)) {
        std::cout << line << std::endl;
        std::istringstream ss(line);

        // Read each column into the data vector
        for (size_t i = 0; i < data.size(); ++i) {
            ss >> data[i];
            if (ss.peek() == ',') ss.ignore();
        }

        // Assign the specific columns to your variables
        double *input = new double[3]{ data[3], data[4], data[5] }; // Columns 4, 5, 6
        data[5] -= 9.8; // Subtract gravity from the z-acceleration
        double *measurement = new double[3]{ data[14], data[15], data[10] }; // Columns 15, 16, 11
        std::cout << data[0] - lastTime << std::endl;
        double* result = iterateFilter(*kf, data[0] - lastTime, input, measurement);

        lastTime = data[0];
        // Output the state to the output file
        for (int i = 0; i < 6; ++i) {
            outputFile << result[i];
            if (i < 5) outputFile << ", ";
        }
        outputFile << std::endl;

        delete[] result;
    }

    delete kf;

    inputFile.close();
    outputFile.close();

    return 0;
}
