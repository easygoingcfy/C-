#include "csv.hpp"

#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <array>

int main() {
    const std::string filename = "test_csv.csv";

    // test file exists
    std::ifstream infile(filename);
    std::ofstream outfile(filename, std::ios_base::app);
    csv::CSVWriter<std::ofstream> writer = csv::make_csv_writer(nullptr);
    csv::CSVWriter<std::ofstream> writer = csv::make_csv_writer(outfile);
    std::vector<std::string> column = {
        "A", "B", "C"
    };
    if (!infile.good()) {
        writer << column;
    }
    writer << std::array<int, 3>({1, 2, 3});
    writer << std::array<int, 3>({4, 5, 6});
    writer << std::array<int, 7>({4, 5, 6, 7, 8, 9, 10});
    return 0;
}