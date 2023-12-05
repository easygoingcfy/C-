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
    auto writer = csv::make_csv_writer(outfile);
    if (!infile.good()) {
        writer << std::vector<std::string>({"A", "B", "C"});
    }
    writer << std::array<int, 3>({1, 2, 3});
    writer << std::array<int, 3>({4, 5, 6});
    return 0;
}