// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2021 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#include <string>
#include <vector>
#include <armadillo>
#include <chrono>

#include "open3d/Open3D.h"
#include <Eigen/StdVector>

using namespace std;
using namespace open3d;

#define CurrTimeMS (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count())

int main(int argc, char *argv[]) {
    if (argc == 2) {
        std::string option(argv[1]);
        if (option == "--skip-for-unit-test") {
            open3d::utility::LogInfo("Skiped for unit test.");
            return 0;
        }
    }

    auto pcd0 = io::CreatePointCloudFromFile("../data/prevf.txt", "xyz", true);
    auto pcd1 = io::CreatePointCloudFromFile("../data/nextf.txt", "xyz", true);

    // auto pcd0 = io::CreatePointCloudFromFile("../data/pos0.txt", "xyz", true);
    // auto pcd1 = io::CreatePointCloudFromFile("../data/pos1.txt", "xyz", true);

    cout << "pcd0 size: " << pcd0->points_.size() << endl;
    cout << "pcd1 size: " << pcd1->points_.size() << endl;

    long t1 = CurrTimeMS;

    auto rr = pipelines::registration::RegistrationICP(*pcd0, *pcd1, 2, 
                                        Eigen::MatrixBase<Eigen::Matrix4d>::Identity(), 
                                        open3d::pipelines::registration::TransformationEstimationPointToPoint(false), 
                                        open3d::pipelines::registration::ICPConvergenceCriteria());

    long t2 = CurrTimeMS;
    std::cout << "icp took " << t2-t1 << " ms.\n";

    cout << rr.transformation_ << endl;

    return 0;
}
