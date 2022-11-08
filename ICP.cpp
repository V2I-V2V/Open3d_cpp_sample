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

// // array size = N*dim
// std::shared_ptr<open3d::geometry::PointCloud> convert2pcd(float *data, size_t N, size_t dim=3) {
//     assert(dim > 2);
//     if (dim > 3) {
//         Eigen::Map<Eigen::MatrixXf> M(data, N, dim);
//         Eigen::MatrixXf fmat = M.leftCols(3);
//         Eigen::MatrixXd dmat = fmat.cast<double>();
//         Eigen::Vector3d *vv = reinterpret_cast<Eigen::Vector3d *>(dmat.data());
//         vector<Eigen::Vector3d> vvec(vv, vv+N);
//         return std::make_shared<open3d::geometry::PointCloud>(vvec);
//     } else {
//         Eigen::Map<Eigen::VectorXf> fvec(data, N * dim);
//         Eigen::VectorXd dvec = fvec.cast<double>();
//         Eigen::Vector3d *vv = reinterpret_cast<Eigen::Vector3d *>(dvec.data());
//         vector<Eigen::Vector3d> vvec(vv, vv+N);
//         return std::make_shared<open3d::geometry::PointCloud>(vvec);
//     }
// }

std::shared_ptr<open3d::geometry::PointCloud> convert2pcd(float *data, size_t N, size_t dim=3) {
    assert(dim > 2);
    double * dd = new double[N*3];
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < 3; ++j) {
            dd[i*3+j] = (double) data[i*dim+j];
        }
    }
    Eigen::Vector3d *vv = reinterpret_cast<Eigen::Vector3d *>(dd);
    long t1 = CurrTimeMS;
    auto pcd = std::make_shared<open3d::geometry::PointCloud>();
    pcd->points_.assign(vv, vv+N);
    delete[] dd;
    long t2 = CurrTimeMS;
    std::cout << "PCD init took " << t2-t1 << " ms.\n";
    return pcd;
}

int main(int argc, char *argv[]) {
    if (argc == 2) {
        std::string option(argv[1]);
        if (option == "--skip-for-unit-test") {
            open3d::utility::LogInfo("Skiped for unit test.");
            return 0;
        }
    }

    // auto pcd0 = io::CreatePointCloudFromFile("../data/prevf.txt", "xyz", true);
    // auto pcd1 = io::CreatePointCloudFromFile("../data/nextf.txt", "xyz", true);

    // auto pcd0 = io::CreatePointCloudFromFile("../data/pos0.txt", "xyz", true);
    // auto pcd1 = io::CreatePointCloudFromFile("../data/pos1.txt", "xyz", true);

    arma::fmat pts0(4, 40000, arma::fill::randn);
    arma::fmat pts1(4, 40000, arma::fill::randn);
    
    auto pcd0 = convert2pcd(pts0.memptr(), 40000, 4);
    auto pcd1 = convert2pcd(pts1.memptr(), 40000, 4);

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
