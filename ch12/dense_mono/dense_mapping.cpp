#include <iostream>
#include <vector>
#include <fstream>

using namespace std;

#include <boost/timer.hpp>

// for sophus
#include <sophus/se3.hpp>

using Sophus::SE3d;

// for eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

// ------------------------------------------------------------------
// parameters
const int boarder = 20;         // border pixels
const int width = 640;          // image width
const int height = 480;         // image height
const double fx = 518.0f;       // camera parameters
const double fy = 519.0f;
const double cx = 325.5f;
const double cy = 253.5f;
const int ncc_window_size = 3;    // NCC windows half size
const int ncc_area = (2 * ncc_window_size + 1) * (2 * ncc_window_size + 1);
const double min_cov = 0.1;
const double max_cov = 10;

// ------------------------------------------------------------------

bool readDatasetFiles(
    const string &path,
    vector<string> &color_image_files,
    vector<SE3d> &poses
);

/**
 * update depth based on current image
 * @param ref
 * @param curr
 * @param T_C_R         Transformation from ref to cur
 * @param depth
 * @param depth_cov
 * @return
 **/
bool update(
    const Mat &ref,
    const Mat &curr,
    const SE3d &T_C_R,
    Mat &depth,
    Mat &depth_cov2
);

bool epipolarSearch(
    const Mat &ref,
    const Mat &curr,
    const SE3d &T_C_R,
    const Vector2d &pt_ref,
    const double &depth_mu,
    const double &depth_cov,
    Vector2d &pt_curr,
    Vector2d &epipolar_direction
);

bool updateDepthFilter(
    const Vector2d &pt_ref,
    const Vector2d &pt_curr,
    const SE3d &T_C_R,
    const Vector2d &epipolar_direction,
    Mat &depth,
    Mat &depth_cov2
);

// calculate NCC score
double NCC(const Mat &ref, const Mat &curr, const Vector2d &pt_ref, const Vector2d &pt_curr);

inline double getBilinearInterpolatedValue(const Mat &img, const Vector2d &pt) {
    uchar *d = &img.data[int(pt(1, 0)) * img.step + int(pt(0, 0))];
    double xx = pt(0, 0) - floor(pt(0, 0));
    double yy = pt(1, 0) - floor(pt(1, 0));
    return ((1 - xx) * (1 - yy) * double(d[0]) +
            xx * (1 - yy) * double(d[1]) +
            (1 - xx) * yy * double(d[img.step]) +
            xx * yy * double(d[img.step + 1])) / 255.0;
}

// ------------------------------------------------------------------

void plotDepth(const Mat &depth_truth, const Mat &depth_estimate);

inline Vector3d px2cam(const Vector2d px) {
    return Vector3d(
        (px(0, 0) - cx) / fx,
        (px(1, 0) - cy) / fy,
        1
    );
}

inline Vector2d cam2px(const Vector3d p_cam) {
    return Vector2d(
        p_cam(0, 0) * fx / p_cam(2, 0) + cx,
        p_cam(1, 0) * fy / p_cam(2, 0) + cy
    );
}

// detect whether point is within image
inline bool inside(const Vector2d &pt) {
    return pt(0, 0) >= boarder && pt(1, 0) >= boarder
           && pt(0, 0) + boarder < width && pt(1, 0) + boarder <= height;
}

void showEpipolarMatch(const Mat &ref, const Mat &curr, const Vector2d &px_ref, const Vector2d &px_curr);

void showEpipolarLine(const Mat &ref, const Mat &curr, const Vector2d &px_ref, const Vector2d &px_min_curr,
                      const Vector2d &px_max_curr);
// ------------------------------------------------------------------


int main(int argc, char **argv) {
    if (argc != 2) {
        cout << "Usage: dense_mapping path_to_test_dataset" << endl;
        return -1;
    }

    // Read data from cmd
    vector<string> color_image_files;
    vector<SE3d> poses_TWC;
    bool ret = readDatasetFiles(argv[1], color_image_files, poses_TWC);
    if (ret == false) {
        cout << "Reading image files failed!" << endl;
        return -1;
    }
    cout << "read total " << color_image_files.size() << " files." << endl;

    for (int img_i = 0; img_i < color_image_files.size(); img_i++) {
        cout << "*** Processing image " << img_i << " ***" << endl;

        Mat ref = imread(color_image_files[0], 0);                // gray-scale image
        SE3d pose_ref_TWC = poses_TWC[0];
        double init_depth = 3;    // intializaiton
        double init_cov2 = 1;
        Mat depth(height, width, CV_64F, init_depth);
        Mat depth_cov2(height, width, CV_64F, init_cov2);

        for (int index = 1; index < color_image_files.size(); index++) {
            Mat curr = imread(color_image_files[index], 0);
            if (curr.data == nullptr) continue;
            SE3d pose_curr_TWC = poses_TWC[index];
            SE3d pose_T_C_R = pose_curr_TWC.inverse() * pose_ref_TWC;   // T_C_W * T_W_R = T_C_R
            update(ref, curr, pose_T_C_R, depth, depth_cov2);
            // imshow("image", curr);
            cout << "estimation of image " << index << "..." << endl;
            // cout << "depth = "<< endl << " " << depth << endl << endl;
            // waitKey(1);
        }

        imwrite(std::string(argv[1]) + "/depth/depth_" + std::to_string(img_i) + ".png", depth/init_depth*255);
        color_image_files.push_back(color_image_files.front());
        color_image_files.erase(color_image_files.begin());
        poses_TWC.push_back(poses_TWC.front());
        poses_TWC.erase(poses_TWC.begin());
    }
   
    cout << "done." << endl;

    return 0;
}

bool readDatasetFiles(
    const string &path,
    vector<string> &color_image_files,
    std::vector<SE3d> &poses) {
    ifstream fin(path + "/pose.txt");
    if (!fin) return false;

    while (!fin.eof()) {
        // format: image_filename, tx, ty, tz, qx, qy, qz, qw (TWC)
        string image;
        fin >> image;
        double data[7];
        for (double &d:data) fin >> d;

        color_image_files.push_back(path + string("/images/") + image);
        poses.push_back(
            SE3d(Quaterniond(data[6], data[3], data[4], data[5]),
                 Vector3d(data[0], data[1], data[2]))
        );
        if (!fin.good()) break;
    }
    fin.close();
    return true;
}

// update the depth
bool update(const Mat &ref, const Mat &curr, const SE3d &T_C_R, Mat &depth, Mat &depth_cov2) {
    for (int x = boarder; x < width - boarder; x++)
        for (int y = boarder; y < height - boarder; y++) {

            if (depth_cov2.ptr<double>(y)[x] < min_cov || depth_cov2.ptr<double>(y)[x] > max_cov) // depth already converge or diverge
                continue;

            Vector2d pt_curr;
            Vector2d epipolar_direction;
            bool ret = epipolarSearch(
                ref,
                curr,
                T_C_R,
                Vector2d(x, y),
                depth.ptr<double>(y)[x],
                sqrt(depth_cov2.ptr<double>(y)[x]),
                pt_curr,
                epipolar_direction
            );

            if (ret == false)
                continue;

            // showEpipolarMatch(ref, curr, Vector2d(x, y), pt_curr);

            // udpate depth
            updateDepthFilter(Vector2d(x, y), pt_curr, T_C_R, epipolar_direction, depth, depth_cov2);
        }
}

bool epipolarSearch(
    const Mat &ref, const Mat &curr,
    const SE3d &T_C_R, const Vector2d &pt_ref,
    const double &depth_mu, const double &depth_cov,
    Vector2d &pt_curr, Vector2d &epipolar_direction) {
    Vector3d f_ref = px2cam(pt_ref);
    f_ref.normalize();
    Vector3d P_ref = f_ref * depth_mu;

    Vector2d px_mean_curr = cam2px(T_C_R * P_ref); // projection based on depth
    double d_min = depth_mu - 3 * depth_cov, d_max = depth_mu + 3 * depth_cov;
    if (d_min < 0.1) d_min = 0.1;
    Vector2d px_min_curr = cam2px(T_C_R * (f_ref * d_min));    // min depth projection
    Vector2d px_max_curr = cam2px(T_C_R * (f_ref * d_max));    // max depth projection

    Vector2d epipolar_line = px_max_curr - px_min_curr;
    epipolar_direction = epipolar_line;
    epipolar_direction.normalize();
    double half_length = 0.5 * epipolar_line.norm();
    if (half_length > 100) half_length = 100;   // a threshold for search

    // showEpipolarLine( ref, curr, pt_ref, px_min_curr, px_max_curr );

    // search on epipolar line
    double best_ncc = -1.0;
    Vector2d best_px_curr;
    for (double l = -half_length; l <= half_length; l += 0.7) { // l+=sqrt(2)
        Vector2d px_curr = px_mean_curr + l * epipolar_direction;
        if (!inside(px_curr))
            continue;

        double ncc = NCC(ref, curr, pt_ref, px_curr);
        if (ncc > best_ncc) {
            best_ncc = ncc;
            best_px_curr = px_curr;
        }
    }
    if (best_ncc < 0.85f)      // threshold for good NCC match
        return false;
    pt_curr = best_px_curr;
    return true;
}

double NCC(
    const Mat &ref, const Mat &curr,
    const Vector2d &pt_ref, const Vector2d &pt_curr) {
    double mean_ref = 0, mean_curr = 0;
    vector<double> values_ref, values_curr;
    for (int x = -ncc_window_size; x <= ncc_window_size; x++)
        for (int y = -ncc_window_size; y <= ncc_window_size; y++) {
            double value_ref = double(ref.ptr<uchar>(int(y + pt_ref(1, 0)))[int(x + pt_ref(0, 0))]) / 255.0;
            mean_ref += value_ref;

            double value_curr = getBilinearInterpolatedValue(curr, pt_curr + Vector2d(x, y));
            mean_curr += value_curr;

            values_ref.push_back(value_ref);
            values_curr.push_back(value_curr);
        }

    mean_ref /= ncc_area;
    mean_curr /= ncc_area;

    double numerator = 0, demoniator1 = 0, demoniator2 = 0;
    for (int i = 0; i < values_ref.size(); i++) {
        double n = (values_ref[i] - mean_ref) * (values_curr[i] - mean_curr);
        numerator += n;
        demoniator1 += (values_ref[i] - mean_ref) * (values_ref[i] - mean_ref);
        demoniator2 += (values_curr[i] - mean_curr) * (values_curr[i] - mean_curr);
    }
    return numerator / sqrt(demoniator1 * demoniator2 + 1e-10);
}

bool updateDepthFilter(
    const Vector2d &pt_ref,
    const Vector2d &pt_curr,
    const SE3d &T_C_R,
    const Vector2d &epipolar_direction,
    Mat &depth,
    Mat &depth_cov2) {

    // calculate depth based on trianglation
    SE3d T_R_C = T_C_R.inverse();
    Vector3d f_ref = px2cam(pt_ref);
    f_ref.normalize();
    Vector3d f_curr = px2cam(pt_curr);
    f_curr.normalize();

    // d_ref * f_ref = d_cur * ( R_RC * f_cur ) + t_RC
    // f2 = R_RC * f_cur
    // [ f_ref^T f_ref, -f_ref^T f2 ] [d_ref]   [f_ref^T t]
    // [ f_2^T f_ref, -f2^T f2      ] [d_cur] = [f2^T t   ]
    Vector3d t = T_R_C.translation();
    Vector3d f2 = T_R_C.so3() * f_curr;
    Vector2d b = Vector2d(t.dot(f_ref), t.dot(f2));
    Matrix2d A;
    A(0, 0) = f_ref.dot(f_ref);
    A(0, 1) = -f_ref.dot(f2);
    A(1, 0) = -A(0, 1);
    A(1, 1) = -f2.dot(f2);
    Vector2d ans = A.inverse() * b;
    Vector3d xm = ans[0] * f_ref;
    Vector3d xn = t + ans[1] * f2;
    Vector3d p_esti = (xm + xn) / 2.0;
    double depth_estimation = p_esti.norm();

    // calculate cov
    Vector3d p = f_ref * depth_estimation;
    Vector3d a = p - t;
    double t_norm = t.norm();
    double a_norm = a.norm();
    double alpha = acos(f_ref.dot(t) / t_norm);
    double beta = acos(-a.dot(t) / (a_norm * t_norm));
    Vector3d f_curr_prime = px2cam(pt_curr + epipolar_direction);
    f_curr_prime.normalize();
    double beta_prime = acos(f_curr_prime.dot(-t) / t_norm);
    double gamma = M_PI - alpha - beta_prime;
    double p_prime = t_norm * sin(beta_prime) / sin(gamma);
    double d_cov = p_prime - depth_estimation;
    double d_cov2 = d_cov * d_cov;

    // Gaussian fusion
    double mu = depth.ptr<double>(int(pt_ref(1, 0)))[int(pt_ref(0, 0))];
    double sigma2 = depth_cov2.ptr<double>(int(pt_ref(1, 0)))[int(pt_ref(0, 0))];

    double mu_fuse = (d_cov2 * mu + sigma2 * depth_estimation) / (sigma2 + d_cov2);
    double sigma_fuse2 = (sigma2 * d_cov2) / (sigma2 + d_cov2);

    depth.ptr<double>(int(pt_ref(1, 0)))[int(pt_ref(0, 0))] = mu_fuse;
    depth_cov2.ptr<double>(int(pt_ref(1, 0)))[int(pt_ref(0, 0))] = sigma_fuse2;

    return true;
}

void showEpipolarMatch(const Mat &ref, const Mat &curr, const Vector2d &px_ref, const Vector2d &px_curr) {
    Mat ref_show, curr_show;
    cv::cvtColor(ref, ref_show, CV_GRAY2BGR);
    cv::cvtColor(curr, curr_show, CV_GRAY2BGR);

    cv::circle(ref_show, cv::Point2f(px_ref(0, 0), px_ref(1, 0)), 5, cv::Scalar(0, 0, 250), 2);
    cv::circle(curr_show, cv::Point2f(px_curr(0, 0), px_curr(1, 0)), 5, cv::Scalar(0, 0, 250), 2);

    imshow("ref", ref_show);
    imshow("curr", curr_show);
    waitKey(1);
}

void showEpipolarLine(const Mat &ref, const Mat &curr, const Vector2d &px_ref, const Vector2d &px_min_curr,
                      const Vector2d &px_max_curr) {

    Mat ref_show, curr_show;
    cv::cvtColor(ref, ref_show, CV_GRAY2BGR);
    cv::cvtColor(curr, curr_show, CV_GRAY2BGR);

    cv::circle(ref_show, cv::Point2f(px_ref(0, 0), px_ref(1, 0)), 5, cv::Scalar(0, 255, 0), 2);
    cv::circle(curr_show, cv::Point2f(px_min_curr(0, 0), px_min_curr(1, 0)), 5, cv::Scalar(0, 255, 0), 2);
    cv::circle(curr_show, cv::Point2f(px_max_curr(0, 0), px_max_curr(1, 0)), 5, cv::Scalar(0, 255, 0), 2);
    cv::line(curr_show, Point2f(px_min_curr(0, 0), px_min_curr(1, 0)), Point2f(px_max_curr(0, 0), px_max_curr(1, 0)),
             Scalar(0, 255, 0), 1);

    imshow("ref", ref_show);
    imshow("curr", curr_show);
    waitKey(1);
}
