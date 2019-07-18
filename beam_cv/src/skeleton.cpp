#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <map>

using namespace cv;
using namespace std;

void onMouse(int evt, int x, int y, int flags, void *param)
{
    if (evt == EVENT_LBUTTONDOWN)
    {
        std::cout << x << " , " << y << std::endl;
    }
}

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        cout << " Usage: imskeleton ImageToLoadAndDisplay BinaryThreshold" << endl;
        return -1;
    }

    Mat image;
    image = imread(argv[1], 0); // Read the file

    // namedWindow("Image Review", WINDOW_NORMAL);
    // cv::setMouseCallback("Image Review", onMouse, 0);
    // imshow("Image Review", image);
    // waitKey(0); // Wait for a keystroke in the window

    if (!image.data) // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
        return -1;
    }

    // Get theshold of input
    int thresh;
    std::stringstream ss(argv[2]);
    ss >> thresh;

    // process image used (just temp with example image)
    cv::threshold(image, image, thresh, 255, cv::THRESH_BINARY);
    //cv::bitwise_not(image, image);

    // Initialize skel image and temp storage
    cv::Mat skel(image.size(), CV_8UC1, cv::Scalar(0));
    cv::Mat cm_skel(image.size(), CV_8UC1, cv::Scalar(0));
    cv::Mat temp(image.size(), CV_8UC1);
    cv::Mat remove_noise(image.size(), CV_8UC1, cv::Scalar(0));

    // Connected components
    cv::Mat im_category(image.size(), CV_16UC1);
    cv::Mat my_stats, my_centroids;
    int connectivity = 8;
    int itype = CV_16U;
    int num_comp = cv::connectedComponentsWithStats(image, im_category, my_stats, my_centroids, connectivity, itype);

    im_category.convertTo(im_category, CV_8UC1);

    std::cout << "Components: " << num_comp << std::endl;
    std::cout << "Stats: \n"
              << my_stats << std::endl;

    // Remove small pixel clusters
    for (int x = 0; x < image.rows; ++x) // iterate over skeleton image
    {
        for (int y = 0; y < image.cols; ++y) // iterate over skeleton image
        {
            int seg_category = int(im_category.at<uchar>(x, y));
            if (seg_category > 0)
            {
                if (my_stats.at<int>(seg_category, 4) < 100)
                {
                    image.at<uchar>(x, y) = 0;
                }
            }
        }
    }

    // Perform a closing operation
    cv::Mat element2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(image, image, cv::MORPH_CLOSE, element2);

    // Create a copy of the binary image
    cv::Mat image_copy = image.clone();

    num_comp = cv::connectedComponentsWithStats(image, im_category, my_stats, my_centroids, connectivity, itype);
    im_category.convertTo(im_category, CV_8UC1);

    // namedWindow("Image1", WINDOW_NORMAL);
    // namedWindow("Image2", WINDOW_NORMAL);
    // imshow("Image1", image);
    // imshow("Image2", image_copy);
    // waitKey(0); // Wait for a keystroke in the window

    // Declare structuring element for open function
    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

    bool done = false;
    while (!done)
    {
        cv::morphologyEx(image, temp, cv::MORPH_OPEN, element);
        cv::bitwise_not(temp, temp);
        cv::bitwise_and(image, temp, temp);
        cv::bitwise_or(skel, temp, skel);
        cv::erode(image, image, element);

        double max;
        cv::minMaxLoc(image, 0, &max);
        done = (max == 0);
        //imshow("Live Builder", skel);
        //waitKey(0);
    }

    // Initialize map for widths
    std::map<int, std::vector<double>> crack_map;

    // Calc the length of the crack
    int crack_length = cv::sum(skel)[0] / 255;
    std::cout << "Total Length : " << crack_length << std::endl;

    // Calc average width (pixels) by dividing total area by total length
    double avg_width = cv::sum(image_copy)[0] / cv::sum(skel)[0];
    std::cout << "Average width: " << avg_width << " pixels." << std::endl;

    // Example forced width calculation
    int resolution = std::nearbyint(std::sqrt(avg_width) * 0.5f) * 2.0f * 5 + 1;
    //int resolution = 9;
    std::cout << "Filter Resolution: " << resolution << std::endl;
    int stepper = (resolution - 1) / 2;
    double scale = std::floor(1.0 / num_comp * 255); // scale for colormap

    // initialize smaller window matrices
    cv::Mat im_orig(resolution, resolution, CV_8UC1);
    cv::Mat im_skel(resolution, resolution, CV_8UC1);
    for (int x = 0; x < skel.rows; ++x) // iterate over skeleton image
    {
        for (int y = 0; y < skel.cols; ++y) // iterate over skeleton image
        {
            int crack_num = int(im_category.at<uchar>(x, y));
            cm_skel.at<uchar>(x, y) = crack_num * scale;
            if (skel.at<uchar>(x, y) == 255) // if skeleton pixel
            {
                for (int i = 0; i < im_skel.rows; ++i) // iterate over window
                {
                    for (int j = 0; j < im_skel.cols; ++j)
                    {
                        // populate window with the values from skeleton and binary image
                        int idx_1 = x + i - stepper;
                        int idx_2 = y + j - stepper;

                        im_orig.at<uchar>(i, j) = image_copy.at<uchar>(idx_1, idx_2);
                        im_skel.at<uchar>(i, j) = skel.at<uchar>(idx_1, idx_2);
                    }
                }

                double width = cv::sum(im_orig)[0] / cv::sum(im_skel)[0];
                crack_map[crack_num].push_back(width);
            }
        }
    }

    for (int i = 1; i < num_comp; ++i) // 0 is all points not showing crack
    {
        std::cout << "Results for Crack: " << i << std::endl;
        if (crack_map[i].size() > 50)
        {
            std::cout << "Crack Length: " << crack_map[i].size() << std::endl;
            double max_width = *max_element(crack_map[i].begin(), crack_map[i].end());
            double min_width = *min_element(crack_map[i].begin(), crack_map[i].end());
            std::cout << "Max: " << max_width << " and Min: " << min_width << std::endl;
        }
        else
        {
            std::cout << "Not enough data to yield meaningful results" << std::endl;
        };
    }

    applyColorMap(cm_skel, cm_skel, COLORMAP_HOT);

    namedWindow("Original Image", WINDOW_NORMAL);
    namedWindow("Image Skeleton", WINDOW_NORMAL);
    imshow("Original Image", cm_skel);
    imshow("Image Skeleton", skel);
    waitKey(0); // Wait for a keystroke in the window
    return 0;
}