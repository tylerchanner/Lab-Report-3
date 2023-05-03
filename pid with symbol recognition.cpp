// Include files for required libraries
#include <stdio.h>
#include <time.h>

#include "opencv_aee.hpp"
#include "main.hpp" // You can use this file for declaring defined values and functions
#include "pi2c.h"

float prevError = 0;

Pi2c car(0x22); // Configure the I2C interface to the Car as a global variable

string compareImages(const Mat &target, const vector<string> &filenames, Mat &bestMatchImage)
{
    double minDiff = 1e9; // Initialize minimum difference as a large number
    string bestMatch;     // Variable to store the best match filename

    // Iterate through all filenames
    for (size_t i = 0; i < filenames.size(); ++i)
    {
        Mat image = imread(filenames[i], IMREAD_GRAYSCALE); // Read the image in grayscale
        resize(image, image, Size(350, 350));               // Resize the image to 350x350 pixels

        double diff = norm(target, image, NORM_L2); // Calculate L2 norm between the target and the resized image
        if (diff < minDiff)
        {                             // If the difference is smaller than the current minimum difference
            minDiff = diff;           // Update the minimum difference
            bestMatch = filenames[i]; // Update the best match filename
            bestMatchImage = image;   // Update the best match image
        }
    }

    return bestMatch; // Return the best match filename
}

struct PointComparator
{
    bool operator()(const Point2f &a, const Point2f &b)
    {
        return a.y < b.y; // Compare points based on their y-coordinates
    }
};

// Function to find the corners of a contour
vector<Point2f> findCorners(const vector<Point>& contour) {
    RotatedRect rotatedRect = minAreaRect(contour); // Find the minimum area rectangle enclosing the contour
    Point2f rectPoints[4];
    rotatedRect.points(rectPoints); // Get the four corner points of the rectangle

    sort(rectPoints, rectPoints + 4, PointComparator()); // Sort the corner points based on their y-coordinates

    // Swap corner points if necessary to maintain a consistent order
    if (rectPoints[0].x > rectPoints[1].x) {
        swap(rectPoints[0], rectPoints[1]);
    }

    if (rectPoints[2].x < rectPoints[3].x) {
        swap(rectPoints[2], rectPoints[3]);
    }

    vector<Point2f> corners(rectPoints, rectPoints + 4); // Store the sorted corner points in a vector
    return corners; // Return the corners
}

void setup(void)
{
    setupCamera(320, 240); // Enable the camera for OpenCV
}

int main(int argc, char **argv)
{
    setup(); // Call a setup function to prepare IO and devices

    class ScalarLowHSV{
        public:
        float x;
        float y;
        float z;

        ScalarLowHSV(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

        class ScalarHighHSV{
        public:
        float x;
        float y;
        float z;

        ScalarHighHSV(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

    cv::namedWindow("lineView");
    cv::namedWindow("croppedLineView1");
    cv::namedWindow("croppedLineView2");
    cv::namedWindow("croppedLineView3");
    cv::namedWindow("croppedLineView4");
    cv::namedWindow("croppedLineView5");
    cv::namedWindow("croppedLineView6");
    cv::namedWindow("croppedLineView7");
    cv::namedWindow("croppedLineView8");

    while (1) // Main loop to perform image processing
    {
        Mat frame;
        Mat hsvFrame;
        Mat lineView;
        Mat rotatedFrame;
        int values[8];

        Scalar ScalarHighHSV(255.0f, 255.0f, 50.0f)
        Scalar ScalarLowHSV(0.0f, 0.0f, 0.0f)
        
        while (frame.empty())
        {
            frame = captureFrame(); // Capture a frame from the camera and store in a new matrix variable
        }

        cv::Point2f centre((319) / 2, (239) / 2);
        cv::Mat flip = cv::getRotationMatrix2D(centre, 180, 1);
        cv::warpAffine(frame, rotatedFrame, flip, frame.size());
        // Display the image in the window
        cv::imshow("rotated", rotatedFrame);

        cvtColor(rotatedFrame, hsvFrame, COLOR_BGR2HSV);
        inRange(hsvFrame, Scalar(0, 0, 0), Scalar(255, 255, 50), lineView);
        cv::imshow("lineView", lineView);

        Mat cropped1 = lineView(Rect(0, 0, 40, 1));
        values[0] = countNonZero(cropped1);
        cv::imshow("croppedLineView1", cropped1);

        Mat cropped2 = lineView(Rect(40, 0, 40, 1));
        values[1] = countNonZero(cropped2);
        cv::imshow("croppedLineView2", cropped2);

        Mat cropped3 = lineView(Rect(80, 0, 40, 1));
        values[2] = countNonZero(cropped3);
        cv::imshow("croppedLineView3", cropped3);

        Mat cropped4 = lineView(Rect(120, 0, 40, 1));
        values[3] = countNonZero(cropped4);
        cv::imshow("croppedLineView4", cropped4);

        Mat cropped5 = lineView(Rect(160, 0, 40, 1));
        values[4] = countNonZero(cropped5);
        cv::imshow("croppedLineView5", cropped5);

        Mat cropped6 = lineView(Rect(200, 0, 40, 1));
        values[5] = countNonZero(cropped6);
        cv::imshow("croppedLineView6", cropped6);

        Mat cropped7 = lineView(Rect(240, 0, 40, 1));
        values[6] = countNonZero(cropped7);
        cv::imshow("croppedLineView7", cropped7);

        Mat cropped8 = lineView(Rect(280, 0, 40, 1));
        values[7] = countNonZero(cropped8);
        cv::imshow("croppedLineView8", cropped8);

        int i;
        float offset = -139.5;
        float sumOfCounts = 0;
        float sumOfWeights = 0;

        for (i = 0; i <= 7; i++)
        {
            offset += 40;
            values[i] = 40 - values[i];
            sumOfCounts += values[i] * offset;
            sumOfWeights += values[i];
        }

        float weightedAverage = sumOfCounts / sumOfWeights;
        float error = (37 - weightedAverage);

        float integral;
        // Calculate integral and derivative terms for PID controller
        float dt = 1.0 / 30; // assuming camera runs at 30 fps
        integral += error * dt;
        float derivative = (error - prevError) / dt;
        // Calculate output of PID controller
        float Kp = 10, Ki = 0, Kd = 0;
        float u = ((Kp * error) + (Ki * integral) + (Kd * derivative));

        // Send output to ESP32 over I2C
        uint8_t output_byte = (uint8_t)output;
        arduino.i2cRead(car, output_byte);

        // Update previous error
        prevError = error;

        int key = cv::waitKey(1); // Wait 1ms for a keypress (required to update windows)

           Scalar lowHSV(142, 12, 36); // Define the lower HSV threshold for colour-based segmentation
        Scalar highHSV(179, 152, 187); // Define the upper HSV threshold for colour-based segmentation

        Mat hsvImage, mask;
        cvtColour(src, hsvImage, COLOUR_BGR2HSV); // Convert the source image to HSV colour space
        inRange(hsvImage, lowHSV, highHSV, mask); // Create a binary mask based on the HSV threshold values

        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        findContours(mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE); // Find contours in the binary mask

        // Draw the contours on a copy of the source image
        Mat contoursImage = src.clone();
            for (size_t i = 0; i < contours.size(); i++) {
            drawContours(contoursImage, contours, i, Scalar(0, 255, 0), 2); // Draw each contour on the contoursImage
        }
        imshow("Contours", contoursImage); // Display the contoursImage

        double maxPerimeter = 0; // Initialize maximum perimeter as 0
        vector<Point> maxContour; // Variable to store the contour with the maximum perimeter

        // Iterate through all contours
        for (size_t i = 0; i < contours.size(); i++) {
            double perimeter = arcLength(contours[i], true); // Calculate the perimeter of the contour
            if (perimeter > maxPerimeter) { // If the perimeter is larger than the current maximum perimeter
                maxPerimeter = perimeter; // Update the maximum perimeter
                maxContour = contours[i]; // Update the maxContour
            }
        }

        if (maxContour.empty()) { // If no contour is found
            cerr << "No border found" << endl; // Display an error message
            return 1; // Exit with an error code
        }

        vector<Point2f> srcCorners = findCorners(maxContour); // Find the corners of the maxContour

        vector<Point2f> dstCorners(4);
        dstCorners[0] = Point2f(0, 0); // Define the destination top-left corner
        dstCorners[1] = Point2f(350, 0); // Define the destination top-right corner
        dstCorners[2] = Point2f(350, 350); // Define the destination bottom-right corner
        dstCorners[3] = Point2f(0, 350); // Define the destination bottom-left corner

        // Calculate the perspective transformation matrix
        Mat perspectiveMatrix = getPerspectiveTransform(srcCorners, dstCorners);
        Mat warpedImage; // Variable to store the rectified image
        // Apply perspective transformation to the source image and store the result in warpedImage
        warpPerspective(src, warpedImage, perspectiveMatrix, Size(350, 350));

        cvtColour(warpedImage, warpedImage, COLOUR_BGR2GRAY); // Convert the rectified image to grayscale
        imshow("Warped Image", warpedImage); // Display the rectified grayscale image

        // Define the filenames of the reference shape images
        vector<string> shapeFilenames;
        shapeFilenames.push_back("Triangle.png");
        shapeFilenames.push_back("Star.png");
        shapeFilenames.push_back("Umbrella.png");
        shapeFilenames.push_back("Circle.png");

        Mat bestMatchImage; // Variable to store the best match image
        // Find the best match image by comparing the rectified grayscale image to the reference images
        string bestMatch = compareImages(warpedImage, shapeFilenames, bestMatchImage);

        if(bestmatch == "Triangle.png"){Scalar ScalarHighHSV(140.0f, 255.0f, 255.0f);Scalar ScalarLowHSV(100.0f, 150.0f, 0.0f)}
        if(bestmatch == "Star.png"){Scalar ScalarHighHSV(70.0f, 255.0f, 255.0f);Scalar ScalarLowHSV(40.0f, 68.0f, 57.0f)}
        if(bestmatch == "Umbrella.png"){Scalar ScalarHighHSV(30.0f, 255.0f, 255.0f);Scalar ScalarLowHSV(20.0f, 100.0f, 100.0f)}
        if(bestmatch == "Circle.png"){Scalar ScalarHighHSV(10.0f, 255.0f, 255.0f);Scalar ScalarLowHSV(0.50f, 50.0f, 0.0f)}
        

        key = (key == 255) ? -1 : key; // Check if the ESC key has been pressed
        if (key == 27)
            break;
    }
}

closeCV(); // Disable the camera and close any windows

return 0;
