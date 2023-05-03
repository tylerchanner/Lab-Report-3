string compareImages(const Mat& target, const vector<string>& filenames, Mat& bestMatchImage) {
    double minDiff = 1e9; // Initialize minimum difference as a large number
    string bestMatch; // Variable to store the best match filename

    // Iterate through all filenames
    for (size_t i = 0; i < filenames.size(); ++i) {
        Mat image = imread(filenames[i], IMREAD_GRAYSCALE); // Read the image in grayscale
        resize(image, image, Size(350, 350)); // Resize the image to 350x350 pixels

        double diff = norm(target, image, NORM_L2); // Calculate L2 norm between the target and the resized image
        if (diff < minDiff) { // If the difference is smaller than the current minimum difference
            minDiff = diff; // Update the minimum difference
            bestMatch = filenames[i]; // Update the best match filename
            bestMatchImage = image; // Update the best match image
        }
    }

    return bestMatch; // Return the best match filename
}

struct PointComparator {
    bool operator()(const Point2f& a, const Point2f& b) {
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

int main() {
    Mat src = imread("star1.png"); // Read the source image

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

    cout << "Best match: " << bestMatch << endl; // Display the best match filename

    imshow("Best Match Image", bestMatchImage); // Display the best match image
}
