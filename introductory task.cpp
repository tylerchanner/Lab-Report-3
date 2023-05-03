// Initialize a variable to store image data (frame)
Mat frame;

// Iterate through all provided image files
for (int i = 1; i < argc; i++)
{
    // Load the image into the frame variable
    frame = imread(argv[i]);

    // Check if the image is loaded successfully
    if (frame.empty())
    {
        // Display an error message and continue to the next iteration if the image is not loaded successfully
        std::cout << "Could not read image file " << argv[i] << std::endl;
        continue;
    }

    // Enter a loop to apply color filtering and isolate color channels
    while (1)
    {
        // Display the original image
        imshow("Original Image", frame);

        // Create structuring elements for morphological operations
        Mat spotFilter = cv::getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
        Mat maskMorph = cv::getStructuringElement(MORPH_ELLIPSE, Size(10, 10));

        // Convert the image to the HSV color space and apply inRange for the green channel
        Mat frameHSV_G;
        cvtColor(frame, frameHSV_G, COLOR_BGR2HSV);
        inRange(frameHSV_G, Scalar(40, 68, 57), Scalar(70, 255, 255), frameHSV_G);

        // Perform erosion and dilation operations for the green channel
        Mat erodedG, dilatedG;
        cv::erode(frameHSV_G, erodedG, spotFilter);
        cv::dilate(erodedG, dilatedG, maskMorph);

        // Convert the image to the HSV color space and apply inRange for the blue channel
        Mat frameHSV_B;
        cvtColor(frame, frameHSV_B, COLOR_BGR2HSV);
        inRange(frameHSV_B, Scalar(100, 150, 0), Scalar(140, 255, 255), frameHSV_B);

        // Perform erosion and dilation operations for the blue channel
        Mat erodedB, dilatedB;
        cv::erode(frameHSV_B, erodedB, spotFilter);
        cv::dilate(erodedB, dilatedB, maskMorph);

        // Convert the image to the HSV color space and apply inRange for the red lower channel
        Mat frameHSV_RL;
        cvtColor(frame, frameHSV_RL, COLOR_BGR2HSV);
        inRange(frameHSV_RL, Scalar(0, 50, 50), Scalar(10, 255, 255), frameHSV_RL);

        // Declare variables for storing eroded and dilated images of the red lower channel
        Mat erodedRL, dilatedRL;

        // Perform erosion operation on the red lower channel
        cv::erode(frameHSV_RL, erodedRL, spotFilter);

        // Perform dilation operation on the eroded red lower channel
        cv::dilate(erodedRL, dilatedRL, maskMorph);

        // Convert the image to the HSV color space and apply inRange for the red upper channel
        Mat frameHSV_RU;
        cvtColor(frame, frameHSV_RU, COLOR_BGR2HSV);
        inRange(frameHSV_RU, Scalar(170, 50, 50), Scalar(180, 255, 255), frameHSV_RU);

        // Declare variables for storing eroded and dilated images of the red upper channel
        Mat erodedRU, dilatedRU;

        // Perform erosion operation on the red upper channel
        cv::erode(frameHSV_RU, erodedRU, spotFilter);

        // Perform dilation operation on the eroded red upper channel
        cv::dilate(erodedRU, dilatedRU, maskMorph);

        // Display the filtered images for each channel
        imshow("Green Isolated", dilatedG);
        imshow("Blue Isolated", dilatedB);
        imshow("Red LOWER Isolated", dilatedRL);
        imshow("Red UPPER Isolated", dilatedRU);
        cv::waitKey(1);

        // Declare and initialize variables for non-zero pixel counts for each color channel
        int nonzeroB;
        int nonzeroG;
        int nonzeroRU;
        int nonzeroRL;
        int nonzeroR;

        // Count the number of non-zero pixels in each color channel and store the counts in the corresponding variable
        nonzeroB = countNonZero(dilatedB);
        nonzeroG = countNonZero(dilatedG);
        nonzeroRU = countNonZero(dilatedRU);
        nonzeroRL = countNonZero(dilatedRL);
        nonzeroR = nonzeroRU + nonzeroRL;

        // Wait for a key press for 1000 milliseconds (1 second)
        cv::waitKey(1000); // adjust to see how it works

        // Determine which color channel has the most non-zero pixels and print the corresponding message
        if (nonzeroB > nonzeroG && nonzeroB > nonzeroR)
        {
            std::cout << "The object is Blue" << std::endl;
            break;
        }

        if (nonzeroG > nonzeroB && nonzeroG > nonzeroR)
        {
            std::cout << "The object is Green" << std::endl;
            break;
        }

        if (nonzeroR > nonzeroG && nonzeroR > nonzeroB)
        {
            std::cout << "The object is Red" << std::endl;
            break;
        }

        // Close all windows
        destroyAllWindows();
    }
}