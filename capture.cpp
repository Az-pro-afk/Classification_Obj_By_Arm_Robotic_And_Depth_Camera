#include "capture.h"

capture::capture(QObject *parent): QThread{parent}
{
    cameraMatrix = cv::Mat::eye   (3,3, CV_64F);
    distCoeeffs  = cv::Mat::zeros (5,1, CV_64F);
    cameraMatrix.at<double>(0, 0) = 606.281066894531; // Fx
    cameraMatrix.at<double>(1, 1) = 606.417541503906; // Fy
    cameraMatrix.at<double>(0, 2) = 327.089965820313; // PPX
    cameraMatrix.at<double>(1, 2) = 240.459075927734; //PPY
    R_cam2base.copyTo(homogeneous_matrix(cv::Rect(0, 0, 3, 3)));
    t_cam2base.copyTo(homogeneous_matrix(cv::Rect(3, 0, 1, 3)));
    //-----------------------------------the code below is using for model ------------------
}
capture::~capture()
{
}

void capture::run()
{
    rs2::align align_to_color(RS2_STREAM_COLOR);

    decimation_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2.0);
    spatial_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.55f);
    spatial_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 8);       //try recommend
    temporal_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.4);
    temporal_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20.0);
    threshold_filter.set_option(RS2_OPTION_MIN_DISTANCE, 0.0);
    threshold_filter.set_option(RS2_OPTION_MAX_DISTANCE, 1.0);
    hole_filling_filter.set_option(RS2_OPTION_HOLES_FILL, 2);

    while(cameraState){
        QElapsedTimer time;
        time.start();

        rs2::frameset data = pipe.wait_for_frames();
        data = align_to_color.process(data);
        frame color = data.get_color_frame();
        frame depth;
        //-----------------------------------apply filter cho depth --------------
        if(isFilter == true){
           /*frame*/ depth = data.get_depth_frame();
 // Áp dụng các bộ lọc
//        depth = decimation_filter.process(depth);
           depth = depth_to_disparity.process(depth);
    //        depth = spatial_filter.process(depth);
           depth = temporal_filter.process(depth);
           depth = disparity_to_depth.process(depth);
           depth = hole_filling_filter.process(depth);
    //        depth = color_map.process(depth);
           depth = depth.apply_filter(colorMap);
        }
        else if(isFilter == false)
        {
            /*frame*/ depth = data.get_depth_frame().apply_filter(colorMap);
        }
        //-----------------------------------------------------------------
        auto stream = color.get_profile().as<rs2::video_stream_profile>();
        // Get depth frame
        rs2::depth_frame depth_frame = data.get_depth_frame();

        const int w_depth = depth.as<rs2::video_frame>().get_width();
        const int h_depth = depth.as<rs2::video_frame>().get_height();

        const int w_color = color.as<rs2::video_frame>().get_width();
        const int h_color = color.as<rs2::video_frame>().get_height();

        cv::Mat depthFrame(cv::Size(w_depth,h_depth), CV_8UC3, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat colorFrame(cv::Size(w_color,h_color), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);

        if(!colorFrame.empty() && !depthFrame.empty())
        {
            intrin = stream.get_intrinsics();
            // colorFrame = objDetect_4(colorFrame, centroidObj);

            if (detectAruco == true)
            {
                detectAndDrawArUco(colorFrame);
                center_pixel_distance = depth_frame.get_distance( pixel[0], pixel[1]);

                rs2_deproject_pixel_to_point (point_in_camera_space, &intrin, pixel, center_pixel_distance);
                std::cout << "3D Point in Camera Space: (" << point_in_camera_space[0] << ", " << point_in_camera_space[1] << ", " << point_in_camera_space[2] << ")" << std::endl;
                qDebug()<< "center_pixel_distance: "<< center_pixel_distance;
            }

            if (autoRunSystemBool)
            {
                detectAruco = false;
                colorFrame = objDetect_4(colorFrame, centroidObj);
                posObjZCoor = depth_frame.get_distance( centroidObj.x, centroidObj.y);
                               // std::cout << "have value of pos: " <<posObjZCoor;
                float pixel[2] = { float(centroidObj.x), float(centroidObj.y) };
                float point_in_camera_space[3];
                /*Use rs2_deproject_pixel_to_point to get the 3D coordinates*/
                rs2_deproject_pixel_to_point(point_in_camera_space, &intrin, pixel, posObjZCoor);
                caculatePosCam2Base(point_in_camera_space, translation_result);
            }
            if (streamOpt == 0)
            {
                pixmapColor = cvMatToQPixmap(colorFrame);
            }
            else
            {
                pixmapColor = cvMatToQPixmap(depthFrame);
            }
        }
        Q_EMIT newPixmapCaptured(pixmapColor);
    }
}
//---------------------------------------dont touch-------------------------------------------------
QImage capture::cvMatToQImage(const Mat &inMat){
    switch ( inMat.type() )
    {
    // 8-bit, 4 channel
    case CV_8UC4:
    {
        QImage image( inMat.data,
                     inMat.cols, inMat.rows,
                     static_cast<int>(inMat.step),
                     QImage::Format_ARGB32 );

        return image;
    }

        // 8-bit, 3 channel
    case CV_8UC3:
    {
        QImage image( inMat.data,
                     inMat.cols, inMat.rows,
                     static_cast<int>(inMat.step),
                     QImage::Format_RGB888 );

        return image.rgbSwapped();
    }

        // 8-bit, 1 channel
    case CV_8UC1:
    {
#if QT_VERSION >= QT_VERSION_CHECK(5, 15, 2)
        QImage image( inMat.data,
                     inMat.cols, inMat.rows,
                     static_cast<int>(inMat.step),
                     QImage::Format_Grayscale8 );
#else
        static QVector<QRgb>  sColorTable;

        // only create our color table the first time
        if ( sColorTable.isEmpty() )
        {
            sColorTable.resize( 256 );

            for ( int i = 0; i < 256; ++i )
            {
                sColorTable[i] = qRgb( i, i, i );
            }
        }

        QImage image( inMat.data,
                     inMat.cols, inMat.rows,
                     static_cast<int>(inMat.step),
                     QImage::Format_Indexed8 );

        image.setColorTable( sColorTable );
#endif

        return image;
    }

    default:
        qWarning() << "ASM::cvMatToQImage() - cv::Mat image type not handled in switch:" << inMat.type();
        break;
    }

    return QImage();
}
QPixmap capture::cvMatToQPixmap(const Mat &inMat){
    return QPixmap::fromImage(cvMatToQImage(inMat));
}
//--------------------------------Caculate 3D-------------------------------------------------------
void capture::caculatePosCam2Base(float point_in_camera_space[3], cv::Mat& t_result)
{
    cv::Mat pos_homogeneous = (cv::Mat_<double>(4, 1) <<
                                point_in_camera_space[0]*1000,
                                point_in_camera_space[1]*1000,
                                point_in_camera_space[2]*1000,
                                1);

    cv::Mat result = homogeneous_matrix * pos_homogeneous;

    // Trích xuất phần dịch chuyển t từ result
    t_result = result(cv::Rect(0, 0, 1, 3));   
}
//--------------------------------------------------------------------------------------------------
cv::Mat capture::objDetect_1(const cv::Mat& frameInput, cv::Point &center)          // using HSV for object
{

    cv::Rect roiRect(200, 74, 640 - 200, 279 - 74); // x, y, width, height
    cv::Mat frame = frameInput.clone(); // Clone the input frame to keep the original size

    // // Create a mask of zeros (same size as the original frame)
    cv::Mat mask = cv::Mat::zeros(frame.size(), CV_8UC1);

    // // Set the ROI part of the mask to 255 (indicating the area to be processed)
    mask(roiRect).setTo(255);

    // // Convert the frame to HSV
    cv::Mat colorFrameHSV;
    cv::cvtColor(frame, colorFrameHSV, cv::COLOR_BGR2HSV);

    // cv::imshow("colorFrame", mask);
    // cv::waitKey(1);

    // Create a final mask that combines all color ranges
    cv::Mat maskAll = cv::Mat::zeros(frame.size(), CV_8UC1);
    std::vector<cv::Mat> masks;
    std::vector<std::pair<cv::Scalar, cv::Scalar>> colorRanges = {
        // {cv::Scalar(6, 212, 128),   cv::Scalar(17, 255, 206)},   // Orange
        // {cv::Scalar(38, 79, 61),    cv::Scalar(56, 231, 159)},   // Green
        // {cv::Scalar(20, 129, 100),  cv::Scalar(31, 255, 211)},   // Yellow
        // {cv::Scalar(0, 102, 100),   cv::Scalar(10, 255, 219)},   // Red (lower range)
        // {cv::Scalar(160, 102, 100), cv::Scalar(179, 255, 219)}   // Red (upper range)

        {cv::Scalar(6, 212, 128),   cv::Scalar(23, 255, 255)},   // Orange
        {cv::Scalar(38, 82, 110),    cv::Scalar(68, 169, 237)},   // Green
        {cv::Scalar(5, 156, 128),  cv::Scalar(41, 255, 255)},   // Yellow
        {cv::Scalar(0,165,119),   cv::Scalar(10,215,255)},   // Red (lower range)
        {cv::Scalar(88,175,163), cv::Scalar(179,215,255)}   // Red (upper range)

    };
    std::vector<std::string> colorNames = {"Orange", "Green", "Yellow", "Red", "Red"};

    for (const auto& range : colorRanges) {
        cv::Mat maskColor;
        cv::inRange(colorFrameHSV, range.first, range.second, maskColor);
        maskAll |= maskColor;
        masks.push_back(maskColor);
    }

    // Apply the mask to restrict processing to the ROI
    cv::Mat maskedAll;
    maskAll.copyTo(maskedAll, mask);

    // Apply morphological operations to clean the mask
    int kernelSize = 7;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernelSize, kernelSize));
    cv::erode(maskedAll, maskedAll, kernel);
    cv::dilate(maskedAll, maskedAll, kernel);


    for (size_t j = 0; j < masks.size(); ++j) {
        cv::Mat maskedColor;
        masks[j].copyTo(maskedColor, mask);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(maskedColor, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area > 5000) {
                cv::Moments M = cv::moments(contour);
                cv::Point centroid(M.m10 / M.m00, M.m01 / M.m00);

                // cv::drawContours(frame, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 168, 200), 2);                // ve duong bao vat the
                cv::Rect boundingRect = cv::boundingRect(contour);

                double minDist = std::numeric_limits<double>::max();
                cv::Point closestPoint_temp;
                for (const auto& point : contour) {
                    double dist = cv::norm(point - centroid);
                    if (dist < minDist) {
                        minDist = dist;
                        closestPoint_temp = point;
                    }
                }

                double slope, y_intercept;
                cv::Point farthest_point;
                if (closestPoint_temp.x != centroid.x) {
                    slope = static_cast<double>(centroid.y - closestPoint_temp.y) / (centroid.x - closestPoint_temp.x);
                    y_intercept = closestPoint_temp.y - slope * closestPoint_temp.x;

                    double max_dist = 0;
                    for (const auto& point : contour) {
                        double y_intercept_diff = point.y - slope * point.x;
                        if (std::abs(y_intercept_diff - y_intercept) < 15)
                        {
                            double dist = cv::norm(point - closestPoint_temp);
                            if (dist > max_dist)
                            {
                                max_dist = dist;
                                farthest_point = point;
                            }
                        }
                    }
                } else {
                    slope = std::numeric_limits<double>::infinity();
                    y_intercept = closestPoint_temp.y;

                    double max_dist = 0;
                    for (const auto& point : contour) {
                        if (std::abs(point.x - closestPoint_temp.x) < 5)
                        {
                            double dist = cv::norm(point - centroid);
                            if (dist > max_dist)
                            {
                                max_dist = dist;
                                farthest_point = point;
                            }
                        }
                    }
                }

                cv::rectangle(frame, boundingRect, cv::Scalar(0, 168, 200), 2);

                center = centroid;
                if (colorNames[j] == "Orange") {
                    this->objectColor = 0;
                } else if (colorNames[j] == "Green") {
                    this->objectColor = 1;
                } else if (colorNames[j] == "Yellow") {
                    this->objectColor = 2;
                } else if (colorNames[j] == "Red") {
                    this->objectColor = 3;
                }

                if (centroid.x > 220 && centroid.x < 640) {
                    closestPoint = closestPoint_temp;
                    farthestPoint = farthest_point;
                    distanceGripper = minDist;

                    cv::Point point1_line2(109, 364);
                    cv::Point point2_line2(323, 364);

                    bool isQuadrant3or4 = (closestPoint.y < centroid.y && closestPoint.x < centroid.x) ||
                                          (farthestPoint.y > centroid.y && farthestPoint.x > centroid.x);

                    bool isQuadrant3or4_line2 = (closestPoint.y > centroid.y && closestPoint.x < centroid.x) ||
                                                (farthestPoint.y < centroid.y && farthestPoint.x > centroid.x);

                    if (isQuadrant3or4 || isQuadrant3or4_line2) {
                        double slope_line1 = static_cast<double>(closestPoint.y - farthestPoint.y) / (closestPoint.x - farthestPoint.x);
                        double slope_line2 = static_cast<double>(point2_line2.y - point1_line2.y) / (point2_line2.x - point1_line2.x);

                        double angle_rad = std::atan(std::abs((slope_line2 - slope_line1) / (1 + slope_line1 * slope_line2)));
                        double angle_deg =  (angle_rad * (180 / CV_PI) ) + 15.00;
                        if (slope_line2 < slope_line1) {
                            angle_deg = -angle_deg;
                        }
                        rtAngle = angle_deg;
                        prepareGrip = true;
                    }
                }
            }
        }
    }
    return frame;
}

cv::Mat capture::objDetect(const cv::Mat& frameInput, cv::Point &center)
{
    // Define the ROI (Region of Interest)
    cv::Rect roiRect(200, 74, 640 - 200, 279 - 74); // x, y, width, height
    cv::Mat frame = frameInput.clone(); // Clone the input frame to keep the original size

    // Create a mask of zeros (same size as the original frame)
    cv::Mat mask = cv::Mat::zeros(frame.size(), CV_8UC1);

    // Set the ROI part of the mask to 255 (indicating the area to be processed)
    mask(roiRect).setTo(255);

    // Convert the frame to grayscale
    cv::Mat grayFrame;
    cv::cvtColor(frame, grayFrame, cv::COLOR_BGR2GRAY);

    // Apply thresholding to create a binary image
    cv::Mat binaryImage;
    cv::threshold(grayFrame, binaryImage, 128, 255, cv::THRESH_BINARY);

    cv::imshow("mask", binaryImage);
    cv::waitKey(1);

    // Apply the mask to restrict processing to the ROI
    cv::Mat maskedBinary;
    binaryImage.copyTo(maskedBinary, mask);

    // Apply morphological operations to clean the mask
    int kernelSize = 7;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernelSize, kernelSize));
    cv::erode(maskedBinary, maskedBinary, kernel);
    cv::dilate(maskedBinary, maskedBinary, kernel);

    // Find contours on the restricted area
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(maskedBinary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area > 5000) {
            cv::Moments M = cv::moments(contour);
            cv::Point centroid(M.m10 / M.m00, M.m01 / M.m00);

            // Draw bounding box around the contour
            cv::Rect boundingRect = cv::boundingRect(contour);

            double minDist = std::numeric_limits<double>::max();
            cv::Point closestPoint_temp;
            for (const auto& point : contour) {
                double dist = cv::norm(point - centroid);
                if (dist < minDist) {
                    minDist = dist;
                    closestPoint_temp = point;
                }
            }

            double slope, y_intercept;
            cv::Point farthest_point;
            if (closestPoint_temp.x != centroid.x) {
                slope = static_cast<double>(centroid.y - closestPoint_temp.y) / (centroid.x - closestPoint_temp.x);
                y_intercept = closestPoint_temp.y - slope * closestPoint_temp.x;

                double max_dist = 0;
                for (const auto& point : contour) {
                    double y_intercept_diff = point.y - slope * point.x;
                    if (std::abs(y_intercept_diff - y_intercept) < 15) {
                        double dist = cv::norm(point - closestPoint_temp);
                        if (dist > max_dist) {
                            max_dist = dist;
                            farthest_point = point;
                        }
                    }
                }
            } else {
                slope = std::numeric_limits<double>::infinity();
                y_intercept = closestPoint_temp.y;

                double max_dist = 0;
                for (const auto& point : contour) {
                    if (std::abs(point.x - closestPoint_temp.x) < 5) {
                        double dist = cv::norm(point - centroid);
                        if (dist > max_dist) {
                            max_dist = dist;
                            farthest_point = point;
                        }
                    }
                }
            }

            if (farthest_point != cv::Point(0, 0)) {
                // cv::circle(frame, farthest_point, 5, cv::Scalar(0, 255, 0), -1); // Draw circle at the farthest point
                // cv::line(frame, closestPoint_temp, farthest_point, cv::Scalar(0, 168, 200), 2); // Draw line
            }

            cv::rectangle(frame, boundingRect, cv::Scalar(0, 168, 200), 2);

            center = centroid;

            if (centroid.x > 220 && centroid.x < 640) {
                closestPoint = closestPoint_temp;
                farthestPoint = farthest_point;
                distanceGripper = minDist;

                cv::Point point1_line2(109, 364);
                cv::Point point2_line2(323, 364);

                bool isQuadrant3or4 = (closestPoint.y < centroid.y && closestPoint.x < centroid.x) ||
                                      (farthestPoint.y > centroid.y && farthestPoint.x > centroid.x);

                bool isQuadrant3or4_line2 = (closestPoint.y > centroid.y && closestPoint.x < centroid.x) ||
                                            (farthestPoint.y < centroid.y && farthestPoint.x > centroid.x);

                if (isQuadrant3or4 || isQuadrant3or4_line2) {
                    double slope_line1 = static_cast<double>(closestPoint.y - farthestPoint.y) / (closestPoint.x - farthestPoint.x);
                    double slope_line2 = static_cast<double>(point2_line2.y - point1_line2.y) / (point2_line2.x - point1_line2.x);

                    double angle_rad = std::atan(std::abs((slope_line2 - slope_line1) / (1 + slope_line1 * slope_line2)));
                    double angle_deg =  (angle_rad * (180 / CV_PI) ) + 15.00;
                    if (slope_line2 < slope_line1) {
                        angle_deg = -angle_deg;
                    }
                    rtAngle = angle_deg;
                    prepareGrip = true;
                }
            }
        }
    }

    return frame;
}

cv::Mat capture::objDetect_2(const cv::Mat& frameInput, cv::Point &center)      //using background subtraction
{
    // Load the background image (only load once using static keyword)
    static cv::Mat background = cv::imread("D:/03_08_GUI_Python/captured_frame.jpg", cv::IMREAD_GRAYSCALE);
    // Clone the input frame to keep the original size
    cv::Mat frame = frameInput.clone();

    // Convert the frame to grayscale
    cv::Mat grayFrame;
    cv::cvtColor(frame, grayFrame, cv::COLOR_BGR2GRAY);

    grayFrame += 20;
    // Define the Region of Interest (ROI)
    cv::Rect roiRect(250, 100, 160, 130); 
    // Subtract the background from the current frame in the ROI
    cv::Mat fgMask;
    cv::absdiff(grayFrame(roiRect), background(roiRect), fgMask);

    // Display the background subtraction result
    cv::imshow("Background Subtraction", fgMask);
    cv::waitKey(1);

    // Apply median filter to reduce noise
    cv::medianBlur(fgMask, fgMask, 5);  // Applying a 5x5 median filter

    // Display the background subtraction result
    cv::imshow("Median Filter", fgMask);
    cv::waitKey(1);

    // Apply a threshold to create a binary mask
    cv::threshold(fgMask, fgMask, 25 , 255, cv::THRESH_BINARY);

    // // Display the background subtraction result
    cv::imshow("Thresholded", fgMask);
    cv::waitKey(1);

    // Apply morphological operations to clean the foreground mask
    int kernelSize = 3;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernelSize, kernelSize));

    int erodeIterations = 2;
    int dilateIterations = 1;

    // Apply erosion multiple times using a loop
    for (int i = 0; i < erodeIterations; i++) {
        cv::erode(fgMask, fgMask, kernel);
    }

    // // Display the background subtraction result
    cv::imshow("erode", fgMask);
    cv::waitKey(1);

    // Apply dilation multiple times using a loop
    for (int i = 0; i < dilateIterations; i++) {
        cv::dilate(fgMask, fgMask, kernel);
    }

    // // Display the background subtraction result
    cv::imshow("dilate", fgMask);
    cv::waitKey(1);

    // Find contours on the cleaned foreground mask
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(fgMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Iterate through each detected contour
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area > 3000) {  // Filter out small contours based on area
            // Calculate the centroid of the contour
            cv::Moments M = cv::moments(contour);
            cv::Point centroid(M.m10 / M.m00, M.m01 / M.m00);

            // Adjust centroid position relative to the full frame
            centroid.x += roiRect.x;
            centroid.y += roiRect.y;

            // Draw bounding box around the detected object
            cv::Rect boundingRect = cv::boundingRect(contour);
            boundingRect.x += roiRect.x;
            boundingRect.y += roiRect.y;
            cv::rectangle(frame, boundingRect, cv::Scalar(0, 168, 200), 2);

            // Draw the centroid on the frame
            cv::circle(frame, centroid, 5, cv::Scalar(0, 0, 255), -1);  // Red circle with a radius of 5 pixels

            cv::drawContours(frame, contours, -1, cv::Scalar(0, 255, 0), 2, cv::LINE_8, cv::noArray(), INT_MAX, roiRect.tl());

            // Find the closest point on the contour to the centroid
            double minDist = std::numeric_limits<double>::max();
            cv::Point closestPoint;
            for (const auto& point : contour) {
                cv::Point adjustedPoint = point + roiRect.tl(); // Adjust contour points to full frame coordinates
                double dist = cv::norm(adjustedPoint - centroid);
                if (dist < minDist) {
                    minDist = dist;
                    closestPoint = adjustedPoint;
                }
            }

            // Calculate the angle between the center and the closest point with respect to the x-axis
            double deltaX = closestPoint.x - centroid.x;
            double deltaY = closestPoint.y - centroid.y;
            double angleRad = std::atan2(deltaY, deltaX);  // atan2 calculates the angle in radians
            double angleDeg = angleRad * (180.0 / CV_PI);  // Convert to degrees

            // Draw a line from the center to the closest point
            cv::line(frame, centroid, closestPoint, cv::Scalar(0, 255, 0), 2);

            // Set the center point of the detected object
            center = centroid;
            rtAngle = angleDeg;

            prepareGrip = true;
        }
    }
    std::cout << "center object: " <<centroidObj << std::endl;

    return frame;
}

cv::Mat capture::objDetect_3(const cv::Mat& frameInput, cv::Point &center)      //using background subtraction
{
    // Load the background image (only load once using static keyword)
    static cv::Mat background = cv::imread("D:/03_08_GUI_Python/captured_frame.jpg", cv::IMREAD_GRAYSCALE);


    // Clone the input frame to keep the original size
    cv::Mat frame = frameInput.clone();

    // Convert the frame to grayscale
    cv::Mat grayFrame;
    cv::cvtColor(frame, grayFrame, cv::COLOR_BGR2GRAY);

    grayFrame += 27;
    // Define the Region of Interest (ROI)
    cv::Rect roiRect(250, 100, 300, 130);

    // Subtract the background from the current frame in the ROI
    cv::Mat fgMask;
    cv::absdiff(grayFrame(roiRect), background(roiRect), fgMask);

    // Display the background subtraction result
    cv::imshow("Background Subtraction", fgMask);
    cv::waitKey(1);

    // Display the background subtraction result
    cv::imshow("Median Filter", fgMask);
    cv::waitKey(1);

    // Apply a threshold to create a binary mask
    cv::threshold(fgMask, fgMask, 35 , 255, cv::THRESH_BINARY);

    // Display the background subtraction result
    cv::imshow("Thresholded", fgMask);
    cv::waitKey(1);

    // Apply morphological operations to clean the foreground mask
    int kernelSize = 3;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernelSize, kernelSize));

    int erodeIterations = 1;
    int dilateIterations = 3;

    // Apply erosion multiple times using a loop
    for (int i = 0; i < erodeIterations; i++) {
        cv::erode(fgMask, fgMask, kernel);
    }

    // Display the background subtraction result
    cv::imshow("erode", fgMask);
    cv::waitKey(1);

    // Apply dilation multiple times using a loop
    for (int i = 0; i < dilateIterations; i++) {
        cv::dilate(fgMask, fgMask, kernel);
    }

    // Display the background subtraction result
    cv::imshow("dilate", fgMask);
    cv::waitKey(1);

    // Find contours on the cleaned foreground mask
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(fgMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Store detected centroids and bounding boxes
    std::vector<cv::Point> detectedCentroids;
    std::vector<cv::Rect> detectedBoundingBoxes;

    // Iterate through each detected contour
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area > 3000) {  // Filter out small contours based on area
            // Calculate the centroid of the contour
            cv::Moments M = cv::moments(contour);
            cv::Point centroid(M.m10 / M.m00, M.m01 / M.m00);

            // Adjust centroid position relative to the full frame
            centroid.x += roiRect.x;
            centroid.y += roiRect.y;

            // Draw bounding box around the detected object
            cv::Rect boundingRect = cv::boundingRect(contour);
            boundingRect.x += roiRect.x;
            boundingRect.y += roiRect.y;
            cv::rectangle(frame, boundingRect, cv::Scalar(0, 168, 200), 2);


            // Store detected centroids and bounding boxes ---------------- For tracking-----------
            detectedCentroids.push_back(centroid);
            detectedBoundingBoxes.push_back(boundingRect);
            double minDist = std::numeric_limits<double>::max();
            cv::Point closestPoint;
            for (const auto& point : contour) {
                cv::Point adjustedPoint = point + roiRect.tl(); // Adjust contour points to full frame coordinates
                double dist = cv::norm(adjustedPoint - centroid);
                if (dist < minDist) {
                    minDist = dist;
                    closestPoint = adjustedPoint;
                }
            }

            // Calculate the angle between the center and the closest point with respect to the horizontal line y = 177
            double deltaX = closestPoint.x - centroid.x;
            double deltaY = closestPoint.y - centroid.y;

            // Calculate angle using atan2
            double angleRad = std::atan2(deltaY, deltaX);

            // Convert to degrees
            double angleDeg = angleRad * (180.0 / CV_PI);

            // Adjust the angle to be within -90 to 90 degrees
            if (angleDeg > 90) {
                angleDeg -= 180;
            } else if (angleDeg < -90) {
                angleDeg += 180;
            }
            center = centroid;
            rtAngle = angleDeg;
            prepareGrip = true;
        }
    }
    std::cout << "center object: " <<centroidObj << std::endl;

    // Associate detections with tracked objects
    for (auto& obj : trackedObjects) {
        double minDistance = std::numeric_limits<double>::max();
        int bestMatch = -1;

        // Find the closest detected object to the tracked object
        for (size_t i = 0; i < detectedCentroids.size(); ++i) {
            double distance = cv::norm(obj.centroid - detectedCentroids[i]);
            if (distance < minDistance) {
                minDistance = distance;
                bestMatch = static_cast<int>(i);
            }
        }

        // If a match is found, update the tracked object
        if (bestMatch != -1 && minDistance < 50.0) {  // Threshold for matching
            obj.centroid = detectedCentroids[bestMatch];
            obj.boundingBox = detectedBoundingBoxes[bestMatch];
            obj.lostFrames = 0;
            detectedCentroids.erase(detectedCentroids.begin() + bestMatch);
            detectedBoundingBoxes.erase(detectedBoundingBoxes.begin() + bestMatch);
        } else {
            obj.lostFrames++;
        }
    }

    // Remove objects that have been lost for too many frames
    trackedObjects.erase(
        std::remove_if(trackedObjects.begin(), trackedObjects.end(),
                       [this](const TrackedObject& obj) { return obj.lostFrames > MAX_LOST_FRAMES; }),
        trackedObjects.end());

    // Add new objects that weren't matched with existing ones
    for (size_t i = 0; i < detectedCentroids.size(); ++i) {
        TrackedObject newObj;
        newObj.centroid = detectedCentroids[i];
        newObj.boundingBox = detectedBoundingBoxes[i];
        newObj.id = nextObjectId++;
        newObj.lostFrames = 0;
        trackedObjects.push_back(newObj);
    }

    // Draw tracked objects
    for (const auto& obj : trackedObjects) {
        cv::rectangle(frame, obj.boundingBox, cv::Scalar(0, 168, 200), 2);
        cv::circle(frame, obj.centroid, 5, cv::Scalar(0, 0, 255), -1);
        std::string label = "ID: " + std::to_string(obj.id);
        cv::putText(frame, label, obj.boundingBox.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
    }
    return frame;
}

cv::Mat capture::objDetect_4(const cv::Mat& frameInput, cv::Point &center)      //using background subtraction
{
    cv::Mat background = cv::imread("D:/03_08_GUI_Python/captured_frame.jpg", cv::IMREAD_GRAYSCALE);
    cv::Mat frame = frameInput.clone();
    cv::Mat grayFrame;
    cv::cvtColor(frame, grayFrame, cv::COLOR_BGR2GRAY);
    grayFrame += 27;
    cv::Rect roiRect(100, 104, 450, 140);
    cv::Mat fgMask;
    cv::absdiff(grayFrame(roiRect), background(roiRect), fgMask);
    cv::threshold(fgMask, fgMask, 35 , 255, cv::THRESH_BINARY);

    int kernelSize = 3;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernelSize, kernelSize));
    int erodeIterations = 1;
    int dilateIterations = 3;
    for (int i = 0; i < erodeIterations; i++) {
        cv::erode(fgMask, fgMask, kernel);
    }
    for (int i = 0; i < dilateIterations; i++) {
        cv::dilate(fgMask, fgMask, kernel);
    }
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(fgMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<cv::Point> detectedCentroids;
    std::vector<cv::Rect> detectedBoundingBoxes;

    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);

        if (area > 4500) {  // Filter out small contours based on area
            // Calculate the centroid of the contour
            cv::Moments M = cv::moments(contour);
            cv::Point centroid(M.m10 / M.m00, M.m01 / M.m00);

            // Adjust centroid position relative to the full frame
            centroid.x += roiRect.x;
            centroid.y += roiRect.y;

            // Draw bounding box around the detected object
            cv::Rect boundingRect = cv::boundingRect(contour);
            boundingRect.x += roiRect.x;
            boundingRect.y += roiRect.y;

            // Store detected centroids and bounding boxes ---------------- For tracking-----------
            detectedCentroids.push_back(centroid);
            detectedBoundingBoxes.push_back(boundingRect);
            // Find the closest point on the contour to the centroid
            double minDist = std::numeric_limits<double>::max();
            cv::Point closestPoint;
            for (const auto& point : contour) {
                cv::Point adjustedPoint = point + roiRect.tl(); // Adjust contour points to full frame coordinates
                double dist = cv::norm(adjustedPoint - centroid);
                if (dist < minDist) {
                    minDist = dist;
                    closestPoint = adjustedPoint;
                }
            }
            // Calculate the angle between the center and the closest point with respect to the horizontal line y = 177
            double deltaX = closestPoint.x - centroid.x;
            double deltaY = closestPoint.y - centroid.y;

            // Calculate angle using atan2
            double angleRad = std::atan2(deltaY, deltaX);

            // Convert to degrees
            double angleDeg = angleRad * (180.0 / CV_PI);

            // Adjust the angle to be within -90 to 90 degrees
            if (angleDeg > 90) {
                angleDeg -= 180;
            } else if (angleDeg < -90) {
                angleDeg += 180;
            }
            center = centroid;
            rtAngle = angleDeg;
        }
    }

    // Cập nhật các đối tượng được theo dõi (trackedObjects)
    for (auto& obj : trackedObjects) {
        double minDistance = std::numeric_limits<double>::max();
        int bestMatch = -1;
        for (size_t i = 0; i < detectedCentroids.size(); ++i) {
            double distance = cv::norm(obj.centroid - detectedCentroids[i]);
            if (distance < minDistance) {
                minDistance = distance;
                bestMatch = static_cast<int>(i);
            }
        }
        if (bestMatch != -1 && minDistance < 50.0) {
            obj.centroid = detectedCentroids[bestMatch];
            obj.boundingBox = detectedBoundingBoxes[bestMatch];
            obj.lostFrames = 0;
            detectedCentroids.erase(detectedCentroids.begin() + bestMatch);
            detectedBoundingBoxes.erase(detectedBoundingBoxes.begin() + bestMatch);
        } else {
            obj.lostFrames++;
        }
    }

    // Loại bỏ các đối tượng bị mất theo dõi quá lâu
    trackedObjects.erase(
        std::remove_if(trackedObjects.begin(), trackedObjects.end(),
                       [this](const TrackedObject& obj) { return obj.lostFrames > MAX_LOST_FRAMES; }),
        trackedObjects.end());

    // Thêm các đối tượng mới
    for (size_t i = 0; i < detectedCentroids.size(); ++i) {
        TrackedObject newObj;
        newObj.centroid = detectedCentroids[i];
        newObj.boundingBox = detectedBoundingBoxes[i];
        newObj.id = nextObjectId++;
        newObj.lostFrames = 0;
        trackedObjects.push_back(newObj);
    }

    cv::Mat mask = cv::Mat::zeros(frame.size(), CV_8UC1);

    // Tìm đối tượng có ID nhỏ nhất
    if (!trackedObjects.empty()) {
        auto minIdObj = std::min_element(trackedObjects.begin(), trackedObjects.end(),
                                         [](const TrackedObject& a, const TrackedObject& b) {
                                             return a.id < b.id;
                                         });

        cv::Rect minBoundingBox = minIdObj->boundingBox;
        center = minIdObj->centroid;
        int expandSize = 20;
        // Điều chỉnh tọa độ x, y, width, height của bounding box
        minBoundingBox.x = std::max(0, minBoundingBox.x - expandSize); 
        minBoundingBox.y = std::max(0, minBoundingBox.y - expandSize);
        minBoundingBox.width = std::min(frame.cols - minBoundingBox.x, minBoundingBox.width + 2 * expandSize); 
        minBoundingBox.height = std::min(frame.rows - minBoundingBox.y, minBoundingBox.height + 2 * expandSize);

        //--------------------------------------

        mask(minBoundingBox).setTo(cv::Scalar(255));
        // frameAI(minBoundingBox).setTo(cv::Scalar(255));

        cv::Mat result;
        frame.copyTo(result, mask);

        if(tcpSend)
        {
            // Chuyển đổi và gửi ảnh qua tín hiệu emit
            QPixmap pixmapModel = cvMatToQPixmap(result);
            Q_EMIT newPixmapCapturedForModel(pixmapModel);
        }
    }
    else
    {
        center.x = 0;
        center.y = 0;
        QString filePath = "D:/03_08_GUI_Python/captured_frame.jpg";
        QPixmap pixmapModel;
        pixmapModel.load(filePath);
        Q_EMIT newPixmapCapturedForModel(pixmapModel);
    }
    cv::rectangle(frame, roiRect, cv::Scalar(0, 150, 160), 2);
    return frame;
}
std::vector<TrackedObject>& capture::getTrackedObjects() {
    return trackedObjects;
}

//----------------------------------------Something Stuff-------------------------------------------
void capture::toggleArucoDetection()
{
    detectAruco = !detectAruco;
    qDebug() << "value of detectArUco: "<< detectAruco;
}
void capture::toggleAutoRunSystem()
{
    autoRunSystemBool = !autoRunSystemBool;
}

void capture::detectAndDrawArUco(cv::Mat &colorFrame)
{
   cv::Mat clonedFrame = colorFrame.clone();

   std::vector<int> ids;
   std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
   cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_250);
   cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();

   // Detect ArUco markers in the cloned image
   cv::aruco::detectMarkers(clonedFrame, dictionary, markerCorners, ids, detectorParams, rejectedCandidates);

   // Estimate pose of the markers
   cv::aruco::estimatePoseSingleMarkers(markerCorners, 65.0, cameraMatrix, distCoeeffs, rvecs, tvecs);

   for(size_t i = 0; i < ids.size(); ++i) {
       // Draw the axis on the detected marker in the cloned frame
       cv::aruco::drawAxis(clonedFrame, cameraMatrix, distCoeeffs, rvecs[i], tvecs[i], 18.0);  // Length of the axis lines is set to 50.0

       // Calculate the center of the marker
       cv::Point2f center(0,0);
       for (const auto& corner : markerCorners[i]) {
           center.x += corner.x;
           center.y += corner.y;
       }
       center.x /= markerCorners[i].size();
       center.y /= markerCorners[i].size();

       // Draw a circle at the center of the marker in the cloned frame
       cv::circle(clonedFrame, center, 5, cv::Scalar(0, 255, 0), -1);

       // Store the center pixel coordinates
       pixel[0] = center.x;
       pixel[1] = center.y;
   }

   // Display the cloned frame with detected markers and axes
   cv::imshow("Detected ArUco", clonedFrame);
   cv::waitKey(1);
}
