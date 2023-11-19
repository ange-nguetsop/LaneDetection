// LaneDetection.cpp : Diese Datei enthält die Funktion "main". Hier beginnt und endet die Ausführung des Programms.
//
#pragma warning(disable : 4996)
#include <iostream>
#include <vector>
#include "stdafx.h"

using namespace cv;
using namespace std;

Mat RegionOfInterest(Mat source);
// Fonction pour calculer la distance entre deux points
float distanceBetweenPoints(Point p1, Point p2);

// Fonction pour trouver les deux lignes les plus éloignées
std::vector<Vec4i> findFarthestLines(std::vector<Vec4i> linesP); 

Point2f findIntersection(Vec4i line1, Vec4i line2);

void calculateLineEquation(Point p1, Point p2, float& m, float& c);

// Fonction pour calculer le point d'intersection d'une ligne avec le bas de l'image
Point intersectionWithBottomLine(Point linePoint1, Point linePoint2, int imageWidth, int imageHeight);

int main()
{
   cv::Mat im = cv::imread("original_lane_detection_5.jpg");

  // cv::GaussianBlur(im, im, cv::Size(3, 3), 0, 0);

    if (im.empty())
    {
        std::cout << "Impossible to open the file" << std::endl;
        return -1;
    }

    cv::Mat hls;
    cv::cvtColor(im, hls, cv::COLOR_BGR2HLS);

    if (im.rows != hls.rows)
    {
        std::cout << "the two images should have the same size" << std::endl;
        return -1;
    }
       
    // Split HLS channels
    cv::Mat channels[3];
    cv::split(hls, channels);

    //Split BGR channels
    cv::Mat bgrChannels[3];
    cv::split(im, bgrChannels);
  
    // Perform Sobel edge detection on the L channel
    cv::Mat sobelX, sobelY, sobelCombined;
    //cv::Canny(channels[1], sobelCombined, 80, 80 * 2, 3);

    Sobel(channels[1], sobelX, CV_16S, 1, 0); // Sobel on the L channel along x-axis
    Sobel(channels[1], sobelY, CV_16S, 0, 1); // Sobel on the L channel along y-axis

    // Convert back to 8-bit depth
    cv::convertScaleAbs(sobelX, sobelX);
    cv::convertScaleAbs(sobelY, sobelY);

    // Combine x and y edges (optional)
    cv::addWeighted(sobelX, 0.5, sobelY, 0.5, 0, sobelCombined);
    cv::GaussianBlur(sobelCombined, sobelCombined, cv::Size(5, 5), 0, 0);
    cv::threshold(sobelCombined, sobelCombined, 115, 255, cv::THRESH_BINARY); // faire varier la valeur de threshold en fonction de la qualité de l'image
     
    //Display the Sobel edge-detected images
    //cv::imshow("Sobel Combined", sobelCombined);

    // Apply binary thresholding on the S channel
    int thresholdValue = 80; // Adjust this threshold value as needed
    cv::Mat thresIm;
    cv::threshold(channels[2], thresIm, thresholdValue, 255, cv::THRESH_BINARY);

   // cv::imshow("Threshold", thresIm);

    // Apply binary thresholding on the Red channel
    int tresh_value = 120;
    cv::Mat redTreshIm;
    cv::threshold(bgrChannels[2], redTreshIm, tresh_value, 255, cv::THRESH_BINARY);

    //cv::imshow("ThresholdRed", thresIm);

    cv::Mat res;
    cv::bitwise_and(thresIm, redTreshIm, res);
    cv::bitwise_or(sobelCombined, res, res);

    cv::Mat dst,cdst,cdstP;
    GaussianBlur(res, dst, cv::Size(3, 3), 0, 0);
    Canny(dst, dst, 50, 50 * 2);
    cv::Mat roi = RegionOfInterest(dst);
    cv::Mat croi = roi.clone();
    // Copy edges to the images that will display the results in BGR
    cvtColor(roi, cdst, COLOR_GRAY2BGR);

    // Probabilistic Line Transform
    vector<Vec4i> linesP; // will hold the results of the detection
    HoughLinesP(roi, linesP, 1, CV_PI / 180, 50, 50, 10); // runs the actual detection

        // Trouver les deux lignes les plus éloignées
    std::vector<Vec4i> farthestLines = findFarthestLines(linesP);

    // Trouver le point de convergence (Vanishing point) des deux lignes détectées
    Point2f vanishingPoint = findIntersection(farthestLines[0], farthestLines[1]);

    // Calculer les équations des deux lignes
    float m1, c1, m2, c2;
    calculateLineEquation(Point(farthestLines[0][0], farthestLines[0][1]), Point(farthestLines[0][2], farthestLines[0][3]), m1, c1);
    calculateLineEquation(Point(farthestLines[1][0], farthestLines[1][1]), Point(farthestLines[1][2], farthestLines[1][3]), m2, c2);

    // Dessiner les lignes jusqu'au point de convergence
    // Calculer les coordonnées du dernier point de la ligne affichable sur l'image

    Point intersectionL1 = intersectionWithBottomLine(Point(farthestLines[0][0], farthestLines[0][1]), Point(farthestLines[0][2], farthestLines[0][3]), cdst.cols, cdst.rows);
    Point intersectionL2 = intersectionWithBottomLine(Point(farthestLines[1][0], farthestLines[1][1]), Point(farthestLines[1][2], farthestLines[1][3]), cdst.cols, cdst.rows);

    //line(cdst, intersectionL1, vanishingPoint, Scalar(255, 0, 0), 3, LINE_AA);
    //line(cdst, intersectionL2, vanishingPoint, Scalar(255, 0, 0), 3, LINE_AA);
    std::vector<Point> pts_roi;
    pts_roi.push_back(intersectionL1);
    pts_roi.push_back(intersectionL2);
    pts_roi.push_back(vanishingPoint);

    // Afficher le Vanishing point
    circle(cdst, vanishingPoint, 3, Scalar(0, 255, 255), -1);
    cv::fillPoly(cdst, pts_roi, Scalar(0, 255, 0));
    float alpha = 0.5;
    cv::Mat result;
    // Vérifier si les dimensions des images sont identiques

    // Superposition des deux images avec transparence
    addWeighted(im, alpha, cdst, 1 - alpha, 0, result);

    cv::imshow("First Step: highleight the lanes", res);
    cv::imwrite("First_step.png", res);
    cv::imshow("Second step: Find the contours of the processed image", dst);
    cv::imwrite("Seconf_step.png", dst);
    cv::imshow("Third step: Find the region of interest", roi);
    cv::imwrite("Third_step.png", roi);
    cv::imshow("Fourth step: detect and draw the contours of interest", cdst);
    cv::imwrite("Fourth_step.png", cdst);
    cv::imshow("Final step: Overlay the contours of interest on the original image", result);
    cv::imwrite("Result.png", result);
    cv::waitKey(0);
    cv::destroyAllWindows();


}

Mat RegionOfInterest(Mat source)
{
    /* In an ideal situation, the ROI should only contain the road lanes.
    We want to filter out all the other stuff, including things like arrow road markings.
    We try to achieve that by creating two trapezoid masks: one big trapezoid and a smaller one.
    The smaller one goes inside the bigger one. The pixels in the space between them will be kept and all the other pixels
    will be masked. If it goes well, the space between the two trapezoids contains only the lanes. */

    // Parameters big trapezoid
    float trapezoidBottomWidth = 1; // Width of bottom edge of trapezoid, expressed as percentage of image width
    float trapezoidTopWidth = 0.07; // Above comment also applies here, but then for the top edge of trapezoid
    float trapezoidHeight = 0.5; // Height of the trapezoid expressed as percentage of image height

    // Parameters small trapezoid
    float smallBottomWidth = 0.45; // This will be added to trapezoidBottomWidth to create a less wide bottom edge
    float smallTopWidth = 0.3; // We multiply the percentage trapoezoidTopWidth with this parameter to create a less wide top edge
    float smallHeight = 1.2; // Height of the small trapezoid expressed as percentage of height of big trapezoid

    // This parameter will make the trapezoids float just above the bottom edge of the image
    float bar = 0.97;

    // Vector which holds all the points of the two trapezoids
    std::vector<Point> pts;

    // Large trapezoid
    pts.push_back(cv::Point((source.cols * (1 - trapezoidBottomWidth)) / 2, source.rows * bar)); // Bottom left
    pts.push_back(cv::Point((source.cols * (1 - trapezoidTopWidth)) / 2, source.rows - source.rows * trapezoidHeight)); // Top left
    pts.push_back(cv::Point(source.cols - (source.cols * (1 - trapezoidTopWidth)) / 2, source.rows - source.rows * trapezoidHeight)); // Top right
    pts.push_back(cv::Point(source.cols - (source.cols * (1 - trapezoidBottomWidth)) / 2, source.rows * bar)); // Bottom right

    // Small trapezoid
    pts.push_back(cv::Point((source.cols * (1 - trapezoidBottomWidth + smallBottomWidth)) / 2, source.rows * bar)); // Bottom left
    pts.push_back(cv::Point((source.cols * (1 - trapezoidTopWidth * smallTopWidth)) / 2, source.rows - source.rows * trapezoidHeight * smallHeight)); // Top left
    pts.push_back(cv::Point(source.cols - (source.cols * (1 - trapezoidTopWidth * smallTopWidth)) / 2, source.rows - source.rows * trapezoidHeight * smallHeight)); // Top right
    pts.push_back(cv::Point(source.cols - (source.cols * (1 - trapezoidBottomWidth + smallBottomWidth)) / 2, source.rows * bar)); // Bottom right

    // Create the mask
    Mat mask = Mat::zeros(source.size(), source.type());
    fillPoly(mask, pts, Scalar(255, 255, 255));

    // Dessiner les trapezoids sur l'image source
    std::vector<std::vector<Point>> trapVec = { {pts[0], pts[1], pts[2], pts[3]}, {pts[4], pts[5], pts[6], pts[7]} };
    polylines(source, trapVec, true, Scalar(0,0, 255), 2); // Dessiner les trapezoids (vert, épaisseur 2)

    /* And here we basically put the mask over the source image,
    meaning we return an all black image, except for the part where the mask image
    has nonzero pixels: all the pixels in the space between the two trapezoids */
    Mat maskedImage;
    bitwise_and(source, mask, maskedImage);

    return maskedImage;
}

// Fonction pour calculer la distance entre deux points
float distanceBetweenPoints(Point p1, Point p2) {
    return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
}

// Fonction pour trouver les deux lignes les plus éloignées
std::vector<Vec4i> findFarthestLines(std::vector<Vec4i> linesP) {
    std::vector<Vec4i> farthestLines;

    // Initialisation des variables pour les deux lignes les plus éloignées
    float maxDistance = 0;
    Vec4i farthestLine1, farthestLine2;

    // Parcourir chaque paire de lignes pour trouver les deux plus éloignées
    for (size_t i = 0; i < linesP.size(); i++) {
        Vec4i line1 = linesP[i];
        for (size_t j = i + 1; j < linesP.size(); j++) {
            Vec4i line2 = linesP[j];

            // Calculer la distance entre les extrémités des lignes
            float distance = distanceBetweenPoints(Point(line1[0], line1[1]), Point(line2[0], line2[1])) +
                distanceBetweenPoints(Point(line1[2], line1[3]), Point(line2[2], line2[3]));

            // Vérifier si cette distance est plus grande que la précédente
            if (distance > maxDistance) {
                maxDistance = distance;
                farthestLine1 = line1;
                farthestLine2 = line2;
            }
        }
    }

    // Ajouter les deux lignes les plus éloignées au vecteur résultant
    farthestLines.push_back(farthestLine1);
    farthestLines.push_back(farthestLine2);

    return farthestLines;
}

// Fonction pour calculer le point d'intersection de deux lignes
Point2f findIntersection(Vec4i line1, Vec4i line2) {
    float x1 = line1[0], y1 = line1[1], x2 = line1[2], y2 = line1[3];
    float x3 = line2[0], y3 = line2[1], x4 = line2[2], y4 = line2[3];

    float ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));

    float x = x1 + ua * (x2 - x1);
    float y = y1 + ua * (y2 - y1);

    return Point2f(x, y);
}

void calculateLineEquation(Point p1, Point p2, float& m, float& c) {
    // Calculer la pente (m)
    m = static_cast<float>(p2.y - p1.y) / static_cast<float>(p2.x - p1.x);

    // Calculer l'ordonnée à l'origine (c)
    c = p1.y - m * p1.x;
}

// Fonction pour calculer le point d'intersection d'une ligne avec le bas de l'image
Point intersectionWithBottomLine(Point linePoint1, Point linePoint2, int imageWidth, int imageHeight) {
    Point intersectionPoint;

    // Coordonnées des extrémités de la ligne
    float x1 = linePoint1.x;
    float y1 = linePoint1.y;
    float x2 = linePoint2.x;
    float y2 = linePoint2.y;

    // Equation de la ligne : y = mx + c
    float m = (y2 - y1) / (x2 - x1); // Calcul de la pente (m)
    float c = y1 - m * x1; // Calcul de l'ordonnée à l'origine (c)

    // Calcul du point d'intersection avec le bas de l'image (y = imageHeight)
    intersectionPoint.y = imageHeight;
    intersectionPoint.x = (intersectionPoint.y - c) / m;

    // Limiter le point d'intersection pour qu'il soit dans les limites de l'image
    intersectionPoint.x = std::max(0, std::min(static_cast<int>(intersectionPoint.x), imageWidth));

    return intersectionPoint;
}