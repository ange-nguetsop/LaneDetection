# Lane Detection Project

## Overview

The goal of this code is to detect and highlight road lane lines within an image. It involves a multi-step process for image preprocessing, edge detection, region of interest (ROI) extraction, lane line identification, and overlaying the detected lanes onto the original image.

## Image Preprocessing

### Blurring and Color Space Transformation

The initial stage involves blurring the image to reduce noise, followed by transforming the image from the BGR color space to the HLS color space. HLS images are more adaptable to varying lighting conditions, addressing issues arising from extreme brightness or shadows caused by sunlight.

### Edge Detection using Sobel

Sobel edge detection is utilized due to its resilience against minor pixel variations. It's applied along both x and y axes to generate an edge-detected image, providing a complete contours representation.
![first step: Edges detection with Sobel](..\LaneDetection\First_step.png)

### Binary Thresholding

Binary thresholding on the S (saturation) channel isolates white or yellow lane markings. Pixels with high saturation values (typically above 80) are recognized as potential lane colors. Additionally, thresholding on the R (red) channel further identifies lane colors, and the intersection of these operations highlights preprocessed lane markings.

## Region of Interest (ROI)

### Canny Edge Detection and Trapezoidal Masks

Canny edge detection identifies crucial contours in the preprocessed image. Trapezoidal masks define the Region of Interest (ROI), extracting only road lane information while filtering out extraneous elements such as arrow road signs. The enclosed space ideally encompasses lane details, aiding lane line detection.

## Lane Line Detection

### Hough Transform and Line Selection

The HoughLinesP() function initially yields potential lines corresponding to lane edges. Through selection, the code retains only the extreme ends of the left and right lanes, outlining the outer lane edges.

## Overlaying Detected Lanes

Finally, the detected lane lines are overlaid onto the original image, providing a visual indication of lane lines within the original context.

## Conclusion

Feel free to modify and enhance the code to suit specific requirements. Feedback and improvements to the code are welcomed and encouraged.

