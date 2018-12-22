package org.firstinspires.ftc.teamcode;

import java.io.*;
import java.util.*;
import java.io.File;
import java.io.FileOutputStream;
import java.util.List;
import java.util.ArrayList;
import java.util.Date;
import java.text.SimpleDateFormat;

import android.graphics.Bitmap;
import android.os.Environment;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;
import org.opencv.core.Core;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.core.Point;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

// Use OpenCV to analyze an image from the camera and detect the gold mineral.
class LCHSAutoVisionOCV {

    private static final String TAG = "LCHSAutoVisionOCV";
    private static final SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMddHHmmss");
    private File picturesDir;
    private String picturesPath;
    private static final String imageFilePrefix = "Image";
    private boolean debugVerbose = false;

    // Object for Vuforia.
    LCHSAutoVuforia autoVuforia;

    // OpenCV values.
    // The following constants were determined via the eclipse test project for image
    // processing OpenCVObjectRecognition.

    // Low and high HSV thresholds.
    private static final int iLowH = 10;
    private static final int iHighH = 50;
    private static final int iLowS = 100;
    private static final int iHighS = 255;
    private static final int iLowV = 100;
    private static final int iHighV = 220;

    // Gaussian blur kernel size.
    private static final int gaussianBlurKernelSize = 45;

    // double <region of interest: percentage to crop off the top of the image>
    private static final double cropPercentC = 0; //50; // center
    private static final double cropPercentLR = 0; //40; // left and right

    // int <minimum area of an enclosing rectangle to be considered a gold mineral>
    private static final int minAreaC = 15000; // center
    private static final int minAreaLR = 8000; // left and right

    // double <the maximum ratio of the image width to height>
    // double <the maximum ratio of the image height to width>
    private static final double maxRatioWH = 1.5;
    private static final double maxRatioHW = 1.5;

    // Initialize the OpenCV library.
    //** PUT into LCHSAutoVisionOCV
    private static boolean openCVInitialized = false;

    static {
        if (OpenCVLoader.initDebug())
            openCVInitialized = true;
    }

    public LCHSAutoVisionOCV(LCHSAutoVuforia pAutoVuforia, boolean pDebugVerbose) {
        autoVuforia = pAutoVuforia;
        debugVerbose = pDebugVerbose;

        picturesDir = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES);
        try {
            picturesPath = picturesDir.getCanonicalPath();
        } catch (IOException iox) {
            throw new AutonomousRobotException(TAG, "Error accessing the PICTURES directory"
                    + iox.getMessage());
        }

        // A failure in OpenCV initialization will prevent us from recognizing
        // the gold mineral sample, but do not treat this as fatal.
        if (!openCVInitialized)
            RobotLog.dd(TAG, "Failure in OpenCV initialization");
    }

    public boolean findGoldMineralOpenCV(LCHSValues.MineralPosition pMineralPosition) throws InterruptedException {

        RobotLog.dd(TAG, "In findGoldMineralOpenCV");
        List<Bitmap> vuforiaImages = autoVuforia.getVuforiaImages();

        int imageCollectionSize = vuforiaImages.size();
        RobotLog.dd(TAG, "Got " + imageCollectionSize + " images from Vuforia");
        if (imageCollectionSize == 0)
            return false; // don't crash

        // Get the last image in the collection.
        Bitmap oneCameraImage = vuforiaImages.get(--imageCollectionSize);

        // If in TEST mode write all images out to a file.
        if (debugVerbose) {
            String fileDate = dateFormat.format(new Date());
            for (int imX = 0; imX < vuforiaImages.size(); imX++) {
                // Append image index number to the date
                writeImageFileFromBitmap(vuforiaImages.get(imX), fileDate + "_" + Integer.toString(imX));
            }
        }

        RobotLog.dd(TAG, "Using the camera image at index " + imageCollectionSize);
        RobotLog.dd(TAG, "Looking for a gold mineral at position " + pMineralPosition);
        boolean retVal = findGoldInImageOpenCV(oneCameraImage, pMineralPosition);
        if (retVal)
            RobotLog.dd(TAG, "Found a gold mineral");
        else
            RobotLog.dd(TAG, "Did not find a gold mineral");
        return retVal;
    }


    private boolean findGoldInImageOpenCV(Bitmap pLandscapeBitmap, LCHSValues.MineralPosition pMineralPosition) {

        boolean retVal = false;

        // Write the bitmap image out to a jpeg file.
        RobotLog.dd(TAG, "Entered LCHSAutoVisionOCV.findGoldInImageOpenCV()");
        String fileDate = dateFormat.format(new Date());
        writeImageFileFromBitmap(pLandscapeBitmap, fileDate);

        try {

            // Convert the bitmap returned by Vuforia into a Mat for image processing. The bitmap is in
            // landscape format.

            // Neither of these two methods worked - the area of the gold mineral was shrunk to a very
            // small number. Not debugged, however, just switched to reading in the jpg file.
 //           Mat imgOriginal = new Mat();
 //            = new Mat(pLandscapeBitmap.getHeight(), pLandscapeBitmap.getWidth(), CvType.CV_8UC3);
 //           Utils.bitmapToMat(pLandscapeBitmap, imgOriginal);

            // Put the input image into a matrix.
            //  Here's one way (found on the internet, not tried):
            //  ByteArrayOutputStream baoStream = new ByteArrayOutputStream();
            //  portraitBitmap.compress(Bitmap.CompressFormat.JPG, 100, baoStream);
            //  byte[] imageBytes = baoStream.toByteArray();
            //  baoStream.close();
            //  Mat img = Imgcodecs.imdecode(new MatOfByte(imageBytes), Imgcodecs.IMREAD_COLOR); // for bitmap

            // But it seems easier to just read the compressed file back in.
            String imageFilename = imageFilePrefix + "_IMG_" + fileDate + ".jpg";
            File imageFile = new File(picturesDir, imageFilename);

            Mat imgOriginal = Imgcodecs.imread(imageFile.getPath()); // default flag = IMREAD_COLOR

            RobotLog.dd(TAG, "Image height " + imgOriginal.height());
            RobotLog.dd(TAG, "Image width " + imgOriginal.width());

            // Define a region of interest. The ROI is defined as the part of the image that remains
            // after the crop area is taken off the top.
            int roiY = 0;
            if ((pMineralPosition == LCHSValues.MineralPosition.CENTER) && (cropPercentC > 0))
                roiY = (int) (imgOriginal.height() * (cropPercentC / 100.0));
            else if (((pMineralPosition == LCHSValues.MineralPosition.LEFT) || (pMineralPosition == LCHSValues.MineralPosition.RIGHT)) && (cropPercentLR > 0))
                roiY = (int) (imgOriginal.height() * (cropPercentLR / 100.0));
            if (roiY != 0) {
                Rect region_of_interest = new Rect(0, roiY, (int) imgOriginal.width(), (int) imgOriginal.height() - roiY);
                imgOriginal = new Mat(imgOriginal, region_of_interest); // crop to region of interest
                String roiFileName = picturesPath + "/" + imageFilePrefix + "_ROI_" + fileDate + ".jpg";
                RobotLog.dd(TAG, "Writing " + roiFileName);
                Imgcodecs.imwrite(roiFileName, imgOriginal);
            }

            // Convert the RGB image to HSV.
            Mat imgHSV = new Mat();
            Imgproc.cvtColor(imgOriginal, imgHSV, Imgproc.COLOR_BGR2HSV);
            String hsvFileName = picturesPath + "/" + imageFilePrefix + "_HSV_" + fileDate + ".jpg";
            RobotLog.dd(TAG, "Writing " + hsvFileName);
            Imgcodecs.imwrite(hsvFileName, imgHSV);

            // Remove noise by Gaussian blurring.
            Imgproc.GaussianBlur(imgHSV, imgHSV, new Size(gaussianBlurKernelSize, gaussianBlurKernelSize), 0);

            // Perform color thresholding.
            Mat imgThresholded = new Mat();
            Core.inRange(imgHSV, new Scalar(iLowH, iLowS, iLowV), new Scalar(iHighH, iHighS, iHighV), imgThresholded);

            // morphological opening (remove small objects from the foreground)
            Imgproc.erode(imgThresholded, imgThresholded,
                    Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));
            Imgproc.dilate(imgThresholded, imgThresholded,
                    Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));

            // morphological closing (fill small holes in the foreground)
            Imgproc.dilate(imgThresholded, imgThresholded,
                    Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));
            Imgproc.erode(imgThresholded, imgThresholded,
                    Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));

            // Identify the contours
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(imgThresholded, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            RobotLog.dd(TAG, "Number of gold contours : " + contours.size());

            int numberOfGoldMineralsFound = 0;
            if (contours.size() != 0) {

                // Found at least one gold blob of some sort.
                // Apply filters to eliminate false positives (especially from the gold minerals
                // in the crater).
                double largestArea = 0;
                MatOfPoint largestContour = null;
                Rect boundingRect = null;
                for (MatOfPoint contour : contours) {
                    double area = Imgproc.contourArea(contour, false);
                    RobotLog.dd(TAG, "Contour area " + area);

                    boundingRect = Imgproc.boundingRect(contour);
                    boundingRect = Imgproc.boundingRect(contour);
                    StringBuilder foundGold = new StringBuilder(
                            "Found gold contour at x " + boundingRect.x + ", y " + boundingRect.y + ", w "
                                    + boundingRect.width + ", h " + boundingRect.height + ", bounding rect area = "
                                    + boundingRect.area());
                    RobotLog.dd(TAG, foundGold.toString());

                    // Always draw a box around the gold blob.
                    RotatedRect rotatedRect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));

                    // Draw the contours in white
                    Point[] vertices = new Point[4];
                    rotatedRect.points(vertices);
                    MatOfPoint points = new MatOfPoint(vertices);
                    Imgproc.drawContours(imgThresholded, Arrays.asList(points), -1, new Scalar(255, 255, 255), 4);

                    // Test for minimum area, width to height ratio, and height to width ratio.
                    // Minimum area for gold mineral in the center position.
                    if ((pMineralPosition == LCHSValues.MineralPosition.CENTER) && (minAreaC > 0)) {
                        if (area < minAreaC) {
                            // Gold blob is too small.
                            RobotLog.dd(TAG, "False positive from center view: gold blob too small");
                            continue;
                        }
                    }

                    // Minimum area for gold mineral in the oblique (left and right) positions.
                    if (((pMineralPosition == LCHSValues.MineralPosition.LEFT) || (pMineralPosition == LCHSValues.MineralPosition.RIGHT)) && (minAreaLR > 0)) {
                        if (area < minAreaLR) {
                            // Gold blob is too small.
                            RobotLog.dd(TAG, "False positive from oblique view: gold blob too small");
                            continue;
                        }
                    }

                    // Maximum ratio of width to height.
                    if (maxRatioWH > 0) {
                        if (boundingRect.width / boundingRect.height > maxRatioWH) {
                            // Gold blob is too wide in relation to its height.
                            RobotLog.dd(TAG, "False positive: gold blob too wide");
                            continue;
                        }
                    }

                    // Maximum ratio of height to width.
                    if (maxRatioHW > 0) {
                        if (boundingRect.height / boundingRect.width > maxRatioWH) {
                            // Gold blob is too tall in relation to its width.
                            RobotLog.dd(TAG, "False positive: gold blob too tall");
                            continue;
                        }
                    }

                    // Passed all filters. Found a gold mineral.
                    numberOfGoldMineralsFound++;
                    retVal = true;
                    RobotLog.dd(TAG, "Found a gold mineral");
                    if (area > largestArea) {
                        largestArea = area;
                        largestContour = contour;
                    }
                }

                // Draw a circle in the center of the largest contour.
                if (largestContour != null) {

                    // Get the centroid of the rectangle
                    Moments moments = Imgproc.moments(largestContour);
                    Point centroid = new Point(moments.get_m10() / moments.get_m00(),
                            moments.get_m01() / moments.get_m00());

                    // Draw a black circle around the centroid.
                    Imgproc.circle(imgThresholded, centroid, 10, new Scalar(0, 0, 0), 4);
                }

                // Write the final version to a file.
                String goldFileName = picturesPath + "/" + imageFilePrefix + "_GOLD_" + fileDate + ".jpg";
                RobotLog.dd(TAG, "Writing " + goldFileName);
                Imgcodecs.imwrite(goldFileName, imgThresholded);

                RobotLog.dd(TAG, "Number of gold minerals found: " + numberOfGoldMineralsFound);
            }
        } catch (Exception error) {
            throw new AutonomousRobotException(TAG, "File error"
                    + error.getMessage());
        }

        //** At some point we may want to calculate the angle to the center of the gold object,
        //** See web links saved in OneNote/Technical.
        return retVal;
    }

    // Write a landscape bitmap out to a file.
    public void writeImageFileFromBitmap(Bitmap pBitmap, String pFileDate) {

        Bitmap landscapeBitmap = Bitmap.createBitmap(pBitmap);
        String imageFilename = imageFilePrefix + "_IMG_" + pFileDate + ".jpg";

        // Write the image out to a jpeg file.
        File imageFile = new File(picturesDir, imageFilename);
        try {
            FileOutputStream fos = new FileOutputStream(imageFile);
            landscapeBitmap.compress(Bitmap.CompressFormat.JPEG, 100, fos);
            fos.close();
            RobotLog.ii(TAG, "File " + imageFilename + " saved");
        } catch (Exception error) {
            throw new AutonomousRobotException(TAG, "File " + imageFilename + " not saved"
                    + error.getMessage());
        }
    }


}

