package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_GOLD_MINERAL;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_SILVER_MINERAL;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.TFOD_MODEL_ASSET;

public class LCHSAutoVisionTF {
    private static final String TAG = "LCHSAutoVisionTF";

    private LinearOpMode linearOpMode;
    private LCHSHardwareMap robot;
    private boolean debugVerbose = false;

    // Object for Vuforia.
    private LCHSAutoVuforia autoVuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    // double <the maximum ratio of the image width to height>
    // double <the maximum ratio of the image height to width>
    private static final double maxRatioWHTensorflow = 1.5;
    private static final double maxRatioHWTensorflow = 1.5;

    private TFObjectDetector tfod;

    public LCHSAutoVisionTF(LinearOpMode pLinear, LCHSHardwareMap pHardwareMap, LCHSAutoVuforia pAutoVuforia, boolean pDebugVerbose) {
        linearOpMode = pLinear;
        robot = pHardwareMap;
        autoVuforia = pAutoVuforia;
        debugVerbose = pDebugVerbose;

        /**
         * Initialize the Tensor Flow Object Detection engine.
         */
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            int tfodMonitorViewId = robot.hwMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", robot.hwMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, autoVuforia.getVuforiaForTensorflow());
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
            RobotLog.dd(TAG, "Tensorflow initialized");
        } else {
            RobotLog.dd(TAG, "Tensorflow not initialized");
            tfod = null;
        }
    }

    public void activate() {
        if (tfod != null)
            tfod.activate();
    }

    public void shutdown() {
        if (tfod != null)
            tfod.shutdown();
    }

//
// Need to change the algorithm for evalutaing gold and silver minerals after testing
// on 11/20/2018.

/*

Algorithm as of Competition 0, 17-Nov-2018:
 Collect a List of Tensorflow recognitions (themselves a List) until tfod.getUpdatedRecognitions() returns null
 or the loop count is exhausted.

 Assume the last List of recognitions is the most accurate. Loop through this List until we've isolated the silver
 or gold mineral with the smallest angle to the target. Minerals must pass a width to height and a height to width
 ratio filter which attempts to eliminate false positives. There may be no minerals of either or both types that are
 recognized and pass the filters. If, at the end of the evaluation, at least one mineral of each type is in contention,
 we take the mineral with the lowest angle to the target.

*/

/*

Analysis of logs from testing on 20-Nov-2018 where we missed the gold mineral in the center position even though we
started from the easier right (depot) position.

// 11-20 16:00:23.679  7026  7428 D LCHSAuto: Got 5 sets of recognitions
// 1. The first 4 recognitions see the gold with .95 to .99 confidence, the 5th (which failed W/H) only with .75
// None of the first 4 would have failed W/H ratio test.
// Failed width/height test, ratio = 1.65, limit = 1.5; 148 x 1.5 = 222
11-20 16:00:23.695  7026  7428 D LCHSAuto: Detected 1 objects
11-20 16:00:23.695  7026  7428 D LCHSAuto: Found Gold Mineral
11-20 16:00:23.695  7026  7428 D LCHSAuto: Image width 1280, Object left 576, right 822
11-20 16:00:23.695  7026  7428 D LCHSAuto: Top 576.6448364257812, Bottom 725.5516357421875
11-20 16:00:23.696  7026  7428 D LCHSAuto: Height 148.9068, Width 245.35248, Area 36534.65234375
11-20 16:00:23.696  7026  7428 D LCHSAuto: Angle 3.1543163517487818, Confidence 0.75390625
11-20 16:00:23.696  7026  7428 D LCHSAuto: Irregular image: blob too wide
11-20 16:00:23.696  7026  7428 D LCHSAuto: All recognitions filtered out, returning false

// 2. Failed to see the gold mineral in the center because there was a silver with a smaller angle smallest angle.
Why there was a silver in front of the depot is unclear; maybe the silver on the left-hand side is visible. Notice
that the silver mineral is out-of-range; the "top" is at -72.
11-20 16:39:02.728  9318  9460 D LCHSAuto: Detected 2 objects
11-20 16:39:02.728  9318  9460 D LCHSAuto: Found Silver Mineral
11-20 16:39:02.728  9318  9460 D LCHSAuto: Image width 720, Object left 117, right 276
11-20 16:39:02.729  9318  9460 D LCHSAuto: Top -72.72891235351562, Bottom 91.68339538574219
11-20 16:39:02.729  9318  9460 D LCHSAuto: Height 164.41231, Width 159.18097, Area 26171.310546875
11-20 16:39:02.729  9318  9460 D LCHSAuto: Angle -8.563528376948998, Confidence 0.85546875

// Looks like the gold mineral was at the LH edge of the frame (left = 8) but the confidence level is .97
11-20 16:39:02.729  9318  9460 D LCHSAuto: Found Gold Mineral
11-20 16:39:02.729  9318  9460 D LCHSAuto: Image width 720, Object left 8, right 176
11-20 16:39:02.729  9318  9460 D LCHSAuto: Top 669.638916015625, Bottom 842.4877319335938
11-20 16:39:02.729  9318  9460 D LCHSAuto: Height 172.84882, Width 167.34875, Area 28926.033203125
11-20 16:39:02.729  9318  9460 D LCHSAuto: Angle -13.89762305972343, Confidence 0.97265625
11-20 16:39:02.729  9318  9460 D LCHSAuto: Mineral with the smallest angle is silver 8.563528376948998

*/

/*

New algorithm starting 24-Nov-2018:
Take all Lists of recognitions into account. Keep track of the qualifying gold and silver minerals with the
highest confidence level. Qualify minerals based on "top" and "left" - all must be within the image area -
and their width to height and height to width ratios. If, at the end of the evaluation, at least one mineral
of each type is in contention, take the mineral with the highest confidence level.

*/

    // Use Tensorflow to find the gold mineral.
// The sample uses a camera frame that includes all 3 minerals. We can't place
// the camera far back enough to achieve this. So we have to deal with frames
// that may include more than one mineral. This means that we have to filter
// out false positives (e.g. we're pointing at a silver mineral but there is
// a gold mineral to the side or behind) and false negatives (e.g. we're pointing
// at a gold mineral but there is a silver mineral to the side or behind). See the
// filters below.
    public boolean findGoldMineralTensorflow() {

        RobotLog.dd(TAG, "In findGoldMineralTensorflow()");

        if (tfod == null) {
            RobotLog.dd(TAG, "findGoldMineralTensorflow(): tensorflow not initialized");
            return false;
        }

        //** WOULD this work??
        // CameraDevice.getInstance().stop(); // after TensorFlow initialization
        // CameraDevice.getInstance().start();

        List<Recognition> updatedRecognitions = null;
        List<List<Recognition>> allRecognitions = new ArrayList<>();
        boolean gotOneRecognition = false;
        int tIndex;
        for (tIndex = 0; tIndex < 10; tIndex++) {
            if (!linearOpMode.opModeIsActive()) {
                RobotLog.dd(TAG, "In findGoldMiineralTensorflow(): opMode is inactive, returning false");
                //tfod.shutdown();
                return false;
            }

            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions == null && !gotOneRecognition) {
                RobotLog.dd(TAG, "updatedRecognitions is null at loop count " + tIndex);
                android.os.SystemClock.sleep(250);
                continue;
            }

            // In a previous iteration we got at least one recognition but now getUpdatedRecognitions returned null.
            // We're done.
            if (updatedRecognitions == null && gotOneRecognition) {
                break;
            }

            if (updatedRecognitions.size() == 0) {
                RobotLog.dd(TAG, "updatedRecognitions is empty at loop count " + tIndex);
                android.os.SystemClock.sleep(250);
                continue;
            }

            // Got a list of recognitions; add it to the collection and keep going.
            gotOneRecognition = true;
            allRecognitions.add(updatedRecognitions);
            android.os.SystemClock.sleep(100);
        } // for

        if (!gotOneRecognition) {
            RobotLog.dd(TAG, "No recognitions found");
            RobotLog.dd(TAG, "Returning false from findGoldMineralTensorflow() with loop index " + tIndex);
            return false;
        }

        // Analyze all recognitions.
        RobotLog.dd(TAG, "Finished looking for sets of recognitions with loop index " + tIndex);
        int allRecognitionsSize = allRecognitions.size();
        RobotLog.dd(TAG, "Got " + allRecognitionsSize + " sets of recognitions");

        // Got a list of recognitions to work through.
        // Fields for logging and filtering:
        String label = null;
        int imageWidth = 0;
        int imageHeight = 0;
        int left = 0;
        int right = 0;
        double angle = 0.0d;
        double top = 0.0d;
        double bottom = 0.0d;
        float height = 0.0f;
        float width = 0.0f;
        float confidence = 0.0f;
        double area = 0.0d;

        int totalGoldMinerals = 0;
        int totalGoldQualified = 0;
        double greatestQualifiedGoldConfidence = 0.0d;
        int greatestQualifiedGoldConfidenceIndex = 0;

        int totalGoldDisqualified = 0;
        double greatestDisqualifiedGoldConfidence = 0.0d;
        int greatestDisqualifiedGoldConfidenceIndex = 0;

        int totalSilverMinerals = 0;
        int totalSilverQualified = 0;
        double greatestQualifiedSilverConfidence = 0.0d;
        int greatestQualifiedSilverConfidenceIndex = 0;

        int totalSilverDisqualified = 0;
        double greatestDisqualifiedSilverConfidence = 0.0d;
        int greatestDisqualifiedSilverConfidenceIndex = 0;

        // Also keep track of disqualified recognitions, i.e. those that were filtered out,
        // in case we come to the end of the recogntion process with no qualified recognitions.
        for (int rInx = 0; rInx < allRecognitions.size(); rInx++) {
            RobotLog.dd(TAG, "Tensorflow recognitons at index " + rInx);
            RobotLog.dd(TAG, "Detected " + allRecognitions.get(rInx).size() + " objects");
            for (Recognition recognition : allRecognitions.get(rInx)) {
                label = recognition.getLabel();
                if (label.equals(LABEL_GOLD_MINERAL) || (label.equals(LABEL_SILVER_MINERAL))) {
                    imageWidth = recognition.getImageWidth();
                    imageHeight = recognition.getImageHeight();
                    left = (int) recognition.getLeft();
                    right = (int) recognition.getRight();
                    angle = recognition.estimateAngleToObject(AngleUnit.DEGREES);
                    top = recognition.getTop();
                    bottom = recognition.getBottom();
                    height = recognition.getHeight();
                    width = recognition.getWidth();
                    area = height * width;
                    confidence = recognition.getConfidence();

                    RobotLog.dd(TAG, "Found " + label);
                    RobotLog.dd(TAG, "Image width " + imageWidth + ", Object left " + left + ", right " + right);
                    RobotLog.dd(TAG, "Image height " + imageHeight + ", Object top " + top + ", bottom " + bottom);
                    RobotLog.dd(TAG, "Object height " + height + ", width " + width + ", area " + area);
                    RobotLog.dd(TAG, "Angle " + angle + ", confidence " + confidence);

                    // Apply secondary filters on the size of the mineral(s) found.
                    if (label.equals(LABEL_GOLD_MINERAL)) {
                        totalGoldMinerals++; // count all before applying filters

                        // Make sure the image is in bounds on the left and the top. As noted during testing, the values
                        // of "right" and "bottom" can exceed the image size!?
                        if ((left < 0) || (height < 0)) {
                            // Object is out of bounds.
                            totalGoldDisqualified++;
                            if (confidence > greatestDisqualifiedGoldConfidence) {
                                greatestDisqualifiedGoldConfidence = confidence;
                                greatestDisqualifiedGoldConfidenceIndex = rInx;
                            }

                            RobotLog.dd(TAG, "Object out of bounds");
                            continue;
                        }

                        // Test for width to height ratio, and height to width ratio.
                        // Maximum ratio of width to height.
                        if (maxRatioWHTensorflow > 0) {
                            if (width / height > maxRatioWHTensorflow) {
                                // Object is too wide in relation to its height.
                                totalGoldDisqualified++;
                                if (confidence > greatestDisqualifiedGoldConfidence) {
                                    greatestDisqualifiedGoldConfidence = confidence;
                                    greatestDisqualifiedGoldConfidenceIndex = rInx;
                                }

                                RobotLog.dd(TAG, "Irregular object: too wide");
                                continue;
                            }
                        }

                        // Maximum ratio of height to width.
                        if (maxRatioHWTensorflow > 0) {
                            if (height / width > maxRatioWHTensorflow) {
                                // Object is too tall in relation to its width.
                                totalGoldDisqualified++;
                                if (confidence > greatestDisqualifiedGoldConfidence) {
                                    greatestDisqualifiedGoldConfidence = confidence;
                                    greatestDisqualifiedGoldConfidenceIndex = rInx;
                                }

                                RobotLog.dd(TAG, "Irregular object: too tall");
                                continue;
                            }
                        }

                        // If we made it here then we've got a qualified gold mineral.
                        totalGoldQualified++;
                        if (confidence > greatestQualifiedGoldConfidence) {
                            greatestQualifiedGoldConfidence = confidence;
                            greatestQualifiedGoldConfidenceIndex = rInx;
                        }
                        continue;
                    } // Got a gold mineral


                    // Hang on to the silver mineral with highest confidence.
                    if (label.equals(LABEL_SILVER_MINERAL)) {
                        totalSilverMinerals++; // count all before applying filters

                        // Make sure the image is in bounds on the left and the top. As noted during testing, the values
                        // of "right" and "bottom" can exceed the image size!?
                        if ((left < 0) || (height < 0)) {
                            // Object is out of bounds.
                            totalSilverDisqualified++;
                            if (confidence > greatestDisqualifiedSilverConfidence) {
                                greatestDisqualifiedSilverConfidence = confidence;
                                greatestDisqualifiedSilverConfidenceIndex = rInx;
                            }

                            RobotLog.dd(TAG, "Object out of bounds");
                            continue;
                        }

                        // Test for width to height ratio, and height to width ratio.
                        // Maximum ratio of width to height.
                        if (maxRatioWHTensorflow > 0) {
                            if (width / height > maxRatioWHTensorflow) {
                                // Object is too wide in relation to its height.
                                totalSilverDisqualified++;
                                if (confidence > greatestDisqualifiedSilverConfidence) {
                                    greatestDisqualifiedSilverConfidence = confidence;
                                    greatestDisqualifiedSilverConfidenceIndex = rInx;
                                }

                                RobotLog.dd(TAG, "Irregular object: too wide");
                                continue;
                            }
                        }

                        // Maximum ratio of height to width.
                        if (maxRatioHWTensorflow > 0) {
                            if (height / width > maxRatioWHTensorflow) {
                                // Object is too tall in relation to its width.
                                totalSilverDisqualified++;
                                if (confidence > greatestDisqualifiedSilverConfidence) {
                                    greatestDisqualifiedSilverConfidence = confidence;
                                    greatestDisqualifiedSilverConfidenceIndex = rInx;
                                }

                                RobotLog.dd(TAG, "Irregular object: too tall");
                                continue;
                            }
                        }

                        // If we made it here then we've got a qualified gold mineral.
                        totalSilverQualified++;
                        if (confidence > greatestQualifiedSilverConfidence) {
                            greatestQualifiedSilverConfidence = confidence;
                            greatestQualifiedSilverConfidenceIndex = rInx;
                        }

                        continue;
                    } // Got a silver mineral
                } // got a gold or silver mineral
            } // for (one List of actual recognitions)
        } // for (all Lists of recognitions)


        // Now analyze the results of the filtering.
        // If only one type of mineral was recognized then we have to go with that one.
        //** This filter allows recognition sets with only unqualified gold or silver minerals
        // to pass. Consider changing the filters here and below to default to silver if no
        // qualified minerals are found.
        if ((totalGoldMinerals != 0) && (totalSilverMinerals == 0)) {
            RobotLog.dd(TAG, "Found only " + totalGoldMinerals + " gold mineral(s)");
            return true;
        }

        if ((totalGoldMinerals == 0) && (totalSilverMinerals != 0)) {
            RobotLog.dd(TAG, "Found only " + totalSilverMinerals + " silver mineral(s)");
            return false;
        }

        // Now look at minerals that passed all the filters.
        // If we have only gold or only silver report the results accordingly.
        if ((totalGoldQualified != 0) || (totalSilverQualified != 0)) {
            if ((totalGoldQualified != 0) && (totalSilverQualified == 0)) {
                RobotLog.dd(TAG, "Found qualified gold minerals only in " + totalGoldQualified + " set(s) of recognitions");
                return true;
            }

            if ((totalGoldQualified == 0) && (totalSilverQualified != 0)) {
                RobotLog.dd(TAG, "Found qualified silver minerals only in " + totalSilverQualified + " set(s) of recognition");
                return false;
            }

            // We have minerals of both types. Compare their confidence levels.
            if (greatestQualifiedGoldConfidence >= greatestQualifiedSilverConfidence) {
                RobotLog.dd(TAG, "Mineral with the greatest confidence is gold: " + greatestQualifiedGoldConfidence);
                RobotLog.dd(TAG, "In outer List of recognitions at index " + greatestQualifiedGoldConfidenceIndex);
                return true;
            } else {
                RobotLog.dd(TAG, "Mineral with the greatest confidence is silver: " + greatestQualifiedSilverConfidence);
                RobotLog.dd(TAG, "In outer List of recognitions at index " + greatestQualifiedSilverConfidenceIndex);
                return false;
            }
        }

        // Now we have only zero or more disqualified minerals of each type.
        // If only one type of mineral was recognized then we have to go with that one.
        if ((totalGoldDisqualified != 0) || (totalSilverDisqualified != 0)) {
            if ((totalGoldDisqualified != 0) && (totalSilverDisqualified == 0)) {
                RobotLog.dd(TAG, "Found only " + totalGoldDisqualified + " disqualified gold mineral(s)");
                return true;
            }

            if ((totalGoldDisqualified == 0) && (totalSilverDisqualified != 0)) {
                RobotLog.dd(TAG, "Found only " + totalSilverDisqualified + " disqualified silver mineral(s)");
                return false;
            }

            // We have both disqualified gold and disqualified silver minerals.
            // All we can do is compare their confidence levels.
            if (greatestDisqualifiedGoldConfidence >= greatestDisqualifiedSilverConfidence) {
                RobotLog.dd(TAG, "Disqualified mineral with the greatest confidence is gold: " + greatestDisqualifiedGoldConfidence);
                RobotLog.dd(TAG, "In outer List of recognitions at index " + greatestDisqualifiedGoldConfidenceIndex);
                return true;
            } else {
                RobotLog.dd(TAG, "Disqualified mineral with the greatest confidence is silver: " + greatestDisqualifiedSilverConfidence);
                RobotLog.dd(TAG, "In outer List of recognitions at index " + greatestDisqualifiedSilverConfidenceIndex);
                return false;
            }
        }

        // We should never reach here!!
        RobotLog.dd(TAG, "Programming error: all recognitions filtered out, returning false");
        return false;
    }

}
