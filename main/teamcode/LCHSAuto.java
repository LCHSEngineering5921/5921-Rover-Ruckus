package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.Vuforia;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.opencv.android.OpenCVLoader;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_GOLD_MINERAL;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_SILVER_MINERAL;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.TFOD_MODEL_ASSET;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;


public class LCHSAuto {

    private static final String TAG = "LCHSAuto";
    private static final SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMddHHmmss");

    private LinearOpMode linearOpMode;
    private Configuration config;
    private LCHSValues.MineralPosition foundGoldPosition = LCHSValues.MineralPosition.UNKNOWN;

    private LCHSValues.OpMode autoOpMode;
    private boolean debugVerbose = false;

    // Set up all devices.
    private LCHSHardwareMap robot;

    private enum DriveMode {STRAIGHT, STRAFE}

    private double desiredHeading = 0.0; // always normalized

    public static final double COUNTS_PER_MOTOR_REV = 1120;
    public static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    public static final double WHEEL_DIAMETER_INCHES = 4.0;
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private static final double DRIVE_POWER = 0.5; // default power for straight runs and strafing
    private static final double MINIMUM_DRIVE_POWER = 0.3; // minimum power to turn the wheels

    // Turn constants
    private static final double TURN_POWER = 0.3; // 11/13/18 was 0.5;  // Nominal half speed for better accuracy.
    private static final double MINIMUM_TURN_POWER = 0.2; // 11/14/18 was 0.3; // floor for power to the motors
    private static final double TURN_INNER_THRESHOLD_DEGREES = 1.0; // As tight as we can make it with an integer gyro
    private static final double TURN_OUTER_THRESHOLD_DEGREES_FACTOR = 15.0;
    private static final double TURN_OUTER_THRESHOLD_DEGREES = 2.0;
    private static final double TURN_MAXIMUM_STALL_TIME = 5000; // milliseconds
    private static final double TURN_OUTER_TARGET_STALL_TIME = 1500; // milliseconds
    private static final double CENTER_SKEW_THRESHOLD_DEGREES = 3.0;

    // PID constants
    private static final double P_DRIVE_COEFF = 0.07; // 2017-18 0.1;     // Larger is more responsive, but also less stable
    private static final double P_TURN_COEFF = 0.05;  // 11/13/18 was 0.2;      // Larger is more responsive, but also less stable

    // Vuforia and Tensorflow variables from sample ConceptTensorFlowObjectDetection
    private static final String VUFORIA_KEY = "\"AblqOtv/////AAAAGdNKXPkjgkKGpGIKO9aGT7wPTFlXhki/3S+80P7yBeTE37kLJOvzyL57aik4RAD6LXyHPDRMdo4PJnxwaCzQj1+8CxEv1Z/dTr1IcjnZmsGC1d6zui4rxx1docFTjjpebkimp6A95reel2toZqogkexy/txx0KPHyF09E6jzZSV9AjFlexbSLe1/SfqoMa4TRLIBgWK3/Sj/TYY/b0WAwJOJYnFoqZAtvEyasqnBG9wulo/A6UKtKmRxmiWQ6QY/x2NAi4giPONyaWRAvFvSWUQjRkTFzCc9npy10RMX4qHZK0ow79YfMnyZqnRwHAR4Fp4HzPgdnmlf0yK/lQ3jxCLKT9cDdhNcayDeOrx6Y197";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;
    private List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    // double <the maximum ratio of the image width to height>
    // double <the maximum ratio of the image height to width>
    private static final double maxRatioWHTensorflow = 1.5;
    private static final double maxRatioHWTensorflow = 1.5;

    private TFObjectDetector tfod;

    // Initialize the OpenCV library.
    private static boolean openCVInitialized = false;

    static {
        if (OpenCVLoader.initDebug())
            openCVInitialized = true;
    }

    // Default for the vision system to use. Can be set in the configuration file.
    private LCHSValues.MineralVisionSystem mineralVision = LCHSValues.MineralVisionSystem.TENSORFLOW;

    // For synchronization with the tilt thread.
    Lock tiltLock = new ReentrantLock();
    Condition tiltCondition = tiltLock.newCondition();
    volatile boolean tiltMoveComplete = false;

    // Separate thread fpr the block lifter motor.
    private Thread tiltThread;


    // LCHSAuto must only be instantiated from runOpMode() in each autonomous opmode
    // because the hardwareMap is not initialized until then.
    public LCHSAuto(LCHSValues.OpMode pOpMode, LinearOpMode pLinear) {

        RobotLog.dd(TAG, "Autonomous OpMode: " + pOpMode);
        linearOpMode = pLinear; // get access to LinearOpMode public fields and methods
        autoOpMode = pOpMode;

        // Read the configuration file for the selected opmode.
        config = new Configuration(autoOpMode);
        mineralVision = config.getMineralVisionSystem();

        // Initialize the hardware.
        robot = new LCHSHardwareMap(pLinear.hardwareMap, true);

        robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer,
        // so initialize both.
        if (mineralVision == LCHSValues.MineralVisionSystem.TENSORFLOW) {
            initTensorFlow();

            // A failure in Tensorflow initialization will prevent us from recognizing
            // the gold mineral sample, but do not treat this as fatal.
            if (tfod == null)
                RobotLog.dd(TAG, "Failure in Tensorflow initialization");
        } else {
            initVuforiaOpenCV();

            // A failure in OpenCV initialization will prevent us from recognizing
            // the gold mineral sample, but do not treat this as fatal.
            if (!openCVInitialized)
                RobotLog.dd(TAG, "Failure in OpenCV initialization");
        }
    }

    // NOTE: from the ftc documentation - "Please do not swallow the InterruptedException,
    // as it is used in cases where the op mode needs to be terminated early."
    public void runRobot() throws InterruptedException {

        RobotLog.dd(TAG, "At start");
        RobotLog.dd(TAG, "Using " + mineralVision + " vision system for mineral detection");

        // Safety check against ftc runtime initialization errors.
        // Make sure the opmode is still active.
        if (!linearOpMode.opModeIsActive())
            throw new AutonomousRobotException(TAG, "OpMode unexpectedly inactive in runRobot()");

        double currentHeading = getIMUHeading();
        RobotLog.dd(TAG, "Desired heading: " + desiredHeading + ", Current heading: " + currentHeading);

        // Activate vision system for mineral detection.
        if (mineralVision == LCHSValues.MineralVisionSystem.TENSORFLOW) {
            if (tfod != null)
                tfod.activate();
        } else { // OPENCV
            //** MAY NEED THIS
            //     CameraDevice.getInstance().setFlashTorchMode(true);
        }

        // Follow the choreography specified in the configuration file.
        switch (autoOpMode) {
            // Process the commands (steps in a choreography) as they occur in the configuration file.
            case TEST:
                debugVerbose = true; // extra logging in test mode
            case RIGHT_SOLO:
            case LEFT_SOLO:
            case RIGHT_COOP:
            case LEFT_COOP: {
                doAutoOpMode(config.getStepsInChoreography());
                break;
            }
            default: {
                throw new AutonomousRobotException(TAG, "Unrecognized OpMode: " + autoOpMode);
            }
        }

        // DONE
        if (mineralVision == LCHSValues.MineralVisionSystem.TENSORFLOW) {
            if (tfod != null) {
                tfod.shutdown();
            }
        } else { // OPENCV
            //** MAY NEED THIS FOR OpenCV; put at the end of sampling?
            //     CameraDevice.getInstance().setFlashTorchMode(false);
        }

        RobotLog.dd(TAG, "Exiting LCHSAuto");
        linearOpMode.telemetry.addData("LCHSAuto ", "complete");
        linearOpMode.telemetry.update();
    }


    //===============================================================================================
    //===============================================================================================

    // Generic autonomous OpMode handler.
    // Process the commands (steps in a choreography) as they occur in the configuration file.
    // As needed, maintain context between commands, e.g. see targetJewelDirection.
    private void doAutoOpMode(Configuration.StepsInChoreography pSteps) throws InterruptedException {
        List<Configuration.Step> steps = pSteps.getSteps();

        // local variables needed  for context between steps.
        // int targetJewelDirection = 0;

        for (Configuration.Step currentStep : steps) {

            // Make sure the opmode is still active.
            if (!linearOpMode.opModeIsActive())
                throw new AutonomousRobotException(TAG, "OpMode went inactive");

            Configuration.Command command = currentStep.getCommand();
            RobotLog.dd(TAG, "Autonomous command " + command);
            List<String> parameters = currentStep.getParameters();
            String parameter;

            switch (command) {

                // ----- Move the robot in a straight line without gyro correction -------
                case STRAIGHT: {
                    if (parameters.size() != 2)
                        throw new AutonomousRobotException(TAG, "Incorrect number of parameters");

                    double driveDistance = Double.parseDouble(parameters.get(0));
                    double drivePower = Double.parseDouble(parameters.get(1));
                    RobotLog.dd(TAG, "Drive straight without gyro: " + driveDistance + " inches, power: " + drivePower);
                    RobotLog.dd(TAG, "Desired gyro heading " + desiredHeading);
                    RobotLog.dd(TAG, "Current gyro heading " + getIMUHeading());

                    driveRobotWithoutGyro(DriveMode.STRAIGHT, driveDistance, drivePower);
                    RobotLog.dd(TAG, "Straight done");
                    RobotLog.dd(TAG, "Current gyro heading " + getIMUHeading());
                    break;
                }

                // ----- Move the robot in a straight line with gyro correction -------
                case STRAIGHT_GYRO: {
                    if (parameters.size() != 2)
                        throw new AutonomousRobotException(TAG, "Incorrect number of parameters");

                    double driveDistance = Double.parseDouble(parameters.get(0));
                    double drivePower = Double.parseDouble(parameters.get(1));
                    RobotLog.dd(TAG, "Drive straight with gyro: " + driveDistance + " inches, power: " + drivePower);
                    RobotLog.dd(TAG, "Desired gyro heading " + desiredHeading);
                    RobotLog.dd(TAG, "Current gyro heading " + getIMUHeading());

                    driveRobotWithGyro(DriveMode.STRAIGHT, driveDistance, drivePower, desiredHeading);
                    break;
                }

                // ----- Strafe the robot without gyro correction -------
                // Negative distance for a strafe left.
                case STRAFE: {
                    if (parameters.size() != 2)
                        throw new AutonomousRobotException(TAG, "Incorrect number of parameters");

                    double driveDistance = Double.parseDouble(parameters.get(0));
                    double drivePower = Double.parseDouble(parameters.get(1));
                    RobotLog.dd(TAG, "Strafe without gyro: " + driveDistance + " inches, power: " + drivePower);
                    RobotLog.dd(TAG, "Desired gyro heading " + desiredHeading);
                    RobotLog.dd(TAG, "Current gyro heading " + getIMUHeading());

                    driveRobotWithoutGyro(DriveMode.STRAFE, driveDistance, drivePower);
                    break;
                }

                // ----- Strafe the robot with gyro correction -------
                /*
                case STRAFE_GYRO: {
                    if (parameters.size() != 2)
                        throw new AutonomousRobotException(TAG, "Incorrect number of parameters");

                    double driveDistance = Double.parseDouble(parameters.get(0));
                    double drivePower = Double.parseDouble(parameters.get(1));
                    RobotLog.dd(TAG, "Strafe with gyro: " + driveDistance + " inches, power: " + drivePower);
                    RobotLog.dd(TAG, "Desired gyro heading " + desiredHeading);
                    RobotLog.dd(TAG, "Current gyro heading " + getIMUHeading());

                    driveRobotWithGyro(DriveMode.STRAFE, driveDistance, drivePower, desiredHeading);
                    break;
                }
                */

                // -------------- Turn the robot --------------
                // This turn always uses the shortest distance to the target, even if the actual
                // turn direction ends up being the opposite of that requested.
                case TURN: {
                    if (parameters.size() != 1)
                        throw new AutonomousRobotException(TAG, "Incorrect number of parameters");

                    double turnDegrees = Double.parseDouble(parameters.get(0));
                    RobotLog.dd(TAG, "Turn " + turnDegrees + " degrees");
                    RobotLog.dd(TAG, "Desired heading before turn " + desiredHeading);
                    RobotLog.dd(TAG, "Current heading before turn " + getIMUHeading());

                    desiredHeading = DEGREES.normalize(desiredHeading + turnDegrees); // set target heading
                    RobotLog.dd(TAG, "Desired heading after turn " + desiredHeading);

                    gyroTurn(desiredHeading);
                    break;
                }

                // Turn the robot in the requested direction even if it is not the shortest distance
                // to the target.
                // 26-Nov-2018 Test program on Winfows Visual Studio 2017 AngleCalculator
                // has been verified to on the test cases:
                // 0 -180 1.0 (normalized 179 CCW, unnormalized -181 CW)
                // 0 180 -1.0 (normalized -179 CW, unnormalized 181 CW)
                case TURN_REQUESTED_DIRECTION: {
                    if (parameters.size() != 2)
                        throw new AutonomousRobotException(TAG, "Incorrect number of parameters");

                    double turnDegrees = Double.parseDouble(parameters.get(0));
                    double turnPower = Double.parseDouble(parameters.get(1));
                    double desiredHeadingBeforeTurn = desiredHeading;
                    double currentHeading = getIMUHeading();
                    RobotLog.dd(TAG, "Turn " + turnDegrees + " degrees");
                    RobotLog.dd(TAG, "Desired heading before turn " + desiredHeading);
                    RobotLog.dd(TAG, "Current heading before turn " + currentHeading);

                    desiredHeading = DEGREES.normalize(desiredHeading + turnDegrees); // set target heading
                    RobotLog.dd(TAG, "Projected normalized desired heading after turn " + desiredHeading);
                    double normalizedTurn = getError(desiredHeading, currentHeading, true);
                    RobotLog.dd(TAG, "Normalized turn " + normalizedTurn);

                    // Check if the direction of the requested turn is the same as the normalized direction.
                    // The sign of the normalized turn determines the direction of its turn.
                    double unnormalizedTurn = getError(desiredHeadingBeforeTurn + turnDegrees, currentHeading, false);
                    RobotLog.dd(TAG, "Unnormalized turn " + unnormalizedTurn);

                    // Now make an adjustment to the desired heading depending on the signs of the
                    // requested turn and the normalized turn.
                   // if ((turnDegrees != 0) && (normalizedTurn != 0) && (unnormalizedTurn != 0) && ((normalizedTurn * unnormalizedTurn) < 0.0f)) {
                        //** ALWAYS Use unnormalized desired heading.
                        RobotLog.dd(TAG, "Turn in requested direction " + unnormalizedTurn);
                        gyroTurnFull(turnPower, P_TURN_COEFF, desiredHeading, false);
                   // } else {
                    //    RobotLog.dd(TAG, "Turn in requested direction = normalized turn");
                    //    gyroTurnFull(turnPower, P_TURN_COEFF, desiredHeading, true);
                    // }

                    break;
                }

                // -------------- Turn the robot with an angle, power level, and turn coefficient from the configuration file --------------
                case TURN_WITH_POWER: {
                    if (parameters.size() != 3)
                        throw new AutonomousRobotException(TAG, "Incorrect number of parameters");

                    double turnDegrees = Double.parseDouble(parameters.get(0));
                    double turnPower = Double.parseDouble(parameters.get(1));
                    double turnCoefficient = Double.parseDouble(parameters.get(2));
                    RobotLog.dd(TAG, "Turn " + turnDegrees + " degrees, power " + turnPower + ", coefficient " + turnCoefficient);
                    RobotLog.dd(TAG, "Desired heading before turn " + desiredHeading);
                    RobotLog.dd(TAG, "Current heading before turn " + getIMUHeading());

                    desiredHeading = DEGREES.normalize(desiredHeading + turnDegrees); // set target heading
                    RobotLog.dd(TAG, "Desired heading after turn " + desiredHeading);

                    gyroTurnFull(turnPower, turnCoefficient, desiredHeading, true);
                    break;
                }

                // Disengage and let the robot down off the rover.
                case LAND: {

                    robot.hookServo.setPosition(LCHSHardwareMap.HOOK_SERVO_OPEN);

                    while (robot.tilt.getCurrentPosition() < LCHSHardwareMap.TILT_POS_AFTER_GRAVITY) {
                        android.os.SystemClock.sleep(20);
                    }

                    robot.tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.tilt.setTargetPosition(LCHSHardwareMap.TILT_POS_ROBOT_IS_LEVEL);
                    robot.tilt.setPower(0.3);

                    while (Math.abs(robot.tilt.getCurrentPosition() - LCHSHardwareMap.TILT_POS_ROBOT_IS_LEVEL) > 50) {
                        android.os.SystemClock.sleep(20);
                    }

                    robot.tilt.setPower(0.0);
                    robot.tilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    RobotLog.dd(TAG, "Gyro heading after landing " + getIMUHeading());
                    break;
                }

                // Tilt the boom to its carrying position.
                case TILT: {
                    if (parameters.size() != 1)
                        throw new AutonomousRobotException(TAG, "Incorrect number of parameters");

                    int targetTiltPosition = Integer.parseInt(parameters.get(0));

                    // Tilt the boom in a separate thread.
                    tiltLock.lock();
                    tiltMoveComplete = false; // make sure the synchronization flag starts off false
                    tiltLock.unlock();

                    RobotLog.dd(TAG, "Starting TILT thread");
                    tiltThread = new TiltThread(targetTiltPosition);
                    tiltThread.start();
                    break;
                }

                // Wait for the Tilt thread to complete.
                case TILT_WAIT: {

                    tiltLock.lock();
                    while (!tiltMoveComplete)
                        tiltCondition.await();
                    tiltMoveComplete = false;
                    tiltLock.unlock();
                    RobotLog.dd(TAG, "Synchronized with tilt thread");
                    break;
                }

                case CLAIM: {
                    if (parameters.size() != 1)
                        throw new AutonomousRobotException(TAG, "Incorrect number of parameters");

                    int targetBoomPosition = Integer.parseInt(parameters.get(0));
                    robot.boom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.boom.setTargetPosition(targetBoomPosition);
                    robot.boom.setPower(0.5);

                    while (Math.abs(robot.boom.getCurrentPosition() - targetBoomPosition) > 50) {
                        android.os.SystemClock.sleep(20);
                    }

                    // Use flappers to spit out OR use gate to drop??
                    //robot.gateLeft.setPosition(LCHSHardwareMap.GATE_LEFT_SERVO_OPEN);
                    robot.intakeRight.setPower(-0.5);
                    robot.intakeLeft.setPower(-0.5); // used to be -0.8, lowered power so less throw

                    android.os.SystemClock.sleep(2000);

                    robot.intakeLeft.setPower(0.0);
                    robot.intakeRight.setPower(0.0);

                    break;
                }

                // Test the sampling of a mineral with OpenCV.
                case FIND_GOLD_OPENCV: {
                    parameter = parameters.get(0); // where to find gold: LEFT, CENTER, RIGHT

                    if (findGoldMineralOpenCV(LCHSValues.MineralPosition.valueOf(parameter))) {
                        linearOpMode.telemetry.addData(TAG, "Found a gold mineral");
                        RobotLog.dd(TAG, parameter + " position: found a gold mineral");
                    } else {
                        linearOpMode.telemetry.addData(TAG, "No gold mineral found");
                        RobotLog.dd(TAG, parameter + " position: no gold mineral found");
                    }
                    linearOpMode.telemetry.update();
                    break;
                }

                // Test the sampling of a mineral with Tensorflow.
                case FIND_GOLD_TENSORFLOW: {
                    if (findGoldMineralTensorflow()) {
                        linearOpMode.telemetry.addData(TAG, "Found a gold mineral");
                        RobotLog.dd(TAG, "Found a gold mineral");
                    } else {
                        linearOpMode.telemetry.addData(TAG, "No gold mineral found");
                        RobotLog.dd(TAG, "No gold mineral found");
                    }
                    linearOpMode.telemetry.update();
                    break;
                }

                // When the robot starts from either the red right or blue right positions
                // the closest corner of the field contains the depot.
                case SAMPLE_DEPOT: {
                    // Check that we've got the right number of parameters.
                    if (parameters.size() != (Configuration.mineralPositions.length * Configuration.depotStartParameters.length))
                        throw new AutonomousRobotException(TAG, "Missing expected parameter(s)");

                    // Find the gold mineral and turn towards the depot.
                    // sampleAndTurn returns the next parameter
                    int mineralParameterIndex = sampleAndTurn(currentStep);

                    // Approach the depot.
                    double approachDistance = Double.parseDouble(parameters.get(mineralParameterIndex++));
                    RobotLog.dd(TAG, "Approach depot distance " + approachDistance);

                    if (approachDistance != 0) {
                        driveRobotWithGyro(DriveMode.STRAIGHT, approachDistance, DRIVE_POWER, desiredHeading);
                        RobotLog.dd(TAG, "Approach done");
                    }

                    // Turn towards the depot for claim
                    double turnToDepotForClaim = Double.parseDouble(parameters.get(mineralParameterIndex++));
                    RobotLog.dd(TAG, "Turn to depot for claim " + turnToDepotForClaim);

                    if (turnToDepotForClaim != 0) {
                        desiredHeading = DEGREES.normalize(desiredHeading + turnToDepotForClaim); // set target heading
                        gyroTurn(desiredHeading);
                        RobotLog.dd(TAG, "Turn done");
                    }

                    break;
                }

                // When the robot starts from either the red left or blue left positions
                // the closest corner of the field contains the crater.
                case SAMPLE_CRATER: {
                    // Check that we've got the right number of parameters.
                    if (parameters.size() != (Configuration.mineralPositions.length * Configuration.craterStartParameters.length))
                        throw new AutonomousRobotException(TAG, "Missing expected parameter(s)");

                    // Find the gold mineral and turn towards the crater.
                    int mineralParameterIndex = sampleAndTurn(currentStep);

                    // Approach the crater by going straight until the gyro indicates that the robot
                    // has reached the target incline (pitch), i.e. the robot's wheels have started to
                    // climb the crater.
                    // Note: because of the position of the gyro on the robot, pitch is reported as roll.
                    double targetPitchAtCrater = Double.parseDouble(parameters.get(mineralParameterIndex++));
                    RobotLog.dd(TAG, "Expected pitch at the crater " + targetPitchAtCrater);

                    // Get the timeout in milliseconds from the configuration file.
                    // Sorry about this but the Configuration class expects a double so we have to convert here.
                    int inclineTimeout = (int) Math.round(Double.parseDouble(parameters.get(mineralParameterIndex++)));
                    if (targetPitchAtCrater != 0) {

                        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        robot.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        robot.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        RobotLog.dd(TAG, "Start drive towards crater");

                        // For debugging
                        double pitchBefore = robot.imu.getAngularOrientation().toAxesReference(INTRINSIC).toAxesOrder(ZYX).secondAngle;
                        linearOpMode.telemetry.addData("Pitch Before ", pitchBefore);
                        linearOpMode.telemetry.update();

                        double pitch = 0.0;
                        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
                        robot.leftFront.setPower(DRIVE_POWER);
                        robot.rightFront.setPower(DRIVE_POWER);
                        robot.leftBack.setPower(DRIVE_POWER);
                        robot.rightBack.setPower(DRIVE_POWER);

                        // Move towards the crater until the robot reaches the target pitch or the timer expires.
                        while (linearOpMode.opModeIsActive() && timer.time() < inclineTimeout && pitch <= targetPitchAtCrater) {
                            Orientation angles = robot.imu.getAngularOrientation().toAxesReference(INTRINSIC).toAxesOrder(ZYX);
                            pitch = angles.secondAngle;
                            RobotLog.dd(TAG, "IMU pitch " + pitch);
                            android.os.SystemClock.sleep(20); // don't overwhelm the imu
                        }

                        robot.leftFront.setPower(0);
                        robot.rightFront.setPower(0);
                        robot.leftBack.setPower(0);
                        robot.rightBack.setPower(0);

                        // For debugging
                        double pitchAfter = robot.imu.getAngularOrientation().toAxesReference(INTRINSIC).toAxesOrder(ZYX).secondAngle;
                        linearOpMode.telemetry.addData("Pitch After ", pitchAfter);
                        linearOpMode.telemetry.update();

                        RobotLog.dd(TAG, "Robot is parked on crater at pitch " + pitch);
                    }

                    break;
                }

                // After sampling in front of the crater, proceed to the depot and make a claim
                // with our marker.
                // DEPOT_REMOTE parameter identifiers for moving the robot from the crater to the depot.
                /* public static final String[] depotRemoteParameters = {"vuforia",
                        "reverseC", "%d", "%d", // distance in inches, power
                        "turnV", "%d", "%d", "%d", // angle, power, turn coefficient
                        "approachV", "%d", "%d", // distance in inches, power
                        "turnD", "%d", "%d", "%d", // angle, power, turn coefficient
                        "approachD", "%d", "%d"// distance in inches, power
                }; */
                case DEPOT_REMOTE: {
                    // Check that we've got the right number of parameters
                    if (parameters.size() != (Configuration.mineralPositions.length * Configuration.depotRemoteParameters.length))
                        throw new AutonomousRobotException(TAG, "Missing expected parameter(s)");

                    // Select which set of parameters to start with based on where the gold mineral
                    // was found.
                    if (foundGoldPosition == LCHSValues.MineralPosition.UNKNOWN) {
                        // Mineral recognition failed so abort the trip to the remote depot.
                        RobotLog.dd(TAG, "Aborting trip to the remote depot; stop autnomous run");
                        return;
                    }

                    // Set index for LEFT, CENTER, or RIGHT.
                    int mineralParameterIndex = Configuration.depotRemoteParameters.length * foundGoldPosition.ordinal();

                    // The "vuforia" parameter is a double but we'll use it as a boolean:
                    // 0 means do not use Vuforia for navigation, non-zero means use Vuforia.
                    boolean useVuforiaNavigation = false;
                    if (Double.parseDouble(parameters.get(mineralParameterIndex++)) != 0.0)
                        useVuforiaNavigation = true;

                    // From the crater back up towards the center position.
                    double reverseC = Double.parseDouble(parameters.get(mineralParameterIndex++));
                    double reverseCPower = Double.parseDouble(parameters.get(mineralParameterIndex++));
                    RobotLog.dd(TAG, "Reverse to center distance " + reverseC + " with power " + reverseCPower);

                    if (reverseC != 0) {
                        driveRobotWithGyro(DriveMode.STRAIGHT, reverseC, reverseCPower, desiredHeading);
                        RobotLog.dd(TAG, "Reverse done");
                    }

                    // Turn towards the Vuforia target.
                    double turnV = Double.parseDouble(parameters.get(mineralParameterIndex++));
                    double turnVPower = Double.parseDouble(parameters.get(mineralParameterIndex++));
                    double turnVCoefficient = Double.parseDouble(parameters.get(mineralParameterIndex++));
                    RobotLog.dd(TAG, "Turn towards the Vuforia target " + turnV + " degrees");
                    RobotLog.dd(TAG, "With power " + turnVPower + " and coeff " + turnVCoefficient);

                    if (turnV != 0) {
                        desiredHeading = DEGREES.normalize(desiredHeading + turnV); // set target heading
                        gyroTurnFull(turnVPower, turnVCoefficient, desiredHeading, true);
                        RobotLog.dd(TAG, "Turn done");
                    }

                    // Approach the Vuforia target.
                    double approachV = Double.parseDouble(parameters.get(mineralParameterIndex++));
                    double approachVPower = Double.parseDouble(parameters.get(mineralParameterIndex++));
                    RobotLog.dd(TAG, "Approach Vuforia target; distance " + approachV + ". power " + approachVPower);

                    if (approachV != 0) {
                        driveRobotWithGyro(DriveMode.STRAIGHT, approachV, approachVPower, desiredHeading);
                        RobotLog.dd(TAG, "Approach done");
                    }

                    //** If Vuforia navigation has been selected, here's where we use it to adjust our
                    // next turn and the length of our approach to the depot.
                    if (useVuforiaNavigation) {

                        OpenGLMatrix lastLocation = null;

                        for (VuforiaTrackable trackable : allTrackables) {
                            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                            if (robotLocationTransform != null) {

                                if ((trackable.getName().equals("Blue-Rover")) ||
                                        (trackable.getName().equals("Red-Footprint"))) {
                                    lastLocation = robotLocationTransform;
                                    RobotLog.dd(TAG, "Found Vuforia target " + trackable.getName());
                                    break;
                                }
                            }
                        }

                        if (lastLocation == null) {
                           RobotLog.dd(TAG, "Did not find the Vuforia target");
                            useVuforiaNavigation = false;
                        } else {


                                           // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                //telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                //        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
               // telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

                        }
                    }

                    // Turn towards the depot.
                    double turnD = Double.parseDouble(parameters.get(mineralParameterIndex++));
                    double turnDPower = Double.parseDouble(parameters.get(mineralParameterIndex++));
                    double turnDCoefficient = Double.parseDouble(parameters.get(mineralParameterIndex++));
                    RobotLog.dd(TAG, "Turn towards the depot " + turnD + " degrees");
                    RobotLog.dd(TAG, "With power " + turnDPower + " and coeff " + turnDCoefficient);

                    if (turnD != 0) {
                        desiredHeading = DEGREES.normalize(desiredHeading + turnD); // set target heading
                        gyroTurnFull(turnDPower, turnDCoefficient, desiredHeading, true);
                        RobotLog.dd(TAG, "Turn done");
                    }

                    // Approach the depot.
                    double approachD = Double.parseDouble(parameters.get(mineralParameterIndex++));
                    double approachDPower = Double.parseDouble(parameters.get(mineralParameterIndex++));
                    RobotLog.dd(TAG, "Approach depot; distance " + approachD + ". power " + approachDPower);

                    if (approachD != 0) {
                        driveRobotWithGyro(DriveMode.STRAIGHT, approachD, approachDPower, desiredHeading);
                        RobotLog.dd(TAG, "Approach done");
                    }

                    break;
                }

                // Pause the robot.
                // parameters: number of milliseconds to sleep (positive integer)
                case SLEEP: {
                    if (parameters.size() != 1)
                        throw new AutonomousRobotException(TAG, "Incorrect number of parameters");

                    int sleepValue = Integer.parseInt(parameters.get(0));
                    android.os.SystemClock.sleep(sleepValue);
                    break;
                }

                default: {
                    throw new AutonomousRobotException(TAG, "Unrecognized step in choreography");
                }
            }
        }
    }

    // Initialization for Vuforia when it will be used to deliver camera images
    // which we will then feed to OpenCV.
    private void initVuforiaOpenCV() {
        // Use this constructor if you want the camera monitor to be turned on.
        // int cameraMonitorViewId = robot.hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hwMap.appContext.getPackageName());
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        initVuforia();

        // Initialize for frame capture.
        vuforia.setFrameQueueCapacity(1);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
    }


    // Initialization for Tensorflow.
    // Taken from the FTC sample ConceptTensorFlowObjectDetection.
    //** Refactoring: sets "tfod" - but this could be a class field inside a TensorFlowRecognition class
    private void initTensorFlow() {

        RobotLog.dd(TAG, "In initTensorFlow()");

        initVuforia();

        /**
         * Initialize the Tensor Flow Object Detection engine.
         */
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            int tfodMonitorViewId = robot.hwMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", robot.hwMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
            RobotLog.dd(TAG, "Tensorflow initialized");
        } else {
            RobotLog.dd(TAG, "Tensorflow not initialized");
            tfod = null;
        }
    }


    //** Refactoring: creates the "vuforia" object.
    private void initVuforia() {
        final float mmPerInch = 25.4f;
        final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
        final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        int cameraMonitorViewId = robot.hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = BACK;
        //**parameters.fillCameraMonitorViewParent = false; //** TRY THIS - even default constructor shows the monitor

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        //List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        //** LATER - only load the trackables we need:
        // "Red-Footprint" and "Blue-Rover" - since we don't know our alliance (it's not encoded in the opmode).
        allTrackables.addAll(targetsRoverRuckus);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * This Rover Ruckus sample places a specific target in the middle of each perimeter wall.
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        /**
         * To place the BlueRover target in the middle of the blue perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Then, we translate it along the Y axis to the blue perimeter wall.
         */
        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        /**
         * To place the RedFootprint target in the middle of the red perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 180 around the field's Z axis so the image is flat against the red perimeter wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative Y axis to the red perimeter wall.
         */
        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        /**
         * To place the FrontCraters target in the middle of the front perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 90 around the field's Z axis so the image is flat against the front wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative X axis to the front perimeter wall.
         */
        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        /**
         * To place the BackSpace target in the middle of the back perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it -90 around the field's Z axis so the image is flat against the back wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the X axis to the back perimeter wall.
         */
        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        /**
         * Create a transformation matrix describing where the phone is on the robot.
         *
         * The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
         * pointing to the LEFT side of the Robot.  It's very important when you test this code that the top of the
         * camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip the phone.
         *
         * If using the rear (High Res) camera:
         * We need to rotate the camera around it's long axis to bring the rear camera forward.
         * This requires a negative 90 degree rotation on the Y axis
         *
         * If using the Front (Low Res) camera
         * We need to rotate the camera around it's long axis to bring the FRONT camera forward.
         * This requires a Positive 90 degree rotation on the Y axis
         *
         * Next, translate the camera lens to where it is on the robot.
         * In this example, it is centered (left to right), but 110 mm forward of the middle of the robot, and 200 mm above ground level.
         */

        final int CAMERA_FORWARD_DISPLACEMENT = 110;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, -90, 0, 0));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }
    }


    // Look for the gold mineral at each of the three sample positions.
    //** Refactoring: writes class field (enum) foundGoldPosition
    private int sampleAndTurn(Configuration.Step pStep) throws InterruptedException {

        Configuration.Command command = pStep.getCommand();
        List<String> parameters = pStep.getParameters();
        LCHSValues.MineralPosition lastSearchForMineral = LCHSValues.MineralPosition.CENTER;
        boolean foundGold = false;

        // Try to find the gold mineral in each of the three sample positions in the order
        // CENTER, LEFT, and RIGHT.
        // Remember that there are three sets of 4 parameters in the List of parameters: LEFT, CENTER,
        // and RIGHT. The starting index for each set is determined by the number of parameters for
        // each set. Start with center because the movements are the most straightforward.
        int itemsInParameterSet = 0;
        if (command == Configuration.Command.SAMPLE_DEPOT)
            itemsInParameterSet = Configuration.depotStartParameters.length;
        else
            itemsInParameterSet = Configuration.craterStartParameters.length;

        // Set the index into the List of parameters.
        int mineralParameterIndex = itemsInParameterSet * LCHSValues.MineralPosition.CENTER.ordinal();

        // Turn towards the CENTER sample.
        double turnDegreesC = Double.parseDouble(parameters.get(mineralParameterIndex));

        RobotLog.dd(TAG, "Turn towards CENTER mineral " + turnDegreesC);
        RobotLog.dd(TAG, "Desired heading before turn " + desiredHeading);
        RobotLog.dd(TAG, "Current heading before turn " + getIMUHeading());

        desiredHeading = DEGREES.normalize(desiredHeading + turnDegreesC); // set target heading
        RobotLog.dd(TAG, "Desired heading after turn " + desiredHeading);

        if (turnDegreesC != 0) {
            gyroTurn(desiredHeading);
            android.os.SystemClock.sleep(500);
        }

        // Even if the number of degrees to turn towards the center mineral is 0,
        // which will be the typical case, make sure that the robot is not skewed
        // from previous steps, e.g. a 180 turn.
        double degreeDifference = DEGREES.normalize(desiredHeading - getIMUHeading());
        if (Math.abs(degreeDifference) >= CENTER_SKEW_THRESHOLD_DEGREES) {
            RobotLog.dd(TAG, "De-skewing before CENTER mineral");
            RobotLog.dd(TAG, "Desired heading " + desiredHeading);
            RobotLog.dd(TAG, "Current heading before turn " + getIMUHeading());

            gyroTurn(desiredHeading);
            android.os.SystemClock.sleep(500); // settle for the camera
        }

        // At this point the robot should not be skewed but it may be shifted left or right.
        // Right now we just have to live with this.

        if (mineralVision == LCHSValues.MineralVisionSystem.OPENCV)
            foundGold = findGoldMineralOpenCV(LCHSValues.MineralPosition.CENTER);
        else foundGold = findGoldMineralTensorflow();

        if (!foundGold) { // gold at center?
            // No, turn towards the LEFT sample.
            mineralParameterIndex = itemsInParameterSet * LCHSValues.MineralPosition.LEFT.ordinal(); // should be 0
            double turnDegreesL = Double.parseDouble(parameters.get(mineralParameterIndex));
            RobotLog.dd(TAG, "Turn towards LEFT mineral " + turnDegreesL);
            RobotLog.dd(TAG, "Desired heading before turn " + desiredHeading);
            RobotLog.dd(TAG, "Current heading before turn " + getIMUHeading());

            desiredHeading = DEGREES.normalize(desiredHeading + turnDegreesL); // set target heading
            RobotLog.dd(TAG, "Desired heading after turn " + desiredHeading);

            if (turnDegreesL != 0) {
                gyroTurn(desiredHeading);
                android.os.SystemClock.sleep(500); // settle for the camera
            }

            lastSearchForMineral = LCHSValues.MineralPosition.LEFT;
            if (mineralVision == LCHSValues.MineralVisionSystem.OPENCV)
                foundGold = findGoldMineralOpenCV(LCHSValues.MineralPosition.LEFT);
            else foundGold = findGoldMineralTensorflow();

            if (!foundGold) {
                // Turn towards the RIGHT sample.
                mineralParameterIndex = itemsInParameterSet * LCHSValues.MineralPosition.RIGHT.ordinal();
                double turnDegreesR = Double.parseDouble(parameters.get(mineralParameterIndex));

                // Since we've already turned left we have to undo that turn.
                turnDegreesR += -turnDegreesL;
                RobotLog.dd(TAG, "Turn towards the RIGHT mineral " + turnDegreesR);
                RobotLog.dd(TAG, "Desired heading before turn " + desiredHeading);
                RobotLog.dd(TAG, "Current heading before turn " + getIMUHeading());

                desiredHeading = DEGREES.normalize(desiredHeading + turnDegreesR); // set target heading
                RobotLog.dd(TAG, "Desired heading after turn " + desiredHeading);

                if (turnDegreesR != 0) {
                    gyroTurn(desiredHeading);
                    android.os.SystemClock.sleep(500); // settle for the camera
                }

                lastSearchForMineral = LCHSValues.MineralPosition.RIGHT;
                if (mineralVision == LCHSValues.MineralVisionSystem.OPENCV)
                    foundGold = findGoldMineralOpenCV(LCHSValues.MineralPosition.RIGHT);
                else foundGold = findGoldMineralTensorflow();
            }
        }

        // If we haven't found the gold mineral anywhere that means we are pointing to the
        // RIGHT. Turn back to the CENTER position and just plow ahead.
        if (foundGold) {
            RobotLog.dd(TAG, "Found gold at " + lastSearchForMineral + " position");
            foundGoldPosition = lastSearchForMineral;
        } else {
            RobotLog.dd(TAG, "Did not find the gold mineral anywhere!");
            foundGoldPosition = LCHSValues.MineralPosition.UNKNOWN;

            // Since we've already turned RIGHT we have to undo that turn.
            double turnDegreesR = Double.parseDouble(parameters.get(mineralParameterIndex));
            turnDegreesR = -turnDegreesR;
            RobotLog.dd(TAG, "Turn back towards the CENTER mineral " + turnDegreesR);
            RobotLog.dd(TAG, "Desired heading before turn " + desiredHeading);
            RobotLog.dd(TAG, "Current heading before turn " + getIMUHeading());

            desiredHeading = DEGREES.normalize(desiredHeading + turnDegreesR); // set target heading
            RobotLog.dd(TAG, "Desired heading after turn " + desiredHeading);

            if (turnDegreesR != 0) {
                gyroTurn(desiredHeading);
            }

            mineralParameterIndex = itemsInParameterSet * LCHSValues.MineralPosition.CENTER.ordinal(); // reset to CENTER
        }

        // Now facing a (hopefully gold) mineral.
        // Make a "post recognition turn" to avoid other minerals.
        mineralParameterIndex++; // all cases: advance to postRT parameter
        double postRecognitionTurn = Double.parseDouble(parameters.get(mineralParameterIndex));
        RobotLog.dd(TAG, "Post recognition turn " + postRecognitionTurn);
        RobotLog.dd(TAG, "Turn " + postRecognitionTurn);
        RobotLog.dd(TAG, "Desired heading before turn " + desiredHeading);
        RobotLog.dd(TAG, "Current heading before turn " + getIMUHeading());

        desiredHeading = DEGREES.normalize(desiredHeading + postRecognitionTurn); // set target heading
        RobotLog.dd(TAG, "Desired heading after turn " + desiredHeading);

        if (postRecognitionTurn != 0) {
            gyroTurn(desiredHeading);
        }

        // Knock the gold mineral off its taped area.
        mineralParameterIndex++; // all cases: advance to knock_distance parameter
        double knockDistance = Double.parseDouble(parameters.get(mineralParameterIndex));
        RobotLog.dd(TAG, "Knock gold mineral " + knockDistance);

        driveRobotWithGyro(DriveMode.STRAIGHT, knockDistance, DRIVE_POWER, desiredHeading);
        RobotLog.dd(TAG, "Knocked off the gold mineral");

        // Turn towards the final target (depot or crater).
        mineralParameterIndex++; // advance to next parameter (turn angle)
        double turnDepotDegrees = Double.parseDouble(parameters.get(mineralParameterIndex));
        RobotLog.dd(TAG, "Turn " + turnDepotDegrees);
        RobotLog.dd(TAG, "Desired heading before turn " + desiredHeading);
        RobotLog.dd(TAG, "Current heading before turn " + getIMUHeading());

        desiredHeading = DEGREES.normalize(desiredHeading + turnDepotDegrees); // set target heading
        RobotLog.dd(TAG, "Desired heading after turn " + desiredHeading);

        if (turnDepotDegrees != 0) {
            gyroTurn(desiredHeading);
        }

        // All done with sampling. If we are using Tensorflow, shut down now.
        if ((mineralVision == LCHSValues.MineralVisionSystem.TENSORFLOW) && tfod != null) {
            tfod.shutdown();
            tfod = null;
        }

        return ++mineralParameterIndex; // advance to next parameter and return
    }

    /**
     * Method to drive on a fixed compass bearing based on encoder counts (RUN_TO_POSITION).
     * Move will stop if either of these conditions occur:
     * 1) Move reaches the desired position
     * 2) Driver stops the opmode running.
     *
     * @param pDistance Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param pPower    Positive power for forward motion.
     */
    // !!Note: The sign values of various parameters and variables in this method and which motors are
    // set as forward and reverse in the class initialization section above are interdependent.
    public void driveRobotWithoutGyro(DriveMode pDriveMode,
                                      double pDistance,
                                      double pPower) {

        int moveCounts;
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;
        double power;

        // Ensure that the opmode is still active
        RobotLog.dd(TAG, "Starting driveRobotWithoutGyro");
        if (!linearOpMode.opModeIsActive() || (pDistance == 0.0) || (pPower <= 0.0)) {
            RobotLog.dd(TAG, "Early exit from driveRobot: distance = " + pDistance + ", power = " + pPower);
            return;
        }

        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //RobotLog.dd(TAG, "driveRobotWithoutGyro after RESET");
        //RobotLog.dd(TAG, "Left front motor position " + robot.leftFront.getCurrentPosition());
        // RobotLog.dd(TAG, "Left back motor position " + robot.leftBack.getCurrentPosition());
        // RobotLog.dd(TAG, "Right front motor position " + robot.rightFront.getCurrentPosition());
        // RobotLog.dd(TAG, "Right back motor position " + robot.rightBack.getCurrentPosition());

        // Determine the new target position in encoder counts.
        moveCounts = (int) (pDistance * COUNTS_PER_INCH);
        RobotLog.dd(TAG, "Number of counts to drive " + moveCounts);

        if (pDriveMode == DriveMode.STRAIGHT) {
            newLeftFrontTarget = robot.leftFront.getCurrentPosition() + moveCounts;
            newRightFrontTarget = robot.rightFront.getCurrentPosition() + moveCounts;
            newLeftBackTarget = robot.leftBack.getCurrentPosition() + moveCounts;
            newRightBackTarget = robot.rightBack.getCurrentPosition() + moveCounts;
        } else {
            // Must be a strafe.
            newLeftFrontTarget = robot.leftFront.getCurrentPosition() + moveCounts;
            newRightFrontTarget = robot.rightFront.getCurrentPosition() - moveCounts;
            newLeftBackTarget = robot.leftBack.getCurrentPosition() - moveCounts;
            newRightBackTarget = robot.rightBack.getCurrentPosition() + moveCounts;
        }

        // Set target and turn on RUN_TO_POSITION.
        robot.leftFront.setTargetPosition(newLeftFrontTarget);
        robot.rightFront.setTargetPosition(newRightFrontTarget);
        robot.leftBack.setTargetPosition(newLeftBackTarget);
        robot.rightBack.setTargetPosition(newRightBackTarget);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start motion.
        power = Range.clip(Math.abs(pPower), 0.0, 1.0);
        robot.leftFront.setPower(power);
        robot.rightFront.setPower(power);
        robot.leftBack.setPower(power);
        robot.rightBack.setPower(power);

        // Keep looping while we are still active and all motors are running.
        while (linearOpMode.opModeIsActive() &&
                (robot.leftFront.isBusy() && robot.rightFront.isBusy()
                        && robot.leftBack.isBusy() && robot.rightBack.isBusy())) {
            android.os.SystemClock.sleep(20);
        }

        // Stop all motion.
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);

        // Log positions of all motors.
        RobotLog.dd(TAG, "driveRobotWithoutGyro done");
        RobotLog.dd(TAG, "Left front motor position " + robot.leftFront.getCurrentPosition());
        RobotLog.dd(TAG, "Left back motor position " + robot.leftBack.getCurrentPosition());
        RobotLog.dd(TAG, "Right front motor position " + robot.rightFront.getCurrentPosition());
        RobotLog.dd(TAG, "Right back motor position " + robot.rightBack.getCurrentPosition());

        // Turn off RUN_TO_POSITION.
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RobotLog.dd(TAG, "driveRobotWithoutGyro done");
        RobotLog.dd(TAG, "Current gyro heading " + getIMUHeading());
    }

    /**
     * Method to drive on a target compass bearing (angle) based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param pDistance       Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param pPower          Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param pDesiredHeading Absolute Angle (in Degrees) relative to last gyro reset.
     *                        0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     */
    // !!Note: The sign values of various parameters and variables in this method and which motors are
    // set as forward and reverse in the class initialization section above are interdependent.
    public void driveRobotWithGyro(DriveMode pDriveMode,
                                   double pDistance,
                                   double pPower,
                                   double pDesiredHeading) {

        int moveCounts;
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        double power = Range.clip(Math.abs(pPower), 0.0, 1.0);
        double error;
        double steer;
        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;

        // Ensure that the opmode is still active and that there is something to do.
        RobotLog.dd(TAG, "Starting driveRobotWithGyro");
        if (!linearOpMode.opModeIsActive() || (pDistance == 0.0) || (pPower == 0.0)) {
            RobotLog.dd(TAG, "Early exit from driveRobotWithGyro: distance = " + pDistance + ", power = " + pPower);
            return;
        }

        // Ensure the robot is stationary then reset the encoders.
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Turn on RUN_WITH_ENCODERS and then manipulate power levels.
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Determine new target position in encoder counts.
        moveCounts = (int) (pDistance * COUNTS_PER_INCH);
        RobotLog.dd(TAG, "Number of counts to drive " + moveCounts);

        // Adjust power depending on the sign of the distance.
        if (pDistance < 0)
            power = -power; // reverse

        if (pDriveMode == DriveMode.STRAIGHT) {
            newLeftFrontTarget = robot.leftFront.getCurrentPosition() + moveCounts;
            newRightFrontTarget = robot.rightFront.getCurrentPosition() + moveCounts;
            newLeftBackTarget = robot.leftBack.getCurrentPosition() + moveCounts;
            newRightBackTarget = robot.rightBack.getCurrentPosition() + moveCounts;

            // Start motion.
            robot.leftFront.setPower(power);
            robot.rightFront.setPower(power);
            robot.leftBack.setPower(power);
            robot.rightBack.setPower(power);
        } else {
            // Must be a strafe.
            newLeftFrontTarget = robot.leftFront.getCurrentPosition() + moveCounts;
            newRightFrontTarget = robot.rightFront.getCurrentPosition() - moveCounts;
            newLeftBackTarget = robot.leftBack.getCurrentPosition() - moveCounts;
            newRightBackTarget = robot.rightBack.getCurrentPosition() + moveCounts;

            // Start motion.
            robot.leftFront.setPower(power);
            robot.rightFront.setPower(-power);
            robot.leftBack.setPower(-power);
            robot.rightBack.setPower(power);
        }

        // Keep looping while we are still active and no motor has reached its encoder target.
        int currentLeftFrontPosition = 0;
        int currentRightFrontPosition = 0;
        int currentLeftBackPosition = 0;
        int currentRightBackPosition = 0;

        double currentHeading;
        while (linearOpMode.opModeIsActive()) {

            currentLeftFrontPosition = robot.leftFront.getCurrentPosition();
            currentRightFrontPosition = robot.rightFront.getCurrentPosition();
            currentLeftBackPosition = robot.leftBack.getCurrentPosition();
            currentRightBackPosition = robot.rightBack.getCurrentPosition();

            RobotLog.dd(TAG, "Left front motor position " + currentLeftFrontPosition);
            RobotLog.dd(TAG, "Left back motor position " + currentLeftBackPosition);
            RobotLog.dd(TAG, "Right front motor position " + currentRightFrontPosition);
            RobotLog.dd(TAG, "Right back motor position " + currentRightBackPosition);

            // Account for target positions that may be either positive or negative.
            if (((newLeftFrontTarget >= 0) && (currentLeftFrontPosition >= newLeftFrontTarget)) ||
                    ((newLeftFrontTarget <= 0) && (currentLeftFrontPosition <= newLeftFrontTarget)))
                break;

            if (((newRightFrontTarget >= 0) && (currentRightFrontPosition >= newRightFrontTarget)) ||
                    ((newRightFrontTarget <= 0) && (currentRightFrontPosition <= newRightFrontTarget)))
                break;

            if (((newLeftBackTarget >= 0) && (currentLeftBackPosition >= newLeftBackTarget)) ||
                    ((newLeftBackTarget <= 0) && (currentLeftBackPosition <= newLeftBackTarget)))
                break;

            if (((newRightBackTarget >= 0) && (currentRightBackPosition >= newRightBackTarget)) ||
                    ((newRightBackTarget <= 0) && (currentRightBackPosition <= newRightBackTarget)))
                break;

            // 12/28/17 Remove sleep because TeleOp runs fine without one.
            // We needed some delay here in 2016-17 because the gyro
            // would otherwise sometimes return 0 and send the robot
            // into a spin.
            // android.os.SystemClock.sleep(20);

            // Adjust relative speed based on heading error.
            currentHeading = getIMUHeading();
            error = getError(pDesiredHeading, currentHeading, true);
            steer = getSteer(error, P_DRIVE_COEFF);
            if (debugVerbose) {
                RobotLog.dd(TAG, "Error " + error + ", steer " + steer);

                Acceleration basicAccel = robot.imu.getAcceleration();
                Acceleration linearAccel = robot.imu.getLinearAcceleration();
                Acceleration overallAccel = robot.imu.getOverallAcceleration();
                RobotLog.dd(TAG, "Acceleration: x " + basicAccel.xAccel + ", y " + basicAccel.yAccel);
                RobotLog.dd(TAG, "Linear acceleration: x " + linearAccel.xAccel + ", y " + linearAccel.yAccel);
                RobotLog.dd(TAG, "Overall acceleration: x " + overallAccel.xAccel + ", y " + overallAccel.yAccel);

            }

            // Here's what happened: left front power was set to 0.5, error was 6 degrees (CW skew),
            // steer was .6 (error * coefficient), so power was set to -.1. This reversed direction of
            // the motor and invalidated the target encoder counts. So we put in the following limits
            // to ensure that the sign of the power never flips. In addition, since zero power causes
            // the motor to brake, we set a lower power limit.
            if (pDriveMode == DriveMode.STRAIGHT) {
                leftFrontPower = power - steer;
                if (power > 0)
                    leftFrontPower = Range.clip(leftFrontPower, MINIMUM_DRIVE_POWER, 1.0);
                else
                    leftFrontPower = Range.clip(leftFrontPower, -1.0, -MINIMUM_DRIVE_POWER);

                leftBackPower = power - steer;
                if (power > 0)
                    leftBackPower = Range.clip(leftBackPower, MINIMUM_DRIVE_POWER, 1.0);
                else
                    leftBackPower = Range.clip(leftBackPower, -1.0, -MINIMUM_DRIVE_POWER);

                rightFrontPower = power + steer;
                if (power > 0)
                    rightFrontPower = Range.clip(rightFrontPower, MINIMUM_DRIVE_POWER, 1.0);
                else
                    rightFrontPower = Range.clip(rightFrontPower, -1.0, -MINIMUM_DRIVE_POWER);

                rightBackPower = power + steer;
                if (power > 0)
                    rightBackPower = Range.clip(rightBackPower, MINIMUM_DRIVE_POWER, 1.0);
                else
                    rightBackPower = Range.clip(rightBackPower, -1.0, -MINIMUM_DRIVE_POWER);
            } else {

                // Must be a strafe.
                // For a strafe to the left, the left front and right rear motors
                // are moving in a CCW direction and the right front and left
                // rear motors are moving in a CW direction.
                // Use a minimum power level to avoid braking on 0 power and to keep
                // the wheels turning.
                leftFrontPower = power - steer;
                if (power > 0)
                    leftFrontPower = Range.clip(leftFrontPower, MINIMUM_DRIVE_POWER, 1.0);
                else
                    leftFrontPower = Range.clip(leftFrontPower, -1.0, -MINIMUM_DRIVE_POWER);

                leftBackPower = -power - steer;
                if (-power > 0)
                    leftBackPower = Range.clip(leftBackPower, MINIMUM_DRIVE_POWER, 1.0);
                else
                    leftBackPower = Range.clip(leftBackPower, -1.0, -MINIMUM_DRIVE_POWER);

                rightFrontPower = -power + steer;
                if (-power > 0)
                    rightFrontPower = Range.clip(rightFrontPower, MINIMUM_DRIVE_POWER, 1.0);
                else
                    rightFrontPower = Range.clip(rightFrontPower, -1.0, -MINIMUM_DRIVE_POWER);

                rightBackPower = power + steer;
                if (power > 0)
                    rightBackPower = Range.clip(rightBackPower, MINIMUM_DRIVE_POWER, 1.0);
                else
                    rightBackPower = Range.clip(rightBackPower, -1.0, -MINIMUM_DRIVE_POWER);
            }

            if (debugVerbose) {
                RobotLog.dd(TAG, "Power LF, LB, RF, RB:  " + leftFrontPower + " " + leftBackPower + " " + rightFrontPower + " " + rightBackPower);
            }

            robot.leftFront.setPower(leftFrontPower);
            robot.rightFront.setPower(rightFrontPower);
            robot.leftBack.setPower(leftBackPower);
            robot.rightBack.setPower(rightBackPower);
        }

        // Stop all motion.
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);

        // Log positions of all motors.
        RobotLog.dd(TAG, "driveRobotWithGyro done");
        RobotLog.dd(TAG, "Left front motor position " + robot.leftFront.getCurrentPosition());
        RobotLog.dd(TAG, "Left back motor position " + robot.leftBack.getCurrentPosition());
        RobotLog.dd(TAG, "Right front motor position " + robot.rightFront.getCurrentPosition());
        RobotLog.dd(TAG, "Right back motor position " + robot.rightBack.getCurrentPosition());

        RobotLog.dd(TAG, "Current gyro heading " + getIMUHeading());
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     *
     * @param angle Absolute angle (in degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     */
    private void gyroTurn(double angle) {
        gyroTurnFull(TURN_POWER, P_TURN_COEFF, angle, true); // use default turn coefficient
    }

    private void gyroTurnFull(double pPower, double coefficient, double angle, boolean pNormalize) {

        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RobotLog.dd(TAG, "Start turn by gyro, power " + pPower + ", coefficient " + coefficient + ", angle " + angle);

        // Keep looping while we are still active, and not on heading.
        double power = pPower;
        boolean useNormalized = pNormalize;
        double normalizedError = 0.0d;
        double unnormalizedError = 0.0d;

        // Determine the outer threshold dynamically as a factor of the requested power level.
        double outerTurnThreshold = power * TURN_OUTER_THRESHOLD_DEGREES_FACTOR;
        RobotLog.dd(TAG, "Outer threshold " + outerTurnThreshold);

        boolean reachedOuterTarget = false;
        double degreeDifference;
        double currentHeading;
        ElapsedTime outerTargetTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime stallTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        stallTimer.reset();
        while (true) {
            if (!linearOpMode.opModeIsActive()) {
                RobotLog.dd(TAG, "OpMode went inactive during turn");
                break;
            }

            // Instead of trying to manage the difference between normalized and unnormalized
            // headings in the getError function itself, we make use of the fact that as soon
            // as the normalized and unnormalized headings have the same sign we can always use
            // the normalized heading. Works for these cases:
            // 0 -180 1.0 (normalized 179 CCW, unnormalized -181 CW)
            // 0 180 -1.0 (normalized -179 CW, unnormalized 181 CW)

            currentHeading = getIMUHeading();
            RobotLog.dd(TAG, "Gyro current " + currentHeading);

            //** 11/28/18 This logic is flawed!! It will not work in the case where the gyro flips
            // back across the boundary. For example, if the turn is -180 and the current heading is
            // +1, a normalized turn would be +179 (counter-clockwise to -180 along the shortest
            // distance) but an unnormalized turn would be -181 (clockwise). Once the current heading
            // gets to 0 we switch to a normalized turn because it would proceed clockwise to the
            // target at -180 and would correctly handle an overshoot into positive territory, e.g.
            // +179. But what if the current heading reaches 0 or -1 but then flips back to +1 (this
            // can happen)? Since we've switched over to a normalized turn, we'd then proceed
            // counter-clockwise from +1 to -180. Not what we want.
            // This condition has not occurred in testing as of 11/28/18.
            if (!useNormalized) {
                normalizedError = getError(angle, currentHeading, true);
                unnormalizedError = getError(angle, currentHeading, false);
                if (normalizedError * unnormalizedError >= 0.0f) {
                    useNormalized = true;
                    RobotLog.dd(TAG, "Switch to normalized turn with error of " + normalizedError);
                } else
                    RobotLog.dd(TAG, "Use unnormalized turn with error of " + unnormalizedError);
            }

            // If the robot has reached the inner turn window, e.g. 1 degree, stop here.
            if (onTurnHeading(power, angle, coefficient, TURN_INNER_THRESHOLD_DEGREES, currentHeading, useNormalized)) {
                RobotLog.dd(TAG, "Turn complete at " + currentHeading);
                break;
            }

            // See if the robot has reached the outer turn window, e.g. 2 degrees.
            if (!reachedOuterTarget) {
                degreeDifference = getError(angle, currentHeading, useNormalized);
                // Determine the outer threshold dynamically as a factor of the power.
                if (Math.abs(degreeDifference) <= outerTurnThreshold) {
                    reachedOuterTarget = true;
                    power = MINIMUM_TURN_POWER; // use minimum power inside the outer degree threahold
                    RobotLog.dd(TAG, "Reached outer turn target at " + currentHeading);

                    // The robot is inside the outer turn window; start a timer within
                    // which the robot should reach the inner target.
                    outerTargetTimer.reset();
                }
            } else {
                // The robot is within the outer turn target. Check if it is stalled
                // there.
                if (outerTargetTimer.time() >= TURN_OUTER_TARGET_STALL_TIME) {
                    // Stop all motion.
                    robot.leftFront.setPower(0);
                    robot.rightFront.setPower(0);
                    robot.leftBack.setPower(0);
                    robot.rightBack.setPower(0);

                    RobotLog.dd(TAG, "Stalled: outer turn target timer expired");
                    RobotLog.dd(TAG, "Current heading " + currentHeading);
                    break;
                }
            }

            // Check the failsafe timer. This timer will expire if the entire turn
            // has not been completed within its window.
            if (stallTimer.time() >= TURN_MAXIMUM_STALL_TIME) {
                // Stop all motion.
                robot.leftFront.setPower(0);
                robot.rightFront.setPower(0);
                robot.leftBack.setPower(0);
                robot.rightBack.setPower(0);

                RobotLog.dd(TAG, "Stalled: turn aborted, not completed under " + TURN_MAXIMUM_STALL_TIME + " ms");
                RobotLog.dd(TAG, "Current heading " + currentHeading);
                break;
            }

            // Continue the turn.
            // 2016-2017 We needed some delay here because the gyro
            // would otherwise sometimes return 0 and send the robot
            // into a spin.
            // 12/28/17 Remove the sleep because TeleOp runs fine without one.
            //android.os.SystemClock.sleep(20);
        }

        // Get the current heading from the IMU.
        currentHeading = getIMUHeading();
        RobotLog.dd(TAG, "IMU heading after turn " + currentHeading);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param power          Desired speed of turn.
     * @param angle          Absolute Angle (in Degrees) relative to last gyro reset.
     *                       0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                       If a relative angle is required, add/subtract from current heading.
     * @param PCoeff         Proportional Gain coefficient
     * @param pTurnThreshold return true if the error is <= this value
     * @return
     */
    private boolean onTurnHeading(double power, double angle, double PCoeff, double pTurnThreshold, double pCurrentHeading,
                                  boolean pNormalize) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle, pCurrentHeading, pNormalize);

        if (Math.abs(error) <= pTurnThreshold) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = power * steer;
            // Set a minimum power level.
            if (rightSpeed > 0)
                rightSpeed = Range.clip(rightSpeed, MINIMUM_TURN_POWER, 1.0);
            else
                rightSpeed = Range.clip(rightSpeed, -1.0, -MINIMUM_TURN_POWER);

            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftFront.setPower(leftSpeed);
        robot.rightFront.setPower(rightSpeed);
        robot.leftBack.setPower(leftSpeed);
        robot.rightBack.setPower(rightSpeed);

        if (debugVerbose) {
            RobotLog.dd(TAG, "Error " + error + ", steer " + steer);
            RobotLog.dd(TAG, "Power left, right:  " + leftSpeed + " " + rightSpeed);
        }

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle relative to global reference established at last gyro reset).
     * @return error angle: Degrees in the range +/- 180. A positive IMU value indicated that the
     * robot's heading is CCW. In this case the error will be negative if the target angle is less
     * than the current IMIU heading (e.g. target angle -5 CW, IMU +5 CCW for an error of -10),
     * requiring a CW turn of -10 degrees CW to correct) heading and the error will be positive if
     * the target angle is greater than the IMU heading, requiring a CCW turn to correct.
     */

    // LCHS: need to support both a normalized turn (shortest distance to the target) and an
    // unnormalized turn (turn in the requested direction even if the turn is longer).
    public double getError(double targetAngle, double pCurrentHeading, boolean pNormalize) {

        double retErr = DEGREES.normalize(targetAngle - pCurrentHeading); // Calculate error in -180 to +180 range.

        if (!pNormalize)
            // Turn in the originally requested direction.
            // Come here if the normalized heading and the unnormalized requested heading have opposite signs.
            retErr = targetAngle - pCurrentHeading;

        return retErr;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }


    private double getIMUHeading() {
        Orientation angles = robot.imu.getAngularOrientation().toAxesReference(INTRINSIC).toAxesOrder(ZYX);
        return (DEGREES.normalize(angles.firstAngle));
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
    private boolean findGoldMineralTensorflow() {

        RobotLog.dd(TAG, "In findGoldMineralTensorflow()");

        if (tfod == null) {
            RobotLog.dd(TAG, "findGoldMineralTensorflow(): tensorflow not initialized");
            return false;
        }

        RobotLog.dd(TAG, "Current heading " + getIMUHeading());
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

    private boolean findGoldMineralOpenCV(LCHSValues.MineralPosition pMineralPosition) throws InterruptedException {

        //** As an experiment get a number of images from Vuforia.
        //** Also look at the Vuforia Camera device - you may be able to get frames directly.
        List<Bitmap> imageCollection = new ArrayList<>();
        List<Bitmap> cameraImages;

        RobotLog.dd(TAG, "In findGoldMineralOpenCV");
        RobotLog.dd(TAG, "Current heading " + getIMUHeading());

        for (int imX = 0; imX < 10; imX++) {
            cameraImages = getCameraImagesFromVuforia();
            if (!cameraImages.isEmpty())
                imageCollection.addAll(cameraImages);
            android.os.SystemClock.sleep(100);
        }

        if (imageCollection.isEmpty()) {
            RobotLog.dd(TAG, "Failed to get a camera image from Vuforia");
            throw new AutonomousRobotException(TAG, "Failed to get a camera image from Vuforia");
        }

        int imageCollectionSize = imageCollection.size();
        RobotLog.dd(TAG, "Got " + imageCollectionSize + " images from Vuforia");
        // Get the last image in the collection.
        Bitmap oneCameraImage = imageCollection.get(--imageCollectionSize);
        MineralImageProcessing minerals = new MineralImageProcessing();

        // If in TEST mode write all images out to a file.
        if (debugVerbose) {
            String fileDate = dateFormat.format(new Date());
            for (int imX = 0; imX < imageCollection.size(); imX++) {
                // Append image index number to the date
                minerals.writeImageFileFromBitmap(imageCollection.get(imX), fileDate + "_" + Integer.toString(imX));
            }
        }

        RobotLog.dd(TAG, "Using the camera image at index " + imageCollectionSize);
        RobotLog.dd(TAG, "Looking for a gold mineral at position " + pMineralPosition);
        boolean retVal = minerals.findGold(oneCameraImage, pMineralPosition);
        if (retVal)
            RobotLog.dd(TAG, "Found a gold mineral");
        else
            RobotLog.dd(TAG, "Did not find a gold mineral");
        return retVal;
    }

    // Gets a camera image from Vuforia.
    private List<Bitmap> getCameraImagesFromVuforia() throws InterruptedException {

        List<Bitmap> bitmapReturn = new ArrayList<>();

        // from 2016-2017?? Thread.sleep(500); // Allow a little time to settle

        /* LCHS 2019 this callback method comes from the sample ConceptVuforiaNavigationWebcam
         * but would require thread synchronization (Condition with timeout) to work here.
        // Access camera images via the Vuforia frame object.
        vuforiaFrameBitmap = null;
        vuforia.getFrameOnce(Continuation.create(ThreadPool.getDefault(), new Consumer<Frame>() {
            @Override
            public void accept(Frame frame) {
                vuforiaFrameBitmap = vuforia.convertFrameToBitmap(frame);
            }
        }));
        */

        //** Possibly increase the queue depth ...
        RobotLog.dd(TAG, "Vuforia frame queue size " + vuforia.getFrameQueue().size());
        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take(); //takes the frame at the head of the queue

        if (frame == null) {
            RobotLog.dd(TAG, "No frame available from Vuforia");
            return bitmapReturn; // return empty
        }

        long numImages = frame.getNumImages();
        RobotLog.dd(TAG, "Got " + numImages + " images from Vuforia");

        // Return all RGB images.
        Bitmap oneBitmap;
        for (int i = 0; i < numImages; i++) {
            //imageFormat =  frame.getImage(i).getFormat();
            RobotLog.dd("LCHS", "Got an image of type " + frame.getImage(i).getFormat());
            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                RobotLog.dd("LCHS", "Got an RGB image at image index " + i);
                Image rgb = frame.getImage(i);
                oneBitmap = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
                oneBitmap.copyPixelsFromBuffer(rgb.getPixels());
                bitmapReturn.add(oneBitmap);
            }
        }

        frame.close(); // release the frame according to R. Atkinson
        return bitmapReturn;
    }

    // Deploys the tilt motor to stow the boom for transport.
    private class TiltThread extends Thread {

        private int tiltClicks;

        TiltThread(int pTiltClicks) {
            this.setName("TiltThread");
            tiltClicks = pTiltClicks;
        }

        @Override
        public void run() {

            RobotLog.dd(TAG, "Executing TILT thread");

            // Run the tilt motor.
            robot.tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.tilt.setTargetPosition(tiltClicks);
            robot.tilt.setPower(0.3);

            while (linearOpMode.opModeIsActive() && robot.tilt.isBusy()) {
                android.os.SystemClock.sleep(20);
            }

            robot.tilt.setPower(0.0);
            robot.tilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // For synchronization - see TILT_WAIT
            tiltLock.lock();
            RobotLog.dd(TAG, "TILT thread complete");
            tiltMoveComplete = true;
            tiltCondition.signal(); // let the main thread know
            tiltLock.unlock();
        }
    }

}

