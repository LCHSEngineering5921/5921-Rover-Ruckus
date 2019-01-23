package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;

import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Collections;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class LCHSAutoMotion {

    public enum DriveMode {STRAIGHT, STRAFE}

    private enum DriveDirection {FORWARD, REVERSE}

    // These power values must never be negative!!
    public static final double DRIVE_POWER = 0.5; //** always parameterize // default power for straight runs and strafing
    private static final double MINIMUM_DRIVE_POWER = 0.3; // minimum power to turn the wheels
    private static final double MINIMUM_STRAFE_POWER = 0.2; //** always parameterize

    // Turn constants
    public static final double TURN_POWER = 0.3; // 11/13/18 was 0.5;  // Nominal half speed for better accuracy.
    public static final double MINIMUM_TURN_POWER = 0.2; // 11/14/18 was 0.3; // floor for power to the motors

    // PID constants
    public static final double P_DRIVE_COEFF = 0.05; //0.07; // 2017-18 0.1;     // Larger is more responsive, but also less stable
    public static final double I_DRIVE_COEFF = P_DRIVE_COEFF / 10;
    public static final double P_TURN_COEFF = 0.05;  // 11/13/18 was 0.2;      // Larger is more responsive, but also less stable

    // Tilt and boom constants
    private static final int TILT_KNOCK_POSITION = 8400;
    private static final double TILT_POWER = 1.0;
    private static final double BOOM_POWER = 1.0;

    private static final String TAG = "LCHSAutoMotion";
    private LinearOpMode linearOpMode;
    private boolean debugVerbose = false;

    // Set up all devices.
    private LCHSHardwareMap robot;

    private static final double COUNTS_PER_MOTOR_REV = 1120;
    private static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final float mmPerInch = 25.4f;

    private static final int LEFT_FRONT_POWER_INDEX = 0;
    private static final int LEFT_BACK_POWER_INDEX = 1;
    private static final int RIGHT_FRONT_POWER_INDEX = 2;
    private static final int RIGHT_BACK_POWER_INDEX = 3;

    private static final double TURN_INNER_THRESHOLD_DEGREES = 1.0; // As tight as we can make it with the gyro
    private static final double TURN_OUTER_THRESHOLD_DEGREES_FACTOR = 15.0;
    private static final double TURN_OUTER_THRESHOLD_DEGREES = 2.0;
    private static final double TURN_MAXIMUM_STALL_TIME = 5000; // milliseconds
    private static final double TURN_OUTER_TARGET_STALL_TIME = 1500; // milliseconds

    private static final double MAX_DISTANCE_FROM_SIDEWALL = 20.0;

    // For synchronization with the tilt thread.
    private Lock tiltLock = new ReentrantLock();
    private Condition tiltCondition = tiltLock.newCondition();
    private volatile boolean tiltMoveComplete = false;

    // Separate thread fpr the block lifter motor.
    private Thread tiltThread;

    // This class controls all of the motions of the robot.
    public LCHSAutoMotion(LinearOpMode pLinear, LCHSHardwareMap pHardwareMap, boolean pDebugVerbose) {
        linearOpMode = pLinear; // get access to LinearOpMode public fields and methods
        robot = pHardwareMap;
        debugVerbose = pDebugVerbose;

        robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        if (debugVerbose) {
            RobotLog.dd(TAG, "driveRobotWithoutGyro done");
            RobotLog.dd(TAG, "Left front motor position " + robot.leftFront.getCurrentPosition());
            RobotLog.dd(TAG, "Left back motor position " + robot.leftBack.getCurrentPosition());
            RobotLog.dd(TAG, "Right front motor position " + robot.rightFront.getCurrentPosition());
            RobotLog.dd(TAG, "Right back motor position " + robot.rightBack.getCurrentPosition());
        }

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
     * @param pPower          Target speed for forward motion.  Parameter must be positive; clipped below.
     * @param pDesiredHeading Absolute Angle (in Degrees) relative to last gyro reset.
     *                        0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     */
    // !!Note: The sign values of various parameters and variables in this method and which motors are
    // set as forward and reverse in the class initialization section above are interdependent.
    public void driveRobotWithGyro(double pDistance, double pPower, double pDesiredHeading) {

        int moveCounts;
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        RobotLog.dd(TAG, "Starting driveRobotWithGyro");
        if ((pDistance == 0.0) || (pPower == 0.0)) {
            RobotLog.dd(TAG, "Early exit from driveRobotWithGyro: distance = " + pDistance + ", power = " + pPower);
            return;
        }

        // Ensure that the opmode is still active
        if (!linearOpMode.opModeIsActive()) {
            RobotLog.dd(TAG, "Early exit from driveRobotWithGyro: opmode is inactive");
            return;
        }

        double power = Range.clip(Math.abs(pPower), MINIMUM_DRIVE_POWER, 1.0);
        power = (pDistance > 0) ? power : -power; // Set power depending on the sign of the distance.
        DriveDirection driveDirection = (power > 0) ? DriveDirection.FORWARD : DriveDirection.REVERSE;

        double error = 0.0;
        double steer = 0.0;

        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;

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

        newLeftFrontTarget = robot.leftFront.getCurrentPosition() + moveCounts;
        newRightFrontTarget = robot.rightFront.getCurrentPosition() + moveCounts;
        newLeftBackTarget = robot.leftBack.getCurrentPosition() + moveCounts;
        newRightBackTarget = robot.rightBack.getCurrentPosition() + moveCounts;

        // Start motion.
        robot.leftFront.setPower(power);
        robot.rightFront.setPower(power);
        robot.leftBack.setPower(power);
        robot.rightBack.setPower(power);

        // Keep looping while we are still active and no motor has reached its encoder target.
        int currentLeftFrontPosition = 0;
        int currentRightFrontPosition = 0;
        int currentLeftBackPosition = 0;
        int currentRightBackPosition = 0;

        MotionPID motionPID = new MotionPID(P_DRIVE_COEFF, I_DRIVE_COEFF);
        if (debugVerbose) {
            RobotLog.dd(TAG, "kp " + P_DRIVE_COEFF + ", ki " + I_DRIVE_COEFF);
        }

        double currentHeading;
        double[] powerValues = new double[4];
        while (linearOpMode.opModeIsActive()) {

            currentLeftFrontPosition = robot.leftFront.getCurrentPosition();
            currentRightFrontPosition = robot.rightFront.getCurrentPosition();
            currentLeftBackPosition = robot.leftBack.getCurrentPosition();
            currentRightBackPosition = robot.rightBack.getCurrentPosition();

            if (debugVerbose) {
                RobotLog.dd(TAG, "Left front motor position " + currentLeftFrontPosition);
                RobotLog.dd(TAG, "Left back motor position " + currentLeftBackPosition);
                RobotLog.dd(TAG, "Right front motor position " + currentRightFrontPosition);
                RobotLog.dd(TAG, "Right back motor position " + currentRightBackPosition);
            }

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

            currentHeading = getIMUHeading();
            error = getError(pDesiredHeading, currentHeading, true);
            steer = motionPID.getPIDValue(error);

            leftFrontPower = power - steer;
            leftBackPower = power - steer;
            rightFrontPower = power + steer;
            rightBackPower = power + steer;

            //? Try a MutableDouble from Apache Commons lang. Then you wouldn't need the array and the
            // fixed indexes.
            powerValues = clipPowerValues(driveDirection, leftFrontPower, leftBackPower, rightFrontPower, rightBackPower,
                    MINIMUM_DRIVE_POWER);
            leftFrontPower = powerValues[LEFT_FRONT_POWER_INDEX];
            leftBackPower = powerValues[LEFT_BACK_POWER_INDEX];
            rightFrontPower = powerValues[RIGHT_FRONT_POWER_INDEX];
            rightBackPower = powerValues[RIGHT_BACK_POWER_INDEX];

            if (debugVerbose) {
                RobotLog.dd(TAG, "IMU " + currentHeading + ", error " + error + ", p+i " + steer);
                RobotLog.dd(TAG, "Power under P+I control LF, LB, RF, RB:  " + leftFrontPower + " " + leftBackPower + " " + rightFrontPower + " " + rightBackPower);
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
        if (debugVerbose) {
            RobotLog.dd(TAG, "Left front motor position " + robot.leftFront.getCurrentPosition());
            RobotLog.dd(TAG, "Left back motor position " + robot.leftBack.getCurrentPosition());
            RobotLog.dd(TAG, "Right front motor position " + robot.rightFront.getCurrentPosition());
            RobotLog.dd(TAG, "Right back motor position " + robot.rightBack.getCurrentPosition());
            RobotLog.dd(TAG, "Current gyro heading " + getIMUHeading());
        }
    }

    // Drive the robot towards a Vumark.
    // Parameters:
    // double pDistance = estimated distance in inches; use this value if the robot
    //   loses track of the Vumark.
    // double pPower = since the Vumark must be in front of the robot this value must
    //   be positive.
    // double pDesiredHeading = the gyro heading to maintain
    // double pLookForVumark = the number of inches to drive before the robot starts
    //   looking for a Vumark. Use when the robot's starting position is out of range
    //   of a Vumark.
    // double pDistanceFromWall = the point at which the robot should stop moving
    //   forward. This value will be compared with the location information provided
    //   by Vuforia.
    // int pMaxTimeWithoutVumark = after the robot has started looking for a Vumark
    //   it must find one within this time value.
    // Return:
    // double = the angle at which the robot must turn to be perpendicular to the
    //   Vumark (and therefore the wall). A positive value indicates a CCW turn,
    //   a negative value a CW turn. A value outside the range -180/+180 indicates
    //   that the robot did not ultimately use the Vumark for navigation.
    public double driveRobotToVumark(double pDistance, double pPower, double pDesiredHeading,
                                     double pLookForVumark, double pDistanceFromWall,
                                     int pMaxTimeWithoutVumark) {

        double retVal = -361.0; //** public const

        int moveCounts;
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        double power = Range.clip(Math.abs(pPower), MINIMUM_DRIVE_POWER, 1.0);
        double error = 0.0;
        double steer = 0.0;

        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;

        // Ensure that the opmode is still active and that there is something to do.
        //** split
        RobotLog.dd(TAG, "Starting driveRobotToVumark");
        if (!linearOpMode.opModeIsActive() || (pDistance <= 0.0) || (pPower == 0.0)) {
            RobotLog.dd(TAG, "Early exit from driveRobotToVumark: distance = " + pDistance);
            return -362.0; //** const
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

        newLeftFrontTarget = robot.leftFront.getCurrentPosition() + moveCounts;
        newRightFrontTarget = robot.rightFront.getCurrentPosition() + moveCounts;
        newLeftBackTarget = robot.leftBack.getCurrentPosition() + moveCounts;
        newRightBackTarget = robot.rightBack.getCurrentPosition() + moveCounts;

        // Start motion.
        robot.leftFront.setPower(power);
        robot.rightFront.setPower(power);
        robot.leftBack.setPower(power);
        robot.rightBack.setPower(power);

        // Keep looping while we are still active and no motor has reached its encoder target.
        int currentLeftFrontPosition = 0;
        int currentRightFrontPosition = 0;
        int currentLeftBackPosition = 0;
        int currentRightBackPosition = 0;

        MotionPID motionPID = new MotionPID(P_DRIVE_COEFF, I_DRIVE_COEFF);
        if (debugVerbose) {
            RobotLog.dd(TAG, "kp " + P_DRIVE_COEFF + ", ki " + I_DRIVE_COEFF);
        }

        double currentHeading;
        double[] powerValues = new double[4];
        while (linearOpMode.opModeIsActive()) {

            currentLeftFrontPosition = robot.leftFront.getCurrentPosition();
            currentRightFrontPosition = robot.rightFront.getCurrentPosition();
            currentLeftBackPosition = robot.leftBack.getCurrentPosition();
            currentRightBackPosition = robot.rightBack.getCurrentPosition();

            if (debugVerbose) {
                RobotLog.dd(TAG, "Left front motor position " + currentLeftFrontPosition);
                RobotLog.dd(TAG, "Left back motor position " + currentLeftBackPosition);
                RobotLog.dd(TAG, "Right front motor position " + currentRightFrontPosition);
                RobotLog.dd(TAG, "Right back motor position " + currentRightBackPosition);
            }

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


            //** NEED Vuforia parameter in the constructor and a vuforia field
        /*
        OpenGLMatrix lastLocation = vuforia.getRobotLocationFromVumark();
        if (lastLocation != null) {
            VectorF translation = lastLocation.getTranslation();
            // Express position (translation) of robot in inches.
                   linearOpMode.telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
            RobotLog.dd(TAG, "Vuforia target location: X " + translation.get(0) / mmPerInch
                    + ", Y " + translation.get(1) / mmPerInch + ", Z " + translation.get(2) / mmPerInch);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            linearOpMode.telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            RobotLog.dd(TAG, "Vuforia heading " + rotation.thirdAngle);
            RobotLog.dd(TAG, "IMU heading " + getIMUHeading());
        }
        */

            currentHeading = getIMUHeading();
            error = getError(pDesiredHeading, currentHeading, true);
            steer = motionPID.getPIDValue(error);

            leftFrontPower = power - steer;
            leftBackPower = power - steer;
            rightFrontPower = power + steer;
            rightBackPower = power + steer;

            //? Try a MutableDouble from Apache Commons lang. Then you wouldn't need the array and the
            // fixed indexes.
            powerValues = clipPowerValues(DriveDirection.FORWARD, leftFrontPower, leftBackPower, rightFrontPower, rightBackPower,
                    MINIMUM_DRIVE_POWER);
            leftFrontPower = powerValues[LEFT_FRONT_POWER_INDEX];
            leftBackPower = powerValues[LEFT_BACK_POWER_INDEX];
            rightFrontPower = powerValues[RIGHT_FRONT_POWER_INDEX];
            rightBackPower = powerValues[RIGHT_BACK_POWER_INDEX];

            if (debugVerbose) {
                RobotLog.dd(TAG, "IMU " + currentHeading + ", error " + error + ", p+i " + steer);
                RobotLog.dd(TAG, "Power under P+I control LF, LB, RF, RB:  " + leftFrontPower + " " + leftBackPower + " " + rightFrontPower + " " + rightBackPower);
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
        RobotLog.dd(TAG, "driveRobotToVumark done");
        if (debugVerbose) {
            RobotLog.dd(TAG, "Left front motor position " + robot.leftFront.getCurrentPosition());
            RobotLog.dd(TAG, "Left back motor position " + robot.leftBack.getCurrentPosition());
            RobotLog.dd(TAG, "Right front motor position " + robot.rightFront.getCurrentPosition());
            RobotLog.dd(TAG, "Right back motor position " + robot.rightBack.getCurrentPosition());
            RobotLog.dd(TAG, "Current gyro heading " + getIMUHeading());
        }

        return retVal;
    }

    // For use in tracking the side wall and the end wall during a run to the depot from a left-side
    // (crater) start. Negative power means the robot moves backwards.
    // Returns true on normal completion, false if an error condition was detected.

    //** 12/29/18 ONLY works when the wall is on the left side of the robot as it faces forward,
    // which is the case when the robot starts opposite the crater.
    public boolean trackWall(double pDesiredHeading, double pDesiredDistanceFromSideWall,
                             double pPower, double pStrafeCoeff, int pFailsafeTimeout,
                             LCHSAutoTarget pTargetEvaluator) {

        RobotLog.dd(TAG, "Starting trackWall");
        if ((pDesiredDistanceFromSideWall <= 0) || (pPower == 0) || (pStrafeCoeff < 0) || (pFailsafeTimeout <= 0)) {
            RobotLog.dd(TAG, "Early exit from trackWall: invalid parameter(s");
            return false;
        }

        // Ensure that the opmode is still active
        if (!linearOpMode.opModeIsActive()) {
            RobotLog.dd(TAG, "Early exit from trackWall: opmode is inactive");
            return false;
        }

        boolean normalCompletion = true; // default return value

        double power = Range.clip(pPower, -1.0, 1.0); // may be negative for movement in reverse
        DriveDirection driveDirection = (power > 0) ? DriveDirection.FORWARD : DriveDirection.REVERSE;

        double error = 0.0;
        double steer = 0.0;

        double leftFrontPower = power;
        double rightFrontPower = power;
        double leftBackPower = power;
        double rightBackPower = power;

        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Start motion.
        robot.leftFront.setPower(leftFrontPower);
        robot.leftBack.setPower(leftBackPower);
        robot.rightFront.setPower(rightFrontPower);
        robot.rightBack.setPower(rightBackPower);

        MotionPID motionPID = new MotionPID(P_DRIVE_COEFF, I_DRIVE_COEFF);
        if (debugVerbose) {
            RobotLog.dd(TAG, "kp " + P_DRIVE_COEFF + ", ki " + I_DRIVE_COEFF);
        }

        // Overall failsafe: if we haven't reached the endpoint when this timer expires,
        // stop.
        ElapsedTime failsafeTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        failsafeTimer.reset();

        // Lost wall timer: if we haven't got any distance readings from the side wall
        // during this time period, stop.
        //ElapsedTime lostSideWallTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        //lostSideWallTimer.reset();
        //final int LOST_SIDEWALL_TIMEOUT = 3000;
        // if the timer expires with wallReadings == 0 then we've lost the wall
        int wallReadings = 0;

        // Keep looping while we are still active and have not met any exit conditions.
        double currentHeading;
        double actualDistanceFromSideWall;
        double[] powerValues = new double[4];
        while (linearOpMode.opModeIsActive()) {

            if (failsafeTimer.time() >= pFailsafeTimeout) {
                RobotLog.dd(TAG, "Wall tracker failsafe timer expired");
                normalCompletion = false;
                break;
            }

            // If we've reached the target we're done.
            if (pTargetEvaluator.reachedTarget()) {
                RobotLog.dd(TAG, "Reached wall tracker target");
                break;
            }

            /*
            if (lostSideWallTimer.time() > LOST_SIDEWALL_TIIMEOUT) {
                if (wallReadings == 0)
                {
                    RobotLog.dd(TAG, "Exceeded lost sidewall timeout, stopping");
                    break;
                } else {
                    wallReadings = 0;
                    lostSideWallTimer.reset();
                }
            }
            */

            actualDistanceFromSideWall = robot.sensorRange.getDistance(DistanceUnit.INCH);
            RobotLog.dd(TAG, "Distance from side wall " + actualDistanceFromSideWall);
            if (actualDistanceFromSideWall <= MAX_DISTANCE_FROM_SIDEWALL)
                wallReadings++;
            else {
                RobotLog.dd(TAG, "Lost track of sidewall, stopping");
                normalCompletion = false;
                break;
            }

            // Apply IMU-based P+I control
            currentHeading = getIMUHeading();
            error = getError(pDesiredHeading, currentHeading, true);
            steer = motionPID.getPIDValue(error);

            leftFrontPower = power - steer;
            leftBackPower = power - steer;
            rightFrontPower = power + steer;
            rightBackPower = power + steer;

            if (debugVerbose) {
                RobotLog.dd(TAG, "IMU " + currentHeading + ", error " + error + ", p+i " + steer);
                RobotLog.dd(TAG, "Power under P+I control LF, LB, RF, RB:  " + leftFrontPower + " " + leftBackPower + " " + rightFrontPower + " " + rightBackPower);
            }

            // If we're outside the sidewall window. Apply strafing power profile as observed in TeleOp.
            // The sign of the power to apply tells us whether we're going forwards or backwards.
            double errorDistance = pDesiredDistanceFromSideWall - actualDistanceFromSideWall;
            double powerCorrectValue = errorDistance * pStrafeCoeff;
            RobotLog.dd(TAG, "Error Distance from Wall: " + errorDistance);
            RobotLog.dd(TAG, "Strafe Value: " + powerCorrectValue);

            //** make parameter for window
            if (Math.abs(errorDistance) < 2) {
                // The robot is inside the window so stay with the normal PID.
                powerValues = clipPowerValues(driveDirection, leftFrontPower, leftBackPower, rightFrontPower, rightBackPower,
                        MINIMUM_DRIVE_POWER);
                leftFrontPower = powerValues[LEFT_FRONT_POWER_INDEX];
                leftBackPower = powerValues[LEFT_BACK_POWER_INDEX];
                rightFrontPower = powerValues[RIGHT_FRONT_POWER_INDEX];
                rightBackPower = powerValues[RIGHT_BACK_POWER_INDEX];
            } else {
                // Apply strafe correction.
                leftFrontPower += powerCorrectValue;
                rightBackPower += powerCorrectValue;
                rightFrontPower -= powerCorrectValue;
                leftBackPower -= powerCorrectValue;

                if (debugVerbose) {
                    RobotLog.dd(TAG, "Apply strafe factor: LF, RB " + powerCorrectValue + "; RF, LB " + -powerCorrectValue);
                    RobotLog.dd(TAG, "Power with strafe factor LF, LB, RF, RB:  " + leftFrontPower + " " + leftBackPower + " " + rightFrontPower + " " + rightBackPower);
                }

                double largest = Collections.max(Arrays.asList(Math.abs(leftFrontPower), Math.abs(leftBackPower),
                        Math.abs(rightFrontPower), Math.abs(rightBackPower)));
                // natural order is least to greatest, so will get largest

                leftFrontPower /= largest;
                leftBackPower /= largest;
                rightFrontPower /= largest;
                rightBackPower /= largest;

                // We still have to protect against flipping the sign of the power.
                powerValues = clipPowerValues(driveDirection, leftFrontPower, leftBackPower, rightFrontPower, rightBackPower,
                        MINIMUM_STRAFE_POWER);
                leftFrontPower = powerValues[LEFT_FRONT_POWER_INDEX];
                leftBackPower = powerValues[LEFT_BACK_POWER_INDEX];
                rightFrontPower = powerValues[RIGHT_FRONT_POWER_INDEX];
                rightBackPower = powerValues[RIGHT_BACK_POWER_INDEX];
            }

            RobotLog.dd(TAG, "Final Power LF: " + leftFrontPower);
            RobotLog.dd(TAG, "Final Power RB: " + rightBackPower);
            RobotLog.dd(TAG, "Final Power LB: " + leftBackPower);
            RobotLog.dd(TAG, "Final Power RF: " + rightFrontPower);

            robot.leftFront.setPower(leftFrontPower);
            robot.leftBack.setPower(leftBackPower);
            robot.rightFront.setPower(rightFrontPower);
            robot.rightBack.setPower(rightBackPower);

        } // while

        // Stop all motion.
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);

        // Reset to default mode.
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RobotLog.dd(TAG, "trackWall done");
        RobotLog.dd(TAG, "Current gyro heading " + getIMUHeading());

        return normalCompletion;
    }


    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     *
     * @param angle Absolute angle (in degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     */
    public void gyroTurn(double angle) {
        gyroTurnFull(TURN_POWER, P_TURN_COEFF, angle, true); // use default turn coefficient
    }

    public void gyroTurnFull(double pPower, double coefficient, double angle, boolean pNormalize) {

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

    public double getIMUHeading() {
        Orientation angles = robot.imu.getAngularOrientation().toAxesReference(INTRINSIC).toAxesOrder(ZYX);
        return (DEGREES.normalize(angles.firstAngle));
    }


    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param power          Desired speed of turn.
     * @param angle          Absolute Angle (in Degrees) relative to last gyro reset.
     *                       0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                       If a relative angle is required, add/subtract from current heading.
     * @param pCoeff         Proportional Gain coefficient
     * @param pTurnThreshold return true if the error is <= this value
     * @return
     */
    private boolean onTurnHeading(double power, double angle, double pCoeff, double pTurnThreshold, double pCurrentHeading,
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
            steer = Range.clip(error * pCoeff, -1, 1); // steer = error * kp
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
            RobotLog.dd(TAG, "IMU " + pCurrentHeading + ", error " + error + ", steer " + steer);
            RobotLog.dd(TAG, "Power left, right:  " + leftSpeed + " " + rightSpeed);
        }

        return onTarget;
    }

    // Perform range clipping on the power to each wheel while taking the direction (sign)
    // of the power into account.
    // Here's what happened: left front power was set to 0.5, error was 6 degrees (CW skew),
    // steer was .6 (error * coefficient), so power was set to -.1. This reversed direction of
    // the motor and invalidated the target encoder counts. So we put in the following limits
    // to ensure that the sign of the power never flips.
    private double[] clipPowerValues(DriveDirection pDriveDirection,
                                     double pLeftFrontPower, double pLeftBackPower,
                                     double pRightFrontPower, double pRightBackPower,
                                     double pMinimumDrivePower) {

        double[] retVal = new double[]{0, 0, 0, 0};
        double minimumDrivePower = Range.clip(Math.abs(pMinimumDrivePower), -1.0, 1.0); // failsafe

        if (pDriveDirection == DriveDirection.FORWARD) {
            minimumDrivePower = (minimumDrivePower < 0) ? 0 : minimumDrivePower; // override sign flip
            retVal[LEFT_FRONT_POWER_INDEX] = Range.clip(pLeftFrontPower, minimumDrivePower, 1.0);
            retVal[LEFT_BACK_POWER_INDEX] = Range.clip(pLeftBackPower, minimumDrivePower, 1.0);
            retVal[RIGHT_FRONT_POWER_INDEX] = Range.clip(pRightFrontPower, minimumDrivePower, 1.0);
            retVal[RIGHT_BACK_POWER_INDEX] = Range.clip(pRightBackPower, minimumDrivePower, 1.0);
        } else {
            minimumDrivePower = (-minimumDrivePower > 0) ? 0 : minimumDrivePower; // override sign flip
            retVal[LEFT_FRONT_POWER_INDEX] = Range.clip(pLeftFrontPower, -1.0, -minimumDrivePower);
            retVal[LEFT_BACK_POWER_INDEX] = Range.clip(pLeftBackPower, -1.0, -minimumDrivePower);
            retVal[RIGHT_FRONT_POWER_INDEX] = Range.clip(pRightFrontPower, -1.0, -minimumDrivePower);
            retVal[RIGHT_BACK_POWER_INDEX] = Range.clip(pRightBackPower, -1.0, -minimumDrivePower);
        }

        return retVal;
    }

    // General-purpose class for PID motor control.
    // NOTE: the timer starts on object construction.
    private class MotionPID {

        private ElapsedTime pidTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ;
        private final double Kp;
        private final double Ki;
        private final double Kd;

        private double errSum;
        private double prevError = 0.0;
        private double proportional;
        private double integrated;
        private double derivative;

        // Proportional, Integrated, Derivative
        private MotionPID(double pKp, double pKi, double pKd) {
            Kp = pKp;
            Ki = pKi;
            Kd = pKd;
            pidTimer.reset();
        }

        // Proportional and Integrated only
        private MotionPID(double pKp, double pKi) {
            Kp = pKp;
            Ki = pKi;
            Kd = 0.0;
            pidTimer.reset();
        }

        // Proportional only
        private MotionPID(double pKp) {
            Kp = pKp;
            Ki = 0.0;
            Kd = 0.0;
            pidTimer.reset();
        }

        // Perform all PID calculations and return composite value.
        private double getPIDValue(double pError) {

            errSum += pError * (pidTimer.time() / 1000); // accumulate for integral
            if (pError == 0)
                errSum = 0; // See "Basics of PID Control in Robots.pptx"
            pidTimer.reset(); // restart timer

            proportional = pError * Kp;
            integrated = errSum * Ki;
            derivative = (pError - prevError) * Kd;
            prevError = pError;

            if (debugVerbose) {
                //** MOVE or split
                //**RobotLog.dd(TAG, "Proportional " + proportional + ", errSum " + errSum + ",  integral " + (I_DRIVE_COEFF * errSum));
                //** RobotLog.dd(TAG, "Power under P+I control LF, LB, RF, RB:  " + leftFrontPower + " " + leftBackPower + " " + rightFrontPower + " " + rightBackPower);
            }

            return Range.clip((proportional + integrated + derivative), -1.0, 1.0);
        }
    }


    // Let the robot down from the rover.
    public void land() {

        RobotLog.dd(TAG, "In land()");

        robot.hookServo.setPosition(LCHSHardwareMap.HOOK_SERVO_OPEN);

        robot.tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //** added 12/17/18
        robot.tilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //** added 12/17/18
        while (robot.tilt.getCurrentPosition() < LCHSHardwareMap.TILT_POS_AFTER_GRAVITY) {
            android.os.SystemClock.sleep(20);
        }

        robot.tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.tilt.setTargetPosition(LCHSHardwareMap.TILT_POS_ROBOT_IS_LEVEL);
        robot.tilt.setPower(TILT_POWER);

        while (Math.abs(robot.tilt.getCurrentPosition() - LCHSHardwareMap.TILT_POS_ROBOT_IS_LEVEL) > 50) {
            android.os.SystemClock.sleep(20);
        }

        robot.tilt.setPower(0.0);
        robot.tilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RobotLog.dd(TAG, "Exit land()");
    }

    // Place our marker into the depot.
    public void claim(int pTargetBoomPosition, int pOuttakeTime, double pOuttakePower) {
        robot.boom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //** added 12/21/18
        robot.boom.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //** added 12/21/18
        robot.boom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.boom.setTargetPosition(pTargetBoomPosition);
        robot.boom.setPower(BOOM_POWER);

        while (Math.abs(robot.boom.getCurrentPosition() - pTargetBoomPosition) > 50) {
            android.os.SystemClock.sleep(20);
        }

        // Use flappers to spit out OR use gate to drop??
        //robot.gateLeft.setPosition(LCHSHardwareMap.GATE_LEFT_SERVO_OPEN);
        robot.intakeRight.setPower(-pOuttakePower);
        robot.intakeLeft.setPower(-pOuttakePower);

        android.os.SystemClock.sleep(pOuttakeTime);

        robot.intakeLeft.setPower(0.0);
        robot.intakeRight.setPower(0.0);
    }

    // For sampling, use boom to knock gold mineral
    public void knockUsingBoom(int targetBoomPosition) {

        //** reinstate?> robot.tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //** added 12/21/18
        robot.tilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //** added 12/21/18
        robot.tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.tilt.setTargetPosition(TILT_KNOCK_POSITION);
        robot.tilt.setPower(TILT_POWER);

        // wait until reached target
        while (Math.abs(robot.tilt.getCurrentPosition() - TILT_KNOCK_POSITION) > 50) {
            android.os.SystemClock.sleep(20);
        }

        //** reinstate? robot.boom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //** added 12/21/18
        //** call extendBoom
        robot.boom.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //** added 12/21/18
        robot.boom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.boom.setTargetPosition(targetBoomPosition);
        robot.boom.setPower(BOOM_POWER);

        int currentBoomPosition = robot.boom.getCurrentPosition();
        int lastBoomPosition = 0;
        // wait until reached target
        while (Math.abs(currentBoomPosition - targetBoomPosition) > 50) {
            lastBoomPosition = currentBoomPosition;
            android.os.SystemClock.sleep(20);
            currentBoomPosition = robot.boom.getCurrentPosition();
            RobotLog.dd(TAG, "Difference in boom position " + Math.abs(currentBoomPosition - lastBoomPosition));
            //if (Math.abs(currentBoomPosition - lastBoomPosition) < 1) break;
        }
        robot.boom.setPower(0);
    }

    //!!!//** added by trinity 12/26/18
    public void extendBoom(int pTargetPosition) {
        robot.boom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.boom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.boom.setTargetPosition(pTargetPosition);
        robot.boom.setPower(BOOM_POWER);
    }

    // Tilt the boom in a separate thread.
    public void tiltStart(int pTargetTiltPosition) {

        tiltLock.lock();
        tiltMoveComplete = false; // make sure the synchronization flag starts off false
        tiltLock.unlock();

        RobotLog.dd(TAG, "Starting TILT thread");
        tiltThread = new TiltThread(pTargetTiltPosition);
        tiltThread.start();
    }

    // Wait for the Tilt thread to complete.
    //**  Use join() instead or await() with timeout
    public void tiltWait() throws InterruptedException {
        tiltLock.lock();
        try {
            while (!tiltMoveComplete)
                tiltCondition.await();
            tiltMoveComplete = false;
        } finally {
            tiltLock.unlock();
            RobotLog.dd(TAG, "Synchronized with tilt thread");
        }
    }


    // Deploys the tilt motor to stow the boom for transport.
    private class TiltThread extends Thread {

        private int tiltClicks;

        private TiltThread(int pTiltClicks) {
            this.setName("TiltThread");
            tiltClicks = pTiltClicks;
        }

        @Override
        public void run() {

            RobotLog.dd(TAG, "Executing TILT thread");

            // Run the tilt motor.
            robot.tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.tilt.setTargetPosition(tiltClicks);
            robot.tilt.setPower(TILT_POWER);

            while (linearOpMode.opModeIsActive() && robot.tilt.isBusy()) {
                android.os.SystemClock.sleep(20);
            }

            robot.tilt.setPower(0.0);
            robot.tilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // For synchronization - see TILT_WAIT
            try {
                tiltLock.lock();
                RobotLog.dd(TAG, "TILT thread complete");
                tiltMoveComplete = true;
                tiltCondition.signal(); // let the main thread know
            } finally {
                tiltLock.unlock();
            }
        }
    }

}

