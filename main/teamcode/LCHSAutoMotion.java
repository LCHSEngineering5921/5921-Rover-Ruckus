package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;

import java.text.SimpleDateFormat;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class LCHSAutoMotion {

    public enum DriveMode {STRAIGHT, STRAFE}

    public static final double DRIVE_POWER = 0.5; // default power for straight runs and strafing
    public static final double MINIMUM_DRIVE_POWER = 0.3; // minimum power to turn the wheels

    // Turn constants
    public static final double TURN_POWER = 0.3; // 11/13/18 was 0.5;  // Nominal half speed for better accuracy.
    public static final double MINIMUM_TURN_POWER = 0.2; // 11/14/18 was 0.3; // floor for power to the motors

    public static final double TILT_POWER = 1.0;
    public static final double BOOM_POWER = 1.0;

    // PID constants
    public static final double P_DRIVE_COEFF = 0.05; //0.07; // 2017-18 0.1;     // Larger is more responsive, but also less stable
    public static final double I_DRIVE_COEFF = P_DRIVE_COEFF / 10;
    public static final double P_TURN_COEFF = 0.05;  // 11/13/18 was 0.2;      // Larger is more responsive, but also less stable

    private static final String TAG = "LCHSAutoMotion";
    private static final SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMddHHmmss");

    private LinearOpMode linearOpMode;
    private boolean debugVerbose = false;

    // Set up all devices.
    private LCHSHardwareMap robot;

    private static final double COUNTS_PER_MOTOR_REV = 1120;
    private static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private static final double TURN_INNER_THRESHOLD_DEGREES = 1.0; // As tight as we can make it with an integer gyro
    private static final double TURN_OUTER_THRESHOLD_DEGREES_FACTOR = 15.0;
    private static final double TURN_OUTER_THRESHOLD_DEGREES = 2.0;
    private static final double TURN_MAXIMUM_STALL_TIME = 5000; // milliseconds
    private static final double TURN_OUTER_TARGET_STALL_TIME = 1500; // milliseconds

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
    public void driveRobotWithGyro(double pDistance, double pPower, double pDesiredHeading) {

        int moveCounts;
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        double power = 0.0;
        double error = 0.0;
        double steer = 0.0;
        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;

        // Ensure that the opmode is still active and that there is something to do.
        RobotLog.dd(TAG, "Starting driveRobotWithGyro");
        if (!linearOpMode.opModeIsActive() || (pDistance == 0.0) || (pPower == 0.0)) {
            RobotLog.dd(TAG, "Early exit from driveRobotWithGyro: distance = " + pDistance);
            return;
        }

        power = Range.clip(Math.abs(pPower), MINIMUM_DRIVE_POWER, 1.0);
        if (pPower != power)
            RobotLog.dd(TAG, "Power clipped from " + pPower + " to " + power);

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

        // PID variables: integral only
        double errSum = 0.0;
        double proportional = 0.0;

        if (debugVerbose) {
            RobotLog.dd(TAG, "kp " + P_DRIVE_COEFF + ", ki " + I_DRIVE_COEFF);
        }

        double currentHeading;
        ElapsedTime pidTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        pidTimer.reset();
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

            currentHeading = getIMUHeading();
            error = getError(pDesiredHeading, currentHeading, true);
            errSum += error * (pidTimer.time() / 1000); // accumulate for integral
            pidTimer.reset(); // restart timer

            steer = Range.clip(error * P_DRIVE_COEFF, -1, 1); // steer = error * kp
            proportional = steer; // save for logging

            // Add integral factor
            steer += (I_DRIVE_COEFF * errSum);

            if (debugVerbose) {
                RobotLog.dd(TAG, "IMU " + currentHeading + ", error " + error + ", p+i " + steer);
                RobotLog.dd(TAG, "Proportional " + proportional + ", errSum " + errSum + ",  integral " + (I_DRIVE_COEFF * errSum));
            }

            // Here's what happened: left front power was set to 0.5, error was 6 degrees (CW skew),
            // steer was .6 (error * coefficient), so power was set to -.1. This reversed direction of
            // the motor and invalidated the target encoder counts. So we put in the following limits
            // to ensure that the sign of the power never flips. In addition, since zero power causes
            // the motor to brake, we set a lower power limit.
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

    // For use in tracking the side wall and the end wall during a run to the depot from a left-side
    // (crater) start.
    public void trackWall(double pDesiredHeading, double pDesiredDistanceFromSideWall,
                          double pPower, double pStrafeCoeff, int pFailsafeTimeout,
                          LCHSAutoTarget pTargetEvaluator) {

        double power = Range.clip(Math.abs(pPower), -1.0, 1.0); // may be negative for movement in reverse
        double powerDirection = (power > 0) ? 1.0 : -1.0;
        double leftFrontPower = power;
        double rightFrontPower = power;
        double leftBackPower = power;
        double rightBackPower = power;

        // Ensure that the opmode is still active
        RobotLog.dd(TAG, "Starting trackWall");
        if (!linearOpMode.opModeIsActive() || (pDesiredDistanceFromSideWall <= 0) ||
                (pPower == 0) || (pStrafeCoeff <= 0) || (pFailsafeTimeout <= 0)) {
            RobotLog.dd(TAG, "Early exit from trackWall: opmode is inactive or one or more parameters == 0");
            return;
        }

        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Start motion.
        robot.leftFront.setPower(leftFrontPower);
        robot.leftBack.setPower(leftBackPower);
        robot.rightFront.setPower(rightFrontPower);
        robot.rightBack.setPower(rightBackPower);

        // P+I variables
        double error = 0.0;
        double steer = 0.0;
        double errSum = 0.0;
        double proportional = 0.0;

        if (debugVerbose) {
            RobotLog.dd(TAG, "kp " + P_DRIVE_COEFF + ", ki " + I_DRIVE_COEFF);
        }

        double currentHeading;
        ElapsedTime pidTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        pidTimer.reset();

        // Overall failsafe: if we haven't reached the endpoint when this timer expires,
        // stop.
        ElapsedTime failsafeTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        failsafeTimer.reset();

        // Lost wall timer: if we haven't got any distance readings from the side wall
        // during this time period, stop.
        //ElapsedTime lostSideWallTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        //lostSideWallTimer.reset();

        final int LOST_SIDEWALL_TIIMEOUT = 3000;
        final double MAX_DISTANCE_FROM_SIDEWALL = 20.0; // inches
        final double SIDEWALL_OUT_OF_RANGE = -1;

        int wallReadings = 0;
        double actualDistanceFromSideWall;


        // Keep looping while we are still active and all motors are running.
        while (linearOpMode.opModeIsActive()) {

            if (failsafeTimer.time() >= pFailsafeTimeout) {
                RobotLog.dd(TAG, "Wall tracker failsafe timer expired");
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
                break;
            }

            // Apply IMU-based P+I control
            currentHeading = getIMUHeading();
            error = getError(pDesiredHeading, currentHeading, true);
            errSum += error * (pidTimer.time() / 1000); // accumulate for integral
            pidTimer.reset(); // restart timer

            steer = Range.clip(error * P_DRIVE_COEFF, -1, 1); // steer = error * kp
            proportional = steer; // save for logging

            // Add integral factor
            steer += (I_DRIVE_COEFF * errSum);
            leftFrontPower = power - steer;
            leftBackPower = power - steer;
            rightFrontPower = power + steer;
            rightBackPower = power + steer;

            if (debugVerbose) {
                RobotLog.dd(TAG, "IMU " + currentHeading + ", error " + error + ", p+i " + steer);
                RobotLog.dd(TAG, "Proportional " + proportional + ", errSum " + errSum + ",  integral " + (I_DRIVE_COEFF * errSum));
                RobotLog.dd(TAG, "Power under P+I control LF, LB, RF, RB:  " + leftFrontPower + " " + leftBackPower + " " + rightFrontPower + " " + rightBackPower);
            }

            // We're outside the sidewall window. Apply strafing power profile as observed in TeleOp.
            // The sign of the power to apply tells us whether we're going forwards or backwards.
            double errorDistance = actualDistanceFromSideWall - pDesiredDistanceFromSideWall;
            double powerCorrectValue = errorDistance * pStrafeCoeff * powerDirection;
            // 3 inches * 0.05 = 0.15
            // pStrafeCoeff is parameter; represents error to power factor

            if (errorDistance > 0) {
                // Too far from wall, strafe left
                leftFrontPower -= powerCorrectValue;
                rightBackPower -= powerCorrectValue;
                rightFrontPower += powerCorrectValue;
                leftBackPower += powerCorrectValue;
            } else if (errorDistance < 0) {
                // Too close to wall, strafe right
                leftFrontPower += powerCorrectValue;
                rightBackPower += powerCorrectValue;
                rightFrontPower -= powerCorrectValue;
                leftBackPower -= powerCorrectValue;
            }

            if (debugVerbose) {
                RobotLog.dd(TAG, "Apply strafe factor: LF, RB " + leftFrontPower + "; RF, LB " + rightFrontPower);
            }


            if (power > 0)
                leftFrontPower = Range.clip(leftFrontPower, MINIMUM_DRIVE_POWER, 1.0);
            else
                leftFrontPower = Range.clip(leftFrontPower, -1.0, -MINIMUM_DRIVE_POWER);

            if (power > 0)
                leftBackPower = Range.clip(leftBackPower, MINIMUM_DRIVE_POWER, 1.0);
            else
                leftBackPower = Range.clip(leftBackPower, -1.0, -MINIMUM_DRIVE_POWER);

            if (power > 0)
                rightFrontPower = Range.clip(rightFrontPower, MINIMUM_DRIVE_POWER, 1.0);
            else
                rightFrontPower = Range.clip(rightFrontPower, -1.0, -MINIMUM_DRIVE_POWER);

            if (power > 0)
                rightBackPower = Range.clip(rightBackPower, MINIMUM_DRIVE_POWER, 1.0);
            else
                rightBackPower = Range.clip(rightBackPower, -1.0, -MINIMUM_DRIVE_POWER);

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
    public void claim(int pTargetBoomPosition) {
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
        robot.intakeRight.setPower(-0.5);
        robot.intakeLeft.setPower(-0.5); // used to be -0.8, lowered power so less throw

        android.os.SystemClock.sleep(2000);

        robot.intakeLeft.setPower(0.0);
        robot.intakeRight.setPower(0.0);
    }

    // For sampling, use boom to knock gold mineral
    public void knockUsingBoom(int targetBoomPosition) {

        final int targetTiltPosition = 8300; //** TOP

        //** reinstate?> robot.tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //** added 12/21/18
        robot.tilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //** added 12/21/18
        robot.tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.tilt.setTargetPosition(targetTiltPosition);
        robot.tilt.setPower(TILT_POWER);

        // wait until reached target
        while (Math.abs(robot.tilt.getCurrentPosition() - targetTiltPosition) > 50) {
            android.os.SystemClock.sleep(20);
        }

        //** reinstate? robot.boom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //** added 12/21/18
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
            RobotLog.dd(TAG, "Difference in boom position", Math.abs(currentBoomPosition - lastBoomPosition));
            //if (Math.abs(currentBoomPosition - lastBoomPosition) < 1) break;
        }
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
    public void tiltWait() throws InterruptedException {
        tiltLock.lock();
        while (!tiltMoveComplete)
            tiltCondition.await();
        tiltMoveComplete = false;
        tiltLock.unlock();
        RobotLog.dd(TAG, "Synchronized with tilt thread");
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
            robot.tilt.setPower(TILT_POWER);

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

