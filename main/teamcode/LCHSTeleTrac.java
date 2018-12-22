package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

// Program that drives and turns the robot and records the movements in the
// log flle when the left bumper button is pressed. The log file can be
// analyzed later to determine distances and turning angles for the autonomous
// opMode.
// Left bumper: start recording
// Right bumper: stop recording

// The first token in the TeleTrac.txt configuration file is used as the label for
// the current run. The number of start/stop pairs is controlled by the number of
// tokens in the configuration file, starting at index 1. To allow for free-form
// testing the special label "UNRESTRICTED_TEST" as the only token means that there
// is no restriction on the number of start/stop pairs. But the pairs will be
// identified by numbers starting at 1 instead of by labels.

@TeleOp(name = "LCHS TeleTrac", group = "TeamCode")
@Disabled
public class LCHSTeleTrac extends LinearOpMode {

    private static final String TAG = "LCHS TeleTrac";
    // Declare hardwareMap object
    private LCHSHardwareMap robot = null;
    private TeleTracConfiguration config;
    private List<String> teleTracLabels;
    private boolean unrestrictedTest = false;

    // The IMU sensor object
    private BNO055IMU imu;
    private Orientation angles;
    private double startIMUHeading;

    private boolean startRecording = false;
    private boolean stopRecording = false;
    private int recordingNumber = 1;
    private ElapsedTime tracTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    // The encoder values must be in the order: left front, left back, right front, right back
    private ArrayList<Integer> encoderValues = new ArrayList<>();
    private int zeroEncoderCount = 0;
    private int positiveEncoderCount = 0;
    private int negativeEncoderCount = 0;

    // Encoder values with the same sign, sorted in ascending order.
    private ArrayList<Integer> sortedEncoderValues;


    @Override
    public void runOpMode() {
        try {
            telemetry.setAutoClear(false); // keep our messages on the driver station

            config = new TeleTracConfiguration();
            teleTracLabels = config.getTeleTracLabels();

            // Display the label for this run.
            telemetry.addData("LCHS TeleTrac ", teleTracLabels.get(0));
            telemetry.update();

            // Important: put the run label into the log so that we can correlate later.
            RobotLog.dd(TAG, "Run label: " + teleTracLabels.get(0));

            // If the label is UNRESTRICTED_TEST then do not control the number of
            // start/stop pairs.
            if (teleTracLabels.get(0).equals(TeleTracConfiguration.UNRESTRICTED_TEST))
                unrestrictedTest = true;

            // Initialize the motors, servos and sensors
            robot = new LCHSHardwareMap(hardwareMap, true, true);

            robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Set teleop default.
            robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            imu = robot.imu; // may be null if there was an error in IMU initialization

            // Setup a variable for each driveX wheel to save power level for telemetry
            double rightFrontP;
            double leftFrontP;
            double rightBackP;
            double leftBackP;

            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {

                double driveX = -gamepad1.right_stick_x;
                double driveY = -gamepad1.right_stick_y;
                double turn = gamepad1.left_stick_x;

                // ---------- Calculate Wheel Power ----------
                leftFrontP = Range.clip(driveY - driveX + turn, -1.0, 1.0);
                rightFrontP = Range.clip(driveY + driveX - turn, -1.0, 1.0);
                leftBackP = Range.clip(driveY + driveX + turn, -1.0, 1.0);
                rightBackP = Range.clip(driveY - driveX - turn, -1.0, 1.0);


                // Send calculated power to wheels
                robot.leftFront.setPower(leftFrontP);
                robot.rightFront.setPower(rightFrontP);
                robot.leftBack.setPower(leftBackP);
                robot.rightBack.setPower(rightBackP);


                // Turn tracking ON
                if (gamepad1.left_bumper) {
                    if (!startRecording) {
                        startRecording = true;
                        stopRecording = false;
                        tracTimer.reset();

                        if (unrestrictedTest) {
                            RobotLog.dd(TAG, "Start unrestricted tracking " + recordingNumber);
                            telemetry.addData("Start unrestricted tracking ", recordingNumber);
                            telemetry.update();
                        } else {
                            RobotLog.dd(TAG, "Start tracking " + teleTracLabels.get(recordingNumber));
                            telemetry.addData("Start tracking ", teleTracLabels.get(recordingNumber));
                            telemetry.update();
                        }

                        startIMUHeading = getIMUHeading();
                        RobotLog.dd(TAG, "IMU heading at start " + startIMUHeading);

                        // Ensure the robot is stationary then reset the encoders.
                        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        // Turn on RUN_WITH_ENCODER and then manipulate power levels.
                        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                }

                // If recording has been started, log values at a fixed interval.
                if (startRecording) {
                    if (tracTimer.time() >= 100) {
                        tracTimer.reset(); // restart timer
                        // RobotLog.dd(TAG, "Track");
                        // getEncoderValues();
                        // logEncoderValues();
                        // logIMUHeading();
                    }
                }

                // Turn tracking OFF
                if (gamepad1.right_bumper) {
                    // Stop recording if recording has not already been stopped
                    // and recording has been started.
                    // Prevents double hits and spurious hits.
                    if (!stopRecording && startRecording) {
                        stopRecording = true;
                        startRecording = false;

                        // Stop all motion.
                        robot.leftFront.setPower(0);
                        robot.rightFront.setPower(0);
                        robot.leftBack.setPower(0);
                        robot.rightBack.setPower(0);

                        if (unrestrictedTest) {
                            RobotLog.dd(TAG, "Stop unrestricted tracking " + recordingNumber);
                            telemetry.addData("Stop unrestricted tracking ", recordingNumber);
                            telemetry.update();
                        } else {
                            RobotLog.dd(TAG, "Stop tracking " + teleTracLabels.get(recordingNumber));
                            telemetry.addData("Stop tracking ", teleTracLabels.get(recordingNumber));
                            telemetry.update();
                        }

                        double stopIMUHeading = getIMUHeading();
                        RobotLog.dd(TAG, "IMU heading at stop " + stopIMUHeading);
                        RobotLog.dd(TAG, "Degrees difference: " + AngleUnit.DEGREES.normalize(stopIMUHeading - startIMUHeading));

                        // Attempt to get the greatest distance moved among the 4 encoder values.
                        // This only works if all 4 values have the same sign.
                        getEncoderValues(); // get the values and the counts of positive and negative values
                        if ((positiveEncoderCount == 4) || (negativeEncoderCount == 4)) {
                            sortedEncoderValues = new ArrayList<>(encoderValues); // don't mess with the original
                            Collections.sort(sortedEncoderValues); // default ascending
                            int greatestEncoderValue;
                            if (positiveEncoderCount == 4)
                                greatestEncoderValue = sortedEncoderValues.get(3); // highest positive
                            else
                                greatestEncoderValue = sortedEncoderValues.get(0); // lowest negative

                            // Convert encoder clicks to inches and log the value;
                            double inchesMoved = greatestEncoderValue / LCHSAutoMotion.COUNTS_PER_INCH;
                            RobotLog.dd(TAG, "Movement in inches: " + inchesMoved);
                        } else if (zeroEncoderCount == 4)
                            RobotLog.dd(TAG, "All 4 encoder values are 0");
                          else
                            RobotLog.dd(TAG, "All 4 encoder values do not have the same sign");

                        logEncoderValues();
                        if (unrestrictedTest)
                            RobotLog.dd(TAG, "End report " + recordingNumber);
                        else
                            RobotLog.dd(TAG, "End report for " + teleTracLabels.get(recordingNumber));

                        // If the run is not an unrestricted test and we've reached the end of the
                        // collection of labels, stop here.
                        recordingNumber++;
                        if (!unrestrictedTest && (recordingNumber == teleTracLabels.size()))
                            requestOpModeStop();

                        // Return to teleop default.
                        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        robot.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        robot.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    }
                }
            }
        } catch (
                AutonomousRobotException arx)

        {
            LCHSLogFatalError.logFatalError(this, arx.getTag(), arx.getMessage());
        }
    }

    // Get the values from the encoders and store them in a collection in the order of
    // left front, left back, rright front, right back.
    private void getEncoderValues() {
        encoderValues.clear();
        encoderValues.add(robot.leftFront.getCurrentPosition());
        encoderValues.add(robot.leftBack.getCurrentPosition());
        encoderValues.add(robot.rightFront.getCurrentPosition());
        encoderValues.add(robot.rightBack.getCurrentPosition());

        // As a side effect, count up the number of positive and negative encodeer values.
        zeroEncoderCount = 0;
        positiveEncoderCount = 0;
        negativeEncoderCount = 0;
        for (Integer i : encoderValues) {
            if (i == 0)
                zeroEncoderCount++;
            else if (i > 0)
                positiveEncoderCount++;
            else
                negativeEncoderCount++;
        }
    }

    // The encoder values in the collection must be in the order of left front, left back,
    // rright front, right back.
    private void logEncoderValues() {
        if (encoderValues.size() != 4)
            throw new AutonomousRobotException(TAG, "Number of enocder values should be 4 but is " + encoderValues.size());

        RobotLog.dd(TAG, "Enocder values LF, LB, RF, RB: " +
                encoderValues.get(0) + " " +
                encoderValues.get(1) + " " +
                encoderValues.get(2) + " " +
                encoderValues.get(3));
    }

    private double getIMUHeading() {
        Orientation angles = robot.imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return (AngleUnit.DEGREES.normalize(angles.firstAngle));
    }

}
