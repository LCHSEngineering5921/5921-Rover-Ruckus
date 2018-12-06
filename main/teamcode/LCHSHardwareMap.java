package org.firstinspires.ftc.teamcode;

/**
 * Created by Dennis on 10/25/2017.
 * Last edited by Trinity on 11/7/2018
 */

import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.bosch.BNO055IMU;

// Common hardware definitions for LCHSAuto and LCHSTeleOp.
public class LCHSHardwareMap {

    private static final String TAG = "LCHSHardwareMap";

    HardwareMap hwMap;

    // Four mecanum drive motors
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    DcMotor tilt;
    DcMotor boom;

    CRServo intakeLeft;
    CRServo intakeRight;

    //Servo gateLeft; //** TODO: remove gate servos
    //Servo gateRight;
    Servo hookServo;

    // AnalogInput tiltSwitch;

    BNO055IMU imu; // Adafruit (Bosch) gyro

    public static final double HOOK_SERVO_OPEN = 0.8;
    public static final double HOOK_SERVO_CLOSED = 0.4;
    public static final double GATE_LEFT_SERVO_OPEN = 0.5;
    public static final double GATE_RIGHT_SERVO_OPEN = 0.5;
    public static final double GATE_LEFT_SERVO_CLOSED = 0.7;
    public static final double GATE_RIGHT_SERVO_CLOSED = 0.3;

    public static final int TILT_POS_AFTER_GRAVITY = 2200;
    public static final int TILT_POS_ROBOT_IS_LEVEL = 3700;
    public static final int TILT_VERTICAL_POSITION = 4250;

    LCHSHardwareMap(HardwareMap hwm, boolean pThrowOnIMUInitializationError) {
        hwMap = hwm;

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        // ---------- INITIALIZE HARDWARE VARIABLES ----------
        // Note that the strings used here as parameters go 'get'
        // must correspond to the names assigned in the robot configuration
        // (on the FTC Robot Controller app on the phone).

        // Motors
        leftFront = hwMap.get(DcMotor.class, "left_front");
        rightFront = hwMap.get(DcMotor.class, "right_front");
        leftBack = hwMap.get(DcMotor.class, "left_back");
        rightBack = hwMap.get(DcMotor.class, "right_back");

        tilt = hwMap.dcMotor.get("tilt");
        boom = hwMap.dcMotor.get("boom");

        // Continuous Rotation Servos
        intakeLeft = hwMap.get(CRServo.class, "intake_left");
        intakeRight = hwMap.get(CRServo.class, "intake_right");

        // Servos
        hookServo = hwMap.servo.get("hook");

        // Analog Input
        // tiltSwitch = hwMap.analogInput.get("tilt_switch");


        // 2018 - 2019
        // The leftFront motor when set to FORWARD turns clockwise with
        // positive power and moves the robot backwards. So if we set
        // the direction to REVERSE then we can apply positive power to
        // make the motor turn counter-clockwise and move the robot forward.
        // ANY CHANGES TO THE MOTOR DIRECTIONS will drastically affect both
        // TeleOp and Autonomous.

        // Motors
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        tilt.setDirection(DcMotor.Direction.FORWARD);
        boom.setDirection(DcMotor.Direction.FORWARD);

        // CRServos
        intakeLeft.setDirection(CRServo.Direction.FORWARD);
        intakeRight.setDirection(CRServo.Direction.FORWARD);

        // Servos
        hookServo.setDirection(Servo.Direction.FORWARD);

        // Set all motors to zero power
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        tilt.setPower(0);
        boom.setPower(0);

        hookServo.setPosition(HOOK_SERVO_CLOSED);


        // Initialize the IMU
        imu = hwMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "LCHS IMU init";
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";

        RobotLog.dd(TAG, "Call IMU initialization");
        imu.initialize(parameters);
        RobotLog.dd(TAG, "Return from IMU initialization");

        // 12/30/17 The ftc library internally retries the IMU if the status is not RUNNING_FUSION
        // so remove the retries here.
        BNO055IMU.SystemStatus localIMUStatus = imu.getSystemStatus();
        RobotLog.dd(TAG, "imu status after init " + localIMUStatus);
        if (localIMUStatus != BNO055IMU.SystemStatus.RUNNING_FUSION) {
            imu = null;
            if (pThrowOnIMUInitializationError)
                throw new AutonomousRobotException(TAG, "Failed to initialize IMU");
        }
    }
}
