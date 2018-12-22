package org.firstinspires.ftc.teamcode;//!!!!!!!!!package org.firstinspires.ftc.teamcode;

/**
 * Created by Dennis on 10/25/2017.
 * Updated by Trinity on 11/10/2018
 */

/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "LCHS TeleOp", group = "TeamCode")
//@Disabled
public class LCHSTeleOp extends LinearOpMode {

    // Declare hardwareMap object
    private LCHSHardwareMap robot = null;

    // The IMU sensor object
    // 2018-19 Do not use IMU in TeleOp because of initialization problems.
    // See notes in LCHSHardware.
    // BNO055IMU imu;

    //===============================================================================

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private static final double TURN_SENSITIVITY = 0.5;

    private static final double TURN_FINE_TUNE_MULT = 0.5;
    private static final double STRAFE_FINE_TUNE_MULT = 0.5;

    private static final double TILT_AUTOMOVE_POWER = 0.5;
    private static final double BOOM_AUTOMOVE_POWER = 0.25;

    private static final double BOOM_SENSITIVITY = 0.5;
    private static final double INTAKE_SENSITIVITY = 1.0;

    private static final int GOLD_TILT_OUTTAKE_POSITION = 2500;
    private static final int GOLD_BOOM_OUTTAKE_POSITION = -1500;

    private static final int SILVER_TILT_OUTTAKE_POSITION = 2600;
    private static final int SILVER_BOOM_OUTTAKE_POSITION = -1250;

    private static final double TILT_ADJUST_WEIGHT = 0.4;
    private static final double TILT_POWER_MINIMUM = 0.8;


    @Override
    public void runOpMode() {
        try {

            // Initialize the motors, servos and sensors
            robot = new LCHSHardwareMap(hardwareMap, false, false);


            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            // 2018-19 imu = robot.imu; // may be null if there was an error in IMU initialization

            // Gyro Assist Toggle
            // 2018-19 boolean gyroAssist = false;

            // 2018-19 Get the PID controller setup
            //PID pid = new PID(0.05);
            //if (imu != null)
            //    pid.setHeading(imu);

            // Setup a variable for each driveX wheel to save power level for telemetry
            double rightFrontP;
            double leftFrontP;
            double rightBackP;
            double leftBackP;

            double intakePL;
            double intakePR;

            // boom correction
            int targetBoomPos = 0;
            boolean targetBoomHold = false;

            // tilt correction
            double tiltP = 0.0;
            int targetTiltPos = 0;
            boolean targetTiltHold = false;

            Button tiltAutoAdjust = new Button("(X)", "Toggle Tilt Auto Adjust");
            boolean doTiltAutoAdjust = false;
            double tiltPowerAdjustMultiplier = 1.0;

            boolean endGame = false;

            Button hookLock = new Button("(Y)", "Toggle Hook Servo");
            boolean hookLockIsOpen = true;

            // ***** Accel correction hack *****
            double accelSum = 0.0;
            double strafeAccel = 0.0;


            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            runtime.reset();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {

                // ---------- Set Zero Power Behavior to BRAKE ----------
                robot.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                robot.boom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.tilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                /* When the right stick on the game controller is pushed forward,
                 gamepad.right_stick_y returns a negative number. The left motor
                 is set to REVERSE, meaning that positive power will turn the
                 wheel counter-clockwise and move the robot forward, so we invert
                 the value from right_stick_y to get a positive value. */

                /* When the right stick on the game controller is pushed to the left,
                 gamepad.right_stick_x returns a negative number. The left motor
                 is set to REVERSE, meaning that positive power will turn the
                 wheel clockwise and cause the robot to strafe to the left. So in
                 order to strafe left when the stick is pushed to the left, we have
                 to invert the value of right_stick_x to make it positive. */

                // NOTE: this is the behavior when the front wheels are toed in and
                // the rear wheels are toed out, which is the case for 2018-2019.

                // ANOTHER NOTE: on sticks, up gives negative and down gives positive

                // ---------- DRIVER 1 INPUT ----------
                double driveX = -gamepad1.right_stick_x;
                double driveY = -gamepad1.right_stick_y;

                boolean doFullPowerTurn = gamepad1.a;
                double turn = doFullPowerTurn ? gamepad1.left_stick_x : gamepad1.left_stick_x * TURN_SENSITIVITY;
                if (gamepad1.right_bumper || gamepad1.left_bumper) {
                    driveX *= STRAFE_FINE_TUNE_MULT;
                    driveY *= STRAFE_FINE_TUNE_MULT;
                    turn *= TURN_FINE_TUNE_MULT;
                }

                double liftUpR = gamepad1.right_trigger; // tilt boom for hooking
                double liftUpL = gamepad1.left_trigger; // two to confirm end game hook on

                boolean gyroTurnOn = gamepad1.x;
                boolean gyroTurnOff = gamepad1.y;

                // ---------- DRIVER 2 INPUT ----------
                // 11/29/18 reduce intake/outtake power; increase tilt power
                double boom = gamepad2.left_stick_y * BOOM_SENSITIVITY;

                boolean slowTilt = gamepad2.x;
                double tilt = slowTilt ? -gamepad2.right_stick_y * 0.5 : -gamepad2.right_stick_y;

                double intakeL = gamepad2.left_trigger * INTAKE_SENSITIVITY;
                double intakeR = gamepad2.right_trigger * INTAKE_SENSITIVITY;
                double outtakeL = gamepad2.left_bumper ? (1.0 * INTAKE_SENSITIVITY) : 0.0;
                double outtakeR = gamepad2.right_bumper ? (1.0 * INTAKE_SENSITIVITY) : 0.0;

                tiltAutoAdjust.updateState(gamepad2.x);
                hookLock.updateState(gamepad2.y);

                boolean goToSilverOutPosition = gamepad2.a;
                boolean goToGoldOutPosition = gamepad2.b;

/*
2018-19 Do not use the IMU during TeleOp -- see LCHSHardware for comments.
                // ---------- Drive with GYRO (PID) ----------
                //Adjust heading, if necessary'
                if (gyroAssist && (imu != null)) {
                    if (turn != 0.0) {
                        pid.setHeading(imu);
                    } else {

                        turn = pid.correctHeading(imu);
                        // ***** Strafe correction hack  --Dennis 11/22/18 *****
                        if (driveY != 0.0) {
                            strafeAccel = imu.getLinearAcceleration().xAccel;
                            double accelIntegratedGain = 0.03;
                            double strafeProportionalGain = 0.3;
                            accelSum = accelSum + strafeAccel * accelIntegratedGain;
                            driveX = driveX + ((strafeProportionalGain * strafeAccel) + (accelSum));
                        } else {
                            accelSum = 0.0;
                        }
                        // ***** End of strafe correction hack *****
                    }
                }
                //Turn gyro assist on or off
                if (gyroTurnOn && (imu != null)) {
                    gyroAssist = true;
                    pid.setHeading(imu);
                } else if (gyroTurnOff) {
                    gyroAssist = false;
                }
                */

                // 2018-2019
                // When strafing, left front and right back get the same power values
                // with the same sign, right front and left back get the same power
                // values with a sign opposite to that of left front, right back.

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


                // ---------- Calculate Intake Power ----------
                intakePL = Range.clip(intakeL - outtakeL, -1.0, 1.0);
                intakePR = Range.clip(intakeR - outtakeR, -1.0, 1.0);

                // Send calculated power to intake motors
                robot.intakeLeft.setPower(intakePL);
                robot.intakeRight.setPower(intakePR);


                // ---------- Boom control ----------
                if (boom == 0.0) { // if boom power is 0, hold the position
                    if (!targetBoomHold) {
                        targetBoomHold = true;
                        targetBoomPos = robot.boom.getCurrentPosition();
                        robot.boom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.boom.setTargetPosition(targetBoomPos);
                        robot.boom.setPower(BOOM_AUTOMOVE_POWER);
                    }
                } else { // else  give power from stick
                    targetBoomHold = false;
                    robot.boom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.boom.setPower(boom);
                }

                // ---------- Tilt control ----------
                if (tilt == 0.0 && !endGame) { // if tilt power is 0 AND not latched, hold the position
                    if (!targetTiltHold) {
                        targetTiltHold = true;
                        targetTiltPos = robot.tilt.getCurrentPosition();
                        robot.tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.tilt.setTargetPosition(targetTiltPos);
                        robot.tilt.setPower(TILT_AUTOMOVE_POWER);
                    }
                } else { // else give power from stick
                    targetTiltHold = false;
                    robot.tilt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    // Calculate needed power adjust depending on whether going with/against gravity
                    int currentTilt = robot.tilt.getCurrentPosition();
                    int clickDistanceToVertical = Math.abs(LCHSHardwareMap.TILT_VERTICAL_POSITION - currentTilt);
                    boolean goingUp = (tilt > 0.0 && currentTilt < LCHSHardwareMap.TILT_VERTICAL_POSITION) ||
                            (tilt < 0.0 && currentTilt > LCHSHardwareMap.TILT_VERTICAL_POSITION);

                    if (doTiltAutoAdjust) {
                        if (goingUp) {
                            tiltPowerAdjustMultiplier = clickDistanceToVertical / LCHSHardwareMap.TILT_VERTICAL_POSITION
                                    * TILT_ADJUST_WEIGHT + TILT_POWER_MINIMUM;
                        } else
                            tiltPowerAdjustMultiplier = TILT_POWER_MINIMUM; // else going down, low power
                    }

                    tiltP = Range.clip(tilt * tiltPowerAdjustMultiplier, -1.0, 1.0);
                    robot.tilt.setPower(tiltP);
                }

                if (liftUpL > 0.0 && liftUpR > 0.0) { // give Driver 1 tilt control to "lower" boom for hooking
                    endGame = true;
                    targetTiltHold = false;
                    robot.tilt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.tilt.setPower(-liftUpR); // tilt negative to "fold" robot
                }


                // ---------- Go To Position Buttons ----------
                // lazy button; goes to position for releasing mineral
                if ((goToSilverOutPosition || goToGoldOutPosition) && (tilt == 0.0 && boom == 0.0)) {
                    robot.tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.boom.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    if (goToSilverOutPosition) {
                        robot.tilt.setTargetPosition(SILVER_TILT_OUTTAKE_POSITION);
                        robot.boom.setTargetPosition(SILVER_BOOM_OUTTAKE_POSITION);
                    } else if (goToGoldOutPosition) {
                        robot.tilt.setTargetPosition(GOLD_TILT_OUTTAKE_POSITION);
                        robot.boom.setTargetPosition(GOLD_BOOM_OUTTAKE_POSITION);
                    }

                    robot.tilt.setPower(TILT_AUTOMOVE_POWER);
                    robot.boom.setPower(BOOM_AUTOMOVE_POWER);
                }


                // Buttons
                // ---------- Do Tilt Auto Adjust ----------
                if (tiltAutoAdjust.isDown()) doTiltAutoAdjust = !doTiltAutoAdjust;

                // ---------- Hook Servo Lock/Unlock ----------
                if (hookLock.isDown()) {
                    if (hookLockIsOpen)
                        robot.hookServo.setPosition(LCHSHardwareMap.HOOK_SERVO_CLOSED);
                    else
                        robot.hookServo.setPosition(LCHSHardwareMap.HOOK_SERVO_OPEN);
                    hookLockIsOpen = !hookLockIsOpen;
                }


                //======================================================================================

                // Show drive data on driver station phone

                telemetry.addData("Driver", 1);
                telemetry.addData("Turn", gamepad1.left_stick_x);
                //telemetry.addData("Gyro Assist", gyroAssist ? "ON" : "OFF");
                /*
                telemetry.addData("LF Wheel", leftFrontP);
                telemetry.addData("RF Wheel", rightFrontP);
                telemetry.addData("LB Wheel", leftBackP);
                telemetry.addData("RB Wheel", rightBackP);
                */

                telemetry.addData("Driver", 2);
                telemetry.addData("Tilt Auto Adjust", doTiltAutoAdjust ? "ON" : "OFF");
                telemetry.addData("Tilt Power", tiltP);
                telemetry.addData("Tilt Position", robot.tilt.getCurrentPosition());
                telemetry.addData("Boom Position", robot.boom.getCurrentPosition());

                telemetry.update();

            }

        } catch (AutonomousRobotException arx) {
            LCHSLogFatalError.logFatalError(this, arx.getTag(), arx.getMessage());
        }
    }


}
