/**
 * Created by Trinity on 11/12/2018
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Iterator;
import java.util.LinkedHashMap;

@TeleOp(name = "LCHSCalibrate", group = "TeamCode")

public class LCHSCalibrate extends LinearOpMode {

    // Define class members
    LCHSHardwareMap robot = null;

    enum DeviceType // for determining selected device type
    {
        MOTOR,
        SERVO,
        ANALOG_INPUT,
        NOT_SELECTED
    }

    int selectedDeviceIndex = 0;
    DeviceType selectedDeviceType = DeviceType.NOT_SELECTED;
    Servo selectedServo = null;
    double servoPosition = 0.0;
    DcMotor motor = null;
    int motorIncrement = 0;
    AnalogInput analogInput = null;


    @Override
    public void runOpMode() {

        // Identify the program
        telemetry.addData("Callibrate Devices Test", 0);
        telemetry.update();

        // Setup hardware map
        robot = new LCHSHardwareMap(hardwareMap, true);

        // Add in our devices
        LinkedHashMap<DcMotor, String> motorDevices = new LinkedHashMap<>();
        LinkedHashMap<Servo, String> servoDevices = new LinkedHashMap<>();
        LinkedHashMap<AnalogInput, String> analogInputDevices = new LinkedHashMap<>();

        motorDevices.put(robot.leftFront, "Left Front Wheel Motor");
        motorDevices.put(robot.rightFront, "Right Front Wheel Motor");
        motorDevices.put(robot.leftBack, "Left Back Wheel Motor");
        motorDevices.put(robot.rightBack, "Right Back Wheel Motor");
        motorDevices.put(robot.boom, "Boom Extend Motor");
        motorDevices.put(robot.tilt, "Boom Tilt Motor");
        motorDevices.put(robot.intakeLeft, "Left Intake Motor");
        motorDevices.put(robot.intakeRight, "Right Intake Motor");
        servoDevices.put(robot.hookServo, "Hook Servo");
        servoDevices.put(robot.gateLeft, "Left Intake Gate Servo");
        servoDevices.put(robot.gateRight, "Right Intake Gate Servo");
        // analogInputDevices.put(robot.tiltSwitch, "Tilt Switch");

        // Set brake for arm motors
        robot.boom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.tilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Initiate buttons
        Button buttonX = new Button("(X)", "Select Servo");
        Button buttonY = new Button("(Y)", "Select Analog Input");
        Button buttonA = new Button("(A)", "Select Motor");
        Button buttonB = new Button("(B)", "Deselect Device Type");
        Button bumperLeft = new Button("(LB)", "Device Index -1");
        Button bumperRight = new Button("(RB)", "Device Index +1");
        Button dpadUp = new Button("DPad Up", "Increment Value 10");
        Button dpadDown = new Button("DPad Down", "Decrement Value 10");
        Button dpadLeft = new Button("DPad Left", "Decrement Value 1");
        Button dpadRight = new Button("Dpad Right", "Increment Value 1");

        // Wait for the start button
        waitForStart();

        while (opModeIsActive()) {

            // Update button state
            buttonX.updateState(gamepad1.x);
            buttonY.updateState(gamepad1.y);
            buttonA.updateState(gamepad1.a);
            buttonB.updateState(gamepad1.b);
            bumperLeft.updateState(gamepad1.left_bumper);
            bumperRight.updateState(gamepad1.right_bumper);
            dpadLeft.updateState(gamepad1.dpad_left);
            dpadUp.updateState(gamepad1.dpad_up);
            dpadDown.updateState(gamepad1.dpad_down);
            dpadRight.updateState(gamepad1.dpad_right);


            // ---------- Select Device ----------
            if (buttonX.isDown()) {
                if (selectedServo != null) servoPosition = selectedServo.getPosition();
                else servoPosition = 0.5;
                selectedDeviceType = DeviceType.SERVO;
            } else if (buttonY.isDown()) {
                selectedDeviceType = DeviceType.ANALOG_INPUT;
            } else if (buttonA.isDown()) {
                motorIncrement = 0;
                selectedDeviceType = DeviceType.MOTOR;
            } else if (buttonB.isDown()) {
                selectedDeviceType = DeviceType.NOT_SELECTED;
                selectedDeviceIndex = 0;
            }

            // Go through devices by incrementing/decrementing index
            if (bumperRight.isDown()) {
                selectedDeviceIndex++;
                motorIncrement = 0;
            } else if (bumperLeft.isDown()) {
                selectedDeviceIndex--;
                if (selectedDeviceIndex < 0) selectedDeviceIndex = 0;
                motorIncrement = 0;
            }

            if (selectedDeviceType == DeviceType.MOTOR) {
                if (motor != null) motor.setPower(0.0); // turn off power for other motor
                Iterator<DcMotor> itr = motorDevices.keySet().iterator();
                // iterate over list of motors to find motor of index nth
                if (selectedDeviceIndex > motorDevices.size() - 1) selectedDeviceIndex = 0;
                for (int i = 0; itr.hasNext(); i++) {
                    motor = itr.next();
                    if (i == selectedDeviceIndex) break;
                }
            } else if (selectedDeviceType == DeviceType.SERVO) {
                Iterator<Servo> itr = servoDevices.keySet().iterator();
                // iterate over list of servos to find servo of index nth
                if (selectedDeviceIndex > servoDevices.size() - 1) selectedDeviceIndex = 0;
                for (int i = 0; itr.hasNext(); i++) {
                    selectedServo = itr.next();
                    if (i == selectedDeviceIndex) break;
                }
            } else if (selectedDeviceType == DeviceType.ANALOG_INPUT) {
                Iterator<AnalogInput> itr = analogInputDevices.keySet().iterator();
                // iterate over list of analog input devices to find device of index nth
                if (selectedDeviceIndex > analogInputDevices.size() - 1) selectedDeviceIndex = 0;
                for (int i = 0; itr.hasNext(); i++) {
                    analogInput = itr.next();
                    if (i == selectedDeviceIndex) break;
                }
            }

            // ---------- Increment Value ----------
            /*if (selectedDeviceType == DeviceType.MOTOR) {
                if (dpadUp.isUp())     motorIncrement += 10;
                if (dpadDown.isUp())   motorIncrement -= 10;
                if (dpadRight.isUp())  motorIncrement += 1;
                if (dpadLeft.isUp())   motorIncrement -= 1;
            } else */
            if (selectedDeviceType == DeviceType.SERVO) {
                if (dpadUp.isUp())     servoPosition += 0.10;
                if (dpadDown.isUp())   servoPosition -= 0.10;
                if (dpadRight.isUp())  servoPosition += 0.01;
                if (dpadLeft.isUp())   servoPosition -= 0.01;
            }


            // =============== Display Values ===============

            if (selectedDeviceType != DeviceType.NOT_SELECTED) {
                // if selected, add instructions
                telemetry.addData(bumperLeft.name, bumperLeft.action);
                telemetry.addData(bumperRight.name, bumperRight.action);
            }
            if (motor != null && selectedDeviceType == DeviceType.MOTOR) {
                // if motor
                /* //USING RUN TO POSITION
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setTargetPosition(motor.getCurrentPosition() + motorIncrement);
                motor.setPower(0.3);
                if (triggerRight > 0.0) motor.setPower(triggerRight);
                telemetry.addData("(Dpad)", "Increment Value [up:+10, down -10, right:+1, down: -1]");
                telemetry.addData("Motor Name: ", motorDevices.get(motor));
                telemetry.addData("Motor Position Increment Value: ", motorIncrement);
                telemetry.addData("Motor Position: ", motor.getCurrentPosition());
                */
                // USING JUST POWER
                double motorP = gamepad1.right_trigger - gamepad1.left_trigger;
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor.setPower(motorP);
                telemetry.addData("Motor Name: ", motorDevices.get(motor));
                telemetry.addData("Motor Power: ", motorP);
                telemetry.addData("Motor Position: ", motor.getCurrentPosition());
            }
            else if (selectedServo != null && selectedDeviceType == DeviceType.SERVO) {
                // if servo
                selectedServo.setPosition(servoPosition);
                telemetry.addData("(Dpad)", "Increment Value [up:+0.10, down -0.10, right:+0.01, down: -0.01]");
                telemetry.addData("Servo Name: ", servoDevices.get(selectedServo));
                telemetry.addData("Servo Position Value: ", "%.2f", servoPosition);
            }
            else if (analogInput != null && selectedDeviceType == DeviceType.ANALOG_INPUT) {
                // if analogInput
                telemetry.addData("Analog Input Name: ", analogInputDevices.get(analogInput));
                telemetry.addData("Analog Input Voltage: ", analogInput.getVoltage());
            }
            else {
                // if not selected, add instructions
                telemetry.addData("Select device type using", "Game Controller 1");
                telemetry.addData("Change power using", "(RT)+ (LT)-");
                telemetry.addData(buttonX.name, buttonX.action);
                telemetry.addData(buttonY.name, buttonY.action);
                telemetry.addData(buttonA.name, buttonA.action);
                telemetry.addData(buttonB.name, buttonB.action);
            }
            telemetry.update();

        }
    }


}
