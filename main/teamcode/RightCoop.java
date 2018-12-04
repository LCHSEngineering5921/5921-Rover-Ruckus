package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Right Coop", group = "TeamCode")
//@Disabled
public class RightCoop extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        telemetry.setAutoClear(false); // keep our messages on the driver station

        // Q: Does this message appear on the driver station?
        telemetry.addData("LCHS Auto", "RIGHT_COOP in runOpMode()");
        telemetry.update();
        // A: Yes

        // LCHSAuto, the common class for all autonomous opmodes, needs
        // access to the public data fields and methods in LinearOpMode.
        try {
            LCHSAuto runAuto = new LCHSAuto(LCHSValues.OpMode.RIGHT_COOP, this);
            waitForStart();
            runAuto.runRobot();
        } catch (AutonomousRobotException arx) {
            LCHSLogFatalError.logFatalError(this, arx.getTag(), arx.getMessage());

            // Keep this comment here as a warning!
            // You can't call System.exit(1); because the robot controller then exits and
            // can't be restarted from the driver station. The result is that if you get
            // a fatal error during the autonomous run and exit you can't restart the robot
            // for the driver-controlled run.
        }
    }
}
