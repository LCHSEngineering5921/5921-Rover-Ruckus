package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

// Common fatal error handling.
public class LCHSLogFatalError {
    public static final void logFatalError(LinearOpMode pLinear, String pTag, String pErrorMessage) {
        RobotLog.ee(pTag, pErrorMessage);
        RobotLog.ee(pTag, "opModeIsActive() is " + pLinear.opModeIsActive());
        pLinear.telemetry.addData("LCHS" + " fatal error", pErrorMessage);
        pLinear.telemetry.update();
   }
}
