package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.util.RobotLog;

//** Make into thread with timeout and exitNow() method??
public class LCHSAutoDepotTarget implements LCHSAutoTarget {

    private final String TAG = "LCHSAutoDepotTarget";

    private final LCHSHardwareMap robot;
    private final float RedLowHSV;
    private final float RedHighHSV;
    private final float BlueLowHSV;
    private final float BlueHighHSV;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    private float hsvValues[] = {0F,0F,0F};
    private float hue;

    public LCHSAutoDepotTarget (LCHSHardwareMap pHardwareMap,
                    float pRedLowHSV, float pRedHighHSV, float pBlueLowHSV, float pBlueHighHSV) {
        robot = pHardwareMap;
        RedLowHSV = pRedLowHSV;
        RedHighHSV = pRedHighHSV;
        BlueLowHSV = pBlueLowHSV;
        BlueHighHSV = pBlueHighHSV;
    }

    @Override
    public boolean reachedTarget() {
        // convert the RGB values to HSV values.
        Color.RGBToHSV(robot.colorSensor.red() * 8, robot.colorSensor.green() * 8, robot.colorSensor.blue() * 8, hsvValues);
        hue = hsvValues[0];
        //RobotLog.dd(TAG, "Color hue " + hue);

        // HSV values are on a scale of 0 to 360 so it is possible to have a "low" value
        // of 350 and a "high" value of 10.
        /*
        if (RedLowHSV <= RedHighHSV) {
            if ((hue >= RedLowHSV) && (hue <= RedHighHSV))
                return true;
        } else {
            // Range crosses the boundary at 0.
            if (((hue >= RedLowHSV) && (hue <= 360)) || ((hue <= RedHighHSV) && (hue >= 0)))
                return true;
        }
        */

        boolean isRed = (RedLowHSV <= RedHighHSV) ? (hue >= RedLowHSV) && (hue <= RedHighHSV) :
                ((hue >= RedLowHSV) && (hue <= 360)) || ((hue <= RedHighHSV) && (hue >= 0));
        boolean isBlue = (hue >= BlueLowHSV) && (hue <= BlueHighHSV);

        return isRed || isBlue;
    }
}
