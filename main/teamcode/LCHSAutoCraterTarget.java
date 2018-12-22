package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//** Make into thread with timeout and exitNow() method??
public class LCHSAutoCraterTarget implements LCHSAutoTarget {

    private final String TAG = "LCHSAutoCraterTarget";

    private final LCHSHardwareMap robot;
    private final double distanceToCrater;

     public LCHSAutoCraterTarget(LCHSHardwareMap pHardwareMap, double pDistanceToCrater) {
        robot = pHardwareMap;
        distanceToCrater = pDistanceToCrater;
    }

    @Override
    public boolean reachedTarget() {
        return robot.sensorRangeCrater.getDistance(DistanceUnit.INCH) <= distanceToCrater;
    }
}
