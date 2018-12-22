package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by Dennis on 10/26/2017.
 */
// This class implements a simple PID controller

public class PID {
    double Kp = 0.0;
    double Ki = 0.0;
    double Kd = 0.0;
    double currentHeading = 0.0;
    Orientation angles;
    double err = 0.0;
    double lastErr = 0.0;
    double errSum = 0.0;
    long lastMilis = 0;

    /* 2018 - 2019 full pid not used
    PID (double p,double i,double d){
        Kp = -p; // 2018 - 2019 we inverted this
        Ki = i;
        Kd = d;
    }
    */

    // 2018 - 2019 Since we didn't use "i" or "d" last year and since we had to invert the sign nof "p"
    // to make pid work this year, simplify the constructor.
    PID (double p){
        Kp = -p; // 2018 - 2019 we inverted this
    }

    void setHeading(BNO055IMU imu){
        currentHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
    double correctHeading(BNO055IMU imu){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // Normalize to take care of the boundary at 180.0/-180.0.
        err = AngleUnit.DEGREES.normalize(currentHeading - angles.firstAngle);
        // previous: err = currentHeading - angles.firstAngle;
        // Proportional correction
        double pErr = err * Kp;
        //Integral correction
        long millis = System.currentTimeMillis() % 1000;
        double delta = (millis - lastMilis)/1000.0;
        lastMilis = millis;
        errSum =+ err * delta;
        if(errSum > 1.0/Ki) errSum = 1.0 / Kp; //** divide by zero!!
        double iErr = Ki * errSum;
        //Derivative correction
        double dErr = err - lastErr;
        lastErr = err;
        dErr = Kd * dErr * delta;
        //** do range clip here
        return Range.clip(pErr + iErr + dErr, -1.0, 1.0);
        //return pErr + iErr + dErr;
    }
}
