package org.firstinspires.ftc.teamcode;

/**
 * Created by phil on 1/15/17.
 */
// User-defined exception class that we can use to consolidate error handling.

public class AutonomousRobotException extends RuntimeException {

    private final String tag;

     public AutonomousRobotException(String pTag, String pErrorMessage) {
        super(pErrorMessage);
        tag = pTag;
    }

    public String getTag() {
        return tag;
    }
}
