package org.firstinspires.ftc.teamcode;

public class LCHSValues {

    public enum Alliance {
        BLUE, RED
    }

    // For each autonomous OpMode registered on the Robot Controller phone there
    // must a value in the enum OpMode.
    public enum OpMode {
        RIGHT_SOLO, LEFT_SOLO, RIGHT_COOP, LEFT_COOP, TEST
        // RIGHT_MIN, RIGHT_MAX, LEFT_MIN, LEFT_MAX, TEST
    }

    public enum MineralVisionSystem {
        OPENCV, TENSORFLOW
    }

    public enum Mineral {
        GOLD, SILVER, NONE
    }

    enum MineralPosition {
        LEFT, CENTER, RIGHT, UNKNOWN
    };

}
