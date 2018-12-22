package org.firstinspires.ftc.teamcode;

public class LCHSValues {

    public enum Alliance {
        BLUE, RED, UNKNOWN
    }

    // For each autonomous OpMode registered on the Robot Controller phone there
    // must a value in the enum OpMode.
    public enum OpMode {
        RIGHT_SOLO, RIGHT_COOP, LEFT_COOP, LEFT_SOLO, TEST
    }

    public enum MineralVisionSystem {
        OPENCV, TENSORFLOW
    }

    public enum Mineral {
        GOLD, SILVER, UNKNOWN
    }

    enum MineralPosition {
        LEFT, CENTER, RIGHT, UNKNOWN
    };

}
