package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.CameraDevice;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import java.util.List;


public class LCHSAuto {

    private static final String TAG = "LCHSAuto";

    private LinearOpMode linearOpMode;
    private Configuration config;
    private LCHSValues.Alliance alliance = LCHSValues.Alliance.UNKNOWN;
    private LCHSValues.MineralPosition foundGoldPosition = LCHSValues.MineralPosition.UNKNOWN;

    private LCHSValues.OpMode autoOpMode;
    private boolean debugVerbose = false;

    // Device control.
    private LCHSHardwareMap robot;
    private LCHSAutoMotion motion;

    private double desiredHeading = 0.0; // always normalized

    private static final double CENTER_SKEW_THRESHOLD_DEGREES = 3.0;
    private static final float RED_HUE_LOW = 350.0f;
    private static final float RED_HUE_HIGH = 10.0f;
    private static final float BLUE_HUE_LOW = 190.0f;
    private static final float BLUE_HUE_HIGH = 210.0f;
    private final static double DISTANCE_TO_CRATER = 6.0;

    // Vision
    private LCHSAutoVisionOCV visionOCV;
    private LCHSAutoVisionTF visionTF;
    private LCHSAutoVuforia autoVuforia;

    // Vision system to use; from the configuration file.
    private LCHSValues.MineralVisionSystem mineralVision;

    // LCHSAuto must only be instantiated in runOpMode() for each autonomous opmode
    // because the hardwareMap is not initialized until then. However, instantiation
    // of LCHSAuto can be done before waitForStart().
    public LCHSAuto(LCHSValues.OpMode pOpMode, LinearOpMode pLinear) {

        RobotLog.dd(TAG, "Autonomous OpMode: " + pOpMode);
        linearOpMode = pLinear; // get access to LinearOpMode public fields and methods
        autoOpMode = pOpMode;
        debugVerbose = (autoOpMode == LCHSValues.OpMode.TEST) ? true : false; // extra logging in test mode

        // Read the configuration file for the selected opmode.
        config = new Configuration(autoOpMode);
        mineralVision = config.getMineralVisionSystem();

        // Initialize the hardware and the class that control robot motion.
        robot = new LCHSHardwareMap(pLinear.hardwareMap, true, true);
        motion = new LCHSAutoMotion(linearOpMode, robot, debugVerbose);

        // Initialize Vuforia.
        autoVuforia = new LCHSAutoVuforia(mineralVision, linearOpMode, robot, debugVerbose);

        // Initialize the selected vision system.
        if (mineralVision == LCHSValues.MineralVisionSystem.TENSORFLOW) {
            visionTF = new LCHSAutoVisionTF(linearOpMode, robot, autoVuforia, debugVerbose);
        } else {
            visionOCV = new LCHSAutoVisionOCV(autoVuforia, debugVerbose);
        }
    }

    // NOTE: from the ftc documentation - "Please do not swallow the InterruptedException,
    // as it is used in cases where the op mode needs to be terminated early."
    public void runRobot() throws InterruptedException {

        RobotLog.dd(TAG, "At start");
        RobotLog.dd(TAG, "Using " + mineralVision + " visionOCV system for mineral detection");

        // Safety check against ftc runtime initialization errors.
        // Make sure the opmode is still active.
        if (!linearOpMode.opModeIsActive())
            throw new AutonomousRobotException(TAG, "OpMode unexpectedly inactive in runRobot()");

        double currentHeading = motion.getIMUHeading();
        RobotLog.dd(TAG, "Desired heading: " + desiredHeading + ", Current heading: " + currentHeading);

        // Activate Tensorflow now if it is the vision system fselected or mineral detection.
        if (mineralVision == LCHSValues.MineralVisionSystem.TENSORFLOW) {
            visionTF.activate();
        }

        // Follow the choreography specified in the configuration file.
        switch (autoOpMode) {
            case RIGHT_SOLO:
            case RIGHT_COOP:
            case LEFT_COOP:
            case LEFT_SOLO:
            case TEST:
            {
                doAutoOpMode(config.getStepsInChoreography());
                break;
            }
            default: {
                throw new AutonomousRobotException(TAG, "Unrecognized OpMode: " + autoOpMode);
            }
        }

        // DONE
        if (mineralVision == LCHSValues.MineralVisionSystem.TENSORFLOW) {
            visionTF.shutdown();
        }

        RobotLog.dd(TAG, "Exiting LCHSAuto");
        linearOpMode.telemetry.addData("LCHSAuto ", "complete");
        linearOpMode.telemetry.update();
    }


    //===============================================================================================
    //===============================================================================================

    // Generic autonomous OpMode handler.
    // Process the commands (steps in a choreography) as they occur in the configuration file.
    // As needed, maintain context between commands, e.g. see targetJewelDirection.
    private void doAutoOpMode(Configuration.StepsInChoreography pSteps) throws InterruptedException {
        List<Configuration.Step> steps = pSteps.getSteps();
        boolean skipRemainingCommands = false;

          for (Configuration.Step currentStep : steps) {

              if (skipRemainingCommands) // premature termination?
                  break;

            // Make sure the opmode is still active.
            if (!linearOpMode.opModeIsActive())
                throw new AutonomousRobotException(TAG, "OpMode went inactive");

            Configuration.Command command = currentStep.getCommand();
            RobotLog.dd(TAG, "Autonomous command " + command);
            List<String> parameters = currentStep.getParameters();
            String parameter;

            switch (command) {

                // ----- Move the robot in a straight line without gyro correction -------
                case STRAIGHT: {
                    if (parameters.size() != 2)
                        throw new AutonomousRobotException(TAG, "Incorrect number of parameters");

                    double driveDistance = Double.parseDouble(parameters.get(0));
                    double drivePower = Double.parseDouble(parameters.get(1));
                    RobotLog.dd(TAG, "Drive straight without gyro: " + driveDistance + " inches, power: " + drivePower);
                    RobotLog.dd(TAG, "Desired gyro heading " + desiredHeading);
                    RobotLog.dd(TAG, "Current gyro heading " + motion.getIMUHeading());

                    motion.driveRobotWithoutGyro(LCHSAutoMotion.DriveMode.STRAIGHT, driveDistance, drivePower);
                    RobotLog.dd(TAG, "Straight done");
                    RobotLog.dd(TAG, "Current gyro heading " + motion.getIMUHeading());
                    break;
                }

                // ----- Move the robot in a straight line with gyro correction -------
                case STRAIGHT_GYRO: {
                    if (parameters.size() != 2)
                        throw new AutonomousRobotException(TAG, "Incorrect number of parameters");

                    double driveDistance = Double.parseDouble(parameters.get(0));
                    double drivePower = Double.parseDouble(parameters.get(1));
                    RobotLog.dd(TAG, "Drive straight with gyro: " + driveDistance + " inches, power: " + drivePower);
                    RobotLog.dd(TAG, "Desired gyro heading " + desiredHeading);
                    RobotLog.dd(TAG, "Current gyro heading " + motion.getIMUHeading());

                    motion.driveRobotWithGyro(driveDistance, drivePower, desiredHeading);
                    break;
                }

                // ----- Strafe the robot without gyro correction -------
                // Negative distance for a strafe left.
                case STRAFE: {
                    if (parameters.size() != 2)
                        throw new AutonomousRobotException(TAG, "Incorrect number of parameters");

                    double driveDistance = Double.parseDouble(parameters.get(0));
                    double drivePower = Double.parseDouble(parameters.get(1));
                    RobotLog.dd(TAG, "Strafe without gyro: " + driveDistance + " inches, power: " + drivePower);
                    RobotLog.dd(TAG, "Desired gyro heading " + desiredHeading);
                    RobotLog.dd(TAG, "Current gyro heading " + motion.getIMUHeading());

                    motion.driveRobotWithoutGyro(LCHSAutoMotion.DriveMode.STRAFE, driveDistance, drivePower);
                    break;
                }

                // -------------- Turn the robot --------------
                // This turn always uses the shortest distance to the target, even if the actual
                // turn direction ends up being the opposite of that requested.
                case TURN: {
                    if (parameters.size() != 1)
                        throw new AutonomousRobotException(TAG, "Incorrect number of parameters");

                    double turnDegrees = Double.parseDouble(parameters.get(0));
                    RobotLog.dd(TAG, "Turn " + turnDegrees + " degrees");
                    RobotLog.dd(TAG, "Desired heading before turn " + desiredHeading);
                    RobotLog.dd(TAG, "Current heading before turn " + motion.getIMUHeading());

                    desiredHeading = DEGREES.normalize(desiredHeading + turnDegrees); // set target heading
                    RobotLog.dd(TAG, "Desired heading after turn " + desiredHeading);

                    motion.gyroTurn(desiredHeading);
                    break;
                }

                // Turn the robot in the requested direction even if it is not the shortest distance
                // to the target.
                // 26-Nov-2018 Test program on Winfows Visual Studio 2017 AngleCalculator
                // has been verified to on the test cases:
                // 0 -180 1.0 (normalized 179 CCW, unnormalized -181 CW)
                // 0 180 -1.0 (normalized -179 CW, unnormalized 181 CW)
                case TURN_REQUESTED_DIRECTION: {
                    if (parameters.size() != 2)
                        throw new AutonomousRobotException(TAG, "Incorrect number of parameters");

                    double turnDegrees = Double.parseDouble(parameters.get(0));
                    double turnPower = Double.parseDouble(parameters.get(1));
                    double desiredHeadingBeforeTurn = desiredHeading;
                    double currentHeading = motion.getIMUHeading();
                    RobotLog.dd(TAG, "Turn " + turnDegrees + " degrees");
                    RobotLog.dd(TAG, "Desired heading before turn " + desiredHeading);
                    RobotLog.dd(TAG, "Current heading before turn " + currentHeading);

                    desiredHeading = DEGREES.normalize(desiredHeading + turnDegrees); // set target heading
                    RobotLog.dd(TAG, "Projected normalized desired heading after turn " + desiredHeading);
                    double normalizedTurn = motion.getError(desiredHeading, currentHeading, true);
                    RobotLog.dd(TAG, "Normalized turn " + normalizedTurn);

                    // Check if the direction of the requested turn is the same as the normalized direction.
                    // The sign of the normalized turn determines the direction of its turn.
                    double unnormalizedTurn = motion.getError(desiredHeadingBeforeTurn + turnDegrees, currentHeading, false);
                    RobotLog.dd(TAG, "Unnormalized turn " + unnormalizedTurn);

                    //** 12/11/2018 This really is broken. The correct way is to always use
                    // the unnormalized desired heading here but the downstream code cannot
                    // handle something like an unnormalized CW turn of -180 starting at -90
                    // because the desired heading would be -270 instead of a normalized +90.
                    // This code would also break on a requested turn of +180 from 0 because
                    // it would normalize to -180. We only dodge a bullet because we our
                    // requested turn is -180 from 0!!
                    RobotLog.dd(TAG, "Turn in requested direction " + unnormalizedTurn);
                    motion.gyroTurnFull(turnPower, LCHSAutoMotion.P_TURN_COEFF, desiredHeading, false);

                    break;
                }

                // -------------- Turn the robot with an angle, power level, and turn coefficient from the configuration file --------------
                case TURN_WITH_POWER: {
                    if (parameters.size() != 3)
                        throw new AutonomousRobotException(TAG, "Incorrect number of parameters");

                    double turnDegrees = Double.parseDouble(parameters.get(0));
                    double turnPower = Double.parseDouble(parameters.get(1));
                    double turnCoefficient = Double.parseDouble(parameters.get(2));
                    RobotLog.dd(TAG, "Turn " + turnDegrees + " degrees, power " + turnPower + ", coefficient " + turnCoefficient);
                    RobotLog.dd(TAG, "Desired heading before turn " + desiredHeading);
                    RobotLog.dd(TAG, "Current heading before turn " + motion.getIMUHeading());

                    desiredHeading = DEGREES.normalize(desiredHeading + turnDegrees); // set target heading
                    RobotLog.dd(TAG, "Desired heading after turn " + desiredHeading);

                    motion.gyroTurnFull(turnPower, turnCoefficient, desiredHeading, true);
                    break;
                }

                // Disengage and let the robot down off the rover.
                case LAND: {
                    motion.land();
                    RobotLog.dd(TAG, "Gyro heading after landing " + motion.getIMUHeading());
                    break;
                }

                // Tilt the boom to its carrying position.
                case TILT: {
                    if (parameters.size() != 1)
                        throw new AutonomousRobotException(TAG, "Incorrect number of parameters");

                    int targetTiltPosition = Integer.parseInt(parameters.get(0));
                    // Tilt the boom in a separate thread.
                    motion.tiltStart(targetTiltPosition);
                    break;
                }

                // Wait for the Tilt thread to complete.
                case TILT_WAIT: {
                    motion.tiltWait();
                    RobotLog.dd(TAG, "Synchronized with tilt thread");
                    break;
                }

                // Deposit our marker in the depot.
                case CLAIM: {
                    if (parameters.size() != 1)
                        throw new AutonomousRobotException(TAG, "Incorrect number of parameters");

                    int targetBoomPosition = Integer.parseInt(parameters.get(0));
                    motion.claim(targetBoomPosition);
                    break;
                }

                // Control the camera flash.
                case TORCH: {
                    parameter = parameters.get(0);
                    RobotLog.dd(TAG, "Parameter: " + parameter);
                    switch (parameter) {
                        case Configuration.TORCH_ON: {

                            // Turn on the light.
                            CameraDevice.getInstance().setFlashTorchMode(true);
                            break;
                        }
                        case Configuration.TORCH_OFF: {

                            // All done with the light.
                            CameraDevice.getInstance().setFlashTorchMode(false);
                            break;
                        }
                        default: {
                            throw new AutonomousRobotException(TAG, "Unrecognized parameter");
                        }
                    }
                    break;
                }

                case VUFORIA: {
                    parameter = parameters.get(0);
                    RobotLog.dd(TAG, "Parameter: " + parameter);
                    switch (parameter) {
                        case Configuration.VUFORIA_ON: {
                            autoVuforia.activateVuforia();
                            break;
                        }
                        case Configuration.VUFORIA_OFF: {
                            autoVuforia.deactivateVuforia();
                            break;
                        }
                        case Configuration.VUFORIA_READ: {
                            boolean foundVumark = autoVuforia.readVumark();
                            break;
                        }
                        default: {
                            throw new AutonomousRobotException(TAG, "Unrecognized parameter");
                        }
                    }
                    break;
                }

                // Test the sampling of a mineral with OpenCV.
                case FIND_GOLD_OPENCV: {
                    if (mineralVision != LCHSValues.MineralVisionSystem.OPENCV)
                        throw new AutonomousRobotException(TAG, "Vision system not set to OPENCV");

                    parameter = parameters.get(0); // where to find gold: LEFT, CENTER, RIGHT

                    if (visionOCV.findGoldMineralOpenCV(LCHSValues.MineralPosition.valueOf(parameter))) {
                        linearOpMode.telemetry.addData(TAG, "Found a gold mineral");
                        RobotLog.dd(TAG, parameter + " position: found a gold mineral");
                    } else {
                        linearOpMode.telemetry.addData(TAG, "No gold mineral found");
                        RobotLog.dd(TAG, parameter + " position: no gold mineral found");
                    }
                    linearOpMode.telemetry.update();
                    break;
                }

                // Test the sampling of a mineral with Tensorflow.
                case FIND_GOLD_TENSORFLOW: {
                    if (mineralVision != LCHSValues.MineralVisionSystem.TENSORFLOW)
                        throw new AutonomousRobotException(TAG, "Vision system not set to TENSORFLOW");

                    if (visionTF.findGoldMineralTensorflow()) {
                        linearOpMode.telemetry.addData(TAG, "Found a gold mineral");
                        RobotLog.dd(TAG, "Found a gold mineral");
                    } else {
                        linearOpMode.telemetry.addData(TAG, "No gold mineral found");
                        RobotLog.dd(TAG, "No gold mineral found");
                    }
                    linearOpMode.telemetry.update();
                    break;
                }

                // When the robot starts from either the red right or blue right positions
                // the closest corner of the field contains the depot.
                case SAMPLE_DEPOT: {
                    // Get the number of parameters for each of LEFT, CENTER, and RIGHT.
                    int numberOfParametersPerMineralPosition = parameters.size() / Configuration.mineralPositions.length;

                    // Find the gold mineral and turn towards the depot.
                    // sampleAndKnock returns the index to the next parameter.
                    int mineralParameterIndex = sampleAndKnock(currentStep, numberOfParametersPerMineralPosition);

                    // Turn towards the depot.
                    double turnDepotDegrees = Double.parseDouble(parameters.get(mineralParameterIndex++));
                    RobotLog.dd(TAG, "Turn towards the depot " + turnDepotDegrees);
                    RobotLog.dd(TAG, "Desired heading before turn " + desiredHeading);
                    RobotLog.dd(TAG, "Current heading before turn " + motion.getIMUHeading());

                    desiredHeading = DEGREES.normalize(desiredHeading + turnDepotDegrees); // set target heading
                    RobotLog.dd(TAG, "Desired heading after turn " + desiredHeading);

                    if (turnDepotDegrees != 0) {
                        motion.gyroTurn(desiredHeading);
                    }

                    // Approach the depot.
                    double approachDistance = Double.parseDouble(parameters.get(mineralParameterIndex++));
                    RobotLog.dd(TAG, "Approach depot distance " + approachDistance);

                    if (approachDistance != 0) {
                        motion.driveRobotWithGyro(approachDistance, LCHSAutoMotion.DRIVE_POWER, desiredHeading);
                        RobotLog.dd(TAG, "Approach done");
                    }

                    // Turn towards the depot for claim
                    double turnToDepotForClaim = Double.parseDouble(parameters.get(mineralParameterIndex++));
                    RobotLog.dd(TAG, "Turn to depot for claim " + turnToDepotForClaim);

                    if (turnToDepotForClaim != 0) {
                        desiredHeading = DEGREES.normalize(desiredHeading + turnToDepotForClaim); // set target heading
                        motion.gyroTurn(desiredHeading);
                        RobotLog.dd(TAG, "Turn done");
                    }

                    break;
                }

                // When the robot starts from either the red left or blue left positions
                // the closest corner of the field contains the crater.
                case SAMPLE_CRATER: {
                    // Get the number of parameters for each of LEFT, CENTER, and RIGHT.
                    int numberOfParametersPerMineralPosition = parameters.size() / Configuration.mineralPositions.length;

                    // Find the gold mineral and turn towards the depot.
                    // sampleAndKnock returns the index to the next parameter.
                    int mineralParameterIndex = sampleAndKnock(currentStep, numberOfParametersPerMineralPosition);
                    RobotLog.dd(TAG, "Robot is parked at the crater");

                    break;
                }

                // After sampling in front of the crater, proceed to the depot and make a claim
                // with our marker.
                // DEPOT_REMOTE parameter identifiers for moving the robot from the crater to the depot.
                /* public static final String[] depotRemoteParameters = {"vuforia",
                        "reverseC", "%d", "%d", // distance in inches, power
                        "turnV", "%d", "%d", "%d", // angle, power, turn coefficient
                        "approachV", "%d", "%d", // distance in inches, power
                        "turnD", "%d", "%d", "%d", // angle, power, turn coefficient
                        "approachD", "%d", "%d", // distance in inches, power
                        "knockC" // knock contingency
                }; */
                case DEPOT_REMOTE: {
                    // Get the number of parameters for each of LEFT, CENTER, and RIGHT.
                    int numberOfParametersPerMineralPosition = parameters.size() / Configuration.mineralPositions.length;

                    // Select which set of parameters to start with based on where the gold mineral
                    // was found.
                    if (foundGoldPosition == LCHSValues.MineralPosition.UNKNOWN) {
                        // Mineral recognition failed so abort the trip to the remote depot.
                        RobotLog.dd(TAG, "Aborting trip to the remote depot; stop autonomous run");
                        return;
                    }

                    // Set index for LEFT, CENTER, or RIGHT.
                    int mineralParameterIndex = numberOfParametersPerMineralPosition * foundGoldPosition.ordinal();

                    // The "vuforia" parameter is a double but we'll use it as a boolean:
                    // 0 means do not use Vuforia for navigation, non-zero means use Vuforia.
                    boolean useVuforiaNavigation = false;
                    if (Integer.parseInt(parameters.get(mineralParameterIndex++)) != 0) {
                        useVuforiaNavigation = true;
                        // Start tracking the data sets we care about.
                        autoVuforia.activateVuforia();
                    }

                    // From the crater back up towards the center position.
                    double reverseC = Double.parseDouble(parameters.get(mineralParameterIndex++));
                    double reverseCPower = Double.parseDouble(parameters.get(mineralParameterIndex++));
                    RobotLog.dd(TAG, "Reverse to center distance " + reverseC + " with power " + reverseCPower);

                    if (reverseC != 0) {
                        motion.driveRobotWithGyro(reverseC, reverseCPower, desiredHeading);
                        RobotLog.dd(TAG, "Reverse done");
                    }

                    // Turn towards the Vuforia target.
                    double turnV = Double.parseDouble(parameters.get(mineralParameterIndex++));
                    double turnVPower = Double.parseDouble(parameters.get(mineralParameterIndex++));
                    double turnVCoefficient = Double.parseDouble(parameters.get(mineralParameterIndex++));
                    RobotLog.dd(TAG, "Turn towards the Vuforia target " + turnV + " degrees");
                    RobotLog.dd(TAG, "With power " + turnVPower + " and coeff " + turnVCoefficient);

                    if (turnV != 0) {
                        desiredHeading = DEGREES.normalize(desiredHeading + turnV); // set target heading
                        motion.gyroTurnFull(turnVPower, turnVCoefficient, desiredHeading, true);
                        RobotLog.dd(TAG, "Turn done");
                    }
                    else {
                        // Special case: if the value of turnV is 0, this indicates that the
                        // trip to the remote depot should be abandoned, e.g. in the case of the
                        // gold mineral on the right, which might result in a collision with our
                        // alliance partner at the depot.
                        // The "knock contingency parameter is the last one in the collection.
                        RobotLog.dd(TAG, "DEPOT_REMOTE knock contingency in effect");

                        //** Park using boom
                        skipRemainingCommands = true; // execute no more commands
                        RobotLog.dd(TAG, "Skipping all remaining commands");
                        break;
                    }

                    // Approach the Vuforia target.
                    double approachV = Double.parseDouble(parameters.get(mineralParameterIndex++));
                    double approachVPower = Double.parseDouble(parameters.get(mineralParameterIndex++));
                    RobotLog.dd(TAG, "Approach Vuforia target; distance " + approachV + ". power " + approachVPower);

                    if (approachV != 0) {
                        motion.driveRobotWithGyro(approachV, approachVPower, desiredHeading);
                        RobotLog.dd(TAG, "Approach done");
                    }

                    //** If Vuforia navigation has been selected, here's where we use it to adjust our
                    // next turn and the length of our approach to the depot.
                    if (useVuforiaNavigation) {

                        //** For now just see if we can read the Vumark.
                        //** Later we'll need location and heading information.
                        useVuforiaNavigation = autoVuforia.readVumark();

                        // Don't need Vuforia tracking any more.
                        autoVuforia.deactivateVuforia();
                    }

                    // Turn towards the depot.
                    double turnD = Double.parseDouble(parameters.get(mineralParameterIndex++));
                    double turnDPower = Double.parseDouble(parameters.get(mineralParameterIndex++));
                    double turnDCoefficient = Double.parseDouble(parameters.get(mineralParameterIndex++));
                    RobotLog.dd(TAG, "Turn towards the depot " + turnD + " degrees");
                    RobotLog.dd(TAG, "With power " + turnDPower + " and coeff " + turnDCoefficient);

                    if (turnD != 0) {
                        desiredHeading = DEGREES.normalize(desiredHeading + turnD); // set target heading
                        motion.gyroTurnFull(turnDPower, turnDCoefficient, desiredHeading, true);
                        RobotLog.dd(TAG, "Turn done");
                    }

                    // Approach the depot.
                    double approachD = Double.parseDouble(parameters.get(mineralParameterIndex++));
                    double approachDPower = Double.parseDouble(parameters.get(mineralParameterIndex++));
                    RobotLog.dd(TAG, "Approach depot; distance " + approachD + ". power " + approachDPower);

                    if (approachD != 0) {
                        motion.driveRobotWithGyro(approachD, approachDPower, desiredHeading);
                        RobotLog.dd(TAG, "Approach done");
                    }

                    break;
                }

                // parameters: <double distance from side wall>,
                // <double power>, <double strafe proportional coefficient>, <int failsafe timeout
                    //** In competition opmodes this only works if the gold mineral is on the
                    // LEFT or in the CENTER. We also have to know our alliance for color
                    // recognition of the depot boundary.
                case TRACK_WALL_DEPOT: {
                    if (parameters.size() != 4)
                        throw new AutonomousRobotException(TAG, "Incorrect number of parameters");

                    double distanceFromSideWall = Double.parseDouble(parameters.get(0));
                    double power = Double.parseDouble(parameters.get(1));
                    double strafeCoeff = Double.parseDouble(parameters.get(2));
                    int failsafeTimeout = Integer.parseInt(parameters.get(3));

                    RobotLog.dd(TAG, "Desired gyro heading " + desiredHeading);
                    RobotLog.dd(TAG, "Distance from side wall " + distanceFromSideWall);
                    RobotLog.dd(TAG, "Power " + power);
                    RobotLog.dd(TAG, "Strafe proportional coefficient " + strafeCoeff);
                    RobotLog.dd(TAG, "Failsafe timeout " + failsafeTimeout);

                    robot.colorSensor.enableLed(true);

                    LCHSAutoDepotTarget depotTarget = new LCHSAutoDepotTarget(robot,
                            RED_HUE_LOW, RED_HUE_HIGH, BLUE_HUE_LOW, BLUE_HUE_HIGH);
                    motion.trackWall(desiredHeading, distanceFromSideWall, power, strafeCoeff,
                            failsafeTimeout, depotTarget);

                    robot.colorSensor.enableLed(false);

                    RobotLog.dd(TAG, "Wall tracking complete");
                    break;
                }

                case TRACK_WALL_CRATER: {
                    if (parameters.size() != 4)
                        throw new AutonomousRobotException(TAG, "Incorrect number of parameters");

                    double distanceFromSideWall = Double.parseDouble(parameters.get(0));
                    double power = Double.parseDouble(parameters.get(1));
                    double strafeCoeff = Double.parseDouble(parameters.get(2));
                    int failsafeTimeout = Integer.parseInt(parameters.get(3));

                    RobotLog.dd(TAG, "Desired gyro heading " + desiredHeading);
                    RobotLog.dd(TAG, "Distance from side wall " + distanceFromSideWall);
                    RobotLog.dd(TAG, "Power " + power);
                    RobotLog.dd(TAG, "Strafe proportional coefficient " + strafeCoeff);
                    RobotLog.dd(TAG, "Failsafe timeout " + failsafeTimeout);

                    LCHSAutoCraterTarget craterTarget = new LCHSAutoCraterTarget(robot, DISTANCE_TO_CRATER);
                    motion.trackWall(desiredHeading, distanceFromSideWall, power, strafeCoeff, failsafeTimeout,
                            craterTarget);
                    RobotLog.dd(TAG, "Wall tracking complete");
                    break;
                }


                // Pause the robot.
                // parameters: number of milliseconds to sleep (positive integer)
                case SLEEP: {
                    if (parameters.size() != 1)
                        throw new AutonomousRobotException(TAG, "Incorrect number of parameters");

                    int sleepValue = Integer.parseInt(parameters.get(0));
                    android.os.SystemClock.sleep(sleepValue);
                    break;
                }

                default: {
                    throw new AutonomousRobotException(TAG, "Unrecognized step in choreography");
                }
            }
        }
    }


    // Look for the gold mineral at each of the three sample positions.
    private int sampleAndKnock(Configuration.Step pStep, int pItemsInParameterSet) throws InterruptedException {

        Configuration.Command command = pStep.getCommand();
        List<String> parameters = pStep.getParameters();
        int itemsInParameterSet = pItemsInParameterSet;
        LCHSValues.MineralPosition lastSearchForMineral = LCHSValues.MineralPosition.CENTER;
        boolean foundGold = false;

        // Set the index into the List of parameters.
        // itemsInParameterSet is the number of parameters that belong to each of LEFT, CENTER, and RIGHT.
        int mineralParameterIndex = itemsInParameterSet * LCHSValues.MineralPosition.CENTER.ordinal();

        // Make sure that the robot is not skewed from previous steps.
        double degreeDifference = DEGREES.normalize(desiredHeading - motion.getIMUHeading());
        if (Math.abs(degreeDifference) >= CENTER_SKEW_THRESHOLD_DEGREES) {
            RobotLog.dd(TAG, "De-skewing before CENTER mineral");
            RobotLog.dd(TAG, "Desired heading " + desiredHeading);
            RobotLog.dd(TAG, "Current heading before turn " + motion.getIMUHeading());

            motion.gyroTurn(desiredHeading);
        }

        // Turn towards the CENTER sample.
        double turnDegreesC = Double.parseDouble(parameters.get(mineralParameterIndex));

        RobotLog.dd(TAG, "Turn towards CENTER mineral " + turnDegreesC);
        RobotLog.dd(TAG, "Desired heading before turn " + desiredHeading);
        RobotLog.dd(TAG, "Current heading before turn " + motion.getIMUHeading());

        desiredHeading = DEGREES.normalize(desiredHeading + turnDegreesC); // set target heading
        RobotLog.dd(TAG, "Desired heading after turn " + desiredHeading);

        if (turnDegreesC != 0) {
            motion.gyroTurn(desiredHeading);
        }

        // In all cases give the camera time to settle.
        // It's hard to prove but it seems as though Tensorflow is sometimes processing
        // residual images. Are they from the internal Vuforia queue?
        android.os.SystemClock.sleep(1000);

        // At this point the robot should not be skewed but it may be shifted left or right.
        // Right now we just have to live with this.
        RobotLog.dd(TAG, "Current heading " + motion.getIMUHeading());
        if (mineralVision == LCHSValues.MineralVisionSystem.OPENCV)
            foundGold = visionOCV.findGoldMineralOpenCV(LCHSValues.MineralPosition.CENTER);
        else foundGold = visionTF.findGoldMineralTensorflow();

        if (!foundGold) { // gold at center?
            // No, turn towards the LEFT sample.
            mineralParameterIndex = itemsInParameterSet * LCHSValues.MineralPosition.LEFT.ordinal(); // should be 0
            double turnDegreesL = Double.parseDouble(parameters.get(mineralParameterIndex));
            RobotLog.dd(TAG, "Turn towards LEFT mineral " + turnDegreesL);
            RobotLog.dd(TAG, "Desired heading before turn " + desiredHeading);
            RobotLog.dd(TAG, "Current heading before turn " + motion.getIMUHeading());

            desiredHeading = DEGREES.normalize(desiredHeading + turnDegreesL); // set target heading
            RobotLog.dd(TAG, "Desired heading after turn " + desiredHeading);

            if (turnDegreesL != 0) {
                motion.gyroTurn(desiredHeading);
                android.os.SystemClock.sleep(500); // settle for the camera
            }

            lastSearchForMineral = LCHSValues.MineralPosition.LEFT;
            if (mineralVision == LCHSValues.MineralVisionSystem.OPENCV)
                foundGold = visionOCV.findGoldMineralOpenCV(LCHSValues.MineralPosition.LEFT);
            else foundGold = visionTF.findGoldMineralTensorflow();

            if (!foundGold) {
                // Turn towards the RIGHT sample.
                mineralParameterIndex = itemsInParameterSet * LCHSValues.MineralPosition.RIGHT.ordinal();
                double turnDegreesR = Double.parseDouble(parameters.get(mineralParameterIndex));

                // Since we've already turned left we have to undo that turn.
                turnDegreesR += -turnDegreesL;
                RobotLog.dd(TAG, "Turn towards the RIGHT mineral " + turnDegreesR);
                RobotLog.dd(TAG, "Desired heading before turn " + desiredHeading);
                RobotLog.dd(TAG, "Current heading before turn " + motion.getIMUHeading());

                desiredHeading = DEGREES.normalize(desiredHeading + turnDegreesR); // set target heading
                RobotLog.dd(TAG, "Desired heading after turn " + desiredHeading);

                if (turnDegreesR != 0) {
                    motion.gyroTurn(desiredHeading);
                    android.os.SystemClock.sleep(500); // settle for the camera
                }

                lastSearchForMineral = LCHSValues.MineralPosition.RIGHT;
                if (mineralVision == LCHSValues.MineralVisionSystem.OPENCV)
                    foundGold = visionOCV.findGoldMineralOpenCV(LCHSValues.MineralPosition.RIGHT);
                else foundGold = visionTF.findGoldMineralTensorflow();
            }
        }

        // If we haven't found the gold mineral anywhere that means we are pointing to the
        // RIGHT. Turn back to the CENTER position and just plow ahead.
        if (foundGold) {
            RobotLog.dd(TAG, "Found gold at " + lastSearchForMineral + " position");
            foundGoldPosition = lastSearchForMineral;
        } else {
            RobotLog.dd(TAG, "Did not find the gold mineral anywhere!");
            foundGoldPosition = LCHSValues.MineralPosition.UNKNOWN;

            // Since we've already turned RIGHT we have to undo that turn.
            double turnDegreesR = Double.parseDouble(parameters.get(mineralParameterIndex));
            turnDegreesR = -turnDegreesR;
            RobotLog.dd(TAG, "Turn back towards the CENTER mineral " + turnDegreesR);
            RobotLog.dd(TAG, "Desired heading before turn " + desiredHeading);
            RobotLog.dd(TAG, "Current heading before turn " + motion.getIMUHeading());

            desiredHeading = DEGREES.normalize(desiredHeading + turnDegreesR); // set target heading
            RobotLog.dd(TAG, "Desired heading after turn " + desiredHeading);

            if (turnDegreesR != 0) {
                motion.gyroTurn(desiredHeading);
            }

            mineralParameterIndex = itemsInParameterSet * LCHSValues.MineralPosition.CENTER.ordinal(); // reset to CENTER
        }

        // Now facing a (hopefully gold) mineral.
        // Make a "post recognition turn" to avoid other minerals.
        mineralParameterIndex++; // all cases: advance to postRT parameter
        double postRecognitionTurn = Double.parseDouble(parameters.get(mineralParameterIndex));
        RobotLog.dd(TAG, "Post recognition turn " + postRecognitionTurn);
        RobotLog.dd(TAG, "Turn " + postRecognitionTurn);
        RobotLog.dd(TAG, "Desired heading before turn " + desiredHeading);
        RobotLog.dd(TAG, "Current heading before turn " + motion.getIMUHeading());

        desiredHeading = DEGREES.normalize(desiredHeading + postRecognitionTurn); // set target heading
        RobotLog.dd(TAG, "Desired heading after turn " + desiredHeading);

        if (postRecognitionTurn != 0) {
            motion.gyroTurn(desiredHeading);
        }

        // Knock the gold mineral off its taped area.
        mineralParameterIndex++; // all cases: advance to knock_distance parameter
        int targetBoomPosition = Integer.parseInt(parameters.get(mineralParameterIndex));
        RobotLog.dd(TAG, "Knock gold mineral " + targetBoomPosition);

        motion.knockUsingBoom(targetBoomPosition);
        RobotLog.dd(TAG, "Knocked off the gold mineral");

        // All done with sampling. If we are using Tensorflow, shut down now.
        if ((mineralVision == LCHSValues.MineralVisionSystem.TENSORFLOW)) {
            visionTF.shutdown();
        }

        return ++mineralParameterIndex; // advance to next parameter and return
    }
}

