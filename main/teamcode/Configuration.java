package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*

Read the configuration file RobotConfig.txt to set the runtime environment.

For each autonomous OpMode registered on the Robot Controller phone there
must be a value in the enum LCHSAuto.OpMode and there must be a single line
in the configuration file that begins with the string value of the OpMode's
enum.

One or more strings must follow the OpMode; the strings are grouped into
steps, each of which consists of a command to the robot followed by 0 or
more parameters. For example, for a robot with mecanum wheels the command
to move the robot might look like this:
STRAIGHT 12.0 0.5
where "STRAIGHT" is the command, 12.0 is the first operand (distance in inches),
and 0.5 is the second operand (power level).
The command to turn the robot in place might then be:
TURN 90.0 (counter-clockwise 90 degrees).

// During testing it is sometimes convenient to skip steps.
     // If a command in the configuration file is proceeded by an asterisk,
     // e.g. *BLOCK_LIFTER_MOTOR 5050, then parse the command parameters
     // but do not include the command and its parameters in the final
     // choreography. This way you can leave the orginal parameters in
     // place without having to change them to zero.

The actual commands and parameters for this year's robot are listed below.

An OpMode followed by one or more steps is called a choreography.
*/

public class Configuration {

    private static final String TAG = "Config";

    // Steps in a choreography
    public enum Command {
        STRAIGHT, // parameters: <(double) distance in inches (negative for reverse)>, <(double) power>
        STRAIGHT_GYRO, // parameters: <(double) distance in inches (negative for reverse)>, <(double) power>
        STRAFE, // parameters: <(double) distance in inches (negative for left)>, <(double) power>
        TURN, // parameters: <(double) angle (0.0 to 180.0 CCW, -0.0 to -180.0 CW)>
        TURN_REQUESTED_DIRECTION, // parameters: <(double) angle (0.0 to 180.0 CCW, -0.0 to -180.0 CW)>, <(double) power>
        TURN_WITH_POWER, // parameters: <(double) angle (0.0 to 180.0 CCW, -0.0 to -180.0 CW)>, <(double) power>, <(double> turn coefficient>
        LAND, // parameters: <none
        TILT, // parameters: <int clicks for tilt motor>
        TILT_WAIT, // parameters: <none>
        CLAIM, // parameters: <(int> clicks for boom target position>
        VISION, // parameters: <(enum) LCHSValues.MineralVisionSystem>
        FIND_GOLD_OPENCV, // parameters: <enum value LEFT, CENTER, or RIGHT> for testing
        FIND_GOLD_TENSORFLOW, // parameters <none>
        SAMPLE_DEPOT, // parameters: see SAMPLE parameter identifiers below
        SAMPLE_CRATER, // parameters: see SAMPLE parameter identifiers below
        DEPOT_REMOTE, // parameters: see DEPOT_REMOTE parameter identifiers below
        SLEEP, // parameters: <(positive int) number of milliseconds to sleep>
    }

    // Parameter identifiers

    // SAMPLE parameter identifiers for the three minerals.
    public static final String[] mineralPositions = {"[LEFT]", "[CENTER]", "[RIGHT]"};

    // FUTURE 1: add the data type after each parameter id (which would then have to be parsed out)
    // "incline double", "timeout int"
    // FUTURE 2: allow for data types and multiple parameters such as "incline double timeout int"
    // FUTURE 3: put the rules into an XML file
    // SAMPLE parameter identifiers for the red left and blue left starting positions.
    public static final String[] craterStartParameters = {"turn", "postRT",
            "knock", "turn_crater", "incline", "timeout"};

    // SAMPLE parameter identifiers for the red right and blue right starting positions.
    public static final String[] depotStartParameters = {"turn", "postRT",
    //        "knock", "turn_depot", "approach_depot"};
              "knock", "turn_depot", "approach_depot", "turn_claim"}; // 11/21/18 added turn_claim

    // DEPOT_REMOTE parameter identifiers for moving the robot from the crater to the depot.
    public static final String[] depotRemoteParameters = {"vuforia", "reverseC",
    "turnV", "approachV", "turnD", "approachD"};

    private List<String> tokens;
    private StepsInChoreography stepsInChoreography;

    // Default for the vision system to use. May be overridden by the VISION command.
    private LCHSValues.MineralVisionSystem mineralVisionSystem = LCHSValues.MineralVisionSystem.TENSORFLOW;


    public Configuration(LCHSValues.OpMode pOpMode) {

        LCHSValues.OpMode opMode = pOpMode;
        String opModeString = opMode.toString();

        // Make sure we can access the configuration file.
        if (!(isExternalStorageReadable() || isExternalStorageWritable()))
            throw new AutonomousRobotException(TAG, "Disaster: can't access file system");

        try {
            // Creates the file that it is going to read from the fileName string.
            File configFile = new File(Environment.getExternalStoragePublicDirectory(
                    Environment.DIRECTORY_PICTURES), "RobotConfig.txt");
            FileReader fr = new FileReader(configFile);
            BufferedReader br = new BufferedReader(fr);

            // Reads the file one line at a time, finds the first instance of white space,
            // matches the token to the left of the white space against the OpMode enum
            // value passed in to the constructor, and, on a successful match, saves the
            // remainder of the line since it contains the steps the robot is to take for
            // the OpMode.
            String line;
            boolean opModeFound = false;
            while ((line = br.readLine()) != null) {
                line = line.trim();

                // Allow blank lines between each opmode or at the end of the file.
                // But note that you cannot split a choreography across multiple lines.
                if (line.length() == 0)
                    continue;

                if (line.startsWith("//")) // allow comments
                    continue;

                // Split the line into tokens based on white space.
                tokens = new ArrayList(Arrays.asList(line.trim().split("\\s+"))); // tokenize on white space
                if (tokens.size() < 2)
                    throw new AutonomousRobotException(TAG, "Disaster: line in configuration does not contain at least 2 tokens");

                // See if the first token matches an OpMode.
                String firstToken = tokens.get(0);
                if (firstToken.equals(opModeString)) {
                    opModeFound = true;
                    break; // NOTE: stops on the first match
                }
            }
            br.close();
            if (!opModeFound) {
                throw new AutonomousRobotException(TAG, "Disaster: no OpMode found in configuration file");
            }
        } catch (FileNotFoundException ex) {
            throw new AutonomousRobotException(TAG, ex.getMessage());
        } catch (IOException ex) {
            throw new AutonomousRobotException(TAG, ex.getMessage());
        }

        // We don't need the OpMode anymore.
        tokens.remove(0);

        // Now determine the actual steps in the choreography.
        stepsInChoreography = new StepsInChoreography(tokens);
    }

    public StepsInChoreography getStepsInChoreography() {
        return stepsInChoreography;
    }

    // Return the vision system to use for detecting the color of minerals as set by the
    // VISION command. Return a default if no VISION command was encountered.
    public LCHSValues.MineralVisionSystem getMineralVisionSystem() {
        return mineralVisionSystem;
    }

    // Checks if external storage is available for read and write.
    private boolean isExternalStorageWritable() {
        String state = Environment.getExternalStorageState();
        if (Environment.MEDIA_MOUNTED.equals(state))
            return true;
        return false;
    }

    // Checks if external storage is available to at least read.
    private boolean isExternalStorageReadable() {
        String state = Environment.getExternalStorageState();
        if (Environment.MEDIA_MOUNTED.equals(state) ||
                Environment.MEDIA_MOUNTED_READ_ONLY.equals(state))
            return true;
        return false;
    }

    // Inner class for the steps in a choreography.
    class StepsInChoreography {

        private List<Step> steps = new ArrayList<>();

        StepsInChoreography(List<String> tokens) {

            // Parse and convert each command and its parameters.
            int numTokens = tokens.size();
            int tokenIndex = 0;
            String parameter;
            while (true) {
                String commandString = tokens.get(tokenIndex);

                // Check for the indication that a command should be skipped.
                boolean skipCommand = false;
                if (commandString.charAt(0) == '*') {
                    skipCommand = true;
                    // Remove the comment character (asterisk).
                    commandString = new StringBuilder(commandString).deleteCharAt(0).toString();
                }

                Command command = Command.valueOf(commandString); // invalid enum throws IllegalArgumentException
                List<String> parameters = new ArrayList<>();

                switch (command) {
                    // These commands have two parameters, both of which are doubles for distance or angle and power:
                    // (double) distance in inches (negative for reverse), (double) power
                    // (double) angle (0.0 to 180.0 CCW, -0.0 to -180.0 CW)>, (double) power
                    case STRAIGHT:
                    case STRAIGHT_GYRO:
                    case STRAFE:
                        // case STRAFE_GYRO:
                     case TURN_REQUESTED_DIRECTION:
                    {
                        tokenIndex++; // advance to parameter
                        if (tokenIndex == numTokens)
                            throw new AutonomousRobotException(TAG, "Missing parameter in choreography after: " + commandString);

                        parameter = tokens.get(tokenIndex);
                        parameters.add(parameter);
                        Double.parseDouble(parameter); // check number format

                        tokenIndex++; // advance to second parameter
                        if (tokenIndex == numTokens)
                            throw new AutonomousRobotException(TAG, "Missing second parameter in choreography after: " + commandString);

                        parameter = tokens.get(tokenIndex);
                        parameters.add(parameter);
                        double power = Double.parseDouble(parameter); // check number format
                        if ((power < -1.0) || (power > 1.0))
                            throw new AutonomousRobotException(TAG, "Power value out of range");

                        if (!skipCommand)
                            steps.add(new Step(command, parameters));
                        break;
                    }

                    // Turn the robot.
                    // parameters: <(double) angle (0.0 to 180.0 CCW, -0.0 to -180.0 CW)>
                    case TURN: {
                        tokenIndex++; // advance to parameter
                        if (tokenIndex == numTokens)
                            throw new AutonomousRobotException(TAG, "Missing parameter in choreography after: " + commandString);

                        parameter = tokens.get(tokenIndex);
                        parameters.add(parameter);
                        double angle = Double.parseDouble(parameter); // check number format
                        if ((angle > 180.0) || (angle < -180.0))
                            throw new AutonomousRobotException(TAG, "Turn angle out of range");

                        if (!skipCommand)
                            steps.add(new Step(command, parameters));
                        break;
                    }

                    // TURN_WITH_POWER: for specifying the exact amount of power and the proportional coefficient for a turn.
                    // parameters: <(double) angle (0.0 to 180.0 CCW, -0.0 to -180.0 CW)>, <(double) power>, <(double> turn coefficient>
                    case TURN_WITH_POWER: {
                        tokenIndex++; // advance to angle parameter
                        if (tokenIndex == numTokens)
                            throw new AutonomousRobotException(TAG, "Missing parameter in choreography after: " + commandString);

                        parameter = tokens.get(tokenIndex);
                        parameters.add(parameter);
                        Double.parseDouble(parameter); // check number format

                        tokenIndex++; // advance to power parameter
                        if (tokenIndex == numTokens)
                            throw new AutonomousRobotException(TAG, "Missing second parameter in choreography after: " + commandString);

                        parameter = tokens.get(tokenIndex);
                        parameters.add(parameter);
                        double power = Double.parseDouble(parameter); // check number format
                        if ((power < -1.0) || (power > 1.0))
                            throw new AutonomousRobotException(TAG, "Power value out of range");

                        tokenIndex++; // advance to coefficient parameter
                        if (tokenIndex == numTokens)
                            throw new AutonomousRobotException(TAG, "Missing third parameter in choreography after: " + commandString);

                        parameter = tokens.get(tokenIndex);
                        parameters.add(parameter);

                        if (!skipCommand)
                            steps.add(new Step(command, parameters));
                        break;
                    }

                     // TILT // parameters: <int clicks for tilt motor>
                    // CLAIM, // parameters: <(int> clicks for boom target position>
                    case TILT:
                    case CLAIM: {
                        tokenIndex++; // advance to parameter
                        if (tokenIndex == numTokens)
                            throw new AutonomousRobotException(TAG, "Missing parameter in choreography after: " + commandString);

                        parameter = tokens.get(tokenIndex);
                        parameters.add(parameter);
                        Integer.parseInt(parameter); // check number format

                        if (!skipCommand)
                            steps.add(new Step(command, parameters)); // parameters will be empty
                        break;
                    }

                    // parameters: <none>
                    case LAND:
                    case TILT_WAIT: {
                        if (!skipCommand)
                            steps.add(new Step(command, parameters)); // parameters will be empty
                        break;
                    }

                    // The vision system to use for this opmode.
                    // parameters: <(enum) LCHSValues.MineralVisionSystem>
                    case VISION: {
                        // parameters: <(enum) LCHSValues.MineralVisionSystem>
                        tokenIndex++; // advance to parameter
                        if (tokenIndex == numTokens)
                            throw new AutonomousRobotException(TAG, "Missing parameter in choreography after: " + commandString);

                        parameter = tokens.get(tokenIndex);
                        mineralVisionSystem = LCHSValues.MineralVisionSystem.valueOf(parameter); // invalid enum throws IllegalArgumentException

                        // No need to include the vision system value in the set of commands since it can be accessed
                        // via the method getMineralVisionSystem().
                        break;
                    }

                    // For use during testing.
                    // parameters <enum value LEFT, CENTER, RIGHT>
                    case FIND_GOLD_OPENCV: {
                        tokenIndex++; // advance to parameter
                        if (tokenIndex == numTokens)
                            throw new AutonomousRobotException(TAG, "Missing parameter in choreography after: " + commandString);

                        parameter = tokens.get(tokenIndex);
                        parameters.add(parameter);
                        LCHSValues.MineralPosition.valueOf(parameter); // invalid enum throws IllegalArgumentException

                        if (!skipCommand)
                            steps.add(new Step(command, parameters));
                        break;
                    }

                    // For use during testing.
                    // parameters <none>
                    case FIND_GOLD_TENSORFLOW: {
                        if (!skipCommand)
                            steps.add(new Step(command, parameters)); // parameters will be empty
                        break;
                    }

                    //** The parsing of SAMPLE_DEPOT, SAMPLE_CRATER, and DEPOT_REMOTE can be combined
                        // into a single method. Pass the tokens list and the parameter array as arguments.

                    // Movements for the right starting position.
                    // The robot must turn towards a sample, detect the gold sample,
                    // advance to knock off the gold sample, turn towards the depot,
                    // and advance to the edge of the depot.
                    case SAMPLE_DEPOT: {
                        // The total number of parameters is determined by the number of samples
                        // and the size of the parameter array.
                        // In the configuration file a sample location identifier, e.g. "[LEFT]",
                        // precedes each set.
                        for (int i = 0; i < mineralPositions.length; i++) {
                            tokenIndex++;
                            if (tokenIndex == numTokens)
                                throw new AutonomousRobotException(TAG, "Missing parameter " + mineralPositions[i] + " after: " + commandString);

                            parameter = tokens.get(tokenIndex);
                            if (!parameter.equals(mineralPositions[i]))
                                throw new AutonomousRobotException(TAG, "Unexpected parameter " + parameter + " after " + mineralPositions[i] + " after: " + commandString);

                            // For each mineral position there are 4 parameters. //** changed to 5
                            // A parameter identifier precedes each parameter.
                            for (int j = 0; j < depotStartParameters.length; j++) {
                                tokenIndex++; // advance to parameter identifier
                                if (tokenIndex == numTokens)
                                    throw new AutonomousRobotException(TAG, "Missing parameter " + depotStartParameters[j] + " after " + mineralPositions[i] + " after: " + commandString);

                                parameter = tokens.get(tokenIndex); // parameter identifier
                                if (!parameter.equals(depotStartParameters[j]))
                                    throw new AutonomousRobotException(TAG, "Unexpected parameter " + parameter + " after " + mineralPositions[i] + " after: " + commandString);

                                tokenIndex++; // advance to parameter value
                                if (tokenIndex == numTokens)
                                    throw new AutonomousRobotException(TAG, "Missing value after " + depotStartParameters[j] + " after " + mineralPositions[i] + " after: " + commandString);

                                parameter = tokens.get(tokenIndex);
                                parameters.add(parameter);
                                Double.parseDouble(parameter); // check number format
                            }
                        }

                        if (!skipCommand)
                            steps.add(new Step(command, parameters));
                        break;
                    }

                    // Movements for the left starting position.
                    // The robot must turn towards a sample, detect the gold sample,
                    // advance to knock off the gold sample, turn towards the crater,
                    // and advance until it reaches the incline angle.
                    case SAMPLE_CRATER: {
                        // The total number of parameters is determined by the number of samples
                        // and the size of the parameter array.
                        // In the configuration file a sample location identifier, e.g. "[LEFT]",
                        // precedes each set.
                        for (int i = 0; i < mineralPositions.length; i++) {
                            tokenIndex++;
                            if (tokenIndex == numTokens)
                                throw new AutonomousRobotException(TAG, "Missing parameter " + mineralPositions[i] + " after: " + commandString);

                            parameter = tokens.get(tokenIndex);
                            if (!parameter.equals(mineralPositions[i]))
                                throw new AutonomousRobotException(TAG, "Unexpected parameter " + parameter + " after " + mineralPositions[i] + " after: " + commandString);

                            // For each mineral position there are 4 parameters.
                            // A parameter identifier precedes each parameter.
                            for (int j = 0; j < craterStartParameters.length; j++) {
                                tokenIndex++; // advance to parameter identifier
                                if (tokenIndex == numTokens)
                                    throw new AutonomousRobotException(TAG, "Missing parameter " + craterStartParameters[j] + " after " + mineralPositions[i] + " after: " + commandString);

                                parameter = tokens.get(tokenIndex); // parameter identifier
                                if (!parameter.equals(craterStartParameters[j]))
                                    throw new AutonomousRobotException(TAG, "Unexpected parameter " + parameter + " after " + mineralPositions[i] + " after: " + commandString);

                                tokenIndex++; // advance to parameter value
                                if (tokenIndex == numTokens)
                                    throw new AutonomousRobotException(TAG, "Missing value after " + craterStartParameters[j] + " after " + mineralPositions[i] + " after: " + commandString);

                                parameter = tokens.get(tokenIndex);
                                parameters.add(parameter);
                                Double.parseDouble(parameter); // check number format
                            }
                        }

                        if (!skipCommand)
                            steps.add(new Step(command, parameters));
                        break;
                    }

                    // After sampling in front of the crater, proceed to the depot and make a claim
                        // with our marker.
                   case DEPOT_REMOTE: {
                        // The total number of parameters is determined by the number of samples
                        // and the size of the parameter array.
                        // In the configuration file a sample location identifier, e.g. "[LEFT]",
                        // precedes each set.
                        for (int i = 0; i < mineralPositions.length; i++) {
                            tokenIndex++;
                            if (tokenIndex == numTokens)
                                throw new AutonomousRobotException(TAG, "Missing parameter " + mineralPositions[i] + " after: " + commandString);

                            parameter = tokens.get(tokenIndex);
                            if (!parameter.equals(mineralPositions[i]))
                                throw new AutonomousRobotException(TAG, "Unexpected parameter " + parameter + " after " + mineralPositions[i] + " after: " + commandString);

                            // For each mineral position there are 4 parameters. //** changed to 5
                            // A parameter identifier precedes each parameter.
                            for (int j = 0; j < depotRemoteParameters.length; j++) {
                                tokenIndex++; // advance to parameter identifier
                                if (tokenIndex == numTokens)
                                    throw new AutonomousRobotException(TAG, "Missing parameter " + depotRemoteParameters[j] + " after " + mineralPositions[i] + " after: " + commandString);

                                parameter = tokens.get(tokenIndex); // parameter identifier
                                if (!parameter.equals(depotRemoteParameters[j]))
                                    throw new AutonomousRobotException(TAG, "Unexpected parameter " + parameter + " after " + mineralPositions[i] + " after: " + commandString);

                                tokenIndex++; // advance to parameter value
                                if (tokenIndex == numTokens)
                                    throw new AutonomousRobotException(TAG, "Missing value after " + depotRemoteParameters[j] + " after " + mineralPositions[i] + " after: " + commandString);

                                parameter = tokens.get(tokenIndex);
                                parameters.add(parameter);
                                Double.parseDouble(parameter); // check number format
                            }
                        }

                        if (!skipCommand)
                            steps.add(new Step(command, parameters));
                        break;
                    }

                    // Pause the robot.
                    // parameters: <(positive int) number of milliseconds to sleep>
                    case SLEEP: {
                        tokenIndex++; // advance to parameter
                        if (tokenIndex == numTokens)
                            throw new AutonomousRobotException(TAG, "Missing parameter in choreography after: " + commandString);

                        parameter = tokens.get(tokenIndex);
                        parameters.add(parameter);
                        int sleep = Integer.parseInt(parameter); // check number format
                        if (sleep < 0)
                            throw new AutonomousRobotException(TAG, "Sleep value < 0");

                        if (!skipCommand)
                            steps.add(new Step(command, parameters));
                        break;
                    }
                    default: {
                        throw new AutonomousRobotException(TAG, "Unrecognized command in choreography: " + commandString);
                    }

                } // switch
                tokenIndex++;
                if (tokenIndex == numTokens)
                    break;
            }
        }

        public List<Step> getSteps() {
            return steps;
        }
    }

    // The user of this class will have to interpret the parameters
    // depending on the command. So, for example, if the command is
    // STRAIGHT then there will be one parameter, which should be
    // interpreted as a double.
    class Step {

        private final Command command;
        private final List<String> parameters;

        Step(Command pCommand, List<String> pParameters) {
            command = pCommand;
            parameters = pParameters;
        }

        public Command getCommand() {
            return command;
        }

        public List<String> getParameters() {
            return parameters;
        }

    }

}
