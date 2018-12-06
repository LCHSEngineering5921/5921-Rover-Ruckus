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

Read the configuration file TeleTrac.txt to get labels that will be put
into the Robot Controller log file when the TeleTrac opmode runs.

Stop at the first non-commented line and tokenize that line on white space.
The first token will be the label for a run of the TeleTrac opmode. The
subsequent tokens will be labels for each start/stop tracking pair; each
of these tokens must bein with "STRAIGHT" or "TURN".

*/

public class TeleTracConfiguration {

    private static final String TAG = "TeleTracConfig";
    public static final String UNRESTRICTED_TEST = "UNRESTRICTED_TEST";
    public static final String STRAIGHT = "STRAIGHT";
    public static final String TURN = "TURN";

    private List<String> tokens;

    public TeleTracConfiguration() {

        // Make sure we can access the configuration file.
        if (!(isExternalStorageReadable() || isExternalStorageWritable()))
            throw new AutonomousRobotException(TAG, "Disaster: can't access file system");

        try {
            // Creates the file that it is going to read from the fileName string.
            File configFile = new File(Environment.getExternalStoragePublicDirectory(
                    Environment.DIRECTORY_PICTURES), "TeleTracConfig.txt");
            FileReader fr = new FileReader(configFile);
            BufferedReader br = new BufferedReader(fr);

            // Reads the file one line at a time until it find a non-commented line.
            // It then splits the line into tokens.
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
                break; // break on first non-commented line
            }
            br.close();

        } catch (FileNotFoundException ex) {
            throw new AutonomousRobotException(TAG, ex.getMessage());
        } catch (IOException ex) {
            throw new AutonomousRobotException(TAG, ex.getMessage());
        }

        // Make sure there was at least one non-commented line.
        if (tokens.isEmpty())
            throw new AutonomousRobotException(TAG, "No non-commented lines in configuration file");

        // If the first token is not UNRESTRICTED_TEST then tokens 1 - n must begin with the
        // string "STRAIGHT" or "TURN".
        if (!tokens.get(0).equals(UNRESTRICTED_TEST)) {
            // In this case there must be att least two tokens.
            if (tokens.size() < 2)
                throw new AutonomousRobotException(TAG, "Disaster: line in configuration does not contain at least 2 tokens");

            // Starting with the second token, make sure each token starts with either "STRAIGHT"
            // or "TURN".
            for (int i = 1; i < tokens.size(); i++)
                if (!((tokens.get(i).startsWith(STRAIGHT)) || (tokens.get(i).startsWith(TURN))))
                    throw new AutonomousRobotException(TAG, "Label must start with STRAIGHT or TURN");
        }
        // The first token is UNRESTRICTED_TEST: it must be the only token.
        else if (tokens.size() != 1)
            throw new AutonomousRobotException(TAG, "UNRESTRICTED_TEST must be tthe only token");
    }

    public List<String> getTeleTracLabels() {
        return tokens;
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

}
