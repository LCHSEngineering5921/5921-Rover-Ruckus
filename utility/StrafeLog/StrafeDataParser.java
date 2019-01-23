import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;
import java.io.File;

/**
 *  @author Trinity Chung 
 *  Created 11/19/2018
 * 
 * 0     1            2     3     4 5               6        7    8    9    10
 * 01-02 16:50:58.038 11635 11739 D LCHSAutoMotion: Distance from side wall 7.04724409448819
 * 01-02 16:50:58.070 11635 11739 D LCHSAutoMotion: IMU 0.9375, error -0.9375, p+i -0.0475128484546875
 *                                                  6     7     8   9       10 11  12   13    14                  15                   16                 17
 * 01-02 16:50:58.070 11635 11739 D LCHSAutoMotion: Power under P+I control LF, LB, RF, RB:  -0.5524871515453125 -0.5524871515453125 -0.6475128484546875 -0.6475128484546875
 *                                                  6      7        8    9    10
 * 01-02 16:50:58.070 11635 11739 D LCHSAutoMotion: Error Distance from Wall: 0.9527559055118102
 *                                                  6      7       8
 * 01-02 16:50:58.070 11635 11739 D LCHSAutoMotion: Strafe Value: 0.023818897637795256
 *                                                  6      7    8   9
 * 01-02 16:50:58.070 11635 11739 D LCHSAutoMotion: Final Power LF: -0.5524871515453125
 * 01-02 16:50:58.070 11635 11739 D LCHSAutoMotion: Final Power RB: -0.6475128484546875
 * 01-02 16:50:58.070 11635 11739 D LCHSAutoMotion: Final Power LB: -0.5524871515453125
 * 01-02 16:50:58.071 11635 11739 D LCHSAutoMotion: Final Power RF: -0.6475128484546875
 * 
 */


public class StrafeDataParser {

    public static void main(String[] args) {
        
        // set path to robotlog; if there is an arg, use as file path
        String filePath = System.getProperty("user.dir") + "/robotControllerLog.txt";
        if (args.length > 0) filePath = args[0];
        File robotControllerLog = new File(filePath);

        // set path to new log; if there is an arg, use as file path
        String newFilePath = System.getProperty("user.dir") + "/StrafeData.txt";
        if (args.length > 1) newFilePath = args[1];

        // use StringBuilder instead of String for better performance
        StringBuilder parsedContents = new StringBuilder();

        // read robotControllerLog.txt file
        try (BufferedReader br = new BufferedReader(new FileReader(robotControllerLog))) {
            String line;

            while ((line = br.readLine()) != null) {
                // only looking for lines with Acceleration
                if (!line.contains("AutoMotion")) continue;

                // split string on whitespace; remove if empty (consecutive white space)
                String[] words = Arrays.stream(line.split("\\s"))
                    .filter(value -> value != null && value.length() > 0)
                    .toArray(size -> new String[size]);

                if (line.contains("Power under P+I control")) {
                    String timestamp = words[1];
                    String lf = words[14];
                    String lb = words[15];
                    String rf = words[16];
                    String rb = words[17];
                    parsedContents.append(String.format("\n%s\t\t%s\t%s\t%s\t%s", timestamp, lf, rb, lb, rf));
                } else if (line.contains("Error Distance from Wall")) {
                    String errorDistance = words[10];
                    parsedContents.append(String.format("\t%s", errorDistance));
                } else if (line.contains("Strafe Value")) {
                    String strafeValue = words[8];
                    parsedContents.append(String.format("\t%s\t", strafeValue));
                } else if (line.contains("Final Power")) {
                    String finalPower = words[9];
                    parsedContents.append(String.format("\t%s", finalPower));
                }
            }

            br.close();
            
        } catch (IOException ex) {
            System.out.println("Error reading file.");
            ex.printStackTrace();
        } finally {
            // write formatted file
            try (BufferedWriter bw = new BufferedWriter(new FileWriter(newFilePath))) {
                bw.write(parsedContents.toString());
                bw.close();
                System.out.println("Result: " + newFilePath);
            } catch (IOException ex) {
                System.out.println("Error writing file.");
                ex.printStackTrace();
            }
        }
        
    }

}