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
 *  Parse robotControllerLog.txt for Acceleration values, which is formatted like this:
 *  0     1             2     3    4 5                 6      7             8 9  10                  11  12
 *  11-19 15:51:25.603  7386  7566 D LCHSAcceleration: Linear Acceleration  | X: 0.029999999329447746 Y: -0.07000000029802322
 *  11-19 15:51:25.640  7386  7566 D LCHSAcceleration: Overall Acceleration | X: -0.6899999976158142 Y: -0.05999999865889549
 * 
 */


public class AccelerationDataParser {

    public static void main(String[] args) {

        // timer for performance comparison
        long startTime = System.nanoTime();
        
        // set path to robotlog; if there is an arg, use as file path
        String filePath = System.getProperty("user.dir") + "/robotControllerLog.txt";
        if (args.length > 0) filePath = args[0];
        File robotControllerLog = new File(filePath);

        // set path to new log; if there is an arg, use as file path
        String newFilePath = System.getProperty("user.dir") + "/AccelerationData.txt";
        if (args.length > 1) newFilePath = args[1];

        // use StringBuilder instead of String for better performance
        StringBuilder parsedContents = new StringBuilder();
        //String parsedContents = "";

        // read robotControllerLog.txt file
        try (BufferedReader br = new BufferedReader(new FileReader(robotControllerLog))) {
            String line;

            while ((line = br.readLine()) != null) {
                // only looking for lines with Acceleration
                if (!line.contains("Acceleration")) continue;

                // split string on whitespace; remove if empty (consecutive white space)
                String[] words = Arrays.stream(line.split("\\s"))
                    .filter(value -> value != null && value.length() > 0)
                    .toArray(size -> new String[size]);

                if (line.contains("Linear Acceleration")) {
                    String timestamp = words[1];
                    String lax = words[10];
                    String lay = words[12];
                    parsedContents.append(String.format("\n%s\t%s\t%s", timestamp, lax, lay));
                    //parsedContents += String.format("\n%s\t%s\t%s", timestamp, lax, lay);
                } else if (line.contains("Overall Acceleration")) {
                    String oax = words[10];
                    String oay = words[12];
                    parsedContents.append(String.format("\t%s\t%s", oax, oay));
                    //parsedContents += String.format("\t%s\t%s", oax, oay);
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
                //bw.write(parsedContents);
                bw.close();

                System.out.println("Result: " + newFilePath);

                // show time
                long elapsedTime = System.nanoTime() - startTime;
                System.out.println("Elapsed Time: " + elapsedTime);
            } catch (IOException ex) {
                System.out.println("Error writing file.");
                ex.printStackTrace();
            }
        }
        
    }

}