/**
 * 
 */
package net.lcec.ftc.stringy;

import java.util.List;

/**
 * @author trinity
 *
 */
public class LeftCoop {

	// WOULD BE
	// public void runOpMode() throws InterruptedException {
	public static void main(String[] args) {
		
		try {
			Configuration config = new Configuration(Configuration.filePath);

			// WOULD HAVE
			//Auto runAuto = new Auto(config.opModes.);
			//waitForStart();
			//runAuto.runRobot();
			
			List<Command> steps = config.opModes.get("LEFT_COOP");
			for (Command step : steps) {
				step.execute(true);
			}
			
			System.out.println("DONE");
			
		} catch (Exception ex) {
			ex.printStackTrace();
		}
		
		
	}

	
}
