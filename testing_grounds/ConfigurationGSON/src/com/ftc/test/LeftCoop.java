/**
 * 
 */
package com.ftc.test;

import java.util.List;

/**
 * @author trinity
 *
 */
public class LeftCoop {

	// WOULD BE
	// public void runOpMode() throws InterruptedException {
	public static void main(String[] args) {
		
		String configFilePath = System.getProperty("user.dir") + "/src/com/ftc/test/robotConfig.json";
		
		try {
			Configuration config = new Configuration(configFilePath);

			// WOULD HAVE
			//Auto runAuto = new Auto(config.opModes.);
			//waitForStart();
			//runAuto.runRobot();
			
			List<Command> steps = config.opModes.get("LEFT_COOP");
			for (Command step : steps) {
				step.execute(true);
			}
			
		} catch (Exception ex) {
			ex.printStackTrace();
		}
		
		
	}

	
}
