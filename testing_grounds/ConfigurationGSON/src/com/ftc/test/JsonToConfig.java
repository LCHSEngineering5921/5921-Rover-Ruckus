package com.ftc.test;

/**
 * @author trinity
 *
 */

public class JsonToConfig {

	public static void main(String[] args) {
		
		String configFilePath = System.getProperty("user.dir") + "/src/com/ftc/test/robotConfig.json";
		try {
			Configuration config = new Configuration(configFilePath);
			config.displayContents();
			
			//System.out.println(config.opModes.toString());
			
		} catch (Exception ex) {
			ex.printStackTrace();
		}
	}

}
