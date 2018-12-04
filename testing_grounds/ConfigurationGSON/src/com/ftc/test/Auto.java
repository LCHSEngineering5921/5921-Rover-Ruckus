package com.ftc.test;

public class Auto {

	public static void main(String[] args) {
		
		String configFilePath = "/Users/trinity/Documents/Robotics/LCHS 5921 FTC/ConfigurationTest/src/com/ftc/test/robotConfigTest.json";
		try {
			
			Configuration config = new Configuration(configFilePath);
			config.displayContents();
			
		} catch (Exception ex) {
			ex.printStackTrace();
		}
	}

}
