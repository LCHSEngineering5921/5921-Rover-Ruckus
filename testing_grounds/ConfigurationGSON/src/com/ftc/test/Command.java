/**
 * 
 */
package com.ftc.test;

import java.util.HashMap;
import java.util.Map;

import com.ftc.test.Sample;

/**
 * @author trinity
 *
 */
public class Command {
	
	public Map<String, Object> properties;

	private String command;
	private boolean loud = false; // loud means verbose logging
	

	//--------- CONSTRUCTORS ----------
	public Command(String name) {
		this.command = name;
		this.properties = new HashMap<String, Object>();
	}
	
	public Command(String name, HashMap<String, Object> properties) {
		this.command = name;
		this.properties = properties;
	}

	//--------- FUNCTIONS ----------
	@SuppressWarnings("unchecked")
	
	public void execute(boolean beLoud) { // execute command
		loud = beLoud;
		
		if (is("TURN", "angle", "power")) {
			shout("WOOT I'M TURNING " + properties.get("angle") +" DEGREES NOW");
			
		} else if (is("LAND", "tilt_target")) {
			shout("I JUST LANDED, " + properties.get("tilt_target") +" AS TARGET TILT");
			
		} else if (is("SAMPLE_CRATER", "left", "center", "right")) {
			if (!has((Map<String, Object>) properties.get("left"), Sample.CRATER_PARAMETERS) ||
				!has((Map<String, Object>) properties.get("center"), Sample.CRATER_PARAMETERS) ||
				!has((Map<String, Object>) properties.get("right"), Sample.CRATER_PARAMETERS)) return;
			
			Sample.findGoldOCV();
			
		} else System.out.println("ERROR EXECUTING: " + command);

	}
	
	public void execute() {
		execute(false);
	}
	
	private boolean is(String name, String...keys) {
		return command.equalsIgnoreCase(name) && has(properties, keys);
	}
	
	private boolean has(Map<String,Object> map, String...keys) {
		for (String key : keys) { // check if keys exist
			if (!map.containsKey(key)) {
				System.out.println("IN COMMAND " + command + ": MISSING KEY: " + key);
				return false;
			}
		}
		return true;
	}
	
	private void shout(String message) {
		if (loud) System.out.println(message);
	}
	
}
