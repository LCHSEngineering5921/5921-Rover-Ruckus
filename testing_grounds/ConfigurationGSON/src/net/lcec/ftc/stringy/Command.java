package net.lcec.ftc.stringy;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import net.lcec.ftc.stringy.Sample;

/**
 * <h1>Command</h1>
 * This version uses <code>String<code>s for the property values.
 * The result is a more descriptive structure, but less flexibility for different
 * types of property values.
 *   
 * @author trinity
 *   
 */
public class Command {

	//--------- CLASS VARIABLES ----------
	public Map<String, Map<String, String>> properties;

	private String command;
	private boolean loud = false;
	

	//--------- CONSTRUCTORS ----------
	public Command(String name) {
		this.command = name;
		this.properties = new HashMap<String, Map<String, String>>();
	}
	
	public Command(String name, HashMap<String, Map<String, String>> properties) {
		this.command = name;
		this.properties = properties;
	}
	
	
	// --------- FUNCTIONS ----------
	// PUBLIC
	
	/**
	 * This method is executed for each Command in the OpMode.
	 * The class variable <code>command<code> is used to identify which
	 * block to execute.  The class variable <code>properties<code>
	 * are read and used at runtime.
	 * 
	 * @param	beLoud  whether or not debugging should be verbose
	 * @return	none
	 */
	public void execute(boolean beLoud) { // execute command
		loud = beLoud;
		
		if (is("TURN", "angle", "power")) {
			shout("Turn angle: " + getDouble("angle"));
			
		} else if (is("LAND", "tilt_target")) {
			shout("Tilt target position: " + getInt("tilt_target"));
			
		} else if (is("SAMPLE_CRATER", Sample.SAMPLE_PARAMETERS, Sample.CRATER_PARAMETERS)) {
			shout("Sample Crater Left Parameters: " + properties.get("left"));
			Sample.findGoldOCV();
			
		} else System.out.println("ERROR EXECUTING: " + command);

	}

	/**
	 * This method is executed for each Command in the OpMode.
	 * The command name <code>command<code> is used to identify which
	 * block to execute.  The <code>properties<code> for the command
	 * are read at runtime.
	 * <p>
	 * Default setting is that debugging is not verbose.
	 * 
	 * @return	none
	 */
	public void execute() {
		execute(false);
	}

	/**
	 * Log message if loud is true; for verbose logging
	 * 
	 * @param	message	contents of log
	 * @return	none
	 */
	public void shout(Object message) {
		if (loud) System.out.println(message.toString());
	}
	
	
	// PRIVATE
	private boolean is(String name, String...expectedKeys) {
		String[] expectedValues = {"value", "type"};
		return command.equalsIgnoreCase(name) && has(expectedKeys, expectedValues);
	}

	private boolean is(String name, String[] keys, String[] values) {
		return command.equalsIgnoreCase(name) && has(keys, values);
	}
	
	private boolean has(String[] keys, String...values) {
		String[] existingKeys = properties.keySet().toArray(new String[properties.size()]);
		for (String key : keys) { // check if keys exist
			if (!Arrays.asList(existingKeys).contains(key)) {
				System.out.println("IN COMMAND " + command + ": MISSING KEY: " + key);
				return false;
			}
		}
		return true;
	}
	
	private int getInt(String key) throws RuntimeException {
		if (!properties.containsKey(key))
			throw new RuntimeException("Requested key " + key + " does not exist in " + command);
		if (!properties.get(key).containsKey("value"))
			throw new RuntimeException(key + " does not contain a value");
		if (!properties.get(key).containsKey("type"))
			throw new RuntimeException(key + " does not specify an expected type");
		if (!properties.get(key).get("type").equalsIgnoreCase("int")) 
			throw new RuntimeException("Expected type of " + key + " is " + properties.get(key).get("type") + ", not int");
		
		return Integer.parseInt(properties.get(key).get("value"));
	}
	
	private double getDouble(String key) throws RuntimeException {
		if (!properties.containsKey(key))
			throw new RuntimeException("Requested key " + key + " does not exist in " + command);
		if (!properties.get(key).containsKey("value"))
			throw new RuntimeException(key + " does not contain a value");
		if (!properties.get(key).containsKey("type"))
			throw new RuntimeException(key + " does not specify an expected type");
		if (!properties.get(key).get("type").equalsIgnoreCase("double"))
			throw new RuntimeException("Expected type of " + key + " is " + properties.get(key).get("type") + ", not double");
		
		return Double.parseDouble(properties.get(key).get("value"));
	}
	
	private String getString(String key) throws RuntimeException {
		if (!properties.containsKey(key))
			throw new RuntimeException("Requested key " + key + " does not exist in " + command);
		if (!properties.get(key).containsKey("value"))
			throw new RuntimeException(key + " does not contain a value");
		if (!properties.get(key).containsKey("type"))
			throw new RuntimeException(key + " does not specify an expected type");
		if (!properties.get(key).get("type").equalsIgnoreCase("string"))
			throw new RuntimeException("Expected type of " + key + " is " + properties.get(key).get("type") + ", not string");
		
		return properties.get(key).get("value");
	}
	

}
