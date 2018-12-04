package com.ftc.test;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.ftc.test.Configuration;

/**
 * @author Trinity Chung
 *
 */

public class ManualConfigToJson {

	public static void main(String[] args) {
		
		try {
			
			Configuration config = new Configuration();
			Configuration.OpMode opMode = config.new OpMode("generic_opmode");
			Configuration.OpMode.Command command = opMode.new Command("generic_command");
			
			Configuration.OpMode.Command land = opMode.new Command("land");
			Configuration.OpMode leftCoop = config.new OpMode("left_coop");
			Configuration.OpMode.Command lcTurn = opMode.new Command("turn");
			Configuration.OpMode.Command lcSample = opMode.new Command("sample");
			Configuration.OpMode rightCoop = config.new OpMode("right_coop");
			Configuration.OpMode.Command rcTurn = opMode.new Command("turn");
			
			//shared commands
			land.properties.add(command.new Property("tilt_target", 2200));
			
			//LEFTCOOP
			leftCoop.commands.add(land);
			leftCoop.commands.add(lcTurn);
			leftCoop.commands.add(lcSample);
			
			//lcTurn
			lcTurn.properties.add(command.new Property("power", 0.5));
			lcTurn.properties.add(command.new Property("angle", 90.0));
			
			//lcSample
			Map<String, Double> lcSamplePositionProperties = new HashMap<String, Double>();
			lcSamplePositionProperties.put("lcTurn", 0.1);
			lcSamplePositionProperties.put("postRT", 24.0);
			lcSamplePositionProperties.put("knock", 12.0);

			lcSample.properties.add(command.new Property("left", lcSamplePositionProperties));
			lcSample.properties.add(command.new Property("center", lcSamplePositionProperties));
			lcSample.properties.add(command.new Property("right", lcSamplePositionProperties));
			
			//RIGHTCOOP
			leftCoop.commands.add(land);
			rightCoop.commands.add(rcTurn);

			//rcTurn
			rcTurn.properties.add(command.new Property("power", 1.0));
			rcTurn.properties.add(command.new Property("angle", -90.0));
			
			//OPMODES
			List<Configuration.OpMode> opModes = new ArrayList<>();
			opModes.add(leftCoop);
			opModes.add(rightCoop);
			
			//CONFIG
			config.opModes = opModes;
			config.displayContents();
			
			
		} catch (Exception ex) {
			ex.printStackTrace();
		}
	}

}
