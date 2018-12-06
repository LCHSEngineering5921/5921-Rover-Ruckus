package com.ftc.test;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.ftc.test.Configuration;

/**
 * @author trinity
 *
 */

public class ManualConfigToJson {

	public static void main(String[] args) {
		
		try {
			
			Configuration config = new Configuration();
			
			Command land = new Command("land");

			List<Command> leftCoop = new ArrayList<Command>();
			Command lcTurn = new Command("turn");
			Command lcSample = new Command("sample_crater");
			

			List<Command> rightCoop = new ArrayList<Command>();
			Command rcTurn = new Command("turn");
			
			//shared commands
			land.properties.put("tilt_target", 2200);
			
			//LEFTCOOP
			leftCoop.add(land);
			leftCoop.add(lcTurn);
			leftCoop.add(lcSample);
			
			//lcTurn
			lcTurn.properties.put("power", 0.5);
			lcTurn.properties.put("angle", 90.0);
			
			//lcSample
			Map<String, Double> lcSamplePositionProperties = new HashMap<String, Double>();

			lcSamplePositionProperties.put("turn", 0.1);
			lcSamplePositionProperties.put("postRT", 24.0);
			lcSamplePositionProperties.put("knock", 12.0);

			lcSample.properties.put("left", lcSamplePositionProperties);
			lcSample.properties.put("center", lcSamplePositionProperties);
			lcSample.properties.put("right", lcSamplePositionProperties);
			
			//RIGHTCOOP
			rightCoop.add(land);
			rightCoop.add(rcTurn);

			//rcTurn
			rcTurn.properties.put("power", 1.0);
			rcTurn.properties.put("angle", -90.0);
			
			//OPMODES
			Map<String, List<Command>> opModes = new HashMap<String, List<Command>>();
			opModes.put("LEFT_COOP", leftCoop);
			opModes.put("RIGHT_COOP", rightCoop);
			
			//CONFIG
			config.opModes = opModes;
			config.displayContents();
			
			
		} catch (Exception ex) {
			ex.printStackTrace();
		}
	}

}
