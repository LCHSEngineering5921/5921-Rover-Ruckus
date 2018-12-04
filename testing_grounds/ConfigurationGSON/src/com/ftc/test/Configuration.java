package com.ftc.test;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.lang.reflect.Type;

import com.google.gson.reflect.TypeToken;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonIOException;
import com.google.gson.JsonSyntaxException;

/**
 * @author Trinity Chung
 *
 * Configuration
 *   - OpMode(s)
 *     - Command(s)
 *       - Property(s)
 *       
 * TODO: use maps instead of lists
 * to achieve robotConfigIdeal.json
 *
 */

public class Configuration {
	
	List<OpMode> opModes;

	public Configuration() {
		this.opModes = new ArrayList<OpMode>();
	}
	
	public Configuration(List<OpMode> opModes) {
		this.opModes = opModes;
	}
	
	public Configuration(String configFilePath) throws JsonIOException, JsonSyntaxException, FileNotFoundException {
		Gson gson = new Gson();
		
		Type type = new TypeToken<List<OpMode>>(){}.getType();
		opModes = gson.fromJson(new FileReader(configFilePath), type);
		
	}
	
	public void displayContents() {
		Gson gsonPretty = new GsonBuilder().setPrettyPrinting().create();
		Gson gsonUgly = new Gson();
		System.out.println(gsonPretty.toJson(opModes));
		System.out.println(gsonUgly.toJson(opModes));
	}

	public class OpMode {
		String opMode;
		List<Command> commands;
		
		public OpMode(String name) {
			this.opMode = name;
			this.commands = new ArrayList<Command>();
		}
		
		public OpMode(String name, List<Command> commands) {
			this.opMode = name;
			this.commands = commands;
		}
		
		public String toString() {
			return opMode + ": " + commands.toString();
		}
		
		public class Command {
			String command;
			List<Property> properties;
			
			public void execute() {
				//execute command
			}

			public Command(String name) {
				this.command = name;
				this.properties = new ArrayList<Property>();
			}
			
			public Command(String name, List<Property> properties) {
				this.command = name;
				this.properties = properties;
			}
			
			public class Property {
				String key;
				Object value;
				
				public Property() {
					this.key = "";
					this.value = "";
				}
				
				public Property(String key, Object value) {
					this.key = key;
					this.value = value;
				}
				
				
			}
		}
	}

    
}

