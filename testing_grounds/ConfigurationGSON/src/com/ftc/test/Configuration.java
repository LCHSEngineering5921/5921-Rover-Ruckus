package com.ftc.test;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.lang.reflect.Type;

import com.google.gson.reflect.TypeToken;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonIOException;
import com.google.gson.JsonSyntaxException;

/**
 * @author trinity
 *
 * Configuration
 *   - opModes = Map<String, List<Command>>
 *   - Command
 *       - properties = Map<String, Object>
 *       
 * TODO: use maps instead of lists
 * to achieve robotConfigIdeal.json
 *
 */

public class Configuration {
	
	Map<String, List<Command>> opModes;

	//--------- CONSTRUCTORS ----------
	public Configuration() {
		this.opModes = new HashMap<String, List<Command>>();
	}
	
	public Configuration(HashMap<String, List<Command>> opModes) {
		this.opModes = opModes;
	}
	
	public Configuration(String configFilePath) throws JsonIOException, JsonSyntaxException, FileNotFoundException {
		Gson gson = new Gson();
		Type type = new TypeToken<HashMap<String, List<Command>>>(){}.getType();
		opModes = gson.fromJson(new FileReader(configFilePath), type);
	}

	//--------- FUNCTIONS ----------
	public void displayContents() {
		Gson gsonPretty = new GsonBuilder().setPrettyPrinting().create();
		Gson gsonUgly = new Gson();
		System.out.println(gsonPretty.toJson(opModes));
		System.out.println(gsonUgly.toJson(opModes));
	}

    
}

