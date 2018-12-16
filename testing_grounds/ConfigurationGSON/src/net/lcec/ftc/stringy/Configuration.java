package net.lcec.ftc.stringy;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
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
 * <h1>Configuration</h1>
 * The configuration contains a list of opModes, each of which contains a list of commands,
 * each of which contains properties.  In this version, all values are <code>String<code>s.
 * The constructor reads a json file, and automatically wraps quotes around values that do
 * not already have them.  Comments must be at the start of a line (ignoring whitespace).
 *  
 * @author trinity
 *   
 */
public class Configuration {

	public static final String filePath = System.getProperty("user.dir") + "/src/net/lcec/ftc/stringy/robotConfig.json";
	public Map<String, List<Command>> opModes;

	
	//--------- CONSTRUCTORS ----------
	public Configuration() throws JsonIOException, JsonSyntaxException, FileNotFoundException {
		this(filePath);
	}
	
	public Configuration(String configFilePath) throws JsonIOException, JsonSyntaxException, FileNotFoundException {
		Gson gson = new Gson();
		Type type = new TypeToken<HashMap<String, List<Command>>>(){}.getType();
		
		StringBuilder configContents = new StringBuilder();
		
		 try {
	            FileReader fr = new FileReader(configFilePath);
	            BufferedReader br = new BufferedReader(fr);

	            String line;
	            while ((line = br.readLine()) != null) {
	            	line = line.replaceAll("\\s", "");
	                if (line.length() == 0 || line.startsWith("//")) continue; // allow comments
	                configContents.append(line);
	            }
	            
	            char[] chars = configContents.toString().toCharArray();
	            configContents.setLength(0); // clear configContents
	            
                for (int i = 0; i < chars.length; i++) {
       
            		boolean missingQuoteAfter =  chars[i] == ':' && chars[i+1] != '"' && chars[i+1] != '[' && chars[i+1] != '{';
            		boolean missingQuoteBefore = i+1 < chars.length && (chars[i+1] == '}' || chars[i+1] == ',') && 
            										(chars[i] != '"' && chars[i] != '}' && chars[i] != ']');
            		if (missingQuoteAfter || missingQuoteBefore) {
            			configContents.append(chars[i] + "\"");
            		} else configContents.append(chars[i]);
                	
                }
	            
	            br.close();
	        } catch (FileNotFoundException ex) {
	        		ex.printStackTrace();
	        } catch (IOException ex) {
        		ex.printStackTrace();
	        }
		 
		opModes = gson.fromJson(configContents.toString(), type);
	}

	
	//--------- FUNCTIONS ----------
	public void displayContents() {
		Gson gsonPretty = new GsonBuilder().setPrettyPrinting().create();
		Gson gsonUgly = new Gson();
		System.out.println(gsonPretty.toJson(opModes));
		System.out.println(gsonUgly.toJson(opModes));
	}

    
}

