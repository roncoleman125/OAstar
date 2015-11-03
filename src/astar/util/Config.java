package astar.util;

import static astar.util.Config.Heuristic.CHECKERS;
import static astar.util.Config.Heuristic.EUCLIDEAN;
import static astar.util.Config.Heuristic.MANHATTAN;
import static astar.util.Config.Heuristic.SSE;
import static astar.util.Config.Objective.BASIC;
import static astar.util.Config.Objective.PRETTY;
import static astar.util.Config.Objective.STEALTHY;
import java.io.FileReader;
import java.io.IOException;
import java.util.logging.Level;
import java.util.logging.Logger;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

/**
 *
 * @author Marist User
 */
public class Config {
    public enum Heuristic {
        EUCLIDEAN,
        MANHATTAN,
        CHECKERS,
        SSE
    }
    
    public enum Objective {
        BASIC,
        PRETTY,
        STEALTHY
    };
    
    public Heuristic heuristic = EUCLIDEAN;
    public Objective objective = BASIC;
    public final int WORLD_WIDTH = 20;
    public final int WORLD_HEIGHT = 20;
    public Map map = new Map(WORLD_WIDTH,WORLD_HEIGHT);
    
    /** Path of the config file */
    protected final static String CONFIG_PATH = "astar.json";
    
        /** JSON parser to read and process the config file */
    protected static JSONParser parser = new JSONParser();
    
    /** This one and only configuration */
    protected static Config config;
    
    
    /**
     * Constructor can only be constructed through singleton.
     */
    private Config() {
        
    }
    
    /**
     * Gets an instance of a configuration singleton from default file.
     * @return Config
     */
    public static Config getInstance() {
        return getInstance(CONFIG_PATH);
    }
    
    public static Config getInstance(String path) {
        if (config != null)
            return config;

        config = new Config();

        config.load(CONFIG_PATH);

        return config;
    }
    
    /**
     * Gets a configuration single t
     * @param path Path to the config file.
     */
    public  void load(String path) {
        try {
            JSONObject json = (JSONObject) parser.parse(new FileReader(path));
                        
            map.width = ((Long) json.get("map.width")).intValue();
            
            map.height = ((Long) json.get("map.height")).intValue();
            
            String h = (String) json.get("heuristic");
            switch (h) {
                case "euclidean":
                    heuristic = EUCLIDEAN;
                    break;
                case "manhattan":
                    heuristic = MANHATTAN;
                    break;
                case "checkers":
                    heuristic = CHECKERS;
                    break;
                case "sse":
                    heuristic = SSE;
                    break;
                default:
                    System.err.println("bad heuristic in "+CONFIG_PATH);
            }
            
            String obj = (String) json.get("objective");
            switch (obj) {
                case "basic":
                    objective = BASIC;
                    break;
                case "pretty":
                    objective = PRETTY;
                    break;
                case "stealthy":
                    objective = STEALTHY;
                    break;
                default:
                    System.err.println("bad objective in "+CONFIG_PATH);
            }

        } catch (IOException | ParseException ex) {
            Logger.getLogger(Config.class.getName()).log(Level.SEVERE, null, ex);
        }
    }
}
