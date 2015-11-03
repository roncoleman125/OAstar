/*
 Copyright (c) 2015 Ron Coleman
 Permission is hereby granted, free of charge, to any person obtaining
 a copy of this software and associated documentation files (the
 "Software"), to deal in the Software without restriction, including
 without limitation the rights to use, copy, modify, merge, publish,
 distribute, sublicense, and/or sell copies of the Software, and to
 permit persons to whom the Software is furnished to do so, subject to
 the following conditions:
 The above copyright notice and this permission notice shall be
 included in all copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
package astar;

import astar.util.Config;
import static astar.util.Config.Heuristic.EUCLIDEAN;
import astar.util.Config.Objective;
import static astar.util.Config.Objective.BASIC;
import static astar.util.Config.Objective.PRETTY;
import static astar.util.Config.Objective.STEALTHY;
import java.io.*;
import java.util.*;

/**
 * Main class to implement A* path finding.
 *
 * @author Ron
 */
public class Astar {
    public final static boolean PRIORITY_STRAIGHT = false;

    public final static char SYMBOL_DEST = 'D';
    public final static char SYMBOL_START = 'S';
    public final static char SYMBOL_OBSTACLE = '#';
    public final static char SYMBOL_BUG = '?';
    public final static char SYMBOL_FREE = ' ';
    public final static char SYMBOL_STEP = '+';

    public final static int NO_LIMIT = 10000;
    
    private final static int LUCKY_2 = 2;
    private final static int LUCKY_8 = 8;
    private final static int LUCKY_10 = 10;
    private final static int LUCKY_13 = 13;

    private BufferedReader reader;
    private int width;
    private int height;
    private char[][] tileMap;
    private int destX;
    private int destY;
    private int startX;
    private int startY;
    private LinkedList openList = new LinkedList();
    private LinkedList closedList = new LinkedList();

    // Offsets relative to current position in map
    private int[][] xyOffsets = {
        {-1, 0},    // W
        {-1,-1},    // NW
        { 0,-1},    // N
        { 1,-1},    // NE
        { 1, 0},    // E
        { 1, 1},    // SE
        { 0, 1},    // S
        {-1, 1}};   // SW

    // Next offset index
    private int indOffset;

    /**
     * Constructor.
     *
     * @param name File name to load the tile map file
     */
    public Astar(String name) {
        try {
            reader = new BufferedReader(new FileReader(name));
        } catch (Exception e) {
            System.err.println(e);
        }
    }

    /**
     * Constructor.
     * @param startX 
     * @param tileMap Tile map
     * @param startY
     * @param destX
     * @param destY
     */
    public Astar(char[][] tileMap, int startX, int startY, int destX, int destY) {
        this.tileMap = tileMap;
        this.startX = startX;
        this.startY = startY;
        this.destX = destX;
        this.destY = destY;
        this.width = tileMap[0].length;
        this.height = tileMap.length;
    }

    /**
     * Finds a basic path from source to destination.
     * @return Node representing the reverse path.
     */
    public Node find() {
        return find(BASIC, Integer.MAX_VALUE);
    }

    /**
     * Finds a path with a give objective 
     * @param objective Objective
     * @return Node representing the reverse path
     */
    public Node find(Objective objective) {
        return find(objective, Integer.MAX_VALUE);
    }

    /**
     * Find path find source to destination.
     * @param objective Search objective
     * @param limit Maximum number of nodes to generate.
     * @return Destination node if path found, null if no path found.
     */
    public Node find(Objective objective, int limit) {
        Node dest = new Node(destX, destY);

        // Start is first node to analyze
        moveToOpen(new Node(startX, startY));

        // Trace a path until we have no options
        while (!openList.isEmpty()) {
            
            // If we're entering here for the first time, START will be only
            // node and lowest cost node
            Node curNode = getLowestCostNode();

            // We're done if we get to the destination node
            if (curNode.equals(dest)) {
                return relink(curNode);
            }

            // Otherwise move current node to closed as we won't analyze it
            // further, only it's adjacent nodes
            moveToClosed(curNode);

            // Reset the adjacency state
            reset();

            // Analyze each of the adjacent nodes
            do {
                // Get next adjacent to current node, if there is one
                Node adjNode = getAdjacent(curNode);

                // If there are no more adjacent nodes to consider, we're done
                // with the current node
                if (adjNode == null) {
                    break;
                }

                // Get the known cost, namely, how far we've travelled so far
                double g = adjNode.length();
                
                // Get the heuristic cost, namely, how far we have to go
                double h = calculateHeuristic(adjNode, dest);

                // This is the standard A* cost
                double cost = g + h;

                // Get any surcharges for non-standard A*
                cost += surcharge(objective,curNode,adjNode,h);

                // This node has that cost
                adjNode.setCost(cost);

                // Add it to the open list to explore later
                openList.add(adjNode);

                // If we exceed the node limit, then there is no path!
                if (Node.idCount > limit) {
                    return null;
                }

            } while (true);

        }
        return null;
    }
    
    protected double surcharge(Objective objective, Node node, Node adjNode, double heuristic) {
        double surcharge = 0.0;
        
        if (objective == STEALTHY && node.getParent() != null) {
//                    int dx = curNode.getX() - adj.getX();
//                    int dy = curNode.getY() - adj.getY();
//
//                	if(dx != 0 && dy == 0 || dx == 0 && dy != 0)
//                		cost -= heuristic * 0.05;
                    if (hugsWall(adjNode)) {
                        surcharge -= heuristic * 0.10;
                    }            
        }
        else if(objective == PRETTY && node.getParent() != null) {
                    Node parent = node.getParent();

                    int dx1 = parent.getX() - node.getX();

                    int dy1 = parent.getY() - node.getY();

                    int dx2 = node.getX() - adjNode.getX();

                    int dy2 = node.getY() - adjNode.getY();

                    boolean zags = dx1 != dx2 || dy1 != dy2;

                    if (!zags) {
                        adjNode.setInertia(node.getInertia() + 1);
                    }

//                    // Rabin algorithm
//                    if(zags)
//                        cost += .001;

                    double strength = adjNode.getInertia();

                    if (strength < LUCKY_8) {
                        boolean hugs = hugsWall(adjNode);

                        if (hugs && !zags) {
                            surcharge += LUCKY_10;
                        } else if (!hugs && zags) {
                            surcharge += LUCKY_2;
                        } else if (hugs && zags) {
                            surcharge += LUCKY_13;
                        }
                    }            
        }
        
        return surcharge;
    }

    /**
     * Relink the child nodes properly since the child references are leftover
     * references from scanning the adjacent nodes.
     *
     * @param path Path through world
     * @return Node
     */
    protected Node relink(Node path) {
        Node anode = path;

        Node child = null;

        while (anode != null) {
            anode.setChild(child);

            child = anode;

            anode = anode.getParent();
        }

        return path;
    }

    protected boolean hugsWall(Node anode) {
        int x = anode.getX();
        int y = anode.getY();

        return isObstacle(x - 1, y)
                || isObstacle(x + 1, y)
                || isObstacle(x, y - 1)
                || isObstacle(x, y + 1);
    }

    /**
     * Get next adjacent node relative to parent node.
     *
     * @param parent Parent node
     * @return Next adjacent node.
     */
    protected Node getAdjacent(Node parent) {
        int x = parent.getX();
        int y = parent.getY();

        while (indOffset < xyOffsets.length) {
            int adjX = x + xyOffsets[indOffset][0];
            int adjY = y + xyOffsets[indOffset][1];

            indOffset++;

            if (adjX < 0 || adjX >= width || adjY < 0 || adjY >= height) {
                continue;
            }

            if (onOpenList(adjX, adjY) || onClosedList(adjX, adjY) || isObstacle(adjX, adjY)) {
                continue;
            }

            Node adjacent = new Node(adjX, adjY);
            adjacent.setParent(parent);
            
            return adjacent;
        }

        return null;
    }

    /**
     * Move node to open list.
     *
     * @param node Node to put on open list.
     */
    protected void moveToOpen(Node node) {
        openList.add(node);
    }

    /**
     * Determines if node on open list.
     *
     * @param node Node to test.
     * @return True if node is on open list.
     */
    protected boolean onOpenList(Node node) {
        return onOpenList(node.getX(), node.getY());
    }

    /**
     * Determines if node at x, y on open list.
     *
     * @param x X coordinate.
     * @param y Y coordinate.
     * @return True if node at coordinate on open list.
     */
    protected boolean onOpenList(int x, int y) {
        ListIterator iter = openList.listIterator();

        while (iter.hasNext()) {
            Node candidate = (Node) iter.next();
            if (candidate.getX() == x && candidate.getY() == y) {
                return true;
            }
        }

        return false;
    }

    /**
     * Determines if node on closed list.
     *
     * @param node Node to test.
     * @return True if node on closed list.
     */
    protected boolean onClosedList(Node node) {
        return onClosedList(node.getX(), node.getY());
    }

    /**
     * Determines if node at x, y on closed list.
     *
     * @param x X coordinate.
     * @param y Y coordinate.
     * @return True if node at coordinate on closed list.
     */
    protected boolean onClosedList(int x, int y) {
        ListIterator iter = closedList.listIterator();

        while (iter.hasNext()) {
            Node candidate = (Node) iter.next();
            if (candidate.getX() == x && candidate.getY() == y) {
                return true;
            }
        }

        return false;
    }

    /**
     * Determines if node is an obstacle.
     *
     * @param node Node to test.
     * @return True if node an obstacle.
     */
    protected boolean isObstacle(Node node) {
        return isObstacle(node.getX(), node.getY());
    }

    /**
     * Determines if node at x, y is an obstacle.
     *
     * @param x X coordinate.
     * @param y Y coordinate.
     * @return True if node an obstacle.
     */
    protected boolean isObstacle(int x, int y) {
        char sym = tileMap[y][x];

        return sym == SYMBOL_OBSTACLE;
    }

    /**
     * Move node to closed list.
     *
     * @param node Node to move to closed list.
     */
    protected void moveToClosed(Node node) {
        ListIterator iter = openList.listIterator();

        while (iter.hasNext()) {
            Node candidate = (Node) iter.next();
            if (candidate == node) {
                iter.remove();
                closedList.add(node);
            }
        }
    }

    /**
     * Calculate heuristic part of cost using Manhattan distance.
     *
     * @param adj Adjacent node.
     * @param dest Destination node.
     * @return Distance.
     */
    protected double calculateHeuristic(Node adj, Node dest) {
        double dx = adj.getX() - dest.getX();

        double dy = adj.getY() - dest.getY();

        double h = Double.POSITIVE_INFINITY;
        
        switch(Config.getInstance().heuristic) {
            case EUCLIDEAN:
                h = goEuclidean(dx, dy);
                break;
            case MANHATTAN:
                h = goManhattan(dx,dy);
                break;
            case CHECKERS:
                h = goCheckers(dx,dy);
                break;
            default:
                assert(false);
        }

        return h;
    }

    private double goCheckers(double dx, double dy) {
        return Math.max(Math.abs(dx), Math.abs(dy));
    }

    private double goSSE(double dx, double dy) {
        return dx * dx + dy * dy;
    }

    private double goManhattan(double dx, double dy) {
        return Math.abs(dx) + Math.abs(dy);
    }

    private double goEuclidean(double dx, double dy) {
        return Math.sqrt(goSSE(dx, dy));
    }

    /**
     * Find lowest cost node. Could be improved if nodes added using insertion
     * sort.
     *
     * @return Node with lowest cost.
     */
    protected Node getLowestCostNode() {
        ListIterator iter = openList.listIterator();
        double minCost = Double.MAX_VALUE;
        Node minNode = null;
        while (iter.hasNext()) {
            Node node = (Node) iter.next();
            double cost = node.getCost();
            if (cost < minCost) {
                minCost = cost;
                minNode = node;
            }
        }
        return minNode;
    }

    /**
     * Reset the offset index.
     */
    protected void reset() {
        indOffset = 0;
    }

    public char[][] getTileMap() {
        return tileMap;
    }

    /**
     * Load the tile map from a file. Must invoke constructor with file
     * parameter before invoking this method.
     */
    public void loadMap() {
        try {
            StringTokenizer dims = new StringTokenizer(reader.readLine());

            width = Integer.parseInt(dims.nextToken());

            height = Integer.parseInt(dims.nextToken());

            tileMap = new char[height][width];

            for (int k = 0; k < height; k++) {
                String row = reader.readLine();

                if (row.length() != width) {
                    throw new Exception("bad row width");
                }

                for (int j = 0; j < width; j++) {
                    char sym = row.charAt(j);
                    tileMap[k][j] = sym;

                    if (sym == SYMBOL_DEST) {
                        destX = j;
                        destY = k;
                    }

                    if (sym == SYMBOL_START) {
                        startX = j;
                        startY = k;
                    }
                }
            }
        } catch (Exception e) {
        }
    }

    /**
     * Main method (for debugging).
     *
     * @param args the command line arguments
     */
    public static void main(String[] args) {
        // Set up the random seed
        long seed = System.currentTimeMillis();
        
        if(args.length != 0)
            seed = Long.parseLong(args[0]);
       
        // Get the world configuration and create the world
        Config config = Config.getInstance();
        
        int width = config.map.width;
        int height = config.map.height;
        LevelGenerator world = new LevelGenerator(width, height, seed);

        // Set start to the upper left corner
        world.layout(0,0);

        // Create an A* pathfinder for this world
        Node.idCount = 0;
        Astar astar = new Astar(world.getMap(), world.getStartX(), world.getStartY(), world.getDestX(), world.getDestY());
        
        // Find a path for the configured objective
        Objective objective = config.objective;
        
        Node path = astar.find(objective);

        // If we find a path, output the statistics
        if (path != null) {
            // Draw the path in the world
            world.walk(path);
            
            // Output the stats
            System.out.println(world);

            System.out.println(path+"\n");
            
            System.out.println("path length: "+path.length());
        } else {
            System.out.println("NO PATH !");
        }
        
        // Output the remaining statistics
        System.out.println("node count: "+Node.idCount);
                
        System.out.println("seed: "+seed);
    }
}
