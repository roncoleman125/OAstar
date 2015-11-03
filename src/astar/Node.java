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

/**
 * This class implements the node in A* pathfinding.
 * @author Ron Coleman
 */
public class Node {

    /**
     * Total number of nodes created.
     */
    
    public static int idCount;
    private int x;
    private int y;
    private Node parent;
    private Node child;
    private double cost = 0;
    private int steps = 0;
    private int id;
    private double inertia;

    /**
     * Constructor.
     *
     * @param x X coordinate.
     * @param y Y coordinate.
     */
    public Node(int x, int y) {
        this.x = x;
        this.y = y;
        this.cost = Long.MAX_VALUE;
        this.id = idCount++;
    }

    /**
     * Copy constructor
     *
     * @param node Node to copy.
     */
    public Node(Node node) {
        this(node.x, node.y);
        this.id = idCount++;
    }

    /**
     * Get the parent node.
     *
     * @return Parent node.
     */
    public Node getParent() {
        return parent;
    }
    
    /**
     * Sets the parent of this node
     * @param node Parent node
     */
    public void setParent(Node node) {
        this.parent = node;
        this.parent.child = node;
    }

    /**
     * Get cost of the node, namely, steps plus heuristic.
     *
     * @return Cost.
     */
    public double getCost() {
        return cost;
    }

    /**
     * Tests if this node equal to another node.
     *
     * @param node Node.
     * @return True, if two nodes equal, false otherwise.
     */
    public boolean equals(Node node) {
        return node.x == x && node.y == y;
    }

    /**
     * Gets the node at x, y if it exists going forward this node.
     *
     * @param x
     * @param y
     * @return
     */
    public Node getNode(int x, int y) {
        if (equals(x, y)) {
            return this;
        }

        Node anode = this.getChild();
        while (anode != null) {
            if (anode.equals(x, y)) {
                return anode;
            }
            anode = anode.getChild();
        }
        return null;
    }

    /**
     * Tests if the node is equal to the x, y coordinate.
     *
     * @param x
     * @param y
     * @return
     */
    public boolean equals(int x, int y) {
        return this.x == x && this.y == y;
    }

    /**
     * Get X coordinate of node.
     *
     * @return X coordinate.
     */
    public int getX() {
        return x;
    }

    /**
     * Get Y coordinate of node.
     *
     * @return Y coordinate.
     */
    public int getY() {
        return y;
    }

    /**
     * Get steps from parent.
     *
     * @return Steps from parent.
     */
    public int length() {
        steps = 1;
        
        Node node = this;
        
        while(node.getParent() != null) {
            steps++;
            
            node = node.getParent();
            
        }
        
        return steps;
    }

    /**
     * Set cost of node.
     *
     * @param cost Cost of node.
     */
    public void setCost(double cost) {
        this.cost = cost;
    }

    /**
     * Returns string representation of the node.
     * @return String representation of the node
     */
    @Override
    public String toString() {
        
        StringBuilder sb = new StringBuilder();
        Node node = this;

        do {
            sb.append("[").append(LevelGenerator.encode(node.getX())).append(",").append(LevelGenerator.encode(node.getY())).append("]");
            
            if(node.getParent() != null)
                sb.append(" <- ");

            node = node.getParent();
        } while (node != null);
        
        return sb.toString();
    }

    public Node getChild() {
        return child;
    }

    /**
     * Gets the path inertia.
     * @return Double
     */
    public double getInertia() {
        return inertia;
    }

    /**
     * Sets the child of this node.
     * @param child 
     */
    public void setChild(Node child) {
        this.child = child;
    }

    /**
     * Sets number of steps of this node.
     * @param steps Number of steps
     */
    public void setSteps(int steps) {
        this.steps = steps;
    }

    /**
     * Sets Y coordinate of this node.
     * @param y Y coordinate
     */
    public void setY(int y) {
        this.y = y;
    }

    /**
     * Sets X coordinate of this node.
     * @param x 
     */
    public void setX(int x) {
        this.x = x;
    }

    /**
     * Sets inertia of this node.
     * @param strength Inertia
     */
    public void setInertia(double strength) {
        this.inertia = strength;
    }

    /**
     * Increments the inertia.
     */
    public void incrementInertia() {
        inertia += 1;
    }
}
