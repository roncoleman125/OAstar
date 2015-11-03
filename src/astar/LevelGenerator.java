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

import java.util.Random;

/**
 * This class generates a random world.
 * @author Ron Coleman
 */
public class LevelGenerator {
  public final static int INSET = 1;
  public final static double BARRIER_FACTOR = 0.45;
  protected char tileMap[][];
  protected int width;
  protected int height;
  protected int startX;
  protected int startY;
  protected int destX;
  protected int destY;
  protected int minXDist;
  protected int minYDist;
  protected Random ran = new Random();
   
  /** Creates a new instance of LevelGenerator
     * @param width Width of map
     * @param height Height of map
     * @param seed Random seed
     */
  public LevelGenerator(int width, int height,long seed) {
    this.width = width;
    this.height = height;
      
    tileMap = new char[height][width];
        
    for(int i=0; i < height; i++) {
      for(int j=0; j < width; j++) {
        tileMap[i][j] = Astar.SYMBOL_FREE;
      }
    }
    
    ran.setSeed(seed);
    
    minXDist = (int) (0.25 * width);
    minYDist = (int) (0.25 * height);
    
    //layoutSrcDest();
   
    //layoutBarriers();
  }
  
  /**
   * Gets the tile map.
   * @return 2D char array
   */
  public char[][] getMap() {
    return tileMap;
  }
  
    /**
   * Gets start X coordinate.
   * @return Integer
   */
  public int getStartX() {
    return startX;
  }
  
    /**
   * Gets start Y coordinate.
   * @return Integer
   */
  public int getStartY() {
    return startY;
  }
    /**
   * Gets destination X coordinate.
   * @return Integer
   */
  public int getDestX() {
    return destX;
  }
  
  /**
   * Gets destination Y coordinate.
   * @return Integer
   */
  public int getDestY() {
    return destY;
  }
  
  /**
   * Returns string representation of the level.
   * @return String
   */
  @Override
  public String toString() {
    
    StringBuilder sb = new StringBuilder();
    
    // Print the header row
    sb.append("  ");
    
    for(int w=0; w < width; w++)
        sb.append(encode(w)).append(" ");
    sb.append("\n");
    
    // Print each cell of the map by rows
    for(int h=0; h < height; h++) {
      sb.append(encode(h)).append(" ");
      
      // print the colums of each row
      for(int w=0; w < width; w++) {
        sb.append(tileMap[w][h]).append(" ");
      }
      sb.append("\n");
    }
    
    return sb.toString();
  }
  
  public void layout(int startX,int startY) {
      this.startX = startX;
      this.startY = startY;
      
      layoutDest();

      tileMap[startY][startX] = Astar.SYMBOL_START;
      tileMap[destY][destX] = Astar.SYMBOL_DEST;
      
      this.layoutObstacles();
  }
  
  public void layout() {
      layoutStart();
      
      layoutDest();
      
      tileMap[startY][startX] = Astar.SYMBOL_START;
      tileMap[destY][destX] = Astar.SYMBOL_DEST;
      
      layoutObstacles();
  }
  
  protected void layoutStart() {
    do {
      startX = ran.nextInt(width);
      startY = ran.nextInt(height);
    } while(startX - INSET <= 0 || width - startX <= INSET || startY - INSET <= 0 || height - startY <= INSET);      
  }
  protected void layoutDest() {
     do {
      destX = ran.nextInt(width);
      destY = ran.nextInt(height);
    } while(Math.abs(destX - startX) <= minXDist || Math.abs(destY - startY) <= minYDist ||
            destX - INSET <= 0 || width - destX <= INSET || destY - INSET <= 0 || height - destY <= INSET);     
  }
    
//  public void layoutStartDest() {  
//    layoutStart();
//    layoutDest();
//      
//    tileMap[startY][startX] = Astar.SYMBOL_START;
//    tileMap[destY][destX] = Astar.SYMBOL_DEST;     
//  }
  
  public void walk(Node node) {
      Node end = node;
      do {
          int x = node.getX();
          
          int y = node.getY();
          
          // Don't step on start or destination
          if(node != end && node.getParent() != null)
            tileMap[y][x] = Astar.SYMBOL_STEP;
          
          node = node.getParent();
      } while(node != null);
  }
    
  public void layoutObstacles() {
	/*
    // Put down horizontal barrier
    if(startY - destY > 0) {
      // Along top of src
      for(int k=startX-2; k < startX+2; k++)
        tileMap[k][startY-2] = Astar.SYM_OBSTACLE;
    }
    else {
      // Along bottom of dest
      for(int k=startX-2; k < startX+2; k++)
        tileMap[k][startY+2] = Astar.SYM_OBSTACLE;
    }

    // Put down vertical barrier
    int side = ran.nextInt(2) == 0 ? -2 : 2;
    for(int j=startY-2; j < startY+2; j++)
      tileMap[startX+side][j] = Astar.SYM_OBSTACLE; 
    */
	  
    // Randomly deposit obstacles
    int numBarriers = (int) (width * height * BARRIER_FACTOR + 0.5);
    
    for(int i=0; i < numBarriers; i++) {
      int x = ran.nextInt(height);
      int y = ran.nextInt(height);
      
      if(x == startX && y == startY)
         continue;
      
      if(x == destX && y == destY)
        continue;
      
      tileMap[y][x] = Astar.SYMBOL_OBSTACLE;
    }
  }     
  
  public static char encode(int index) {
    final String encode = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
    
    assert(index >= 0 && index < encode.length());
    
    return encode.charAt(index);
  }
}

