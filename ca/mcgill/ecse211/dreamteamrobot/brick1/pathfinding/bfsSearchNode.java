package ca.mcgill.ecse211.dreamteamrobot.brick1.pathfinding;

import java.util.ArrayList;
import java.util.List;

/**
 * Class used for breadth-first search of board's graph. It holds a block of the board (a node) and the path
 * leading up to it from the root (start block).
 */
public class bfsSearchNode {

    List<Integer> pathToNode;
    Integer block;

    /**
     * Constructor
     * @param path Path to this vertex/node/block.
     * @param block Block.
     */
    public bfsSearchNode (List<Integer> path, Integer block) {
        this.pathToNode = path;
        this.block = block;
    }

    /**
     * Special constructor for root block (no path to it)
     * @param block Block.
     */
    public bfsSearchNode (Integer block) {
        this.pathToNode = new ArrayList<>();
        this.block = block;
    }

    /**
     * @return Block held in bfsSearchNode.
     */
    public Integer getBlock() {
        return block;
    }

    /**
     * @return Path to block, excluding itself.
     */
    public List<Integer> getPathToNode() {
        return pathToNode;
    }
}