package ca.mcgill.ecse211.dreamteamrobot.brick1.pathfinding;

import ca.mcgill.ecse211.dreamteamrobot.brick1.kinematicmodel.KinematicModel;
import ca.mcgill.ecse211.dreamteamrobot.brick1.navigation.Location;

import java.util.ArrayList;
import java.util.List;

/**
 * Class containing static methods for generating and manipulating paths between points on the board.
 */
public class PathFinder {

    /** Static Variables */
    public static Graph board;

    /** Variables For Static Procedures */
    public static List<Integer> listBlockPath;
    public static List<Integer> recSearchPreviouslySeenVertices;

    /**
     * Sets up board without any obstacles.
     */
    public static void setupFullyConnectedBoard () {
        // Connect all the things!
        for (int i = 0 ; i < 144 ; i++ ) {

            switch (i) {
                // Special consideration for corners.
                case 0:
                    board.addEdge(0, 12);
                    board.addEdge(0, 1);
                    break;
                case 11:
                    board.addEdge(11, 10);
                    board.addEdge(11, 23);
                    break;
                case 143:
                    board.addEdge(143, 142);
                    board.addEdge(143, 131);
                    break;
                case 132:
                    board.addEdge(132, 133);
                    board.addEdge(132, 120);
                    break;
                // Left vertical band between corners.
                case 12:
                case 24:
                case 36:
                case 48:
                case 60:
                case 72:
                case 84:
                case 96:
                case 108:
                case 120:
                    board.addEdge(i, i-12);
                    board.addEdge(i, i+12);
                    board.addEdge(i, i+1);
                    break;
                // Bottom horizontal band between corners.
                case 1:
                case 2:
                case 3:
                case 4:
                case 5:
                case 6:
                case 7:
                case 8:
                case 9:
                case 10:
                    board.addEdge(i, i-1);
                    board.addEdge(i, i+1);
                    board.addEdge(i, i+12);
                    break;
                // Right vertical band between corners.
                case 23:
                case 35:
                case 47:
                case 59:
                case 71:
                case 83:
                case 95:
                case 107:
                case 119:
                case 131:
                    board.addEdge(i, i-1);
                    board.addEdge(i, i+12);
                    board.addEdge(i, i-12);
                    break;
                // Top horizontal band between corners.
                case 133:
                case 134:
                case 135:
                case 136:
                case 137:
                case 138:
                case 139:
                case 140:
                case 141:
                case 142:
                    board.addEdge(i, i-1);
                    board.addEdge(i, i+1);
                    board.addEdge(i, i-12);
                    break;
                // Everything in the center 11x11 box.
                default:
                    board.addEdge(i, i-1); // add connection to left node
                    board.addEdge(i, i+1); // add connection to right node
                    board.addEdge(i, i-12); // add connection to below node
                    board.addEdge(i, i+12); // add connection to above node
                    break;

            }

        }

    }

    /**
     * Removes edges into defense zone.
     */
    public static void blockOutDefenseZone () {
        int positionOfDefenderLine = KinematicModel.roundData.get("d1"); // [1, 5]
        for (int j = 0 ; j < positionOfDefenderLine ; j++ ) {
            for (int i = (122 - 12*j) ; i <= (129 - 12*j) ; i++ ) {
                board.disconnectVertex(i);
            }
        }
    }

    /**
     * Removes edges into ball box.
     */
    public static void blockOutBallBox () {
        int ll_x = KinematicModel.roundData.get("ll-x");
        int ll_y = KinematicModel.roundData.get("ll-y");
        int ur_x = KinematicModel.roundData.get("ur-x");
        int ur_y = KinematicModel.roundData.get("ur-y");

        // TODO: Figure out efficient method for blocking out ball box.
    }

    /**
     * Generates path list between a start point and end point in (x,y) format.
     * @param startPoint Start point in (x,y) format.
     * @param endPoint End point in (x,y) format.
     * @return A path in the form of a list of (x,y) destinations or null if no path exists.
     */
    public static List<Location> generatePath (Location startPoint, Location endPoint) {

        /** 1. Figure out what block startPoint is in. */

        int divisibleX = ((int) startPoint.getX()) / 30;
        double remainderX = (startPoint.getX()) % 30.00;
        int divisibleY = ((int) startPoint.getY()) / 30;
        double remainderY = (startPoint.getY()) % 30.00;

        int startBlock;
        // If 30 is divisible into start point x at least once, then
        // the block is between 2 and 11 in the x direction.
        if (divisibleX > 0) startBlock = divisibleX + 1;
        else {
            if (remainderX > 0) startBlock = 1;
            else startBlock = 0;
        }
        // Now we need to figure out how 'high' on the board the current block is.
        if (divisibleY > 0) startBlock += (divisibleY+1) * 12;
        else {
            if (remainderY > 0) startBlock += 12;
        }

        /** 2. Figure out what block endPoint is in. */

        divisibleX = ((int) endPoint.getX()) / 30;
        remainderX = (endPoint.getX()) % 30.00;
        divisibleY = ((int) endPoint.getY()) / 30;
        remainderY = (endPoint.getY()) % 30.00;

        int endBlock;
        // If 30 is divisible into start point x at least once, then
        // the block is between 2 and 11 in the x direction.
        if (divisibleX > 0) endBlock = divisibleX + 1;
        else {
            if (remainderX > 0) endBlock = 1;
            else endBlock = 0;
        }
        // Now we need to figure out how 'high' on the board the current block is.
        if (divisibleY > 0) endBlock += (divisibleY+1) * 12;
        else {
            if (remainderY > 0) endBlock += 12;
        }

        /** 3. Determine block path. */
        List<Integer> blockPath = generateBlockPath(startBlock, endBlock);
        if (blockPath == null) return null;

        /** 4. Simplify block path */
        // TODO: Develop algorithm for simplifying block path.


        /** 5. Return. */
        return convertBlockPathToLocationPath(blockPath);

    }

    /**
     * Given starting block and ending block, generates path between them.
     * @param startBlock start block
     * @param endBlock end block
     * @return Path between startBlock and endBlock if such a path exists. Null if not path exists.
     */
    private static List<Integer> generateBlockPath (int startBlock, int endBlock) {

        // Reset path list and search history.
        listBlockPath = new ArrayList<>();
        recSearchPreviouslySeenVertices = new ArrayList<>();

        // Run breadth-first search.
        boolean wasPathGenerated = recSearch(startBlock, endBlock);

        // If breadth-first search found a path, return it. If
        // it didn't, return null.
        if (wasPathGenerated) return listBlockPath;
        else return null;

    }

    /**
     * Recursive depth-first search modified to save path to end block.
     * @param currentVertex Current node in the graph.
     * @param searchingForVertex Node being searched for.
     * @return
     */
    private static boolean recSearch (int currentVertex, int searchingForVertex) {

        // TODO: Write actual breadth-first search...

        // Check if current vertex is the one we want.
        // If it is, add it to the list (it will be the first one / last one on the list)
        // and return true.
        if (currentVertex == searchingForVertex) {
            listBlockPath.add(currentVertex);
            return true;
        }

        // If we didn't get lucky, continue search.
        // Grab all neighbours to the current vertex.
        List<Integer> listNeighbours = board.getVerticesAdjacentTo(currentVertex);
        // Remove any vertices that we've seen before from the list.
        for (Integer neighbour : listNeighbours) {
            if (recSearchPreviouslySeenVertices.contains(neighbour)) listNeighbours.remove(neighbour);
        }

        // If it has no new neighbours, return false (clearly vertex we want is not here)
        if (listNeighbours.size() == 0) return false;

        // Otherwise go through list of neighbours, checking if any lead to vertex we want.
        for (Integer currentNeighbour : listNeighbours) {
            // Mark that we've seen this vertex.
            recSearchPreviouslySeenVertices.add(currentNeighbour);
            // If this neighbour is or continues path to searchingForVertex,
            // then add it to the block path list (at the beginning) and return true.
            if (recSearch(currentNeighbour, searchingForVertex)) {
                listBlockPath.add(0, currentVertex);
                return true;
            }
        }

        return false;

    }

    /**
     * Converts a path represented by a list of blocks into a path represented by a list of locations (x,y).
     * @param blockPath Block path to convert.
     * @return Block path converted to location path.
     */
    private static List<Location> convertBlockPathToLocationPath (List<Integer> blockPath) {

        List<Location> locationPath = new ArrayList<>();

        for (Integer currentBlock : blockPath) {

            // Determine y position of block.
            double positionY = 0.00; // placeholder value so compiler doesn't freak out.
            int count = 0;
            for (int i = 11 ; i < 144 ; i += 12) {
                if (currentBlock <= i) {
                    positionY = -15.00 + count*30.00;
                    break;
                }
                count++;
            }

            // Determine x position of block.
            int remainder = currentBlock % 12;
            double positionX = -15.00 + remainder*30.00;

            // Add new (x,y) location to locationPath.
            locationPath.add(new Location(positionX, positionY));

        }

        return locationPath;

    }

}
