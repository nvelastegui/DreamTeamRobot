package ca.mcgill.ecse211.dreamteamrobot.brick1.pathfinding;

import ca.mcgill.ecse211.dreamteamrobot.brick1.kinematicmodel.KinematicModel;
import ca.mcgill.ecse211.dreamteamrobot.brick1.navigation.Location;

import java.util.*;

/**
 * Class containing static methods for generating and manipulating paths between points on the board.
 */
public class PathFinder {

    /** Static Variables */
    public static Graph board;

    /** Variables For Static Procedures */
    public static Queue<bfsSearchNode> bfsQueue;
    public static List<Integer> visitedNodes;

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

        // Grab bounding parameters for ball box.
        int ll_x = KinematicModel.roundData.get("ll-x") + 1; // [-1, 11] -> if it's 10, corresponds to box 11 -> [0, 11]
        int ll_y = KinematicModel.roundData.get("ll-y") + 1; // [-1, 11] -> if it's 10, corresponds to box 11 -> [0, 11]
        int ur_x = KinematicModel.roundData.get("ur-x"); // [-1, 11] -> [0, 11]
        int ur_y = KinematicModel.roundData.get("ur-y"); // [-1, 11] -> [0, 11]

        // Block out the box.
        for (int x = ll_x ; x <= ur_x ; x++) {
            for (int y = ll_y ; y <= ur_y ; y++) {
                int block = x + y*12;
                board.disconnectVertex(block);
            }
        }

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
        List<Location> simplifiedPath = simplifyLocationList(convertBlockPathToLocationPath(blockPath));

        System.out.println("\n\nGENERATED NEW PATH\n");
        for (Location current : simplifiedPath) {
            System.out.println("Location: (" + current.getX() + "," + current.getY() + ")");
        }
        System.out.println("\n");


        /** 5. Return. */
        return simplifiedPath;

    }

    /**
     * Given starting block and ending block, generates path between them.
     * @param startBlock start block
     * @param endBlock end block
     * @return Path between startBlock and endBlock if such a path exists. Null if no path exists.
     */
    private static List<Integer> generateBlockPath (int startBlock, int endBlock) {

        // Reset path list and search history.
        bfsQueue = new LinkedList<>();
        visitedNodes = new ArrayList<>();

        // Get the raw block path from BFS
        List<Integer> blockPathWithStartBlockAtFront = bfsSearch(startBlock, endBlock);
        if (blockPathWithStartBlockAtFront == null) return null;

        // We don't want the starting block in the path (at the start) because having the
        // first destination be so close to the starting point will mess up navigator.
        // So we remove the first block from the list, which corresponds to (x,y) in the center
        // of the block that the robot starts in.
        blockPathWithStartBlockAtFront.remove(0);

        // Return block path now
        return blockPathWithStartBlockAtFront;

    }

    /**
     * Iterative breadth-first search modified to output shortest path to end block.
     * @param root Current node in the graph.
     * @param searchingForVertex Node being searched for.
     * @return
     */
    private static List<Integer> bfsSearch (int root, int searchingForVertex) {

        // Add the root to visited nodes.
        visitedNodes.add(root);

        // Add the root to the queue (using the bfsSearchNode structure).
        bfsSearchNode rootNode = new bfsSearchNode(root);
        bfsQueue.add(rootNode);

        while (!bfsQueue.isEmpty()) {

            // Grab top node off queue.
            bfsSearchNode current = bfsQueue.poll();

            // System.out.println("Entering while. Current node: " + current.getBlock());

            // Get path up to that node (ie. path from root to current)
            List<Integer> pathToNode = current.getPathToNode();

            // System.out.println("Current path to node: ");
//            for (Integer whtever : pathToNode) {
//                System.out.println(whtever);
//            }

            // Is it the vertex we want?
            if (current.getBlock() == searchingForVertex) {
                // If it is, return the path to it plus itself appended to the end.
                pathToNode.add(current.getBlock());
                return pathToNode;
            }

            // System.out.println("Now checking neighbours...");

            // If it's not, add its neighbours to the queue and
            // log their individual paths.
            List<Integer> neighboursOfCurrent = board.getVerticesAdjacentTo(current.getBlock());
            for (Integer neighbourOfCurrent : neighboursOfCurrent) {
                // ONLY if we have not visited the vertex before...
                if (!visitedNodes.contains(neighbourOfCurrent)) {
                    // System.out.println("Unseen node: " + neighbourOfCurrent);
                    // Log that we have visited it.
                    visitedNodes.add(neighbourOfCurrent);
                    // Create a new bfsSearchNode for it and add it to queue.
                    List<Integer> weGonBeAlright = new ArrayList<>();
                    weGonBeAlright.addAll(pathToNode);
                    weGonBeAlright.add(current.getBlock());
                    bfsSearchNode newNode = new bfsSearchNode(
                            weGonBeAlright,
                            neighbourOfCurrent
                    );
                    bfsQueue.add(newNode);
                }
            }
        }

        // If we never found anything, it's because no path exists.
        return null;

    }

    /**
     * Determines board block given x and y coordinates.
     * @param x x position
     * @param y y position
     * @return Block number.
     */
    public static int determineBlockByCoordinate (double x, double y) {

        int divisibleX = ((int) x) / 30;
        double remainderX = (x) % 30.00;
        int divisibleY = ((int) y) / 30;
        double remainderY = (y) % 30.00;

        int block;
        // If 30 is divisible into start point x at least once, then
        // the block is between 2 and 11 in the x direction.
        if (divisibleX > 0) block = divisibleX + 1;
        else {
            if (remainderX > 0) block = 1;
            else block = 0;
        }
        // Now we need to figure out how 'high' on the board the current block is.
        if (divisibleY > 0) block += (divisibleY+1) * 12;
        else {
            if (remainderY > 0) block += 12;
        }

        return block;
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

    /**
     * Analyzes a list of locations and removes fluff (multiple locations along a line).
     * @param expensiveList List to be simplified.
     * @return Simplified list.
     */
    private static List<Location> simplifyLocationList (List<Location> expensiveList) {

        // If the list is small just keep it as is.
        if (expensiveList.size() < 3) return expensiveList;

        // Crete return list.
        List<Location> simplifiedPath = new ArrayList<>();

        // Initialize the direction and set 'last' location to first in expensiveList.
        // 0 - vertical, 1 - horizontal
        int direction = determineDirection(expensiveList.get(0), expensiveList.get(1));
        // Log initial value for last location.
        Location lastLocation = expensiveList.get(0);

        // Add the first location to the simplified list and remove it from the expensive list.
        simplifiedPath.add(lastLocation);
        expensiveList.remove(0);

        // Cycle through expensive list
        while (expensiveList.size() > 0) {

            // Grab current location.
            Location current = expensiveList.get(0);

            // Grab next location
            Location nextLocation;
            try {
                nextLocation = expensiveList.get(expensiveList.indexOf(current) + 1);
            } catch (NullPointerException|IndexOutOfBoundsException e) {
                // If this is the last location in the list, then just add it the simplified path and break.
                simplifiedPath.add(current);
                break;
            }

            // Check if trio (last, current, next) are aligned (same direction).
            // If they are, remove the current location from expensive list and keep the direction.
            if (areSameDirection(direction, lastLocation, current) && areSameDirection(direction, current, nextLocation)) {
                // Update direction.
                direction = determineDirection(lastLocation, current);
                // Update last location.
                lastLocation = current;
                // Remove current from the expensive list.
                expensiveList.remove(current);
            }
            // If the three are not in the same direction...
            else {
                // Update the direction.
                direction = determineDirection(current, nextLocation);
                // Add the current location to the simplified list.
                simplifiedPath.add(current);
                // Set the previous location properly.
                lastLocation = current;
                // Remove it from expensive list.
                expensiveList.remove(current);
            }

        }

        return simplifiedPath;

    }

    /**
     *
     * @param direction
     * @param lastLocation
     * @param currentLocation
     * @return
     */
    private static boolean areSameDirection (int direction, Location lastLocation, Location currentLocation) {
        switch (direction) {
            case 0:
                return (lastLocation.getY() == currentLocation.getY());
            case 1:
                return (lastLocation.getX() == currentLocation.getX());
            default:
                return false;
        }
    }

    /**
     * @param lastLocation Previous location.
     * @param currentLocation Next location.
     * @return Direction created by two locations (horizontal (1) or vertical (0)).
     */
    private static int determineDirection (Location lastLocation, Location currentLocation) {
        if (lastLocation.getX() == currentLocation.getX()) return 1;
        else return 0;
    }


}

















