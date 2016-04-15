package ca.mcgill.ecse211.dreamteamrobot.brick1.pathfinding;

/******************************************************************************
 *  ATTENTION: This class is loosely based on code from
 *  http://algs4.cs.princeton.edu/41graph/AdjMatrixGraph.java.html
 ******************************************************************************/

import java.util.ArrayList;
import java.util.List;

/**
 * Undirected graph used for modelling game board. Stored with adjacency matrix.
 */
public class Graph {

    /** Variables */
    private int V;
    private int E;
    private boolean[][] adj;

    /** Constructor.
     * Creates empty graph with specified number of nodes.
     * @param V Number of nodes in empty graph.
     */
    public Graph(int V) {
        if (V < 0) throw new RuntimeException("Number of vertices must be nonnegative");
        this.V = V;
        this.E = 0;
        this.adj = new boolean[V][V];
    }

    /**
     * @return Number of vertices in graph.
     */
    public int getV() { return V; }

    /**
     * @return Number of edges in graph.
     */
    public int getE() { return E; }

    /**
     * Adds an edge to the graph.
     * @param v One end of edge.
     * @param w Other end of edge.
     */
    public void addEdge (int v, int w) {
        if (!adj[v][w]) E++;
        adj[v][w] = true;
        adj[w][v] = true;
    }

    /**
     * Removes an edge from the graph.
     * @param v One end of the edge.
     * @param w Other end of the edge.
     */
    public void removeEdge (int v, int w) {
        // If the edge exists, decrease the Edge Count
        // and delete the edge.
        if (adj[v][w]) E--;
        adj[v][w] = false;
        adj[w][v] = false;
    }

    /**
     * Removes all edges associated with the given vertex.
     * @param vertex Vertex to remove edges for.
     */
    public void disconnectVertex (int vertex) {
        for (int i = 0 ; i < V ; i++ ) {
            if (adj[vertex][i]) E--;
            adj[vertex][i] = false;
            adj[i][vertex] = false;
        }
    }

    /**
     * Checks if the given edge exists in the graph.
     * @param v First parameter in edge.
     * @param w Second parameter in edge.
     * @return True if edge exists. False if not.
     */
    public boolean containsEdge (int v, int w) {
        return adj[v][w];
    }

    /**
     * @param givenVertex Vertex to evaluate.
     * @return List of vertices adjacent to v.
     */
    public List<Integer> getVerticesAdjacentTo (int givenVertex) {
        List<Integer> listNeighbours = new ArrayList<>();
        for (int i = 0; i < V; i++) {
            if (adj[givenVertex][i]) listNeighbours.add(i);
        }
        return listNeighbours;
    }


}