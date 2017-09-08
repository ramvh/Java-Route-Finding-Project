/*
 * @author Haley Mar
 * Date: 6/30/2017
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between crossing roads
 *
 */
 
package roadgraph;


import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;


public class MapGraph {
	
	// A map relating points to map nodes. All nodes are accessible through this map
	private HashMap<GeographicPoint, MapNode> vertices;
	// A map of all edges
	private Map<Integer, MapEdge> edges;
	
	//Constructor
	public MapGraph()
	{		
		vertices = new HashMap<GeographicPoint, MapNode>();
		edges = new HashMap<Integer, MapEdge>();
	}
	
	// Return vertices collection
	public HashMap<GeographicPoint, MapNode> getVerticesMap(){
		return this.vertices;		
	}
	
	// Return edge collection
	public Map<Integer, MapEdge> getEdgeMap(){
		return edges;
	}
	
	// Return The number of vertices (road intersections) in the graph.
	public int getNumVertices()
	{
		return vertices.values().size();
	}
	
	// Return The vertices (intersections) in this graph as GeographicPoints
	public Set<GeographicPoint> getVertices()
	{
		return vertices.keySet();
	}
	
	// Return The number of edges (road segments) in the graph.	
	public int getNumEdges()
	{
		return edges.size();
	}

	// Add a node corresponding to an intersection 
	public void addVertex(double lat, double lon){
		GeographicPoint gp = new GeographicPoint(lat, lon);
		this.addVertex(gp);		
	}
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		MapNode node = vertices.get(location);
		if(node == null){
			node = new MapNode(location);
			vertices.put(location, node);
		} else {
			System.out.println("Warning: Node at " + location + " already exists on map." );
		}
		return false;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, 
			String roadName, String roadType, double length) {

		MapNode n1 = vertices.get(from);
		MapNode n2 = vertices.get(to);
		
		if(n1 == null){
			throw new NullPointerException("addEdge: 'from' location: " + from + " is not in graph.");
		}
		if(n2 == null){
			throw new NullPointerException("addEdge: 'to' location: " + to + " is not in graph.");
		}
		addEdge(n1,n2,roadName,roadType,length);
	}

	
	public void addEdge(GeographicPoint from, GeographicPoint to, 
			String roadName, String roadType){

		MapNode n1 = vertices.get(from);
		MapNode n2 = vertices.get(to);
		
		if(n1 == null){
			throw new NullPointerException("addEdge: 'from' location: " + from + " is not in graph.");
		}
		if(n2 == null){
			throw new NullPointerException("addEdge: 'to' location: " + to + " is not in graph.");
		}
		addEdge(n1,n2,roadName,roadType,MapEdge.DEFAULT_LENGTH);
	}
	
	
	public void addEdge(double lat1, double lon1, 
			double lat2, double lon2, String roadName,
			String roadType){

		GeographicPoint pt1 = new GeographicPoint(lat1, lon1);
		GeographicPoint pt2 = new GeographicPoint(lat2, lon2);
		
		MapNode n1 = vertices.get(pt1);
		MapNode n2 = vertices.get(pt2);
		
		if(n1 == null){
			throw new NullPointerException("addEdge: pt1: " + pt1 + " is not in graph.");
		}
		if(n2 == null){
			throw new NullPointerException("addEdge: pt2" + pt2 + " is not in graph.");
		}
		addEdge(n1,n2,roadName,roadType,MapEdge.DEFAULT_LENGTH);
	}	
	
	
	public void addEdge(MapNode n1, MapNode n2, 
			String roadName, String roadType, double length) {
		
		MapEdge edge = new MapEdge(roadName, roadType, n1, n2, length);
		edges.put(GraphUtility.generateHashMapKeyForEdge(n1, n2), edge);
		n1.addEdge(edge);		
	}
	
	
	/** Find the path from start to end using breadth first search
	 * 
	 * @param start The starting location
	 * @param end The end location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to end (including both start and end).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint end) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, end, temp);
	}
	
	/** Find the path from start to end using breadth first search
	 * 
	 * @param start The starting location
	 * @param end The end location
	 * @param nodeSearched A hook for visualization.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to end (including both start and end).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint end, 
			 					     Consumer<GeographicPoint> nodeSearched)
	{
		if(start == null || end == null) throw new NullPointerException();
		
		MapNode startNode = vertices.get(start);
		MapNode endNode = vertices.get(end);
		
		if(startNode == null){
			System.err.println("Start node: " + startNode + " does not exist");
		}
		if(endNode == null){
			System.err.println("End node: " + endNode + " does not exist");
		}
		
		HashMap<MapNode,MapNode> parent = new HashMap<MapNode,MapNode>();
		Queue<MapNode> q = new LinkedList<MapNode>();	
		HashSet<MapNode> visited = new HashSet<MapNode>();
		q.add(startNode);
		MapNode next = null;
		
		while(!q.isEmpty()){
			// Prep for next node
			next = q.remove();
			// Hook for visualization
			nodeSearched.accept(next.getLocation());
			
			if(next.equals(endNode)) break;
			
			// Get neighboring vertices
			HashSet<MapNode> neighbors = getNeighbor(next);
			// Loop through all unvisited neighbors, add to queue.
			for(MapNode neighbor : neighbors){
				if(!visited.contains(neighbor)){
					visited.add(neighbor);
					parent.put(neighbor, next);
					q.add(neighbor);
				}
			}
		}		
		if(!next.equals(endNode)){
			System.err.println("No path found from " + start + " to" + end);
			return null;
		}
		// Reconstruct the parent path
		List<GeographicPoint> path = reconstructPath(parent, startNode, endNode);
		return path;
	}
	
	// Reconstruct a path from start to finish using the parent MapNode
	private List<GeographicPoint> reconstructPath(HashMap<MapNode, MapNode> parent, 
			MapNode startNode,
			MapNode endNode) {
		
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		MapNode current = endNode;
		
		while(!current.equals(startNode)){
			path.addFirst(current.getLocation());
			current = parent.get(current);
		}
		
		// Add start
		path.addFirst(startNode.getLocation());
		return path;
	}

	private Set<MapNode> getNeighbor(MapNode node){		
		return node.getNeighbor();
	}

	/////////////////////////////////////////////////////////////////////////////
	// For DEBUGGING.  Print the Nodes in the graph
/*		public void printNodes()
		{
			System.out.println("****PRINTING NODES ********");
			System.out.println("There are " + getNumVertices() + " Nodes: \n");
			for (GeographicPoint pt : vertices.keySet())
			{
				MapNode n = vertices.get(pt);
				System.out.println(n);
			}
		}

		// For DEBUGGING.  Print the Edges in the graph
		public void printEdges()
		{
			System.out.println("******PRINTING EDGES******");
			System.out.println("There are " + getNumEdges() + " Edges:\n");
			for (MapEdge e : edges)
			{
				System.out.println(e);
			}

		}*/
	/////////////////////////////////////////////////////////////////////////////
		
	/** Find the path from start to end using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param end The end location
	 * @return The list of intersections that form the shortest path from 
	 *   start to end, inclusive.
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint end) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, end, temp);
	}
	
	/** Find the path from start to end using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param end The end location
	 * @param nodeSearched A hook for visualization.
	 * @return The list of intersections that form the shortest path from 
	 *   start to end, inclusive.
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint end, Consumer<GeographicPoint> nodeSearched)
	{
		if(start == null || end == null) 
			throw new NullPointerException("Cannot find route from or to null node");
		
		MapNode startNode = vertices.get(start);
		MapNode endNode = vertices.get(end);
		
		if(startNode == null){
			System.err.println("Start node: " + startNode + " does not exist");
			return null;
		}
		if(endNode == null){
			System.err.println("End node: " + endNode + " does not exist");
			return null;
		}		
		
		PriorityQueue<MapNode> pq = new PriorityQueue<MapNode>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		HashMap<MapNode, MapNode> parent = new HashMap<MapNode, MapNode>();
		
		// Set distances to infinity
		for(MapNode n : vertices.values()){
			n.setDistance(Double.POSITIVE_INFINITY);
		}
		// Enqueue starting node
		startNode.setDistance(0);
		pq.add(startNode);
		MapNode next = null;
		// Count nodes visited
		int intCnt = 0;
		
		while(!pq.isEmpty()){
			next = pq.remove();
			intCnt++;
			
			// Hook for visualization.
			nodeSearched.accept(next.getLocation());
			System.out.println("Dijkstra currently visiting: " + next);
			
			if(next.equals(endNode)) break;

			if(!visited.contains(next)){
				visited.add(next);
				Set<MapEdge> edges = next.getEdges();
				
				for (MapEdge edge : edges) {
					MapNode neighbor = edge.getEndNode();					

					if (!visited.contains(neighbor)) {
						double dblCurrDist = edge.getLength() + next.getDistance();
						if (dblCurrDist < neighbor.getDistance()) {
							parent.put(neighbor, next);
							neighbor.setDistance(dblCurrDist);
							pq.add(neighbor);
						}
					}
				}
			}
		}
		if (!next.equals(endNode)) {
			System.out.println("Dijkstra - No path found from " + start + " to" + end);
			return null;
		}
		List<GeographicPoint> path = reconstructPath(parent, startNode, endNode);
		System.out.println("Dijkstra - Nodes visited in search: " + intCnt);

		return path;
	}

	/** Find the path from start to end using A-Star search
	 * 
	 * @param start The starting location
	 * @param end The end location
	 * @return The list of intersections that form the shortest path from 
	 *   start to end, inclusive.
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint end) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, end, temp);
	}
	
	/** Find the path from start to end using A-Star search
	 * 
	 * @param start The starting location
	 * @param end The end location
	 * @param nodeSearched A hook for visualization.
	 * @return The list of intersections that form the shortest path from 
	 *   start to end, inclusive.
	 */
	@SuppressWarnings("unused")
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		boolean debug = false;
		// Set up
		if (start == null || goal == null)
			throw new NullPointerException("Cannot find route from or to null node");
		
		MapNode startNode = vertices.get(start);
		MapNode endNode = vertices.get(goal);
		
		if (startNode == null) {
			System.err.println("Start node " + start + " does not exist");
			return null;
		}
		if (endNode == null) {
			System.err.println("End node " + goal + " does not exist");
			return null;
		}

		HashMap<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>();
		PriorityQueue<MapNode> toExplore = new PriorityQueue<MapNode>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		
		// initialize distance for all nodes
		for (MapNode n : vertices.values()) {
			n.setDistance(Double.POSITIVE_INFINITY);
			n.setActualDistance(Double.POSITIVE_INFINITY);
		}
		
		startNode.setDistance(0);
		startNode.setActualDistance(0);

		toExplore.add(startNode);
		
		int intCnt = 0;
		MapNode next = null;
		
		while (!toExplore.isEmpty()) {
			next = toExplore.remove();
			
			nodeSearched.accept(next.getLocation());
            intCnt++;
            
			//if(debug) {
			//  System.out.println("\nA* visiting" + next+"\nActual = "+
			//							next.getActualDistance()+", Pred: "+next.getDistance());
            //}
			
			if (next.equals(endNode)) break;
			if(!visited.contains(next)) {
				visited.add(next);
				Set<MapEdge> edges = next.getEdges();
				for (MapEdge edge : edges) {
					MapNode neighbor = edge.getEndNode();
					if (!visited.contains(neighbor)) {

						double currDist = edge.getLength()+next.getActualDistance();
						// core of A* is just to add to currDist the cost of getting to
						// the destination
						double predDist = currDist+ (neighbor.getLocation()).distance(endNode.getLocation());
						if(predDist < neighbor.getDistance()){
							// debug
							if(debug) {
							  System.out.println("Adding to queue node at: "+neighbor.getLocation());
							  System.out.println("Curr dist: "+currDist+" Pred Distance: " + predDist);
							}
							parentMap.put(neighbor, next);
							neighbor.setActualDistance(currDist);
							neighbor.setDistance(predDist);
							toExplore.add(neighbor);
						}
					}
				}
			}
		}
		if (!next.equals(endNode)) {
			System.out.println("A* - No path found from " +start+ " to " + goal);
			return null;
		}
		// Reconstruct the parent path
		List<GeographicPoint> path =
				reconstructPath(parentMap, startNode, endNode);
		System.out.println("A* - Nodes visited in search: "+intCnt);
		return path;
	}

	
	
	public static void main(String[] args)
	{
		/*
		//Used for testing purposes
		
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);

		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);

		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart, testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart, testEnd);

		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);

		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart, testEnd);
		testroute2 = testMap.aStarSearch(testStart, testEnd);

		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart, testEnd);
		testroute2 = testMap.aStarSearch(testStart, testEnd);
		
		
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);

		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);	

		*/		
	}
	
}
