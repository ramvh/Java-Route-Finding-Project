/**
 * @author Haley Mar
 * Date: 6/30/2017
 * MapNode is the super class that MapGraph and MapEdge inherit from.
 */
 
package roadgraph;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

import geography.GeographicPoint;


class MapNode implements Comparable<Object>{


	// List of edges from node
	private HashSet<MapEdge> edges;
	// Coordinates of node
	private GeographicPoint location;
	private double distance;
	private double actualDistance;
	
	MapNode(GeographicPoint location){
		this.location = location;
		this.edges = new HashSet<MapEdge>();
		this.distance = 0.0;
		this.actualDistance = 0.0;
	}

    // Comparable
	@Override
	public int compareTo(Object o) {
		MapNode m = (MapNode)o; 
		return ((Double)this.getDistance()).compareTo((Double) m.getDistance());
	}
	

	public void addEdge(MapEdge edge) {
		edges.add(edge);
	}

	public GeographicPoint getLocation() {
		return location;
	}
	
	// Returns whether 2 nodes are equal.
	// Nodes are considered equal if their locations are the same,
	// even if their street list is different.
	public boolean equals(Object o){
		if(!(o instanceof MapNode) || (o == null)){
			return false;
		}
		MapNode node = (MapNode)o;
		
		return node.location.equals(this.location);
	}

	// Find all neighbors of vertex
	Set<MapNode> getNeighbor() {

		Set<MapNode> neighbors = new HashSet<MapNode>();
		for(MapEdge edge : edges) 
			neighbors.add(edge.getOtherNodes(this));
		return neighbors;
	}
	
	// Used in dijkstra algorithm to set all distances to infinity
	public void setDistance(double dblDistance){
		this.distance = dblDistance;
	}

	// Return edges originating from node
	public Set<MapEdge> getEdges() {
		return edges;
	}

	// Return node distance
	public double getDistance() {
		return this.distance;
	}

	// Set actual distance from vertex to n
	public void setActualDistance(double actualDistance) {
		this.actualDistance = actualDistance;
	}

	// Return actual distance from vertex to n
	public double getActualDistance() {		
		return this.actualDistance;
	}
}
