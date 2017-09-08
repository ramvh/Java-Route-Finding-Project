/**
 * @author Haley Mar
 * Date: 6/30/2017
 * Summary: A directed edge in a map graph from Node start to Node end
 */
package roadgraph;


class MapEdge {
	
	private String roadName;
	private String roadType;
	private MapNode start;
	private MapNode end;
	private double length;
	static final double DEFAULT_LENGTH = 0.01;
	
	/** Create a new MapEdge obj
	 * 
	 * @param roadName
	 * @param n1 The point at one end of the segment
	 * @param n2 The point at the other end of the segment
	 */
	
	// Constructor
	MapEdge(String roadName, MapNode n1, MapNode n2){
		this(roadName, "", n1, n2, DEFAULT_LENGTH);
	}

	//Constructor
	MapEdge(String roadName, String roadType, MapNode n1, MapNode n2, double length) {
		this.roadName = roadName;
		this.start = n1;
		this.end = n2;
		this.roadType = roadType;
		this.length = length;
	}

	// Used in BFS to find neighbors
	MapNode getOtherNodes(MapNode node) {
		
		if(node.equals(start))
			return end;
		else if (node.equals(end))
			return start;
		else throw new IllegalArgumentException("Looking for a point that is not in the edge.");		
	}

	// Return last node
	public MapNode getEndNode() {
		return end;
	}

	// Return edge length
	public double getLength() {
		return this.length;
	}
	
	// Return edge road type
/*	public String getRoadType(){
		return this.roadType;
	}*/
}
