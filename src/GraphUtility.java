package roadgraph;

public class GraphUtility {

	public static Integer generateHashMapKeyForEdge(MapNode start, MapNode end){
		return start.hashCode() + end.hashCode();
	}
}
