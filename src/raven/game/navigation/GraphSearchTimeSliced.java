package raven.game.navigation;

import java.util.List;
import java.util.Vector;

import raven.math.graph.GraphEdge;
import raven.math.graph.GraphEdgeFactory;
import raven.math.graph.GraphNode;
import raven.math.graph.SparseGraph;
import raven.utils.StreamUtils;
import raven.math.graph.GraphSearchDijkstra;

public class GraphSearchTimeSliced<NodeType extends NavGraphNode<T>,EdgeType extends NavGraphEdge, T>  {

	public enum SearchType{AStar, Dijkstra};
	public enum SearchStatus{
		target_found(1),
		target_not_found(2), 
		search_incomplete(4);

		private int value;
		private SearchStatus(int i) {value = i;}
		public int getValue() {return value;}
	};

	private Vector<Double> costToThisNode; 

	private  Vector<EdgeType>  shortestPathTree;
	private Vector<EdgeType>  searchFrontier;

	private int source;
	private int target;
	private SparseGraph<NodeType, EdgeType> graph=new SparseGraph<NodeType, EdgeType>();
	public List<PathEdge> path;
	private SearchType searchType;
	public GraphSearchTimeSliced(){

	}
	public SearchType GetType(){
		return searchType;
	}
}