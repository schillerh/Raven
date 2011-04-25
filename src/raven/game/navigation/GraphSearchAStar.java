package raven.game.navigation;

import java.util.ArrayList;
import java.util.List;
import java.util.TreeMap;
import java.util.Vector;
import java.util.Map.Entry;

import raven.game.RavenBot;
import raven.game.triggers.Trigger;
import raven.math.graph.*;

public class GraphSearchAStar<GraphType extends SparseGraph<? extends GraphNode, ? extends GraphEdge>, Heuristic> extends GraphSearchTimeSliced<EdgeType extends GraphEdge>> {
	
	//private:
		  
		  //create typedefs for the node and edge types used by the graph
		 // typedef typename graph_type::EdgeType Edge;
		  //typedef typename graph_type::NodeType Node;
	private SparseGraph<? extends GraphNode, ? extends GraphEdge> graph;
	/** this vector contains the edges that comprise the shortest path tree -
	 * a directed subtree of the graph that encapsulates the best paths from
	 * every node on the SPT to the source node */
	private List<GraphEdge> shortestPathTree;
	
	/** this is indexed into by node index and holds the total cost of the
	 * best path found so far to the given node. For example,
	 * m_CostToNode.get(5) will hold the total cost of all the edges that
	 * comprise the best path to node 5, found so far in the search (if node 5
	 * is present and has been visited) */
	private List<Double> costToNode;
	
	/** this is an indexed (by node) vector of 'parent' edges leading to nodes
	 * connected to the SPT but that have not been added to the SPT yet. This
	 * is a little like the stack or queue used in BST and DST searches. */
	private List<GraphEdge> searchFrontier;
	
	int source;
	int target;
	
		private SparseGraph<NavGraphNode<Trigger<RavenBot>>, NavGraphEdge>  edge =new SparseGraph<NavGraphNode<Trigger<RavenBot>>, NavGraphEdge>();
		private NavGraphNode<GraphNode> node =  new NavGraphNode<GraphNode>();
		//private NavGraphNode graph = new NavGraphNode();


		  //indexed into my node. Contains the 'real' accumulative cost to that node
		  private List<Double> gCosts; 

		  //indexed into by node. Contains the cost from adding m_GCosts[n] to
		  //the heuristic cost from n to the target node. This is the vector the
		  //iPQ indexes into.
		  private List<Double> fCosts;

		  //private Vector<GraphEdge> shortestPathTree = new Vector<GraphEdge>();
		  //private Vector<GraphEdge> searchFrontier = new Vector<GraphEdge>();

	//	  private int source;
		//  private int target;

		  //create an indexed priority queue of nodes. The nodes with the
		  //lowest overall F cost (G+H) are positioned at the front.
		//  IndexedPriorityQLow<Double>    PQ;
		  public double hCalculate(SparseGraph<? extends GraphNode, ? extends GraphEdge> graph2, int nd1, int nd2)
		  {
		    return graph2.getNode(nd1).pos().distance(graph2.getNode(nd2).pos());
		  }
		
		  
		  private void search() {

				/* create an indexed priority queue that sorts smallest to largest
				 * (front to back).Note that the maximum number of elements the iPQ
				 * may contain is N. This is because no node can be represented on
				 * the queue more than once. */
				TreeMap<Double,Integer> queue = new TreeMap<Double,Integer>();
				
				// put the source node is not empty
				queue.put(0.0, source);
				
				while (!queue.isEmpty()) {
					// get lowest cost node from the queue. Don't forget, the return
					// value is a *node index*, not the node itself. This node is the
					// node not already on the SPT that is the closest to the source
					// node
					Entry<Double,Integer> queueEntry = queue.firstEntry();
					queue.remove(queueEntry.getKey());
					int nextClosestNode = queueEntry.getValue();
					
					// move this edge from the frontier to the shortest path tree
					shortestPathTree.add(nextClosestNode, searchFrontier.get(nextClosestNode));

					// if the target has been found exit
					if (nextClosestNode == target) {
						return;
					}
					
					// for each edge connected to the next closest node
					for (GraphEdge edge : graph.getEdges(nextClosestNode)) {
						double HCost = hCalculate(graph, target,edge.to());
						double GCost = gCosts.get(nextClosestNode)+ edge.cost();
						
						// if this edge has never been on the frontier make a note of
						// the cost to get to the node it points to, then add the edge
						// to the frontier and the destination node to the PQ.
						if (searchFrontier.get(edge.to()) == null) {
							fCosts.set(edge.to(),GCost + HCost);
							gCosts.set(edge.to(),GCost);
							searchFrontier.set(edge.to(), edge);
							queue.put(GCost, edge.to());
						}
						// else test to see if the cost to reach the destination node
						// via the current node is cheaper than the cheapest cost
						// found so far. If this path is cheaper, we assign the new
						// cost to the destination node, update its entry in the PQ to
						// reflect the change and add the edge to the frontier
						else if ( (GCost < gCosts.get(edge.to())) && shortestPathTree.get(edge.to()) == null) {
							costToNode.set(edge.to(), GCost);
							
							fCosts.set(edge.to(), GCost + HCost);
							gCosts.set(edge.to(), GCost);
							queue.remove(edge.to());
							queue.put(GCost, edge.to());
							
							searchFrontier.set(edge.to(), edge);
						}

					}
				}
			}

			public GraphSearchAStar(SparseGraph<? extends GraphNode, ? extends GraphEdge> graph, int source, int target) {
				this.graph = graph;
				this.source = source;
				this.target = target;
				this.shortestPathTree = new ArrayList<GraphEdge>(graph.numNodes());
				this.searchFrontier = new ArrayList<GraphEdge>(graph.numNodes());
				this.costToNode = new ArrayList<Double>(graph.numNodes());
				
				// The algorithm requires the array's be filled with 0s
				for (int i = 0; i < graph.numNodes(); i++) {
					searchFrontier.add(null);
					costToNode.add(0.0);
				}
		public GraphSearchAStar(GraphEdge g, int source,int target) {  
		     PQ =new IndexedPriorityQLow<double>(FCosts, edge.numNodes());
		     g= new GraphEdge();
		    //put the source node on the queue
		    PQ.insert(source);
		  }

		   
		  //When called, this method pops the next node off the PQ and examines all
		  //its edges. The method returns an enumerated value (target_found,
		  //target_not_found, search_incomplete) indicating the status of the search
		  public int cycleOnce(){
			//if the PQ is empty the target has not been found
			  if (PQ.isEmpty())
			  {
			    return target_not_found;
			  }

			  //get lowest cost node from the queue
			  int NextClosestNode = PQ.Pop();

			  //put the node on the SPT
			  shortestPathTree[NextClosestNode] = searchFrontier[NextClosestNode];

			  //if the target has been found exit
			  if (NextClosestNode == target)
			  {
			    return target_found;
			  }

		  }

		  //returns the vector of edges that the algorithm has examined
		  public List<GraphEdge> getSPT(){return shortestPathTree;}

		  //returns a vector of node indexes that comprise the shortest path
		  //from the source to the target
			public List<Integer> getPathToTarget() {
				List<Integer> path = new ArrayList<Integer>();
				
				// just return an empty path if no target or no path found
				if (target < 0) {
					return path;
				}
				
				int node = target;
				
				path.add(node);
				
				while (node != source && shortestPathTree.get(node) != null) {
					node = shortestPathTree.get(node).from();
					path.add(node);
				}
				
				return path;
			}

		  //returns the path as a list of PathEdges
		  public List<PathEdge> getPathAsPathEdges(){
			  
		  }

		  //returns the total cost to the target
		  public Double getCostToTarget(){return GCosts[target];}

		//-----------------------------------------------------------------------------
		  
		  //now to test all the edges attached to this node
		  graph_type::ConstEdgeIterator ConstEdgeItr(graph, NextClosestNode);
		  for (E=ConstEdgeItr.begin();
		      !ConstEdgeItr.end();
		       pE=ConstEdgeItr.next())
		  {
		    //calculate the heuristic cost from this node to the target (H)                       
		    double HCost = getCostToTarget()(m_Graph, m_iTarget, E.To()); 

		    //calculate the 'real' cost to this node from the source (G)
		    double GCost = GCosts[NextClosestNode] + (from, target).cost();

		    //if the node has not been added to the frontier, add it and update
		    //the G and F costs
		    if (m_SearchFrontier[pE->To()] == NULL)
		    {
		      m_FCosts[pE->To()] = GCost + HCost;
		      m_GCosts[pE->To()] = GCost;

		      m_pPQ->insert(pE->To());

		      m_SearchFrontier[pE->To()] = pE;
		    }

		    //if this node is already on the frontier but the cost to get here
		    //is cheaper than has been found previously, update the node
		    //costs and frontier accordingly.
		    else if ((GCost < m_GCosts[pE->To()]) && (m_ShortestPathTree[pE->To()]==NULL))
		    {
		      m_FCosts[pE->To()] = GCost + HCost;
		      m_GCosts[pE->To()] = GCost;

		      m_pPQ->ChangePriority(pE->To());

		      m_SearchFrontier[pE->To()] = pE;
		    }
		  }
		  
		  //there are still nodes to explore
		  return search_incomplete;
		}

		//-----------------------------------------------------------------------------
		template <class graph_type, class heuristic>
		std::list<int> 
		Graph_SearchAStar_TS<graph_type, heuristic>::GetPathToTarget()const
		{
		  std::list<int> path;

		  //just return an empty path if no target or no path found
		  if (m_iTarget < 0)  return path;    

		  int nd = m_iTarget;

		  path.push_back(nd);
		    
		  while ((nd != m_iSource) && (m_ShortestPathTree[nd] != 0))
		  {
		    nd = m_ShortestPathTree[nd]->From();

		    path.push_front(nd);
		  }

		  return path;
		} 


		//-------------------------- GetPathAsPathEdges -------------------------------
		//
		//  returns the path as a list of PathEdges
		//-----------------------------------------------------------------------------
		template <class graph_type, class heuristic>
		std::list<PathEdge> 
		Graph_SearchAStar_TS<graph_type, heuristic>::GetPathAsPathEdges()const
		{
		  std::list<PathEdge> path;

		  //just return an empty path if no target or no path found
		  if (m_iTarget < 0)  return path;    

		  int nd = m_iTarget;
		    
		  while ((nd != m_iSource) && (m_ShortestPathTree[nd] != 0))
		  {
		    path.push_front(PathEdge(m_Graph.GetNode(m_ShortestPathTree[nd]->From()).Pos(),
		                             m_Graph.GetNode(m_ShortestPathTree[nd]->To()).Pos(),
		                             m_ShortestPathTree[nd]->Flags(),
		                             m_ShortestPathTree[nd]->IDofIntersectingEntity()));

		    nd = m_ShortestPathTree[nd]->From();
		  }

		  return path;
		}
