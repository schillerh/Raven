package raven.game;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import com.thoughtworks.xstream.annotations.XStreamAlias;

import raven.game.messaging.RavenMessage;
import raven.game.navigation.NavGraphEdge;
import raven.game.navigation.NavGraphNode;
import raven.game.triggers.Trigger;
import raven.game.triggers.TriggerHealthGiver;
import raven.game.triggers.TriggerOnButtonSendMsg;
import raven.game.triggers.TriggerSoundNotify;
import raven.game.triggers.TriggerSystem;
import raven.game.triggers.TriggerWeaponGiver;
import raven.math.CellSpacePartition;
import raven.math.Vector2D;
import raven.math.Wall2D;
import raven.math.graph.GraphNode;
import raven.math.graph.SparseGraph;
import raven.script.RavenScript;
import raven.ui.GameCanvas;
import raven.utils.Pair;

@XStreamAlias("RavenMap")
public class RavenMap {
	
	/** the walls that comprise the current map's architecture. */
	private ArrayList<Wall2D> walls;
	
	/** trigger are objects that define a region of space. When a raven bot
	 * enters that area, it 'triggers' an event. That event may be anything
	 * from increasing a bot's health to opening a door or requesting a lift.
	 */
	private TriggerSystem<Trigger<RavenBot>> triggerSystem;
	
	/** this holds a number of spawn positions. When a bot is instantiated it
	 * will appear at a randomly selected point chosen from this vector */
	private ArrayList<Vector2D> spawnPoints;
	
	/** a map may contain a number of sliding doors. */
	private ArrayList<RavenDoor> doors;
	
	/** this map's accompanying navigation graph */
	private SparseGraph<NavGraphNode<Trigger<RavenBot>>, NavGraphEdge> navGraph;
	
	/** the graph nodes will be partitioned enabling fast lookup */
	transient private CellSpacePartition<NavGraphNode<Trigger<RavenBot>>> spacePartition;
	
	/** the size of the search radius the cellspace partition uses when
	 * looking for neighbors */
	transient double cellSpaceNeighborhoodRange;
	
	int sizeX = 0;
	int sizeY = 0;
	
	/* this will hold a pre-calculated lookup table of the cost to travel
	 * from */
	transient private Map<Pair<Integer, Integer>, Double> pathCosts;

	/** the path this file was loaded from. null if unsaved. */
	transient private String path;

	/** the name of this map, as displayed to the user */
	private String name;
		
	private void partitionNavGraph() {
		spacePartition = new CellSpacePartition<NavGraphNode<Trigger<RavenBot>>>(sizeX, sizeY,
				RavenScript.getInt("NumCellsX"), RavenScript.getInt("NumCellsY"),
				navGraph.numNodes());
		
		// add the graph nodes to the space partition
		for (NavGraphNode<Trigger<RavenBot>> node : navGraph) {
			spacePartition.addEntity(node);
		}
	}
	
	public void addSpawnPoint(Vector2D point) {
		spawnPoints.add(point);
	}
	
	public void addSpawnPoint(double x, double y) {
		spawnPoints.add(new Vector2D(x, y));
	}
	
	public void addHealthGiver(Vector2D position, int radius, int healthPlus, int respawnDelay) {
		TriggerHealthGiver healthGiver = new TriggerHealthGiver(position, radius, healthPlus, respawnDelay);
		
		triggerSystem.register(healthGiver);
		
		// Let the corresponding NavGraphNode point to this object
		NavGraphNode<Trigger<RavenBot>> node = new NavGraphNode<Trigger<RavenBot>>(navGraph.getNextFreeNodeIndex(), position);
		node.setExtraInfo(healthGiver);
		navGraph.addNode(node);
		
		healthGiver.setGraphNodeIndex(node.index());
		
		// register the entity
		EntityManager.registerEntity(healthGiver);
	}
	
	public void addWeaponGiver(RavenObject typeOfWeapon, Vector2D position, int radius, int respawnDelay) {
		TriggerWeaponGiver weaponGiver = new TriggerWeaponGiver(position, radius, respawnDelay);
		weaponGiver.setEntityType(typeOfWeapon);
		
		// add it to the appropriate vectors
		triggerSystem.register(weaponGiver);
		
		// Create a corresponding navGraph node
		NavGraphNode<Trigger<RavenBot>> node = new NavGraphNode<Trigger<RavenBot>>(navGraph.getNextFreeNodeIndex(), position);
		node.setExtraInfo(weaponGiver);
		navGraph.addNode(node);
		
		weaponGiver.setGraphNodeIndex(node.index());

	}
	
	public void addDoor(int id, Vector2D pos1, Vector2D pos2, int timeout) {
		RavenDoor door = new RavenDoor(id, pos1, pos2, timeout);
		
		doors.add(door);
		Trigger t = addDoorTrigger(pos1, pos2, RavenMessage.MSG_OPEN_SESAME, id);
		door.addSwitch(t.ID());
		// register the entity
		EntityManager.registerEntity(door);
	}
	
	public Trigger<RavenBot> addDoorTrigger(Vector2D topLeft, Vector2D bottomRight, RavenMessage msg, int receiver) {
		TriggerOnButtonSendMsg<RavenBot> trigger = new TriggerOnButtonSendMsg<RavenBot>(topLeft, bottomRight, msg, receiver);
		triggerSystem.register(trigger);
		// register the entity
		EntityManager.registerEntity(trigger);
		return trigger;
	}
	
	public void clear() {
		// delete the triggers
		triggerSystem.clear();
		
		// delete the doors
		doors.clear();
		
		walls.clear();
		
		spawnPoints.clear();
		
		// delete the navgraph
		navGraph = null;
		
		// delete the partition info
		spacePartition = null;
	}
	
	public RavenMap() {
		triggerSystem = new TriggerSystem<Trigger<RavenBot>>();
		doors = new ArrayList<RavenDoor>();
		walls = new ArrayList<Wall2D>();
		spawnPoints = new ArrayList<Vector2D>();
		sizeX = sizeY = 500;
		navGraph = new SparseGraph<NavGraphNode<Trigger<RavenBot>>, NavGraphEdge>();
		spacePartition = new CellSpacePartition<NavGraphNode<Trigger<RavenBot>>>(0.0, 0.0, 0, 0, 0);
		cellSpaceNeighborhoodRange = 0.0;
	}
	
	private Object readResolve() {
		cellSpaceNeighborhoodRange = navGraph.calculateAverageGraphEdgeLength() + 1;
		
		partitionNavGraph();
		
		pathCosts = navGraph.createAllPairsCostsTable();
		
		return this;
	}
	

	
	/**
	 * adds a wall and returns a pointer to that wall. (this method can be
	 * used by objects such as doors to add walls to the environment)
	 * @param from wall's starting point
	 * @param to wall's ending point
	 * @return the new wall created
	 */
	public Wall2D addWall(Vector2D from, Vector2D to) {
		Wall2D wall = new Wall2D(from, to);
		walls.add(wall);
		return wall;
	}
	
	public void addSoundTrigger(RavenBot soundSource, double range) {
		triggerSystem.register(new TriggerSoundNotify(soundSource, range));
	}
	
	public double calculateCostToTravelBetweenNodes(int node1, int node2) {
		if (node1 < 0 || node2 < 0 || node1 >= navGraph.numNodes() || node2 >= navGraph.numNodes())
			throw new IndexOutOfBoundsException("Invalid node index: " + node1 + " to " + node2);
		
		return pathCosts.get(new Pair<Integer,Integer>(node1, node2));
	}
	
	/** returns the position of a graph node selected at random */
	public Vector2D getRandomNodeLocation() {
		int randIndex = (int)(Math.random() * navGraph.numActiveNodes());
		
		GraphNode node = null;
		for (int i = 0; i < navGraph.numNodes(); i++) {
			node = navGraph.getNode(i);
			if (node.index() != GraphNode.INVALID_NODE_INDEX) {
				randIndex--;
			}
			if (randIndex < 0) {
				break;
			}
		}
		
		return node.pos();
		
	}
	
	public void updateTriggerSystem(double delta, List<RavenBot> bots) {
		triggerSystem.update(delta, bots);
	}
	
	// Accessors
	
	public List<Trigger<RavenBot>> getTriggers() {
		return triggerSystem.getTriggers();
	}
	
	public List<Wall2D> getWalls() {
		return walls;
	}
	
	public SparseGraph<NavGraphNode<Trigger<RavenBot>>, NavGraphEdge> getNavGraph() {
		return navGraph;
	}
	
	public List<RavenDoor> getDoors() {
		return doors;
	}
	
	public List<Vector2D> getSpawnPoints() {
		return spawnPoints;
	}
	
	public CellSpacePartition<NavGraphNode<Trigger<RavenBot>>> getCellSpace() {
		return spacePartition;
	}
	
	public Vector2D getRandomSpawnPoint() {
		return spawnPoints.get((int)(Math.random() * spawnPoints.size()));
	}
	
	public int getSizeX() { 
		return sizeX;
	}
	
	public int getSizeY() {
		return sizeY;
	}
	
	public int getMaxDimension() { 
		return Math.max(sizeX, sizeY);
	}
	
	public double getCellSpaceNeighborhoodRange() {
		return cellSpaceNeighborhoodRange;
	}

	public void render() {
		//draw basic background
		GameCanvas.whiteBrush();
		int offset = 20;
		GameCanvas.filledRect(0, offset, sizeX, sizeY+offset);
		
		// render the navgraph
		if (RavenUserOptions.showGraph) {
			navGraph.render(RavenUserOptions.showNodeIndices);
		}
		
		// render any doors
		for (RavenDoor door : doors) {
			door.render();
		}
		
		// render all triggers
		triggerSystem.render();
		
		// render all walls
		for (Wall2D wall : walls) {
			GameCanvas.thickBlackPen();
			wall.render();
		}
		
		// render spawn points
		for (Vector2D point : spawnPoints) {
			GameCanvas.greyBrush();
			GameCanvas.greyPen();
			GameCanvas.filledCircle(point, 7);
		}
		
	}
	
	@Override
	public boolean equals(Object other){
		if (this == other) return true;
		
		if(! (other instanceof RavenMap)) return false;
		
		RavenMap toCompare = (RavenMap) other;
		
		if( (walls.equals(toCompare.walls)) && (triggerSystem.equals(toCompare.triggerSystem)) 
				&& (spawnPoints.equals(toCompare.spawnPoints)) && (doors.equals(toCompare.doors)) && 
				(navGraph.equals(toCompare.navGraph)) && (spacePartition.equals(toCompare.spacePartition)) &&
				(cellSpaceNeighborhoodRange == toCompare.cellSpaceNeighborhoodRange) && 
				(sizeX == toCompare.sizeX) && (sizeY == toCompare.sizeY)) {
			return true;
		}
		else return false;		
	}
	
	@Override
	public int hashCode() {
		int result = 0;
		result += walls.hashCode();
		result += triggerSystem.hashCode();
		result += spawnPoints.hashCode();
		result += doors.hashCode();
		result += navGraph.hashCode();
		result += spacePartition.hashCode();
		result += cellSpaceNeighborhoodRange;
		result += sizeX;
		result += sizeY;
		
		// modded by a prime.  I chose 101 because I don;t think we'll have a lot more that that
		// many maps total.
		return result % 101;
	}

	public String getPath() { return path; }
	public void setPath(String path) { this.path = path; }
	public String getName() { return name; }
	public void setName(String name) { this.name = name; }

	public void setSize(int width, int height) {
		this.sizeX = width;
		this.sizeY = height;		
	}
}
