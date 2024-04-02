package uuspaceagent.exploration;

import eu.iv4xr.framework.spatial.Vec3;

public interface Explorable<NodeId> {
    /**
     * An iterable that can be used by pathfinders to explore a node's connections.
     *
     * @param id the index of the vertex to inspect.
     * @return an iterable of the connected neighbours.
     */
    public Iterable<NodeId> neighbours(NodeId id);

    /**
     * An iterable that can be used by pathfinders to explore a node's connections.
     * Different from normal {@link #neighbours()}, it allows unexplored nodes as neighbours
     *
     * @param id the index of the vertex to inspect.
     * @return an iterable of the connected neighbours.
     */
    public Iterable<NodeId> neighbours_explore(NodeId id);

    /**
     * The estimated distance between two arbitairy vertices.
     *
     * @param from: the index of the vertex to travel from.
     * @param to: the index of the vertex to travel to.
     * @return the estimated distance.
     */
    public float heuristic(NodeId from, NodeId to);

    /**
     * The distance between two connected vertices.
     *
     * @param from: the index of the vertex to travel from.
     * @param to: the index of the vertex to travel to.
     * @return the measured distance. NaN if nodes are not connected.
     */
    public float distance(NodeId from, NodeId to);

    /**
     * The exploration status of a vertex.
     *
     * @param node: the index of the vertex to check.
     * @return whether the node is explored or not
     */
    public boolean isUnknown(NodeId node);

    /**
     * Updates all unknown nodes inside the observation radius to have the OPEN label.
     * @param pos: the position of the agent
     * @param observation_radius: the observation radius of the agent
     */
    public void updateUnknown(Vec3 pos, float observation_radius);
}
