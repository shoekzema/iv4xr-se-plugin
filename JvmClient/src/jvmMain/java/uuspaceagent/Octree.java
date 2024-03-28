package uuspaceagent;

import eu.iv4xr.framework.mainConcepts.WorldEntity;
import eu.iv4xr.framework.spatial.Vec3;
import uuspaceagent.exploration.Explorable;

import java.io.PrintWriter;
import java.util.*;
import java.util.List;

public class Octree implements Explorable<Octree> {
    public Boundary boundary;
    public ArrayList<Octree> children;
    public Octree parent;
    byte label; // empty = 0, full = 1, mixed = 2, mixed-unknown = 3
    byte code; // 0-7, Morton code (Z) order
    public static float MIN_NODE_SIZE = 1;


    public Octree(Boundary boundary, Octree parent, byte code, byte label) {
        this.boundary = boundary;
        this.children = new ArrayList<>(0);
        this.parent = parent;
        this.code = code;
        this.label = label;
    }


    /**
     *  Initializes the octrees root boundary based on the first observed position and the observation radius.
     *  Call in UUSeAgentstate3D on first observation.
     */
    public void initializeGrid(Vec3 pos, float observation_radius) {
        boundary = new Boundary(Vec3.sub(pos, new Vec3(observation_radius)), 2 * observation_radius);
//        boundary.lowerBounds = Vec3.sub(pos, new Vec3(observation_radius * 0.5f));
//        boundary.upperBounds = Vec3.add(pos, new Vec3(observation_radius * 0.5f));
    }


    /**
     * Calculate the octree leaf-node that contains a given position.
     */
    public Octree gridProjectedLocation(Vec3 pos) {
        if (children.isEmpty())
            return this;

        if (pos.x <= boundary.center().x) {
            if (pos.y <= boundary.center().y) {
                if (pos.z <= boundary.center().z)
                    return children.get(0).gridProjectedLocation(pos);
                // else pos.z > center.z
                return children.get(4).gridProjectedLocation(pos);
            }
            // else pos.y > center.y
            if (pos.z <= boundary.center().z)
                return children.get(2).gridProjectedLocation(pos);
            return children.get(6).gridProjectedLocation(pos);
        }
        // else pos.x > center.x
        if (pos.y <= boundary.center().y) {
            if (pos.z <= boundary.center().z)
                return children.get(1).gridProjectedLocation(pos);
            // else pos.z > center.z
            return children.get(5).gridProjectedLocation(pos);
        }
        // else pos.y > center.y
        if (pos.z <= boundary.center().z)
            return children.get(3).gridProjectedLocation(pos);
        // else pos.z > center.z
        return children.get(7).gridProjectedLocation(pos);
    }


    /**
     * Return the actual location of the center of an octree node; the location is expressed
     * as a 3D position in the space.
     */
    public Vec3 getCubeCenterLocation(Octree node) { return node.boundary.center(); }


    public void addObstacle(WorldEntity block) {
        // If the node is already full, do nothing
        if (this.label == Label.BLOCKED)
            return;

        var blockBB = new Boundary(Vec3.sub(block.position, new Vec3(1.25f)), 2.5f);

        // If the node is entirely wall
        if (blockBB.contains(this.boundary)) {
            this.label = Label.BLOCKED;
            if (!children.isEmpty()) {
                children = new ArrayList<>(0);
            }
            return;
        }
        // Otherwise, it is partially block/unknown

        // If we cannot subdivide further, force a full label
        if (boundary.size() < MIN_NODE_SIZE) {
            this.label = Label.BLOCKED;
            return;
        }
        // If not already subdivided, subdivide
        if (this.label != Label.MIXED) {
            this.subdivide(this.label);
            this.label = Label.MIXED;
        }

        // If this block intersects
        if (blockBB.intersects(this.boundary)) {
            children.forEach(node ->
                    node.addObstacle(block));
        }
        // Check if every child-node is full, then become blocking and throw them away.
        if (children.stream().noneMatch(node -> node.label != Label.BLOCKED)) {
            this.label = Label.BLOCKED;
            children = new ArrayList<>(0);
        }
    }


    public void removeObstacle(WorldEntity block) {
        // If the node is already empty, do nothing
        if (this.label == Label.OPEN)
            return;

        var blockBB = new Boundary(Vec3.sub(block.position, new Vec3(1.25f)), 2.5f);

        // If the node is entirely inside the removed block
        if (blockBB.contains(this.boundary)) {
            this.label = Label.OPEN;
            if (!children.isEmpty()) {
                children = new ArrayList<>(0);
            }
            return;
        }
        // Otherwise, it is partially inside the removed block

        // If we cannot subdivide further, force an open label
        if (boundary.size() < MIN_NODE_SIZE) {
            this.label = Label.OPEN;
            return;
        }
        // If not already subdivided, subdivide
        if (this.label != Label.MIXED) {
            this.subdivide(this.label);
            this.label = Label.MIXED;
        }

        if (blockBB.intersects(this.boundary)) {
            children.forEach(node -> node.removeObstacle(block));
        }
        // Check if every child-node is full, then become blocking and throw them away.
        if (children.stream().noneMatch(node -> node.label != Label.BLOCKED)) {
            this.label = Label.BLOCKED;
            children = new ArrayList<>(0);
        }
    }


    /**
     *  Updates the octree using a list of obstacles (blocks) and a viewing range. (slow)
     */
    public void update(List<WorldEntity> blocks, Boundary range) {
        // If the quad is fully outside viewing distance
        if (!range.intersects(this.boundary)) {
            return;
        }
        // If the quad has children
        if (!this.children.isEmpty()) {
            children.forEach(node ->
                    node.update(blocks, range));

            // If every child-node is full, become blocking and throw them away.
            if (children.stream().noneMatch(node -> node.label != Label.BLOCKED)) {
                this.label = Label.BLOCKED;
                children = new ArrayList<>(0);
            }
            // If every child-node is empty, become empty and throw them away.
            else if (children.stream().noneMatch(q -> q.label != Label.OPEN)) {
                this.label = Label.OPEN;
                children = new ArrayList<>(0);
            }
            return;
        }
        // Else
        // If the quad is entirely wall
        if (blocks.stream().anyMatch(block -> new Boundary(Vec3.sub(block.position, new Vec3(1.25f)), 2.5f).contains(this.boundary))) {
            this.label = Label.BLOCKED;
            if (!children.isEmpty()) {
                children = new ArrayList<>(0);
            }
            return;
        }
        // If the quad is fully empty
        if (blocks.stream().noneMatch(block -> new Boundary(Vec3.sub(block.position, new Vec3(1.25f)), 2.5f).intersects(this.boundary))) {
            this.label = Label.OPEN;
            if (!children.isEmpty()) {
                children = new ArrayList<>(0);
            }
            return;
        }
        // Otherwise, it is partially wall/unknown

        // If we cannot subdivide further, force a full label
        if (boundary.size() < MIN_NODE_SIZE) {
            this.label = Label.BLOCKED;
            return;
        }
        // If not already subdivided, subdivide
        if (this.label != Label.MIXED) {
            this.subdivide(this.label);
            this.label = Label.MIXED;
        }
        for (var block : blocks) {
            // If this wall intersects
            if (new Boundary(Vec3.sub(block.position, new Vec3(1.25f)), 2.5f).intersects(this.boundary)) {
                children.forEach(node ->
                        node.update(blocks, range));
            }
        }
        // Check if every child-node is full, then become blocking and throw them away.
        if (children.stream().noneMatch(node -> node.label != Label.BLOCKED)) {
            this.label = Label.BLOCKED;
            children = new ArrayList<>(0);
        }
    }


    public void subdivide(byte label) {
        children = new ArrayList<>(8);
        float half = boundary.size() * 0.5f;
        children.add(0, new Octree(new Boundary(boundary.position, half),
                this, (byte) 1, label));
        children.add(1, new Octree(new Boundary(Vec3.add(boundary.position, new Vec3(half, 0, 0)), half),
                this, (byte) 2, label));
        children.add(2, new Octree(new Boundary(Vec3.add(boundary.position, new Vec3(0, half, 0)), half),
                this, (byte) 3, label));
        children.add(3, new Octree(new Boundary(Vec3.add(boundary.position, new Vec3(half, half, 0)), half),
                this, (byte) 4, label));

        children.add(4, new Octree(new Boundary(Vec3.add(boundary.position, new Vec3(0, 0, half)), half),
                this, (byte) 5, label));
        children.add(5, new Octree(new Boundary(Vec3.add(boundary.position, new Vec3(half, 0, half)), half),
                this, (byte) 6, label));
        children.add(6, new Octree(new Boundary(Vec3.add(boundary.position, new Vec3(0, half, half)), half),
                this, (byte) 7, label));
        children.add(7, new Octree(new Boundary(Vec3.add(boundary.position, new Vec3(half, half, half)), half),
                this, (byte) 8, label));
    }


    /**
     *  Just like "subdivide()", but takes the old rootnode as input to make it a childnode
     * @param child The old rootnode
     * @param code A code [1-8] denoting the position of the old rootnode
     */
    public void subdivideExpand(Octree child, byte code) {
        children = new ArrayList<>(8);

        child.parent = this;
        child.code = code;
        float half = boundary.size() * 0.5f;

        for (byte i = 1; i <= 8; i++) {
            if (code == i) {
                children.add(child);
                continue;
            }
            Boundary bb;
            switch (i) {
                case 1  -> bb = new Boundary(         boundary.position,                              half);
                case 2  -> bb = new Boundary(Vec3.add(boundary.position, new Vec3(half, 0, 0)), half);
                case 3  -> bb = new Boundary(Vec3.add(boundary.position, new Vec3(0, half, 0)), half);
                case 4  -> bb = new Boundary(Vec3.add(boundary.position, new Vec3(half, half, 0)), half);
                case 5  -> bb = new Boundary(Vec3.add(boundary.position, new Vec3(0, 0, half)), half);
                case 6  -> bb = new Boundary(Vec3.add(boundary.position, new Vec3(half, 0, half)), half);
                case 7  -> bb = new Boundary(Vec3.add(boundary.position, new Vec3(0, half, half)), half);
                default -> bb = new Boundary(Vec3.add(boundary.position, new Vec3(half, half, half)), half);
            }
            children.add(new Octree(bb, this, i, Label.UNKNOWN));
        }
    }

    /**
     *  Check if the octree node fully contains the viewing range. If not, expand it outwards.
     * @return The new root octree node
     */
    public Octree checkAndExpand(Boundary range) {
        // If the rootNode of the Octree fully contain the viewing range, return nothing
        if (this.boundary.contains(range))
            return null;

        // Otherwise, find the direction to expand in
        int oldcode;
        if (range.upperBounds().x >= this.boundary.upperBounds().x) { // expand to the right
            if (range.upperBounds().y >= this.boundary.upperBounds().y) { // expand towards up-right
                if (range.upperBounds().z >= this.boundary.upperBounds().z) // expand towards up-right-back
                    oldcode = 1;
                else // expand towards up-right-front
                    oldcode = 5;

            }
            else { // expand towards down-right
                if (range.upperBounds().z >= this.boundary.upperBounds().z) // expand towards down-right-back
                    oldcode = 3;
                else // expand towards down-right-front
                    oldcode = 7;
            }
        }
        else { // expand to the left
            if (range.upperBounds().y >= this.boundary.upperBounds().y) { // expand towards up-left
                if (range.upperBounds().z >= this.boundary.upperBounds().z) // expand towards up-left-back
                    oldcode = 4;
                else // expand towards up-left-front
                    oldcode = 8;
            } else { // expand towards down-left
                if (range.upperBounds().z >= this.boundary.upperBounds().z) // expand towards down-left-back
                    oldcode = 2;
                else // expand towards down-left-front
                    oldcode = 6;
            }
        }

        Octree newRoot;
        float newSize = boundary.size() * 2;
        float oldSize = boundary.size();
        switch (oldcode) {
            case 1 -> newRoot = new Octree(
                    new Boundary(boundary.position, newSize),
                    null, (byte) 0, Label.MIXED);
            case 2 -> newRoot = new Octree(
                    new Boundary(Vec3.sub(boundary.position, new Vec3(oldSize, 0, 0)), newSize),
                    null, (byte) 0, Label.MIXED);
            case 3 -> newRoot = new Octree(
                    new Boundary(Vec3.sub(boundary.position, new Vec3(0, oldSize, 0)), newSize),
                    null, (byte) 0, Label.MIXED);
            case 4 -> newRoot = new Octree(
                    new Boundary(Vec3.sub(boundary.position, new Vec3(oldSize, oldSize, 0)), newSize),
                    null, (byte) 0, Label.MIXED);

            case 5 -> newRoot = new Octree(
                    new Boundary(Vec3.sub(boundary.position, new Vec3(0, 0, oldSize)), newSize),
                    null, (byte) 0, Label.MIXED);
            case 6 -> newRoot = new Octree(
                    new Boundary(Vec3.sub(boundary.position, new Vec3(oldSize, 0, oldSize)), newSize),
                    null, (byte) 0, Label.MIXED);
            case 7 -> newRoot = new Octree(
                    new Boundary(Vec3.sub(boundary.position, new Vec3(0, oldSize, oldSize)), newSize),
                    null, (byte) 0, Label.MIXED);
            default -> newRoot = new Octree(
                    new Boundary(Vec3.sub(boundary.position, new Vec3(oldSize, oldSize, oldSize)), newSize),
                    null, (byte) 0, Label.MIXED);
        }
        newRoot.subdivideExpand(this, (byte) oldcode);
        return newRoot;
    }

    // ========================================================================================================== //

    public List<Octree> getTopEdgeChildrenLeafs(Stack<Integer> codes) {
        List<Octree> list = new ArrayList<>();
        if (this.children.isEmpty()) {
            if (this.label != Label.BLOCKED)
                list.add(this);
            return list;
        }
        if (!codes.isEmpty()) {
            list.addAll(children.get(codes.pop()).getTopEdgeChildrenLeafs(codes));
        }
        else {
            list.addAll(children.get(0).getTopEdgeChildrenLeafs(codes));
            list.addAll(children.get(1).getTopEdgeChildrenLeafs(codes));
            list.addAll(children.get(4).getTopEdgeChildrenLeafs(codes));
            list.addAll(children.get(5).getTopEdgeChildrenLeafs(codes));
        }
        return list;
    }
    public List<Octree> getRightEdgeChildrenLeafs(Stack<Integer> codes) {
        List<Octree> list = new ArrayList<>();
        if (this.children.isEmpty()) {
            if (this.label != Label.BLOCKED)
                list.add(this);
            return list;
        }
        if (!codes.isEmpty()) {
            list.addAll(children.get(codes.pop()).getRightEdgeChildrenLeafs(codes));
        }
        else {
            list.addAll(children.get(1).getRightEdgeChildrenLeafs(codes));
            list.addAll(children.get(3).getRightEdgeChildrenLeafs(codes));
            list.addAll(children.get(5).getRightEdgeChildrenLeafs(codes));
            list.addAll(children.get(7).getRightEdgeChildrenLeafs(codes));
        }
        return list;
    }
    public List<Octree> getBottomEdgeChildrenLeafs(Stack<Integer> codes) {
        List<Octree> list = new ArrayList<>();
        if (this.children.isEmpty()) {
            if (this.label != Label.BLOCKED)
                list.add(this);
            return list;
        }
        if (!codes.isEmpty()) {
            list.addAll(children.get(codes.pop()).getBottomEdgeChildrenLeafs(codes));
        }
        else {
            list.addAll(children.get(2).getBottomEdgeChildrenLeafs(codes));
            list.addAll(children.get(3).getBottomEdgeChildrenLeafs(codes));
            list.addAll(children.get(6).getBottomEdgeChildrenLeafs(codes));
            list.addAll(children.get(7).getBottomEdgeChildrenLeafs(codes));
        }
        return list;
    }
    public List<Octree> getLeftEdgeChildrenLeafs(Stack<Integer> codes) {
        List<Octree> list = new ArrayList<>();
        if (this.children.isEmpty()) {
            if (this.label != Label.BLOCKED)
                list.add(this);
            return list;
        }
        if (!codes.isEmpty()) {
            list.addAll(children.get(codes.pop()).getLeftEdgeChildrenLeafs(codes));
        }
        else {
            list.addAll(children.get(0).getLeftEdgeChildrenLeafs(codes));
            list.addAll(children.get(2).getLeftEdgeChildrenLeafs(codes));
            list.addAll(children.get(4).getLeftEdgeChildrenLeafs(codes));
            list.addAll(children.get(6).getLeftEdgeChildrenLeafs(codes));
        }
        return list;
    }
    public List<Octree> getFrontEdgeChildrenLeafs(Stack<Integer> codes) {
        List<Octree> list = new ArrayList<>();
        if (this.children.isEmpty()) {
            if (this.label != Label.BLOCKED)
                list.add(this);
            return list;
        }
        if (!codes.isEmpty()) {
            list.addAll(children.get(codes.pop()).getFrontEdgeChildrenLeafs(codes));
        }
        else {
            list.addAll(children.get(0).getFrontEdgeChildrenLeafs(codes));
            list.addAll(children.get(1).getFrontEdgeChildrenLeafs(codes));
            list.addAll(children.get(2).getFrontEdgeChildrenLeafs(codes));
            list.addAll(children.get(3).getFrontEdgeChildrenLeafs(codes));
        }
        return list;
    }
    public List<Octree> getBackEdgeChildrenLeafs(Stack<Integer> codes) {
        List<Octree> list = new ArrayList<>();
        if (this.children.isEmpty()) {
            if (this.label != Label.BLOCKED)
                list.add(this);
            return list;
        }
        if (!codes.isEmpty()) {
            list.addAll(children.get(codes.pop()).getBackEdgeChildrenLeafs(codes));
        }
        else {
            list.addAll(children.get(4).getBackEdgeChildrenLeafs(codes));
            list.addAll(children.get(5).getBackEdgeChildrenLeafs(codes));
            list.addAll(children.get(6).getBackEdgeChildrenLeafs(codes));
            list.addAll(children.get(7).getBackEdgeChildrenLeafs(codes));
        }
        return list;
    }

    public List<Octree> getTopLeftEdgeChildrenLeafs(Stack<Integer> codes) {
        List<Octree> list = new ArrayList<>();
        if (this.children.isEmpty()) {
            if (this.label != Label.BLOCKED)
                list.add(this);
            return list;
        }
        if (!codes.isEmpty()) {
            list.addAll(children.get(codes.pop()).getTopLeftEdgeChildrenLeafs(codes));
        }
        else {
            list.addAll(children.get(0).getTopLeftEdgeChildrenLeafs(codes));
            list.addAll(children.get(4).getTopLeftEdgeChildrenLeafs(codes));
        }
        return list;
    }
    public List<Octree> getTopRightEdgeChildrenLeafs(Stack<Integer> codes) {
        List<Octree> list = new ArrayList<>();
        if (this.children.isEmpty()) {
            if (this.label != Label.BLOCKED)
                list.add(this);
            return list;
        }
        if (!codes.isEmpty()) {
            list.addAll(children.get(codes.pop()).getTopRightEdgeChildrenLeafs(codes));
        }
        else {
            list.addAll(children.get(1).getTopRightEdgeChildrenLeafs(codes));
            list.addAll(children.get(5).getTopRightEdgeChildrenLeafs(codes));
        }
        return list;
    }
    public List<Octree> getBottomLeftEdgeChildrenLeafs(Stack<Integer> codes) {
        List<Octree> list = new ArrayList<>();
        if (this.children.isEmpty()) {
            if (this.label != Label.BLOCKED)
                list.add(this);
            return list;
        }
        if (!codes.isEmpty()) {
            list.addAll(children.get(codes.pop()).getBottomLeftEdgeChildrenLeafs(codes));
        }
        else {
            list.addAll(children.get(2).getBottomLeftEdgeChildrenLeafs(codes));
            list.addAll(children.get(6).getBottomLeftEdgeChildrenLeafs(codes));
        }
        return list;
    }
    public List<Octree> getBottomRightEdgeChildrenLeafs(Stack<Integer> codes) {
        List<Octree> list = new ArrayList<>();
        if (this.children.isEmpty()) {
            if (this.label != Label.BLOCKED)
                list.add(this);
            return list;
        }
        if (!codes.isEmpty()) {
            list.addAll(children.get(codes.pop()).getBottomRightEdgeChildrenLeafs(codes));
        }
        else {
            list.addAll(children.get(3).getBottomRightEdgeChildrenLeafs(codes));
            list.addAll(children.get(7).getBottomRightEdgeChildrenLeafs(codes));
        }
        return list;
    }
    public List<Octree> getLeftFrontEdgeChildrenLeafs(Stack<Integer> codes) {
        List<Octree> list = new ArrayList<>();
        if (this.children.isEmpty()) {
            if (this.label != Label.BLOCKED)
                list.add(this);
            return list;
        }
        if (!codes.isEmpty()) {
            list.addAll(children.get(codes.pop()).getLeftFrontEdgeChildrenLeafs(codes));
        }
        else {
            list.addAll(children.get(0).getLeftFrontEdgeChildrenLeafs(codes));
            list.addAll(children.get(2).getLeftFrontEdgeChildrenLeafs(codes));
        }
        return list;
    }
    public List<Octree> getRightFrontEdgeChildrenLeafs(Stack<Integer> codes) {
        List<Octree> list = new ArrayList<>();
        if (this.children.isEmpty()) {
            if (this.label != Label.BLOCKED)
                list.add(this);
            return list;
        }
        if (!codes.isEmpty()) {
            list.addAll(children.get(codes.pop()).getRightFrontEdgeChildrenLeafs(codes));
        }
        else {
            list.addAll(children.get(1).getRightFrontEdgeChildrenLeafs(codes));
            list.addAll(children.get(3).getRightFrontEdgeChildrenLeafs(codes));
        }
        return list;
    }
    public List<Octree> getLeftBackEdgeChildrenLeafs(Stack<Integer> codes) {
        List<Octree> list = new ArrayList<>();
        if (this.children.isEmpty()) {
            if (this.label != Label.BLOCKED)
                list.add(this);
            return list;
        }
        if (!codes.isEmpty()) {
            list.addAll(children.get(codes.pop()).getLeftBackEdgeChildrenLeafs(codes));
        }
        else {
            list.addAll(children.get(4).getLeftBackEdgeChildrenLeafs(codes));
            list.addAll(children.get(6).getLeftBackEdgeChildrenLeafs(codes));
        }
        return list;
    }
    public List<Octree> getRightBackEdgeChildrenLeafs(Stack<Integer> codes) {
        List<Octree> list = new ArrayList<>();
        if (this.children.isEmpty()) {
            if (this.label != Label.BLOCKED)
                list.add(this);
            return list;
        }
        if (!codes.isEmpty()) {
            list.addAll(children.get(codes.pop()).getRightBackEdgeChildrenLeafs(codes));
        }
        else {
            list.addAll(children.get(5).getRightBackEdgeChildrenLeafs(codes));
            list.addAll(children.get(7).getRightBackEdgeChildrenLeafs(codes));
        }
        return list;
    }
    public List<Octree> getTopFrontEdgeChildrenLeafs(Stack<Integer> codes) {
        List<Octree> list = new ArrayList<>();
        if (this.children.isEmpty()) {
            if (this.label != Label.BLOCKED)
                list.add(this);
            return list;
        }
        if (!codes.isEmpty()) {
            list.addAll(children.get(codes.pop()).getTopFrontEdgeChildrenLeafs(codes));
        }
        else {
            list.addAll(children.get(0).getTopFrontEdgeChildrenLeafs(codes));
            list.addAll(children.get(1).getTopFrontEdgeChildrenLeafs(codes));
        }
        return list;
    }
    public List<Octree> getBottomFrontEdgeChildrenLeafs(Stack<Integer> codes) {
        List<Octree> list = new ArrayList<>();
        if (this.children.isEmpty()) {
            if (this.label != Label.BLOCKED)
                list.add(this);
            return list;
        }
        if (!codes.isEmpty()) {
            list.addAll(children.get(codes.pop()).getBottomFrontEdgeChildrenLeafs(codes));
        }
        else {
            list.addAll(children.get(2).getBottomFrontEdgeChildrenLeafs(codes));
            list.addAll(children.get(3).getBottomFrontEdgeChildrenLeafs(codes));
        }
        return list;
    }
    public List<Octree> getTopBackEdgeChildrenLeafs(Stack<Integer> codes) {
        List<Octree> list = new ArrayList<>();
        if (this.children.isEmpty()) {
            if (this.label != Label.BLOCKED)
                list.add(this);
            return list;
        }
        if (!codes.isEmpty()) {
            list.addAll(children.get(codes.pop()).getTopBackEdgeChildrenLeafs(codes));
        }
        else {
            list.addAll(children.get(4).getTopBackEdgeChildrenLeafs(codes));
            list.addAll(children.get(5).getTopBackEdgeChildrenLeafs(codes));
        }
        return list;
    }
    public List<Octree> getBottomBackEdgeChildrenLeafs(Stack<Integer> codes) {
        List<Octree> list = new ArrayList<>();
        if (this.children.isEmpty()) {
            if (this.label != Label.BLOCKED)
                list.add(this);
            return list;
        }
        if (!codes.isEmpty()) {
            list.addAll(children.get(codes.pop()).getBottomBackEdgeChildrenLeafs(codes));
        }
        else {
            list.addAll(children.get(6).getBottomBackEdgeChildrenLeafs(codes));
            list.addAll(children.get(7).getBottomBackEdgeChildrenLeafs(codes));
        }
        return list;
    }

    public List<Octree> getTopLeftFrontEdgeChildrenLeafs() {
        List<Octree> list = new ArrayList<>();
        if (this.children.isEmpty()) {
            if (this.label != Label.BLOCKED)
                list.add(this);
            return list;
        }
        return children.get(0).getTopLeftFrontEdgeChildrenLeafs();
    }
    public List<Octree> getTopRightFrontEdgeChildrenLeafs() {
        List<Octree> list = new ArrayList<>();
        if (this.children.isEmpty()) {
            if (this.label != Label.BLOCKED)
                list.add(this);
            return list;
        }
        return children.get(1).getTopRightFrontEdgeChildrenLeafs();
    }
    public List<Octree> getBottomLeftFrontEdgeChildrenLeafs() {
        List<Octree> list = new ArrayList<>();
        if (this.children.isEmpty()) {
            if (this.label != Label.BLOCKED)
                list.add(this);
            return list;
        }
        return children.get(2).getBottomLeftFrontEdgeChildrenLeafs();
    }
    public List<Octree> getBottomRightFrontEdgeChildrenLeafs() {
        List<Octree> list = new ArrayList<>();
        if (this.children.isEmpty()) {
            if (this.label != Label.BLOCKED)
                list.add(this);
            return list;
        }
        return children.get(3).getBottomRightFrontEdgeChildrenLeafs();
    }
    public List<Octree> getTopLeftBackEdgeChildrenLeafs() {
        List<Octree> list = new ArrayList<>();
        if (this.children.isEmpty()) {
            if (this.label != Label.BLOCKED)
                list.add(this);
            return list;
        }
        return children.get(4).getTopLeftBackEdgeChildrenLeafs();
    }
    public List<Octree> getTopRightBackEdgeChildrenLeafs() {
        List<Octree> list = new ArrayList<>();
        if (this.children.isEmpty()) {
            if (this.label != Label.BLOCKED)
                list.add(this);
            return list;
        }
        return children.get(5).getTopRightBackEdgeChildrenLeafs();
    }
    public List<Octree> getBottomLeftBackEdgeChildrenLeafs() {
        List<Octree> list = new ArrayList<>();
        if (this.children.isEmpty()) {
            if (this.label != Label.BLOCKED)
                list.add(this);
            return list;
        }
        return children.get(6).getBottomLeftBackEdgeChildrenLeafs();
    }
    public List<Octree> getBottomRightBackEdgeChildrenLeafs() {
        List<Octree> list = new ArrayList<>();
        if (this.children.isEmpty()) {
            if (this.label != Label.BLOCKED)
                list.add(this);
            return list;
        }
        return children.get(7).getBottomRightBackEdgeChildrenLeafs();
    }

    // ========================================================================================================== //

    public List<Octree> getTopNeighbour(Stack<Integer> codes) {
        switch (this.code) {
            case 1 -> { // top-left-front
                codes.push(2);
                return parent.getTopNeighbour(codes);
            }
            case 2 -> { // top-right-front
                codes.push(3);
                return parent.getTopNeighbour(codes);
            }
            case 3 -> { // bottom-left-front
                return parent.children.get(0).getBottomEdgeChildrenLeafs(codes);
            }
            case 4 -> { // bottom-right-front
                return parent.children.get(1).getBottomEdgeChildrenLeafs(codes);
            }
            case 5 -> { // top-left-back
                codes.push(6);
                return parent.getTopNeighbour(codes);
            }
            case 6 -> { // top-right-back
                codes.push(7);
                return parent.getTopNeighbour(codes);
            }
            case 7 -> { // bottom-left-back
                return parent.children.get(4).getBottomEdgeChildrenLeafs(codes);
            }
            case 8 -> { // bottom-right-back
                return parent.children.get(5).getBottomEdgeChildrenLeafs(codes);
            }
        }
        return null; // Should never happen
    }
    public List<Octree> getRightNeighbour(Stack<Integer> codes) {
        switch (this.code) {
            case 1 -> { // top-left-front
                return parent.children.get(1).getLeftEdgeChildrenLeafs(codes);
            }
            case 2 -> { // top-right-front
                codes.push(0);
                return parent.getRightNeighbour(codes);
            }
            case 3 -> { // bottom-left-front
                return parent.children.get(3).getLeftEdgeChildrenLeafs(codes);
            }
            case 4 -> { // bottom-right-front
                codes.push(2);
                return parent.getRightNeighbour(codes);
            }
            case 5 -> { // top-left-back
                return parent.children.get(5).getLeftEdgeChildrenLeafs(codes);
            }
            case 6 -> { // top-right-back
                codes.push(4);
                return parent.getRightNeighbour(codes);
            }
            case 7 -> { // bottom-left-back
                return parent.children.get(7).getLeftEdgeChildrenLeafs(codes);
            }
            case 8 -> { // bottom-right-back
                codes.push(6);
                return parent.getRightNeighbour(codes);
            }
        }
        return null; // Should never happen
    }
    public List<Octree> getBottomNeighbour(Stack<Integer> codes) {
        switch (this.code) {
            case 1 -> { // top-left-front
                return parent.children.get(2).getTopEdgeChildrenLeafs(codes);
            }
            case 2 -> { // top-right-front
                return parent.children.get(3).getTopEdgeChildrenLeafs(codes);
            }
            case 3 -> { // bottom-left-front
                codes.push(0);
                return parent.getBottomNeighbour(codes);
            }
            case 4 -> { // bottom-right-front
                codes.push(1);
                return parent.getBottomNeighbour(codes);
            }
            case 5 -> { // top-left-back
                return parent.children.get(6).getTopEdgeChildrenLeafs(codes);
            }
            case 6 -> { // top-right-back
                return parent.children.get(7).getTopEdgeChildrenLeafs(codes);
            }
            case 7 -> { // bottom-left-back
                codes.push(4);
                return parent.getBottomNeighbour(codes);
            }
            case 8 -> { // bottom-right-back
                codes.push(5);
                return parent.getBottomNeighbour(codes);
            }
        }
        return null; // Should never happen
    }
    public List<Octree> getLeftNeighbour(Stack<Integer> codes) {
        switch (this.code) {
            case 1 -> { // top-left-front
                codes.push(1);
                return parent.getLeftNeighbour(codes);
            }
            case 2 -> { // top-right-front
                return parent.children.get(0).getRightEdgeChildrenLeafs(codes);
            }
            case 3 -> { // bottom-left-front
                codes.push(3);
                return parent.getLeftNeighbour(codes);
            }
            case 4 -> { // bottom-right-front
                return parent.children.get(2).getRightEdgeChildrenLeafs(codes);
            }
            case 5 -> { // top-left-back
                codes.push(5);
                return parent.getLeftNeighbour(codes);
            }
            case 6 -> { // top-right-back
                return parent.children.get(4).getRightEdgeChildrenLeafs(codes);
            }
            case 7 -> { // bottom-left-back
                codes.push(7);
                return parent.getLeftNeighbour(codes);
            }
            case 8 -> { // bottom-right-back
                return parent.children.get(6).getRightEdgeChildrenLeafs(codes);
            }
        }
        return null; // Should never happen
    }
    public List<Octree> getFrontNeighbour(Stack<Integer> codes) {
        switch (this.code) {
            case 1 -> { // top-left-front
                codes.push(4);
                return parent.getFrontNeighbour(codes);
            }
            case 2 -> { // top-right-front
                codes.push(5);
                return parent.getFrontNeighbour(codes);
            }
            case 3 -> { // bottom-left-front
                codes.push(6);
                return parent.getFrontNeighbour(codes);
            }
            case 4 -> { // bottom-right-front
                codes.push(7);
                return parent.getFrontNeighbour(codes);
            }
            case 5 -> { // top-left-back
                return parent.children.get(0).getBackEdgeChildrenLeafs(codes);
            }
            case 6 -> { // top-right-back
                return parent.children.get(1).getBackEdgeChildrenLeafs(codes);
            }
            case 7 -> { // bottom-left-back
                return parent.children.get(2).getBackEdgeChildrenLeafs(codes);
            }
            case 8 -> { // bottom-right-back
                return parent.children.get(3).getBackEdgeChildrenLeafs(codes);
            }
        }
        return null; // Should never happen
    }
    public List<Octree> getBackNeighbour(Stack<Integer> codes) {
        switch (this.code) {
            case 1 -> { // top-left-front
                return parent.children.get(4).getFrontEdgeChildrenLeafs(codes);
            }
            case 2 -> { // top-right-front
                return parent.children.get(5).getFrontEdgeChildrenLeafs(codes);
            }
            case 3 -> { // bottom-left-front
                return parent.children.get(6).getFrontEdgeChildrenLeafs(codes);
            }
            case 4 -> { // bottom-right-front
                return parent.children.get(7).getFrontEdgeChildrenLeafs(codes);
            }
            case 5 -> { // top-left-back
                codes.push(0);
                return parent.getBackNeighbour(codes);
            }
            case 6 -> { // top-right-back
                codes.push(1);
                return parent.getBackNeighbour(codes);
            }
            case 7 -> { // bottom-left-back
                codes.push(2);
                return parent.getBackNeighbour(codes);
            }
            case 8 -> { // bottom-right-back
                codes.push(3);
                return parent.getBackNeighbour(codes);
            }
        }
        return null; // Should never happen
    }

    // ========================================================================================================== //

    public List<Octree> getTopLeftNeighbour(Stack<Integer> codes) {
        switch (this.code) {
            case 1 -> { // top-left-front
                codes.push(3);
                return parent.getTopLeftNeighbour(codes);
            }
            case 2 -> { // top-right-front
                codes.push(2);
                return parent.children.get(0).getTopNeighbour(codes);
            }
            case 3 -> { // bottom-left-front
                codes.push(1);
                return parent.children.get(0).getLeftNeighbour(codes);
            }
            case 4 -> { // bottom-right-front
                return parent.children.get(0).getBottomRightEdgeChildrenLeafs(codes);
            }
            case 5 -> { // top-left-back
                codes.push(7);
                return parent.getTopLeftNeighbour(codes);
            }
            case 6 -> { // top-right-back
                codes.push(6);
                return parent.children.get(4).getTopNeighbour(codes);
            }
            case 7 -> { // bottom-left-back
                codes.push(5);
                return parent.children.get(4).getLeftNeighbour(codes);
            }
            case 8 -> { // bottom-right-back
                return parent.children.get(4).getBottomRightEdgeChildrenLeafs(codes);
            }
        }
        return null; // Should never happen
    }
    public List<Octree> getTopRightNeighbour(Stack<Integer> codes) {
        switch (this.code) {
            case 1 -> { // top-left-front
                codes.push(3);
                return parent.children.get(1).getTopNeighbour(codes);
            }
            case 2 -> { // top-right-front
                codes.push(2);
                return parent.getTopRightNeighbour(codes);
            }
            case 3 -> { // bottom-left-front
                return parent.children.get(1).getBottomLeftEdgeChildrenLeafs(codes);
            }
            case 4 -> { // bottom-right-front
                codes.push(0);
                return parent.children.get(1).getRightNeighbour(codes);
            }
            case 5 -> { // top-left-back
                codes.push(7);
                return parent.children.get(5).getTopNeighbour(codes);
            }
            case 6 -> { // top-right-back
                codes.push(6);
                return parent.getTopRightNeighbour(codes);
            }
            case 7 -> { // bottom-left-back
                return parent.children.get(5).getBottomLeftEdgeChildrenLeafs(codes);
            }
            case 8 -> { // bottom-right-back
                codes.push(4);
                return parent.children.get(5).getRightNeighbour(codes);
            }
        }
        return null; // Should never happen
    }
    public List<Octree> getBottomLeftNeighbour(Stack<Integer> codes) {
        switch (this.code) {
            case 1 -> { // top-left-front
                codes.push(3);
                return parent.children.get(2).getLeftNeighbour(codes);
            }
            case 2 -> { // top-right-front
                return parent.children.get(2).getTopRightEdgeChildrenLeafs(codes);
            }
            case 3 -> { // bottom-left-front
                codes.push(1);
                return parent.getBottomLeftNeighbour(codes);
            }
            case 4 -> { // bottom-right-front
                codes.push(0);
                return parent.children.get(2).getBottomNeighbour(codes);
            }
            case 5 -> { // top-left-back
                codes.push(7);
                return parent.children.get(6).getRightNeighbour(codes);
            }
            case 6 -> { // top-right-back
                return parent.children.get(6).getTopRightEdgeChildrenLeafs(codes);
            }
            case 7 -> { // bottom-left-back
                codes.push(5);
                return parent.getBottomLeftNeighbour(codes);
            }
            case 8 -> { // bottom-right-back
                codes.push(4);
                return parent.children.get(6).getTopNeighbour(codes);
            }
        }
        return null; // Should never happen
    }
    public List<Octree> getBottomRightNeighbour(Stack<Integer> codes) {
        switch (this.code) {
            case 1 -> { // top-left-front
                return parent.children.get(3).getTopLeftEdgeChildrenLeafs(codes);
            }
            case 2 -> { // top-right-front
                codes.push(2);
                return parent.children.get(3).getRightNeighbour(codes);
            }
            case 3 -> { // bottom-left-front
                codes.push(1);
                return parent.children.get(3).getBottomNeighbour(codes);
            }
            case 4 -> { // bottom-right-front
                codes.push(0);
                return parent.getBottomRightNeighbour(codes);
            }
            case 5 -> { // top-left-back
                return parent.children.get(7).getTopLeftEdgeChildrenLeafs(codes);
            }
            case 6 -> { // top-right-back
                codes.push(6);
                return parent.children.get(7).getRightNeighbour(codes);
            }
            case 7 -> { // bottom-left-back
                codes.push(5);
                return parent.children.get(7).getBottomNeighbour(codes);
            }
            case 8 -> { // bottom-right-back
                codes.push(4);
                return parent.getBottomRightNeighbour(codes);
            }
        }
        return null; // Should never happen
    }

    public List<Octree> getLeftFrontNeighbour(Stack<Integer> codes) {
        switch (this.code) {
            case 1 -> { // top-left-front
                codes.push(5);
                return parent.getLeftFrontNeighbour(codes);
            }
            case 2 -> { // top-right-front
                codes.push(4);
                return parent.children.get(0).getFrontNeighbour(codes);
            }
            case 3 -> { // bottom-left-front
                codes.push(7);
                return parent.getLeftFrontNeighbour(codes);
            }
            case 4 -> { // bottom-right-front
                codes.push(6);
                return parent.children.get(2).getFrontNeighbour(codes);
            }
            case 5 -> { // top-left-back
                codes.push(1);
                return parent.children.get(0).getLeftNeighbour(codes);
            }
            case 6 -> { // top-right-back
                return parent.children.get(0).getRightBackEdgeChildrenLeafs(codes);
            }
            case 7 -> { // bottom-left-back
                codes.push(3);
                return parent.children.get(2).getLeftNeighbour(codes);
            }
            case 8 -> { // bottom-right-back
                return parent.children.get(2).getRightBackEdgeChildrenLeafs(codes);
            }
        }
        return null; // Should never happen
    }
    public List<Octree> getRightFrontNeighbour(Stack<Integer> codes) {
        switch (this.code) {
            case 1 -> { // top-left-front
                codes.push(5);
                return parent.children.get(1).getFrontNeighbour(codes);
            }
            case 2 -> { // top-right-front
                codes.push(4);
                return parent.getRightFrontNeighbour(codes);
            }
            case 3 -> { // bottom-left-front
                codes.push(7);
                return parent.children.get(3).getFrontNeighbour(codes);
            }
            case 4 -> { // bottom-right-front
                codes.push(6);
                return parent.getRightFrontNeighbour(codes);
            }
            case 5 -> { // top-left-back
                return parent.children.get(1).getLeftBackEdgeChildrenLeafs(codes);
            }
            case 6 -> { // top-right-back
                codes.push(0);
                return parent.children.get(1).getRightNeighbour(codes);
            }
            case 7 -> { // bottom-left-back
                return parent.children.get(3).getLeftBackEdgeChildrenLeafs(codes);
            }
            case 8 -> { // bottom-right-back
                codes.push(2);
                return parent.children.get(3).getRightNeighbour(codes);
            }
        }
        return null; // Should never happen
    }
    public List<Octree> getLeftBackNeighbour(Stack<Integer> codes) {
        switch (this.code) {
            case 1 -> { // top-left-front
                codes.push(5);
                return parent.children.get(4).getLeftNeighbour(codes);
            }
            case 2 -> { // top-right-front
                return parent.children.get(4).getRightFrontEdgeChildrenLeafs(codes);
            }
            case 3 -> { // bottom-left-front
                codes.push(7);
                return parent.children.get(6).getLeftNeighbour(codes);
            }
            case 4 -> { // bottom-right-front
                return parent.children.get(6).getRightFrontEdgeChildrenLeafs(codes);
            }
            case 5 -> { // top-left-back
                codes.push(1);
                return parent.getLeftBackNeighbour(codes);
            }
            case 6 -> { // top-right-back
                codes.push(0);
                return parent.children.get(4).getBackNeighbour(codes);
            }
            case 7 -> { // bottom-left-back
                codes.push(3);
                return parent.getLeftBackNeighbour(codes);
            }
            case 8 -> { // bottom-right-back
                codes.push(2);
                return parent.children.get(6).getBackNeighbour(codes);
            }
        }
        return null; // Should never happen
    }
    public List<Octree> getRightBackNeighbour(Stack<Integer> codes) {
        switch (this.code) {
            case 1 -> { // top-left-front
                return parent.children.get(5).getLeftFrontEdgeChildrenLeafs(codes);
            }
            case 2 -> { // top-right-front
                codes.push(4);
                return parent.children.get(5).getRightNeighbour(codes);
            }
            case 3 -> { // bottom-left-front
                return parent.children.get(7).getLeftFrontEdgeChildrenLeafs(codes);
            }
            case 4 -> { // bottom-right-front
                codes.push(6);
                return parent.children.get(7).getRightNeighbour(codes);
            }
            case 5 -> { // top-left-back
                codes.push(1);
                return parent.children.get(5).getBackNeighbour(codes);
            }
            case 6 -> { // top-right-back
                codes.push(0);
                return parent.getRightBackNeighbour(codes);
            }
            case 7 -> { // bottom-left-back
                codes.push(3);
                return parent.children.get(7).getBackNeighbour(codes);
            }
            case 8 -> { // bottom-right-back
                codes.push(2);
                return parent.getRightBackNeighbour(codes);
            }
        }
        return null; // Should never happen
    }

    public List<Octree> getTopFrontNeighbour(Stack<Integer> codes) {
        switch (this.code) {
            case 1 -> { // top-left-front
                codes.push(6);
                return parent.getTopFrontNeighbour(codes);
            }
            case 2 -> { // top-right-front
                codes.push(7);
                return parent.getTopFrontNeighbour(codes);
            }
            case 3 -> { // bottom-left-front
                codes.push(4);
                return parent.children.get(0).getFrontNeighbour(codes);
            }
            case 4 -> { // bottom-right-front
                codes.push(5);
                return parent.children.get(1).getFrontNeighbour(codes);
            }
            case 5 -> { // top-left-back
                codes.push(2);
                return parent.children.get(0).getTopNeighbour(codes);
            }
            case 6 -> { // top-right-back
                codes.push(3);
                return parent.children.get(1).getTopNeighbour(codes);
            }
            case 7 -> { // bottom-left-back
                return parent.children.get(0).getBottomBackEdgeChildrenLeafs(codes);
            }
            case 8 -> { // bottom-right-back
                return parent.children.get(1).getBottomBackEdgeChildrenLeafs(codes);
            }
        }
        return null; // Should never happen
    }
    public List<Octree> getBottomFrontNeighbour(Stack<Integer> codes) {
        switch (this.code) {
            case 1 -> { // top-left-front
                codes.push(6);
                return parent.children.get(2).getFrontNeighbour(codes);
            }
            case 2 -> { // top-right-front
                codes.push(7);
                return parent.children.get(3).getFrontNeighbour(codes);
            }
            case 3 -> { // bottom-left-front
                codes.push(4);
                return parent.getBottomFrontNeighbour(codes);
            }
            case 4 -> { // bottom-right-front
                codes.push(5);
                return parent.getBottomFrontNeighbour(codes);
            }
            case 5 -> { // top-left-back
                return parent.children.get(2).getTopBackEdgeChildrenLeafs(codes);
            }
            case 6 -> { // top-right-back
                return parent.children.get(3).getTopBackEdgeChildrenLeafs(codes);
            }
            case 7 -> { // bottom-left-back
                codes.push(0);
                return parent.children.get(2).getBottomNeighbour(codes);
            }
            case 8 -> { // bottom-right-back
                codes.push(1);
                return parent.children.get(3).getBottomNeighbour(codes);
            }
        }
        return null; // Should never happen
    }
    public List<Octree> getTopBackNeighbour(Stack<Integer> codes) {
        switch (this.code) {
            case 1 -> { // top-left-front
                codes.push(6);
                return parent.children.get(4).getTopNeighbour(codes);
            }
            case 2 -> { // top-right-front
                codes.push(7);
                return parent.children.get(5).getTopNeighbour(codes);
            }
            case 3 -> { // bottom-left-front
                return parent.children.get(4).getBottomFrontEdgeChildrenLeafs(codes);
            }
            case 4 -> { // bottom-right-front
                return parent.children.get(5).getBottomFrontEdgeChildrenLeafs(codes);
            }
            case 5 -> { // top-left-back
                codes.push(2);
                return parent.getTopBackNeighbour(codes);
            }
            case 6 -> { // top-right-back
                codes.push(3);
                return parent.getTopBackNeighbour(codes);
            }
            case 7 -> { // bottom-left-back
                codes.push(0);
                return parent.children.get(4).getBackNeighbour(codes);
            }
            case 8 -> { // bottom-right-back
                codes.push(1);
                return parent.children.get(5).getBackNeighbour(codes);
            }
        }
        return null; // Should never happen
    }
    public List<Octree> getBottomBackNeighbour(Stack<Integer> codes) {
        switch (this.code) {
            case 1 -> { // top-left-front
                return parent.children.get(6).getTopFrontEdgeChildrenLeafs(codes);
            }
            case 2 -> { // top-right-front
                return parent.children.get(7).getTopFrontEdgeChildrenLeafs(codes);
            }
            case 3 -> { // bottom-left-front
                codes.push(4);
                return parent.children.get(6).getBottomNeighbour(codes);
            }
            case 4 -> { // bottom-right-front
                codes.push(5);
                return parent.children.get(7).getBottomNeighbour(codes);
            }
            case 5 -> { // top-left-back
                codes.push(2);
                return parent.children.get(6).getBackNeighbour(codes);
            }
            case 6 -> { // top-right-back
                codes.push(3);
                return parent.children.get(7).getBackNeighbour(codes);
            }
            case 7 -> { // bottom-left-back
                codes.push(0);
                return parent.getBottomBackNeighbour(codes);
            }
            case 8 -> { // bottom-right-back
                codes.push(1);
                return parent.getBottomBackNeighbour(codes);
            }
        }
        return null; // Should never happen
    }

    // ========================================================================================================== //

    public List<Octree> getTopLeftFrontNeighbour(Stack<Integer> codes) {
        switch (this.code) {
            case 1 -> { // top-left-front
                codes.push(7);
                return parent.getTopLeftFrontNeighbour(codes);
            }
            case 2 -> { // top-right-front
                codes.push(6);
                return parent.children.get(0).getTopFrontNeighbour(codes);
            }
            case 3 -> { // bottom-left-front
                codes.push(5);
                return parent.children.get(0).getLeftFrontNeighbour(codes);
            }
            case 4 -> { // bottom-right-front
                codes.push(4);
                return parent.children.get(0).getFrontNeighbour(codes);
            }
            case 5 -> { // top-left-back
                codes.push(3);
                return parent.children.get(0).getTopLeftNeighbour(codes);
            }
            case 6 -> { // top-right-back
                codes.push(2);
                return parent.children.get(0).getTopNeighbour(codes);
            }
            case 7 -> { // bottom-left-back
                codes.push(1);
                return parent.children.get(0).getLeftNeighbour(codes);
            }
            case 8 -> { // bottom-right-back
                return parent.children.get(0).getBottomRightBackEdgeChildrenLeafs();
            }
        }
        return null; // Should never happen
    }
    public List<Octree> getTopRightFrontNeighbour(Stack<Integer> codes) {
        switch (this.code) {
            case 1 -> { // top-left-front
                codes.push(7);
                return parent.children.get(1).getTopFrontNeighbour(codes);
            }
            case 2 -> { // top-right-front
                codes.push(6);
                return parent.getTopRightFrontNeighbour(codes);
            }
            case 3 -> { // bottom-left-front
                codes.push(5);
                return parent.children.get(1).getFrontNeighbour(codes);
            }
            case 4 -> { // bottom-right-front
                codes.push(4);
                return parent.children.get(1).getRightFrontNeighbour(codes);
            }
            case 5 -> { // top-left-back
                codes.push(3);
                return parent.children.get(1).getTopNeighbour(codes);
            }
            case 6 -> { // top-right-back
                codes.push(2);
                return parent.children.get(1).getTopRightNeighbour(codes);
            }
            case 7 -> { // bottom-left-back
                return parent.children.get(1).getBottomLeftBackEdgeChildrenLeafs();
            }
            case 8 -> { // bottom-right-back
                codes.push(0);
                return parent.children.get(1).getRightNeighbour(codes);
            }
        }
        return null; // Should never happen
    }
    public List<Octree> getBottomLeftFrontNeighbour(Stack<Integer> codes) {
        switch (this.code) {
            case 1 -> { // top-left-front
                codes.push(7);
                return parent.children.get(2).getLeftFrontNeighbour(codes);
            }
            case 2 -> { // top-right-front
                codes.push(6);
                return parent.children.get(2).getFrontNeighbour(codes);
            }
            case 3 -> { // bottom-left-front
                codes.push(5);
                return parent.getBottomLeftFrontNeighbour(codes);
            }
            case 4 -> { // bottom-right-front
                codes.push(4);
                return parent.children.get(2).getBottomFrontNeighbour(codes);
            }
            case 5 -> { // top-left-back
                codes.push(3);
                return parent.children.get(2).getLeftNeighbour(codes);
            }
            case 6 -> { // top-right-back
                return parent.children.get(2).getTopRightBackEdgeChildrenLeafs();
            }
            case 7 -> { // bottom-left-back
                codes.push(1);
                return parent.children.get(2).getBottomLeftNeighbour(codes);
            }
            case 8 -> { // bottom-right-back
                codes.push(0);
                return parent.children.get(2).getBottomNeighbour(codes);
            }
        }
        return null; // Should never happen
    }
    public List<Octree> getBottomRightFrontNeighbour(Stack<Integer> codes) {
        switch (this.code) {
            case 1 -> { // top-left-front
                codes.push(7);
                return parent.children.get(3).getFrontNeighbour(codes);
            }
            case 2 -> { // top-right-front
                codes.push(6);
                return parent.children.get(3).getRightFrontNeighbour(codes);
            }
            case 3 -> { // bottom-left-front
                codes.push(5);
                return parent.children.get(3).getBottomFrontNeighbour(codes);
            }
            case 4 -> { // bottom-right-front
                codes.push(4);
                return parent.getBottomRightFrontNeighbour(codes);
            }
            case 5 -> { // top-left-back
                return parent.children.get(3).getTopRightBackEdgeChildrenLeafs();
            }
            case 6 -> { // top-right-back
                codes.push(2);
                return parent.children.get(3).getRightNeighbour(codes);
            }
            case 7 -> { // bottom-left-back
                codes.push(1);
                return parent.children.get(3).getBottomNeighbour(codes);
            }
            case 8 -> { // bottom-right-back
                codes.push(0);
                return parent.children.get(3).getBottomRightNeighbour(codes);
            }
        }
        return null; // Should never happen
    }

    public List<Octree> getTopLeftBackNeighbour(Stack<Integer> codes) {
        switch (this.code) {
            case 1 -> { // top-left-front
                codes.push(7);
                return parent.children.get(4).getTopLeftNeighbour(codes);
            }
            case 2 -> { // top-right-front
                codes.push(6);
                return parent.children.get(4).getTopNeighbour(codes);
            }
            case 3 -> { // bottom-left-front
                codes.push(5);
                return parent.children.get(4).getLeftNeighbour(codes);
            }
            case 4 -> { // bottom-right-front
                return parent.children.get(4).getBottomRightFrontEdgeChildrenLeafs();
            }
            case 5 -> { // top-left-back
                codes.push(3);
                return parent.getTopLeftBackNeighbour(codes);
            }
            case 6 -> { // top-right-back
                codes.push(2);
                return parent.children.get(4).getTopBackNeighbour(codes);
            }
            case 7 -> { // bottom-left-back
                codes.push(1);
                return parent.children.get(4).getLeftBackNeighbour(codes);
            }
            case 8 -> { // bottom-right-back
                codes.push(0);
                return parent.children.get(4).getBackNeighbour(codes);
            }
        }
        return null; // Should never happen
    }
    public List<Octree> getTopRightBackNeighbour(Stack<Integer> codes) {
        switch (this.code) {
            case 1 -> { // top-left-front
                codes.push(7);
                return parent.children.get(5).getTopNeighbour(codes);
            }
            case 2 -> { // top-right-front
                codes.push(6);
                return parent.children.get(5).getTopRightNeighbour(codes);
            }
            case 3 -> { // bottom-left-front
                return parent.children.get(5).getBottomLeftFrontEdgeChildrenLeafs();
            }
            case 4 -> { // bottom-right-front
                codes.push(4);
                return parent.children.get(5).getRightNeighbour(codes);
            }
            case 5 -> { // top-left-back
                codes.push(3);
                return parent.children.get(5).getTopBackNeighbour(codes);
            }
            case 6 -> { // top-right-back
                codes.push(2);
                return parent.getTopRightBackNeighbour(codes);
            }
            case 7 -> { // bottom-left-back
                codes.push(1);
                return parent.children.get(5).getBackNeighbour(codes);
            }
            case 8 -> { // bottom-right-back
                codes.push(0);
                return parent.children.get(5).getRightBackNeighbour(codes);
            }
        }
        return null; // Should never happen
    }
    public List<Octree> getBottomLeftBackNeighbour(Stack<Integer> codes) {
        switch (this.code) {
            case 1 -> { // top-left-front
                codes.push(7);
                return parent.children.get(6).getLeftNeighbour(codes);
            }
            case 2 -> { // top-right-front
                return parent.children.get(6).getTopRightFrontEdgeChildrenLeafs();
            }
            case 3 -> { // bottom-left-front
                codes.push(5);
                return parent.children.get(6).getBottomLeftNeighbour(codes);
            }
            case 4 -> { // bottom-right-front
                codes.push(4);
                return parent.children.get(6).getBottomNeighbour(codes);
            }
            case 5 -> { // top-left-back
                codes.push(3);
                return parent.children.get(6).getLeftBackNeighbour(codes);
            }
            case 6 -> { // top-right-back
                codes.push(2);
                return parent.children.get(6).getBackNeighbour(codes);
            }
            case 7 -> { // bottom-left-back
                codes.push(1);
                return parent.getBottomLeftBackNeighbour(codes);
            }
            case 8 -> { // bottom-right-back
                codes.push(0);
                return parent.children.get(6).getBottomBackNeighbour(codes);
            }
        }
        return null; // Should never happen
    }
    public List<Octree> getBottomRightBackNeighbour(Stack<Integer> codes) {
        switch (this.code) {
            case 1 -> { // top-left-front
                return parent.children.get(7).getTopLeftFrontEdgeChildrenLeafs();
            }
            case 2 -> { // top-right-front
                codes.push(6);
                return parent.children.get(7).getRightNeighbour(codes);
            }
            case 3 -> { // bottom-left-front
                codes.push(5);
                return parent.children.get(7).getBottomNeighbour(codes);
            }
            case 4 -> { // bottom-right-front
                codes.push(4);
                return parent.children.get(7).getBottomRightNeighbour(codes);
            }
            case 5 -> { // top-left-back
                codes.push(3);
                return parent.children.get(7).getBackNeighbour(codes);
            }
            case 6 -> { // top-right-back
                codes.push(2);
                return parent.children.get(7).getRightBackNeighbour(codes);
            }
            case 7 -> { // bottom-left-back
                codes.push(1);
                return parent.children.get(7).getBottomBackNeighbour(codes);
            }
            case 8 -> { // bottom-right-back
                codes.push(0);
                return parent.getBottomRightBackNeighbour(codes);
            }
        }
        return null; // Should never happen
    }

    // ========================================================================================================== //

    @Override
    public Iterable<Octree> neighbours(Octree node) {
        List<Octree> candidates = new LinkedList<>();

        // Directional
        List<Octree> temp = node.getTopNeighbour(new Stack<>());
        if (temp != null)
            candidates.addAll(temp);
        temp = node.getRightNeighbour(new Stack<>());
        if (temp != null)
            candidates.addAll(temp);
        temp = node.getBottomNeighbour(new Stack<>());
        if (temp != null)
            candidates.addAll(temp);
        temp = node.getLeftNeighbour(new Stack<>());
        if (temp != null)
            candidates.addAll(temp);
        temp = node.getFrontNeighbour(new Stack<>());
        if (temp != null)
            candidates.addAll(temp);
        temp = node.getBackNeighbour(new Stack<>());
        if (temp != null)
            candidates.addAll(temp);

        // Diagonal (2D)
        temp = node.getTopLeftNeighbour(new Stack<>());
        if (temp != null)
            candidates.addAll(temp);
        temp = node.getTopRightNeighbour(new Stack<>());
        if (temp != null)
            candidates.addAll(temp);
        temp = node.getBottomLeftNeighbour(new Stack<>());
        if (temp != null)
            candidates.addAll(temp);
        temp = node.getBottomRightNeighbour(new Stack<>());
        if (temp != null)
            candidates.addAll(temp);
        temp = node.getLeftFrontNeighbour(new Stack<>());
        if (temp != null)
            candidates.addAll(temp);
        temp = node.getRightFrontNeighbour(new Stack<>());
        if (temp != null)
            candidates.addAll(temp);
        temp = node.getLeftBackNeighbour(new Stack<>());
        if (temp != null)
            candidates.addAll(temp);
        temp = node.getRightBackNeighbour(new Stack<>());
        if (temp != null)
            candidates.addAll(temp);
        temp = node.getTopFrontNeighbour(new Stack<>());
        if (temp != null)
            candidates.addAll(temp);
        temp = node.getBottomFrontNeighbour(new Stack<>());
        if (temp != null)
            candidates.addAll(temp);
        temp = node.getTopBackNeighbour(new Stack<>());
        if (temp != null)
            candidates.addAll(temp);
        temp = node.getBottomBackNeighbour(new Stack<>());
        if (temp != null)
            candidates.addAll(temp);

        // Diagonal (3D)
        temp = node.getTopLeftFrontNeighbour(new Stack<>());
        if (temp != null)
            candidates.addAll(temp);
        temp = node.getTopRightFrontNeighbour(new Stack<>());
        if (temp != null)
            candidates.addAll(temp);
        temp = node.getBottomLeftFrontNeighbour(new Stack<>());
        if (temp != null)
            candidates.addAll(temp);
        temp = node.getBottomRightFrontNeighbour(new Stack<>());
        if (temp != null)
            candidates.addAll(temp);
        temp = node.getTopLeftBackNeighbour(new Stack<>());
        if (temp != null)
            candidates.addAll(temp);
        temp = node.getTopRightBackNeighbour(new Stack<>());
        if (temp != null)
            candidates.addAll(temp);
        temp = node.getBottomLeftBackNeighbour(new Stack<>());
        if (temp != null)
            candidates.addAll(temp);
        temp = node.getBottomRightBackNeighbour(new Stack<>());
        if (temp != null)
            candidates.addAll(temp);

        return candidates;
    }

    @Override
    public float heuristic(Octree from, Octree to) {
        // using Manhattan distance...
        return Math.abs(to.boundary.center().x - from.boundary.center().x)
                + Math.abs(to.boundary.center().y - from.boundary.center().y)
                + Math.abs(to.boundary.center().z - from.boundary.center().z);
    }

    @Override
    public float distance(Octree from, Octree to) {
        return Math.abs(to.boundary.center().x - from.boundary.center().x)
                + Math.abs(to.boundary.center().y - from.boundary.center().y)
                + Math.abs(to.boundary.center().z - from.boundary.center().z);
    }

    @Override
    public boolean isUnknown(Octree node) {
        return node.label == Label.UNKNOWN;
    }

    public void export(PrintWriter printWriter) {
        if (children.isEmpty())
            printWriter.printf("%f %f %f %f %d %n", boundary.position.x, boundary.position.y, boundary.position.z, boundary.size(), label);
        for (Octree child : children) {
            child.export(printWriter);
        }
    }

    public int countNodes() {
        int count = 1;
        if (!children.isEmpty()) {
            for (int i = 0; i < 8; i++) {
                count += children.get(i).countNodes();
            }
        }
        return count;
    }
}
