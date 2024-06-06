package uuspaceagent;

import eu.iv4xr.framework.mainConcepts.WorldEntity;
import eu.iv4xr.framework.spatial.Vec3;
import uuspaceagent.exploration.Explorable;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import static java.lang.Math.min;
import static java.lang.Math.max;

public class VoxelGrid implements Explorable<DPos3> {

    /**
     * The assumed height of the player characters. It is 1.8, we conservatively
     * assume it is 2f.
     */
    public static float AGENT_HEIGHT = 1.8f ;
    public static float AGENT_WIDTH  = 1f ;
    public static float BLOCK_SIZE   = 2.5f ;

    public Boundary2 boundary;
    public float voxelSize;
    float squareDiagonalLength;
    float cubeDiagonalLength;
    ArrayList<ArrayList<ArrayList<Voxel>>> grid;

    public Voxel get(int x, int y, int z) {
        return grid.get(x).get(y).get(z);
    }
    public Voxel get(DPos3 pos) {
        return grid.get(pos.x).get(pos.y).get(pos.z);
    }
    public DPos3 size() { return new DPos3(grid.size(), grid.get(0).size(), grid.get(0).get(0).size()); }

    public VoxelGrid(float voxelSize) {
        this.voxelSize = voxelSize;
        squareDiagonalLength = new Vec3(0,voxelSize,voxelSize).length();
        cubeDiagonalLength = new Vec3(voxelSize,voxelSize,voxelSize).length();
    }

    /**
     *  Initializes the starting grid based on the first observed position and the observation radius.
     *  Call in UUSeAgentstate3D on first observation.
     */
    public void initializeGrid(Vec3 pos, float observation_radius) {
        // radius +10   so that the agent can move a little before having to rebuild the whole grid (expand)
        Vec3 lowerB = Vec3.sub(pos, new Vec3(observation_radius + BLOCK_SIZE));
        Vec3 upperB = Vec3.add(pos, new Vec3(observation_radius + BLOCK_SIZE));
        boundary = new Boundary2(lowerB, upperB);

        int initialSize = (int) ((boundary.upperBounds.x - boundary.lowerBounds.x) / voxelSize);
        grid = new ArrayList<>(initialSize);
        for (int x = 0; x < initialSize; x++) {
            grid.add(x, new ArrayList<>(initialSize));
            for (int y = 0; y < initialSize; y++) {
                grid.get(x).add(y, new ArrayList<>(initialSize));
                for (int z = 0; z < initialSize; z++) {
                    float distance = Vec3.dist(pos, new Vec3(x * voxelSize + boundary.lowerBounds.x,
                                                             y * voxelSize + boundary.lowerBounds.y,
                                                             z * voxelSize + boundary.lowerBounds.z));
                    grid.get(x).get(y).add(z, new Voxel(distance <= observation_radius ? Label.OPEN : Label.UNKNOWN));
                }
            }
        }
    }

    /**
     * Calculate the unit-cube that contains a point p.
     */
    public DPos3 gridProjectedLocation(Vec3 p) {
        p = Vec3.sub(p, boundary.lowerBounds) ;
        /*
        int x = (int) (Math.floor(p.x / CUBE_SIZE)) ;
        int y = (int) (Math.floor(p.y / CUBE_SIZE)) ;
        int z = (int) (Math.floor(p.z / CUBE_SIZE)) ;
        */
        int x = myFloor(p.x / voxelSize) ;
        int y = myFloor(p.y / voxelSize) ;
        int z = myFloor(p.z / voxelSize) ;

        return new DPos3(x,y,z) ;
    }

    /**
     * Return the actual location of the center of a unit cube; the location is expressed
     * as a 3D position in the space.
     */
    public Vec3 getCubeCenterLocation(DPos3 cube) {
        float x = (((float) cube.x) + 0.5f) * voxelSize + boundary.lowerBounds.x ;
        float y = (((float) cube.y) + 0.5f) * voxelSize + boundary.lowerBounds.y ;
        float z = (((float) cube.z) + 0.5f) * voxelSize + boundary.lowerBounds.z ;
        return new Vec3(x,y,z) ;
    }

    public static int myFloor(float x) {
        return (int) Math.floor(x) ;
    }

    List<DPos3> getObstructedCubes(WorldEntity block) {

        List<DPos3> obstructed = new LinkedList<>() ;

        Vec3 maxCorner = SEBlockFunctions.getBaseMaxCorner(block) ; // should add rotation if it is not a cube. TODO.
        Vec3 minCorner = SEBlockFunctions.getBaseMinCorner(block) ; // should add rotation if it is not a cube. TODO.


        // add some padding due to agent's body width/height:
        //      note:  agent height = 1.8, about 0.5 above feet is the rotation point
        //      note2: padding is too much because we only go to voxels center, so remove voxelSize / 2
        //      note3: gridProjectedLocation() rounds things down. Blocks are size 2.5, so maximum error is 0.5
        //             for voxelSize 2.5, so we remove BLOCK_SIZE % voxelSize / 2 from padding.
        //      (max with 0, so no negative padding)
        //Vec3 hpadding = Vec3.mul(new Vec3(AGENT_WIDTH, 0, AGENT_WIDTH), 0.6f) ;
        Vec3 vpadding = new Vec3(max(0, (AGENT_HEIGHT - voxelSize - BLOCK_SIZE % voxelSize) * 0.5f)) ;
        //minCorner = Vec3.sub(minCorner, hpadding) ;
        minCorner = Vec3.sub(minCorner, vpadding) ;
        maxCorner = Vec3.add(maxCorner, vpadding) ;

        var corner1 = gridProjectedLocation(minCorner) ;
        var corner2 = gridProjectedLocation(maxCorner) ;
        // all squares between these two corners are blocked:
        for(int x = corner1.x; x<=corner2.x; x++) {
            for (int y = Math.max(0, corner1.y); y <= corner2.y; y++) {
                // PS: cubes below y=0 are below the ground surface and hence won't obstruct
                // navigation on and above the surface.
                for (int z = corner1.z; z<=corner2.z; z++) {
                    var cube = new DPos3(x,y,z) ;
                    obstructed.add(cube) ;
                }
            }
        }
        return obstructed ;
    }

    public void addObstacle(WorldEntity block) {

        var obstructedCubes = getObstructedCubes(block);
        for(var voxel : obstructedCubes) {
            get(voxel).label = Label.BLOCKED;
        }
    }

    public void removeObstacle(WorldEntity block) {

        var obstructedCubes = getObstructedCubes(block) ;
        for(var voxel : obstructedCubes) {
            get(voxel).label = Label.OPEN;
        }
    }

    public void setUnknown(WorldEntity block) {

        var obstructedCubes = getObstructedCubes(block) ;
        for(var voxel : obstructedCubes) {
            get(voxel).label = Label.UNKNOWN;
        }
    }

    public void setOpen(WorldEntity block) {

        var obstructedCubes = getObstructedCubes(block) ;
        for(var voxel : obstructedCubes) {
            get(voxel).label = Label.OPEN;
        }
    }

    /**
     * Check if the VoxelGrid fully contains the viewing range. If not, expand it outwards.
     */
    public void checkAndExpand(Boundary range) {
        // If the VoxelGrid boundary fully contain the viewing range, do nothing
        if (this.boundary.contains(range))
            return;


        // Otherwise, expand
        if (range.pos().x < this.boundary.lowerBounds.x) { // expand to the left
            DPos3 gridSize = size();
            int diff = (int) Math.ceil(this.boundary.lowerBounds.x - range.pos().x);
            for (int x = 0; x < diff; x++) {
//                ArrayList<Voxel> temp = Stream.generate(Voxel::new)
//                        .limit(diff).collect(Collectors.toCollection(ArrayList::new));
                ArrayList<ArrayList<Voxel>> newX = Stream.generate(() -> Stream.generate(Voxel::new)
                                .limit(gridSize.z)
                                .collect(Collectors.toCollection(ArrayList::new)))
                        .limit(gridSize.y)
                        .collect(Collectors.toCollection(ArrayList::new));
                grid.add(0, newX);
            }
            boundary.lowerBounds.x -= voxelSize * diff;
        }
        else if (range.upperBounds().x > this.boundary.upperBounds.x) { // expand to the right
            DPos3 gridSize = size();
            int diff = (int) Math.ceil(range.upperBounds().x - this.boundary.upperBounds.x);
            for (int x = 0; x < diff; x++) {
                ArrayList<ArrayList<Voxel>> newX = Stream.generate(() -> Stream.generate(Voxel::new)
                                .limit(gridSize.z)
                                .collect(Collectors.toCollection(ArrayList::new)))
                        .limit(gridSize.y)
                        .collect(Collectors.toCollection(ArrayList::new));
                grid.add(newX);
            }
            boundary.upperBounds.x += voxelSize * diff;
        }
        if (range.pos().y < this.boundary.lowerBounds.y) { // expand downwards
            DPos3 gridSize = size();
            int diff = (int) Math.ceil(this.boundary.lowerBounds.y - range.pos().y);
            for (int x = 0; x < gridSize.x; x++) {
                for (int y = 0; y < diff; y++) {
                    ArrayList<Voxel> newY = Stream.generate(Voxel::new)
                            .limit(gridSize.z).collect(Collectors.toCollection(ArrayList::new));
                    grid.get(x).add(0, newY);
                }
            }
            boundary.lowerBounds.y -= voxelSize * diff;
        }
        else if (range.upperBounds().y > this.boundary.upperBounds.y) { // expand upwards
            DPos3 gridSize = size();
            int diff = (int) Math.ceil(range.upperBounds().y - this.boundary.upperBounds.y);
            for (int x = 0; x < gridSize.x; x++) {
                for (int y = 0; y < diff; y++) {
                    ArrayList<Voxel> newY = Stream.generate(Voxel::new)
                            .limit(gridSize.z).collect(Collectors.toCollection(ArrayList::new));
                    grid.get(x).add(newY);
                }
            }
            boundary.upperBounds.y += voxelSize * diff;
        }
        if (range.pos().z < this.boundary.lowerBounds.z) { // expand to the front
            DPos3 gridSize = size();
            int diff = (int) Math.ceil(this.boundary.lowerBounds.z - range.pos().z);
            for (int x = 0; x < gridSize.x; x++) {
                for (int y = 0; y < gridSize.y; y++) {
                    for (int z = 0; z < diff; z++) {
//                        float distance = Vec3.dist(pos, new Vec3(x * voxelSize + boundary.position.x,
//                                y * voxelSize + boundary.position.y,
//                                boundary.position.z - z * voxelSize));
                        grid.get(x).get(y).add(0, new Voxel());
                    }
                }
            }
            boundary.lowerBounds.z -= voxelSize * diff;
        }
        else if (range.upperBounds().z > this.boundary.upperBounds.z) { // expand to the back
            DPos3 gridSize = size();
            int diff = (int) Math.ceil(range.upperBounds().z - this.boundary.upperBounds.z);
            for (int x = 0; x < gridSize.x; x++) {
                for (int y = 0; y < gridSize.y; y++) {
                    for (int z = 0; z < diff; z++) {
//                        float distance = Vec3.dist(pos, new Vec3(x * voxelSize + boundary.position.x,
//                                y * voxelSize + boundary.position.y,
//                                (gridSize.z - 1 + z) * voxelSize + boundary.position.z));
                        grid.get(x).get(y).add(new Voxel());
                    }
                }
            }
            boundary.upperBounds.z += voxelSize * diff;
        }
    }

    /**
     * Checks if the voxel is outside the grid and if so, expands the grid to contain the voxel
     * @param voxel: The voxel position that is or is not yet in the grid
     * @param pos: The agent position (unused with current implementation)
     * @param observation_radius: speaks for itself
     * @return X,Y,Z amount it expanded towards minus (used to add to subsequent calls that do not re-evaluate the gridSize)
     */
    public DPos3 checkAndExpand2(DPos3 voxel, Vec3 pos, float observation_radius) {
        var retVal = new DPos3(0, 0, 0);
        DPos3 gridSize = size();

        if (voxel.x < 0) {
            int diff = -1 * voxel.x;
            for (int x = 0; x < diff; x++) {
//                ArrayList<Voxel> temp = Stream.generate(Voxel::new)
//                        .limit(diff).collect(Collectors.toCollection(ArrayList::new));
                ArrayList<ArrayList<Voxel>> newX = Stream.generate(() -> Stream.generate(Voxel::new)
                                .limit(gridSize.z)
                                .collect(Collectors.toCollection(ArrayList::new)))
                        .limit(gridSize.y)
                        .collect(Collectors.toCollection(ArrayList::new));
                grid.add(0, newX);
                boundary.lowerBounds.x -= voxelSize;
                retVal.x += diff;
            }
        }
        else if (voxel.x >= gridSize.x) {
            int diff = voxel.x - (gridSize.x-1);
            for (int x = 0; x < diff; x++) {
                ArrayList<ArrayList<Voxel>> newX = Stream.generate(() -> Stream.generate(Voxel::new)
                                .limit(gridSize.z)
                                .collect(Collectors.toCollection(ArrayList::new)))
                        .limit(gridSize.y)
                        .collect(Collectors.toCollection(ArrayList::new));
                grid.add(newX);
            }
        }
        if (voxel.y < 0) {
            int diff = -1 * voxel.y;
            for (int x = 0; x < gridSize.x; x++) {
                for (int y = 0; y < diff; y++) {
                    ArrayList<Voxel> newY = Stream.generate(Voxel::new)
                            .limit(gridSize.z).collect(Collectors.toCollection(ArrayList::new));
                    grid.get(x).add(0, newY);
                }
            }
            boundary.lowerBounds.y -= voxelSize;
            retVal.y++;
        }
        else if (voxel.y >= gridSize.y) {
            int diff = voxel.y - (gridSize.y-1);
            for (int x = 0; x < gridSize.x; x++) {
                for (int y = 0; y < diff; y++) {
                    ArrayList<Voxel> newY = Stream.generate(Voxel::new)
                            .limit(gridSize.z).collect(Collectors.toCollection(ArrayList::new));
                    grid.get(x).add(newY);
                }
            }
        }
        if (voxel.z < 0) {
            int diff = -1 * voxel.z;
            for (int x = 0; x < gridSize.x; x++) {
                for (int y = 0; y < gridSize.y; y++) {
                    for (int z = 0; z < diff; z++) {
//                        float distance = Vec3.dist(pos, new Vec3(x * voxelSize + boundary.position.x,
//                                y * voxelSize + boundary.position.y,
//                                boundary.position.z - z * voxelSize));
                        grid.get(x).get(y).add(0, new Voxel());
                    }
                }
            }
            boundary.lowerBounds.z -= voxelSize;
            retVal.z++;
        }
        else if (voxel.z >= gridSize.z) {
            int diff = voxel.z - (gridSize.z-1);
            for (int x = 0; x < gridSize.x; x++) {
                for (int y = 0; y < gridSize.y; y++) {
                    for (int z = 0; z < diff; z++) {
//                        float distance = Vec3.dist(pos, new Vec3(x * voxelSize + boundary.position.x,
//                                y * voxelSize + boundary.position.y,
//                                (gridSize.z - 1 + z) * voxelSize + boundary.position.z));
                        grid.get(x).get(y).add(new Voxel());
                    }
                }
            }
        }
        return retVal;
    }

    @Override
    public Iterable<DPos3> neighbours(DPos3 p) {
        List<DPos3> candidates = new LinkedList<>() ;

        for (int x = max(p.x-1, 0); x <= min(p.x+1, size().x-1); x++) {
            for (int y = max(p.y-1, 0); y <= min(p.y+1, size().y-1); y++) {
                for (int z = max(p.z-1, 0); z <= min(p.z+1, size().z-1); z++) {

                    if(x==p.x && y==p.y && z==p.z) continue;
                    var neighbourCube = new DPos3(x,y,z) ; // a neighbouring cube

                    if(get(x,y,z).label == Label.OPEN)
                        candidates.add(neighbourCube) ;
                }
            }
        }

        return candidates ;
    }

    @Override
    public Iterable<DPos3> neighbours_explore(DPos3 p) {
        List<DPos3> candidates = new LinkedList<>() ;

        for (int x = max(p.x-1, 0); x <= min(p.x+1, size().x-1); x++) {
            for (int y = max(p.y-1, 0); y <= min(p.y+1, size().y-1); y++) {
                for (int z = max(p.z-1, 0); z <= min(p.z+1, size().z-1); z++) {
                    // If the neighbour is outside the grid, add it to the neighbours
                    // Note: do NOT use outside of explore()
                    if (x < 0 || y < 0 || z < 0 || x >= size().x || y >= size().y || z >= size().z) {
                        candidates.add(new DPos3(x,y,z));
                        continue;
                    }

                    if(x==p.x && y==p.y && z==p.z) continue;
                    var neighbourCube = new DPos3(x,y,z) ; // a neighbouring cube

                    if(get(x,y,z).label == Label.OPEN || get(x,y,z).label == Label.UNKNOWN)
                        candidates.add(neighbourCube) ;
                }
            }
        }

        return candidates ;
    }

    @Override
    public float heuristic(DPos3 from, DPos3 to) {
        // using Manhattan distance...
        // return voxelSize * (float) (Math.abs(to.x - from.x) + Math.abs(to.y - from.y) + Math.abs(to.z - from.z)) ;

        // using Euclidean distance...
        return voxelSize * (float) Math.sqrt((to.x - from.x) * (to.x - from.x) + (to.y - from.y) * (to.y - from.y) + (to.z - from.z) * (to.z - from.z)) ;
    }

    @Override
    public float distance(DPos3 from, DPos3 to) {
        if (from.x != to.x && from.y != to.y && from.z != to.z) {
            return cubeDiagonalLength ;
        }
        if(from.x == to.x) {
            if (from.y == to.y || from.z == to.z) {
                return voxelSize;
            }
            return squareDiagonalLength ;
        }
        if(from.y == to.y) {
            if (from.x == to.x || from.z == to.z) {
                return voxelSize;
            }
            return squareDiagonalLength ;
        }
        return squareDiagonalLength ;
    }

    @Override
    public DPos3 phantom_neighbour(DPos3 node, DPos3 real_neighbour) {
        return real_neighbour;
    }

    @Override
    public boolean isUnknown(DPos3 node) {
        if (node.x < 0 || node.y < 0 || node.z < 0 || node.x >= size().x || node.y >= size().y || node.z >= size().z)
            return true; // if outside the grid, it must be unknown
        return get(node).label == Label.UNKNOWN;
    }

    @Override
    public void updateUnknown(Vec3 pos, float observation_radius) {
        Vec3 minCorner = Vec3.sub(pos, new Vec3(observation_radius));
        Vec3 maxCorner = Vec3.add(pos, new Vec3(observation_radius));
        var corner1 = gridProjectedLocation(minCorner) ;
        var corner2 = gridProjectedLocation(maxCorner) ;

        for(int x = max(corner1.x, 0); x <= min(corner2.x, size().x-1); x++) {
            for (int y = max(corner1.y, 0); y <= min(corner2.y, size().y-1); y++) {
                for (int z = max(corner1.z, 0); z <= min(corner2.z, size().z-1); z++) {
                    float distance = Vec3.dist(pos, new Vec3(x * voxelSize + boundary.lowerBounds.x,
                            y * voxelSize + boundary.lowerBounds.y,
                            z * voxelSize + boundary.lowerBounds.z));
                    if (get(x, y, z).label == Label.UNKNOWN && distance <= observation_radius) {
                        get(x, y, z).label = Label.OPEN;
                    }
                }
            }
        }
    }
}
