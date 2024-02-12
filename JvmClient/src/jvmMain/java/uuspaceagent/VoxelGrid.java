package uuspaceagent;

import eu.iv4xr.framework.extensions.pathfinding.Navigatable;
import eu.iv4xr.framework.mainConcepts.WorldEntity;
import eu.iv4xr.framework.spatial.Vec3;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

public class VoxelGrid implements Navigatable<DPos3> {

    /**
     * The assumed height of the player characters. It is 1.8, we conservatively
     * assume it is 2f.
     */
    public static float AGENT_HEIGHT = 2f ;
    public static float AGENT_WIDTH  = 1f ;
    public static float CUBE_SIZE = 0.5f ;

    public Boundary boundary;
    public float voxelSize;
    ArrayList<ArrayList<ArrayList<Voxel>>> grid;

    public VoxelGrid(Boundary boundary, float voxelSize) {
//        grid = new ArrayList<>(Collections.nCopies(initialSize,
//                    new ArrayList<>(Collections.nCopies(initialSize,
//                        new ArrayList<>(Collections.nCopies(initialSize, (byte)0)))
//                    )
//               ));
        this.boundary = boundary;
        this.voxelSize = voxelSize;
        int initialSize = (int) ((boundary.upperBounds.x - boundary.lowerBounds.x) / voxelSize);

        grid = new ArrayList<>(initialSize);
        for (int x = 0; x < initialSize; x++) {
            int xpos = (int) (boundary.lowerBounds.x + x * voxelSize);
            grid.add(x, new ArrayList<>(initialSize));
            for (int y = 0; y < initialSize; y++) {
                int ypos = (int) (boundary.lowerBounds.y + y * voxelSize);
                grid.get(x).add(y, new ArrayList<>(initialSize));
                for (int z = 0; z < initialSize; z++) {
                    int zpos = (int) (boundary.lowerBounds.z + z * voxelSize);
                    grid.get(x).get(y).add(z, new Voxel(new DPos3(xpos, ypos, zpos)));
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
        int x = myFloor(p.x / CUBE_SIZE) ;
        int y = myFloor(p.y / CUBE_SIZE) ;
        int z = myFloor(p.z / CUBE_SIZE) ;

        return new DPos3(x,y,z) ;
    }

    /**
     * Return the actual location of the center of a unit cube; the location is expressed
     * as a 3D position in the space.
     */
    public Vec3 getCubeCenterLocation(DPos3 cube) {
        float x = (((float) cube.x) + 0.5f) * CUBE_SIZE + boundary.lowerBounds.x ;
        float y = (((float) cube.y) + 0.5f) * CUBE_SIZE + boundary.lowerBounds.y ;
        float z = (((float) cube.z) + 0.5f) * CUBE_SIZE + boundary.lowerBounds.z ;
        return new Vec3(x,y,z) ;
    }

    public static int myFloor(float x) {
        return (int) Math.floor(x) ;
    }

    List<DPos3> getObstructedCubes(WorldEntity block) {

        List<DPos3> obstructed = new LinkedList<>() ;

        Vec3 maxCorner = SEBlockFunctions.getBaseMaxCorner(block) ; // should add rotation if it is not a cube. TODO.
        Vec3 minCorner = SEBlockFunctions.getBaseMinCorner(block) ; // should add rotation if it is not a cube. TODO.

        // Check if the block is below the ground surface. If it is, it can obstruct movement on or above
        // the surface.

        // Adding 0.1 offset for some bit of seemingly inaccuracy in the sampling of the agent's
        // ground position, which is used as the base of the origin position of this grid.
        // Without this offset, blocks that are just below the grid surface, and touching it to form
        // the grid's solid floor will appear as obstructing.
        float correction_offset = 0.1f ;
        if(maxCorner.y <= boundary.lowerBounds.y + correction_offset) {
            // the block is UNDER the ground-surface. So, it won't obstruct either.
            return obstructed ;
        }

        // TODO: a more general approach.
        // add some padding due to agent's body width:
        Vec3 hpadding = Vec3.mul(new Vec3(AGENT_WIDTH,0,AGENT_WIDTH), 0.6f) ;
        Vec3 vpadding = new Vec3(0, AGENT_HEIGHT, 0) ;
        minCorner = Vec3.sub(minCorner,hpadding) ;
        minCorner = Vec3.sub(minCorner, vpadding) ;
        maxCorner = Vec3.add(maxCorner,hpadding) ;
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

        var obstructedCubes = getObstructedCubes(block) ;
        for(var voxel : obstructedCubes) {
            grid.get(voxel.x).get(voxel.y).get(voxel.z).label = Label.BLOCKED;
        }
    }

    @Override
    public Iterable<DPos3> neighbours(DPos3 p) {
        List<DPos3> candidates = new LinkedList<>() ;

//            int xi = (int) ((p.x - boundary.lowerBounds.x) / voxelSize);
//            int yi = (int) ((p.y - boundary.lowerBounds.y) / voxelSize);
//            int zi = (int) ((p.z - boundary.lowerBounds.z) / voxelSize);

        if (grid.get(p.x-1).get(p.y).get(p.z).label != Label.BLOCKED)
            candidates.add(new DPos3(p.x-1, p.y, p.z));
        if (grid.get(p.x+1).get(p.y).get(p.z).label != Label.BLOCKED)
            candidates.add(new DPos3(p.x+1, p.y, p.z));

        if (grid.get(p.x).get(p.y-1).get(p.z).label != Label.BLOCKED)
            candidates.add(new DPos3(p.x, p.y-1, p.z));
        if (grid.get(p.x).get(p.y+1).get(p.z).label != Label.BLOCKED)
            candidates.add(new DPos3(p.x, p.y+1, p.z));

        if (grid.get(p.x).get(p.y).get(p.z-1).label != Label.BLOCKED)
            candidates.add(new DPos3(p.x, p.y, p.z-1));
        if (grid.get(p.x).get(p.y).get(p.z+1).label != Label.BLOCKED)
            candidates.add(new DPos3(p.x, p.y, p.z+1));

        return candidates ;
    }

    @Override
    public float heuristic(DPos3 from, DPos3 to) {
        // using Manhattan distance...
        return CUBE_SIZE * (float) (Math.abs(to.x - from.x) + Math.abs(to.y - from.y) + Math.abs(to.z - from.z)) ;
    }

    @Override
    public float distance(DPos3 from, DPos3 to) {
        return CUBE_SIZE;
    }
}
