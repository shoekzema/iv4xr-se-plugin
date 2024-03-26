package uuspaceagent;

import eu.iv4xr.framework.extensions.pathfinding.Navigatable;
import eu.iv4xr.framework.mainConcepts.WorldEntity;
import eu.iv4xr.framework.spatial.Vec3;
import uuspaceagent.exploration.Explorable;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

public class VoxelGrid implements Explorable<DPos3> {

    /**
     * The assumed height of the player characters. It is 1.8, we conservatively
     * assume it is 2f.
     */
    public static float AGENT_HEIGHT = 2f ;
    public static float AGENT_WIDTH  = 1f ;

    public Boundary boundary;
    public float voxelSize;
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
    }

    /**
     *  Initializes the starting grid based on the first observed position and the observation radius.
     *  Call in UUSeAgentstate3D on first observation.
     */
    public void initializeGrid(Vec3 pos, float observation_radius) {
        // radius +10   so that the agent can move a little before having to rebuild the whole grid (expand)
        Vec3 lowerB = Vec3.sub(pos, new Vec3(observation_radius + 10));
        boundary = new Boundary(lowerB, 2 * (observation_radius + 10));

        int initialSize = (int) ((boundary.upperBounds.x - boundary.lowerBounds.x) / voxelSize);
        grid = new ArrayList<>(initialSize);
        for (int x = 0; x < initialSize; x++) {
            grid.add(x, new ArrayList<>(initialSize));
            for (int y = 0; y < initialSize; y++) {
                grid.get(x).add(y, new ArrayList<>(initialSize));
                for (int z = 0; z < initialSize; z++) {
                    grid.get(x).get(y).add(z, new Voxel());
                }
            }
        }
//        grid = new ArrayList<>(Collections.nCopies(initialSize,
//                    new ArrayList<>(Collections.nCopies(initialSize,
//                        new ArrayList<>(Collections.nCopies(initialSize, (byte)0)))
//                    )
//               ));
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


        // TODO: a more general approach for 3D. So not assuming y is the up-axis
        // add some padding due to agent's body width:
//        Vec3 hpadding = Vec3.mul(new Vec3(AGENT_WIDTH, 0, AGENT_WIDTH), 0.6f) ;
//        Vec3 vpadding = new Vec3(0, AGENT_HEIGHT, 0) ;
//        minCorner = Vec3.sub(minCorner, hpadding) ;
//        minCorner = Vec3.sub(minCorner, vpadding) ;
//        maxCorner = Vec3.add(maxCorner, hpadding) ;
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
        DPos3 min = new DPos3(0, 0, 0);

        for(var voxel : obstructedCubes) {
            DPos3 newMin = checkAndExpand(DPos3.add(voxel, min));
            min.x = Math.max(min.x, newMin.x);
            min.y = Math.max(min.y, newMin.y);
            min.z = Math.max(min.z, newMin.z);

            get(DPos3.add(voxel, min)).label = Label.BLOCKED;
        }
    }

    public void removeObstacle(WorldEntity block) {

        var obstructedCubes = getObstructedCubes(block) ;
        for(var voxel : obstructedCubes) {
            get(voxel).label = Label.OPEN;
        }
    }

    /**
     * Checks if the voxel is outside the grid and if so, expands the grid to contain the voxel
     * @param voxel
     */
    public DPos3 checkAndExpand(DPos3 voxel) {
        var retVal = new DPos3(0, 0, 0);
        DPos3 gridSize = size();

        while (voxel.x < 0) {
            grid.add(0, new ArrayList<>(gridSize.y));
            for (int y = 0; y < gridSize.y; y++) {
                grid.get(0).add(y, new ArrayList<>(gridSize.z));
                for (int z = 0; z < gridSize.z; z++) {
                    grid.get(0).get(y).add(z, new Voxel());
                }
            }
            voxel.x++;
            boundary.lowerBounds.x -= voxelSize;
            retVal.x++;
        }
        while (voxel.x >= gridSize.x) {
            grid.add(new ArrayList<>(gridSize.y));
            for (int y = 0; y < gridSize.y; y++) {
                grid.get(grid.size()-1).add(y, new ArrayList<>(gridSize.z));
                for (int z = 0; z < gridSize.z; z++) {
                    grid.get(gridSize.x-1).get(y).add(z, new Voxel());
                }
            }
            boundary.upperBounds.x += voxelSize;
        }
        while (voxel.y < 0) {
            for (int x = 0; x < gridSize.x; x++) {
                grid.get(x).add(0, new ArrayList<>(gridSize.z));
                for (int z = 0; z < gridSize.z; z++) {
                    grid.get(x).get(0).add(z, new Voxel());
                }
            }
            voxel.y++;
            boundary.lowerBounds.y -= voxelSize;
            retVal.y++;
        }
        while (voxel.y >= gridSize.y) {
            for (int x = 0; x < gridSize.x; x++) {
                grid.get(x).add(new ArrayList<>(gridSize.z));
                for (int z = 0; z < gridSize.z; z++) {
                    grid.get(x).get(gridSize.y-1).add(z, new Voxel());
                }
            }
            boundary.upperBounds.z += voxelSize;
        }
        while (voxel.z < 0) {
            for (int x = 0; x < gridSize.x; x++) {
                for (int y = 0; y < gridSize.y; y++) {
                    grid.get(x).get(y).add(0, new Voxel());
                }
            }
            voxel.z++;
            boundary.lowerBounds.z -= voxelSize;
            retVal.z++;
        }
        while (voxel.z >= gridSize.z) {
            for (int x = 0; x < gridSize.x; x++) {
                for (int y = 0; y < gridSize.y; y++) {
                    grid.get(x).get(y).add(new Voxel());
                }
            }
            boundary.upperBounds.z += voxelSize;
        }
        return retVal;
    }

    @Override
    public Iterable<DPos3> neighbours(DPos3 p) {
        List<DPos3> candidates = new LinkedList<>() ;

        for (int x = p.x-1 ; x <= p.x+1 ; x++) {
            for (int y = p.y-1 ; y <= p.y+1 ; y++) {
                for (int z = p.z-1; z <= p.z+1 ; z++) {
                    if(x==p.x && y==p.y && z==p.z) continue;
                    var neighbourCube = new DPos3(x,y,z) ; // a neighbouring cube
                    if(get(x,y,z).label != Label.BLOCKED)
                        candidates.add(neighbourCube) ;
                }
            }
        }

        return candidates ;
    }

    @Override
    public float heuristic(DPos3 from, DPos3 to) {
        // using Manhattan distance...
        return voxelSize * (float) (Math.abs(to.x - from.x) + Math.abs(to.y - from.y) + Math.abs(to.z - from.z)) ;
    }

    @Override
    public float distance(DPos3 from, DPos3 to) {
        return voxelSize;
    }

    @Override
    public boolean isUnknown(DPos3 node) {
        return false;
    }
}
