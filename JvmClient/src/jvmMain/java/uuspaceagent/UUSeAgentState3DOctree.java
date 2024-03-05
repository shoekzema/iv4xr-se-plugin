package uuspaceagent;

import environments.SeEnvironmentKt;
import eu.iv4xr.framework.extensions.pathfinding.Navigatable;
import eu.iv4xr.framework.mainConcepts.WorldEntity;
import eu.iv4xr.framework.mainConcepts.WorldModel;
import eu.iv4xr.framework.spatial.Vec3;
import spaceEngineers.model.CharacterObservation;
import spaceEngineers.model.Observation;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

public class UUSeAgentState3DOctree extends UUSeAgentState<Octree> {

    float OBSERVATION_RADIUS = 50.0f;
    boolean printed = false;

    public Octree grid = new Octree(null, null, 0, Label.UNKNOWN) ;

    public UUSeAgentState3DOctree(String agentId) {
        super(agentId);
    }

    @Override
    public Octree getGridPos(Vec3 targetLocation) {
        return grid.gridProjectedLocation(targetLocation);
    }

    @Override
    public Vec3 getBlockCenter(Vec3 targetLocation) {
        return grid.getCubeCenterLocation(grid.gridProjectedLocation(targetLocation));
    }

    @Override
    public Vec3 getBlockCenter(Octree targetLocation) {
        return grid.getCubeCenterLocation(targetLocation);
    }

    @Override
    public Vec3 getOrigin() {
        return grid.boundary.lowerBounds;
    }

    @Override
    public Navigatable<Octree> getGrid() {
        return grid;
    }

    @Override
    public void updateState(String agentId) {

        super.updateState(agentId);

        // get the new WOM. Currently, it does not include agent's extended properties, so we add them
        // explicitly here:
        WorldModel newWom = env().observe() ;
        //System.out.println(">>>-- agent pos as received from SE:" + newWom.position);
        // HACK: SE gives generated-id to the agent, replace that:
        newWom.agentId = this.agentId ;
        // HACK: because wom that comes from SE has its wom.elements read-only :|
        var origElements = newWom.elements ;
        newWom.elements = new HashMap<>() ;
        for (var e : origElements.entrySet()) newWom.elements.put(e.getKey(),e.getValue()) ;
        CharacterObservation agentObs = env().getController().getObserver().observe() ;
        newWom.elements.put(this.agentId, agentAdditionalInfo(agentObs)) ;
        assignTimeStamp(newWom, updateCount) ;

        // The obtained wom also does not include blocks observed. So we get them explicitly here:
        // Well, we will get ALL blocks. Note that S=some blocks may change state or disappear,
        // compared to what the agent currently has it its state.wom.
        Observation rawGridsAndBlocksStates = env().getController().getObserver().observeBlocks() ;
        WorldModel gridsAndBlocksStates = SeEnvironmentKt.toWorldModel(rawGridsAndBlocksStates) ;
        // HACK: make the grids and blocks marked as dynamic elements. SE sends them as non-dymanic
        // that will cause them to be ignored by mergeObservation.
        SEBlockFunctions.hackForceDynamicFlag(gridsAndBlocksStates) ;
        // assign a fresh timestamp too:
        assignTimeStamp(gridsAndBlocksStates, updateCount) ;
        for(var e : gridsAndBlocksStates.elements.entrySet()) {
            newWom.elements.put(e.getKey(), e.getValue()) ;
        }

//        newWom.elements.forEach((k, v) -> {
//            if (!Objects.equals(k, "se0")) {
//                v.elements.forEach((k2, v2) -> {
//                    System.out.println((String) v2.properties.get("blockType") + ": " + (int)(v2.position.x) + " " + (int)(v2.position.y) + " " + (int)(v2.position.z));
//                });
//            }
//        });

        // updating the count:
        updateCount++ ;

        if(wom == null) {
            // this is the first observation
            wom = newWom ;
            grid.initializeGrid(wom.position, OBSERVATION_RADIUS);
        }
        else {
            // MERGING the two woms:
            wom.mergeNewObservation(newWom) ;
            // merges the woms, but cannot easily be used for exploration because everything outside the viewing distance
            // is thrown away out of the wom

            System.out.println("========================================================");

            List<String> tobeRemoved = wom.elements.keySet().stream()
                    .filter(id -> ! newWom.elements.keySet().contains(id))
                    .collect(Collectors.toList());
            for(var id : tobeRemoved) wom.elements.remove(id) ;
            // Then, we remove disappearing blocks (from grids that remain):
            for(var cubegridOld : wom.elements.values()) {
                var cubeGridNew = newWom.elements.get(cubegridOld.id) ;
                tobeRemoved.clear();
                tobeRemoved = cubegridOld.elements.keySet().stream()
                        .filter(blockId -> ! cubeGridNew.elements.keySet().contains(blockId))
                        .collect(Collectors.toList());
                for(var blockId : tobeRemoved) cubegridOld.elements.remove(blockId) ;
            }
        }

//        for(var block : SEBlockFunctions.getAllBlocks(gridsAndBlocksStates)) {
//            grid.addObstacle(block);
//        }
        Boundary observation_radius = new Boundary(Vec3.sub(wom.position, new Vec3(OBSERVATION_RADIUS * 0.5f)), OBSERVATION_RADIUS);
        grid.update(SEBlockFunctions.getAllBlocks(gridsAndBlocksStates), observation_radius);

//        // then, there may also be new blocks ... we add them to the nav-grid:
//        // TODO: this assumes doors are initially closed. Calculating blocked squares
//        // for open-doors is more complicated. TODO.
//        for(var block : SEBlockFunctions.getAllBlocks(gridsAndBlocksStates)) {
//            navgrid.addObstacle(block);
//            // check if it is a door, and get its open/close state:
//            Boolean isOpen = SEBlockFunctions.geSlideDoorState(block) ;
//            if (isOpen != null) {
//                navgrid.setObstacleBlockingState(block, !isOpen);
//            }
//            // check if it is a button panel, and make it not blocking
//            if(block.type.equals("block"))
//                if (block.getStringProperty("blockType").contains("ButtonPanel"))
//                    navgrid.setObstacleBlockingState(block, false);
//        }
//        // updating dynamic blocking-state: (e.g. handling doors)
//        // TODO!


        if (!printed) { exportGrid(); printed = true; }
    }

    public void exportManualProfileShit() { // TODO: fix

        int gridMemSize = 1; // in bytes

        int womMemSize = 0;
        for (Map.Entry<String, WorldEntity> entry : wom.elements.entrySet()) {
            womMemSize += entry.getValue().elements.size() * 325; // 325 is a calculated estimation of amount of bytes used per SE block
        }

        System.out.println("Memory Usage:");
        if (gridMemSize > 1000000)
            System.out.printf("Grid: %f GB %n", (float)gridMemSize / 1000000);
        else if (gridMemSize > 1000)
            System.out.printf("Grid: %f MB %n", (float)gridMemSize / 1000);
        else
            System.out.printf("Grid: %d B %n", gridMemSize);

        if (womMemSize > 1000000)
            System.out.printf("WOM: %f GB %n", (float)womMemSize / 1000000);
        else if (womMemSize > 1000)
            System.out.printf("WOM: %f MB %n", (float)womMemSize / 1000);
        else
            System.out.printf("WOM: %d B %n", womMemSize);
    }

    public void exportGrid() {

        Observation rawGridsAndBlocksStates = env().getController().getObserver().observeBlocks() ;
        WorldModel gridsAndBlocksStates = SeEnvironmentKt.toWorldModel(rawGridsAndBlocksStates) ;

//        Vec3 doorpos = new Vec3(0);
//        for(var block : SEBlockFunctions.getAllBlocks(gridsAndBlocksStates)) {
//            if (SEBlockFunctions.geSlideDoorState(block) != null) {
//                doorpos = block.position;
//            }
//        }
        Boundary observation_radius = new Boundary(Vec3.sub(wom.position, new Vec3(OBSERVATION_RADIUS * 0.5f)), OBSERVATION_RADIUS);
        grid.update(SEBlockFunctions.getAllBlocks(gridsAndBlocksStates), observation_radius);

        try {
            System.out.println(System.getProperty("user.dir"));
            FileWriter fileWriter = new FileWriter("Octree_visualization.txt");
            PrintWriter printWriter = new PrintWriter(fileWriter);
//            Vec3 player_pos = grid.getCubeCenterLocation(grid.gridProjectedLocation(new Vec3(wom.position.x, wom.position.y + grid.AGENT_HEIGHT * 0.5f, wom.position.z)));
//            Vec3 door_pos = grid.getCubeCenterLocation(grid.gridProjectedLocation(doorpos));
//            printWriter.printf("player: %f %f %f %n", player_pos.x, player_pos.y, player_pos.z);
//            printWriter.printf("door: %f %f %f %n", door_pos.x, door_pos.y, door_pos.z);
            grid.export(printWriter);

            printWriter.close();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}
