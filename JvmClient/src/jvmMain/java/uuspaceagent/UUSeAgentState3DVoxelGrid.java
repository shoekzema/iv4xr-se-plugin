package uuspaceagent;

import environments.SeEnvironmentKt;
import eu.iv4xr.framework.mainConcepts.WorldEntity;
import eu.iv4xr.framework.mainConcepts.WorldModel;
import eu.iv4xr.framework.spatial.Vec3;
import spaceEngineers.model.CharacterObservation;
import spaceEngineers.model.Observation;
import uuspaceagent.exploration.Explorable;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

public class UUSeAgentState3DVoxelGrid extends UUSeAgentState<DPos3> {

    public VoxelGrid grid = new VoxelGrid(1) ;

    public UUSeAgentState3DVoxelGrid(String agentId) {
        super(agentId);
    }

    @Override
    public DPos3 getGridPos(Vec3 targetLocation) {
        return grid.gridProjectedLocation(targetLocation);
    }

    @Override
    public Vec3 getBlockCenter(Vec3 targetLocation) {
        return grid.getCubeCenterLocation(grid.gridProjectedLocation(targetLocation));
    }

    @Override
    public Vec3 getBlockCenter(DPos3 targetLocation) {
        return grid.getCubeCenterLocation(targetLocation);
    }

    @Override
    public Vec3 getOrigin() {
        return grid.boundary.position;
    }

    @Override
    public Explorable<DPos3> getGrid() {
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

        // updating the count:
        updateCount++ ;

        if(wom == null) {
            // this is the first observation
            wom = newWom ;
            grid.initializeGrid(wom.position, OBSERVATION_RADIUS);

            for(var block : SEBlockFunctions.getAllBlocks(gridsAndBlocksStates)) {
                addToGrid(block, wom.position);
            }
        }
        else {
            // MERGING the two woms:
            var changes = wom.mergeNewObservation(newWom) ;
            // merges the woms, but cannot easily be used for exploration because everything outside the viewing distance
            // is thrown away out of the wom

            System.out.println("========================================================");

            // Remove grids that are not in the WOM anymore
            List<String> tobeRemoved = wom.elements.keySet().stream()
                    .filter(id -> ! newWom.elements.containsKey(id))
                    .collect(Collectors.toList());
            for(var id : tobeRemoved) wom.elements.remove(id) ;

            for(var cubeGridNew : changes) {
                var cubegridOld = cubeGridNew.getPreviousState();

                // Then, we remove disappearing blocks (from grids that changed):
                tobeRemoved.clear();
                tobeRemoved = cubegridOld.elements.keySet().stream()
                        .filter(blockId -> !cubeGridNew.elements.containsKey(blockId))
                        .collect(Collectors.toList());

                boolean rebuild = false;
                for (var blockId : tobeRemoved) {
                    var block = cubegridOld.elements.get(blockId);
                    if (Vec3.dist(block.position, newWom.position) < OBSERVATION_RADIUS) {
                        grid.removeObstacle(block);
                        rebuild = true;

                        // Removing a block makes all voxels it overlaps with empty, so go over all blocks to check
                        // if some voxels overlapped with multiple blocks.
                        for (var block2 : SEBlockFunctions.getAllBlocks(gridsAndBlocksStates)) {
                            addToGrid(block2, newWom.position);
                        }
                    }
                }
                if (!rebuild) {
                    // We add new blocks (from grids that changed):
                    List<String> tobeAdded = cubeGridNew.elements.keySet().stream()
                            .filter(id -> !cubegridOld.elements.containsKey(id))
                            .collect(Collectors.toList());
                    for (var blockId : tobeAdded) {
                        addToGrid(cubeGridNew.elements.get(blockId), newWom.position);
                    }
                }
            }
        }
    }

    public void addToGrid(WorldEntity block, Vec3 pos) {
        Boolean isOpen = SEBlockFunctions.getSlideDoorState(block) ;
        // open-doors are more complicated. TODO.
        if (isOpen != null) {
            if (!isOpen)
                grid.addObstacle(block, pos, OBSERVATION_RADIUS);
        }
        else if(block.getStringProperty("blockType").contains("ButtonPanel")) {
            var test = 1;
        }
        else
            grid.addObstacle(block, pos, OBSERVATION_RADIUS);
    }

    public void exportManualProfileShit() {

        DPos3 size = grid.size();
        int gridMemSize = size.x * size.y * size.z; // in bytes

        int womMemSize = 0;
        for (Map.Entry<String, WorldEntity> entry : wom.elements.entrySet()) {
            womMemSize += entry.getValue().elements.size() * 325; // 325 is a calculated estimation of amount of bytes used per SE block
        }

        System.out.println("Memory Usage:");
        if (gridMemSize > 1000000)
            System.out.printf("Grid: %f MB %n", (float)gridMemSize / 1000000);
        else if (gridMemSize > 1000)
            System.out.printf("Grid: %f kB %n", (float)gridMemSize / 1000);
        else
            System.out.printf("Grid: %d B %n", gridMemSize);

        if (womMemSize > 1000000)
            System.out.printf("WOM: %f MB %n", (float)womMemSize / 1000000);
        else if (womMemSize > 1000)
            System.out.printf("WOM: %f kB %n", (float)womMemSize / 1000);
        else
            System.out.printf("WOM: %d B %n", (womMemSize));
    }

    public void exportGrid() {

        WorldModel wom = env().observe() ;

        Observation rawGridsAndBlocksStates = env().getController().getObserver().observeBlocks() ;
        WorldModel gridsAndBlocksStates = SeEnvironmentKt.toWorldModel(rawGridsAndBlocksStates) ;

        Vec3 doorpos = new Vec3(0);
        for(var block : SEBlockFunctions.getAllBlocks(gridsAndBlocksStates)) {
            grid.addObstacle(block, wom.position, OBSERVATION_RADIUS);
            if (SEBlockFunctions.getSlideDoorState(block) != null) {
                doorpos = block.position;
            }
        }

        try {
            System.out.println(System.getProperty("user.dir"));
            FileWriter fileWriter = new FileWriter("3D_Internal_WOM.txt");
            PrintWriter printWriter = new PrintWriter(fileWriter);
            Vec3 player_pos = grid.getCubeCenterLocation(grid.gridProjectedLocation(new Vec3(wom.position.x, wom.position.y + VoxelGrid.AGENT_HEIGHT * 0.5f, wom.position.z)));
            Vec3 door_pos = grid.getCubeCenterLocation(grid.gridProjectedLocation(doorpos));
            printWriter.printf("player: %f %f %f %n", player_pos.x, player_pos.y, player_pos.z);
            printWriter.printf("door: %f %f %f %n", door_pos.x, door_pos.y, door_pos.z);
            for (int x = 0; x < grid.grid.size(); x++) {
                for (int y = 0; y < grid.grid.get(x).size(); y++) {
                    for (int z = 0; z < grid.grid.get(x).get(y).size(); z++) {
                        if (grid.get(x, y, z).label == Label.UNKNOWN) {
                            Vec3 block_pos = grid.getCubeCenterLocation(new DPos3(x, y, z));
                            printWriter.printf("%f %f %f %n", block_pos.x, block_pos.y, block_pos.z);
                        }
                    }
                }
            }
            printWriter.close();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}
