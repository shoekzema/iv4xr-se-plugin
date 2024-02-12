package uuspaceagent;

import environments.SeEnvironmentKt;
import eu.iv4xr.framework.extensions.pathfinding.Navigatable;
import eu.iv4xr.framework.mainConcepts.WorldModel;
import eu.iv4xr.framework.spatial.Vec3;
import spaceEngineers.model.CharacterObservation;
import spaceEngineers.model.Observation;

import java.util.HashMap;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;

public class UUSeAgentState3D extends UUSeAgentState {

    public VoxelGrid grid = new VoxelGrid(new Boundary(new Vec3(-50), 100), 0.5f) ;

    public UUSeAgentState3D(String agentId) {
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
        return grid.boundary.lowerBounds;
    }

    @Override
    public Navigatable<DPos3> getGrid() {
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
        assignTimeStamp(newWom,updateCount) ;

        // The obtained wom also does not include blocks observed. So we get them explicitly here:
        // Well, we will get ALL blocks. Note that S=some blocks may change state or disappear,
        // compared to what the agent currently has it its state.wom.
        Observation rawGridsAndBlocksStates = env().getController().getObserver().observeBlocks() ;
        WorldModel gridsAndBlocksStates = SeEnvironmentKt.toWorldModel(rawGridsAndBlocksStates) ;
        // HACK: make the grids and blocks marked as dynamic elements. SE sends them as non-dymanic
        // that will cause them to be ignored by mergeObservation.
        SEBlockFunctions.hackForceDynamicFlag(gridsAndBlocksStates) ;
        // assign a fresh timestamp too:
        assignTimeStamp(gridsAndBlocksStates,updateCount) ;
        for(var e : gridsAndBlocksStates.elements.entrySet()) {
            newWom.elements.put(e.getKey(), e.getValue()) ;
        }

        newWom.elements.forEach((k, v) -> {
            if (!Objects.equals(k, "se0")) {
                v.elements.forEach((k2, v2) -> {
                    System.out.println((String) v2.properties.get("blockType") + ": " + (int)(v2.position.x) + " " + (int)(v2.position.y) + " " + (int)(v2.position.z));
                });
            }
        });

        // updating the count:
        updateCount++ ;

        if(wom == null) {
            // this is the first observation
            wom = newWom ;
        }
        else {
            // MERGING the two woms:
            wom.mergeNewObservation(newWom) ;
            // merges the woms, but cannot easily be used for exploration because everything outside the viewing distance
            // is thrown away out of the wom

//            wom.elements.forEach((key, value) -> {
//                System.out.println(value.elements.size());
//                value.elements.forEach((key2, value2) -> {
//                    System.out.println(value2.type + ": " + value2.position);
//                });
//            });
            System.out.println("========================================================");

            // HOWEVER, some blocks and grids-of-blocks may have been destroyed, hence
            // do not exist anymore. We need to remove them from state.wom. This is handled
            // below.
            // First, remove disappearing "cube-grids" (composition of blocks)
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

//            // updating the "navigational-2DGrid:
//            var blocksInWom =  SEBlockFunctions.getAllBlockIDs(wom) ;
//            List<String> toBeRemoved = navgrid.allObstacleIDs.stream()
//                    .filter(id -> !blocksInWom.contains(id))
//                    .collect(Collectors.toList());
//            // first, removing obstacles that no longer exist:
//            for(var id : toBeRemoved) {
//                navgrid.removeObstacle(id);
//            }
        }

        for(var block : SEBlockFunctions.getAllBlocks(gridsAndBlocksStates)) {
            grid.addObstacle(block);
        }

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
    }
}
