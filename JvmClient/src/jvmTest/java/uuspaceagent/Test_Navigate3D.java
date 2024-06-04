package uuspaceagent;

import eu.iv4xr.framework.mainConcepts.TestAgent;
import eu.iv4xr.framework.mainConcepts.WorldEntity;
import eu.iv4xr.framework.spatial.Vec3;
import nl.uu.cs.aplib.mainConcepts.GoalStructure;
import nl.uu.cs.aplib.utils.Pair;
import org.junit.jupiter.api.Test;
import spaceEngineers.controller.useobject.UseObjectExtensions;
import spaceEngineers.model.Block;

import java.time.Duration;
import java.time.Instant;

import static nl.uu.cs.aplib.AplibEDSL.*;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static uuspaceagent.PrintInfos.showWOMAgent;
import static uuspaceagent.TestUtils.*;
import static uuspaceagent.UUGoalLib.*;

public class Test_Navigate3D {
    public Pair<TestAgent, UUSeAgentState> deployAgent(String worldname) throws InterruptedException {
        var agentAndState = loadSE3D2(worldname);
        TestAgent agent = agentAndState.fst;
        var state = agentAndState.snd;
        Thread.sleep(1000);
        state.updateState(state.agentId);
        console(showWOMAgent(state.wom));
        return new Pair<>(agent, state);
    }

    public void test_Goal(TestAgent agent, UUSeAgentState state, GoalStructure G) throws InterruptedException {
        agent.setGoal(G);
        Instant start = Instant.now();
        int turn = 0;
        while (G.getStatus().inProgress()) {
            console(">> [" + turn + "] " + showWOMAgent(state.wom));
            agent.update();
            //Thread.sleep(50);
            turn++;
            //if (turn >= 10000) break;
        }
        Instant end = Instant.now();
        long timeElapsed = Duration.between(start, end).toMillis();
        console("!!! total time elapsed (in milliseconds): " + timeElapsed);

        if (state instanceof UUSeAgentState3DOctree) {
            ((UUSeAgentState3DOctree) state).exportGrid();
        } else if (state instanceof UUSeAgentState3DVoxelGrid) {
            ((UUSeAgentState3DVoxelGrid) state).exportGrid();
        }
        TestUtils.closeConnectionToSE(state);
    }

//    @Test
//    public void test_navigate3DTo() throws InterruptedException {
//        console("*** start test...");
//        var agentAndState = deployAgent("myworld-3 3D-nav");
//        // agent start location = <9, -5, 55>
//        TestAgent agent = agentAndState.fst;
//        var state = agentAndState.snd;
//        state.navgrid.enableFlying = true;
//
//        // agent halfway goal location = <34, 17.5f, 75>
//        // agent end goal location =     <51, 17.5f, 75>
//        // faraway asteroid location =   <-2133, 1773, -170>
//        // below platform =              <9, -15, 55>
//        // behind wall =                 <9, -5, 45>
//        Vec3 dest = new Vec3(58,17.5f,75);
//        GoalStructure G = DEPLOYonce(agent, UUGoalLib.closeTo(dest));
//        test_Goal(agentAndState.fst, agentAndState.snd, G);
//        G.printGoalStructureStatus();
//        assertTrue(G.getStatus().success());
//    }
//
//    @Test
//    public void test_navigate3DMaze() throws InterruptedException {
//        console("*** start test...");
//        var agentAndState = deployAgent("3D maze -glass");
//        // agent start location = <9, -5, 55>
//        //Thread.sleep(5000);
//        TestAgent agent = agentAndState.fst;
//        var state = agentAndState.snd;
//        state.navgrid.enableFlying = true;
//
//        // maze center location =       <80, 80, 80>
//        // button in maze location =    <19, 70, 23>, look towards ~(0.4, -0.1, -1), up = (-1, 0, 0)
//        // door location =              <-10, -1.3, 5>, look towards (-1, 0, 0), up = (0, -1, 0)
//        Vec3 dest = new Vec3(80,80,80);
//        GoalStructure G = DEPLOYonce(agent, UUGoalLib.closeTo(dest));
//        test_Goal(agentAndState.fst, agentAndState.snd, G);
//        G.printGoalStructureStatus();
//        assertTrue(G.getStatus().success());
//    }

    @Test
    public void test_navigate3DToDoor() throws InterruptedException {
        console("*** start test...");
        var agentAndState = deployAgent("myworld-3 with open door"); //myworld-3 3D-nav //Almost Empty v2 //myworld-3 with open door
        // agent start location = <9, -5, 55>
        TestAgent agent = agentAndState.fst;
        // var state = agentAndState.snd;

        GoalStructure G = DEPLOYonce(agent, UUGoalLib.close3DTo(agent,
                "LargeBlockSlideDoor",
                SEBlockFunctions.BlockSides.FRONT,
                50f,
                0.5f));
        test_Goal(agentAndState.fst, agentAndState.snd, G);
        G.printGoalStructureStatus();
        assertTrue(G.getStatus().success());
    }

    @Test
    public void test_navigate3DToBattery() throws InterruptedException {
        console("*** start test...");
        var agentAndState = deployAgent("myworld-3 Indoors"); //myworld-3 3D-nav //Almost Empty v2 //myworld-3 with open door
        // agent start location = <9, -5, 55>
        TestAgent agent = agentAndState.fst;
        // var state = agentAndState.snd;

        GoalStructure G = DEPLOYonce(agent, UUGoalLib.smartClose3DTo(
                agent,
                "LargeBlockBatteryBlock",
                SEBlockFunctions.BlockSides.FRONT,
                20f,
                0.5f));
        test_Goal(agent, agentAndState.snd, G);
        G.printGoalStructureStatus();
        assertTrue(G.getStatus().success());
    }

    @Test
    public void test_open_area() throws InterruptedException {
        console("*** start test...");
        var agentAndState = deployAgent("Glass box");
        TestAgent agent = agentAndState.fst;

        GoalStructure G = DEPLOYonce(agent, UUGoalLib.smartClose3DTo(
                agent,
                "TargetDummy",
                SEBlockFunctions.BlockSides.BACK,
                20f,
                0.5f));
        test_Goal(agent, agentAndState.snd, G);
        G.printGoalStructureStatus();
        assertTrue(G.getStatus().success());
    }

    @Test
    public void test_labrecruits_level() throws InterruptedException {
        console("*** start test...");
        var agentAndState = deployAgent("CR3_3_3_M");
        // agent start location = <0, 0, 0>, forward = <0, 1, 0>, up = <0, 0, 1>
        TestAgent agent = agentAndState.fst;
        // var state = agentAndState.snd;
        if (agentAndState.snd instanceof UUSeAgentState2D)
            ((UUSeAgentState2D) agentAndState.snd).navgrid.enableFlying = true ;

        GoalStructure G = SEQ(
                DEPLOYonce(agent, closeToButton(0)),
                lift((UUSeAgentState S) -> {
                    UseObjectExtensions useUtil = new UseObjectExtensions(S.env().getController().getSpaceEngineers());
                    WorldEntity button = (WorldEntity) S.buttons.get(0);
                    if (button == null) return false;
                    Block targetBlock = S.env().getBlock(button.id); //S.env().getController().getObserver().observe().getTargetBlock();
                    useUtil.pressButton(targetBlock, 0);
                    S.updateDoors();
                    return true;
                }),
                DEPLOYonce(agent, closeToButton(1)),
                lift((UUSeAgentState S) -> {
                    UseObjectExtensions useUtil = new UseObjectExtensions(S.env().getController().getSpaceEngineers());
                    WorldEntity button = (WorldEntity) S.buttons.get(1);
                    if (button == null) return false;
                    Block targetBlock = S.env().getBlock(button.id); //S.env().getController().getObserver().observe().getTargetBlock();
                    useUtil.pressButton(targetBlock, 2);
                    S.updateDoors();
                    return true;
                }),
                DEPLOYonce(agent, closeToButton(3)),
                lift((UUSeAgentState S) -> {
                    UseObjectExtensions useUtil = new UseObjectExtensions(S.env().getController().getSpaceEngineers());
                    WorldEntity button = (WorldEntity) S.buttons.get(3);
                    if (button == null) return false;
                    Block targetBlock = S.env().getBlock(button.id); //S.env().getController().getObserver().observe().getTargetBlock();
                    useUtil.pressButton(targetBlock, 3);
                    S.updateDoors();
                    return true;
                }),
                DEPLOYonce(agent, closeTo(
                        agent,
                        "TargetDummy",
                        (UUSeAgentState state) -> (WorldEntity e)
                                ->
                                "TargetDummy".equals(e.getStringProperty("blockType"))
                                        && Vec3.sub(e.position, state.wom.position).lengthSq() <= 400,
                        SEBlockFunctions.BlockSides.BACK,
                        0.5f
                ))
        );
        test_Goal(agent, agentAndState.snd, G);
        G.printGoalStructureStatus();
        assertTrue(G.getStatus().success());
    }
}
