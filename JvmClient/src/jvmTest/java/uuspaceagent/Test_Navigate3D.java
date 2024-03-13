package uuspaceagent;

import eu.iv4xr.framework.mainConcepts.TestAgent;
import nl.uu.cs.aplib.mainConcepts.GoalStructure;
import nl.uu.cs.aplib.utils.Pair;
import org.junit.jupiter.api.Test;

import static nl.uu.cs.aplib.AplibEDSL.DEPLOYonce;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static uuspaceagent.PrintInfos.showWOMAgent;
import static uuspaceagent.TestUtils.*;

public class Test_Navigate3D {
    public Pair<TestAgent, UUSeAgentState3DOctree> deployAgent(String worldname) throws InterruptedException {
        var agentAndState = loadSE3D(worldname);
        TestAgent agent = agentAndState.fst;
        var state = agentAndState.snd;
        Thread.sleep(1000);
        state.updateState(state.agentId);
        console(showWOMAgent(state.wom));
        return new Pair<>(agent, state);
    }

    public void test_Goal(TestAgent agent, UUSeAgentState3DOctree state, GoalStructure G) throws  InterruptedException {
        agent.setGoal(G);
        int turn = 0;
        while (G.getStatus().inProgress()) {
            console(">> [" + turn + "] " + showWOMAgent(state.wom));
            agent.update();
            //Thread.sleep(50);
            turn++;
            if (turn >= 1400) break;
        }
        state.exportGrid();
        state.exportManualProfileShit();
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
}
