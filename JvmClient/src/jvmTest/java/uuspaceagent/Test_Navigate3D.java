package uuspaceagent;

import eu.iv4xr.framework.mainConcepts.TestAgent;
import eu.iv4xr.framework.spatial.Vec3;
import nl.uu.cs.aplib.mainConcepts.GoalStructure;
import nl.uu.cs.aplib.utils.Pair;
import org.junit.jupiter.api.Test;

import static nl.uu.cs.aplib.AplibEDSL.DEPLOYonce;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static uuspaceagent.PrintInfos.showWOMAgent;
import static uuspaceagent.TestUtils.console;
import static uuspaceagent.TestUtils.loadSE;

public class Test_Navigate3D {
    public Pair<TestAgent, UUSeAgentState> deployAgent() throws InterruptedException {
        var agentAndState = loadSE("myworld-3 3D-nav");
        TestAgent agent = agentAndState.fst;
        UUSeAgentState state = agentAndState.snd;
        Thread.sleep(1000);
        state.updateState(state.agentId);
        // agent start location = <9, -5, 55>
        console(showWOMAgent(state.wom));
        return new Pair<TestAgent, UUSeAgentState>(agent,state);
    }

    public void test_Goal(TestAgent agent, UUSeAgentState state, GoalStructure G) throws  InterruptedException {
        agent.setGoal(G);
        int turn = 0;
        while (G.getStatus().inProgress()) {
            console(">> [" + turn + "] " + showWOMAgent(state.wom));
            agent.update();
            //Thread.sleep(50);
            turn++;
            if (turn >= 1400) break;
        }
        TestUtils.closeConnectionToSE(state);
    }

    @Test
    public void test_navigate3DTo() throws InterruptedException {
        console("*** start test...");
        var agentAndState = deployAgent();
        TestAgent agent = agentAndState.fst;
        UUSeAgentState state = agentAndState.snd;
        state.navgrid.enableFlying = true;

        // agent halfway goal location = <34, 17.5, 75>
        // agent end goal location =     <51, 17.5, 75>
        Vec3 dest = new Vec3(34,17.5f,75);
        GoalStructure G = DEPLOYonce(agent, UUGoalLib.closeTo(dest));
        test_Goal(agentAndState.fst, agentAndState.snd, G);
        G.printGoalStructureStatus();
        assertTrue(G.getStatus().success());
    }
}
