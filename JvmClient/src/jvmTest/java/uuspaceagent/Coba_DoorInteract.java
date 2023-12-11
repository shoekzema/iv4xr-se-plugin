package uuspaceagent;

import eu.iv4xr.framework.mainConcepts.TestAgent;
import eu.iv4xr.framework.mainConcepts.WorldEntity;
import nl.uu.cs.aplib.mainConcepts.GoalStructure;
import org.junit.jupiter.api.Test;
import spaceEngineers.controller.useobject.UseObjectExtensions;
import spaceEngineers.model.Block;
import spaceEngineers.model.CharacterObservation;
import spaceEngineers.model.DoorBase;
import spaceEngineers.model.ToolbarLocation;

import static nl.uu.cs.aplib.AplibEDSL.*;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static uuspaceagent.PrintInfos.showWOMAgent;
import static uuspaceagent.TestUtils.console;
import static uuspaceagent.TestUtils.loadSE;

/**
 * For trying out grinding directly using primitives from SE.
 */
public class Coba_DoorInteract {

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
    public void test() throws InterruptedException {

        var state = loadSE("myworld-3 atdoor").snd;

        state.updateState(state.agentId);

        System.out.println("** Equiping grinder");
        state.env().equip(new ToolbarLocation(0,0));

        state.updateState(state.agentId);

        WorldEntity agentInfo = state.wom.elements.get(state.agentId) ;
        System.out.println("** Agent's info: " + PrintInfos.showWorldEntity(agentInfo));

        //System.out.println("==== " + PrintInfos.showWOMElements(state.wom));
        System.out.println("======");
        CharacterObservation cobs = state.env().getController().getObserver().observe() ;
        if(cobs.getTargetBlock() != null) {
            System.out.println("=== target block: " + cobs.getTargetBlock().getId());
        }
        WorldEntity target = SEBlockFunctions.findClosestBlock(state.wom, "LargeBlockSlideDoor", 10) ;
        String doorId = target.id ;
        System.out.println("** door state 1: " + PrintInfos.showWorldEntity(target));

        UseObjectExtensions useUtil = new UseObjectExtensions(state.env().getController().getSpaceEngineers()) ;

        Block targetBlock = state.env().getController().getObserver().observe().getTargetBlock() ;
        useUtil.openIfNotOpened((DoorBase) targetBlock);

        Thread.sleep(2000);
        state.updateState(state.agentId);

        target = SEBlockFunctions.findClosestBlock(state.wom, "LargeBlockSlideDoor", 10) ;
        //doorId = target.id ;
        System.out.println("** door state 2: " + PrintInfos.showWorldEntity(target));

    }

    @Test
    public void test_button() throws InterruptedException {
        console("*** start test...");
        var agentAndState = loadSE("myworld-3 atdoor");
        TestAgent agent = agentAndState.fst ;
        UUSeAgentState state = agentAndState.snd ;
        state.updateState(state.agentId) ;

        state.env().equip(new ToolbarLocation(0,0));
        state.updateState(state.agentId);

        GoalStructure G = SEQ(
                DEPLOYonce(agent, UUGoalLib.closeToButton(agent,
                        "ButtonPanelLarge",
                        SEBlockFunctions.BlockSides.FRONT,
                        20f,
                        0)),
                lift((UUSeAgentState S) -> {
                    UseObjectExtensions useUtil = new UseObjectExtensions(S.env().getController().getSpaceEngineers());
                    Block targetBlock = S.env().getController().getObserver().observe().getTargetBlock();
                    if (targetBlock == null) return false;
                    useUtil.pressButton(targetBlock, 0);
                    return true;
                }));
        test_Goal(agent, state, G);

        G.printGoalStructureStatus();
        assertTrue(G.getStatus().success());
    }
}
