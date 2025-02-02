package spaceEngineers.game.mockable

import spaceEngineers.controller.SpaceEngineers
import spaceEngineers.model.DefinitionId
import spaceEngineers.model.DoorBase
import spaceEngineers.model.FueledPowerProducer
import spaceEngineers.model.FunctionalBlock
import spaceEngineers.model.GravityGenerator
import spaceEngineers.model.GravityGeneratorSphere
import spaceEngineers.model.MedicalRoom
import spaceEngineers.model.TerminalBlock
import spaceEngineers.model.Vec3F
import spaceEngineers.model.typing.DefinitionIds
import testhelp.MockOrRealGameTest
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

class PolymorphicBlocksTest : MockOrRealGameTest() {

    @Test
    fun smallGenerator() = testContext {
        buildAndCheckType<FueledPowerProducer>(DefinitionId.reactor("LargeBlockSmallGenerator"))
        buildAndCheckType<TerminalBlock>(DefinitionId.reactor("LargeBlockSmallGenerator"))
        buildAndCheckType<FunctionalBlock>(DefinitionId.reactor("LargeBlockSmallGenerator"))
    }

    @Test
    fun medicalRoom() = testContext {
        buildAndCheckType<MedicalRoom>(DefinitionIds.MedicalRoom.LargeMedicalRoom)
    }

    @Test
    fun gravityGenerator() = testContext {
        buildAndCheckType<GravityGenerator>(DefinitionIds.GravityGenerator.EMPTY)
    }

    @Test
    fun gravityGeneratorSphere() = testContext {
        buildAndCheckType<GravityGeneratorSphere>(DefinitionIds.GravityGeneratorSphere.EMPTY)
    }

    @Test
    fun slideDoor() = testContext {
        buildAndCheckType<DoorBase>(DefinitionId.door("LargeBlockSlideDoor"))
        buildAndCheckType<TerminalBlock>(DefinitionId.door("LargeBlockSlideDoor"))
        buildAndCheckType<FunctionalBlock>(DefinitionId.door("LargeBlockSlideDoor"))
    }

    private inline fun <reified T> SpaceEngineers.buildAndCheckType(definitionId: DefinitionId) {
        val z = 1000
        admin.character.teleport(Vec3F(0, 0, z + 15), Vec3F.FORWARD, Vec3F.UP)
        observer.observeNewBlocks()
        val grid = admin.blocks.placeAt(definitionId, Vec3F(0, 0, z + 0), Vec3F.FORWARD, Vec3F.UP)
        val block = grid.blocks.first()
        assertEquals(block.definitionId.type, definitionId.type)
        assertTrue(block is T)
        admin.blocks.remove(block.id)
    }
}
