package bdd

import bdd.repetitiveassert.repeatUntilSuccess
import io.cucumber.java.en.Then
import io.cucumber.java.en.When
import bdd.connection.ConnectionManager
import spaceEngineers.model.Block
import spaceEngineers.model.DefinitionId
import spaceEngineers.model.extensions.allBlocks
import spaceEngineers.model.extensions.blockByCustomName
import spaceEngineers.model.extensions.blocksByCustomName
import spaceEngineers.model.extensions.shortDescription
import testhelp.assertLessThan
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertTrue

class BlockAsserts(connectionManager: ConnectionManager) : AbstractMultiplayerSteps(connectionManager) {


    @Then("Observed grid mass is {double}.")
    fun observed_grid_mass_is(mass: Double) = observers {
        val massByType = definitions.blockDefinitions().associate {
            it.definitionId to it.mass.toDouble()
        }
        assertTrue(
            observer.observeBlocks().grids.any { it.mass == mass.toFloat() },
            "Haven't found any grids with mass $mass, found: ${
                observer.observeBlocks().grids.associate {
                    it.id to "mass: ${it.mass} blocks: ${it.blocks.size}, sum: ${
                        it.blocks.sumOf { massByType[it.definitionId]!! }
                    }"
                }
            }."
        )
    }

    @When("Block type {block} has mass {double}.")
    fun block_type_has_mass(definitionId: DefinitionId, mass: Double) = observers {
        val blockDefinition = definitions.blockDefinitions().first {
            it.definitionId == definitionId
        }
        assertEquals(mass.toFloat(), blockDefinition.mass, message = "Block $definitionId does not have mass $mass")
    }


    @Then("I see no block of type {block}.")
    fun i_see_no_block_of_type(definitionId: DefinitionId) = observers {
        val observation = observer.observeBlocks()
        assertTrue(
            observation.allBlocks
                .none { it.definitionId == definitionId }
        )
    }

    @Then("Target block integrity is less than maximum.")
    fun target_block_integrity_is_less_than_maximum() = observers {
        repeatUntilSuccess {
            checkBlockIntegrityLessThanMaximum(observer.observe().targetBlock)
        }
    }

    @Then("Target block integrity is at maximum.")
    fun target_block_integrity_is_at_maximum() = observers {
        repeatUntilSuccess {
            checkBlockIntegrityAtMaximum(observer.observe().targetBlock)
        }
    }

    @Then("Target block integrity is at {float}%.")
    fun target_block_integrity_is_at(percentageIntegrity: Float) = observers {
        repeatUntilSuccess(delayMs = 1000) {
            val targetBlock = observer.observe().targetBlock
            assertNotNull(targetBlock, "no target block!")
            assertEquals(percentageIntegrity / 100f, targetBlock.integrity / targetBlock.maxIntegrity, 0.0001f)
        }
    }

    @Then("Block {string} integrity is less than maximum.")
    fun block_integrity_is_less_than_maximum(blockName: String) = observers {
        repeatUntilSuccess(delayMs = 1000) {
            val blockObservation = observer.observeBlocks()
            val targetBlock = blockObservation.blockByCustomName(customName = blockName)
            checkBlockIntegrityLessThanMaximum(targetBlock)
        }
    }

    @Then("Block {string} integrity is at maximum.")
    fun block_integrity_is_at_maximum(customName: String) = observers {
        repeatUntilSuccess {
            checkBlockIntegrityAtMaximum(
                observer.observeBlocks().blocksByCustomName(customName).firstOrNull()
            )
        }
    }

    @Then("All blocks less than {int} meters away from {string} have maximum integrity.")
    fun all_blocks_less_than_meters_away_from_have_maximum_integrity(distance: Int, customName: String) = observers {
        observer.observeBlocks().let { blockObservation ->
            val mainBlock = blockObservation.blockByCustomName(customName)
            blockObservation.allBlocks.filter {
                it.id != mainBlock.id &&
                        it.position.distanceTo(mainBlock.position) <= distance
            }.forEach {
                checkBlockIntegrityAtMaximum(it)
            }
        }
    }

    private fun checkBlockIntegrityAtMaximum(block: Block?) {
        assertNotNull(block, "Block not found!")
        assertEquals(block.integrity, block.maxIntegrity, "Not maximum for block ${block.shortDescription}")
    }

    private fun checkBlockIntegrityLessThanMaximum(block: Block?) {
        assertNotNull(block, "Block not found!")
        assertLessThan(block.integrity, block.maxIntegrity, "Not less than maximum for block ${block.shortDescription}")
    }
}
