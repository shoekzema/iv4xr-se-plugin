package bdd

import bdd.connection.ConnectionManager
import bdd.connection.ProcessWithConnection
import bdd.repetitiveassert.repeatUntilSuccess
import io.cucumber.datatable.DataTable
import io.cucumber.java.PendingException
import io.cucumber.java.en.And
import io.cucumber.java.en.Given
import io.cucumber.java.en.Then
import io.cucumber.java.en.When
import kotlinx.coroutines.delay
import spaceEngineers.controller.extensions.typedFocusedScreen
import spaceEngineers.model.BootsColour
import spaceEngineers.model.CharacterStats
import spaceEngineers.model.DefinitionId
import spaceEngineers.model.GasContainerObject
import spaceEngineers.model.HandTool
import spaceEngineers.model.ScreenName
import spaceEngineers.model.Vec3F
import spaceEngineers.model.extensions.allBlocks
import spaceEngineers.movement.FrameSnapshot
import spaceEngineers.movement.InputSnapshot
import spaceEngineers.movement.KeyboardSnapshot
import testhelp.DEFAULT_ORIENTATION_TOLERANCE
import testhelp.DEFAULT_POSITION_TOLERANCE
import testhelp.DEFAULT_SPEED_TOLERANCE
import testhelp.DEFAULT_SPEED_TOLERANCE_OBSERVER
import testhelp.assertGreaterThan
import testhelp.assertLessThan
import testhelp.assertSameDirection
import testhelp.assertVecEquals
import java.lang.Thread.sleep
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertNotEquals
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertTrue
import kotlin.time.Duration.Companion.milliseconds


class CharacterAsserts(connectionManager: ConnectionManager) : AbstractMultiplayerSteps(connectionManager) {


    @Then("Character is standing.")
    fun character_is_standing() = observers {
        assertTrue(observer.observe().movement.isStanding)
    }

    @Then("Character is crouching.")
    fun character_is_crouching() = observers {
        assertTrue(observer.observe().movement.isCrouching)
    }

    @Then("Character boots are {string}.")
    fun character_boots_are(colourName: String) = observers {
        val bootsColour = BootsColour.valueOf(colourName.uppercase())
        repeatUntilSuccess {
            observer.observe().bootsState.let { bootState ->
                assertTrue(bootState.isColour(bootsColour), message = "BootState is $bootState, not $bootsColour")
            }
        }
    }

    @Then("Character boots are green.")
    fun character_boots_are_green() = observers {
        repeatUntilSuccess {
            observer.observe().bootsState.let { bootState ->
                assertTrue(bootState.isGreen(), message = "BootState is $bootState, not green")
            }
        }
    }

    @Then("Character boots are yellow.")
    fun character_boots_are_yellow() = observers {
        repeatUntilSuccess {
            observer.observe().bootsState.let { bootState ->
                assertTrue(bootState.isYellow(), message = "BootState is $bootState, not yellow")
            }
        }
    }

    @Then("Character boots are white.")
    fun character_boots_are_white() = observers {
        repeatUntilSuccess {
            observer.observe().bootsState.let { bootState ->
                assertTrue(bootState.isWhite(), message = "BootState is $bootState, not white")
            }
        }
    }

    @Then("Character speed is {int} m\\/s.")
    fun character_speed_is_100m_s(speed: Int) = observers {
        character_speed_is_m_s_after_milliseconds(speed, 0)
    }

    @Then("jetpack is off.")
    fun jetpack_is_off() {
        mainClient {
            repeatUntilSuccess {
                assertFalse(screens.gamePlay.data().hud.statsWrapper.jetpackOn)
            }
        }
        observers {
            repeatUntilSuccess {
                assertFalse(observer.observe().jetpackRunning)
            }
        }
    }

    @Then("jetpack is on.")
    fun jetpack_is_on() {
        mainClient {
            repeatUntilSuccess {
                assertTrue(screens.gamePlay.data().hud.statsWrapper.jetpackOn)
            }
        }
        observers {
            repeatUntilSuccess {
                assertTrue(observer.observe().jetpackRunning)
            }
        }
    }

    @Then("Character forward orientation is \\({double}, {double}, {double}).")
    fun character_is_facing(x: Double, y: Double, z: Double) = observers {
        val position = Vec3F(x, y, z)
        assertVecEquals(
            position,
            observer.observe().orientationForward,
            absoluteTolerance = DEFAULT_ORIENTATION_TOLERANCE
        )
    }

    @Then("Character speed is {int} m\\/s after {long} milliseconds.")
    fun character_speed_is_m_s_after_milliseconds(speed: Int, delayMs: Long) {
        sleep(delayMs)
        mainClient {
            assertEquals(speed.toFloat(), screens.gamePlay.data().hud.statsWrapper.speed, DEFAULT_SPEED_TOLERANCE)
        }
        observers {
            assertEquals(speed.toFloat(), observer.observe().velocity.length(), DEFAULT_SPEED_TOLERANCE_OBSERVER)
        }
    }


    @Then("Character inventory does not contain ingot {string} anymore.")
    fun character_inventory_does_not_contain_anymore(type: String) = observers {
        val definition = DefinitionId.ingot(type)
        repeatUntilSuccess {
            assertTrue(observer.observe().inventory.items.none { it.id == definition })
        }
    }

    @Then("Character inventory does not contain ore {string} anymore.")
    fun character_inventory_does_not_contain_ore_anymore(type: String) {
        character_inventory_does_not_contain_ore(type = type)
    }

    @Then("Character inventory does not contain ore {string}.")
    fun character_inventory_does_not_contain_ore(type: String) = observers {
        val definition = DefinitionId.ore(type)
        repeatUntilSuccess {
            assertTrue(observer.observe().inventory.items.none { it.id == definition })
        }
    }

    @Given("Character inventory contains no components.")
    fun character_inventory_contains_no_components() = observers {
        assertEquals(0, observer.observe().inventory.items.count { it.id.id == "MyObjectBuilder_Component" })
    }

    @Given("Character stands in front of block {string}.")
    fun character_stands_in_front_of_block(type: String) = observers {
        assertEquals(type, observer.observe().targetBlock?.definitionId?.type)
    }

    @Then("Character inventory contains components:")
    fun character_inventory_contains_components(map: List<Map<String, String>>) = observers {
        repeatUntilSuccess(initialDelayMs = 0, repeats = 2) {
            val items = observer.observe().inventory.items
            map.forEach { row ->
                val definitionId = DefinitionId.create("Component", row["component"]!!)
                val item = items.find { it.id == definitionId }
                assertNotNull(
                    item
                ) {
                    "Item not found: $definitionId"
                }
                assertEquals(
                    item.amount,
                    row["amount"]?.toInt(),
                    "Amount unexpected for $definitionId: ${item.amount} vs ${row["amount"]?.toInt()}"
                )
            }
        }
    }

    @And("Character is at \\({double}, {double}, {double}).")
    fun i_see_character_at_x_y_z(x: Double, y: Double, z: Double) = observers {
        val position = Vec3F(x, y, z)
        assertVecEquals(position, observer.observe().position, absoluteTolerance = DEFAULT_POSITION_TOLERANCE)
    }

    @Then("I can see {int} new block\\(s) with data:")
    fun i_can_see_new_block_with_data(blockCount: Int, data: List<Map<String, String>>) = observers {
        val observation = observer.observeNewBlocks()
        val allBlocks = observation.allBlocks
        assertEquals(
            blockCount,
            allBlocks.size,
            "Expected to see $blockCount blocks, not ${allBlocks.size} ${
                allBlocks.map { it.definitionId.type }.toSet()
            }."
        )
        assertEquals(allBlocks.size, data.size)
        data.forEachIndexed { index, row ->
            val block = allBlocks[index]
            context.lastNewBlock = block
            row["blockType"]?.let {
                assertEquals(it, block.definitionId.type)
            }
            row["integrity"]?.let {
                assertEquals(it.toFloat(), block.integrity)
            }
            row["maxIntegrity"]?.let {
                assertEquals(it.toFloat(), block.maxIntegrity)
            }
            row["buildIntegrity"]?.let {
                assertEquals(it.toFloat(), block.buildIntegrity)
            }
        }
    }

    @Given("character has full hydrogen tank.")
    fun character_has_full_hydrogen_tank() = observers {
        assertEquals(1f, observer.observe().hydrogen, 0.00001f)
    }

    @Given("character has almost full hydrogen tank.")
    fun character_has_almost_full_hydrogen_tank() = observers {
        assertEquals(1f, observer.observe().hydrogen, 0.04f)
    }

    @Given("character has about {int} hydrogen.")
    fun character_has_about_hydrogen(hydrogenPercentage: Int) = observers {
        assertEquals(hydrogenPercentage.toFloat(), observer.observe().hydrogen * 100f, 1f)
    }

    @Then("character doesn't have full hydrogen tank anymore after {int} milliseconds.")
    fun character_doesn_t_have_full_hydrogen_tank_anymore_after_milliseconds(delayMs: Int) = observers {
        delay(delayMs.toLong())
        assertNotEquals(observer.observe().hydrogen, 1f, 0.001f)
    }

    @Then("character has less than {int} hydrogen after {int} milliseconds.")
    fun character_has_less_than_hydrogen_after_milliseconds(hydrogenPercentage: Int, delayMs: Int) = observers {
        delay(delayMs.toLong())
        observer.observe().hydrogen.let { hydrogen ->
            assertLessThan(hydrogen, hydrogenPercentage / 100f, "Hydrogen level is ${hydrogen * 100}")

        }
    }

    @Then("Character has empty hydrogen tank after {int} milliseconds.")
    fun character_has_empty_hydrogen_tank_after_milliseconds(delayMs: Int) = observers {
        delay(delayMs.toLong())
        repeatUntilSuccess {
            assertEquals(0f, observer.observe().hydrogen, 0.03f)
        }
    }

    @Then("Character begins to fall.")
    fun character_begins_to_fall() {
        mainClient {
            repeatUntilSuccess {
                val movement = observer.observe().movement
                assertTrue(movement.isFalling || movement.isJumping, movement.toString())
            }
        }
    }

    @Then("Character begins to fall towards the ground.")
    fun character_begins_to_fall_towards_the_ground() {
        val gravity = mainClient {
            //gravity is not synchronized, we need to take it from the main client
            with(observer.observe()) {
                assertGreaterThan(velocity.length(), 0, message = "Character cannot fall when at 0 speed!")
                assertSameDirection(velocity, gravity)
                gravity
            }
        }
        observers {
            with(observer.observe()) {
                assertGreaterThan(velocity.length(), 0, message = "Character cannot fall when at 0 speed!")
                assertSameDirection(velocity, gravity)
            }
        }
    }

    @Then("Character is jumping.")
    fun character_is_jumping() {
        // jumping is actually for a very short time, the rest is falling (even if it's still going up)
        mainClient {
            repeatUntilSuccess {
                with(observer.observe()) {
                    assertSameDirection(velocity, orientationUp)
                    assertTrue(movement.isFalling || movement.isJumping, movement.toString())
                }
            }
        }
        observers {
            repeatUntilSuccess {
                with(observer.observe()) {
                    //assertSameDirection(velocity, orientationUp)

                    //TODO: check only on clients
                    //assertTrue(movement.isFalling || movement.isJumping, movement.toString())
                }
            }
        }
    }

    @Then("dampeners are off.")
    fun dampeners_are_off() {
        mainClient {
            repeatUntilSuccess {
                assertFalse(screens.gamePlay.data().hud.statsWrapper.dampenersOn)
            }
            repeatUntilSuccess {
                assertFalse(observer.observe().dampenersOn)
            }
        }
        observers {
            //assertFalse(observer.observe().dampenersOn)
        }
    }

    @Then("dampeners are on.")
    fun dampeners_are_on() {
        mainClient {
            assertTrue(screens.gamePlay.data().hud.statsWrapper.dampenersOn)
        }
        observers {
            assertTrue(observer.observe().dampenersOn)
        }
    }

    @Then("Character dampeners are on.")
    fun character_dampeners_are_on() = mainClient {
        repeatUntilSuccess { assertTrue(observer.observe().dampenersOn) }
    }

    @Then("Character dampeners are off.")
    fun character_dampeners_are_off() = mainClient {
        //TODO: maybe they are not synchronized
        repeatUntilSuccess { assertFalse(observer.observe().dampenersOn, "Dampeners are not off.") }
    }

    @Then("UI dampeners are on.")
    fun ui_dampeners_are_on() = mainClient {
        assertTrue(screens.gamePlay.data().hud.statsWrapper.dampenersOn)
    }

    @Then("UI dampeners are off.")
    fun ui_dampeners_are_off() = mainClient {
        assertFalse(screens.gamePlay.data().hud.statsWrapper.dampenersOn)
    }

    @Then("Dampeners are switched to relative mode.")
    fun dampeners_are_switched_to_relative_mode() = mainClient {
        repeatUntilSuccess(repeats = 10) {
            //assertNotNull(observer.observe().relativeDampeningEntity)
            assertTrue(
                screens.gamePlay.data().hud.statsWrapper.relativeDampenersOn,
                "Dampeners are not in relative mode."
            )
        }
    }

    @Then("Character thrusters work against the gravity.")
    fun character_thrusters_work_against_the_gravity() = mainClient {
        with(observer.observe()) {
            assertVecEquals(gravity.normalized(), -orientationUp)
            assertVecEquals(Vec3F.UP, jetpackFinalThrust.normalized())
        }
    }

    @Then("Character is positioned more than {double}m in the {string} from it's original position.")
    fun character_is_positioned_more_than_m_in_the_from_it_s_original_position(distance: Double, direction: String) {
        mainClient {
            val remembered = context.characterObservation ?: error("No original observation recorded for position")
            val new = observer.observe()
            val distanceVector = new.position - remembered.position
            assertGreaterThan(distanceVector.length(), distance.toFloat())
        }
    }

    @When("Character remembers it's position.")
    fun character_remembers_it_s_position() {
        mainClient {
            context.rememberCharacter(observer.observe())
        }
    }

    @And("Character is inside the block.")
    fun character_is_inside_the_block() = mainClient {
        assertNotEquals(
            observer.observe().id,
            observer.observeControlledEntity().id,
            "Character is not inside a block, but it should."
        )
    }

    @And("Character is inside block {string}.")
    fun character_is_inside_block(displayName: String) = mainClient {
        repeatUntilSuccess {
            assertEquals(
                displayName,
                observer.observeControlledEntity().displayName,
                "Character is not inside a block, but it should."
            )
        }
    }

    @And("Character is not inside a block.")
    fun character_is_not_inside_a_block() = mainClient {
        assertEquals(
            observer.observe().id,
            observer.observeControlledEntity().id,
            "Character is inside a block, but it shouldn't."
        )
    }

    @Then("Character speed and direction matches the dampening entity.")
    fun character_speed_and_direction_matches_the_dampening_entity() = observers {
        repeatUntilSuccess(repeats = 100, delayMs = 50) {
            val observation = observer.observe()
            val relativeDampeningEntity = observation.relativeDampeningEntity ?: error("No relative dampening entity!")
            assertVecEquals(observation.velocity, relativeDampeningEntity.velocity, absoluteTolerance = 0.1f)
        }
        repeatUntilSuccess(repeats = 100, delayMs = 50) {
            val observation = observer.observe()
            val relativeDampeningEntity = observation.relativeDampeningEntity ?: error("No relative dampening entity!")
            assertEquals(observation.velocity.length(), relativeDampeningEntity.velocity.length(), 0.1f)
        }
    }

    @Then("Item {string} is removed from the inventory.")
    fun item_is_removed_from_the_inventory(definitionIdStr: String) = mainClient {
        val definitionId = DefinitionId.parse(definitionIdStr)
        repeatUntilSuccess {
            observer.observe().inventory.items.firstOrNull { it.id == definitionId }.let {
                assertNull(it, "Item is not supposed to be in the inventory: ${it?.id}")
            }
        }
    }

    @Then("There is item {string} floating around.")
    fun there_is_item_floating_around(definitionIdStr: String) = observers {
        val definitionId = DefinitionId.parse(definitionIdStr)
        repeatUntilSuccess {
            observer.observeFloatingObjects().firstOrNull { it.itemDefinition.definitionId == definitionId }.let {
                assertNotNull(it, "Item is supposed to float around: ${it?.itemDefinition?.definitionId}")
            }
        }
    }

    @And("There is no item {string} floating around.")
    fun there_is_no_item_floating_around(definitionIdStr: String) = observers {
        val definitionId = DefinitionId.parse(definitionIdStr)
        repeatUntilSuccess {
            observer.observeFloatingObjects().firstOrNull { it.itemDefinition.definitionId == definitionId }.let {
                assertNull(it, "Item is NOT supposed to float around: ${it?.itemDefinition?.definitionId}")
            }
        }
    }

    @Then("Character holds {string}.")
    fun character_holds(definitionIdStr: String) = observers {
        val definitionId = DefinitionId.parse(definitionIdStr)
        repeatUntilSuccess {
            val observation = observer.observe()
            assertNotNull(observation.currentWeapon, "No weapon equipped at all")
            assertEquals(definitionId, observation.currentWeapon?.definitionId)
        }
    }

    @Then("Tool stays active.")
    fun tool_stays_active() = observers {
        repeatUntilSuccess {
            observer.observe().currentWeapon.let { weapon ->
                assertNotNull(weapon, "Character holds no weapon!")
                assertTrue(weapon is HandTool, "The weapon is not HandTol${weapon.javaClass} ${weapon.definitionId}")
                assertTrue(weapon.isShooting, "The tool ${weapon.definitionId} is not active when it should.")
            }
        }
    }

    @Then("Tool is not active.")
    fun tool_is_not_active() = observers {
        repeatUntilSuccess {
            observer.observe().currentWeapon.let { weapon ->
                assertNotNull(weapon)
                assertTrue(weapon is HandTool)
                assertFalse(weapon.isShooting, "The tool ${weapon.definitionId} is active when it shouldn't.")
            }
        }
    }

    @Then("Character energy is depleted.")
    fun character_energy_gets_depleted() = observers {
        repeatUntilSuccess {
            assertEquals(
                0f,
                observer.observe().energy,
                absoluteTolerance = 0.001f,
                "Energy was supposed to be depleted."
            )
        }
    }

    @Then("Character inventory contains item {string}.")
    fun character_inventory_contains_item(definitionIdStr: String) = observers {
        val definition = DefinitionId.parse(definitionIdStr)
        assertInventoryContains(definition)
    }

    @Then("Character inventory contains:")
    fun character_inventory_contains(map: List<Map<String, String>>) = observers {
        repeatUntilSuccess {
            map.forEach { row ->
                val definition = DefinitionId.parse(row.getValue("definitionId"))
                val count = row["count"]?.toIntOrNull()
                repeatUntilSuccess {
                    val items = observer.observe().inventory.items
                    assertTrue(
                        items.any { it.id == definition },
                        "Should contain $definition, but only contains: " + items.map { it.id }.joinToString(",")
                    )
                    count?.let { c ->
                        assertEquals(
                            c,
                            items.filter { item -> item.id == definition }.sumOf { it.amount },
                            "Should contain $c of $definition"
                        )
                    }
                }
            }
        }
    }

    @Then("Character model plays the {string} animation in the Cockpit.")
    fun character_model_plays_the_animation_in_the_cockpit(animationName: String) = observers {
        repeatUntilSuccess {
            debug.characterAnimations().animationsPerLayer.values.any {
                it.contains(animationName)
            }
        }
    }

    @Then("Character model plays the {string} animation.")
    fun character_model_plays_the_animation(animationName: String) = observers {
        repeatUntilSuccess {
            debug.characterAnimations().animationsPerLayer.values.any {
                it.contains(animationName)
            }
        }
    }

    @When("Layer {string} is not in {string} stance.")
    fun character_is_not_in_idle_stance(layer: String, stance: String) = observers {
        assertNotEquals(
            "$layer/$stance",
            debug.characterAnimations().animationsPerLayer.getValue(layer)
        )
    }

    @Then("Layer {string} is in {string} stance.")
    fun character_is_in_stance(layer: String, stance: String) = observers {
        val stc = debug.characterAnimations().animationsPerLayer.getValue(layer)
        assertEquals(
            "$layer/$stance",
            stc,
            message = "Expected layer '${layer}', stance '$stance', but was '$stc'",
        )
    }

    @Given("Character inventory contains ore {string}.")
    fun character_inventory_contains_ore(type: String) = mainClient {
        if (screens.typedFocusedScreen() != ScreenName.Terminal) {
            character.showInventory()
        }
        assertInventoryContains(DefinitionId.create("Ore", type))
    }

    @Given("Character inventory contains ingot {string}.")
    fun character_inventory_contains(type: String) = mainClient {
        if (screens.typedFocusedScreen() != ScreenName.Terminal) {
            character.showInventory()
        }
        assertInventoryContains(DefinitionId.create("Ingot", type))
    }

    private suspend fun ProcessWithConnection.assertInventoryContains(definition: DefinitionId) {
        repeatUntilSuccess {
            val items = observer.observe().inventory.items
            assertTrue(
                items.any { it.id == definition },
                "Character inventory doesn't contain ${definition}, it contains: ${items}"
            )
        }
    }

    @Then("Character stats are:")
    fun character_stats_are_changed_to(dt: DataTable) {
        val dataTable = dt.asMaps().first().map { it.key to it.value.toFloat() }.toMap()
        val relativeTolerance = dataTable["relative_tolerance"] ?: 0.01f
        mainClient {
            val observation = observer.observe() as CharacterStats
            checkStats(dataTable, observation, relativeTolerance)
            checkStats(dataTable, screens.gamePlay.data().hud.statsWrapper, relativeTolerance)
        }
    }

    private fun checkStats(dataTable: Map<String, Float>, stats: CharacterStats, relativeTolerance: Float) {
        checkStat(dataTable, "health", stats.health, relativeTolerance)
        checkStat(dataTable, "oxygen", stats.oxygen, relativeTolerance)
        checkStat(dataTable, "energy", stats.energy, relativeTolerance)
        checkStat(dataTable, "hydrogen", stats.hydrogen, relativeTolerance)
    }

    private fun checkStat(
        dataTable: Map<String, Float>,
        name: String,
        stat: Float,
        relativeTolerance: Float
    ) {
        dataTable[name]?.let { tableStat ->
            assertEquals(
                tableStat / 100f,
                stat,
                absoluteTolerance = 1f * relativeTolerance,
                message = "absoluteTolerance: ${1f * relativeTolerance}"
            )
        }
    }


    @Then("hydrogen is less than maximum.")
    fun hydrogen_is_less_than_maximum() {
        mainClient {
            assertLessThan(observer.observe().hydrogen, 1f)
        }
    }

    @Then("health is less than maximum.")
    fun health_is_less_than_maximum() {
        mainClient {
            repeatUntilSuccess {
                assertLessThan(observer.observe().health, 1f)
            }
        }
    }

    @Then("oxygen is less than maximum.")
    fun oxygen_is_less_than_maximum() {
        mainClient {
            assertLessThan(observer.observe().oxygen, 1f)
        }
    }

    @Then("Content of hydrogen in the {definitionId} in the inventory is less than maximum.")
    fun of_hydrogen_in_the_in_the_inventory_is_less_than_maximum(definitionId: DefinitionId) {
        mainClient {
            val items = observer.observe().inventory.items
            items.filterIsInstance<GasContainerObject>().apply {
                assertTrue(
                    isNotEmpty(),
                    "No item of type $definitionId found in the inventory, found only ${items.map { it.id }}"
                )
            }.forEach {
                assertLessThan(it.gasLevel, 1f)
            }
        }
    }

    @Then("Personal light is off.")
    fun personal_light_is_off() = mainClient {
        assertEquals(0f, observer.observe().currentLightPower)
        //assertFalse(observer.observe().lightEnabled, "light is supposed to be off!")
        //TODO: check on hud too
    }

    @Then("Character is dead.")
    fun character_is_dead() = mainClient {
        repeatUntilSuccess {
            with(observer.observe()) {
                assertTrue(movement.isDead, "Not dead based on CharacterMovement flag: $movement")
                assertEquals(0f, health, "Not dead based on health: $health")
            }
        }
    }

    @Then("Remote terminal for grid {string} is disabled.")
    fun remote_terminal_is_disabled(grid: String) = mainClient {
        delay(500.milliseconds)
        input.startPlaying(
            listOf(
                FrameSnapshot(
                    InputSnapshot(
                        keyboard = KeyboardSnapshot(pressedKeys = listOf(16, 75), text = listOf(' '))
                    )
                )
            )
        )
        delay(500.milliseconds)
        val grids = screens.terminal.remoteAccess.data().grids
        assertTrue(grids.isNotEmpty(), "No grids to see!")
        assertFalse(
            grids.firstOrNull() { it.name == grid }?.isSelectable
                ?: error("No grid with name $grid found, found only ${grids.map { it.name }}"),
            "Grid $grid is Selectable and therefore not disabled"
        )
    }

    @Then("Remote terminal for grid {string} is enabled.")
    fun remote_terminal_is_enabled(grid: String) = mainClient {
        delay(500.milliseconds)
        input.startPlaying(
            listOf(
                FrameSnapshot(
                    InputSnapshot(
                        keyboard = KeyboardSnapshot(pressedKeys = listOf(16, 75), text = listOf(' '))
                    )
                )
            )
        )
        delay(500.milliseconds)
        val grids = screens.terminal.remoteAccess.data().grids
        assertTrue(grids.isNotEmpty(), "No grids to see!")
        assertTrue(
            grids.first { it.name == grid }.isSelectable,
            "Grid $grid is not Selectable and therefore disabled"
        )
    }
}
