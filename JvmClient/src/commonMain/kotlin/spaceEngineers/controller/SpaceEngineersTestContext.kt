package spaceEngineers.controller

import spaceEngineers.model.Block
import spaceEngineers.model.BlockId
import spaceEngineers.model.CharacterObservation
import spaceEngineers.model.Observation
import spaceEngineers.model.ToolbarLocation
import spaceEngineers.model.Vec3F
import spaceEngineers.model.extensions.allBlocks

data class SpaceEngineersTestContext(
    var grinderLocation: ToolbarLocation? = null,
    var torchLocation: ToolbarLocation? = null,
    var lastNewBlock: Block? = null,
    val blockTypeToToolbarLocation: MutableMap<String, ToolbarLocation> = mutableMapOf(),
    var allNewestBlocks: MutableList<Block> = mutableListOf(),
    val observationHistory: MutableList<Observation> = mutableListOf(),
    val characterObservationHistory: MutableList<CharacterObservation> = mutableListOf(),
    var platformOrientationUp: Vec3F? = null,
) {

    var characterObservation: CharacterObservation? = null
        private set

    fun rememberCharacter(characterObservation: CharacterObservation) {
        this.characterObservation = characterObservation
    }

    val lastNewBlockId: BlockId?
        get() = lastNewBlock?.id

    fun updatePlatformOrientationUpIfNotSet(observation: Observation) {
        if (platformOrientationUp == null) {
            updatePlatformOrientationUp(observation)
        }
    }

    fun updatePlatformOrientationUp(observation: Observation) {
        val characterOrientationUp = observation.character.orientationUp
        val blocks = observation.allBlocks
        platformOrientationUp = blocks.map { it.orientationUp }
            .maxByOrNull { it.distanceTo(characterOrientationUp) }
    }

    fun updateToolbarLocation(dataTable: List<Map<String, String>>) {
        blockTypeToToolbarLocation.clear()
        blockTypeToToolbarLocation.putAll(
            dataTable.associate {
                it["blockType"]!! to ToolbarLocation(slot = it["slot"]!!.toInt(), page = it["page"]!!.toInt())
            }
        )
    }

    fun blockToolbarLocation(blockType: String): ToolbarLocation {
        return blockTypeToToolbarLocation[blockType] ?: error("Toolbar location not set for type $blockType")
    }

    fun updateNewBlocks(allBlocks: List<Block>) {
        allBlocks.lastOrNull()?.let {
            lastNewBlock = it
        }
        allNewestBlocks.addAll(allBlocks)
    }

    fun addToHistory(observation: Observation) {
        observationHistory.add(observation)
    }

    fun addToHistory(observation: CharacterObservation) {
        characterObservationHistory.add(observation)
    }
}
