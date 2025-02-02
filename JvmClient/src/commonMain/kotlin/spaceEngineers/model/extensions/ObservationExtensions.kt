package spaceEngineers.model.extensions

import spaceEngineers.controller.Observer
import spaceEngineers.model.Block
import spaceEngineers.model.BlockId
import spaceEngineers.model.CubeGrid
import spaceEngineers.model.Observation
import spaceEngineers.model.TerminalBlock

fun Observation.blocksByCustomName(customName: String): List<TerminalBlock> {
    return allBlocks.filterIsInstance<TerminalBlock>().filter { it.customName == customName }
}

fun Observation.blockByCustomName(customName: String): TerminalBlock {
    return allBlocks.blockByCustomName(customName)
}

fun List<Block>.blockByCustomName(customName: String): TerminalBlock {
    return typedBlockByCustomName(customName = customName)
}

inline fun <reified T : TerminalBlock> Observation.typedBlockByCustomName(customName: String): T {
    return allBlocks.typedBlockByCustomName(customName)
}

inline fun <reified T : TerminalBlock> List<Block>.typedBlockByCustomName(customName: String): T {
    return filterIsInstance<T>().firstOrNull { it.customName == customName } ?: error(
        "Block with name $customName not found, found only ${
        filterIsInstance<T>().map { it.customName }.sorted()
        }"
    )
}

val Observation.allBlocks: List<Block>
    get() {
        return this.grids.flatMap { it.blocks }
    }

fun Observation.blockById(blockId: BlockId): Block {
    return typedBlockById(blockId)
}

fun <T : Block> Observation.typedBlockById(blockId: BlockId): T {
    return allBlocks.firstOrNull { it.id == blockId } as T?
        ?: error("Block by id $blockId not found!")
}

fun Observation.heaviestGrid(): CubeGrid {
    val heaviestGrid = grids.maxByOrNull { grid -> grid.mass }
        ?: error("No grid found!")
    if (heaviestGrid.mass <= 0.0f)
        error("The heaviest grid has zero (or negative) mass!")
    return heaviestGrid
}

fun Observation.largestGrid(): CubeGrid {
    return grids.maxByOrNull { grid -> grid.blocks.size } ?: error("No grid found!")
}

fun Observer.blockById(blockId: BlockId): Block {
    return observeBlocks().blockById(blockId)
}
