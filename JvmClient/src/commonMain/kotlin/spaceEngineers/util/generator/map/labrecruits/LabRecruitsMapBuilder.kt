package spaceEngineers.util.generator.map.labrecruits

import spaceEngineers.controller.ExtendedSpaceEngineers
import spaceEngineers.model.DoorBase
import spaceEngineers.model.extensions.blockByCustomName
import spaceEngineers.util.generator.map.MapPlacer
import java.lang.Thread.sleep

class LabRecruitsMapBuilder(
    val map: LabRecruitsMap,
    val spaceEngineers: ExtendedSpaceEngineers,
    val mapPlacer: MapPlacer = MapPlacer(map, spaceEngineers = spaceEngineers)
) {

    fun generate() {
        map.placeGenerator()
        map.placeGravityGenerator()
        val gridId = mapPlacer.generate()
        createGroups(gridId, map)
        mapButtons(map)
        sleep(1500)
        closeAllDoors(gridId, map)
    }

    private fun closeAllDoors(gridId: String, map: LabRecruitsMap) = se {
        map.doors.values.forEach { door ->
            val doorBlock = admin.observer.gridById(gridId).blocks.blockByCustomName(door.id) as DoorBase
            extensions.blocks.useObject.closeIfNotClosed(doorBlock)
        }
    }

    private fun createGroups(gridId: String, map: LabRecruitsMap) = se {
        map.mappings.forEach { (button, doors) ->
            admin.blocks.createOrUpdateGroup(
                "group-${button.id}",
                gridId,
                doors.map { mapPlacer.blockIdsToCustomNames.getValue(it.id) }
            )
        }
    }

    private fun mapButtons(map: LabRecruitsMap) = se {
        map.mappings.keys.forEach { button ->
            admin.blocks.mapButtonToGroup(
                buttonBlockId = mapPlacer.blockIdsToCustomNames.getValue(button.id),
                buttonIndex = 0,
                action = "Open",
                groupName = "group-${button.id}"
            )
        }
    }

    private fun <T> se(block: ExtendedSpaceEngineers.() -> T): T {
        return block(spaceEngineers)
    }
}
