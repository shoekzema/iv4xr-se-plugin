package spaceEngineers.controller

import spaceEngineers.model.BlockId
import spaceEngineers.model.BlockLocation
import spaceEngineers.model.CubeGrid
import spaceEngineers.model.DefinitionId
import spaceEngineers.model.Vec3F
import spaceEngineers.model.Vec3I

interface BlocksAdmin {

    fun placeAt(
        blockDefinitionId: DefinitionId,
        position: Vec3F,
        orientationForward: Vec3F = Vec3F.FORWARD,
        orientationUp: Vec3F = Vec3F.UP,
        color: Vec3F? = null,
    ): CubeGrid

    fun placeInGrid(
        blockDefinitionId: DefinitionId,
        gridId: String,
        minPosition: Vec3I,
        orientationForward: Vec3I = Vec3I.FORWARD,
        orientationUp: Vec3I = Vec3I.UP,
        color: Vec3F? = null,
    ): String

    fun batchPlaceInGrid(
        gridId: String,
        color: Vec3F? = null,
        blockPlacementConfigs: List<BlockLocation>
    ): List<BlockId>

    fun remove(blockId: BlockId)
    fun setIntegrity(blockId: BlockId, integrity: Float)
    fun setCustomName(blockId: BlockId, customName: String)
    fun createOrUpdateGroup(name: String, gridId: String, blockIds: List<BlockId>)
    fun mapButtonToBlock(buttonBlockId: BlockId, buttonIndex: Int, action: String, targetId: BlockId)
    fun mapButtonToGroup(buttonBlockId: BlockId, buttonIndex: Int, action: String, groupName: String)
    fun setPhysicalItemInInventory(blockId: BlockId, definitionId: DefinitionId, amount: Float)
    fun requestConversionToShip(gridId: String)
    fun requestConversionToStation(gridId: String)
    fun setDestructibleBlocks(gridId: String, destructibleBlocks: Boolean)

    val warhead: WarheadAdmin
    val medicalRoom: MedicalRoomAdmin
    val functionalBlock: FunctionalBlockAdmin
    val terminalBlock: TerminalBlockAdmin
    val sensorBlock: SensorBlockAdmin
    val pistonBase: PistonBaseAdmin
    val thrust: ThrustAdmin
    val timerBlock: TimerBlockAdmin
    val buttonPanel: ButtonPanelAdmin
    val door: DoorBaseAdmin
    val textPanel: TextPanelAdmin
    val beacon: BeaconAdmin
    val survivalKit: SurvivalKitAdmin
    val soundBlock: SoundBlockAdmin
}

interface SoundBlockAdmin {
    fun setRange(blockId: String, range: Float)
    fun setVolume(blockId: String, volume: Float)
    fun setLoopPeriod(blockId: String, loopPeriod: Float)
    fun playSound(blockId: String)
    fun stopSound(blockId: String)
    fun selectSound(blockId: String, cueId: String)
}

interface SurvivalKitAdmin {
    fun setSpawnName(blockId: String, name: String)
}

interface BeaconAdmin {
    fun setRadius(blockId: BlockId, radius: Float)
}

interface TextPanelAdmin {
    fun setPublicTitle(blockId: String, publicTitle: String)
    fun setPrivateTitle(blockId: String, privateTitle: String)
    fun setPrivateDescription(blockId: String, privateDescription: String)
    fun setContentType(blockId: String, contentType: Int)
    fun setTextPadding(blockId: String, padding: Float)
    fun setText(blockId: String, text: String)
    fun setAlignment(blockId: String, alignment: Int)
    fun setFontSize(blockId: String, fontSize: Float)
}

interface DoorBaseAdmin {
    fun setAnyoneCanUse(blockId: String, anyoneCanUse: Boolean)
    fun setOpen(blockId: String, open: Boolean)
}

interface ButtonPanelAdmin {
    fun setAnyoneCanUse(blockId: String, anyoneCanUse: Boolean)
}

interface TimerBlockAdmin {
    fun setTriggerDelay(blockId: BlockId, triggerDelay: Float)
    fun start(blockId: BlockId)
    fun stop(blockId: BlockId)
    fun triggerNow(blockId: BlockId)
}

interface ThrustAdmin {
    fun setThrustOverride(blockId: BlockId, thrustOverride: Float)
}

interface PistonBaseAdmin {
    fun setVelocity(blockId: String, velocity: Float)
    fun recreateTop(blockId: String)
}

interface SensorBlockAdmin {
    fun setFieldMin(blockId: String, fieldMin: Vec3F)
    fun setFieldMax(blockId: String, fieldMax: Vec3F)
}

interface MedicalRoomAdmin {
    fun setRespawnAllowed(blockId: String, respawnAllowed: Boolean)
    fun setHealingAllowed(blockId: String, healingAllowed: Boolean)
    fun setRefuelAllowed(blockId: String, refuelAllowed: Boolean)
    fun setSpawnWithoutOxygenEnabled(blockId: String, spawnWithoutOxygenEnabled: Boolean)
}

interface WarheadAdmin {
    fun explode(blockId: BlockId)
    fun detonate(blockId: BlockId)
    fun startCountdown(blockId: BlockId): Boolean
    fun stopCountdown(): Boolean
    fun setArmed(blockId: BlockId, armed: Boolean)
    fun setCountdownMs(blockId: BlockId, countdown: Int)
}

interface FunctionalBlockAdmin {
    fun setEnabled(blockId: BlockId, enabled: Boolean)
}

interface TerminalBlockAdmin {
    fun setCustomName(blockId: BlockId, customName: String)
    fun setCustomData(blockId: BlockId, customData: String)
    fun setShowInInventory(blockId: String, showInInventory: Boolean)
    fun setShowInTerminal(blockId: String, showInTerminal: Boolean)
    fun setShowOnHUD(blockId: String, showOnHUD: Boolean)
}
