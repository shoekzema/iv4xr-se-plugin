package spaceEngineers.controller

import spaceEngineers.model.Block
import spaceEngineers.model.BlockId
import spaceEngineers.model.CharacterObservation
import spaceEngineers.model.CubeGrid
import spaceEngineers.model.DebugInfo
import spaceEngineers.model.Vec3F

interface SpaceEngineersAdmin {
    val blocks: BlocksAdmin
    val character: CharacterAdmin
    val observer: ObserverAdmin
    fun setFrameLimitEnabled(enabled: Boolean)
    fun updateDefaultInteractDistance(distance: Float)
    fun debugInfo(): DebugInfo
    val tests: TestAdmin
    fun showNotification(text: String)
    fun ping(): String
    fun echo(text: String): String
    fun setCreativeTools(enabled: Boolean)
}

interface TestAdmin {
    fun adminOnly()
    fun gameOnly()
    fun observerOnly()
}

interface CharacterAdmin {
    /**
     * @param position A position within the game scenario itself in system coordinates, uses game units.
     * @param orientationForward Forward vector together with up vector define the character orientation.
     *      Either both or none of them have to be set. The orientation is relative to the system.
     *      They should be normalised vectors and perpendicular.
     * @param orientationUp Complementary vector to the forward vector.
     */
    fun teleport(position: Vec3F, orientationForward: Vec3F? = null, orientationUp: Vec3F? = null): CharacterObservation

    /**
     * @param blockId Id of the block.
     * @param functionIndex Index of useObject. First one is 0 and so on.
     * @param action Action flag, to get possible actions of the block, check [spaceEngineers.model.UseObject.primaryAction] and [spaceEngineers.model.UseObject.secondaryAction] values.
     */
    fun use(blockId: BlockId, functionIndex: Int, action: Int)

    fun create(name: String, position: Vec3F, orientationForward: Vec3F, orientationUp: Vec3F): CharacterObservation

    fun switch(id: String)

    fun remove(id: String)
    fun showTerminal(blockId: String)
    fun die()
    fun mainCharacterId(): String
    fun localCharacterId(): String?
    fun updateEnergy(energy: Float)
    fun updateOxygen(oxygen: Float)
    fun updateHydrogen(hydrogen: Float)
    fun updateHealth(health: Float)
}

interface ObserverAdmin {
    fun observeCharacters(): List<CharacterObservation>
    fun blockById(blockId: BlockId): Block
    fun gridById(gridId: String): CubeGrid
}
