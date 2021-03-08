package spaceEngineers.game

import spaceEngineers.commands.MovementArgs
import spaceEngineers.controller.CharacterController
import spaceEngineers.model.Vec3
import testhelp.mockController
import kotlin.test.Test
import spaceEngineers.controller.observe
import spaceEngineers.model.SeObservation
import testhelp.assertFloatEquals


fun CharacterController.blockingMoveForwardByDistance(distance: Float, velocity: Float = 1f, maxTries: Int = 200): SeObservation {
    val startPosition = observe().position
    repeat(maxTries) {
        moveAndRotate(MovementArgs(movement = Vec3(0f, 0f, -velocity))).let {
            if (it.position.distanceTo(startPosition) >= distance) {
                return it
            }
        }
    }
    error("timeout after $maxTries tries, currentDistance: ${observe().position.distanceTo(startPosition)}")
}

class MoveForwardTest {


    @Test
    fun moveAndRotateTest() = mockController {
        val basePosition = observe().position
        blockingMoveForwardByDistance(10f).let {
            assertFloatEquals(10f, basePosition.distanceTo(it.position))
        }

    }
}