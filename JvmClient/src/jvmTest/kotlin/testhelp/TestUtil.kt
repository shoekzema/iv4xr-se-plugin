package testhelp

import kotlinx.coroutines.runBlocking
import spaceEngineers.controller.ContextControllerWrapper
import spaceEngineers.controller.DataExtendedSpaceEngineers
import spaceEngineers.controller.ExtendedSpaceEngineers
import spaceEngineers.controller.JvmSpaceEngineersBuilder
import spaceEngineers.controller.SpaceEngineers
import spaceEngineers.controller.loadFromTestResources
import spaceEngineers.iv4xr.navigation.Iv4XRAStarPathFinder
import spaceEngineers.transport.jsonrpc.KotlinJsonRpcError
import spaceEngineers.transport.jsonrpc.remoteException

const val TEST_AGENT = SpaceEngineers.DEFAULT_AGENT_ID

val RESOURCES_DIR = "src/jvmTest/resources/"
val MOCK_RESOURCES_DIR = "${RESOURCES_DIR}mock/"
val TEST_MOCK_RESPONSE_LINE = """
{"Id":"Mock","Position":{"X":4.0,"Y":2.0,"Z":0.0},"OrientationForward":{"X":0.0,"Y":0.0,"Z":0.0},"OrientationUp":{"X":0.0,"Y":0.0,"Z":0.0},"Velocity":{"X":0.0,"Y":0.0,"Z":0.0},"Extent":{"X":0.0,"Y":0.0,"Z":0.0},"Entities":[{"Id":"Ente","Position":{"X":3.0,"Y":2.0,"Z":1.0}}],"Grids":[{"Blocks":[{"MaxIntegrity":10.0,"BuildIntegrity":1.0,"Integrity":5.0,"BlockType":"MockBlock","MinPosition":{"X":0.0,"Y":0.0,"Z":0.0},"MaxPosition":{"X":0.0,"Y":0.0,"Z":0.0},"Size":{"X":0.0,"Y":0.0,"Z":0.0},"OrientationForward":{"X":0.0,"Y":0.0,"Z":0.0},"OrientationUp":{"X":0.0,"Y":0.0,"Z":0.0},"Id":"blk","Position":{"X":5.0,"Y":5.0,"Z":5.0}}],"Id":null,"Position":{"X":5.0,"Y":5.0,"Z":5.0}}]}
""".trim()

val SIMPLE_PLACE_GRIND_TORCH = "simple-place-grind-torch"

fun spaceEngineersSimplePlaceGrindTorchSuspend(
    scenarioId: String = SIMPLE_PLACE_GRIND_TORCH,
    agentId: String = TEST_AGENT,
    spaceEngineers: SpaceEngineers = JvmSpaceEngineersBuilder.default().localhost(agentId),
    block: suspend SpaceEngineers.() -> Unit
) {
    try {
        spaceEngineers.session.loadFromTestResources(scenarioId)
        runBlocking {
            block(spaceEngineers)
        }
    } finally {
        spaceEngineers.close()
    }
}

fun spaceEngineersSuspend(
    agentId: String = TEST_AGENT,
    spaceEngineers: ExtendedSpaceEngineers = DataExtendedSpaceEngineers(
        ContextControllerWrapper(
            spaceEngineers = JvmSpaceEngineersBuilder.default().localhost(agentId)
        ),
        pathFinder = Iv4XRAStarPathFinder(),
    ),
    block: suspend ExtendedSpaceEngineers.() -> Unit
) {
    try {
        runBlocking {
            block(spaceEngineers)
        }
    } catch (e: KotlinJsonRpcError) {
        e.remoteException?.stacktrace?.let(::println)
        throw e
    } finally {
        spaceEngineers.close()
    }
}

fun spaceEngineers(
    agentId: String = TEST_AGENT,
    spaceEngineers: SpaceEngineers = ContextControllerWrapper(
        spaceEngineers = JvmSpaceEngineersBuilder.default().localhost(agentId)
    ),
    block: SpaceEngineers.() -> Unit
) {
    try {
        runBlocking {
            block(spaceEngineers)
        }
    } finally {
        spaceEngineers.close()
    }
}
