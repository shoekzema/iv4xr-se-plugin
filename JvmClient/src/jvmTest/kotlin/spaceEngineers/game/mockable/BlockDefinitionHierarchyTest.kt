package spaceEngineers.game.mockable

import testhelp.MockOrRealGameTest
import kotlin.test.Test
import kotlin.test.assertEquals

class BlockDefinitionHierarchyTest : MockOrRealGameTest(inMockResourcesDirectory("BlockDefinitionHierarchyTest.txt")) {

    @Test
    fun size() = testContext {
        definitions.blockDefinitionHierarchy().let { map ->
            assertEquals(89, map.size)
        }
    }

    @Test
    fun print() = testContext {
        definitions.blockDefinitionHierarchy().forEach { it ->
            println(it)
        }
    }
}
