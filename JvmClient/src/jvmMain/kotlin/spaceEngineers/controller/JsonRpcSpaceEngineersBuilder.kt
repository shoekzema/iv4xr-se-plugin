package spaceEngineers.controller

import spaceEngineers.controller.SpaceEngineers.Companion.DEFAULT_AGENT_ID
import spaceEngineers.transport.StringLineReaderWriter
import java.io.File

interface JsonRpcSpaceEngineersBuilder {

    fun localhost(agentId: String = DEFAULT_AGENT_ID): JsonRpcSpaceEngineers

    fun mock(agentId: String = DEFAULT_AGENT_ID, file: File): JsonRpcSpaceEngineers

    fun mock(agentId: String = DEFAULT_AGENT_ID, lineToReturn: String): JsonRpcSpaceEngineers

    fun mock(agentId: String = DEFAULT_AGENT_ID, lines: List<String>): JsonRpcSpaceEngineers

    fun fromStringLineReaderWriter(agentId: String, stringLineReaderWriter: StringLineReaderWriter): JsonRpcSpaceEngineers

    companion object : JsonRpcSpaceEngineersBuilder {

        var builder: JsonRpcSpaceEngineersBuilder = GsonRpcSpaceEngineersBuilder()

        override fun localhost(agentId: String): JsonRpcSpaceEngineers {
            return builder.localhost(agentId)
        }

        override fun mock(agentId: String, file: File): JsonRpcSpaceEngineers {
            return builder.mock(agentId, file)
        }

        override fun mock(agentId: String, lineToReturn: String): JsonRpcSpaceEngineers {
            return builder.mock(agentId, lineToReturn)
        }

        override fun mock(agentId: String, lines: List<String>): JsonRpcSpaceEngineers {
            return builder.mock(agentId, lines)
        }

        override fun fromStringLineReaderWriter(agentId: String, stringLineReaderWriter: StringLineReaderWriter): JsonRpcSpaceEngineers {
            return builder.fromStringLineReaderWriter(agentId, stringLineReaderWriter)
        }

    }
}
