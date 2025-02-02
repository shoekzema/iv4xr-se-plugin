package bdd.setup

import bdd.connection.ConnectionManager
import bdd.connection.ConnectionSetup
import bdd.connection.ProcessWithConnection


class MedbayDSSetup(
    connectionSetup: ConnectionSetup,
    cm: ConnectionManager,
    realConnectionManagerUser: RealConnectionManagerUser = RealConnectionManagerUser(cm)
) : MedbaySetup(connectionSetup = connectionSetup, cm = cm, realConnectionManagerUser = realConnectionManagerUser) {
    val dedicatedServerManager =
        DedicatedServerManager(processConfig = connectionSetup.admin, scenarioDir = cm.config.scenarioPath)

    override fun beforeAll() {
        super.beforeAll()
        DedicatedServerManager.killDedicatedServerWindows()
    }

    override fun afterAll() {
        super.afterAll()
        dedicatedServerManager.close()
    }


    override suspend fun ProcessWithConnection.setupServer(scenarioId: String) {
        connectDirectly(connectionManager.admin.gameProcess.address)
    }

    override suspend fun ProcessWithConnection.setupClient() {
        connectDirectly(connectionManager.admin.gameProcess.address)
    }

    override suspend fun onBeforeScenario(scenarioId: String) {
        if (!dedicatedServerManager.isRunning()) {
            dedicatedServerManager.start(scenarioId)
        }
        //TODO: check if its responding and kill + restart otherwise
    }

}
