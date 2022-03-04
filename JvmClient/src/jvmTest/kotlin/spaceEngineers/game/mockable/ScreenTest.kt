package spaceEngineers.game.mockable

import kotlin.test.Ignore
import testhelp.MockOrRealGameTest
import kotlin.test.Test


@Ignore
class ScreenTest : MockOrRealGameTest(forceRealGame = true, loadScenario = false) {



    @Test
    fun medicalRooms() = testContext {
        screens.medicals.medicalRooms().apply(::println)
    }

    @Test
    fun respawn() = testContext {
        screens.medicals.respawn(1)
    }

    @Test
    fun focusedScreen() = testContext {
        println(screens.focusedScreen())
    }

    @Test
    fun chooseFaction() = testContext {
        screens.medicals.chooseFaction(0)
    }

    @Test
    fun factions() = testContext {
        screens.medicals.factions().apply(::println)
    }

    @Test
    fun waitUntilTheGameLoaded() = testContext {
        screens.waitUntilTheGameLoaded()
    }

    @Test
    fun transferItem() = testContext {
        val firstLeftInventory = screens.terminal.inventory.data().leftInventories.first()
        val firstItem = firstLeftInventory.items.first()
        screens.terminal.inventory.transferInventoryItem(0, 0, firstItem.itemId)
    }

    @Test
    fun switchLeftInventoryToCharacter() = testContext {
        screens.terminal.inventory.left.swapToCharacterOrItem()
    }

    @Test
    fun switchLeftInventoryToGrid() = testContext {
        screens.terminal.inventory.left.swapToGrid()
    }


    @Test
    fun switchRightInventoryToGrid() = testContext {
        screens.terminal.inventory.right.swapToGrid()
    }

    @Test
    fun filterLeftSide() = testContext {
        screens.terminal.inventory.left.filter("Gener")
    }

    @Test
    fun filterLeftEnergy() = testContext {
        screens.terminal.inventory.left.filterEnergy()
    }

    @Test
    fun filterLeftAll() = testContext {
        screens.terminal.inventory.left.filterAll()
    }

    @Test
    fun deposit() = testContext {
        screens.terminal.inventory.deposit()
    }

    @Test
    fun withdraw() = testContext {
        screens.terminal.inventory.withdraw()
    }

    @Test
    fun leftToggleHideEmpty() = testContext {
        screens.terminal.inventory.left.toggleHideEmpty()
    }

    @Test
    fun showTerminal() = testContext {
        character.showTerminal()
    }

    @Test
    fun showInventory() = testContext {
        character.showInventory()
    }

    @Test
    fun selectItemAndClick() = testContext {
        screens.terminal.inventory.left.selectItem(0)
        screens.terminal.inventory.dropSelected()
        //screens.terminal.inventory.left.doubleClickSelectedItem()
    }

    @Test
    fun mainMenuContinue() = testContext {
        screens.mainMenu.`continue`()
    }

    @Test
    fun exitToWindows() = testContext {
        screens.mainMenu.exitToWindows()
        println(screens.messageBox.data())
        screens.messageBox.pressNo()
    }

    @Test
    fun messageBoxData() = testContext {
        println(screens.messageBox.data())
    }

    @Test
    fun character() = testContext {
        screens.mainMenu.character()
    }

    @Test
    fun directConnect() = testContext {
        screens.serverConnect.connect()
    }

    @Test
    fun enterAddress() = testContext {
        screens.mainMenu.joinGame()
        delay(100)
        screens.joinGame.directConnect()
        screens.serverConnect.enterAddress("1.2.3.4:20016")
    }

    @Test
    fun enterAddress2() = testContext {
        screens.serverConnect.enterAddress("1.2.3.4:20016")
        screens.serverConnect.toggleAddServerToFavorites()
        println(screens.serverConnect.data())
    }

    @Test
    fun closeTerminal() = testContext {
        screens.terminal.close()
    }

    @Test
    fun selectLoad() = testContext {
        //screens.mainMenu.loadGame()
        //delay(5000)
        screens.loadGame.doubleClickWorld(0)
    }

    @Test
    fun loadData() = testContext {
        println(screens.loadGame.data())
    }
}
