package spaceEngineers.controller

import spaceEngineers.model.BaseScreenData
import spaceEngineers.model.GamePlayData
import spaceEngineers.model.JoinGameData
import spaceEngineers.model.LoadGameData
import spaceEngineers.model.LoadingData
import spaceEngineers.model.MainMenuData
import spaceEngineers.model.MedicalsData
import spaceEngineers.model.MessageBoxData
import spaceEngineers.model.SaveAsData
import spaceEngineers.model.ServerConnectData
import spaceEngineers.model.TerminalControlPanelData
import spaceEngineers.model.TerminalFactionsData
import spaceEngineers.model.TerminalInfoData
import spaceEngineers.model.TerminalInventoryData
import spaceEngineers.model.TerminalProductionData
import spaceEngineers.model.TerminalRemoteAccessData
import spaceEngineers.model.TerminalScreenData
import spaceEngineers.model.ToolbarConfigData

interface Screens {
    val focusedScreen: FocusedScreen
    fun waitUntilTheGameLoaded()
    val medicals: Medicals
    val terminal: Terminal
    val mainMenu: MainMenu
    val messageBox: MessageBox
    val joinGame: JoinGame
    val serverConnect: ServerConnect
    val loadGame: LoadGame
    val newGame: NewGame
    val gamePlay: GamePlay
    val saveAs: SaveAs
    val toolbarConfig: ToolbarConfig
    val loading: Loading
}

interface FocusedScreen {
    fun data(): BaseScreenData?
    fun closeScreenNow()
    fun closeScreen()
}

interface Loading {
    fun data(): LoadingData
}

interface NewGame {
    fun close()
}

interface ToolbarConfig {
    fun data(): ToolbarConfigData
    fun search(text: String)
    fun dropGridItemToToolbar(gridLocation: Int, toolbarLocation: Int)
    fun close()
    fun selectCategory(index: Int)
}

interface GamePlay {
    fun data(): GamePlayData
    fun showMainMenu()
    fun showToolbarConfig()
}

interface ServerConnect {
    fun close()
    fun data(): ServerConnectData
    fun connect()
    fun enterAddress(address: String)
    fun toggleAddServerToFavorites()
}

interface JoinGame {
    fun close()
    fun directConnect()
    fun joinWorld()
    fun refresh()
    fun serverDetails()
    fun selectTab(index: Int)
    fun selectGame(index: Int)
    fun data(): JoinGameData
}

interface MessageBox {
    fun data(): MessageBoxData
    fun pressYes()
    fun pressNo()
}

interface MainMenu {
    fun data(): MainMenuData
    fun `continue`()
    fun newGame()
    fun loadGame()
    fun joinGame()
    fun options()
    fun character()
    fun exitToWindows()
    fun exitToMainMenu()
    fun saveAs()
    fun save()
}

interface SaveAs {
    fun data(): SaveAsData
    fun pressOk()
    fun pressCancel()
    fun setName(name: String)
}

interface Medicals {
    fun data(): MedicalsData
    fun selectRespawn(roomIndex: Int)
    fun respawn()
    fun join()
    fun selectFaction(factionIndex: Int)
    fun refresh()
    fun showMessageOfTheDay()
    fun showMainMenu()
}

interface Terminal {
    fun data(): TerminalScreenData
    fun selectTab(index: Int)
    fun close()

    val inventory: InventoryTab
    val controlPanel: ControlPanelTab
    val production: ProductionTab
    val info: InfoTab
    val factions: FactionsTab
    val comms: CommsTab
    val gps: GpsTab
    val remoteAccess: RemoteAccess
}

interface RemoteAccess {
    fun data(): TerminalRemoteAccessData
}

interface CommsTab

interface GpsTab

interface FactionsTab {
    fun data(): TerminalFactionsData
    fun create()
    fun join()
    fun cancelJoin()
    fun leave()
    fun proposePeace()
    fun acceptPeace()
    fun declareWar()
    fun cancelRequest()
    fun selectFaction(index: Int)
}

interface ControlPanelTab {
    fun data(): TerminalControlPanelData
    fun filterBlocks(text: String)
    fun enterBlockGroup(text: String)
    fun groupSave()
    fun groupDelete()
    fun transferTo(index: Int)
    fun selectShareMode(index: Int)
}

interface InfoTab {
    fun data(): TerminalInfoData
    fun convertToShip()
    fun convertToStation()
    fun enterGridName(name: String)
    fun renameGrid()
    fun setShowCenterOfMassEnabled(enabled: Boolean)
    fun setShowGravityRangeEnabled(enabled: Boolean)
    fun setShowSensorsFieldRangeEnabled(enabled: Boolean)
    fun setShowAntennaRangeEnabled(enabled: Boolean)
    fun setShowGridPivotEnabled(enabled: Boolean)
    fun setFriendlyAntennaRange(value: Float)
    fun setEnemyAntennaRange(value: Float)
    fun setOwnedAntennaRange(value: Float)
}

interface InventoryTab {
    fun data(): TerminalInventoryData
    fun transferInventoryItemToLeft(sourceInventoryId: Int, destinationInventoryId: Int, itemId: Int)
    fun transferInventoryItemToRight(sourceInventoryId: Int, destinationInventoryId: Int, itemId: Int)
    fun withdraw()
    fun deposit()
    fun dropSelected()
    fun fromBuildPlannerToProductionQueue()
    fun selectedToProductionQueue()

    val left: InventorySide
    val right: InventorySide
}

interface ProductionTab {
    fun data(): TerminalProductionData
    fun addToProductionQueue(index: Int)
    fun removeFromProductionQueue(index: Int)
    fun selectBlueprint(index: Int)
    fun enterBlueprintSearchBox(text: String)
    fun toggleProductionRepeatMode()
    fun toggleProductionCooperativeMode()
    fun selectAssembler(index: Int)
}

interface InventorySide {
    fun filter(text: String)
    fun swapToGrid()
    fun swapToCharacterOrItem()
    fun filterAll()
    fun filterEnergy()
    fun filterShip()
    fun filterSystem()
    fun filterStorage()
    fun toggleHideEmpty()
    fun selectItem(index: Int)
    fun clickSelectedItem()
    fun doubleClickSelectedItem()
}

interface LoadGame {
    fun close()
    fun data(): LoadGameData
    fun filter(text: String)
    fun doubleClickWorld(index: Int)
    fun load()
    fun edit()
    fun delete()
    fun save()
    fun publish()
}
