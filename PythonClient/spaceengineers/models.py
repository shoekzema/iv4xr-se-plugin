# Generated by JVMClient, see DataStructuresGenerator.kt
# pylint: disable=C0103,C0115,C0114,R0902

from dataclasses import dataclass
from typing import List


@dataclass
class AmountedDefinitionId:
    Amount: "int"
    Id: "DefinitionId"


@dataclass
class BaseEntity:
    DefinitionId: "DefinitionId"
    DisplayName: "str"
    Id: "str"
    InScene: "bool"
    Name: "str"
    OrientationForward: "Vec3F"
    OrientationUp: "Vec3F"
    Position: "Vec3F"
    Velocity: "Vec3F"


@dataclass
class BasePose:
    OrientationForward: "Vec3F"
    OrientationUp: "Vec3F"
    Position: "Vec3F"


@dataclass
class Block:
    BuildIntegrity: "float"
    BuiltBy: "str"
    DefinitionId: "DefinitionId"
    Functional: "bool"
    GridPosition: "Vec3I"
    Integrity: "float"
    MaxIntegrity: "float"
    MaxPosition: "Vec3F"
    MinPosition: "Vec3F"
    OwnerId: "str"
    Size: "Vec3F"
    UseObjects: "List[UseObject]"
    Working: "bool"
    Id: "str"
    OrientationForward: "Vec3F"
    OrientationUp: "Vec3F"
    Position: "Vec3F"


@dataclass
class BlockDefinition:
    BuildProgressModels: "List[BuildProgressModel]"
    Components: "List[Component]"
    CubeSize: "CubeSize"
    MountPoints: "List[MountPoint]"
    Size: "Vec3F"
    Type: "str"
    AvailableInSurvival: "bool"
    DefinitionId: "DefinitionId"
    Enabled: "bool"
    Mass: "float"
    Public: "bool"


@dataclass
class BlockOrGroupItem:
    Text: "str"
    Visible: "bool"


@dataclass
class BlueprintDefinition:
    DisplayName: "str"
    Prerequisites: "List[AmountedDefinitionId]"
    Results: "List[AmountedDefinitionId]"


@dataclass
class BootsState:
    Value: "object"


@dataclass
class BuildProgressModel:
    BuildRatioUpperBound: "float"


@dataclass
class CharacterAnimations:
    AnimationsPerLayer: "dict"


@dataclass
class CharacterMovement:
    Direction: "object"
    IsCrouching: "bool"
    IsFalling: "bool"
    IsJumping: "bool"
    IsStanding: "bool"
    Mode: "object"
    Rotation: "object"
    Speed: "object"
    Value: "object"


@dataclass
class CharacterMovementFlags:
    Value: "object"
    WantsWalk: "bool"


@dataclass
class CharacterObservation:
    BootsState: "object"
    Camera: "BasePose"
    CurrentWeapon: "ExtendedEntity"
    DampenersOn: "bool"
    DefinitionId: "DefinitionId"
    DisplayName: "str"
    Extent: "Vec3F"
    Gravity: "Vec3F"
    HeadLocalXAngle: "float"
    HeadLocalYAngle: "float"
    Health: "float"
    HelmetEnabled: "bool"
    Hydrogen: "float"
    Id: "str"
    InScene: "bool"
    Inventory: "Inventory"
    JetpackControlThrust: "Vec3F"
    JetpackFinalThrust: "Vec3F"
    JetpackRunning: "bool"
    Movement: "object"
    MovementFlags: "object"
    Name: "str"
    OrientationForward: "Vec3F"
    OrientationUp: "Vec3F"
    Oxygen: "float"
    Position: "Vec3F"
    RelativeDampeningEntity: "BaseEntity"
    SuitEnergy: "float"
    TargetBlock: "Block"
    TargetUseObject: "UseObject"
    Velocity: "Vec3F"


@dataclass
class Component:
    Count: "int"
    DeconstructItem: "DataPhysicalItemDefinition"
    Definition: "DataPhysicalItemDefinition"


@dataclass
class CubeGrid:
    Blocks: "List[Block]"
    DefinitionId: "DefinitionId"
    DisplayName: "str"
    Id: "str"
    InScene: "bool"
    Mass: "float"
    Name: "str"
    OrientationForward: "Vec3F"
    OrientationUp: "Vec3F"
    Parked: "bool"
    Position: "Vec3F"
    Velocity: "Vec3F"


@dataclass
class CubeSize:
    Value: "float"
    Name: "str"
    Ordinal: "int"


@dataclass
class DataEdge:
    pass


@dataclass
class DataNode:
    pass


@dataclass
class DataPhysicalItemDefinition:
    AvailableInSurvival: "bool"
    DefinitionId: "DefinitionId"
    Enabled: "bool"
    Health: "int"
    Mass: "float"
    Public: "bool"
    Size: "Vec3F"
    Volume: "float"


@dataclass
class DebugInfo:
    Executable: "str"
    IsDedicated: "bool"
    IsServer: "bool"
    MachineName: "str"
    MultiplayerActive: "bool"
    SessionReady: "bool"
    UserName: "str"
    Version: "int"


@dataclass
class DefinitionBase:
    AvailableInSurvival: "bool"
    DefinitionId: "DefinitionId"
    Enabled: "bool"
    Public: "bool"


@dataclass
class DefinitionId:
    Id: "str"
    Type: "str"


@dataclass
class ExtendedEntity:
    DefinitionId: "DefinitionId"
    DisplayName: "str"
    Id: "str"
    InScene: "bool"
    Name: "str"
    Velocity: "Vec3F"
    OrientationForward: "Vec3F"
    OrientationUp: "Vec3F"
    Position: "Vec3F"


@dataclass
class Faction:
    Name: "str"
    Tag: "str"


@dataclass
class File:
    FullName: "str"
    IsDirectory: "bool"
    Name: "str"


@dataclass
class FloatingObject:
    Amount: "float"
    DisplayName: "str"
    EntityId: "int"
    ItemDefinition: "PhysicalItemDefinition"


@dataclass
class FrameSnapshot:
    Input: "InputSnapshot"


@dataclass
class GamePlayData:
    Hud: "Hud"
    OreMarkers: "List[OreMarker]"


@dataclass
class GuiControlBase:
    Enabled: "bool"
    Name: "str"
    Visible: "bool"


@dataclass
class Hud:
    Stats: "dict"
    StatsWrapper: "object"


@dataclass
class InputSnapshot:
    Keyboard: "KeyboardSnapshot"
    Mouse: "MouseSnapshot"


@dataclass
class Inventory:
    CargoPercentage: "float"
    CurrentMass: "float"
    CurrentVolume: "float"
    Id: "int"
    Items: "List[InventoryItem]"
    MaxMass: "float"
    MaxVolume: "float"


@dataclass
class InventoryItem:
    Amount: "int"
    Id: "DefinitionId"
    ItemId: "int"


@dataclass
class JoinGameData:
    Games: "List[ListedGameInformation]"
    JoinWorldButton: "GuiControlBase"
    SelectedTab: "int"
    SelectedTabName: "str"


@dataclass
class KeyboardSnapshot:
    PressedKeys: "List[int]"
    Text: "List[object]"


@dataclass
class ListedGameInformation:
    Server: "str"
    World: "str"


@dataclass
class LoadGameData:
    CurrentDirectory: "File"
    Files: "List[File]"
    RootDirectory: "File"


@dataclass
class LoadingData:
    Cancelled: "bool"
    CloseButtonEnabled: "bool"
    CurrentText: "str"
    IsLoaded: "bool"
    IsOpened: "bool"
    ScreenToLoad: "str"
    ScreenToUnload: "str"
    SkipTransition: "bool"
    Visible: "bool"


@dataclass
class MainMenuData:
    Type: "int"


@dataclass
class MedicalRoomData:
    AvailableIn: "str"
    Name: "str"


@dataclass
class MedicalsData:
    Factions: "List[Faction]"
    IsMotdOpen: "bool"
    IsMultiplayerReady: "bool"
    MedicalRooms: "List[MedicalRoomData]"
    Paused: "bool"
    RespawnButton: "GuiControlBase"
    ShowFactions: "bool"
    ShowMotD: "bool"


@dataclass
class MessageBoxData:
    ButtonType: "int"
    Caption: "str"
    Text: "str"


@dataclass
class MountPoint:
    Default: "bool"
    Enabled: "bool"
    End: "Vec3F"
    ExclusionMask: "int"
    Normal: "Vec3F"
    PressurizedWhenOpen: "bool"
    PropertiesMask: "int"
    Start: "Vec3F"


@dataclass
class MouseSnapshot:
    CursorPositionX: "int"
    CursorPositionY: "int"
    LeftButton: "bool"
    MiddleButton: "bool"
    RightButton: "bool"
    ScrollWheelValue: "int"
    X: "int"
    XButton1: "bool"
    XButton2: "bool"
    Y: "int"


@dataclass
class NavGraph:
    Edges: "List[DataEdge]"
    Nodes: "List[DataNode]"


@dataclass
class Observation:
    Character: "CharacterObservation"
    Grids: "List[CubeGrid]"


@dataclass
class OreMarker:
    Distance: "float"
    Materials: "List[DefinitionId]"
    Position: "Vec3F"
    Text: "str"


@dataclass
class ParticleEffect:
    Name: "str"
    Position: "Vec3F"


@dataclass
class Particles:
    Effects: "List[ParticleEffect]"
    Enabled: "bool"
    Paused: "bool"


@dataclass
class PhysicalItemDefinition:
    Health: "int"
    Mass: "float"
    Size: "Vec3F"
    Volume: "float"
    AvailableInSurvival: "bool"
    DefinitionId: "DefinitionId"
    Enabled: "bool"
    Public: "bool"


@dataclass
class ProductionQueueItem:
    Amount: "int"
    Blueprint: "BlueprintDefinition"


@dataclass
class SaveAsData:
    Name: "str"


@dataclass
class ServerConnectData:
    AddServerToFavorites: "bool"
    Address: "str"


@dataclass
class SliderData:
    MaxValue: "float"
    MinValue: "float"
    Value: "float"


@dataclass
class Sound:
    CueEnum: "str"
    IsPaused: "bool"
    IsPlaying: "bool"


@dataclass
class SoundBanks:
    Sound: "List[Sound]"


@dataclass
class TerminalControlPanelData:
    GridBlocks: "List[BlockOrGroupItem]"
    NewGroupName: "str"
    Owner: "str"
    Search: "str"
    ShareBlock: "List[str]"
    ShareBlockSelectedIndex: "int"
    ShowBLockInToolbarConfig: "bool"
    ShowBlockInTerminal: "bool"
    ShowOnHUD: "bool"
    ToggleBlock: "bool"
    TransferTo: "List[str]"


@dataclass
class TerminalInfoData:
    EnemyAntennaRange: "SliderData"
    FriendlyAntennaRange: "SliderData"
    GridInfo: "str"
    GridName: "str"
    OwnedAntennaRange: "SliderData"
    ShowAntennaRange: "bool"
    ShowCenterOfMass: "bool"
    ShowGravityRange: "bool"
    ShowGridPivot: "bool"
    ShowSensorsFieldRange: "bool"


@dataclass
class TerminalInventoryData:
    LeftInventories: "List[Inventory]"
    RightInventories: "List[Inventory]"


@dataclass
class TerminalProductionData:
    Blueprints: "List[BlueprintDefinition]"
    Inventory: "List[AmountedDefinitionId]"
    ProductionCooperativeMode: "bool"
    ProductionQueue: "List[ProductionQueueItem]"
    ProductionRepeatMode: "bool"


@dataclass
class TerminalScreenData:
    SelectedTab: "str"


@dataclass
class Toolbar:
    Items: "List[ToolbarItem]"
    PageCount: "int"
    SlotCount: "int"


@dataclass
class ToolbarConfigData:
    Categories: "List[str]"
    GridItems: "List[DefinitionId]"
    SearchText: "str"
    SelectedCategories: "List[str]"


@dataclass
class ToolbarItem:
    Enabled: "bool"
    Name: "str"


@dataclass
class ToolbarLocation:
    Page: "int"
    Slot: "int"


@dataclass
class UseObject:
    SupportedActions: "int"
    ContinuousUsage: "bool"
    Name: "str"
    PrimaryAction: "int"
    SecondaryAction: "int"


@dataclass
class Vec2F:
    X: "float"
    Y: "float"


@dataclass
class Vec3F:
    X: "float"
    Y: "float"
    Z: "float"


@dataclass
class Vec3I:
    X: "int"
    Y: "int"
    Z: "int"
