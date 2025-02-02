# Screens

## Introduction

The game is organized into "screens". A screen is something like a scene in game development terminology. There
is `GamePlayScreen` for the main gameplay. There is `TerminalScreen` for operating terminals, there is `MainMenuScreen`,
and many more.

This API allows more advanced features outside the game itself such as loading multiplayer games, joining dedicated
server and lobby games, saving game, checking dialogs, respawning, controlling refineries, assemblers, and more.

The game has its own GUI components. Buttons, text fields, tables, etc. Screen API allows reading important data from
screens and to manipulate screen by pressing buttons, entering text fields, etc.

## Implemented screens

Some screens are fully implemented, some screens only have basic functionality, some have implemented only functionality
necessary for our purposes and some screens are not implemented at all.

To see the list of currently implemented screens, please
check [Screens](https://iv4xr-project.github.io/iv4xr-se-plugin/space-engineers-api/spaceEngineers.controller/-screens/index.html)
interface.

To request new functionality, please create a [GitHub issue](https://github.com/iv4xr-project/iv4xr-se-plugin/issues/new).

## Using API

To get the name of the current screen, call `Screens.focusedScreen` (doesn't work on dedicated server).

If the screen has some data, there is `data()` method, that returns current state of the screen. To get API for a
screen, call `Screens.XXX`, where XXX is required screen. If you call a method on screen without focus, an exception is
thrown.

## Extra points

- There is no synchronization and no waiting for results. If you do a lot of actions in a row, there can be a problem (
  typically an exception saying, that the expected screen doesn't have focus - yet).
- In SE code, all screen classes have `MyGui` prefix and `Screen` suffix, we removed that for API and client side to
  improve readability.
- `Medicals` screen is both for choosing faction and respawn point.
- `MainMenu` is used both for main menu (outside game) and in-game menu (when pressing ESC while playing), even though
  the buttons differ.
- API is trying to throw meaningful exceptions when possible (ex. when trying to click a disabled button). If you run
  into a situation, where API doesn't work, but doesn't throw any exception,
  please [report this](https://github.com/iv4xr-project/iv4xr-se-plugin/issues/new).
