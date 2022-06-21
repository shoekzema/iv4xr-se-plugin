﻿using System.Collections.Generic;
using System.Linq;
using Iv4xr.PluginLib;
using Iv4xr.SpaceEngineers;
using Iv4xr.SpaceEngineers.WorldModel;
using Sandbox.Graphics.GUI;
using SpaceEngineers.Game.GUI;

namespace Iv4xr.SePlugin.Control.Screen
{
    public class MedicalsScreen : AbstractScreen<MyGuiScreenMedicals, object>, IMedicals
    {
        public void SelectRespawn(int roomIndex)
        {
            var table = Screen.Table("m_respawnsTable");
            table.SelectedRowIndex = roomIndex;
            Screen.CallMethod<object>("OnTableItemSelected", new object[] { table, new MyGuiControlTable.EventArgs()});
        }
        
        public void Respawn()
        {
            Screen.ClickButton("m_respawnButton");
        }

        public void ShowMessageOfTheDay()
        {
            Screen.ClickButton("m_MotdButton");
        }

        public void Refresh()
        {
            Screen.ClickButton("m_refreshButton");
        }

        public void ChooseFaction(int factionIndex)
        {
            Screen.Table("m_factionsTable").SelectedRowIndex = factionIndex;
            Screen.ClickButton("m_selectFactionButton");
        }

        public List<MedicalRoom> MedicalRooms()
        {
            return MedicalRoomRows().Select(row => new MedicalRoom()
                    {
                        Name = row.GetCell(0).Text.ToString(),
                        AvailableIn = row.GetCell(1).Text.ToString()
                    }
            ).ToList();
        }

        public List<Faction> Factions()
        {
            return FactionRows().Select(row => new Faction()
                    {
                        Tag = row.GetCell(0).Text?.ToString() ?? "",
                        Name = row.GetCell(1).Text?.ToString() ?? "",
                    }
            ).ToList();
        }

        private List<MyGuiControlTable.Row> MedicalRoomRows()
        {
            return Screen.Table("m_respawnsTable").RowsAsList();
        }

        private List<MyGuiControlTable.Row> FactionRows()
        {
            return Screen.Table("m_factionsTable").RowsAsList();
        }
    }
}
