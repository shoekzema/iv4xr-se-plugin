﻿using System;
using System.Collections.Generic;
using System.Linq;
using Iv4xr.PluginLib;
using Sandbox.Game.Entities.Character;
using Sandbox.Game.Multiplayer;
using Sandbox.Game.World;
using VRage;
using VRageMath;

namespace Iv4xr.SePlugin.Control
{
    public interface IGameSession
    {
        MyCharacter Character { get; }
        void SetCharacter(long characterId);

        MyCharacter CreateCharacter(string name, Vector3D position, Vector3D orientationForward,
            Vector3D orientationUp);

        long CurrentCharacterId { get; }

        void RemoveCharacter(long id);
        string MainCharacterId();
    }

    public class GameSession : IGameSession
    {
        private const long NoCharacter = -1;
        private long m_currentCharacterId = NoCharacter;
        private readonly Random m_random = new Random();

        private MyPlayerCollection Players => MySession.Static.Players;

        public long CurrentCharacterId
        {
            get
            {
                Init();
                return m_currentCharacterId;
            }
            private set => m_currentCharacterId = value;
        }

        private void CheckCharacterExists(long characterId)
        {
            if (GetCharacterByIdOrNull(characterId) == null)
            {
                throw new InvalidOperationException(
                    $"Character ({characterId}) not found! Only found: {FoundCharacterIds()}"
                    );
            }
        }

        public void SetCharacter(long characterId)
        {
            CheckCharacterExists(characterId);
            CurrentCharacterId = characterId;
        }

        public MyCharacter CreateCharacter(string name, Vector3D position, Vector3D orientationForward,
            Vector3D orientationUp)
        {
            var matrix = MatrixD.CreateWorld(position: position, forward: orientationForward, up: orientationUp);
            var identityId = m_random.Next(0, int.MaxValue);
            var steamId = (ulong)m_random.Next(0, int.MaxValue);
            var character = AgentSpawner.SpawnAgent(
                steamId: steamId, name: name, color: Color.White, startPosition: matrix, identityId: identityId
            );
            var characterId = character.CharacterId();
            CurrentCharacterId = characterId;
            return character;
        }

        public void RemoveCharacter(long characterId)
        {
            CheckCharacterExists(characterId);
            if (characterId == GetMainCharacterId())
            {
                // Removing main character in theory kinda works (game doesn't crash, camera movements keep working),
                // but let's not allow it unless we find it useful.
                throw new InvalidOperationException("Cannot remove the main character!");
            }

            var character = GetCharacterById(characterId);
            character.GetPlayerId(out var playerId);
            var player = Players.GetPlayerById(playerId);
            Players.RemovePlayer(player);
            // TODO: Possibly remove identity too. Something like:
            // Players.RemoveIdentity(player.Identity.IdentityId);
            SwitchToMainCharacter();
        }

        public string MainCharacterId()
        {
            return GetMainCharacterId().ToString();
        }

        private MyCharacter GetCharacterByIdOrNull(long characterId)
        {
            //TODO: There is probably a better/faster way to get all current characters (MyCharacter instances) in the game.
            return Players.GetAllIdentities()
                    .Select(i => i.Character)
                    .Where(c => c?.GetIdentity() != null)
                    .FirstOrDefault(c => c.CharacterId() == characterId);
        }

        private string FoundCharacterIds()
        {
            return string.Join(",", Sync.Players.GetOnlinePlayers()
                    .Where(p => p.Character != null)
                    .Select(p => p.Character)
                    .Where(c => c?.GetIdentity() != null).Select(c => c.CharacterId()).ToList());
        }

        private MyCharacter GetCharacterById(long characterId)
        {
            return GetCharacterByIdOrNull(characterId) ??
                   throw new InvalidOperationException(
                       $"Character ({characterId}) not found! Only found: {FoundCharacterIds()}"
                       );
        }

        public MyCharacter Character
        {
            get
            {
                Init();
                return GetCharacterById(CurrentCharacterId);
            }
        }

        public bool Initialized()
        {
            return m_currentCharacterId != NoCharacter;
        }


        private void Init()
        {
            if (!Initialized() && !DebugInfoCreator.Create().IsDedicated)
            {
                SwitchToMainCharacter();
            }
        }

        private long GetMainCharacterId()
        {
            if (!MyVRage.Platform.SessionReady)
            {
                throw new InvalidOperationException("Session is not ready!");
            }

            var session = MySession.Static
                    .ThrowIfNull("MySession.Static");
            var players = session.Players
                    .ThrowIfNull("Sync.Players");
            var onlinePlayers = players.GetOnlinePlayers()
                    .ThrowIfNull("Sync.Players.GetOnlinePlayers");
            return onlinePlayers
                           .FirstOrDefault(p => p.IsLocalPlayer)
                           ?.Character?.CharacterId() ??
                   throw new NullReferenceException("The main character is missing!");
        }

        private void SwitchToMainCharacter()
        {
            CurrentCharacterId = GetMainCharacterId();
        }

        public void EndSession()
        {
            CurrentCharacterId = NoCharacter;
        }
    }
}
