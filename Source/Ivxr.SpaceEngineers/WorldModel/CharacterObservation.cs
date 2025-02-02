﻿using System.Collections.Generic;

namespace Iv4xr.SpaceEngineers.WorldModel
{
    // Intended to be maximally compatible with Iv4xr.framework.world.WorldModel from the iv4XR Java framework.
    public class CharacterObservation: Entity
    {
        public PlainVec3D Extent; // Agent's dimensions (x,y,z size/2)
        public PlainVec3D Gravity;
        
        public Pose Camera;
        public bool JetpackRunning;
        public bool DampenersOn;
        public bool HelmetEnabled;
        public bool LightEnabled;
        public float CurrentLightPower;
        public float Health;
        public float Oxygen;
        public float Hydrogen;
        public float SuitEnergy;
        public float HeadLocalXAngle;
        public float HeadLocalYAngle;
        public Block TargetBlock;
        public UseObject TargetUseObject;
        public CharacterMovementEnum Movement;
        public Inventory Inventory;
        public BootsState BootsState;
        public Entity RelativeDampeningEntity;
        public CharacterMovementFlags MovementFlags;
        public PlainVec3D JetpackControlThrust;
        public PlainVec3D JetpackFinalThrust;
        public Entity CurrentWeapon;
    }
}
