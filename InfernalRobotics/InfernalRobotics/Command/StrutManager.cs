using System;
using System.Collections.Generic;
using InfernalRobotics.Module;
using UniLinq;
using UnityEngine;
using Object = UnityEngine.Object;

namespace InfernalRobotics.Command
{
    /**
     * 
     */
    
    public class StrutManager
    {
        public static readonly string muMechToggle = "MuMechToggle";
        
        private float strutSpring;
        private float strutDamping;
        public bool hasHvDamping;

        private SpringJoint strutJoint;
        private SpringJoint strutHvJoint;

        private Part actuator;
        private Part strutStart;
        private Part strutEnd;
        private Part hvPart;
        
        public bool canCreateStrut;
        public bool canCreateHvStrut;
        
        public bool strutActive;
        public bool hvStrutActive;

        private bool moveMessageProcessed;
        private bool selfMoveMessageProcessed;

        private List<StrutManager> neighbors;
        private List<StrutManager> upwardBranch;
        
        public StrutManager()
        {
            neighbors = new List<StrutManager>();
            upwardBranch = new List<StrutManager>();
            strutActive = false;
            hvStrutActive = false;
            moveMessageProcessed = false;
            selfMoveMessageProcessed = false;
        }

        public void SetActuator(Part actuator)
        {
            this.actuator = actuator;
        }

        public bool Init(float spring, float damping, bool hasHvDamping)
        {
            strutSpring = spring;
            strutDamping = damping;
            this.hasHvDamping = hasHvDamping;

            return Init();
        }
        
        public bool Init()
        {
            
            Logger.Log($"StrutManager: Init: actuator={actuator.name}");
            canCreateStrut = strutSpring > 0.01f;

            strutStart = null;
            strutEnd = null;
            if (canCreateStrut)
            {
                strutStart = FindUpwardNonIRPart(actuator.parent);
                strutEnd = FindClosestNonIRPart(actuator);
                canCreateStrut = strutStart != null && strutEnd != null;
            }

            canCreateHvStrut = hasHvDamping && strutEnd != null;
            
            Logger.Log($"StrutManager({actuator.flightID}): Init: canCreate={canCreateStrut}; hasHv={hasHvDamping}; start={strutStart?.name}, end={strutEnd?.name}");
            neighbors = FindNeighbors(actuator);
            upwardBranch = FindUpwardBranch(actuator).Where(u => !neighbors.Contains(u)).ToList();

            CreateStrut();
            CreateHvStrut();

            Logger.Log($"StrutManager({actuator.flightID}): initialized; strut={strutJoint}, neighbors={neighbors.Count}");
            neighbors.ForEach(n => Logger.Log($"StrutManager({actuator.flightID}): neighbor:{n?.actuator?.name}, {n?.actuator?.flightID}"));
            upwardBranch.ForEach(n => Logger.Log($"StrutManager({actuator.flightID}): upward:{n?.actuator?.name}, {n?.actuator?.flightID}"));

            return canCreateStrut;
        }

        /**
         * Return true if there were some changes 
         */
        public bool OnVesselWasModified()
        {
            // get new endpoints
            Part oldStart = strutStart;
            Part oldEnd = strutEnd;

            bool oldCanCreateStrut = canCreateStrut;
            canCreateStrut = Init();
            Logger.Log($"OnVesselWasModified: canCreate={canCreateStrut}, oldCanCreate={oldCanCreateStrut}");
            Logger.Log($"OnVesselWasModified: newStart={strutStart}, newEnd={strutEnd}");
            Logger.Log($"OnVesselWasModified: oldStart={oldStart}, oldEnd={oldEnd}");
            
            // Check if new endpoints differ from old endpoints
            if (oldStart != strutStart || oldEnd != strutEnd)
            {
                // something has changed
                // destroy old strut, try to create new strut
                DestroyStrut();
                CreateStrut();
                // TODO: Add hvStrut support!
            }
            // nothing has changed
            return false;
        }

        public void Move()
        {
            if (selfMoveMessageProcessed)
                return;

            Move(this, true);
            selfMoveMessageProcessed = true;

            // propagate move message upwards
            upwardBranch.ForEach(u => u.Move(this, false));
        }

        private void Move(StrutManager source, bool isFromNeighbor)
        {
            // we should process move message from other joints once
            // if we receive self-move message, we should also process it once, regardless of messages from other joints
            
            if (moveMessageProcessed && !(source == this && !selfMoveMessageProcessed))
                return;
            moveMessageProcessed = true;

            Logger.Log($"StrutManager({actuator.flightID}): -> got move from {source.actuator.flightID}; hv={hvStrutActive}; hvStrut={strutHvJoint}");
            if (strutActive && (isFromNeighbor || source == this))
            {
                _DestroyStrut();
            }
            
            if (hvStrutActive)
            {
                _DestroyHvStrut();
            }
            neighbors.ForEach(n => {
                 if (n != source) n.Move(this, true);
            });
        }

        public void Stop()
        {
            if (!selfMoveMessageProcessed)
                return;
            Stop(this);
            selfMoveMessageProcessed = false;
            upwardBranch.ForEach(u => u.Stop(this));
        }

        private void Stop(StrutManager source)
        {
            if (!moveMessageProcessed)
                return;
            moveMessageProcessed = false;

            Logger.Log($"StrutManager({actuator.flightID}): -> got stop from {source.actuator.flightID}");
            Stop(this);            
            if (strutActive)
            {
                _CreateStrut();
            }

            if (hvStrutActive)
            {
                _CreateHvStrut();
            }

            neighbors.ForEach(n => {
                if (n != source) n.Stop(this);
            });
        }

        public void CreateStrut()
        {
            if (!HighLogic.LoadedSceneIsFlight || !canCreateStrut)
            {
                return;
            }
            _CreateStrut();
            strutActive = true;
        }
        
        private void _CreateStrut()
        {
            // Create strut between parts on both ends of the joint
            if (strutJoint != null)
            {
                return;
            }
            
            Logger.Log($"StrutManager({actuator.flightID}): Adding damping joint between {strutStart.name} and {strutEnd.name}");

            strutJoint = strutStart.gameObject.AddComponent<SpringJoint>();
            strutJoint.damper = actuator.breakingForce / 2 * strutDamping;
            strutJoint.spring = actuator.breakingForce / 2 * strutSpring;
            strutJoint.minDistance = 0.01f;
            strutJoint.maxDistance = 0.01f;
            strutJoint.tolerance = 0.01f;

            strutJoint.connectedBody = strutEnd.GetComponent<Rigidbody>();
            strutJoint.breakForce = actuator.breakingForce;
            strutJoint.breakTorque = actuator.breakingTorque;
            strutJoint.autoConfigureConnectedAnchor = true;
            strutJoint.axis = Vector3.one;
        }
        
        public void CreateHvStrut()
        {
            if (!HighLogic.LoadedSceneIsFlight || !canCreateHvStrut)
            {
                return;
            }
            Logger.Log($"StrutManager({actuator.flightID}): Looking for heaviest for {strutEnd.name}");
            try
            {
                hvPart = actuator.GetMassivePart(actuator.vessel.rootPart);
            }
            catch (NullReferenceException)
            {
                Logger.Log($"StrutManager({actuator.flightID}): Can't find heaviest part; v={FlightGlobals.ActiveVessel}, actuator={actuator}; falling back to root part");
                hvPart = actuator.vessel.rootPart;
            }
            _CreateHvStrut();
            hvStrutActive = true;
        }
        private void _CreateHvStrut() {

            if (strutEnd == null)
            {
                return;
            }

            if (strutHvJoint != null)
            {
                return;
            }
            
            if (hvPart == null)
            {
                return;
            }

            Logger.Log($"StrutManager({actuator.flightID}): Adding hard joint between {strutEnd.name} and {hvPart.name}");

            strutHvJoint = strutEnd.gameObject.AddComponent<SpringJoint>();
            strutHvJoint.damper = actuator.breakingForce / 8 * strutDamping;
            strutHvJoint.spring = actuator.breakingForce / 8 * strutSpring;
            strutHvJoint.minDistance = 0.05f;
            strutHvJoint.maxDistance = 0.05f;
            strutHvJoint.tolerance = 0.05f;

            strutHvJoint.connectedBody = hvPart.GetComponent<Rigidbody>();
            strutHvJoint.breakForce = actuator.breakingForce;
            strutHvJoint.breakTorque = actuator.breakingTorque;
            strutHvJoint.autoConfigureConnectedAnchor = true;
            strutHvJoint.axis = Vector3.one;
        }

        public void DestroyStrut()
        {
            _DestroyStrut();
            strutActive = false;
        }
        
        public void _DestroyStrut()
        {
            if (!HighLogic.LoadedSceneIsFlight || strutJoint == null)
            {
                return;
            }

            Logger.Log($"StrutManager({actuator.flightID}): Destroying soft joint {strutJoint}");
            Object.DestroyImmediate(strutJoint);
            strutJoint = null;
        }

        public void DestroyHvStrut()
        {
            _DestroyHvStrut();
            hvStrutActive = false;
        }
        
        private void _DestroyHvStrut()
        {
            if (!HighLogic.LoadedSceneIsFlight || strutHvJoint == null)
            {
                return;
            }

            Logger.Log($"StrutManager({actuator.flightID}): Destroying hard joint {strutJoint}");
            Object.DestroyImmediate(strutHvJoint);
            strutHvJoint = null;
        }

        private List<StrutManager> FindUpwardBranch(Part part)
        {
            // Find all possible actuators upward from here
            List<StrutManager> upwardList = new List<StrutManager>();
            foreach (Part p in part.children)
            {
                if (p.Modules.Contains(muMechToggle))
                {
                    upwardList.Add(((ModuleIRServo) (p.Modules[muMechToggle])).StrutManager);
                }
                upwardList.AddRange(FindUpwardBranch(p));
            }

            return upwardList;
        }
        
        private List<StrutManager> FindNeighbors(Part part)
        {
            List<StrutManager> neighborList = FindNeighborsUpward(part);
            neighborList.AddRange(FindNeighborsDownward(part.parent));
            neighborList.RemoveAll(n => n == null);
            return neighborList;
        }

        private List<StrutManager> FindNeighborsUpward(Part part)
        {
            List<StrutManager> neighborsList = new List<StrutManager>();
            // Find closest IR parts up the tree
            foreach (Part p in part.children)
            {
                Logger.Log($"StrutManager({actuator.flightID}): looking for neighbors upward: {p.name}");
                if (p.Modules.Contains(muMechToggle))
                {
                    neighborsList.Add(((ModuleIRServo) (p.Modules[muMechToggle])).StrutManager);
                }
            }

            return neighborsList;
        }

        private List<StrutManager> FindNeighborsDownward(Part part) 
        {
            List<StrutManager> neighborsList = new List<StrutManager>();

            // Find closes IR part down the tree
            if (part.Modules.Contains(muMechToggle))
            {
                Logger.Log($"StrutManager({actuator.flightID}): found neighbor downward: {part.name}");
                neighborsList.Add(((ModuleIRServo) (part.Modules[muMechToggle])).StrutManager);
            }
           
            return neighborsList;
        }
        
        public Part FindClosestNonIRPart(Part start)
        {
            if (start.children == null)
            {
                return null;
            }
            foreach (Part p in start.children)
            {
                if (!p.Modules.Contains(muMechToggle))
                {
                    Logger.Log($"StrutManager({actuator.flightID}): upward non-IR part: {p}");
                    return p;
                }

                Part deepPart = FindClosestNonIRPart(p);
                if (deepPart != null)
                {
                    return deepPart;
                }
            }
            return null;
        }

        public Part FindUpwardNonIRPart(Part start)
        {
            if (start == null)
            {
                return null;
            }

            if (start.Modules.Contains(muMechToggle))
            {
                return FindUpwardNonIRPart(start.parent);
            }

            Logger.Log($"StrutManager({actuator.flightID}): downward non-IR part: {start}");
            return start;
        }

        public float getCurrentSoftForce()
        {
            return strutJoint == null ? 0f : strutJoint.currentForce.magnitude;
        }

        public float getCurrentSoftTorque()
        {
            return strutJoint == null ? 0f : strutJoint.currentTorque.magnitude;
        }
        
        public float getCurrentHardForce()
        {
            return strutHvJoint == null ? 0f : strutHvJoint.currentForce.magnitude;
        }
        public float getCurrentHardTorque()
        {
            return strutHvJoint == null ? 0f : strutHvJoint.currentTorque.magnitude;
        }
    }
}