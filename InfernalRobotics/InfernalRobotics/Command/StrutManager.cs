using System;
using System.Collections;
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

        private bool destroying;
        private bool destroyingHv;
        private bool creating;
        private bool creatingHv;

        private float springForce;

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

        public void Init(float spring, float damping, bool hasHvDamping)
        {
            strutSpring = spring;
            strutDamping = damping;
            this.hasHvDamping = hasHvDamping;

            Init();
        }
        
        public void Init()
        {
            Logger.Log($"StrutManager: Init: actuator={actuator.name}");
            
            springForce = actuator.breakingForce / 8 * strutSpring;
            
            InitStrut();

            InitHvStrut();
            
            Logger.Log($"StrutManager({actuator.flightID}): initialized; strut={strutJoint}, neighbors={neighbors.Count}");
            neighbors.ForEach(n => Logger.Log($"StrutManager({actuator.flightID}): neighbor:{n?.actuator?.name}, {n?.actuator?.flightID}"));
            upwardBranch.ForEach(n => Logger.Log($"StrutManager({actuator.flightID}): upward:{n?.actuator?.name}, {n?.actuator?.flightID}"));
        }

        private void InitStrut()
        {
            canCreateStrut = strutSpring > 0.01f;

            strutStart = null;
            strutEnd = null;
            if (canCreateStrut)
            {
                strutStart = FindUpwardNonIRPart(actuator.parent);
                strutEnd = FindClosestNonIRPart(actuator);
                canCreateStrut = strutStart != null && strutEnd != null;
            }

            FindAllNeighbors();

            if (strutJoint != null)
            {
                _DestroyStrut();
            }
            CreateStrut();
            Logger.Log($"StrutManager({actuator.flightID}): InitStrut: canCreate={canCreateStrut}; start={strutStart?.name}, end={strutEnd?.name}");
        }

        private void FindAllNeighbors()
        {
            neighbors = FindNeighbors(actuator);
            upwardBranch = FindUpwardBranch(actuator).Where(u => !neighbors.Contains(u)).ToList();
        }
        
        private void InitHvStrut()
        {
            if (strutEnd == null)
            {
                strutEnd = FindClosestNonIRPart(actuator);
            }
            canCreateHvStrut = hasHvDamping && strutEnd != null;
            
            if (strutHvJoint != null)
            {
                _DestroyHvStrut();
            }
            CreateHvStrut();
            Logger.Log($"StrutManager({actuator.flightID}): InitHvStrut: canCreate={canCreateHvStrut}; hasHv={hasHvDamping}; hv={hvPart?.name}, end={strutEnd?.name}");
        }


        public void OnPartDie()
        {
            DestroyStrut();
            DestroyHvStrut();
        }
        
        public void OnVesselWasModified()
        {
            // something has changed
            // destroy old strut, try to create new strut
            Logger.Log($"StrutManager({actuator.flightID}): vessel modified");
            
            FindAllNeighbors();
            Logger.Log($"StrutManager({actuator.flightID}): done checking neighbors");
            Logger.Log($"StrutManager({actuator.flightID}): vessel modified; start.v={strutStart?.vessel.name}, end.v={strutEnd?.vessel.name}, hv.v={hvPart?.vessel.name}");

            if (strutActive && (strutStart?.vessel != actuator.vessel || strutEnd?.vessel != actuator.vessel))
            {
                Logger.Log($"StrutManager({actuator.flightID}): vessel modified; start.v={strutStart?.vessel.name}, end.v={strutEnd?.vessel.name}, destroying strut");
                DestroyStrut();
                actuator.StartCoroutine(WaitAndCreateStrut());
            }

            if (hvStrutActive && (strutEnd?.vessel != actuator.vessel || hvPart?.vessel != actuator.vessel))
            {
                Logger.Log($"StrutManager({actuator.flightID}): vessel modified; hv.v={hvPart?.vessel}, end.v={strutEnd?.vessel}, destroying hv strut");
                DestroyHvStrut();
                actuator.StartCoroutine(WaitAndCreateHvStrut());
            }
        }

        private IEnumerator WaitAndCreateHvStrut()
        {
            yield return new WaitForFixedUpdate();
            InitHvStrut();
        }
        
        private IEnumerator WaitAndCreateStrut()
        {
            yield return new WaitForFixedUpdate();
            InitStrut();
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
            
            if (!actuator.vessel.IsControllable || (moveMessageProcessed && !(source == this && !selfMoveMessageProcessed)))
                return;
            moveMessageProcessed = true;

            Logger.Log($"StrutManager({actuator.flightID}): -> got move from {source.actuator.flightID}; hv={hvStrutActive}; hvStrut={strutHvJoint}; ctrl={actuator.vessel.IsControllable}");
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
            if (!moveMessageProcessed || !actuator.vessel.IsControllable)
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
            strutJoint.spring = 1f;
            strutJoint.minDistance = 0.01f;
            strutJoint.maxDistance = 0.01f;
            strutJoint.tolerance = 0.01f;

            strutJoint.connectedBody = strutEnd.GetComponent<Rigidbody>();
            strutJoint.breakForce = actuator.breakingForce;
            strutJoint.breakTorque = actuator.breakingTorque;
            strutJoint.autoConfigureConnectedAnchor = true;
            strutJoint.axis = Vector3.one;
            
            destroying = false;
            creating = true;
            actuator.StartCoroutine(TightenStrut());
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
                hvPart = getHvPartInStage();
                if (hvPart == actuator)
                {
                    Logger.Log($"StrutManager({actuator.flightID}): Heaviest part is actuator; falling back to root part {actuator.vessel.rootPart.name}");
                    hvPart = actuator.vessel.rootPart;
                }
            }
            catch (NullReferenceException)
            {
                Logger.Log($"StrutManager({actuator.flightID}): Can't find heaviest part; v={FlightGlobals.ActiveVessel}, actuator={actuator}; falling back to root part");
                hvPart = actuator.vessel.rootPart;
            }
            Logger.Log($"StrutManager({actuator.flightID}): Heaviest part; actuator.v={actuator.vessel.name}, part={hvPart}, part.v={hvPart.vessel.name}");

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
            strutHvJoint.spring = 1f;
            strutHvJoint.minDistance = 0.05f;
            strutHvJoint.maxDistance = 0.05f;
            strutHvJoint.tolerance = 0.05f;

            strutHvJoint.connectedBody = hvPart.GetComponent<Rigidbody>();
            strutHvJoint.breakForce = actuator.breakingForce;
            strutHvJoint.breakTorque = actuator.breakingTorque;
            strutHvJoint.autoConfigureConnectedAnchor = true;
            strutHvJoint.axis = Vector3.one;

            destroyingHv = false;
            creatingHv = true;
            actuator.StartCoroutine(TightenHvStrut());
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

            if (destroying)
            {
                return;
            }
            creating = false;
            destroying = true;
            actuator.StartCoroutine(LooseStrut());
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

            if (destroyingHv)
            {
                return;
            }
            creatingHv = false;
            destroyingHv = true;
            actuator.StartCoroutine(LooseHvStrut());
        }

        private IEnumerator LooseStrut()
        {
            while (destroying)
            {
                if (strutJoint.spring > 1f)
                {
                    strutJoint.spring -= springForce / 10;
                    yield return new WaitForFixedUpdate();
                }
                else
                {
                    Logger.Log($"StrutManager({actuator.flightID}): Destroying soft joint {strutJoint}");
                    Object.Destroy(strutJoint);
                    strutJoint = null;
                    destroying = false;
                    yield break;
                }
            }
        }
        
        
        private IEnumerator LooseHvStrut()
        {
            while (destroyingHv)
            {
                if (strutHvJoint.spring > 1f)
                {
                    strutHvJoint.spring -= springForce / 10;
                    yield return new WaitForFixedUpdate();
                }
                else
                {
                    Logger.Log($"StrutManager({actuator.flightID}): Destroying hard joint {strutHvJoint}");
                    Object.Destroy(strutHvJoint);
                    strutHvJoint= null;
                    destroyingHv = false;
                    yield break;
                }
            }
        }
        
        private IEnumerator TightenStrut()
        {
            while (creating)
            {
                if (strutJoint.spring < springForce)
                {
                    strutJoint.spring += springForce / 50;
                    yield return new WaitForFixedUpdate();
                }
                else
                {
                    Logger.Log($"StrutManager({actuator.flightID}): Created soft joint {strutJoint}");
                    creating = false;
                    yield break;
                }
            }
        }

        private IEnumerator TightenHvStrut()
        {
            while (creatingHv)
            {
                if (strutHvJoint.spring < springForce)
                {
                    strutHvJoint.spring += springForce / 50;
                    yield return new WaitForFixedUpdate();
                }
                else
                {
                    Logger.Log($"StrutManager({actuator.flightID}): Created hard joint {strutHvJoint}");
                    creatingHv = false;
                    yield break;
                }
            }
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
            if (start?.children == null)
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

        public Part getHvPartInStage()
        {
            float maxMass = 0f;
            Part hv = null;
            foreach (Part p in actuator.vessel.parts)
            {
                if (p.inverseStage != actuator.inverseStage)
                {
                    continue;
                }

                if (maxMass < p.mass)
                {
                    maxMass = p.mass;
                    hv = p;
                }
            }

            Logger.Log($"StrutManager({actuator.flightID}):  heavisest part in inv stage {actuator.inverseStage}: {hv.name} ({maxMass})");
            return hv;
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