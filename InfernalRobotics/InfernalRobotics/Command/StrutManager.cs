using System;
using System.Collections;
using InfernalRobotics.Module;
using UnityEngine;
using Object = UnityEngine.Object;

namespace InfernalRobotics.Command
{
    /**
     *   Class for taking care of spring joints (struts) between parts, attached to both sides of actuator  
     */
    
    public class StrutManager
    {
        public static readonly string muMechToggle = "MuMechToggle";

        private ModuleIRServo module;
        
        private float strutSpring;
        private float strutDamping;
        public bool hasHvDamping;

        private SpringJoint strutJoint;
        private SpringJoint strutHvJoint;

        public Part actuator;
        private Part strutStart;
        private Part strutEnd;
        private Part hvPart;
        
        public bool canCreateStrut;
        public bool canCreateHvStrut;
        
        public bool strutActive;
        public bool hvStrutActive;

        private bool destroying;
        private bool destroyingHv;
        private bool creating;
        private bool creatingHv;

        private float springForce;

        public MessageProcessor MessageProcessor;
        
        public StrutManager(ModuleIRServo module)
        {
            this.module = module;
            
            strutActive = false;
            hvStrutActive = false;
            MessageProcessor = new MessageProcessor(this);
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
            MessageProcessor.Init();

            springForce = actuator.breakingForce / 8 * strutSpring;
            
            InitStrut();
            InitHvStrut();
            
            Logger.Log(
                $"StrutManager({actuator.flightID}): initialized; strut={strutJoint}, neighbors={MessageProcessor.neighbors.Count}");
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

            if (strutJoint != null)
            {
                _DestroyStrut();
            }
            CreateStrut(true);
            Logger.Log(
                $"StrutManager({actuator.flightID}): InitStrut: canCreate={canCreateStrut}; start={strutStart?.name}, end={strutEnd?.name}");
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
            CreateHvStrut(true);
            Logger.Log(
                $"StrutManager({actuator.flightID}): InitHvStrut: canCreate={canCreateHvStrut}; hasHv={hasHvDamping}; hv={hvPart?.name}, end={strutEnd?.name}");
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
            
            MessageProcessor.FindNeighbors(actuator);
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

        internal void Move(bool dropSoftStrut)
        {
            // if this move signal is from a neighbor, we should drop only hv strut
            if (!actuator.vessel.IsControllable)
            {
                return;
            }

            Logger.Log(
                $"StrutManager({actuator.flightID}): -> got move; hv={hvStrutActive}; hvStrut={strutHvJoint}; dropSoft={dropSoftStrut}");
            if (strutActive && dropSoftStrut)
            {
                creating = false;
                _DestroyStrut();
            }
            
            if (hvStrutActive)
            {
                creatingHv = false;
                _DestroyHvStrut();
            }
        }

        internal void Stop(bool isFullStop)
        {
            if (!actuator.vessel.IsControllable)
                return;

            Logger.Log($"StrutManager({actuator.flightID}): got stop; full={isFullStop}");
            
            if (strutActive)
            {
                // Strut may be destroying. If so, stop loosening it and re-create
                if (destroying)
                {
                    destroying = false;
                    Object.Destroy(strutJoint);
                    strutJoint = null;
                }

                _CreateStrut(false);
            }

            if (hvStrutActive && isFullStop)
            {
                if (destroyingHv)
                {
                    destroyingHv = false;
                    Object.Destroy(strutHvJoint);
                    strutHvJoint = null;
                }
                
                _CreateHvStrut(false);
            }
        }

        public void CreateStrut(bool immediate)
        {
            if (!HighLogic.LoadedSceneIsFlight || !canCreateStrut)
            {
                return;
            }
            _CreateStrut(immediate);
            strutActive = true;
        }
        
        private void _CreateStrut(bool immediate)
        {
            // Create strut between parts on both ends of the joint
            if (strutJoint != null)
            {
                return;
            }
            
            Logger.Log($"StrutManager({actuator.flightID}): Adding soft strut between {strutStart.name} and {strutEnd.name}");

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
            
            actuator.StartCoroutine(TightenStrut(immediate));
        }
        
        public void CreateHvStrut(bool immediate)
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

            _CreateHvStrut(immediate);
            hvStrutActive = true;
        }
        private void _CreateHvStrut(bool immediate) {

            if (strutHvJoint != null || strutEnd == null || hvPart == null)
            {
                return;
            }

            Logger.Log($"StrutManager({actuator.flightID}): Adding Hard strut between {strutEnd.name} and {hvPart.name}");

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
            
            actuator.StartCoroutine(TightenHvStrut(immediate));
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
                    Logger.Log($"StrutManager({actuator.flightID}): Destroying soft strut {strutJoint}");
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
                    Logger.Log($"StrutManager({actuator.flightID}): Destroying Hard strut {strutHvJoint}");
                    Object.Destroy(strutHvJoint);
                    strutHvJoint= null;
                    destroyingHv = false;
                    yield break;
                }
            }
        }
        
        private IEnumerator TightenStrut(bool immediate)
        {
            if (immediate)
            {
                strutJoint.spring = springForce;
                yield break;
            }
            
            while (creating)
            {
                float pos = module.rotateJoint ? module.rotation : module.translation;
                Logger.Log($"StrutManager({actuator.flightID}): pos = {pos}; waiting for interpolator (active={module.Interpolator.Active})");

                if (module.Interpolator.Active)
                {
                    if (!creating)
                    {
                        yield break;
                    }
                    yield return new WaitForFixedUpdate();
                }

                if (strutJoint.spring < springForce)
                {
                    if (!creating)
                    {
                        yield break;
                    }
                    strutJoint.spring += springForce / 50;
                    yield return new WaitForFixedUpdate();
                }
                
                if (!creating)
                {
                    yield break;
                }
                Logger.Log($"StrutManager({actuator.flightID}): Re-creating soft strut {strutJoint}");
                Object.Destroy(strutJoint);
                strutJoint = null;
                _CreateStrut(true);
                Logger.Log($"StrutManager({actuator.flightID}): Created soft strut {strutJoint}");
                creating = false;
                yield break;
            }
        }

        private IEnumerator TightenHvStrut(bool immediate)
        {
            if (immediate)
            {
                strutHvJoint.spring = springForce;
                yield break;
            }
            while (creatingHv)
            {
                if (module.Interpolator.Active)
                {
                    if (!creatingHv)
                    {
                        yield break;
                    }
                    yield return new WaitForFixedUpdate();
                }
                
                if (strutHvJoint.spring < springForce)
                {
                    if (!creatingHv)
                    {
                        yield break;
                    }
                    strutHvJoint.spring += springForce / 50;
                    yield return new WaitForFixedUpdate();
                }
                
                if (!creatingHv)
                {
                    yield break;
                }
                Logger.Log($"StrutManager({actuator.flightID}): Re-creating hard strut {strutHvJoint}");
                Object.Destroy(strutHvJoint);
                strutHvJoint = null;
                _CreateHvStrut(true);
                Logger.Log($"StrutManager({actuator.flightID}): Created hard strut {strutHvJoint}");
                creatingHv = false;
                yield break;
            }
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