using System;
using System.Collections;
using System.Collections.Generic;
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

        private bool drawStruts = false;
        
        private ModuleIRServo module;
        
        private float strutSpring;
        private float strutDamping;
        public bool hasHvDamping;

        private List<SpringJoint> strutJoint = new List<SpringJoint>();
        private List<SpringJoint> strutHvJoint = new List<SpringJoint>();

        private Dictionary<int, GameObject> LRs = new Dictionary<int, GameObject>();

        public Part actuator;
        private List<Part> strutStart = new List<Part>();
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
                $"StrutManager({actuator.flightID}): initialized; # struts={strutJoint.Count}, neighbors={MessageProcessor.neighbors.Count}");
         }

        private void InitStrut()
        {
            canCreateStrut = strutSpring > 0.01f;

            strutStart = null;
            strutEnd = null;
            if (canCreateStrut)
            {
                strutStart = FindClosestNonIRParts(actuator);
                strutEnd = FindDownwardNonIRPart(actuator.parent);
                canCreateStrut = strutStart != null && strutEnd != null;
            }

            if (strutJoint != null)
            {
                _DestroyStrut();
            }
            CreateStrut(true);
            Logger.Log(
                $"StrutManager({actuator.flightID}): InitStrut: canCreate={canCreateStrut}; # parts={strutStart?.Count}, end={strutEnd?.name}");
        }

        private void InitHvStrut()
        {
            if (strutStart == null)
            {
                strutStart = FindClosestNonIRParts(actuator);
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
            if (!HighLogic.LoadedSceneIsFlight)
            {
                return;
            }
            // something has changed
            // destroy old strut, try to create new strut
            Logger.Log($"StrutManager({actuator.flightID}): vessel modified");
            
            MessageProcessor.FindNeighbors(actuator);
            //Logger.Log($"StrutManager({actuator.flightID}): vessel modified; start.v={strutEnd?.vessel.name}, end.v={strutEnd?.vessel.name}, hv.v={hvPart?.vessel.name}");

            for (int i = 0; i < strutJoint.Count; i++)
            {
                Part startPart = strutJoint[i].gameObject.GetComponent<Part>();
                if (strutActive && (startPart.vessel != actuator.vessel || strutEnd?.vessel != actuator.vessel))
                {
                    Logger.Log(
                        $"StrutManager({actuator.flightID}): vessel modified; start.v={startPart?.vessel.name}, end.v={strutEnd?.vessel.name}, destroying strut");
                    DestroyStrut();
                    actuator.StartCoroutine(WaitAndCreateStrut());
                }
            }

            for (int i = 0; i < strutHvJoint.Count; i++)
            {
                Part startPart = strutHvJoint[i].gameObject.GetComponent<Part>();
                if (hvStrutActive && (startPart?.vessel != actuator.vessel || hvPart?.vessel != actuator.vessel))
                {
                    Logger.Log(
                        $"StrutManager({actuator.flightID}): vessel modified; hv.v={hvPart?.vessel}, end.v={startPart?.vessel}, destroying hv strut");
                    DestroyHvStrut();
                    actuator.StartCoroutine(WaitAndCreateHvStrut());
                }
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
                $"StrutManager({actuator.flightID}): -> got move; hv={hvStrutActive}; hvStrut={strutHvJoint.Count}; dropSoft={dropSoftStrut}");
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
                    for (int i = 0; i < strutJoint.Count; i++)
                    {
                        Object.Destroy(strutJoint[i]);
                    }

                    strutJoint.Clear();
                    LRDestroy();;
                }

                _CreateStrut(false);
            }

            if (hvStrutActive && isFullStop)
            {
                if (destroyingHv)
                {
                    destroyingHv = false;
                    for (int i = 0; i < strutHvJoint.Count; i++)
                    {
                        Object.Destroy(strutHvJoint[i]);
                    }
                    strutHvJoint.Clear();
                }
                
                _CreateHvStrut(false);
            }
        }

        private void SetJointDistance(Part from, Part to, SpringJoint joint, float delta = 0.2f)
        {
            float distance = 0f;
            //Vector3.Distance(
                //to.transform.InverseTransformPoint(to.rb.centerOfMass),
                //from.transform.InverseTransformPoint(from.rb.centerOfMass));
            //Logger.Log($"StrutManager({actuator.flightID}): ({from.transform.InverseTransformPoint(from.rb.centerOfMass)}) -> ({to.transform.InverseTransformPoint(to.rb.centerOfMass)}) distance={distance}");

            joint.minDistance = distance - delta;
            joint.maxDistance = distance + delta;
            joint.tolerance = 0.01f;
            //joint.anchor = from.rb.centerOfMass;
            //joint.connectedAnchor = to.rb.centerOfMass;
        }

        private SpringJoint ConfigureJoint(Part from, Part to, float damping)
        {
            SpringJoint joint = from.gameObject.AddComponent<SpringJoint>();
            joint.damper = damping;
            joint.spring = 1f;

            SetJointDistance(from, to, joint);

            joint.autoConfigureConnectedAnchor = true;
            joint.connectedBody = to.rb;
            joint.breakForce = actuator.breakingForce;
            joint.breakTorque = actuator.breakingTorque;
            joint.axis = Vector3.one;
            joint.enablePreprocessing = true;

            return joint;
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
            if (creating || strutJoint.Count > 0)
            {
                return;
            }
            // Create strut between parts on both ends of the joint
            LRDestroy();
            strutJoint.Clear();
            for (int i = 0; i < strutStart.Count; i++)
            {
                Logger.Log(
                    $"StrutManager({actuator.flightID}): Adding soft strut between {strutStart[i].name} ({strutStart[i].flightID}) and {strutEnd.name}");

                SpringJoint joint = ConfigureJoint(strutStart[i], strutEnd, actuator.breakingForce / 2 * strutDamping); 
                strutJoint.Add(joint);
            }

            LRDraw();
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
                hvPart = GetHvPartInStage();
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

            if (strutEnd == null || hvPart == null || creatingHv || strutHvJoint.Count > 0)
            {
                return;
            }

            strutHvJoint.Clear();
            for (int i = 0; i < strutStart.Count; i++)
            {
                Logger.Log(
                    $"StrutManager({actuator.flightID}): Adding Hard strut between {strutStart[i].name} ({strutStart[i].flightID}) and {hvPart.name}");

                SpringJoint joint = ConfigureJoint(strutStart[i], hvPart, actuator.breakingForce / 8 * strutDamping);
                strutHvJoint.Add(joint);
            }

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
            Logger.Log(
                $"StrutManager({actuator.flightID}): soft joint count={strutJoint.Count}; destroying={destroying}");
            if (!HighLogic.LoadedSceneIsFlight || strutJoint.Count == 0)
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
            if (!HighLogic.LoadedSceneIsFlight || strutHvJoint.Count == 0)
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
                int nLoosened = 0;
                for (int i = 0; i < strutJoint.Count; i++)
                {
                    nLoosened += _LooseSingleStrut(strutJoint[i]);
                }
                
                if (nLoosened > 0) 
                {
                    yield return new WaitForFixedUpdate();
                }
                else
                {
                    strutJoint.Clear();
                    destroying = false;
                    LRDestroy();
                    yield break;
                }
            }
        }

        private int _LooseSingleStrut(SpringJoint strut)
        {
            if (strut.spring > 1f)
            {
                strut.spring -= springForce / 10;
                return 1;
            }

            Logger.Log($"StrutManager({actuator.flightID}): Destroying strut {strut}");
            Object.Destroy(strut);
            return 0;
        }
        
        private IEnumerator LooseHvStrut()
        {
            while (destroyingHv)
            {
                int nLoosened = 0;
                for (int i = 0; i < strutHvJoint.Count; i++)
                {
                    nLoosened += _LooseSingleStrut(strutHvJoint[i]);
                }
                
                if (nLoosened > 0) 
                {
                    yield return new WaitForFixedUpdate();
                }
                else
                {
                    strutHvJoint.Clear();
                    destroyingHv = false;
                    yield break;
                }
            }
        }

        private int _TightenStrut(SpringJoint strut)
        {
            if (strut.spring < springForce)
            {
                strut.spring += springForce / 50;
                return 1;
            }
            return 0;
        }
        private IEnumerator TightenStrut(bool immediate)
        {
            if (immediate)
            {
                for (int i = 0; i < strutJoint.Count; i++)
                {
                    strutJoint[i].spring = springForce;
                }
                Logger.Log($"StrutManager({actuator.flightID}): set force={springForce} on all struts");
                creating = false;
                yield break;
            }
            
            while (creating)
            {
                if (module.Interpolator.Active)
                {
                    if (!creating)
                    {
                        yield break;
                    }
                    yield return new WaitForFixedUpdate();
                }

                int nTightened = 0;
                for (int i = 0; i < strutJoint.Count; i++)
                {
                    nTightened += _TightenStrut(strutJoint[i]);
                }

                if (nTightened > 0)
                {
                    if (!creating)
                    {
                        yield break;
                    }
                    yield return new WaitForFixedUpdate();
                }
                else
                {
                    Logger.Log($"StrutManager({actuator.flightID}): Setting new soft strut tolerance after tightening");
                    for (int i = 0; i < strutJoint.Count; i ++)
                    {
                        Part to = strutJoint[i].connectedBody.gameObject.GetComponent<Part>();
                        Part from = strutJoint[i].gameObject.GetComponent<Part>();
                        SetJointDistance(from, to, strutJoint[i], 0.02f);
                    }
                    creating = false;
                    yield break;
                }
            }
        }

        private IEnumerator TightenHvStrut(bool immediate)
        {
            if (immediate)
            {
                for (int i = 0; i < strutHvJoint.Count; i++)
                {
                    strutHvJoint[i].spring = springForce;
                }
                Logger.Log($"StrutManager({actuator.flightID}): set force={springForce} on all hvStruts");
                creatingHv = false;
                yield break;
            }
            
            while (creatingHv)
            {
                if (module.Interpolator.Active)
                {
                    if (!creating)
                    {
                        yield break;
                    }
                    yield return new WaitForFixedUpdate();
                }

                int nTightened = 0;
                for (int i = 0; i < strutHvJoint.Count; i++)
                {
                    nTightened += _TightenStrut(strutHvJoint[i]);
                }

                if (nTightened > 0)
                {
                    if (!creating)
                    {
                        yield break;
                    }
                    yield return new WaitForFixedUpdate();
                }
                else
                {
                    Logger.Log($"StrutManager({actuator.flightID}): Setting new hard strut tolerance after tightening");
                    for (int i = 0; i < strutHvJoint.Count; i ++)
                    {
                        Part to = strutHvJoint[i].connectedBody.gameObject.GetComponent<Part>();
                        Part from = strutHvJoint[i].gameObject.GetComponent<Part>();
                        SetJointDistance(from, to, strutHvJoint[i], 0.02f);
                    }
                    creatingHv = false;
                    yield break;
                }
            }
        }

        
        // ReSharper disable once InconsistentNaming
        public List<Part> FindClosestNonIRParts(Part start)
        {
            if (start == null || start?.children.Count == 0)
            {
                return null;
            }

            List<Part> closestParts = new List<Part>();
            foreach (Part p in start.children)
            {
                if (!p.Modules.Contains(muMechToggle))
                {
                    Logger.Log($"StrutManager({actuator.flightID}): upward non-IR part: {p}");
                    closestParts.Add(p);
                }
                else
                {
                    closestParts.AddRange(FindClosestNonIRParts(p));
                }
            }
            return closestParts;
        }

        // ReSharper disable once InconsistentNaming
        public Part FindDownwardNonIRPart(Part start)
        {
            if (start == null)
            {
                return null;
            }

            if (start.Modules.Contains(muMechToggle))
            {
                return FindDownwardNonIRPart(start.parent);
            }

            Logger.Log($"StrutManager({actuator.flightID}): downward non-IR part: {start}");
            return start;
        }

        public Part GetHvPartInStage()
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

            Logger.Log($"StrutManager({actuator.flightID}):  heavisest part in inv stage {actuator.inverseStage}: {hv?.name} ({maxMass})");
            return hv;
        }

        public float GetCurrentSoftForce()
        {
            return strutJoint.Count == 0 || strutJoint[0] == null ? 0f : strutJoint[0].currentForce.magnitude;
        }

        public float GetCurrentSoftTorque()
        {
            return strutJoint.Count == 0 || strutJoint[0] == null  ? 0f : strutJoint[0].currentTorque.magnitude;
        }
        
        public float GetCurrentHardForce()
        {
            return strutHvJoint.Count == 0  || strutHvJoint[0] == null ? 0f : strutHvJoint[0].currentForce.magnitude;
        }
        public float GetCurrentHardTorque()
        {
            return strutHvJoint.Count == 0  || strutHvJoint[0] == null ? 0f : strutHvJoint[0].currentTorque.magnitude;
        }
        
        internal void LRDraw()
        {
            if (!drawStruts)
            {
                return;
            }
            LineRenderer lr;

            for (int i = 0; i < strutJoint.Count; i ++)
            {
                SpringJoint j = strutJoint[i];
                int id = j.GetInstanceID();

                if (LRs.ContainsKey(id))
                {
                    lr = LRs[id].GetComponent<LineRenderer>();
                }
                else
                {
                    GameObject go = new GameObject(id.ToString());
                    Part p = j.gameObject.GetComponent<Part>();

                    Logger.Log($"StrutManager({actuator.flightID}): adding GO to strut id={id}; part: {p}; connected: {j.connectedBody.gameObject.GetComponent<Part>()}");
                    Logger.Log($"StrutManager({actuator.flightID}): LR->WS({j.transform.TransformPoint(j.anchor)}) ({j.connectedBody.transform.TransformPoint(j.connectedAnchor)})");
                    Logger.Log($"StrutManager({actuator.flightID}): LR->LS({j.anchor})");

                    Logger.Log($"StrutManager({actuator.flightID}): local transform: {j.transform.position}; connected local: {j.connectedBody.transform.position}");


                    lr = go.AddComponent<LineRenderer>();
                    lr.positionCount = 2;
                    lr.startColor = Color.red;
                    lr.endColor = lr.startColor;
                    lr.startWidth = 0.03f;
                    lr.endWidth = 0.03f;
                    lr.useWorldSpace = true;
                    lr.material = new Material(Shader.Find("Particles/Additive"));
                    LRs[id] = go;
                }

                lr.SetPosition(0, j.transform.TransformPoint(j.anchor));
                lr.SetPosition(1, j.connectedBody.transform.TransformPoint(j.connectedAnchor));
                //lr.SetPosition(1, j.connectedBody.transform.position);
            }
            
            for (int i = 0; i < strutHvJoint.Count; i ++)
            {
                SpringJoint j = strutHvJoint[i];
                int id = j.GetInstanceID();

                if (LRs.ContainsKey(id))
                {
                    lr = LRs[id].GetComponent<LineRenderer>();
                }
                else
                {
                    GameObject go = new GameObject(id.ToString());
                    Part p = j.gameObject.GetComponent<Part>();

                    // Logger.Log($"StrutManager({actuator.flightID}): adding GO to strut id={id}; part: {p}; connected: {j.connectedBody.gameObject.GetComponent<Part>()}");
                    // Logger.Log($"StrutManager({actuator.flightID}): LR->WS({j.transform.TransformPoint(j.anchor)}) ({j.connectedBody.transform.TransformPoint(j.connectedAnchor)})");
                    // Logger.Log($"StrutManager({actuator.flightID}): LR->LS({j.anchor}) ({j.connectedAnchor})");
                    //
                    // Logger.Log($"StrutManager({actuator.flightID}): local transform: {j.transform.position}; connected local: {j.connectedBody.transform.position}");

                    lr = go.AddComponent<LineRenderer>();
                    lr.positionCount = 2;
                    lr.startColor = Color.green;
                    lr.endColor = lr.startColor;
                    lr.startWidth = 0.03f;
                    lr.endWidth = 0.03f;
                    lr.useWorldSpace = true;
                    lr.material = new Material(Shader.Find("Particles/Additive"));
                    LRs[id] = go;
                }

                lr.SetPosition(0, j.transform.TransformPoint(j.anchor));
                lr.SetPosition(1, j.connectedBody.transform.TransformPoint(j.connectedAnchor));
            }
        }

        private void LRDestroy()
        {
            foreach (var lr in LRs)
            {
                Logger.Log($"StrutManager({actuator.flightID}):  destroying GO of strut id={lr.Value}");
                lr.Value.DestroyGameObject();
            }
            LRs.Clear();
        }
    }
}