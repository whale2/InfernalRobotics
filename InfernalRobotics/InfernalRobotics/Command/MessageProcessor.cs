using System;
using System.Collections.Generic;
using System.Linq;
using InfernalRobotics.Module;

namespace InfernalRobotics.Command
{
    public class MessageProcessor
    {

        /*  This class keeps neighboring actuators and passes messages between them
         *
         *  When part starts moving, it should tell all upward and neighboring parts so they can destroy hvStruts
         *  they might have
         *  When part stops moving, it should tell upward and neighboring parts so they can re-create their hvStruts
         */
        
        private Part actuator => StrutManager.actuator;
        private StrutManager StrutManager;

        internal List<MessageProcessor> neighbors;
        internal List<MessageProcessor> upwardBranch;

        private HashSet<uint> moveMessages;


        public MessageProcessor(StrutManager strutManager)
        {
            StrutManager = strutManager;
            moveMessages = new HashSet<uint>();
        }
        
        public void Init() {

            neighbors = FindNeighbors(actuator);
            upwardBranch = FindUpwardBranch(actuator).Where(u => !neighbors.Contains(u)).ToList();
            neighbors.ForEach(n => Logger.Log(
                $"MessageProcessor({actuator.flightID}): neighbor:{n?.actuator?.name}, {n?.actuator?.flightID}"));
            upwardBranch.ForEach(n => Logger.Log(
                $"MessageProcessor({actuator.flightID}): upward:{n?.actuator?.name}, {n?.actuator?.flightID}"));
        }

        public void ProcessStopMessage(MessageProcessor source)
        {
            Logger.Log($"MessageProcessor({actuator.flightID}): got stop from {source.actuator.flightID}");
            // If nothing more is moving, tell StrutManager about that
            bool removed = moveMessages.Remove(source.actuator.flightID);
            if (removed && source.actuator.flightID == actuator.flightID && moveMessages.Count > 0)
            {
                Logger.Log(
                    $"MessageProcessor({actuator.flightID}): Something else moves, but this joint has stopped");
                StrutManager.Stop(false);
            }
            if (moveMessages.Count == 0)
            {
                Logger.Log(
                    $"MessageProcessor({actuator.flightID}): Nothing else moves, safe to re-create all struts");
                StrutManager.Stop(true);
            }
            else
            {
                Logger.Log($"MessageProcessor({actuator.flightID}): {moveMessages.Count} joints still moving");
                foreach (var ac in moveMessages)
                {
                    Logger.Log($"MessageProcessor({actuator.flightID}): moving: {ac}");
                }
            }

            if (!removed)
            {
                return;
            }
            // Pass stop message to neighbors
            neighbors.ForEach(n => {
                if (n != source) n.ProcessStopMessage(this);
            });
            upwardBranch.ForEach(n => n.ProcessStopMessage(this));
        }

        public void StopReceived()
        {
            Logger.Log($"MessageProcessor({actuator.flightID}): stop received");
            ProcessStopMessage(this);
        }
        
        public void ProcessMoveMessage(MessageProcessor source)
        {
            bool newMessage = moveMessages.Add(source.actuator.flightID);
            Logger.Log(
                $"MessageProcessor({actuator.flightID}): got move from {source.actuator.flightID}; is new={newMessage}");

            if (!newMessage)
            {
                return;
            }
            // Pass move message to neighbors
            neighbors.ForEach(n => {
                if (n != source) n.ProcessMoveMessage(this);
            });
            upwardBranch.ForEach(n => n.ProcessMoveMessage(this));

            // Soft strut should be dropped if our own actuator is moving
            // or if neighboring actuator moves
            // If it is the branch that is moving, soft strut should be left as is
            bool dropSoftStrut = source.actuator.flightID == actuator.flightID ||
                                 neighbors.Contains(source);
            Logger.Log(
                $"MessageProcessor({actuator.flightID}): drop soft={dropSoftStrut}");
            if (newMessage)
            {
                StrutManager.Move(dropSoftStrut);
            }
        }

        public void MoveReceived()
        {
            Logger.Log($"MessageProcessor({actuator.flightID}): move received");
            ProcessMoveMessage(this);
        }
        private List<MessageProcessor> FindUpwardBranch(Part part)
        {
            // Find all possible actuators upward from here
            List<MessageProcessor> upwardList = new List<MessageProcessor>();
            foreach (Part p in part.children)
            {
                if (p.Modules.Contains(StrutManager.muMechToggle))
                {
                    upwardList.Add(((ModuleIRServo) (p.Modules[StrutManager.muMechToggle])).StrutManager.MessageProcessor);
                }
                upwardList.AddRange(FindUpwardBranch(p));
            }

            return upwardList;
        }

        internal List<MessageProcessor> FindNeighbors(Part part)
        {
            List<MessageProcessor> neighborList = FindNeighborsUpward(part);
            neighborList.AddRange(FindNeighborsDownward(part.parent));
            neighborList.RemoveAll(n => n == null);
            Logger.Log($"MessageProcessor({actuator.flightID}): done checking neighbors");
            return neighborList;
        }

        private List<MessageProcessor> FindNeighborsUpward(Part part)
        {
            List<MessageProcessor> neighborsList = new List<MessageProcessor>();
            // Find closest IR parts up the tree
            foreach (Part p in part.children)
            {
                Logger.Log($"StrutManager({actuator.flightID}): looking for neighbors upward: {p?.name}");
                try
                {
                    if (p.Modules.Contains(StrutManager.muMechToggle))
                    {
                        neighborsList.Add(((ModuleIRServo) (p.Modules[StrutManager.muMechToggle])).StrutManager.MessageProcessor);
                    }
                }
                catch (NullReferenceException)
                {
                    Logger.Log($"StrutManager({actuator.flightID}): FindNeighborsUpward: got nullref for part: {p?.name}");
                }
            }

            return neighborsList;
        }

        private List<MessageProcessor> FindNeighborsDownward(Part part) 
        {
            List<MessageProcessor> neighborsList = new List<MessageProcessor>();

            // Find closes IR part down the tree
            try
            {
                if (part.Modules.Contains(StrutManager.muMechToggle))
                {
                    Logger.Log($"StrutManager({actuator.flightID}): found neighbor downward: {part.name}");
                    neighborsList.Add(((ModuleIRServo) (part.Modules[StrutManager.muMechToggle])).StrutManager.MessageProcessor);
                }
            }
            catch (NullReferenceException)
            {
                Logger.Log($"StrutManager({actuator.flightID}): FindNeighborsDownward: got nullref for part: {part?.name}");
            }

            return neighborsList;
        }
    }
}