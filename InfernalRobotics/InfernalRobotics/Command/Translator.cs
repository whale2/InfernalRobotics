using InfernalRobotics.Control;

namespace InfernalRobotics.Command
{
    /*
     * <summary>
     * This class acts as a translator for UI - it implement basic commands for servos
     * and translates them to internal routines for servo controller.
     *
     * Thus later we can add more sophisticated controller commands, or change their behavior
     * per part basis without breaking the way UI works.
     *
     * More basic commands may be added too. Future API calls will most likely be hooked to this class.
     *
     */

    public class Translator
    {
        public void Init(bool motionLock, IServo servo, Interpolator interpolator)
        {
            Init(motionLock, servo, interpolator, null, null);
        }
        
        public void Init(bool motionLock, IServo servo, Interpolator interpolator, 
            BeforeStart beforestart=null, OnStop onstop=null)
        {
            IsMotionLock = motionLock;
            this.servo = servo;
            this.interpolator = interpolator;
            onStop = onstop;
            beforeStart = beforestart;
            this.interpolator.SetOnComplete(OnInterpolatorComplete);
        }

        private IServo servo;
        private Interpolator interpolator;

        public delegate void OnStop();
        public delegate void BeforeStart();

        private OnStop onStop;
        private BeforeStart beforeStart;
        
        // conversion data
        public bool IsMotionLock { get; set; }

        public float GetSpeedUnit()
        {
            if (servo == null)
                return 0f;
            
            return servo.Motor.DefaultSpeed;
        }

        // external interface
        /// <summary>
        /// Move the servo to the specified pos and speed.
        /// </summary>
        /// <param name="pos">Position in external coordinates</param>
        /// <param name="speed">Speed as multiplier</param>
        public void Move(float pos, float speed)
        {
            if (beforeStart != null)
            {
                beforeStart();
            }
            if (!interpolator.Active)
                servo.Mechanism.Reconfigure();


            if (!IsMotionLock)
                interpolator.SetCommand(ToInternalPos(pos), speed * GetSpeedUnit());
            else
                interpolator.SetCommand(0, 0);
        }

        public void MoveIncremental(float posDelta, float speed)
        {
            beforeStart?.Invoke();
            if (!interpolator.Active)
                servo.Mechanism.Reconfigure();

            float axisCorrection = servo.Motor.IsAxisInverted ? -1 : 1;
            interpolator.SetIncrementalCommand(posDelta*axisCorrection, speed * GetSpeedUnit());
        }

        public void Stop()
        {
            Move(0, 0);
            if (onStop != null && !IsMoving())
            {
                onStop();
            }
        }

        public bool IsMoving()
        {
            return interpolator.Active && (interpolator.CmdVelocity != 0f);
        }

        public void OnInterpolatorComplete()
        {
            Logger.Log("[Translator] got OnComplete", Logger.Level.SuperVerbose);
            onStop?.Invoke();
        }
        
        public float ToInternalPos(float externalPos)
        {
            if (servo.Motor.IsAxisInverted)
                return servo.Mechanism.MinPositionLimit + servo.Mechanism.MaxPositionLimit - externalPos;
            return externalPos;
        }

        public float ToExternalPos(float internalPos)
        {
            if (servo.Motor.IsAxisInverted)
                return servo.Mechanism.MinPositionLimit + servo.Mechanism.MaxPositionLimit - internalPos;
            return internalPos;
        }
    }
}