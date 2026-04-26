using System;
using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

public class QuadcopterController : MonoBehaviour
{
    public enum FlightMode
    {
        Manual,
        AutoTarget
    }

    [Serializable]
    public class Rotor
    {
        public string label = "Rotor";
        public Transform transform;

        [Tooltip("+1 CCW, -1 CW. Used for yaw reaction torque and visual spin direction.")]
        [Range(-1f, 1f)]
        public float yawSpinDirection = 1f;

        public Transform visual;
        public Vector3 visualSpinAxis = Vector3.up;
        public float maxVisualSpinSpeed = 3600f;
    }

    [Header("Mode")]
    [SerializeField] private FlightMode flightMode = FlightMode.Manual;

    [Header("Custom Dynamics (No Unity Rigidbody Physics)")]
    [SerializeField] private float mass = 1.4f;
    [SerializeField] private Vector3 inertiaTensorBody = new Vector3(0.02f, 0.04f, 0.02f);
    [SerializeField] private float gravityAcceleration = 9.81f;
    [SerializeField] private float linearDrag = 0.12f;
    [SerializeField] private float angularDrag = 0.18f;
    [SerializeField] private float maxLinearSpeed = 20f;
    [SerializeField] private float maxAngularSpeed = 20f;

    [Header("Rotor Setup (must be 4)")]
    [SerializeField] private Rotor[] rotors = new Rotor[4];
    [SerializeField] private float minRotorThrust = 0f;
    [SerializeField] private float maxRotorThrust = 25f;
    [SerializeField] private float yawDragTorqueCoefficient = 0.04f;
    [SerializeField] private bool autoCenterOfMassFromRotors = true;
    [SerializeField] private Vector3 centerOfMassLocal = Vector3.zero;

    [Header("Manual Controls")]
    [SerializeField] private float manualMaxClimbRate = 3f;
    [SerializeField] private float manualMaxTiltAngle = 20f;
    [SerializeField] private float manualYawRate = 70f;

    [Header("Auto Target")]
    [SerializeField] private Transform autoTargetTransform;
    [SerializeField] private Vector3 autoTargetPosition = new Vector3(0f, 2f, 0f);
    [SerializeField] private bool faceTargetInAutoMode = true;
    [SerializeField] private bool allowMouseClickTarget = true;
    [SerializeField] private Camera targetSelectionCamera;
    [SerializeField] private float autoMaxTiltAngle = 30f;
    [SerializeField] private float autoMaxHorizontalAcceleration = 6f;
    [SerializeField] private float autoMaxVerticalSpeed = 3f;
    [SerializeField] private float autoAltitudePositionGain = 1.2f;

    [Header("Horizontal Position PID (Auto)")]
    [SerializeField] private float horizontalPositionKp = 1.0f;
    [SerializeField] private float horizontalPositionKi = 0.04f;
    [SerializeField] private float horizontalVelocityKd = 1.8f;
    [SerializeField] private float horizontalIntegralLimit = 8f;

    [Header("Vertical Speed PID")]
    [SerializeField] private float verticalSpeedKp = 3.8f;
    [SerializeField] private float verticalSpeedKi = 1.0f;
    [SerializeField] private float verticalSpeedKd = 0.75f;
    [SerializeField] private float verticalIntegralLimit = 4f;

    [Header("Attitude PID (Local Torques)")]
    [SerializeField] private Vector3 attitudeKp = new Vector3(12f, 7f, 12f);
    [SerializeField] private Vector3 attitudeKi = new Vector3(0.6f, 0.35f, 0.6f);
    [SerializeField] private Vector3 attitudeKd = new Vector3(3.5f, 2.4f, 3.5f);
    [SerializeField] private Vector3 attitudeIntegralLimit = new Vector3(0.45f, 0.45f, 0.45f);
    [SerializeField] private float maxPitchTorque = 12f;
    [SerializeField] private float maxYawTorque = 8f;
    [SerializeField] private float maxRollTorque = 12f;

    [Header("Debug")]
    [SerializeField] private bool drawDebugGizmos = true;

    private readonly float[] _rotorThrusts = new float[4];
    private readonly float[,] _mixMatrix = new float[4, 4];
    private readonly float[] _mixVector = new float[4];
    private readonly float[] _mixSolution = new float[4];

    private Vector3 _positionWorld;
    private Quaternion _rotationWorld;
    private Vector3 _linearVelocityWorld;
    private Vector3 _angularVelocityBody;

    private float _manualThrottleInput;
    private float _manualYawInput;
    private float _manualPitchInput;
    private float _manualRollInput;
    private float _manualYawHeading;
    private float _autoYawHeading;

    private Vector3 _horizontalIntegral;
    private float _verticalSpeedIntegral;
    private float _lastVerticalSpeedError;
    private Vector3 _attitudeIntegral;

    private void Awake()
    {
        DisableUnityRigidbodyIfPresent();

        if (targetSelectionCamera == null)
        {
            targetSelectionCamera = Camera.main;
        }

        if (autoCenterOfMassFromRotors)
        {
            TryAutoCenterOfMass();
        }

        SyncStateFromTransform();
        ResetControllers();
    }

    private void OnEnable()
    {
        SyncStateFromTransform();
        ResetControllers();
    }

    private void OnValidate()
    {
        EnsureRotorArraySize();

        mass = Mathf.Max(0.05f, mass);
        gravityAcceleration = Mathf.Max(0f, gravityAcceleration);
        inertiaTensorBody.x = Mathf.Max(0.0001f, inertiaTensorBody.x);
        inertiaTensorBody.y = Mathf.Max(0.0001f, inertiaTensorBody.y);
        inertiaTensorBody.z = Mathf.Max(0.0001f, inertiaTensorBody.z);
        maxLinearSpeed = Mathf.Max(0.1f, maxLinearSpeed);
        maxAngularSpeed = Mathf.Max(0.1f, maxAngularSpeed);
        verticalIntegralLimit = Mathf.Max(0f, verticalIntegralLimit);
        horizontalIntegralLimit = Mathf.Max(0f, horizontalIntegralLimit);

        attitudeIntegralLimit.x = Mathf.Max(0f, attitudeIntegralLimit.x);
        attitudeIntegralLimit.y = Mathf.Max(0f, attitudeIntegralLimit.y);
        attitudeIntegralLimit.z = Mathf.Max(0f, attitudeIntegralLimit.z);
    }

    private void Update()
    {
        ReadManualInputs();
        HandleModeToggle();
        HandleAutoTargetInput();
        AnimateRotors();
    }

    private void FixedUpdate()
    {
        if (!AreRotorsConfigured())
        {
            return;
        }

        float dt = Time.fixedDeltaTime;

        Vector3 desiredUp;
        Vector3 desiredForward;
        float collectiveThrust;

        if (flightMode == FlightMode.Manual)
        {
            BuildManualSetpoint(out desiredUp, out desiredForward, out collectiveThrust, dt);
        }
        else
        {
            BuildAutoSetpoint(out desiredUp, out desiredForward, out collectiveThrust, dt);
        }

        Quaternion desiredRotation = Quaternion.LookRotation(desiredForward, desiredUp);
        Vector3 desiredTorqueLocal = ComputeAttitudeTorqueLocal(desiredRotation, dt);

        MixRotorThrusts(collectiveThrust, desiredTorqueLocal);
        IntegrateCustomDynamics(dt);
    }

    public void SetAutoTarget(Vector3 worldPosition)
    {
        autoTargetPosition = worldPosition;
    }

    private void BuildManualSetpoint(out Vector3 desiredUp, out Vector3 desiredForward, out float collectiveThrust, float dt)
    {
        _manualYawHeading += _manualYawInput * manualYawRate * dt;
        _manualYawHeading = WrapAngle360(_manualYawHeading);

        Vector2 tiltInput = new Vector2(_manualRollInput, _manualPitchInput);
        tiltInput = Vector2.ClampMagnitude(tiltInput, 1f);

        float maxTiltRadians = manualMaxTiltAngle * Mathf.Deg2Rad;
        Vector3 desiredUpInYawFrame = new Vector3(
            -tiltInput.x * Mathf.Tan(maxTiltRadians),
            1f,
            -tiltInput.y * Mathf.Tan(maxTiltRadians)
        ).normalized;

        Quaternion yawRotation = Quaternion.Euler(0f, _manualYawHeading, 0f);
        desiredUp = yawRotation * desiredUpInYawFrame;

        Vector3 yawForward = yawRotation * Vector3.forward;
        desiredForward = Vector3.ProjectOnPlane(yawForward, desiredUp).normalized;

        if (desiredForward.sqrMagnitude < 0.0001f)
        {
            desiredForward = Vector3.ProjectOnPlane(_rotationWorld * Vector3.forward, desiredUp).normalized;
        }

        float desiredVerticalSpeed = _manualThrottleInput * manualMaxClimbRate;
        collectiveThrust = ComputeCollectiveThrustFromVerticalSpeed(desiredVerticalSpeed, dt);
    }

    private void BuildAutoSetpoint(out Vector3 desiredUp, out Vector3 desiredForward, out float collectiveThrust, float dt)
    {
        Vector3 target = autoTargetTransform != null ? autoTargetTransform.position : autoTargetPosition;

        Vector3 positionError = target - _positionWorld;
        Vector3 horizontalError = Vector3.ProjectOnPlane(positionError, Vector3.up);
        Vector3 horizontalVelocity = Vector3.ProjectOnPlane(_linearVelocityWorld, Vector3.up);

        _horizontalIntegral += horizontalError * dt;
        _horizontalIntegral = Vector3.ClampMagnitude(_horizontalIntegral, horizontalIntegralLimit);

        Vector3 horizontalAccelerationCommand =
            horizontalError * horizontalPositionKp +
            _horizontalIntegral * horizontalPositionKi -
            horizontalVelocity * horizontalVelocityKd;

        horizontalAccelerationCommand = Vector3.ClampMagnitude(horizontalAccelerationCommand, autoMaxHorizontalAcceleration);

        Vector3 desiredForceDirection = horizontalAccelerationCommand + Vector3.up * gravityAcceleration;
        desiredUp = desiredForceDirection.sqrMagnitude > 0.001f ? desiredForceDirection.normalized : Vector3.up;

        float tilt = Vector3.Angle(Vector3.up, desiredUp);
        if (tilt > autoMaxTiltAngle)
        {
            desiredUp = Vector3.Slerp(Vector3.up, desiredUp, autoMaxTiltAngle / tilt).normalized;
        }

        if (faceTargetInAutoMode)
        {
            Vector3 flatToTarget = Vector3.ProjectOnPlane(target - _positionWorld, Vector3.up);
            if (flatToTarget.sqrMagnitude > 0.05f)
            {
                _autoYawHeading = Mathf.Atan2(flatToTarget.x, flatToTarget.z) * Mathf.Rad2Deg;
            }
        }

        Quaternion headingRotation = Quaternion.Euler(0f, _autoYawHeading, 0f);
        Vector3 headingForward = headingRotation * Vector3.forward;
        desiredForward = Vector3.ProjectOnPlane(headingForward, desiredUp).normalized;

        if (desiredForward.sqrMagnitude < 0.0001f)
        {
            desiredForward = Vector3.ProjectOnPlane(_rotationWorld * Vector3.forward, desiredUp).normalized;
        }

        float desiredVerticalSpeed = Mathf.Clamp(positionError.y * autoAltitudePositionGain, -autoMaxVerticalSpeed, autoMaxVerticalSpeed);
        collectiveThrust = ComputeCollectiveThrustFromVerticalSpeed(desiredVerticalSpeed, dt);
    }

    private float ComputeCollectiveThrustFromVerticalSpeed(float desiredVerticalSpeed, float dt)
    {
        float currentVerticalSpeed = Vector3.Dot(_linearVelocityWorld, Vector3.up);
        float verticalSpeedError = desiredVerticalSpeed - currentVerticalSpeed;

        _verticalSpeedIntegral += verticalSpeedError * dt;
        _verticalSpeedIntegral = Mathf.Clamp(_verticalSpeedIntegral, -verticalIntegralLimit, verticalIntegralLimit);

        float derivative = (verticalSpeedError - _lastVerticalSpeedError) / Mathf.Max(0.0001f, dt);
        _lastVerticalSpeedError = verticalSpeedError;

        float verticalAccelerationCommand =
            verticalSpeedKp * verticalSpeedError +
            verticalSpeedKi * _verticalSpeedIntegral +
            verticalSpeedKd * derivative;

        float upAlignment = Mathf.Max(0.2f, Vector3.Dot(_rotationWorld * Vector3.up, Vector3.up));
        float collectiveThrust = mass * (gravityAcceleration + verticalAccelerationCommand) / upAlignment;

        return Mathf.Clamp(collectiveThrust, minRotorThrust * 4f, maxRotorThrust * 4f);
    }

    private Vector3 ComputeAttitudeTorqueLocal(Quaternion desiredRotation, float dt)
    {
        Vector3 attitudeErrorLocal = RotationErrorLocal(desiredRotation, _rotationWorld);

        _attitudeIntegral += attitudeErrorLocal * dt;
        _attitudeIntegral = ClampPerAxis(_attitudeIntegral, attitudeIntegralLimit);

        Vector3 proportional = Vector3.Scale(attitudeKp, attitudeErrorLocal);
        Vector3 integral = Vector3.Scale(attitudeKi, _attitudeIntegral);
        Vector3 derivative = -Vector3.Scale(attitudeKd, _angularVelocityBody);

        Vector3 torque = proportional + integral + derivative;
        torque.x = Mathf.Clamp(torque.x, -maxPitchTorque, maxPitchTorque);
        torque.y = Mathf.Clamp(torque.y, -maxYawTorque, maxYawTorque);
        torque.z = Mathf.Clamp(torque.z, -maxRollTorque, maxRollTorque);

        return torque;
    }

    private void MixRotorThrusts(float collectiveThrust, Vector3 torqueLocal)
    {
        for (int i = 0; i < 4; i++)
        {
            Vector3 localPoint = transform.InverseTransformPoint(rotors[i].transform.position) - centerOfMassLocal;

            _mixMatrix[0, i] = 1f;
            _mixMatrix[1, i] = localPoint.z;
            _mixMatrix[2, i] = -localPoint.x;
            _mixMatrix[3, i] = rotors[i].yawSpinDirection * yawDragTorqueCoefficient;
        }

        _mixVector[0] = collectiveThrust;
        _mixVector[1] = torqueLocal.x;
        _mixVector[2] = torqueLocal.z;
        _mixVector[3] = torqueLocal.y;

        bool solved = SolveLinearSystem4x4(_mixMatrix, _mixVector, _mixSolution);

        if (!solved)
        {
            float even = collectiveThrust * 0.25f;
            for (int i = 0; i < 4; i++)
            {
                _mixSolution[i] = even;
            }
        }

        for (int i = 0; i < 4; i++)
        {
            _rotorThrusts[i] = Mathf.Clamp(_mixSolution[i], minRotorThrust, maxRotorThrust);
        }
    }

    private void IntegrateCustomDynamics(float dt)
    {
        Vector3 totalForceBody = Vector3.zero;
        Vector3 totalTorqueBody = Vector3.zero;

        for (int i = 0; i < 4; i++)
        {
            float thrust = _rotorThrusts[i];
            Vector3 forceBody = Vector3.up * thrust;
            totalForceBody += forceBody;

            Vector3 rotorPointBody = transform.InverseTransformPoint(rotors[i].transform.position) - centerOfMassLocal;
            totalTorqueBody += Vector3.Cross(rotorPointBody, forceBody);
            totalTorqueBody += Vector3.up * (rotors[i].yawSpinDirection * yawDragTorqueCoefficient * thrust);
        }

        Vector3 gravityWorld = Vector3.down * (mass * gravityAcceleration);
        Vector3 totalForceWorld = _rotationWorld * totalForceBody + gravityWorld;

        Vector3 linearAccelerationWorld = totalForceWorld / mass;
        _linearVelocityWorld += linearAccelerationWorld * dt;
        _linearVelocityWorld *= 1f / (1f + linearDrag * dt);
        _linearVelocityWorld = Vector3.ClampMagnitude(_linearVelocityWorld, maxLinearSpeed);
        _positionWorld += _linearVelocityWorld * dt;

        Vector3 inertiaOmega = new Vector3(
            inertiaTensorBody.x * _angularVelocityBody.x,
            inertiaTensorBody.y * _angularVelocityBody.y,
            inertiaTensorBody.z * _angularVelocityBody.z);

        Vector3 gyroscopic = Vector3.Cross(_angularVelocityBody, inertiaOmega);

        Vector3 omegaDot = new Vector3(
            (totalTorqueBody.x - gyroscopic.x) / inertiaTensorBody.x,
            (totalTorqueBody.y - gyroscopic.y) / inertiaTensorBody.y,
            (totalTorqueBody.z - gyroscopic.z) / inertiaTensorBody.z);

        _angularVelocityBody += omegaDot * dt;
        _angularVelocityBody *= 1f / (1f + angularDrag * dt);
        _angularVelocityBody = Vector3.ClampMagnitude(_angularVelocityBody, maxAngularSpeed);

        Vector3 deltaAngle = _angularVelocityBody * dt;
        float angle = deltaAngle.magnitude;
        if (angle > 0.000001f)
        {
            Quaternion deltaRotation = Quaternion.AngleAxis(angle * Mathf.Rad2Deg, deltaAngle / angle);
            _rotationWorld = (_rotationWorld * deltaRotation).normalized;
        }

        transform.SetPositionAndRotation(_positionWorld, _rotationWorld);
    }

    private static Vector3 RotationErrorLocal(Quaternion desired, Quaternion current)
    {
        Quaternion error = desired * Quaternion.Inverse(current);
        error.ToAngleAxis(out float angleDeg, out Vector3 axisWorld);

        if (float.IsNaN(axisWorld.x) || float.IsNaN(axisWorld.y) || float.IsNaN(axisWorld.z))
        {
            return Vector3.zero;
        }

        if (angleDeg > 180f)
        {
            angleDeg -= 360f;
        }

        float angleRad = angleDeg * Mathf.Deg2Rad;
        Vector3 axisLocal = Quaternion.Inverse(current) * axisWorld.normalized;
        return axisLocal * angleRad;
    }

    private static Vector3 ClampPerAxis(Vector3 value, Vector3 limits)
    {
        return new Vector3(
            Mathf.Clamp(value.x, -limits.x, limits.x),
            Mathf.Clamp(value.y, -limits.y, limits.y),
            Mathf.Clamp(value.z, -limits.z, limits.z));
    }

    private static bool SolveLinearSystem4x4(float[,] matrix, float[] values, float[] solution)
    {
        const int n = 4;
        float[,] a = new float[n, n + 1];

        for (int r = 0; r < n; r++)
        {
            for (int c = 0; c < n; c++)
            {
                a[r, c] = matrix[r, c];
            }

            a[r, n] = values[r];
        }

        for (int pivotCol = 0; pivotCol < n; pivotCol++)
        {
            int pivotRow = pivotCol;
            float pivotAbs = Mathf.Abs(a[pivotRow, pivotCol]);

            for (int r = pivotCol + 1; r < n; r++)
            {
                float candidate = Mathf.Abs(a[r, pivotCol]);
                if (candidate > pivotAbs)
                {
                    pivotAbs = candidate;
                    pivotRow = r;
                }
            }

            if (pivotAbs < 0.00001f)
            {
                return false;
            }

            if (pivotRow != pivotCol)
            {
                SwapRows(a, pivotRow, pivotCol, n + 1);
            }

            float pivot = a[pivotCol, pivotCol];
            for (int c = pivotCol; c <= n; c++)
            {
                a[pivotCol, c] /= pivot;
            }

            for (int r = 0; r < n; r++)
            {
                if (r == pivotCol)
                {
                    continue;
                }

                float factor = a[r, pivotCol];
                if (Mathf.Abs(factor) < 0.000001f)
                {
                    continue;
                }

                for (int c = pivotCol; c <= n; c++)
                {
                    a[r, c] -= factor * a[pivotCol, c];
                }
            }
        }

        for (int r = 0; r < n; r++)
        {
            solution[r] = a[r, n];
        }

        return true;
    }

    private static void SwapRows(float[,] matrix, int rowA, int rowB, int columnCount)
    {
        for (int c = 0; c < columnCount; c++)
        {
            float t = matrix[rowA, c];
            matrix[rowA, c] = matrix[rowB, c];
            matrix[rowB, c] = t;
        }
    }

    private void AnimateRotors()
    {
        for (int i = 0; i < rotors.Length; i++)
        {
            Rotor rotor = rotors[i];
            if (rotor == null)
            {
                continue;
            }

            Transform visual = rotor.visual != null ? rotor.visual : rotor.transform;
            if (visual == null)
            {
                continue;
            }

            float normalizedThrust = Mathf.InverseLerp(minRotorThrust, maxRotorThrust, _rotorThrusts[i]);
            float spinSpeed = normalizedThrust * rotor.maxVisualSpinSpeed;
            visual.Rotate(rotor.visualSpinAxis.normalized, spinSpeed * rotor.yawSpinDirection * Time.deltaTime, Space.Self);
        }
    }

    private bool AreRotorsConfigured()
    {
        if (rotors == null || rotors.Length != 4)
        {
            return false;
        }

        for (int i = 0; i < 4; i++)
        {
            if (rotors[i] == null || rotors[i].transform == null)
            {
                return false;
            }
        }

        return true;
    }

    private void TryAutoCenterOfMass()
    {
        if (!AreRotorsConfigured())
        {
            return;
        }

        Vector3 sum = Vector3.zero;
        int count = 0;

        for (int i = 0; i < rotors.Length; i++)
        {
            if (rotors[i] == null || rotors[i].transform == null)
            {
                continue;
            }

            sum += transform.InverseTransformPoint(rotors[i].transform.position);
            count++;
        }

        if (count > 0)
        {
            centerOfMassLocal = sum / count;
        }
    }

    private void EnsureRotorArraySize()
    {
        if (rotors != null && rotors.Length == 4)
        {
            for (int i = 0; i < rotors.Length; i++)
            {
                if (rotors[i] == null)
                {
                    rotors[i] = new Rotor();
                }
            }

            return;
        }

        Rotor[] resized = new Rotor[4];
        if (rotors != null)
        {
            int copyCount = Mathf.Min(rotors.Length, resized.Length);
            for (int i = 0; i < copyCount; i++)
            {
                resized[i] = rotors[i];
            }
        }

        for (int i = 0; i < resized.Length; i++)
        {
            if (resized[i] == null)
            {
                resized[i] = new Rotor();
            }
        }

        rotors = resized;
    }

    private void SyncStateFromTransform()
    {
        _positionWorld = transform.position;
        _rotationWorld = transform.rotation;

        _manualYawHeading = transform.eulerAngles.y;
        _autoYawHeading = _manualYawHeading;
    }

    private void ResetControllers()
    {
        _horizontalIntegral = Vector3.zero;
        _verticalSpeedIntegral = 0f;
        _lastVerticalSpeedError = 0f;
        _attitudeIntegral = Vector3.zero;

        _linearVelocityWorld = Vector3.zero;
        _angularVelocityBody = Vector3.zero;
    }

    private void DisableUnityRigidbodyIfPresent()
    {
        Rigidbody rb = GetComponent<Rigidbody>();
        if (rb == null)
        {
            return;
        }

        rb.isKinematic = true;
        rb.useGravity = false;
#if UNITY_6000_0_OR_NEWER
        rb.linearVelocity = Vector3.zero;
#else
        rb.velocity = Vector3.zero;
#endif
        rb.angularVelocity = Vector3.zero;
    }

    private void ReadManualInputs()
    {
        _manualThrottleInput = AxisFromPair(IsThrottleUpPressed(), IsThrottleDownPressed());
        _manualYawInput = AxisFromPair(IsYawRightPressed(), IsYawLeftPressed());
        _manualPitchInput = AxisFromPair(IsPitchForwardPressed(), IsPitchBackwardPressed());
        _manualRollInput = AxisFromPair(IsRollRightPressed(), IsRollLeftPressed());
    }

    private void HandleModeToggle()
    {
        if (!IsToggleModePressedThisFrame())
        {
            return;
        }

        flightMode = flightMode == FlightMode.Manual ? FlightMode.AutoTarget : FlightMode.Manual;
        _manualYawHeading = transform.eulerAngles.y;
        _autoYawHeading = _manualYawHeading;

        _horizontalIntegral = Vector3.zero;
        _verticalSpeedIntegral = 0f;
        _attitudeIntegral = Vector3.zero;
    }

    private void HandleAutoTargetInput()
    {
        if (flightMode != FlightMode.AutoTarget || !allowMouseClickTarget || !IsSetTargetPressedThisFrame())
        {
            return;
        }

        if (targetSelectionCamera == null)
        {
            targetSelectionCamera = Camera.main;
        }

        if (targetSelectionCamera == null)
        {
            return;
        }

        if (!TryGetPointerScreenPosition(out Vector2 pointer))
        {
            return;
        }

        Ray ray = targetSelectionCamera.ScreenPointToRay(pointer);
        if (Physics.Raycast(ray, out RaycastHit hitInfo, 500f))
        {
            SetAutoTarget(hitInfo.point);
            return;
        }

        Plane horizontalPlane = new Plane(Vector3.up, new Vector3(0f, autoTargetPosition.y, 0f));
        if (horizontalPlane.Raycast(ray, out float enter))
        {
            SetAutoTarget(ray.GetPoint(enter));
        }
    }

    private static float AxisFromPair(bool positive, bool negative)
    {
        return (positive ? 1f : 0f) - (negative ? 1f : 0f);
    }

    private static float WrapAngle360(float angle)
    {
        while (angle < 0f)
        {
            angle += 360f;
        }

        while (angle >= 360f)
        {
            angle -= 360f;
        }

        return angle;
    }

    private bool IsThrottleUpPressed()
    {
#if ENABLE_INPUT_SYSTEM
        if (Keyboard.current != null)
        {
            return Keyboard.current.wKey.isPressed;
        }
#endif
#if ENABLE_LEGACY_INPUT_MANAGER
        return Input.GetKey(KeyCode.W);
#else
        return false;
#endif
    }

    private bool IsThrottleDownPressed()
    {
#if ENABLE_INPUT_SYSTEM
        if (Keyboard.current != null)
        {
            return Keyboard.current.sKey.isPressed;
        }
#endif
#if ENABLE_LEGACY_INPUT_MANAGER
        return Input.GetKey(KeyCode.S);
#else
        return false;
#endif
    }

    private bool IsYawLeftPressed()
    {
#if ENABLE_INPUT_SYSTEM
        if (Keyboard.current != null)
        {
            return Keyboard.current.aKey.isPressed;
        }
#endif
#if ENABLE_LEGACY_INPUT_MANAGER
        return Input.GetKey(KeyCode.A);
#else
        return false;
#endif
    }

    private bool IsYawRightPressed()
    {
#if ENABLE_INPUT_SYSTEM
        if (Keyboard.current != null)
        {
            return Keyboard.current.dKey.isPressed;
        }
#endif
#if ENABLE_LEGACY_INPUT_MANAGER
        return Input.GetKey(KeyCode.D);
#else
        return false;
#endif
    }

    private bool IsPitchForwardPressed()
    {
#if ENABLE_INPUT_SYSTEM
        if (Keyboard.current != null)
        {
            return Keyboard.current.iKey.isPressed;
        }
#endif
#if ENABLE_LEGACY_INPUT_MANAGER
        return Input.GetKey(KeyCode.I);
#else
        return false;
#endif
    }

    private bool IsPitchBackwardPressed()
    {
#if ENABLE_INPUT_SYSTEM
        if (Keyboard.current != null)
        {
            return Keyboard.current.kKey.isPressed;
        }
#endif
#if ENABLE_LEGACY_INPUT_MANAGER
        return Input.GetKey(KeyCode.K);
#else
        return false;
#endif
    }

    private bool IsRollLeftPressed()
    {
#if ENABLE_INPUT_SYSTEM
        if (Keyboard.current != null)
        {
            return Keyboard.current.jKey.isPressed;
        }
#endif
#if ENABLE_LEGACY_INPUT_MANAGER
        return Input.GetKey(KeyCode.J);
#else
        return false;
#endif
    }

    private bool IsRollRightPressed()
    {
#if ENABLE_INPUT_SYSTEM
        if (Keyboard.current != null)
        {
            return Keyboard.current.lKey.isPressed;
        }
#endif
#if ENABLE_LEGACY_INPUT_MANAGER
        return Input.GetKey(KeyCode.L);
#else
        return false;
#endif
    }

    private bool IsToggleModePressedThisFrame()
    {
#if ENABLE_INPUT_SYSTEM
        if (Keyboard.current != null)
        {
            return Keyboard.current.mKey.wasPressedThisFrame;
        }
#endif
#if ENABLE_LEGACY_INPUT_MANAGER
        return Input.GetKeyDown(KeyCode.M);
#else
        return false;
#endif
    }

    private bool IsSetTargetPressedThisFrame()
    {
#if ENABLE_INPUT_SYSTEM
        if (Mouse.current != null)
        {
            return Mouse.current.leftButton.wasPressedThisFrame;
        }
#endif
#if ENABLE_LEGACY_INPUT_MANAGER
        return Input.GetMouseButtonDown(0);
#else
        return false;
#endif
    }

    private bool TryGetPointerScreenPosition(out Vector2 position)
    {
#if ENABLE_INPUT_SYSTEM
        if (Mouse.current != null)
        {
            position = Mouse.current.position.ReadValue();
            return true;
        }
#endif
#if ENABLE_LEGACY_INPUT_MANAGER
        position = Input.mousePosition;
        return true;
#else
        position = default;
        return false;
#endif
    }

    private void OnDrawGizmosSelected()
    {
        if (!drawDebugGizmos)
        {
            return;
        }

        if (rotors != null)
        {
            Gizmos.color = Color.yellow;
            for (int i = 0; i < rotors.Length; i++)
            {
                if (rotors[i] == null || rotors[i].transform == null)
                {
                    continue;
                }

                Gizmos.DrawSphere(rotors[i].transform.position, 0.05f);
            }
        }

        Vector3 target = autoTargetTransform != null ? autoTargetTransform.position : autoTargetPosition;
        Gizmos.color = Color.cyan;
        Gizmos.DrawWireSphere(target, 0.25f);
        Gizmos.DrawLine(transform.position, target);
    }
}
