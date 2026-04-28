using System;
using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

[DefaultExecutionOrder(-100)]
public class QuadcopterController : MonoBehaviour
{
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

    [Header("Bouabdallah Dynamics")]
    [SerializeField] private float mass = 1.4f;
    [SerializeField] private Vector3 inertiaTensorBody = new Vector3(0.02f, 0.04f, 0.02f);
    [SerializeField] private float gravityAcceleration = 9.81f;
    [SerializeField] private float linearDrag = 0.12f;
    [SerializeField] private float angularDrag = 0.18f;
    [SerializeField] private float rotorGyroscopicCoefficient = 0.0006f;
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

    [Header("Manual Vertical Stabilization")]
    [SerializeField] private float verticalSpeedKp = 3.8f;
    [SerializeField] private float verticalSpeedKi = 0.65f;
    [SerializeField] private float verticalSpeedKd = 0.35f;
    [SerializeField] private float verticalIntegralLimit = 3f;

    [Header("Manual Attitude Stabilization")]
    [SerializeField] private float attitudeAngleGain = 6f;
    [SerializeField] private float yawHeadingGain = 4f;
    [SerializeField] private Vector3 angularRateGain = new Vector3(7.5f, 5.5f, 7.5f);
    [SerializeField] private float maxPitchTorque = 12f;
    [SerializeField] private float maxYawTorque = 8f;
    [SerializeField] private float maxRollTorque = 12f;

    [Header("Debug")]
    [SerializeField] private bool drawDebugGizmos = true;

    [Header("Rendering")]
    [SerializeField] private bool interpolateRenderedTransform = true;

    private readonly float[] _rotorThrusts = new float[4];
    private readonly Vector3[] _rotorPointsBody = new Vector3[4];
    private readonly Transform[] _rotorVisuals = new Transform[4];
    private readonly Vector3[] _rotorVisualSpinAxes = new Vector3[4];
    private readonly float[] _rotorSpinDirections = new float[4];
    private readonly float[] _rotorMaxVisualSpinSpeeds = new float[4];
    private readonly float[] _rotorYawFactors = new float[4];
    private readonly float[,] _mixMatrix = new float[4, 4];
    private readonly float[,] _mixMatrixInverse = new float[4, 4];
    private readonly float[] _mixVector = new float[4];
    private readonly float[,] _inverseScratch = new float[4, 8];
    private bool _rotorGeometryReady;
    private bool _mixingReady;

    private Vector3 _positionWorld;
    private Quaternion _rotationWorld;
    private Vector3 _previousPositionWorld;
    private Quaternion _previousRotationWorld;
    private Vector3 _linearVelocityWorld;
    private Vector3 _angularVelocityBody;
    private bool _hasPreviousPhysicsState;

    private float _manualThrottleInput;
    private float _manualYawInput;
    private float _manualPitchInput;
    private float _manualRollInput;
    private float _manualYawHeading;

    private float _verticalSpeedIntegral;
    private float _lastVerticalSpeedError;

    private void Awake()
    {
        DisableUnityRigidbodyIfPresent();

        if (autoCenterOfMassFromRotors)
        {
            TryAutoCenterOfMass();
        }

        RefreshRotorGeometry();
        SyncStateFromTransform();
        ResetControllers();
    }

    private void OnEnable()
    {
        RefreshRotorGeometry();
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
        linearDrag = Mathf.Max(0f, linearDrag);
        angularDrag = Mathf.Max(0f, angularDrag);
        rotorGyroscopicCoefficient = Mathf.Max(0f, rotorGyroscopicCoefficient);
        maxLinearSpeed = Mathf.Max(0.1f, maxLinearSpeed);
        maxAngularSpeed = Mathf.Max(0.1f, maxAngularSpeed);
        maxRotorThrust = Mathf.Max(0.01f, maxRotorThrust);
        minRotorThrust = Mathf.Clamp(minRotorThrust, 0f, maxRotorThrust);
        yawDragTorqueCoefficient = Mathf.Max(0f, yawDragTorqueCoefficient);
        manualMaxClimbRate = Mathf.Max(0f, manualMaxClimbRate);
        manualMaxTiltAngle = Mathf.Clamp(manualMaxTiltAngle, 0f, 75f);
        manualYawRate = Mathf.Max(0f, manualYawRate);
        verticalIntegralLimit = Mathf.Max(0f, verticalIntegralLimit);
        attitudeAngleGain = Mathf.Max(0f, attitudeAngleGain);
        yawHeadingGain = Mathf.Max(0f, yawHeadingGain);
        angularRateGain.x = Mathf.Max(0f, angularRateGain.x);
        angularRateGain.y = Mathf.Max(0f, angularRateGain.y);
        angularRateGain.z = Mathf.Max(0f, angularRateGain.z);
        maxPitchTorque = Mathf.Max(0f, maxPitchTorque);
        maxYawTorque = Mathf.Max(0f, maxYawTorque);
        maxRollTorque = Mathf.Max(0f, maxRollTorque);

        RefreshRotorGeometry();
    }

    private void Update()
    {
        ReadManualInputs();
        AnimateRotors();
    }

    private void FixedUpdate()
    {
        if (!_rotorGeometryReady)
        {
            RefreshRotorGeometry();
        }

        if (!_rotorGeometryReady)
        {
            return;
        }

        float dt = Time.fixedDeltaTime;
        CachePreviousPhysicsState();

        Vector3 desiredTorqueBody = ComputeManualTorqueBody(dt);
        float collectiveThrust = ComputeManualCollectiveThrust(dt);

        MixBouabdallahInputs(collectiveThrust, desiredTorqueBody);
        IntegrateBouabdallahDynamics(dt);
    }

    private void LateUpdate()
    {
        ApplyRenderedTransform();
    }

    private Vector3 ComputeManualTorqueBody(float dt)
    {
        _manualYawHeading = WrapAngle360(_manualYawHeading + _manualYawInput * manualYawRate * dt);

        Quaternion desiredRotation = BuildManualDesiredRotation();
        Vector3 rotationErrorBody = RotationErrorBody(desiredRotation, _rotationWorld);

        Vector3 desiredAngularVelocityBody = new Vector3(
            rotationErrorBody.x * attitudeAngleGain,
            rotationErrorBody.y * yawHeadingGain,
            rotationErrorBody.z * attitudeAngleGain);

        if (Mathf.Abs(_manualYawInput) > 0.001f)
        {
            desiredAngularVelocityBody.y = _manualYawInput * manualYawRate * Mathf.Deg2Rad;
        }

        Vector3 rateError = desiredAngularVelocityBody - _angularVelocityBody;
        Vector3 angularAccelerationCommand = Vector3.Scale(angularRateGain, rateError);
        Vector3 inertiaOmega = MultiplyInertia(_angularVelocityBody);

        Vector3 torqueBody = new Vector3(
            inertiaTensorBody.x * angularAccelerationCommand.x,
            inertiaTensorBody.y * angularAccelerationCommand.y,
            inertiaTensorBody.z * angularAccelerationCommand.z);

        torqueBody += Vector3.Cross(_angularVelocityBody, inertiaOmega);

        torqueBody.x = Mathf.Clamp(torqueBody.x, -maxPitchTorque, maxPitchTorque);
        torqueBody.y = Mathf.Clamp(torqueBody.y, -maxYawTorque, maxYawTorque);
        torqueBody.z = Mathf.Clamp(torqueBody.z, -maxRollTorque, maxRollTorque);
        return torqueBody;
    }

    private Quaternion BuildManualDesiredRotation()
    {
        Vector2 tiltInput = new Vector2(_manualRollInput, _manualPitchInput);
        tiltInput = Vector2.ClampMagnitude(tiltInput, 1f);

        float maxTiltRadians = manualMaxTiltAngle * Mathf.Deg2Rad;
        float tiltScale = Mathf.Tan(maxTiltRadians);
        Vector3 desiredUpInYawFrame = new Vector3(
            tiltInput.x * tiltScale,
            1f,
            tiltInput.y * tiltScale).normalized;

        Quaternion yawRotation = Quaternion.Euler(0f, _manualYawHeading, 0f);
        Vector3 desiredUp = yawRotation * desiredUpInYawFrame;
        Vector3 yawForward = yawRotation * Vector3.forward;
        Vector3 desiredForward = Vector3.ProjectOnPlane(yawForward, desiredUp);

        if (desiredForward.sqrMagnitude < 0.0001f)
        {
            desiredForward = Vector3.ProjectOnPlane(_rotationWorld * Vector3.forward, desiredUp);
        }

        if (desiredForward.sqrMagnitude < 0.0001f)
        {
            desiredForward = Vector3.forward;
        }

        return Quaternion.LookRotation(desiredForward.normalized, desiredUp);
    }

    private float ComputeManualCollectiveThrust(float dt)
    {
        float desiredVerticalSpeed = _manualThrottleInput * manualMaxClimbRate;
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

        float upAlignment = Mathf.Max(0.25f, Vector3.Dot(_rotationWorld * Vector3.up, Vector3.up));
        float collectiveThrust = mass * (gravityAcceleration + verticalAccelerationCommand) / upAlignment;

        return Mathf.Clamp(collectiveThrust, minRotorThrust * 4f, maxRotorThrust * 4f);
    }

    private void MixBouabdallahInputs(float collectiveThrust, Vector3 torqueBody)
    {
        _mixVector[0] = collectiveThrust;
        _mixVector[1] = torqueBody.x;
        _mixVector[2] = torqueBody.y;
        _mixVector[3] = torqueBody.z;

        float evenThrust = collectiveThrust * 0.25f;

        for (int i = 0; i < 4; i++)
        {
            float thrust = _mixingReady ? MultiplyMixingRow(i) : evenThrust;
            _rotorThrusts[i] = Mathf.Clamp(thrust, minRotorThrust, maxRotorThrust);
        }
    }

    private void IntegrateBouabdallahDynamics(float dt)
    {
        Vector3 totalForceBody = Vector3.zero;
        Vector3 totalTorqueBody = Vector3.zero;
        float residualRotorSpeed = 0f;

        for (int i = 0; i < 4; i++)
        {
            float thrust = _rotorThrusts[i];
            Vector3 forceBody = Vector3.up * thrust;
            Vector3 rotorPointBody = _rotorPointsBody[i];

            totalForceBody += forceBody;
            totalTorqueBody += Vector3.Cross(rotorPointBody, forceBody);
            totalTorqueBody += Vector3.up * (_rotorYawFactors[i] * thrust);
            residualRotorSpeed += _rotorSpinDirections[i] * Mathf.Sqrt(Mathf.Max(0f, thrust));
        }

        Vector3 gravityWorld = Vector3.down * (mass * gravityAcceleration);
        Vector3 totalForceWorld = _rotationWorld * totalForceBody + gravityWorld;
        Vector3 linearAccelerationWorld = totalForceWorld / mass;

        _linearVelocityWorld += linearAccelerationWorld * dt;
        _linearVelocityWorld *= 1f / (1f + linearDrag * dt);
        _linearVelocityWorld = Vector3.ClampMagnitude(_linearVelocityWorld, maxLinearSpeed);
        _positionWorld += _linearVelocityWorld * dt;

        Vector3 inertiaOmega = MultiplyInertia(_angularVelocityBody);
        Vector3 rigidBodyGyroscopic = Vector3.Cross(_angularVelocityBody, inertiaOmega);
        Vector3 rotorGyroscopic = rotorGyroscopicCoefficient * residualRotorSpeed * Vector3.Cross(_angularVelocityBody, Vector3.up);

        Vector3 omegaDot = new Vector3(
            (totalTorqueBody.x - rigidBodyGyroscopic.x - rotorGyroscopic.x) / inertiaTensorBody.x,
            (totalTorqueBody.y - rigidBodyGyroscopic.y - rotorGyroscopic.y) / inertiaTensorBody.y,
            (totalTorqueBody.z - rigidBodyGyroscopic.z - rotorGyroscopic.z) / inertiaTensorBody.z);

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

        if (!interpolateRenderedTransform)
        {
            ApplySimulationTransform();
        }
    }

    private Vector3 MultiplyInertia(Vector3 bodyAngularVelocity)
    {
        return new Vector3(
            inertiaTensorBody.x * bodyAngularVelocity.x,
            inertiaTensorBody.y * bodyAngularVelocity.y,
            inertiaTensorBody.z * bodyAngularVelocity.z);
    }

    private static Vector3 RotationErrorBody(Quaternion desired, Quaternion current)
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

        return Quaternion.Inverse(current) * axisWorld.normalized * (angleDeg * Mathf.Deg2Rad);
    }

    private float MultiplyMixingRow(int row)
    {
        return
            _mixMatrixInverse[row, 0] * _mixVector[0] +
            _mixMatrixInverse[row, 1] * _mixVector[1] +
            _mixMatrixInverse[row, 2] * _mixVector[2] +
            _mixMatrixInverse[row, 3] * _mixVector[3];
    }

    private bool TryBuildMixingInverse()
    {
        const int n = 4;

        for (int r = 0; r < n; r++)
        {
            for (int c = 0; c < n; c++)
            {
                _inverseScratch[r, c] = _mixMatrix[r, c];
                _inverseScratch[r, c + n] = r == c ? 1f : 0f;
            }
        }

        for (int pivotCol = 0; pivotCol < n; pivotCol++)
        {
            int pivotRow = pivotCol;
            float pivotAbs = Mathf.Abs(_inverseScratch[pivotRow, pivotCol]);

            for (int r = pivotCol + 1; r < n; r++)
            {
                float candidate = Mathf.Abs(_inverseScratch[r, pivotCol]);
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
                SwapRows(_inverseScratch, pivotRow, pivotCol, n * 2);
            }

            float pivot = _inverseScratch[pivotCol, pivotCol];
            for (int c = 0; c < n * 2; c++)
            {
                _inverseScratch[pivotCol, c] /= pivot;
            }

            for (int r = 0; r < n; r++)
            {
                if (r == pivotCol)
                {
                    continue;
                }

                float factor = _inverseScratch[r, pivotCol];
                if (Mathf.Abs(factor) < 0.000001f)
                {
                    continue;
                }

                for (int c = 0; c < n * 2; c++)
                {
                    _inverseScratch[r, c] -= factor * _inverseScratch[pivotCol, c];
                }
            }
        }

        for (int r = 0; r < n; r++)
        {
            for (int c = 0; c < n; c++)
            {
                _mixMatrixInverse[r, c] = _inverseScratch[r, c + n];
            }
        }

        return true;
    }

    private void RefreshRotorGeometry()
    {
        _rotorGeometryReady = false;
        _mixingReady = false;

        if (!AreRotorsConfigured())
        {
            return;
        }

        // U1 is total thrust. The other rows are the body torques produced by r x F
        // and rotor drag in Unity axes: x = pitch, y = yaw, z = roll.
        for (int i = 0; i < 4; i++)
        {
            Rotor rotor = rotors[i];
            _rotorPointsBody[i] = transform.InverseTransformPoint(rotor.transform.position) - centerOfMassLocal;
            _rotorSpinDirections[i] = rotor.yawSpinDirection;
            _rotorMaxVisualSpinSpeeds[i] = rotor.maxVisualSpinSpeed;
            _rotorVisuals[i] = rotor.visual != null ? rotor.visual : rotor.transform;
            _rotorVisualSpinAxes[i] = rotor.visualSpinAxis.sqrMagnitude > 0.0001f ? rotor.visualSpinAxis.normalized : Vector3.up;
            _rotorYawFactors[i] = _rotorSpinDirections[i] * yawDragTorqueCoefficient;

            _mixMatrix[0, i] = 1f;
            _mixMatrix[1, i] = -_rotorPointsBody[i].z;
            _mixMatrix[2, i] = _rotorYawFactors[i];
            _mixMatrix[3, i] = _rotorPointsBody[i].x;
        }

        _rotorGeometryReady = true;
        _mixingReady = TryBuildMixingInverse();
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
        if (!_rotorGeometryReady)
        {
            return;
        }

        for (int i = 0; i < 4; i++)
        {
            float normalizedThrust = Mathf.InverseLerp(minRotorThrust, maxRotorThrust, _rotorThrusts[i]);
            float spinSpeed = normalizedThrust * _rotorMaxVisualSpinSpeeds[i] * _rotorSpinDirections[i];
            _rotorVisuals[i].Rotate(_rotorVisualSpinAxes[i], spinSpeed * Time.deltaTime, Space.Self);
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
        _previousPositionWorld = _positionWorld;
        _previousRotationWorld = _rotationWorld;
        _hasPreviousPhysicsState = false;
        _manualYawHeading = transform.eulerAngles.y;
    }

    private void CachePreviousPhysicsState()
    {
        _previousPositionWorld = _positionWorld;
        _previousRotationWorld = _rotationWorld;
        _hasPreviousPhysicsState = true;
    }

    private void ApplyRenderedTransform()
    {
        if (!interpolateRenderedTransform || !_hasPreviousPhysicsState || Time.fixedDeltaTime <= 0f)
        {
            ApplySimulationTransform();
            return;
        }

        float alpha = Mathf.Clamp01((Time.time - Time.fixedTime) / Time.fixedDeltaTime);
        Vector3 renderPosition = Vector3.Lerp(_previousPositionWorld, _positionWorld, alpha);
        Quaternion renderRotation = Quaternion.Slerp(_previousRotationWorld, _rotationWorld, alpha);
        transform.SetPositionAndRotation(renderPosition, renderRotation);
    }

    private void ApplySimulationTransform()
    {
        transform.SetPositionAndRotation(_positionWorld, _rotationWorld);
    }

    private void ResetControllers()
    {
        _verticalSpeedIntegral = 0f;
        _lastVerticalSpeedError = 0f;
        _linearVelocityWorld = Vector3.zero;
        _angularVelocityBody = Vector3.zero;

        for (int i = 0; i < _rotorThrusts.Length; i++)
        {
            _rotorThrusts[i] = 0f;
        }
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

    private void OnDrawGizmosSelected()
    {
        if (!drawDebugGizmos)
        {
            return;
        }

        Gizmos.color = Color.yellow;
        if (rotors != null)
        {
            for (int i = 0; i < rotors.Length; i++)
            {
                if (rotors[i] == null || rotors[i].transform == null)
                {
                    continue;
                }

                Gizmos.DrawSphere(rotors[i].transform.position, 0.05f);
            }
        }

        Gizmos.color = Color.cyan;
        Gizmos.DrawSphere(transform.TransformPoint(centerOfMassLocal), 0.06f);
    }
}
