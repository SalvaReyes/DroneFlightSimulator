using UnityEngine;

[DefaultExecutionOrder(-50)]
public sealed class DroneCinemachineTargetRig : MonoBehaviour
{
    [SerializeField] private Transform drone;
    [SerializeField] private Transform followTarget;
    [SerializeField] private Transform lookTarget;
    [SerializeField, Min(0f)] private float lookHeight = 0.8f;
    [SerializeField, Min(0f)] private float lookAheadDistance = 1.4f;
    [SerializeField, Min(0f)] private float positionSharpness = 18f;
    [SerializeField, Min(0f)] private float yawSharpness = 7f;
    [SerializeField, Min(0f)] private float velocitySharpness = 8f;

    private Vector3 smoothedPosition;
    private Vector3 smoothedVelocity;
    private Vector3 previousDronePosition;
    private float smoothedYaw;
    private bool hasState;

    public void SetDrone(Transform newDrone)
    {
        drone = newDrone;
        ResetState();
    }

    public void SetTargets(Transform newFollowTarget, Transform newLookTarget)
    {
        followTarget = newFollowTarget;
        lookTarget = newLookTarget;
        ResetState();
    }

    private void OnEnable()
    {
        ResetState();
    }

    private void LateUpdate()
    {
        if (drone == null)
        {
            return;
        }

        float deltaTime = Mathf.Max(Time.deltaTime, 0.0001f);
        if (!hasState)
        {
            ResetState();
            return;
        }

        Vector3 rawVelocity = (drone.position - previousDronePosition) / deltaTime;
        float velocityT = DampT(velocitySharpness, deltaTime);
        smoothedVelocity = Vector3.Lerp(smoothedVelocity, rawVelocity, velocityT);
        previousDronePosition = drone.position;

        float positionT = DampT(positionSharpness, deltaTime);
        smoothedPosition = Vector3.Lerp(smoothedPosition, drone.position, positionT);

        float yaw = CalculateDroneYaw();
        float yawT = DampT(yawSharpness, deltaTime);
        smoothedYaw = Mathf.LerpAngle(smoothedYaw, yaw, yawT);

        ApplyTargets();
    }

    private void ResetState()
    {
        hasState = drone != null;
        if (!hasState)
        {
            return;
        }

        smoothedPosition = drone.position;
        previousDronePosition = drone.position;
        smoothedVelocity = Vector3.zero;
        smoothedYaw = CalculateDroneYaw();
        ApplyTargets();
    }

    private void ApplyTargets()
    {
        Quaternion yawRotation = Quaternion.Euler(0f, smoothedYaw, 0f);

        if (followTarget != null)
        {
            followTarget.SetPositionAndRotation(smoothedPosition, yawRotation);
        }

        if (lookTarget != null)
        {
            Vector3 lookPosition = smoothedPosition + Vector3.up * lookHeight;
            Vector3 horizontalVelocity = Vector3.ProjectOnPlane(smoothedVelocity, Vector3.up);
            if (horizontalVelocity.sqrMagnitude > 0.01f)
            {
                lookPosition += horizontalVelocity.normalized * lookAheadDistance;
            }

            lookTarget.SetPositionAndRotation(lookPosition, yawRotation);
        }
    }

    private float CalculateDroneYaw()
    {
        Vector3 forward = Vector3.ProjectOnPlane(drone.forward, Vector3.up);
        if (forward.sqrMagnitude < 0.0001f)
        {
            forward = Vector3.ProjectOnPlane(drone.up, Vector3.up);
        }

        if (forward.sqrMagnitude < 0.0001f)
        {
            return smoothedYaw;
        }

        return Quaternion.LookRotation(forward.normalized, Vector3.up).eulerAngles.y;
    }

    private static float DampT(float sharpness, float deltaTime)
    {
        return sharpness <= 0f ? 1f : 1f - Mathf.Exp(-sharpness * deltaTime);
    }
}
