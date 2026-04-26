using UnityEngine;

public class DroneFollowCamera : MonoBehaviour
{
    [SerializeField] private Transform target;
    [SerializeField, Min(1f)] private float followDistance = 5.5f;
    [SerializeField] private float followHeight = 2.4f;
    [SerializeField] private float sideOffset = 0f;
    [SerializeField, Min(0f)] private float lookAtHeight = 0.8f;
    [SerializeField, Min(0f)] private float lookAheadDistance = 1.2f;
    [SerializeField, Min(0.01f)] private float positionSmoothTime = 0.18f;
    [SerializeField, Min(0f)] private float yawSharpness = 5f;
    [SerializeField, Min(0f)] private float velocitySharpness = 8f;
    [SerializeField] private float lookSharpness = 12f;
    [SerializeField] private bool snapOnEnable = true;

    private Vector3 positionVelocity;
    private Vector3 smoothedTargetVelocity;
    private Vector3 previousTargetPosition;
    private bool hasPreviousTargetPosition;
    private float smoothedYaw;
    private bool hasSmoothedYaw;

    public void SetTarget(Transform newTarget)
    {
        target = newTarget;
        ResetCameraState();
    }

    public void Configure(float distance, float height)
    {
        followDistance = Mathf.Max(1f, distance);
        followHeight = height;
        ResetCameraState();
    }

    private void OnEnable()
    {
        ResetCameraState();
    }

    private void LateUpdate()
    {
        if (target == null)
        {
            return;
        }

        float deltaTime = Mathf.Max(Time.deltaTime, 0.0001f);
        UpdateTargetVelocity(deltaTime);
        UpdateSmoothedYaw(deltaTime);

        Vector3 desiredPosition = CalculateDesiredPosition();
        transform.position = Vector3.SmoothDamp(
            transform.position,
            desiredPosition,
            ref positionVelocity,
            positionSmoothTime,
            Mathf.Infinity,
            deltaTime);

        Vector3 lookDirection = CalculateLookPoint() - transform.position;
        if (lookDirection.sqrMagnitude < 0.0001f)
        {
            return;
        }

        Quaternion desiredRotation = Quaternion.LookRotation(lookDirection.normalized, Vector3.up);
        float lookT = 1f - Mathf.Exp(-lookSharpness * deltaTime);
        transform.rotation = Quaternion.Slerp(transform.rotation, desiredRotation, lookT);
    }

    private void ResetCameraState()
    {
        positionVelocity = Vector3.zero;
        smoothedTargetVelocity = Vector3.zero;
        hasPreviousTargetPosition = false;
        hasSmoothedYaw = false;

        if (snapOnEnable && target != null)
        {
            smoothedYaw = target.eulerAngles.y;
            hasSmoothedYaw = true;
            previousTargetPosition = target.position;
            hasPreviousTargetPosition = true;
            transform.position = CalculateDesiredPosition();

            Vector3 lookDirection = CalculateLookPoint() - transform.position;
            if (lookDirection.sqrMagnitude > 0.0001f)
            {
                transform.rotation = Quaternion.LookRotation(lookDirection.normalized, Vector3.up);
            }
        }
    }

    private void UpdateTargetVelocity(float deltaTime)
    {
        if (!hasPreviousTargetPosition)
        {
            previousTargetPosition = target.position;
            hasPreviousTargetPosition = true;
            return;
        }

        Vector3 rawVelocity = (target.position - previousTargetPosition) / deltaTime;
        float velocityT = 1f - Mathf.Exp(-velocitySharpness * deltaTime);
        smoothedTargetVelocity = Vector3.Lerp(smoothedTargetVelocity, rawVelocity, velocityT);
        previousTargetPosition = target.position;
    }

    private void UpdateSmoothedYaw(float deltaTime)
    {
        float targetYaw = target.eulerAngles.y;
        if (!hasSmoothedYaw)
        {
            smoothedYaw = targetYaw;
            hasSmoothedYaw = true;
            return;
        }

        float yawT = 1f - Mathf.Exp(-yawSharpness * deltaTime);
        smoothedYaw = Mathf.LerpAngle(smoothedYaw, targetYaw, yawT);
    }

    private Vector3 CalculateDesiredPosition()
    {
        Quaternion yawRotation = Quaternion.Euler(0f, smoothedYaw, 0f);
        Vector3 offset = new Vector3(sideOffset, followHeight, -followDistance);
        return target.position + yawRotation * offset;
    }

    private Vector3 CalculateLookPoint()
    {
        Vector3 lookPoint = target.position + Vector3.up * lookAtHeight;
        if (lookAheadDistance > 0f && smoothedTargetVelocity.sqrMagnitude > 0.01f)
        {
            lookPoint += smoothedTargetVelocity.normalized * lookAheadDistance;
        }

        return lookPoint;
    }
}
