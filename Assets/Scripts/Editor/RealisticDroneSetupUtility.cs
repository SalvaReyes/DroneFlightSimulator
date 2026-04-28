#if UNITY_EDITOR
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

public static class RealisticDroneSetupUtility
{
    private readonly struct RotorSpec
    {
        public readonly string MeshName;
        public readonly string PivotName;
        public readonly string Label;
        public readonly float SpinDirection;

        public RotorSpec(string meshName, string pivotName, string label, float spinDirection)
        {
            MeshName = meshName;
            PivotName = pivotName;
            Label = label;
            SpinDirection = spinDirection;
        }
    }

    private static readonly RotorSpec[] RotorSpecs =
    {
        new RotorSpec("rotor FL", "Rotor_FrontLeft_Pivot", "Rotor_FrontLeft", 1f),
        new RotorSpec("rotor FR", "Rotor_FrontRight_Pivot", "Rotor_FrontRight", -1f),
        new RotorSpec("rotor RR", "Rotor_RearRight_Pivot", "Rotor_RearRight", 1f),
        new RotorSpec("rotor RL", "Rotor_RearLeft_Pivot", "Rotor_RearLeft", -1f)
    };

    [MenuItem("Drone Simulator/Geometry/Configure Drone (1) Flight Controller")]
    public static void ConfigureDroneOneFlightController()
    {
        GameObject drone = FindDroneRoot();
        if (drone == null)
        {
            Debug.LogError("Could not configure realistic drone geometry: no Drone (1)/Drone root with Body and rotor meshes was found.");
            return;
        }

        QuadcopterController controller = drone.GetComponent<QuadcopterController>();
        if (controller == null)
        {
            controller = Undo.AddComponent<QuadcopterController>(drone);
        }

        Transform[] rotorPivots = new Transform[RotorSpecs.Length];
        for (int i = 0; i < RotorSpecs.Length; i++)
        {
            RotorSpec spec = RotorSpecs[i];
            Transform rotorMesh = FindDeepChild(drone.transform, spec.MeshName);
            if (rotorMesh == null)
            {
                Debug.LogError($"Could not configure realistic drone geometry: missing rotor mesh '{spec.MeshName}'.");
                return;
            }

            rotorPivots[i] = CreateOrUpdateRotorPivot(drone.transform, rotorMesh, spec.PivotName);
        }

        ConfigureController(controller, drone.transform, rotorPivots);
        RetargetFollowCameras(drone.transform);
        DisableLegacyDrone(drone);

        Selection.activeGameObject = drone;
        EditorGUIUtility.PingObject(drone);
        EditorSceneManager.MarkSceneDirty(SceneManager.GetActiveScene());
        EditorSceneManager.SaveOpenScenes();

        Debug.Log($"Configured realistic drone geometry for '{drone.name}' with centered rotor pivots.");
    }

    private static GameObject FindDroneRoot()
    {
        GameObject preferred = GameObject.Find("Drone (1)");
        if (IsRealisticDroneRoot(preferred))
        {
            return preferred;
        }

        GameObject fallback = GameObject.Find("Drone");
        if (IsRealisticDroneRoot(fallback))
        {
            return fallback;
        }

        foreach (GameObject root in SceneManager.GetActiveScene().GetRootGameObjects())
        {
            if (IsRealisticDroneRoot(root))
            {
                return root;
            }
        }

        return null;
    }

    private static bool IsRealisticDroneRoot(GameObject candidate)
    {
        return candidate != null
            && candidate.transform.Find("Body") != null
            && FindDeepChild(candidate.transform, "rotor FL") != null
            && FindDeepChild(candidate.transform, "rotor FR") != null
            && FindDeepChild(candidate.transform, "rotor RR") != null
            && FindDeepChild(candidate.transform, "rotor RL") != null;
    }

    private static Transform CreateOrUpdateRotorPivot(Transform droneRoot, Transform rotorMesh, string pivotName)
    {
        Transform pivot = droneRoot.Find(pivotName);
        if (pivot == null)
        {
            GameObject pivotObject = new GameObject(pivotName);
            Undo.RegisterCreatedObjectUndo(pivotObject, $"Create {pivotName}");
            pivot = pivotObject.transform;
            pivot.SetParent(droneRoot, false);
        }

        Vector3 rotorCenter = GetRendererCenter(rotorMesh);
        Undo.RecordObject(pivot, $"Position {pivotName}");
        pivot.SetParent(droneRoot, false);
        pivot.position = rotorCenter;
        pivot.rotation = droneRoot.rotation;
        pivot.localScale = Vector3.one;

        if (rotorMesh.parent != pivot)
        {
            Vector3 worldPosition = rotorMesh.position;
            Quaternion worldRotation = rotorMesh.rotation;
            Vector3 localScale = rotorMesh.localScale;

            Undo.SetTransformParent(rotorMesh, pivot, $"Parent {rotorMesh.name} To Pivot");
            rotorMesh.SetPositionAndRotation(worldPosition, worldRotation);
            rotorMesh.localScale = localScale;
        }

        EditorUtility.SetDirty(pivot);
        EditorUtility.SetDirty(rotorMesh);
        return pivot;
    }

    private static Vector3 GetRendererCenter(Transform rotorMesh)
    {
        Renderer renderer = rotorMesh.GetComponent<Renderer>();
        if (renderer != null)
        {
            return renderer.bounds.center;
        }

        Renderer childRenderer = rotorMesh.GetComponentInChildren<Renderer>();
        return childRenderer != null ? childRenderer.bounds.center : rotorMesh.position;
    }

    private static void ConfigureController(QuadcopterController controller, Transform droneRoot, Transform[] rotorPivots)
    {
        SerializedObject serializedController = new SerializedObject(controller);

        SerializedProperty rotorsProperty = serializedController.FindProperty("rotors");
        rotorsProperty.arraySize = RotorSpecs.Length;

        Vector3 centerOfMassLocal = Vector3.zero;
        for (int i = 0; i < RotorSpecs.Length; i++)
        {
            RotorSpec spec = RotorSpecs[i];
            Transform rotorPivot = rotorPivots[i];
            centerOfMassLocal += droneRoot.InverseTransformPoint(rotorPivot.position);

            SerializedProperty rotorProperty = rotorsProperty.GetArrayElementAtIndex(i);
            rotorProperty.FindPropertyRelative("label").stringValue = spec.Label;
            rotorProperty.FindPropertyRelative("transform").objectReferenceValue = rotorPivot;
            rotorProperty.FindPropertyRelative("yawSpinDirection").floatValue = spec.SpinDirection;
            rotorProperty.FindPropertyRelative("visual").objectReferenceValue = rotorPivot;
            rotorProperty.FindPropertyRelative("visualSpinAxis").vector3Value = Vector3.up;
            rotorProperty.FindPropertyRelative("maxVisualSpinSpeed").floatValue = 3800f;
        }

        centerOfMassLocal /= RotorSpecs.Length;

        SetFloat(serializedController, "mass", 1.4f);
        SetVector3(serializedController, "inertiaTensorBody", new Vector3(0.02f, 0.04f, 0.02f));
        SetFloat(serializedController, "gravityAcceleration", 9.81f);
        SetFloat(serializedController, "linearDrag", 0.12f);
        SetFloat(serializedController, "angularDrag", 0.18f);
        SetFloat(serializedController, "rotorGyroscopicCoefficient", 0.0006f);
        SetFloat(serializedController, "maxLinearSpeed", 20f);
        SetFloat(serializedController, "maxAngularSpeed", 20f);
        SetFloat(serializedController, "minRotorThrust", 0f);
        SetFloat(serializedController, "maxRotorThrust", 24f);
        SetFloat(serializedController, "yawDragTorqueCoefficient", 0.04f);
        SetBool(serializedController, "autoCenterOfMassFromRotors", true);
        SetVector3(serializedController, "centerOfMassLocal", centerOfMassLocal);
        SetFloat(serializedController, "manualMaxClimbRate", 2.8f);
        SetFloat(serializedController, "manualMaxTiltAngle", 18f);
        SetFloat(serializedController, "manualYawRate", 70f);
        SetFloat(serializedController, "verticalSpeedKp", 3.8f);
        SetFloat(serializedController, "verticalSpeedKi", 0.65f);
        SetFloat(serializedController, "verticalSpeedKd", 0.35f);
        SetFloat(serializedController, "verticalIntegralLimit", 3f);
        SetFloat(serializedController, "attitudeAngleGain", 6f);
        SetFloat(serializedController, "yawHeadingGain", 4f);
        SetVector3(serializedController, "angularRateGain", new Vector3(7.5f, 5.5f, 7.5f));
        SetFloat(serializedController, "maxPitchTorque", 12f);
        SetFloat(serializedController, "maxYawTorque", 8f);
        SetFloat(serializedController, "maxRollTorque", 12f);
        SetBool(serializedController, "drawDebugGizmos", true);

        serializedController.ApplyModifiedProperties();
        EditorUtility.SetDirty(controller);
    }

    private static void RetargetFollowCameras(Transform droneRoot)
    {
        DroneFollowCamera[] followCameras = Object.FindObjectsByType<DroneFollowCamera>(FindObjectsInactive.Include);

        foreach (DroneFollowCamera followCamera in followCameras)
        {
            SerializedObject serializedCamera = new SerializedObject(followCamera);
            SerializedProperty targetProperty = serializedCamera.FindProperty("target");
            if (targetProperty == null)
            {
                continue;
            }

            targetProperty.objectReferenceValue = droneRoot;
            serializedCamera.ApplyModifiedProperties();
            EditorUtility.SetDirty(followCamera);
        }
    }

    private static void DisableLegacyDrone(GameObject activeDrone)
    {
        GameObject legacyDrone = GameObject.Find("Drone");
        if (legacyDrone == null || legacyDrone == activeDrone)
        {
            return;
        }

        Undo.RecordObject(legacyDrone, "Disable Legacy Drone");
        legacyDrone.SetActive(false);
        EditorUtility.SetDirty(legacyDrone);
    }

    private static Transform FindDeepChild(Transform parent, string childName)
    {
        foreach (Transform child in parent)
        {
            if (child.name == childName)
            {
                return child;
            }

            Transform nested = FindDeepChild(child, childName);
            if (nested != null)
            {
                return nested;
            }
        }

        return null;
    }

    private static void SetFloat(SerializedObject serializedObject, string propertyName, float value)
    {
        SerializedProperty property = serializedObject.FindProperty(propertyName);
        if (property != null)
        {
            property.floatValue = value;
        }
    }

    private static void SetBool(SerializedObject serializedObject, string propertyName, bool value)
    {
        SerializedProperty property = serializedObject.FindProperty(propertyName);
        if (property != null)
        {
            property.boolValue = value;
        }
    }

    private static void SetVector3(SerializedObject serializedObject, string propertyName, Vector3 value)
    {
        SerializedProperty property = serializedObject.FindProperty(propertyName);
        if (property != null)
        {
            property.vector3Value = value;
        }
    }
}
#endif
