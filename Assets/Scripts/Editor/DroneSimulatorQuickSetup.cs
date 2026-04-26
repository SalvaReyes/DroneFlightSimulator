#if UNITY_EDITOR
using System.IO;
using UnityEditor;
using UnityEngine;

public static class DroneSimulatorQuickSetup
{
    private static readonly string[] RotorNames =
    {
        "Rotor_FrontLeft",
        "Rotor_FrontRight",
        "Rotor_RearRight",
        "Rotor_RearLeft"
    };

    private static readonly Vector3[] RotorLocalPositions =
    {
        new Vector3(-0.28f, 0.02f, 0.28f),
        new Vector3(0.28f, 0.02f, 0.28f),
        new Vector3(0.28f, 0.02f, -0.28f),
        new Vector3(-0.28f, 0.02f, -0.28f)
    };

    private static readonly float[] RotorSpinDirections =
    {
        1f,
        -1f,
        1f,
        -1f
    };

    private const string GeneratedFolder = "Assets/Art/Generated/DroneTesting";

    private sealed class MaterialSet
    {
        public Material Ground;
        public Material Path;
        public Material Gate;
        public Material Waypoint;
        public Material Obstacle;
        public Material StartPad;
        public Material EndPad;
        public Material DroneBody;
        public Material Rotor;
        public Material Target;
    }

    [MenuItem("Tools/Drone Simulator/Create Demo Drone Rig")]
    public static void CreateDemoDroneRig()
    {
        RemoveExistingDemoObjects();

        MaterialSet materials = BuildOrLoadTestingMaterials();
        Transform[] waypoints = BuildTestingCourse(materials);

        Vector3 spawnPoint = waypoints.Length > 0
            ? waypoints[0].position + Vector3.up * 1.4f
            : new Vector3(0f, 2f, 0f);

        Vector3 initialTargetPoint = waypoints.Length > 1
            ? waypoints[1].position
            : spawnPoint + new Vector3(6f, 0f, 0f);

        GameObject root = new GameObject("Drone");
        Undo.RegisterCreatedObjectUndo(root, "Create Demo Drone Rig");
        root.transform.position = spawnPoint;

        QuadcopterController controller = Undo.AddComponent<QuadcopterController>(root);

        CreateBody(root.transform, materials);
        Transform[] rotorTransforms = CreateRotors(root.transform, materials);
        Transform targetMarker = CreateTargetMarker(initialTargetPoint, materials.Target);

        ConfigureController(controller, rotorTransforms, targetMarker);
        SetupMainCamera(root.transform);

        Selection.activeGameObject = root;
        EditorGUIUtility.PingObject(root);
    }

    [MenuItem("Tools/Drone Simulator/Create Testing Course Only")]
    public static void CreateTestingCourseOnly()
    {
        DestroyIfExists("DroneTestingCourse");
        DestroyIfExists("Ground");

        MaterialSet materials = BuildOrLoadTestingMaterials();
        Transform[] waypoints = BuildTestingCourse(materials);

        if (waypoints.Length > 0)
        {
            Selection.activeGameObject = waypoints[0].gameObject;
            EditorGUIUtility.PingObject(waypoints[0]);
        }
    }

    private static void RemoveExistingDemoObjects()
    {
        DestroyIfExists("Drone");
        DestroyIfExists("AutoTarget");
        DestroyIfExists("DroneTestingCourse");
        DestroyIfExists("Ground");
    }

    private static void DestroyIfExists(string objectName)
    {
        GameObject existing = GameObject.Find(objectName);
        if (existing != null)
        {
            Undo.DestroyObjectImmediate(existing);
        }
    }

    private static MaterialSet BuildOrLoadTestingMaterials()
    {
        EnsureFolder("Assets", "Art");
        EnsureFolder("Assets/Art", "Generated");
        EnsureFolder("Assets/Art/Generated", "DroneTesting");

        Texture2D groundTexture = GetOrCreateCheckerTexture(
            $"{GeneratedFolder}/T_GroundChecker.asset",
            new Color(0.13f, 0.14f, 0.15f),
            new Color(0.21f, 0.23f, 0.25f),
            512,
            16);

        Texture2D pathTexture = GetOrCreateCheckerTexture(
            $"{GeneratedFolder}/T_PathChecker.asset",
            new Color(0.95f, 0.95f, 0.95f),
            new Color(0.20f, 0.65f, 0.95f),
            256,
            8);

        Texture2D gateTexture = GetOrCreateCheckerTexture(
            $"{GeneratedFolder}/T_GateChecker.asset",
            new Color(1.0f, 0.25f, 0.20f),
            new Color(0.95f, 0.95f, 0.95f),
            256,
            6);

        MaterialSet materials = new MaterialSet
        {
            Ground = GetOrCreateMaterial($"{GeneratedFolder}/M_Ground.mat", Color.white, groundTexture, new Vector2(24f, 24f)),
            Path = GetOrCreateMaterial($"{GeneratedFolder}/M_Path.mat", Color.white, pathTexture, new Vector2(8f, 8f)),
            Gate = GetOrCreateMaterial($"{GeneratedFolder}/M_Gate.mat", Color.white, gateTexture, new Vector2(2f, 2f)),
            Waypoint = GetOrCreateMaterial($"{GeneratedFolder}/M_Waypoint.mat", new Color(0.25f, 1.0f, 0.45f), null, Vector2.one, true),
            Obstacle = GetOrCreateMaterial($"{GeneratedFolder}/M_Obstacle.mat", new Color(1.0f, 0.70f, 0.15f)),
            StartPad = GetOrCreateMaterial($"{GeneratedFolder}/M_StartPad.mat", new Color(0.10f, 0.85f, 0.35f), null, Vector2.one, true),
            EndPad = GetOrCreateMaterial($"{GeneratedFolder}/M_EndPad.mat", new Color(0.90f, 0.20f, 0.20f), null, Vector2.one, true),
            DroneBody = GetOrCreateMaterial($"{GeneratedFolder}/M_DroneBody.mat", new Color(0.80f, 0.85f, 0.90f)),
            Rotor = GetOrCreateMaterial($"{GeneratedFolder}/M_Rotor.mat", new Color(0.10f, 0.12f, 0.14f)),
            Target = GetOrCreateMaterial($"{GeneratedFolder}/M_Target.mat", new Color(0.95f, 0.90f, 0.15f), null, Vector2.one, true)
        };

        AssetDatabase.SaveAssets();
        return materials;
    }

    private static void EnsureFolder(string parent, string child)
    {
        string fullPath = $"{parent}/{child}";
        if (!AssetDatabase.IsValidFolder(fullPath))
        {
            AssetDatabase.CreateFolder(parent, child);
        }
    }

    private static Texture2D GetOrCreateCheckerTexture(string path, Color colorA, Color colorB, int size, int cellsPerAxis)
    {
        Texture2D texture = AssetDatabase.LoadAssetAtPath<Texture2D>(path);
        if (texture != null)
        {
            return texture;
        }

        texture = new Texture2D(size, size, TextureFormat.RGBA32, false)
        {
            name = Path.GetFileNameWithoutExtension(path),
            wrapMode = TextureWrapMode.Repeat,
            filterMode = FilterMode.Point
        };

        int cellSize = Mathf.Max(1, size / cellsPerAxis);
        Color[] pixels = new Color[size * size];

        for (int y = 0; y < size; y++)
        {
            for (int x = 0; x < size; x++)
            {
                bool useA = ((x / cellSize) + (y / cellSize)) % 2 == 0;
                pixels[y * size + x] = useA ? colorA : colorB;
            }
        }

        texture.SetPixels(pixels);
        texture.Apply();

        AssetDatabase.CreateAsset(texture, path);
        EditorUtility.SetDirty(texture);
        return texture;
    }

    private static Material GetOrCreateMaterial(string path, Color baseColor, Texture2D texture = null, Vector2? tiling = null, bool emissive = false)
    {
        Material material = AssetDatabase.LoadAssetAtPath<Material>(path);
        if (material == null)
        {
            Shader shader = Shader.Find("Universal Render Pipeline/Lit");
            if (shader == null)
            {
                shader = Shader.Find("Standard");
            }

            material = new Material(shader)
            {
                name = Path.GetFileNameWithoutExtension(path)
            };

            AssetDatabase.CreateAsset(material, path);
        }

        if (material.HasProperty("_BaseColor"))
        {
            material.SetColor("_BaseColor", baseColor);
        }

        if (material.HasProperty("_Color"))
        {
            material.SetColor("_Color", baseColor);
        }

        if (texture != null)
        {
            if (material.HasProperty("_BaseMap"))
            {
                material.SetTexture("_BaseMap", texture);
            }

            if (material.HasProperty("_MainTex"))
            {
                material.SetTexture("_MainTex", texture);
            }
        }

        Vector2 textureTiling = tiling ?? Vector2.one;
        if (material.HasProperty("_BaseMap"))
        {
            material.SetTextureScale("_BaseMap", textureTiling);
        }

        if (material.HasProperty("_MainTex"))
        {
            material.SetTextureScale("_MainTex", textureTiling);
        }

        if (material.HasProperty("_Metallic"))
        {
            material.SetFloat("_Metallic", 0f);
        }

        if (material.HasProperty("_Smoothness"))
        {
            material.SetFloat("_Smoothness", 0.16f);
        }

        if (material.HasProperty("_EmissionColor"))
        {
            if (emissive)
            {
                material.EnableKeyword("_EMISSION");
                material.SetColor("_EmissionColor", baseColor * 1.4f);
            }
            else
            {
                material.SetColor("_EmissionColor", Color.black);
            }
        }

        EditorUtility.SetDirty(material);
        return material;
    }

    private static Transform[] BuildTestingCourse(MaterialSet materials)
    {
        GameObject courseRoot = new GameObject("DroneTestingCourse");
        Undo.RegisterCreatedObjectUndo(courseRoot, "Create Drone Testing Course");

        CreatePrimitive(
            "Ground",
            PrimitiveType.Plane,
            courseRoot.transform,
            Vector3.zero,
            Quaternion.identity,
            new Vector3(12f, 1f, 12f),
            materials.Ground);

        Vector3[] waypointPositions =
        {
            new Vector3(0f, 1.8f, 0f),
            new Vector3(8f, 2.4f, 0f),
            new Vector3(12f, 3.2f, 8f),
            new Vector3(2f, 2.8f, 12f),
            new Vector3(-8f, 4.0f, 8f),
            new Vector3(-12f, 3.0f, -2f),
            new Vector3(-2f, 2.0f, -10f),
            new Vector3(8f, 2.2f, -8f)
        };

        GameObject waypointRoot = new GameObject("Waypoints");
        Undo.RegisterCreatedObjectUndo(waypointRoot, "Create Waypoint Root");
        waypointRoot.transform.SetParent(courseRoot.transform, false);

        Transform[] waypoints = new Transform[waypointPositions.Length];

        for (int i = 0; i < waypointPositions.Length; i++)
        {
            Vector3 point = waypointPositions[i];

            GameObject marker = CreatePrimitive(
                $"Waypoint_{i + 1:00}",
                PrimitiveType.Sphere,
                waypointRoot.transform,
                point,
                Quaternion.identity,
                Vector3.one * 0.5f,
                materials.Waypoint,
                true);

            waypoints[i] = marker.transform;

            float poleHeight = Mathf.Max(0.1f, point.y * 0.5f);
            CreatePrimitive(
                $"WaypointPole_{i + 1:00}",
                PrimitiveType.Cylinder,
                waypointRoot.transform,
                new Vector3(point.x, poleHeight, point.z),
                Quaternion.identity,
                new Vector3(0.07f, poleHeight, 0.07f),
                materials.Waypoint);
        }

        GameObject pathRoot = new GameObject("Path");
        Undo.RegisterCreatedObjectUndo(pathRoot, "Create Path Root");
        pathRoot.transform.SetParent(courseRoot.transform, false);

        for (int i = 0; i < waypointPositions.Length; i++)
        {
            Vector3 from = waypointPositions[i];
            Vector3 to = waypointPositions[(i + 1) % waypointPositions.Length];
            CreatePathSegment(pathRoot.transform, i + 1, from, to, materials.Path);
        }

        GameObject gateRoot = new GameObject("Gates");
        Undo.RegisterCreatedObjectUndo(gateRoot, "Create Gate Root");
        gateRoot.transform.SetParent(courseRoot.transform, false);

        for (int i = 1; i < waypointPositions.Length; i += 2)
        {
            Vector3 current = waypointPositions[i];
            Vector3 next = waypointPositions[(i + 1) % waypointPositions.Length];
            Vector3 direction = next - current;
            direction.y = 0f;

            if (direction.sqrMagnitude < 0.01f)
            {
                direction = Vector3.forward;
            }

            Quaternion rotation = Quaternion.LookRotation(direction.normalized, Vector3.up);
            CreateGate(gateRoot.transform, i + 1, current, rotation, materials.Gate);
        }

        GameObject obstacleRoot = new GameObject("Obstacles");
        Undo.RegisterCreatedObjectUndo(obstacleRoot, "Create Obstacle Root");
        obstacleRoot.transform.SetParent(courseRoot.transform, false);
        CreateObstacleField(obstacleRoot.transform, materials.Obstacle);

        CreatePad(courseRoot.transform, "StartPad", waypointPositions[0], 1.2f, materials.StartPad);
        CreatePad(courseRoot.transform, "FinishPad", waypointPositions[waypointPositions.Length - 1], 1.2f, materials.EndPad);

        return waypoints;
    }

    private static void CreatePathSegment(Transform parent, int index, Vector3 from, Vector3 to, Material material)
    {
        Vector3 flatDelta = new Vector3(to.x - from.x, 0f, to.z - from.z);
        float length = flatDelta.magnitude;
        if (length < 0.1f)
        {
            return;
        }

        Vector3 midpoint = (from + to) * 0.5f;
        midpoint.y = 0.03f;

        Quaternion rotation = Quaternion.LookRotation(flatDelta.normalized, Vector3.up);

        CreatePrimitive(
            $"Path_{index:00}",
            PrimitiveType.Cube,
            parent,
            midpoint,
            rotation,
            new Vector3(0.45f, 0.03f, length),
            material,
            true);
    }

    private static void CreateGate(Transform parent, int index, Vector3 center, Quaternion rotation, Material material)
    {
        const float width = 3.4f;
        const float height = 2.4f;
        const float thickness = 0.2f;

        GameObject gateRoot = new GameObject($"Gate_{index:00}");
        Undo.RegisterCreatedObjectUndo(gateRoot, "Create Gate");
        gateRoot.transform.SetParent(parent, false);
        gateRoot.transform.localPosition = center;
        gateRoot.transform.localRotation = rotation;

        CreatePrimitive("Left", PrimitiveType.Cube, gateRoot.transform, new Vector3(-width * 0.5f, 0f, 0f), Quaternion.identity, new Vector3(thickness, height, thickness), material);
        CreatePrimitive("Right", PrimitiveType.Cube, gateRoot.transform, new Vector3(width * 0.5f, 0f, 0f), Quaternion.identity, new Vector3(thickness, height, thickness), material);
        CreatePrimitive("Top", PrimitiveType.Cube, gateRoot.transform, new Vector3(0f, height * 0.5f, 0f), Quaternion.identity, new Vector3(width + thickness * 2f, thickness, thickness), material);
        CreatePrimitive("Bottom", PrimitiveType.Cube, gateRoot.transform, new Vector3(0f, -height * 0.5f, 0f), Quaternion.identity, new Vector3(width + thickness * 2f, thickness, thickness), material);
    }

    private static void CreateObstacleField(Transform parent, Material material)
    {
        Vector3[] positions =
        {
            new Vector3(3f, 0f, 5f),
            new Vector3(6f, 0f, 7f),
            new Vector3(-4f, 0f, 6f),
            new Vector3(-7f, 0f, 3f),
            new Vector3(-5f, 0f, -4f),
            new Vector3(2f, 0f, -5f),
            new Vector3(5f, 0f, -2f)
        };

        float[] heights = { 2.4f, 3.1f, 2.2f, 3.6f, 2.9f, 3.4f, 2.6f };

        for (int i = 0; i < positions.Length; i++)
        {
            float halfHeight = heights[i] * 0.5f;
            CreatePrimitive(
                $"Pylon_{i + 1:00}",
                PrimitiveType.Cylinder,
                parent,
                new Vector3(positions[i].x, halfHeight, positions[i].z),
                Quaternion.identity,
                new Vector3(0.25f, halfHeight, 0.25f),
                material);
        }
    }

    private static void CreatePad(Transform parent, string name, Vector3 nearPoint, float radius, Material material)
    {
        CreatePrimitive(
            name,
            PrimitiveType.Cylinder,
            parent,
            new Vector3(nearPoint.x, 0.03f, nearPoint.z),
            Quaternion.identity,
            new Vector3(radius, 0.03f, radius),
            material,
            true);
    }

    private static void CreateBody(Transform parent, MaterialSet materials)
    {
        GameObject body = CreatePrimitive("Body", PrimitiveType.Cube, parent, Vector3.zero, Quaternion.identity, new Vector3(0.5f, 0.1f, 0.5f), materials.DroneBody);

        if (body.TryGetComponent(out Collider bodyCollider))
        {
            bodyCollider.sharedMaterial = null;
        }

        for (int i = 0; i < 2; i++)
        {
            CreatePrimitive(
                i == 0 ? "Arm_X" : "Arm_Z",
                PrimitiveType.Cube,
                parent,
                Vector3.zero,
                Quaternion.identity,
                i == 0 ? new Vector3(0.65f, 0.04f, 0.06f) : new Vector3(0.06f, 0.04f, 0.65f),
                materials.DroneBody);
        }
    }

    private static Transform[] CreateRotors(Transform parent, MaterialSet materials)
    {
        Transform[] rotorTransforms = new Transform[4];

        for (int i = 0; i < 4; i++)
        {
            GameObject rotor = new GameObject(RotorNames[i]);
            Undo.RegisterCreatedObjectUndo(rotor, "Create Rotor");
            rotor.transform.SetParent(parent, false);
            rotor.transform.localPosition = RotorLocalPositions[i];

            GameObject visual = CreatePrimitive(
                "Visual",
                PrimitiveType.Cylinder,
                rotor.transform,
                Vector3.zero,
                Quaternion.identity,
                new Vector3(0.09f, 0.006f, 0.09f),
                materials.Rotor,
                true);

            // Add simple blades so rotation is visually obvious during play mode.
            CreatePrimitive(
                "Blade_A",
                PrimitiveType.Cube,
                visual.transform,
                Vector3.zero,
                Quaternion.identity,
                new Vector3(0.18f, 0.0025f, 0.018f),
                materials.Path,
                true);

            CreatePrimitive(
                "Blade_B",
                PrimitiveType.Cube,
                visual.transform,
                Vector3.zero,
                Quaternion.identity,
                new Vector3(0.018f, 0.0025f, 0.18f),
                materials.Target,
                true);

            rotorTransforms[i] = rotor.transform;
        }

        return rotorTransforms;
    }

    private static Transform CreateTargetMarker(Vector3 worldPosition, Material material)
    {
        GameObject marker = CreatePrimitive(
            "AutoTarget",
            PrimitiveType.Sphere,
            null,
            worldPosition,
            Quaternion.identity,
            Vector3.one * 0.35f,
            material,
            true);

        marker.transform.position = worldPosition;
        return marker.transform;
    }

    private static void ConfigureController(QuadcopterController controller, Transform[] rotors, Transform targetMarker)
    {
        SerializedObject serializedController = new SerializedObject(controller);

        SerializedProperty flightModeProperty = serializedController.FindProperty("flightMode");
        flightModeProperty.enumValueIndex = 0;

        SerializedProperty rotorsProperty = serializedController.FindProperty("rotors");
        rotorsProperty.arraySize = 4;

        for (int i = 0; i < 4; i++)
        {
            SerializedProperty rotorProperty = rotorsProperty.GetArrayElementAtIndex(i);
            rotorProperty.FindPropertyRelative("label").stringValue = RotorNames[i];
            rotorProperty.FindPropertyRelative("transform").objectReferenceValue = rotors[i];
            rotorProperty.FindPropertyRelative("yawSpinDirection").floatValue = RotorSpinDirections[i];
            rotorProperty.FindPropertyRelative("visual").objectReferenceValue = rotors[i].Find("Visual");
            rotorProperty.FindPropertyRelative("visualSpinAxis").vector3Value = Vector3.up;
            rotorProperty.FindPropertyRelative("maxVisualSpinSpeed").floatValue = 3800f;
        }

        serializedController.FindProperty("mass").floatValue = 1.4f;
        serializedController.FindProperty("inertiaTensorBody").vector3Value = new Vector3(0.02f, 0.04f, 0.02f);
        serializedController.FindProperty("gravityAcceleration").floatValue = 9.81f;
        serializedController.FindProperty("linearDrag").floatValue = 0.12f;
        serializedController.FindProperty("angularDrag").floatValue = 0.18f;
        serializedController.FindProperty("maxLinearSpeed").floatValue = 20f;
        serializedController.FindProperty("maxAngularSpeed").floatValue = 20f;

        serializedController.FindProperty("minRotorThrust").floatValue = 0f;
        serializedController.FindProperty("maxRotorThrust").floatValue = 24f;
        serializedController.FindProperty("manualMaxClimbRate").floatValue = 2.8f;
        serializedController.FindProperty("manualMaxTiltAngle").floatValue = 15f;
        serializedController.FindProperty("manualYawRate").floatValue = 55f;

        serializedController.FindProperty("autoTargetTransform").objectReferenceValue = targetMarker;
        serializedController.FindProperty("autoTargetPosition").vector3Value = targetMarker.position;
        serializedController.FindProperty("allowMouseClickTarget").boolValue = true;
        serializedController.FindProperty("faceTargetInAutoMode").boolValue = true;
        serializedController.FindProperty("autoMaxTiltAngle").floatValue = 30f;
        serializedController.FindProperty("autoMaxHorizontalAcceleration").floatValue = 6f;
        serializedController.FindProperty("autoMaxVerticalSpeed").floatValue = 3f;
        serializedController.FindProperty("autoAltitudePositionGain").floatValue = 1.2f;

        serializedController.FindProperty("horizontalPositionKp").floatValue = 1.0f;
        serializedController.FindProperty("horizontalPositionKi").floatValue = 0.04f;
        serializedController.FindProperty("horizontalVelocityKd").floatValue = 1.8f;

        serializedController.FindProperty("verticalSpeedKp").floatValue = 3.8f;
        serializedController.FindProperty("verticalSpeedKi").floatValue = 1.0f;
        serializedController.FindProperty("verticalSpeedKd").floatValue = 0.75f;

        serializedController.FindProperty("attitudeKp").vector3Value = new Vector3(12f, 7f, 12f);
        serializedController.FindProperty("attitudeKi").vector3Value = new Vector3(0.6f, 0.35f, 0.6f);
        serializedController.FindProperty("attitudeKd").vector3Value = new Vector3(3.5f, 2.4f, 3.5f);
        serializedController.FindProperty("maxPitchTorque").floatValue = 12f;
        serializedController.FindProperty("maxRollTorque").floatValue = 12f;
        serializedController.FindProperty("maxYawTorque").floatValue = 7f;

        serializedController.ApplyModifiedPropertiesWithoutUndo();
        EditorUtility.SetDirty(controller);
    }

    private static void SetupMainCamera(Transform target)
    {
        Camera mainCamera = Camera.main;
        if (mainCamera == null)
        {
            GameObject cameraObject = new GameObject("Main Camera");
            Undo.RegisterCreatedObjectUndo(cameraObject, "Create Main Camera");
            mainCamera = Undo.AddComponent<Camera>(cameraObject);
            cameraObject.tag = "MainCamera";
        }

        DroneFollowCamera follow = mainCamera.GetComponent<DroneFollowCamera>();
        if (follow == null)
        {
            follow = Undo.AddComponent<DroneFollowCamera>(mainCamera.gameObject);
        }

        follow.SetTarget(target);
        mainCamera.transform.position = target.position + new Vector3(0f, 4f, -9f);
        mainCamera.transform.LookAt(target);
        EditorUtility.SetDirty(mainCamera);
    }

    private static GameObject CreatePrimitive(
        string name,
        PrimitiveType primitiveType,
        Transform parent,
        Vector3 localPosition,
        Quaternion localRotation,
        Vector3 localScale,
        Material material,
        bool removeCollider = false)
    {
        GameObject gameObject = GameObject.CreatePrimitive(primitiveType);
        Undo.RegisterCreatedObjectUndo(gameObject, $"Create {name}");
        gameObject.name = name;

        if (parent != null)
        {
            gameObject.transform.SetParent(parent, false);
            gameObject.transform.localPosition = localPosition;
            gameObject.transform.localRotation = localRotation;
            gameObject.transform.localScale = localScale;
        }
        else
        {
            gameObject.transform.position = localPosition;
            gameObject.transform.rotation = localRotation;
            gameObject.transform.localScale = localScale;
        }

        Renderer renderer = gameObject.GetComponent<Renderer>();
        if (renderer != null)
        {
            renderer.sharedMaterial = material;
        }

        if (removeCollider)
        {
            Collider collider = gameObject.GetComponent<Collider>();
            if (collider != null)
            {
                Undo.DestroyObjectImmediate(collider);
            }
        }

        return gameObject;
    }
}
#endif
