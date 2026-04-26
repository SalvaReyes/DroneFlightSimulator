using System;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

public static class StaticLightingBakeUtility
{
    private const string ScenePath = "Assets/Scenes/SampleScene.unity";

    private static readonly StaticEditorFlags StaticLightingFlags =
        StaticEditorFlags.ContributeGI |
        StaticEditorFlags.BatchingStatic |
        StaticEditorFlags.OccluderStatic |
        StaticEditorFlags.OccludeeStatic |
        StaticEditorFlags.ReflectionProbeStatic;

    [MenuItem("Drone Simulator/Lighting/Configure Static Baked Shadows")]
    public static void ConfigureSampleSceneStaticLighting()
    {
        Scene scene = OpenSampleScene();
        ConfigureStaticLightingInOpenScene();
        EditorSceneManager.MarkSceneDirty(scene);
        EditorSceneManager.SaveScene(scene);
        Debug.Log("Static baked lighting configured for SampleScene.");
    }

    [MenuItem("Drone Simulator/Lighting/Bake Static Shadows")]
    public static void BakeSampleSceneLighting()
    {
        Scene scene = OpenSampleScene();
        ConfigureStaticLightingInOpenScene();
        EditorSceneManager.MarkSceneDirty(scene);
        EditorSceneManager.SaveScene(scene);

        bool baked = Lightmapping.Bake();
        EditorSceneManager.SaveScene(scene);

        if (!baked)
        {
            Debug.LogError("Static lighting bake failed for SampleScene.");
            if (Application.isBatchMode)
            {
                EditorApplication.Exit(1);
            }

            return;
        }

        Debug.Log("Static lighting bake completed for SampleScene.");
        if (Application.isBatchMode)
        {
            EditorApplication.Exit(0);
        }
    }

    private static Scene OpenSampleScene()
    {
        Scene activeScene = SceneManager.GetActiveScene();
        if (activeScene.path == ScenePath)
        {
            return activeScene;
        }

        return EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
    }

    private static void ConfigureStaticLightingInOpenScene()
    {
        foreach (GameObject root in SceneManager.GetActiveScene().GetRootGameObjects())
        {
            ConfigureStaticFlagsRecursive(root.transform, IsStaticLightingRoot(root.name));
        }

        foreach (Light light in UnityEngine.Object.FindObjectsByType<Light>(FindObjectsInactive.Exclude))
        {
            if (light.type != LightType.Directional)
            {
                continue;
            }

            light.lightmapBakeType = LightmapBakeType.Baked;
            light.shadows = LightShadows.Soft;
        }
    }

    private static void ConfigureStaticFlagsRecursive(Transform current, bool staticLightingRoot)
    {
        bool isStaticObject = staticLightingRoot && !IsDynamicObject(current);
        GameObjectUtility.SetStaticEditorFlags(current.gameObject, isStaticObject ? StaticLightingFlags : (StaticEditorFlags)0);

        MeshRenderer meshRenderer = current.GetComponent<MeshRenderer>();
        if (meshRenderer != null)
        {
            meshRenderer.receiveGI = isStaticObject ? ReceiveGI.Lightmaps : ReceiveGI.LightProbes;
            meshRenderer.scaleInLightmap = isStaticObject ? GetScaleInLightmap(current.name) : 0f;
            meshRenderer.stitchLightmapSeams = isStaticObject;
        }

        foreach (Transform child in current)
        {
            ConfigureStaticFlagsRecursive(child, staticLightingRoot);
        }
    }

    private static bool IsStaticLightingRoot(string objectName)
    {
        return objectName.Equals("DroneTestingCourse", StringComparison.Ordinal) ||
               objectName.Equals("Gates", StringComparison.Ordinal) ||
               objectName.Equals("Waypoints", StringComparison.Ordinal) ||
               objectName.Equals("Obstacles", StringComparison.Ordinal) ||
               objectName.Equals("Ground", StringComparison.Ordinal);
    }

    private static bool IsDynamicObject(Transform transform)
    {
        while (transform != null)
        {
            string objectName = transform.name;
            if (objectName.Equals("Drone", StringComparison.Ordinal) ||
                objectName.Equals("Main Camera", StringComparison.Ordinal) ||
                objectName.Equals("Directional Light", StringComparison.Ordinal))
            {
                return true;
            }

            transform = transform.parent;
        }

        return false;
    }

    private static float GetScaleInLightmap(string objectName)
    {
        if (objectName.Equals("Ground", StringComparison.Ordinal))
        {
            return 0.45f;
        }

        if (objectName.StartsWith("Gate_", StringComparison.Ordinal) ||
            objectName.StartsWith("WaypointPole_", StringComparison.Ordinal))
        {
            return 0.25f;
        }

        return 0.2f;
    }
}
