using System.Collections.Generic;
using UnityEngine;

public class CatmullRomSpline : MonoBehaviour
{
    public FBPositioner fbGroup;

    [SerializeField]
    private bool _includePitchAngle = false;
    [SerializeField]
    private bool _debugVis = false;
    [SerializeField]
    private bool _debugSplineVis = false;
    [SerializeField]
    private FishboneSplineHelper.SplineMethod _splineMethod = FishboneSplineHelper.SplineMethod.CubicSpline;

    void OnDrawGizmos()
    {
        if (transform.childCount < 3)
            return;

        var controlPoints = new Vector3[]
        {
            transform.GetChild(0).position,
            transform.GetChild(1).position,
            transform.GetChild(2).position
        };

        float minDist = fbGroup.transform.localScale.z * fbGroup.transform.GetChild(0).localScale.z * 2f;

        Pose[] worldPoses = FishboneSplineHelper.GetFanOutFishbonePointsInWorldSpace(
            controlPoints,
            minDist,
            fbGroup.ChildCount,
            _includePitchAngle,
            FishboneSplineHelper.FanOutSplineParameters.StartRollAngle,
            FishboneSplineHelper.FanOutSplineParameters.EndRollAngle,
            _splineMethod);

        Pose[] localPoses = FishboneSplineHelper.GetFanOutFishbonePointsInLocalSpace(worldPoses, fbGroup.transform);
        fbGroup.transform.position = controlPoints[2];
        if (fbGroup != null) fbGroup.StartPositionAnim(localPoses);

        if (_debugVis)
        {
            for (int i = 0; i < localPoses.Length; i++)
            {
                var worldPosition = fbGroup.transform.TransformPoint(localPoses[i].position);
                Gizmos.DrawSphere(worldPosition, 0.075f);
                if (i > 0)
                {
                    var previousWorldPosition = fbGroup.transform.TransformPoint(localPoses[i - 1].position);
                    Gizmos.DrawLine(worldPosition, previousWorldPosition);
                }
            }
        }
        if (_debugSplineVis)
        {
            DrawSplineGizmo(controlPoints);
        }
    }

    void DrawSplineGizmo(Vector3[] controlPoints)
    {
        int resolution = 64;
        float invRes = 1f / resolution;
        Vector3 p0 = controlPoints[0];
        Vector3 p1 = controlPoints[1];
        Vector3 p2 = controlPoints[2];
        Vector3[] augmented = FishboneSplineHelper.AugmentControlPoints(p0, p1, p2);

        Gizmos.color = Color.yellow;
        Gizmos.DrawSphere(p0, 0.1f);
        Gizmos.DrawSphere(p1, 0.1f);
        Gizmos.DrawSphere(p2, 0.1f);

        // CatmullRom
        Gizmos.color = Color.green;
        Vector3 prev = FishboneSplineHelper.CatmullRom(augmented[0], augmented[1], augmented[2], augmented[3], 0f);
        for (int i = 1; i <= resolution; i++)
        {
            float t = i * invRes;
            Vector3 cur = FishboneSplineHelper.CatmullRom(augmented[0], augmented[1], augmented[2], augmented[3], t);
            Gizmos.DrawLine(prev, cur);
            prev = cur;
        }
        prev = FishboneSplineHelper.CatmullRom(augmented[1], augmented[2], augmented[3], augmented[4], 0f);
        for (int i = 1; i <= resolution; i++)
        {
            float t = i * invRes;
            Vector3 cur = FishboneSplineHelper.CatmullRom(augmented[1], augmented[2], augmented[3], augmented[4], t);
            Gizmos.DrawLine(prev, cur);
            prev = cur;
        }

        // Quadratic Bezier
        Gizmos.color = Color.red;
        prev = FishboneSplineHelper.QuadBezier(p0, p1, p2, 0f);
        for (int i = 1; i <= resolution; i++)
        {
            float t = i * invRes;
            Vector3 cur = FishboneSplineHelper.QuadBezier(p0, p1, p2, t);
            Gizmos.DrawLine(prev, cur);
            prev = cur;
        }

        // Cubic Spline
        Gizmos.color = Color.cyan;
        prev = FishboneSplineHelper.CubicSpline(p0, p1, p2, 0f);
        for (int i = 1; i <= resolution; i++)
        {
            float t = i * invRes;
            Vector3 cur = FishboneSplineHelper.CubicSpline(p0, p1, p2, t);
            Gizmos.DrawLine(prev, cur);
            prev = cur;
        }

        // Circle Arc
        Gizmos.color = Color.magenta;
        prev = FishboneSplineHelper.CircleArc(p0, p1, p2, 0f);
        for (int i = 1; i <= resolution; i++)
        {
            float t = i * invRes;
            Vector3 cur = FishboneSplineHelper.CircleArc(p0, p1, p2, t);
            Gizmos.DrawLine(prev, cur);
            prev = cur;
        }
    }
}

public static class FishboneSplineHelper
{
    private static int _splineResolution = 256;
    private static int _pivotFishbone = 2;

    public static class FanOutSplineParameters
    {
        public const float MinDistBetweenFishbones = 1.5f;
        public const int TotalFishboneCount = 5;
        public const float StartRollAngle = 45.0f;
        public const float EndRollAngle = 90.0f;
    }

    public enum SplineMethod
    {
        CatmullRom,
        StraightLine,
        Bezier,
        CubicSpline,
        CircleArc
    }

    public static Vector3[] AugmentControlPoints(Vector3 pos1, Vector3 pos2, Vector3 pos3)
    {
        return new Vector3[] { pos1, pos1, pos2, pos3, pos3 };
    }

    public static Vector3 CatmullRom(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
    {
        return 0.5f * ((2f * p1) +
                       (-p0 + p2) * t +
                       t * t * (2f * p0 - 5f * p1 + 4f * p2 - p3) +
                       t * t * t * (-p0 + 3f * p1 - 3f * p2 + p3));
    }

    private static Vector3 CatmullRomFull(Vector3[] augmented, float t)
    {
        if (t <= 0.5f)
            return CatmullRom(augmented[0], augmented[1], augmented[2], augmented[3], t * 2f);
        else
            return CatmullRom(augmented[1], augmented[2], augmented[3], augmented[4], (t - 0.5f) * 2f);
    }

    public static Vector3 QuadBezier(Vector3 p0, Vector3 p1, Vector3 p2, float t)
    {
        float u = 1f - t;
        return u * u * p0 +
               2f * u * t * p1 +
               t * t * p2;
    }

    public static Vector3 CubicSpline(Vector3 p0, Vector3 p1, Vector3 p2, float t)
    {
        Vector3 M1 = 1.5f * (p0 - 2f * p1 + p2);

        if (t <= 0.5f)
        {
            float u = t * 2f;
            return p0 + (p1 - p0) * u + M1 * u * (u * u - 1f) / 6f;
        }
        else
        {
            float u = t * 2f - 1f;
            return p1 + (p2 - p1) * u + M1 * u * (1f - u) * (u - 2f) / 6f;
        }
    }

    public static Vector3 CircleArc(Vector3 p0, Vector3 p1, Vector3 p2, float t)
    {
        Vector3 a = p1 - p0;
        Vector3 b = p2 - p0;
        Vector3 axb = Vector3.Cross(a, b);
        float denom = 2f * axb.sqrMagnitude;

        if (denom < 1e-6f)
            return Vector3.Lerp(p0, p2, t);

        Vector3 center = p0 + (Vector3.Cross(axb, a) * b.sqrMagnitude + Vector3.Cross(b, axb) * a.sqrMagnitude) / denom;
        Vector3 normal = axb.normalized;
        Vector3 fromP0 = p0 - center;
        Vector3 fromP1 = p1 - center;
        Vector3 fromP2 = p2 - center;

        float totalAngle = Vector3.SignedAngle(fromP0, fromP2, normal);
        float midAngle = Vector3.SignedAngle(fromP0, fromP1, normal);

        bool needsAdjust;
        if (totalAngle > 0f)
            needsAdjust = midAngle < 0f || midAngle > totalAngle;
        else
            needsAdjust = midAngle > 0f || midAngle < totalAngle;

        if (needsAdjust)
            totalAngle += totalAngle > 0f ? -360f : 360f;

        Quaternion rot = Quaternion.AngleAxis(totalAngle * t, normal);
        return center + rot * fromP0;
    }

    private static Pose[] ArcLengthSectionPoints(System.Func<float, Vector3> evaluate, Vector3 pivotTarget, float minDist, float startRollAngle, float stepRollAngle, int split, int fbCount)
    {
        float invRes = 1f / _splineResolution;

        float[] arcLengths = new float[_splineResolution + 1];
        Vector3[] samples = new Vector3[_splineResolution + 1];
        samples[0] = evaluate(0f);
        arcLengths[0] = 0f;

        for (int i = 1; i <= _splineResolution; i++)
        {
            samples[i] = evaluate(i * invRes);
            arcLengths[i] = arcLengths[i - 1] + Vector3.Distance(samples[i - 1], samples[i]);
        }

        float totalLength = arcLengths[_splineResolution];
        int maxPoints = Mathf.FloorToInt(totalLength / minDist) + 1;

        List<Vector3> allPoints = new List<Vector3>(maxPoints);
        int pivotIndex = 0;
        float closestDistSq = float.MaxValue;

        for (int n = 0; n < maxPoints; n++)
        {
            float targetArc = n * minDist;
            if (targetArc > totalLength)
                break;

            int lo = 0, hi = _splineResolution;
            while (hi - lo > 1)
            {
                int mid = (lo + hi) / 2;
                if (arcLengths[mid] <= targetArc)
                    lo = mid;
                else
                    hi = mid;
            }

            float segLen = arcLengths[hi] - arcLengths[lo];
            float frac = segLen > 0f ? (targetArc - arcLengths[lo]) / segLen : 0f;
            Vector3 point = Vector3.Lerp(samples[lo], samples[hi], frac);
            allPoints.Add(point);

            float distSq = (point - pivotTarget).sqrMagnitude;
            if (distSq < closestDistSq)
            {
                closestDistSq = distSq;
                pivotIndex = allPoints.Count - 1;
            }
        }

        int beforePivot = split - 1;
        int startIndex = pivotIndex - beforePivot;
        int endIndex = startIndex + fbCount - 1;

        if (startIndex < 0)
            startIndex = 0;
        if (endIndex >= allPoints.Count)
        {
            endIndex = allPoints.Count - 1;
            startIndex = Mathf.Max(0, endIndex - fbCount + 1);
        }

        int actualCount = endIndex - startIndex + 1;

        Pose[] poses = new Pose[actualCount];
        for (int i = 0; i < actualCount; i++)
        {
            int idx = startIndex + i;
            Vector3 direction;
            if (idx < allPoints.Count - 1)
                direction = allPoints[idx + 1] - allPoints[idx];
            else
                direction = allPoints[idx] - allPoints[idx - 1];

            Vector3 euler = Quaternion.LookRotation(direction).eulerAngles;
            float roll = -(startRollAngle + stepRollAngle * i);
            poses[i] = new Pose(allPoints[idx], Quaternion.Euler(euler.x, euler.y, roll));
        }

        return poses;
    }

    public static Pose[] GetFanOutFishbonePointsInWorldSpace(Vector3[] controlPoints, float minDistBetweenPoints, int totalPointCount, bool includePitchAngle, float startRollAngle, float endRollAngle, SplineMethod splineMethod = SplineMethod.CatmullRom)
    {
        var p1p2Vec = controlPoints[1] - controlPoints[0];
        var p2p3Vec = controlPoints[2] - controlPoints[1];

        if (!includePitchAngle)
        {
            p1p2Vec.y = p2p3Vec.y = 0;
        }

        float clampedExitAngle = Mathf.Clamp(Vector3.SignedAngle(p1p2Vec, p2p3Vec, Vector3.up), -endRollAngle, endRollAngle);
        float interpolatedStartRollAngle = startRollAngle * (clampedExitAngle / endRollAngle);
        float stepAngle = (clampedExitAngle - interpolatedStartRollAngle) / (totalPointCount - 1);

        Pose[] result;
        switch (splineMethod)
        {
            case SplineMethod.StraightLine:
                result = new Pose[totalPointCount];
                Vector3 directionVec = controlPoints[2] - controlPoints[1];
                Vector3 step = directionVec.normalized * minDistBetweenPoints;
                Vector3 eulerAngles = Quaternion.LookRotation(directionVec).eulerAngles;

                float angle = Vector3.SignedAngle(controlPoints[2] - controlPoints[1], controlPoints[1] - controlPoints[0], Vector3.up);
                Quaternion rotation = Quaternion.Euler(eulerAngles.x, eulerAngles.y, Mathf.Clamp(angle, -endRollAngle, endRollAngle));
                for (int i = 0; i < totalPointCount; i++)
                {
                    result[i] = new Pose(controlPoints[1] + step * i, rotation);
                }
                break;

            case SplineMethod.Bezier:
            case SplineMethod.CubicSpline:
            case SplineMethod.CircleArc:
            case SplineMethod.CatmullRom:
            default:
                System.Func<float, Vector3> evaluate;
                if (splineMethod == SplineMethod.Bezier)
                    evaluate = t => QuadBezier(controlPoints[0], controlPoints[1], controlPoints[2], t);
                else if (splineMethod == SplineMethod.CubicSpline)
                    evaluate = t => CubicSpline(controlPoints[0], controlPoints[1], controlPoints[2], t);
                else if (splineMethod == SplineMethod.CircleArc)
                    evaluate = t => CircleArc(controlPoints[0], controlPoints[1], controlPoints[2], t);
                else
                {
                    var augmented = AugmentControlPoints(controlPoints[0], controlPoints[1], controlPoints[2]);
                    evaluate = t => CatmullRomFull(augmented, t);
                }

                result = ArcLengthSectionPoints(evaluate, controlPoints[1], minDistBetweenPoints, interpolatedStartRollAngle, stepAngle, _pivotFishbone, totalPointCount);
                break;
        }

        return result;
    }

    public static Pose[] GetFanOutFishbonePointsInLocalSpace(Pose[] posesInWorldSpace, Transform fbGroup)
    {
        Pose[] posesInLocalSpace = new Pose[posesInWorldSpace.Length];
        Quaternion invRotation = Quaternion.Inverse(fbGroup.rotation);
        for (int i = 0; i < posesInWorldSpace.Length; i++)
        {
            posesInLocalSpace[i] = new Pose(fbGroup.InverseTransformPoint(posesInWorldSpace[i].position), invRotation * posesInWorldSpace[i].rotation);
        }
        return posesInLocalSpace;
    }
}
