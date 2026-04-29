using System.Collections.Generic;
using UnityEngine;

public class CatmullRomSpline : MonoBehaviour
{
    [Range(1, 5)]
    public int split = 2;
    public float startRollAngle = 45f;
    public float endRollAngle = 90f;
    public FBPositioner fbGroup;

    [SerializeField]
    private bool _includePitchAngle = false;
    [SerializeField]
    private bool _debugVis = false;
    [SerializeField]
    private bool _debugSplineVis = false;

    void OnDrawGizmos()
    {
        if (transform.childCount < 3)
        {
            return; // Need at least 4 points to form a spline
        }

        var controlPoints = new Vector3[]
        {
            transform.GetChild(0).position,
            transform.GetChild(1).position,
            transform.GetChild(2).position
        };
        Pose[] poses = GetCatmullRomSegmentPoints(controlPoints);
        fbGroup?.StartPositionAnim(poses);
        if (_debugVis)
        {
            for (int i = 0; i < poses.Length; i++)
            {
                var worldPosition = fbGroup.transform.TransformPoint(poses[i].position);
                Gizmos.DrawSphere(worldPosition, 0.075f);
                if(i > 0)
                {
                    var previousWorldPosition = fbGroup.transform.TransformPoint(poses[i - 1].position);
                    Gizmos.DrawLine(worldPosition, previousWorldPosition); // Draw line segment
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
        Vector3[] augmented = AugmentControlPoints(p0, p1, p2);

        // Draw control points
        Gizmos.color = Color.yellow;
        Gizmos.DrawSphere(p0, 0.1f);
        Gizmos.DrawSphere(p1, 0.1f);
        Gizmos.DrawSphere(p2, 0.1f);

        // CatmullRom — two segments (before: augmented[0..3], after: augmented[1..4])
        Gizmos.color = Color.green;
        Vector3 prev = CatmullRom(augmented[0], augmented[1], augmented[2], augmented[3], 0f);
        for (int i = 1; i <= resolution; i++)
        {
            float t = i * invRes;
            Vector3 cur = CatmullRom(augmented[0], augmented[1], augmented[2], augmented[3], t);
            Gizmos.DrawLine(prev, cur);
            prev = cur;
        }
        prev = CatmullRom(augmented[1], augmented[2], augmented[3], augmented[4], 0f);
        for (int i = 1; i <= resolution; i++)
        {
            float t = i * invRes;
            Vector3 cur = CatmullRom(augmented[1], augmented[2], augmented[3], augmented[4], t);
            Gizmos.DrawLine(prev, cur);
            prev = cur;
        }

        // Quadratic Bezier
        Gizmos.color = Color.red;
        prev = QuadBezier(p0, p1, p2, 0f);
        for (int i = 1; i <= resolution; i++)
        {
            float t = i * invRes;
            Vector3 cur = QuadBezier(p0, p1, p2, t);
            Gizmos.DrawLine(prev, cur);
            prev = cur;
        }

        // Cubic Spline
        Gizmos.color = Color.cyan;
        prev = CubicSpline(p0, p1, p2, 0f);
        for (int i = 1; i <= resolution; i++)
        {
            float t = i * invRes;
            Vector3 cur = CubicSpline(p0, p1, p2, t);
            Gizmos.DrawLine(prev, cur);
            prev = cur;
        }

        // Circle Arc
        Gizmos.color = Color.magenta;
        prev = CircleArc(p0, p1, p2, 0f);
        for (int i = 1; i <= resolution; i++)
        {
            float t = i * invRes;
            Vector3 cur = CircleArc(p0, p1, p2, t);
            Gizmos.DrawLine(prev, cur);
            prev = cur;
        }
    }
    Vector3[] AugmentControlPoints(Vector3 pos1, Vector3 pos2, Vector3 pos3)
    {
        // Duplicate the first and last points to create a 5-point control array for Catmull-Rom
        return new Vector3[] { pos1, pos1, pos2, pos3, pos3 };
    }

    Vector3 CatmullRomFull(Vector3[] augmented, float t)
    {
        if (t <= 0.5f)
        {
            return CatmullRom(augmented[0], augmented[1], augmented[2], augmented[3], t * 2f);
        }
        else
        {
            return CatmullRom(augmented[1], augmented[2], augmented[3], augmented[4], (t - 0.5f) * 2f);
        }
    }

    // Arc-length LUT: build LUT, binary search for equidistant points, pivot windowing
    Pose[] ArcLengthSectionPoints(System.Func<float, Vector3> evaluate, Vector3 pivotTarget, float minDist, float startRollAngle, float stepRollAngle, int split, int fbCount)
    {
        int splineResolution = 256;
        float invRes = 1f / splineResolution;

        // Pass 1: Build arc-length LUT
        float[] arcLengths = new float[splineResolution + 1];
        Vector3[] samples = new Vector3[splineResolution + 1];
        samples[0] = evaluate(0f);
        arcLengths[0] = 0f;

        for (int i = 1; i <= splineResolution; i++)
        {
            samples[i] = evaluate(i * invRes);
            arcLengths[i] = arcLengths[i - 1] + Vector3.Distance(samples[i - 1], samples[i]);
        }

        float totalLength = arcLengths[splineResolution];
        int maxPoints = Mathf.FloorToInt(totalLength / minDist) + 1;

        // Pass 2: Place equidistant points via binary search on LUT
        List<Vector3> allPoints = new List<Vector3>(maxPoints);
        int pivotIndex = 0;
        float closestDistSq = float.MaxValue;

        for (int n = 0; n < maxPoints; n++)
        {
            float targetArc = n * minDist;
            if (targetArc > totalLength)
            {
                break;
            }

            int lo = 0, hi = splineResolution;
            while (hi - lo > 1)
            {
                int mid = (lo + hi) / 2;
                if (arcLengths[mid] <= targetArc)
                {
                    lo = mid;
                }
                else
                {
                    hi = mid;
                }
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

        // Select centered window around pivot
        int beforePivot = split - 1;
        int startIndex = pivotIndex - beforePivot;
        int endIndex = startIndex + fbCount - 1;

        if (startIndex < 0)
        {
            startIndex = 0;
        }
        if (endIndex >= allPoints.Count)
        {
            endIndex = allPoints.Count - 1;
            startIndex = Mathf.Max(0, endIndex - fbCount + 1);
        }

        int actualCount = endIndex - startIndex + 1;

        // Build poses with roll angles
        Pose[] poses = new Pose[actualCount];
        for (int i = 0; i < actualCount; i++)
        {
            int idx = startIndex + i;
            Vector3 direction;
            if (idx < allPoints.Count - 1)
            {
                direction = allPoints[idx + 1] - allPoints[idx];
            }
            else
            {
                direction = allPoints[idx] - allPoints[idx - 1];
            }

            Vector3 euler = Quaternion.LookRotation(direction).eulerAngles;
            float roll = -(startRollAngle + stepRollAngle * i);
            poses[i] = new Pose(allPoints[idx], Quaternion.Euler(euler.x, euler.y, roll));
        }

        return poses;
    }

    // Spline method to use for situating single fishbones during fanout and for harpoon
    public enum SplineMethod
    {
        CatmullRom,
        StraightLine,
        Bezier,
        CubicSpline,
        CircleArc
    }

    Pose[] GetCatmullRomSegmentPoints(Vector3[] controlPoints)
    {
        var minDistBetweenPoints = fbGroup.transform.localScale.z * fbGroup.transform.GetChild(0).localScale.z * 2f;

        // Vectors between control points
        var p1p2Vec = controlPoints[1] - controlPoints[0];  // Vector from 1st control point to 2nd control point
        var p2p3Vec = controlPoints[2] - controlPoints[1];  // Vector from 2nd control point to 3rd control point

        if (!_includePitchAngle)
        {
            p1p2Vec.y = p2p3Vec.y = 0;
        }

        float clampedExitAngle = Mathf.Clamp(Vector3.SignedAngle(p1p2Vec, p2p3Vec, Vector3.up), -endRollAngle, endRollAngle);
        float interpolatedStartRollAngle = startRollAngle * (clampedExitAngle / endRollAngle);
        float stepAngle = (clampedExitAngle - interpolatedStartRollAngle) / (fbGroup.ChildCount - 1);

        Pose[] result;
        int totalPointCount = fbGroup.ChildCount;
        var splineMethod = SplineMethod.CubicSpline; // You can make this configurable if needed
        switch(splineMethod)
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
                {
                    evaluate = t => QuadBezier(controlPoints[0], controlPoints[1], controlPoints[2], t);
                }
                else if (splineMethod == SplineMethod.CubicSpline)
                {
                    evaluate = t => CubicSpline(controlPoints[0], controlPoints[1], controlPoints[2], t);
                }
                else if (splineMethod == SplineMethod.CircleArc)
                {
                    evaluate = t => CircleArc(controlPoints[0], controlPoints[1], controlPoints[2], t);
                }
                else
                {
                    var augmentedControlPoints = AugmentControlPoints(controlPoints[0], controlPoints[1], controlPoints[2]);
                    evaluate = t => CatmullRomFull(augmentedControlPoints, t);
                }

                Pose[] splinePoses = ArcLengthSectionPoints(evaluate, controlPoints[1], minDistBetweenPoints, interpolatedStartRollAngle, stepAngle, split, totalPointCount);
                int splineCount = splinePoses?.Length ?? 0;

                Quaternion invRot = Quaternion.Inverse(fbGroup.transform.rotation);
                result = new Pose[splineCount];
                for (int i = 0; i < splineCount; i++)
                {
                    result[i] = new Pose(fbGroup.transform.InverseTransformPoint(splinePoses[i].position), invRot * splinePoses[i].rotation);
                }
                fbGroup.transform.position = controlPoints[2];
                break;

        }

        return result;
    }

    Vector3 CatmullRom(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
    {
        // Catmull-Rom spline formula
        return 0.5f * ((2f * p1) +
                       (-p0 + p2) * t +
                       t * t * (2f * p0 - 5f * p1 + 4f * p2 - p3) +
                       t * t * t * (-p0 + 3f * p1 - 3f * p2 + p3));
    }

    Vector3 QuadBezier(Vector3 p0, Vector3 p1, Vector3 p2, float t)
    {
        // Quadratic Bezier curve formula
        float u = 1f - t;
        return u * u * p0 +
               2f * u * t * p1 +
               t * t * p2;
    }

    Vector3 CubicSpline(Vector3 p0, Vector3 p1, Vector3 p2, float t)
    {
        // Natural cubic spline through 3 points
        // Passes through p0 at t=0, p1 at t=0.5, p2 at t=1
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

    Vector3 CircleArc(Vector3 p0, Vector3 p1, Vector3 p2, float t)
    {
        // Arc of the circumcircle passing through all 3 points
        Vector3 a = p1 - p0;
        Vector3 b = p2 - p0;
        Vector3 axb = Vector3.Cross(a, b);
        float denom = 2f * axb.sqrMagnitude;

        // Collinear fallback
        if (denom < 1e-6f)
        {
            return Vector3.Lerp(p0, p2, t);
        }

        // Circumcenter
        Vector3 center = p0 + (Vector3.Cross(axb, a) * b.sqrMagnitude + Vector3.Cross(b, axb) * a.sqrMagnitude) / denom;
        Vector3 normal = axb.normalized;
        Vector3 fromP0 = p0 - center;
        Vector3 fromP1 = p1 - center;
        Vector3 fromP2 = p2 - center;

        // Signed angles from p0 to p1 and p2 around normal
        float totalAngle = Vector3.SignedAngle(fromP0, fromP2, normal);
        float midAngle = Vector3.SignedAngle(fromP0, fromP1, normal);

        // Ensure arc goes p0 -> p1 -> p2
        bool needsAdjust;
        if (totalAngle > 0f)
        {
            needsAdjust = midAngle < 0f || midAngle > totalAngle;
        }
        else
        {
            needsAdjust = midAngle > 0f || midAngle < totalAngle;
        }
        if (needsAdjust)
        {
            totalAngle += totalAngle > 0f ? -360f : 360f;
        }

        Quaternion rot = Quaternion.AngleAxis(totalAngle * t, normal);
        return center + rot * fromP0;
    }
}
