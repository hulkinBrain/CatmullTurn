using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Linq;
using System.Reflection;
using UnityEngine;

public class CatmullRomSpline : MonoBehaviour
{
    public int split = 2;
    public float startRollAngle = 45f;
    public float endRollAngle = 90f;
    public FBPositioner fbGroup;
    public int resolution = 20;

    [SerializeField]
    private bool _includePitchAngle = false;
    [SerializeField]
    private bool _debugVis = false;

    void OnDrawGizmos()
    {
        if (transform.childCount < 3) return; // Need at least 4 points to form a spline

        Pose[] poses = GetCatmullRomSegmentPoints(GetControlPoints());
        fbGroup?.StartPositionAnim(poses);
        if (_debugVis)
        {
            for (int i = 0; i < poses.Length; i++)
            {
                Gizmos.DrawSphere(poses[i].position, (i % resolution == 0) ? 0.1f : 0.075f);
                if(i > 0)
                {
                    Gizmos.DrawLine(poses[i].position, poses[i - 1].position); // Draw line segment
                }
            }
        }
    }
    List<Vector3> GetControlPoints(Vector3 beforePosition, Vector3 onPosition, Vector3 afterPosition)
    {
        return new List<Vector3>() 
        {
            beforePosition,
            beforePosition,
            onPosition,
            afterPosition,
            afterPosition
        };
    }

    List<Vector3> GetControlPoints()
    {
        return GetControlPoints(transform.GetChild(0).position, transform.GetChild(1).position, transform.GetChild(2).position);
    }

    List<Pose> BeforeSectionPoints(List<Vector3> controlPoints, float minDistBetweenFishbones, float startRollAngle, float stepRollAngle, int split)
    {
        List<Pose> poses = new();
        float accumulatedDistance = 0;

        var beforeSectionControlPoints = controlPoints.GetRange(0, 4);
        poses.Add(new Pose(beforeSectionControlPoints[2], Quaternion.identity));

        Queue<Vector3> intraBeforePoints = new();
        for (int i = resolution; i >= 0; i--)
        {
            float t = i / (float)resolution; // Normalize t between 0 and 1
            Vector3 point = CatmullRom(beforeSectionControlPoints[0], beforeSectionControlPoints[1], beforeSectionControlPoints[2], beforeSectionControlPoints[3], t);
            intraBeforePoints.Enqueue(point);
            if (intraBeforePoints.Count > 1)
            {
                if (intraBeforePoints.Count > 2)
                {
                    intraBeforePoints.Dequeue();
                }
                var p1 = intraBeforePoints.ElementAt(1);
                var p2 = intraBeforePoints.ElementAt(0);
                var pCurrTopLastDistance = Vector3.Distance(p1, p2);
                accumulatedDistance += pCurrTopLastDistance;
                if (accumulatedDistance >= minDistBetweenFishbones)
                {
                    accumulatedDistance = 0;

                    var intraP2P1Vec = p2 - p1;
                    var angle = Quaternion.LookRotation(intraP2P1Vec).eulerAngles;
                    var pose = new Pose(p1, Quaternion.Euler(angle.x, angle.y, -(startRollAngle + stepRollAngle * (split - poses.Count - 1))));
                    poses.Add(pose);
                }
            }
            if (poses.Count == split)
            {
                break;
            }
        }
        poses.Reverse();
        poses = poses.GetRange(0, poses.Count - 1);
        return poses;
    }

    List<Pose> AfterSectionPoints(List<Vector3> controlPoints, float minDistBetweenFishbones, float startRollAngle, float stepRollAngle, int split)
    {
        List<Pose> poses = new();
        var afterSectionControlPoints = controlPoints.GetRange(1, 4);
        Queue<Vector3> intraAfterPoints = new();
        float accumulatedDistance = 0;

        poses.Add(new Pose(afterSectionControlPoints[1], Quaternion.identity));
        for (int i = 0; i <= resolution; i++)
        {
            float t = i / (float)resolution; // Normalize t between 0 and 1
            Vector3 point = CatmullRom(afterSectionControlPoints[0], afterSectionControlPoints[1], afterSectionControlPoints[2], afterSectionControlPoints[3], t);
            intraAfterPoints.Enqueue(point);
            if (intraAfterPoints.Count > 1)
            {
                if (intraAfterPoints.Count > 2)
                {
                    intraAfterPoints.Dequeue();
                }
                var p1 = intraAfterPoints.ElementAt(0);
                var p2 = intraAfterPoints.ElementAt(1);
                var pCurrTopLastDistance = Vector3.Distance(p1, p2);
                accumulatedDistance += pCurrTopLastDistance;
                if (accumulatedDistance >= minDistBetweenFishbones)
                {
                    accumulatedDistance = 0;

                    var intraP2P1Vec = p2 - p1;
                    var angle = Quaternion.LookRotation(intraP2P1Vec).eulerAngles;
                    var pose = new Pose(p1, Quaternion.Euler(angle.x, angle.y, -(startRollAngle + stepRollAngle * (split + poses.Count))));
                    poses.Add(pose);
                    if (poses.Count == 2)
                    {
                        angle = Quaternion.LookRotation(poses[^1].position - poses[^2].position).eulerAngles;
                        poses[^2] = new Pose(poses[^2].position, Quaternion.Euler(angle.x, angle.y, -(startRollAngle + stepRollAngle * (split - 1))));
                    }
                }
            }
            if (poses.Count == split)
            {
                break;
            }
        }
        return poses;
    }

    Pose[] GetCatmullRomSegmentPoints(List<Vector3> controlPoints)
    {
        //var resolution = 20;
        var minDistBetweenFishbones = fbGroup.transform.localScale.z * fbGroup.transform.GetChild(0).localScale.z * 2;

        // Vectors between control points
        var p1p2Vec = controlPoints[1] - controlPoints[2];  // Vector from 1st control point to 2nd control point
        var p2p3Vec = controlPoints[2] - controlPoints[3];  // Vector from 2nd control point to 3rd control point

        if (!_includePitchAngle)
        {
            p1p2Vec.y = p2p3Vec.y = 0;
        }

        // Clamps exit angle between -endRollAngle and endRollAngle to avoid extreme roll angles for fishbones
        float clampedExitAngle = Mathf.Clamp(Vector3.SignedAngle(p1p2Vec, p2p3Vec, Vector3.up), -endRollAngle, endRollAngle);
        
        // Interpolate "startRollAngle" for first fishbone from 0 to 45deg based on "clampedExitAngle"
        float interpolatedStartRollAngle = startRollAngle * (clampedExitAngle / endRollAngle);

        // Calculate the step increment in roll angle for consequent fishbones
        float stepAngle = (clampedExitAngle - interpolatedStartRollAngle) / (fbGroup.ChildCount - 1);

        var beforeSectionPoses = BeforeSectionPoints(controlPoints, minDistBetweenFishbones, interpolatedStartRollAngle, stepAngle, split);
        var afterSectionPoses = AfterSectionPoints(controlPoints, minDistBetweenFishbones, interpolatedStartRollAngle, stepAngle, split);

        List<Pose> fbPoses = new();
        fbPoses.AddRange(beforeSectionPoses);
        fbPoses.AddRange(afterSectionPoses);
        return fbPoses.ToArray();
    }

    Vector3 CatmullRom(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
    {
        // Catmull-Rom spline formula
        return 0.5f * ((2f * p1) +
                       (-p0 + p2) * t +
                       (2f * p0 - 5f * p1 + 4f * p2 - p3) * t * t +
                       (-p0 + 3f * p1 - 3f * p2 + p3) * t * t * t);
    }

    public void DoAnimate()
    {
        fbGroup?.StartPositionAnim(GetCatmullRomSegmentPoints(GetControlPoints()));
    }
}
