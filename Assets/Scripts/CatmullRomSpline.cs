using System.Collections.Generic;
using UnityEngine;

public class CatmullRomSpline : MonoBehaviour
{
    public int split = 2;
    public float startRollAngle = 45f;
    public float endRollAngle = 90f;
    public FBPositioner positioner;

    [SerializeField]
    private bool _includePitchAngle = false;
    [SerializeField]
    private bool _debugVis = false;

    void OnDrawGizmos()
    {
        if (transform.childCount < 3) return; // Need at least 4 points to form a spline

        if(_debugVis)
        {
            var poses = GetCatmullRomSegmentPoints(GetControlPoints());

            for (int i = 0; i < poses.Length; i++)
            {
                Gizmos.DrawSphere(poses[i].position, 0.1f);
                if(i > 0)
                {
                    Gizmos.DrawLine(poses[i].position, poses[i - 1].position); // Draw line segment
                }
            }
        }
    }
    List<Vector3> GetControlPoints(Vector3 beforePosition, Vector3 onPosition, Vector3 afterPosition)
    {
        List<Vector3> controlPoints = new()
        {
            beforePosition,
            beforePosition,
            onPosition,
            afterPosition,
            afterPosition
        };
        return controlPoints;
    }

    List<Vector3> GetControlPoints()
    {
        return GetControlPoints(transform.GetChild(0).position, transform.GetChild(1).position, transform.GetChild(2).position);
    }


    Pose[] GetCatmullRomSegmentPoints(List<Vector3> controlPoints)
    {
        List<Pose> fbPoses = new();
        List<Vector3> fbPositions = new();
        var resolution = positioner.ChildCount;
        var p1p2Vec = controlPoints[1] - controlPoints[2];
        var p2p3Vec = controlPoints[2] - controlPoints[3];
        if(!_includePitchAngle)
        {
            p1p2Vec.y = p2p3Vec.y = 0;
        }
        float clampedExitAngle = Mathf.Clamp(Vector3.SignedAngle(p1p2Vec, p2p3Vec, Vector3.up), -endRollAngle, endRollAngle);
        float interpolatedStartRollAngle = startRollAngle * (clampedExitAngle / endRollAngle);  // Interpolate "startRollAngle" for first fishbone from 0 to 45deg based on "clampedExitAngle"
        float stepAngle = (clampedExitAngle - interpolatedStartRollAngle) / (resolution - 1);   // Calculate the step increment in roll angle for consequent fishbones

        int sectionCount = 2;   // There are 2 sections. The before section and after section from which the halfpipe points will be taken

        for (int i = 0; i < sectionCount; i++)
        {
            var points = controlPoints.GetRange(i, 4);

            int startIndex;
            int endIndex;
            if(i == 0)
            {
                startIndex = resolution - split + 1;
                endIndex = resolution;
            }
            else
            {
                startIndex = 1;
                endIndex = resolution - split;
            }
            for (int j = startIndex; j <= endIndex; j++)
            {
                float t = j / (float)resolution; // Normalize t between 0 and 1
                Vector3 point = CatmullRom(points[0], points[1], points[2], points[3], t);
                fbPositions.Add(point);
                if(fbPositions.Count > 1)
                {
                    var p1 = fbPositions[^2];
                    var p2 = fbPositions[^1];
                    var intraP2P1Vec = p2 - p1;
                    if(!_includePitchAngle)
                    {
                        p1.y = p2.y = intraP2P1Vec.y = 0;
                    }
                    var angle = Quaternion.LookRotation(intraP2P1Vec).eulerAngles;
                    var pose = new Pose(p1, Quaternion.Euler(angle.x, angle.y, -(interpolatedStartRollAngle + (fbPositions.Count - 2) * stepAngle)));
                    fbPoses.Add(pose);

                    // For final fishbone
                    if(fbPositions.Count == resolution)
                    {
                        angle = Quaternion.LookRotation(-p2p3Vec).eulerAngles;
                        pose = new Pose(p2, Quaternion.Euler(angle.x, angle.y, -(clampedExitAngle)));
                        fbPoses.Add(pose);
                    }
                }
            }
        }
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
        positioner?.StartPositionAnim(GetCatmullRomSegmentPoints(GetControlPoints()));
    }
}
