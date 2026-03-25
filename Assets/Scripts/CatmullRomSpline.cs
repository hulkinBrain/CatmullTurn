using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using UnityEngine;

public class CatmullRomSpline : MonoBehaviour
{
    public int split = 2;
    public float startRollAngle = 45f;
    public float endRollAngle = 90f;
    public FBPositioner positioner;
    public int resolution = 20;

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

    Pose[] GetCatmullRomSegmentPoints(List<Vector3> controlPoints)
    {
        List<Pose> fbPoses = new();
        List<Vector3> fbPositions = new();
        List<Pose> debugPoints = new();

        List<int> poseIndices = new();
        //var resolution = 20;
        var minDistBetweenFishbones = positioner.transform.localScale.z * positioner.transform.GetChild(0).localScale.z;

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
        float stepAngle = (clampedExitAngle - interpolatedStartRollAngle) / (resolution - 1);

        var points = controlPoints.GetRange(0, 4);
        int startIndex = resolution;
        int endIndex = 0;
        Queue<Vector3> intraPoints = new();
        float accumulatedDistance = 0f;

        #region Before section
        for(int i = startIndex; i >=endIndex; i--)
        {
            float t = i / (float)resolution; // Normalize t between 0 and 1
            Vector3 point = CatmullRom(points[0], points[1], points[2], points[3], t);
            debugPoints.Add(new Pose(point, Quaternion.identity));
            intraPoints.Enqueue(point);
            if(i == startIndex)
            {
                var pose = new Pose(point, Quaternion.identity);
                fbPoses.Add(pose);
                fbPositions.Add(point);
            }
            if (intraPoints.Count > 1)
            {
                if(intraPoints.Count > 2)
                {
                    intraPoints.Dequeue();
                }
                var p1 = intraPoints.ElementAt(1);
                var p2 = intraPoints.ElementAt(0);
                var pCurrTopLastDistance = Vector3.Distance(p1, p2);
                accumulatedDistance += pCurrTopLastDistance;
                if (accumulatedDistance >= minDistBetweenFishbones)
                {
                    accumulatedDistance = 0;

                    var intraP2P1Vec = p2 - p1;
                    var angle = Quaternion.LookRotation(intraP2P1Vec).eulerAngles;
                    var pose = new Pose(p1, Quaternion.Euler(angle.x, angle.y, -(interpolatedStartRollAngle + stepAngle * (split - fbPositions.Count))));
                    fbPoses.Add(pose);
                    fbPositions.Add(p1);
                }
            }
            if(fbPositions.Count == split)
            {
                break;
            }
        }
        intraPoints.Clear();
        #endregion

        #region After region
        #endregion

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
