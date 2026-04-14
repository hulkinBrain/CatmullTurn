using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FBPositioner : MonoBehaviour
{

    bool _isStarted;
    Pose[] _poses;
    Transform _fishboneGroup;
    public float AnimDuration = 1f;

    public void Awake()
    {
        _fishboneGroup = transform;
    }
    public int ChildCount
    {
        get { return transform.childCount; }
    }

    // Update is called once per frame
    void Update()
    {
    }

    public void StartPositionAnim(Pose[] poses)
    {
        _fishboneGroup = transform;
        _poses = poses;
        for (int i = 0; i < poses.Length; i++)
        {
            var child = _fishboneGroup.GetChild(i);
            child.localPosition = poses[i].position;
            child.localRotation = poses[i].rotation;
        }
        _poses = poses;
    }
}
