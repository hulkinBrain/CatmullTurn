using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FBPositioner : MonoBehaviour
{

    bool _isStarted;
    Pose[] _poses;
    float _interpFactor;
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
        UpdatePositionAnim();
    }

    public void StartPositionAnim(Pose[] poses)
    {
        _fishboneGroup = transform;
        _poses = poses;
        _fishboneGroup.position = poses[0].position;
        for (int i = 0; i < _fishboneGroup.childCount; i++)
        {
            var child = _fishboneGroup.GetChild(i);
            child.position = poses[i].position;
            child.localRotation = poses[i].rotation;
        }
        _poses = poses;
        //_isStarted = true;
        _interpFactor = 0f;
    }

    void UpdatePositionAnim()
    {
        if (_isStarted && _interpFactor < 1f)
        {
            _interpFactor = Mathf.Clamp01((_interpFactor + Time.deltaTime) / AnimDuration);
            var intraFbInterpFactor = _interpFactor % _fishboneGroup.childCount;
        }
    }
}
