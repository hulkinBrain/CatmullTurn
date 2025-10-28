using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(CatmullRomSpline))]
public class EditorButton : Editor
{
    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();

        if(GUILayout.Button("Animate"))
        {
            CatmullRomSpline catmullRom = (CatmullRomSpline)target;
            catmullRom.DoAnimate();
        }
    }
}
