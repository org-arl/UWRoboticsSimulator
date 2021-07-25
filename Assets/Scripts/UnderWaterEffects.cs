//Controls the parameters for underwater distortion effects. Called by 'StartUnderwater' to activate the effects on cameras.
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode, ImageEffectAllowedInSceneView]
public class UnderWaterEffects : MonoBehaviour
{
    public Material _mat;
    // Parameters to control the Underwater effect
    [Range(0.001f, 0.1f)]
    public float _pixelOffset;
    [Range(0.1f, 20f)]
    public float _noiseScale;
    [Range(0.1f, 20f)]
    public float _noiseFrequency;
    [Range(0.1f, 30f)]
    public float _noiseSpeed;

    public float _depthStart;
    public float _depthDistance;


    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        _mat.SetFloat("_NoiseFrequency", _noiseFrequency); // Setting the values from the inspector panel. 
        _mat.SetFloat("_NoiseSpeed", _noiseSpeed);
        _mat.SetFloat("_NoiseScale", _noiseScale);
        _mat.SetFloat("_PixelOffset", _pixelOffset);

        _mat.SetFloat("_DepthStart", _depthStart);
        _mat.SetFloat("_DepthDistance", _depthDistance);
    }
    private void OnRenderImage(RenderTexture source, RenderTexture destination)
    {
        Graphics.Blit(source, destination, _mat);
    }
}
