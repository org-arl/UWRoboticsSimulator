// Controls the caustic projection on object surfaces to create a lightening shift effect due to water.
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Wateranim : MonoBehaviour
{

	public float fps = 30.0f; // FPS of the projection
	public Texture2D[] frames;

	private int frameIndex;
	private Projector projector;
    // It takes in 15 images as input and play them on loop to hget a continuous effect. 
    void Start()
	{
		projector = GetComponent<Projector>();
		NextFrame();
		InvokeRepeating("NextFrame", 1 / fps, 1 / fps);
	}

	void NextFrame()
	{
		projector.material.SetTexture("_ShadowTex", frames[frameIndex]);
		frameIndex = (frameIndex + 1) % frames.Length;
	}

}
