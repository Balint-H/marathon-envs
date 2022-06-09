using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

public class ScreenshotHandler : TrainingEventHandler
{
    [SerializeField]
    private Camera camera;

    [SerializeField]
    string fileName;

    public override EventHandler Handler => (object sender, EventArgs e) => UnityEngine.ScreenCapture.CaptureScreenshot(Path.Combine(Application.persistentDataPath, fileName));



    public void Capture()
    {
        RenderTexture activeRenderTexture = RenderTexture.active;
        Debug.Log(camera);
        RenderTexture.active = camera.targetTexture;

        camera.Render();

        Texture2D image = new Texture2D(camera.targetTexture.width, camera.targetTexture.height);
        image.ReadPixels(new Rect(0, 0, camera.targetTexture.width, camera.targetTexture.height), 0, 0);
        image.Apply();
        RenderTexture.active = activeRenderTexture;

        byte[] bytes = image.EncodeToPNG();
        Destroy(image);

        Debug.Log(bytes);

        File.WriteAllBytes(Path.Combine(Application.persistentDataPath, fileName), bytes);
    }
}
