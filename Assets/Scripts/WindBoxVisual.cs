using UnityEngine;

[ExecuteAlways]
public class WindBoxVisual : MonoBehaviour
{
    public PlayerController player;
    public WindBox windBox;
    public Transform visualCube;

    void Update()
    {
        if (!windBox || !visualCube) return;

        if (player != null && player.bladeSpinning)
        {
            visualCube.gameObject.SetActive(true);
        }
        else
        {
            visualCube.gameObject.SetActive(false);
            return;
        }

        visualCube.localPosition = Vector3.zero;
        visualCube.localRotation = Quaternion.identity;
        visualCube.localScale = windBox.size;
    }
}