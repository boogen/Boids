using UnityEngine;

public class CameraFollow : MonoBehaviour
{
    public BoidController boidController;
    public Vector3 offset = new Vector3(0, 30, -40);
    public float smoothSpeed = 2f;

    void LateUpdate()
    {
        if (boidController == null) return;

        Vector3 center = (Vector3)boidController.FlockCenter;
        Vector3 targetPos = center + offset;

        transform.position = Vector3.Lerp(transform.position, targetPos, Time.deltaTime * smoothSpeed);
        transform.LookAt(center);
    }
}
