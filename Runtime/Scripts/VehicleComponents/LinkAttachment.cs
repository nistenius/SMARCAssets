using UnityEngine;
using Utils = DefaultNamespace.Utils;
using Force;

namespace VehicleComponents
{
    [AddComponentMenu("Smarc/VehicleComponents/LinkAttachment")]
    public class LinkAttachment : MonoBehaviour
    {
        [Header("Link attachment")] 
        [Tooltip("The name of the link the sensor should be attached to. Leave empty to not attach to anything.")]
        public string linkName = "";

        [Tooltip("If true, will try on FixedUpdates to attach, if false attach only on Awake")]
        public bool retryUntilSuccess = true;

        [Tooltip("ROS uses a different camera reference frame than Unity, this rotates the Unity camera to match that.")]
        public bool rotateForROSCamera = false;

        [Tooltip("Rotate the object with respect to the attached link after attaching.")]
        public float roll = 0f, pitch = 0f, yaw = 0f;

        [Tooltip("Should the orientation of the object be fixed, even if the link moves?")]
        public bool FixedRotation = false;
        Quaternion initialRotation;

        protected GameObject attachedLink;
        protected ArticulationBody parentArticulationBody;
        protected ArticulationBody articulationBody;
        protected MixedBody mixedBody;
        protected MixedBody parentMixedBody;

        // Diagnosis 2026-08-09 (silent core/sbg_imu + core/compass): a lookup failure in
        // Attach() during Awake used to SetActive(false) the whole GameObject, which made
        // retryUntilSuccess dead code (a disabled object never runs FixedUpdate) and killed
        // every sensor+publisher on the object while the Inspector still showed them
        // "enabled". Failure paths now honor retryUntilSuccess and log visibly.
        int attachAttempts = 0;

        protected void Awake()
        {
            Attach();
        }

        // Returns true if we should keep the object alive and retry next FixedUpdate.
        bool AttachFailed(string reason)
        {
            if (retryUntilSuccess)
            {
                // First failure, then roughly every 5 s at 50 Hz physics.
                if (attachAttempts % 250 == 0)
                    Debug.LogWarning($"[{transform.name}] Attach failed ({reason}), attempt {attachAttempts + 1}. Retrying every FixedUpdate.");
                attachAttempts++;
                return true;
            }
            Debug.LogWarning($"[{transform.name}] Attach failed ({reason}). Disabling {gameObject.name}.");
            gameObject.SetActive(false);
            return false;
        }

        protected void Attach()
        {
            if (linkName == "")
            {
                // dont attach to anything, but still try to get mixedbody
                attachedLink = gameObject;
                initialRotation = transform.rotation;
                GetMixedBody();
                return;
            }

            var theRobot = Utils.FindParentWithTag(gameObject, "robot", false);
            if (theRobot == null)
            {
                AttachFailed("no parent tagged [robot]");
                return;
            }

            Transform[] attachableTFs = Utils.FindAllChildrenWithName(theRobot, linkName, activeOnly: true);
            if (attachableTFs.Length == 0)
            {
                AttachFailed($"no active object named [{linkName}] under [{theRobot.name}]");
                return;
            }
            if (attachableTFs.Length > 1)
            {
                Debug.Log($"Multiple active objects with name [{linkName}] found under parent [{theRobot.name}]. Attaching to the first one.");
            }
            attachedLink = attachableTFs[0].gameObject;

            transform.SetPositionAndRotation(
                attachedLink.transform.position,
                attachedLink.transform.rotation
            );
            transform.Rotate(Vector3.up, yaw);
            transform.Rotate(Vector3.right, pitch);
            transform.Rotate(Vector3.forward, roll);

            if (rotateForROSCamera)
            {
                transform.Rotate(Vector3.up, 90);
                transform.Rotate(Vector3.right, -90);
                transform.Rotate(Vector3.forward, 180);
            }

            transform.SetParent(attachedLink.transform);
            initialRotation = transform.rotation;

            GetMixedBody();

            if (attachAttempts > 0)
                Debug.LogWarning($"[{transform.name}] Attached to [{linkName}] after {attachAttempts} failed attempts — link appeared late in the hierarchy.");
        }


        public MixedBody GetMixedBody()
        {
            if(mixedBody == null || parentMixedBody == null)
            {
                if (TryGetComponent(out ArticulationBody ab))
                    mixedBody = new MixedBody(ab, null);
                else if (TryGetComponent(out Rigidbody rb))
                    mixedBody = new MixedBody(null, rb);

                if (transform.parent.TryGetComponent(out ArticulationBody parentAB))
                    parentMixedBody = new MixedBody(parentAB, null);
                else if (transform.parent.TryGetComponent(out Rigidbody parentRB))
                    parentMixedBody = new MixedBody(null, parentRB);


                if (mixedBody == null || !mixedBody.isValid) mixedBody = parentMixedBody;
            }
            return mixedBody;
        }


        protected void FixedUpdate()
        {
            if (attachedLink == null && retryUntilSuccess) Attach();

            if (FixedRotation) transform.rotation = initialRotation;
        }

        void OnDrawGizmosSelected()
        {
            // Draw a semitransparent red cube at the transforms position
            Gizmos.color = new Color(1, 0, 0, 0.2f);
            Gizmos.DrawCube(transform.position, new Vector3(0.1f, 0.1f, 0.1f));
        }

        void OnDrawGizmos()
        {
            // Draw a semitransparent green cube at the transforms position
            Gizmos.color = new Color(0, 1, 0, 0.2f);
            Gizmos.DrawCube(transform.position, new Vector3(0.1f, 0.1f, 0.1f));
        }
    }
}