using System.Collections.Generic;
using UnityEngine;
using Force;

namespace DataCube.Station
{
    /// <summary>
    /// Makes the Data Cube field station a DYNAMIC object: on Play it falls the last few
    /// centimetres onto the deck it is posed over, settles, welds itself to the vessel, and from
    /// then on rides wherever the vessel goes.
    ///
    /// ---------------------------------------------------------------------------------------
    /// THE THING THAT IS NOT OBVIOUS, AND THAT DECIDED THIS DESIGN: MILOU HAS NO DECK.
    ///
    /// Milou.prefab carries exactly three solid colliders — Hull, Cabin, Engine, all boxes — plus
    /// twenty-two ForcePoint spheres that are all triggers. The Hull box spans local y -0.017 to
    /// +0.906; it is a crude envelope for buoyancy and quay contact, not a surface anybody stands
    /// on. In KristinebergEmpty the station is posed at local y +0.376, which is where the deck
    /// LOOKS like it is. Those two numbers disagree by 0.53 m.
    ///
    /// So a naive "add a Rigidbody and let it fall" does the wrong thing twice over: the case
    /// starts life 0.53 m INSIDE the Hull box, which PhysX resolves by firing it across the
    /// water, and if it survived that it would come to rest half a metre above the visible deck,
    /// standing on nothing. Dropping onto existing geometry was never an option — the geometry
    /// is not there.
    ///
    /// What this does instead: it creates a thin DECK PAD collider, at runtime, at the height the
    /// case was posed at, parented to the vessel body so it rides along; drops the case onto that;
    /// and tells PhysX to ignore every other collider on the vessel. The deck plane is therefore
    /// DEFINED BY WHERE YOU PUT THE CASE. That is a real measurement (someone placed it by eye
    /// against the visual hull) rather than a constant invented in a script — this project has
    /// paid for the other kind: the station's "MEASURED +/-0.014 m" position on 2026-08-18 was a
    /// hardcoded constant 49.4 m from the truth. Move the case in the editor and the pad moves
    /// with it. Set useExplicitDeckSurface if you would rather state the height outright.
    /// ---------------------------------------------------------------------------------------
    ///
    /// WHY THIS IS A RUNTIME COMPONENT AND NOT A PREFAB EDIT. StationBuilder.BuildCaseVisual
    /// states the rule it was built under: "NO COLLIDERS ON ANY OF IT. The station sits on a boat
    /// deck and must never become something the vehicle's sonar or obstacle stack reacts to."
    /// A collider baked into datacube_station_01.prefab would exist in every scene forever,
    /// whether or not anyone wanted physics. Here the collider, the pad and the body are all
    /// built in Awake and exist only while the game is playing. Remove this component and the
    /// station is exactly the inert prop it was.
    ///
    /// AND THE RULE SURVIVES, WHICH IS CHECKABLE ARITHMETIC. Sonar.cs casts with layerMask ~0 and
    /// ForwardBottomProfiler sets _mask = ~0, so no layer assignment could hide a new collider
    /// from a sensor. What saves it is geometry: the case is 0.185 m tall standing at local
    /// y 0.376, i.e. y 0.376 to 0.561, and the pad is 0.02 m thick just below it — all of it
    /// strictly inside the Hull box's 0.906 m ceiling and 2.30 x 5.90 m footprint. Every new
    /// collider is ENCLOSED by a collider the sensors already saw, so no ray can reach one
    /// without having hit the hull first. If a sonar image ever changes because of this
    /// component, that claim is what was wrong — re-check the numbers above before suspecting
    /// the sensor.
    ///
    /// WHY THE STATION MUST STAY A SCENE ROOT. Milou's base_link is an ArticulationBody (mass
    /// 400), not a Rigidbody, and Unity does not support a Rigidbody inside an ArticulationBody
    /// hierarchy. Parenting the case to the boat to make it "follow" is the obvious move and it
    /// is the wrong one. The case stays a root object and is held by a FixedJoint; Awake refuses
    /// outright if it finds itself parented into an articulation chain.
    ///
    /// CONSEQUENCE YOU ASKED FOR EXPLICITLY (Ivan, 2026-08-19). The station's GPS reports its own
    /// transform, so once the case rides the boat the ADR-007 "measured launch point" MOVES. That
    /// is the honest model — a station on a deck really does move — but anything downstream that
    /// assumed a static launch reference is now wrong and should be found, not worked around.
    /// The GPS stays at 1 Hz, which is still adequate at boat speeds.
    /// </summary>
    [DisallowMultipleComponent]
    public class StationDeckPhysics : MonoBehaviour
    {
        [Header("Vessel")]
        [Tooltip("The vessel body to land on and weld to — drag Milou's base_link here. Leave " +
                 "empty and Awake casts straight down and names whatever body it finds, which is " +
                 "convenient but is a guess; assigning it is a statement.")]
        public ArticulationBody deckArticulationBody;

        [Tooltip("Same role, for a vessel built on a Rigidbody instead. Set at most one of the two.")]
        public Rigidbody deckRigidbody;

        [Tooltip("How far down Awake looks for a vessel body when neither field above is assigned.")]
        public float deckSearchDistanceM = 5f;

        [Header("Deck pad — the surface that does not otherwise exist")]
        [Tooltip("Off: the deck plane is the bottom of the case exactly where you posed it. " +
                 "On: use deckSurfaceLocalY below instead, in the vessel body's local frame.")]
        public bool useExplicitDeckSurface = false;

        [Tooltip("Deck height in the vessel body's local frame, used only when the box above is " +
                 "ticked. For reference, Milou's Hull box top is at local y 0.906 and the case is " +
                 "posed at 0.376 — the deck is the second number, not the first.")]
        public float deckSurfaceLocalY = 0.376f;

        [Tooltip("Footprint of the pad, metres. Big enough that the case cannot slide off it " +
                 "during the drop, small enough to stay well inside the Hull box (which is " +
                 "2.30 x 5.90 m) so it adds nothing a sensor can see.")]
        public Vector2 deckPadSizeM = new Vector2(1.2f, 1.2f);

        [Tooltip("Pad thickness, metres. Thin, but not so thin that a fast contact tunnels it.")]
        public float deckPadThicknessM = 0.02f;

        [Header("Case")]
        [Tooltip("Exterior of the Peli iM2400, metres, matching StationBuilder's CaseLen/Hgt/Wid " +
                 "(0.488 along local X, 0.185 up, 0.386 along local Z). Rim, handle and latches " +
                 "stand a few mm proud of this and are deliberately not modelled: one box is one " +
                 "contact manifold, and a case does not rest on its latches.")]
        public Vector3 caseSizeM = new Vector3(0.488f, 0.185f, 0.386f);

        [Tooltip("Kilograms. NOT A MEASUREMENT — no physical unit exists yet, which is why " +
                 "fleet.yaml tags the station's RTK and Succorfish modem user_provided. This is a " +
                 "loaded-case guess: case, GPS-RTK2, modem, battery. Weigh the real one, replace " +
                 "this, and say where the number came from when you do.")]
        public float massKg = 8f;

        [Tooltip("Friction of the case against the pad. Moulded polypropylene on a wet deck is " +
                 "slippery; lower this if you want it to slide before the weld takes.")]
        public float frictionCoefficient = 0.6f;

        [Header("Drop")]
        [Tooltip("Metres between the bottom of the case and the deck plane at the first physics " +
                 "step. This is the whole visible fall, so keep it small — the case is posed ON " +
                 "the deck and a large value just makes it hover before dropping.")]
        public float startGapM = 0.10f;

        [Header("Weld")]
        [Tooltip("Weld the case to the vessel once it stops moving. Untick for friction only — " +
                 "more realistic, and a bobbing deck will walk the case over the side eventually.")]
        public bool weldWhenSettled = true;

        [Tooltip("Earliest weld, seconds after Play. Below this the case is free to fall and bounce.")]
        public float minSettleSeconds = 1.5f;

        [Tooltip("Latest weld, seconds after Play — reached only if the case never comes to rest. " +
                 "The weld then happens anyway and says so, rather than silently never welding.")]
        public float maxSettleSeconds = 10f;

        [Tooltip("Speeds below which the case counts as at rest, m/s and rad/s.")]
        public float restLinearSpeed = 0.05f;
        public float restAngularSpeed = 0.10f;

        [Header("State — read only, filled in at runtime")]
        public string deckBodyName = "(none)";
        public float deckSurfaceWorldY;
        public bool landed;
        public bool welded;

        Rigidbody _rb;
        BoxCollider _box;
        BoxCollider _pad;
        PhysicsMaterial _phys;
        float _t;
        bool _inert;   // a refusal happened in Awake; do nothing for the rest of the session

        void Awake()
        {
            if (!GuardHierarchy()) return;
            if (!BuildCaseCollider()) return;
            if (!ResolveVesselBody()) return;

            BuildDeckPad();
            IgnoreEverythingButThePad();
            PlaceAboveDeckPlane();
            BuildBody();
        }

        // ---------------------------------------------------------------- guards

        bool GuardHierarchy()
        {
            var parentAB = transform.parent != null
                         ? transform.parent.GetComponentInParent<ArticulationBody>()
                         : null;
            if (parentAB != null)
            {
                Refuse($"'{name}' is parented under the ArticulationBody '{parentAB.name}'. Unity " +
                       "does not support a Rigidbody inside an ArticulationBody hierarchy. Drag " +
                       "the station out to the scene root — it does not need to be a child of the " +
                       "boat to ride on it; that is what the weld is for.");
                return false;
            }

            if (deckArticulationBody != null && deckRigidbody != null)
            {
                Refuse("Both Deck Articulation Body and Deck Rigidbody are assigned. Two answers " +
                       "to 'what is the vessel' is how a scene ends up welded to the wrong thing. " +
                       "Clear one.");
                return false;
            }
            return true;
        }

        // ---------------------------------------------------------------- case collider

        /// <summary>One box, on a child of base_link named so it is obvious in the runtime
        /// Hierarchy that it was not in the prefab. Deliberately NOT inside 'Visual':
        /// StationBuilder.RestyleCase destroys and rebuilds that whole subtree, and a collider
        /// living there would vanish the next time the case is restyled.</summary>
        bool BuildCaseCollider()
        {
            var baseLink = transform.Find("base_link");
            if (baseLink == null)
            {
                Refuse("No child called 'base_link'. Refusing to guess where the case is — this " +
                       "component expects the datacube_station_01 prefab layout.");
                return false;
            }

            var go = new GameObject("Physics (runtime, StationDeckPhysics)");
            go.transform.SetParent(baseLink, false);

            _box = go.AddComponent<BoxCollider>();
            _box.size = caseSizeM;
            // base_link's origin is the BOTTOM of the case: StationBuilder puts Base at
            // baseH*0.5 and Lid on top of it, spanning 0 to CaseHgt. So the box centre is half
            // the height up and the contact point is the origin — which is what keeps the gap
            // arithmetic below checkable by eye.
            _box.center = new Vector3(0f, caseSizeM.y * 0.5f, 0f);

            // Built in code rather than loaded from Runtime/Materials/Physic: this material only
            // ever exists during Play, so an asset (plus its .meta, plus its guid in git) would be
            // three files of ceremony for one number.
            // sharedMaterial, not material: the setter for `material` clones per collider, and
            // one friction number described in two places is one place too many.
            _phys = new PhysicsMaterial("StationCase (runtime)")
            {
                staticFriction  = frictionCoefficient,
                dynamicFriction = frictionCoefficient,
                bounciness      = 0.05f
            };
            _box.sharedMaterial = _phys;
            return true;
        }

        // ---------------------------------------------------------------- vessel

        bool ResolveVesselBody()
        {
            if (deckArticulationBody != null) { deckBodyName = deckArticulationBody.name + " (assigned)"; return true; }
            if (deckRigidbody != null)        { deckBodyName = deckRigidbody.name + " (assigned)";        return true; }

            foreach (var h in CastDown())
            {
                var ab = h.collider.GetComponentInParent<ArticulationBody>();
                if (ab != null)
                {
                    deckArticulationBody = ab;
                    deckBodyName = $"{ab.name} (FOUND by downcast, via collider '{h.collider.name}' " +
                                   "— nobody assigned it)";
                    Debug.Log($"[StationDeckPhysics] Vessel resolved to {deckBodyName}. Note the " +
                              "collider named there is what the ray HIT, not the deck: on Milou " +
                              "that is the Hull envelope, whose top is half a metre above the " +
                              "visible deck. The pad below is what the case actually lands on.");
                    return true;
                }

                var rb = h.collider.GetComponentInParent<Rigidbody>();
                if (rb != null)
                {
                    deckRigidbody = rb;
                    deckBodyName = $"{rb.name} (FOUND by downcast, via collider '{h.collider.name}')";
                    Debug.Log($"[StationDeckPhysics] Vessel resolved to {deckBodyName}.");
                    return true;
                }
            }

            // Staying inert rather than falling: with no vessel there is nothing to weld to and
            // nothing to stand on, and a station that has silently dropped out of the world is
            // worse than one that never moved.
            Refuse($"No vessel body found within {deckSearchDistanceM} m below the station, and " +
                   "none assigned. The station stays inert. Drag the vessel's base_link into the " +
                   "Deck Articulation Body field.");
            return false;
        }

        RaycastHit[] CastDown()
        {
            // Start above the case so the ray cannot begin inside a collider, and drop triggers:
            // Milou's twenty-two ForcePoint spheres are all triggers and would otherwise be the
            // first thing every cast found.
            var origin = transform.position + Vector3.up * (caseSizeM.y + 0.5f);
            var hits = Physics.RaycastAll(origin, Vector3.down,
                                          deckSearchDistanceM + caseSizeM.y + 0.5f,
                                          ~0, QueryTriggerInteraction.Ignore);
            System.Array.Sort(hits, (a, b) => a.distance.CompareTo(b.distance));

            var keep = new List<RaycastHit>(hits.Length);
            foreach (var h in hits)
                if (!h.collider.transform.IsChildOf(transform)) keep.Add(h);
            return keep.ToArray();
        }

        Transform VesselTransform =>
            deckArticulationBody != null ? deckArticulationBody.transform : deckRigidbody.transform;

        // ---------------------------------------------------------------- deck pad

        void BuildDeckPad()
        {
            var vessel = VesselTransform;

            deckSurfaceWorldY = useExplicitDeckSurface
                ? vessel.TransformPoint(new Vector3(0f, deckSurfaceLocalY, 0f)).y
                : transform.position.y;   // the bottom of the case, exactly as posed

            var go = new GameObject("DeckPad (runtime, StationDeckPhysics)");
            go.transform.SetParent(vessel, true);
            // Centred under the case, top face on the deck plane. Parented WITH world position
            // kept, so it rides the vessel from here on without any per-frame work.
            go.transform.position = new Vector3(transform.position.x,
                                                deckSurfaceWorldY - deckPadThicknessM * 0.5f,
                                                transform.position.z);
            go.transform.rotation = vessel.rotation;

            _pad = go.AddComponent<BoxCollider>();
            _pad.size = new Vector3(deckPadSizeM.x, deckPadThicknessM, deckPadSizeM.y);
            _pad.sharedMaterial = _phys;

            var local = vessel.InverseTransformPoint(go.transform.position);
            Debug.Log($"[StationDeckPhysics] Deck pad {deckPadSizeM.x:F2} x {deckPadThicknessM:F3} x " +
                      $"{deckPadSizeM.y:F2} m created on '{vessel.name}' at local " +
                      $"({local.x:F3}, {local.y:F3}, {local.z:F3}); deck plane world y = " +
                      $"{deckSurfaceWorldY:F3}, taken from " +
                      (useExplicitDeckSurface ? "deckSurfaceLocalY" : "the posed bottom of the case") +
                      ". Milou has no deck collider of its own — see this class's summary.");
        }

        /// <summary>The case is posed INSIDE the Hull envelope, so without this every fixed step
        /// would be PhysX trying to expel an 8 kg box from a 400 kg one. Pairwise ignores rather
        /// than layers, because Sonar and ForwardBottomProfiler cast with mask ~0 and a layer
        /// carved out for collision would change what the sensors see.</summary>
        void IgnoreEverythingButThePad()
        {
            int n = 0;
            foreach (var c in VesselTransform.GetComponentsInChildren<Collider>(true))
            {
                if (c == _pad || c.isTrigger) continue;
                Physics.IgnoreCollision(_box, c, true);
                n++;
            }
            Debug.Log($"[StationDeckPhysics] Case collides with the pad only — {n} other vessel " +
                      "collider(s) ignored (Hull, Cabin, Engine on Milou). They are still there " +
                      "for every other body and for every sensor; only this one pair is muted.");
        }

        void PlaceAboveDeckPlane()
        {
            float wanted = deckSurfaceWorldY + startGapM;
            float delta = wanted - transform.position.y;
            transform.position = new Vector3(transform.position.x, wanted, transform.position.z);
            Debug.Log($"[StationDeckPhysics] Case lifted {delta:+0.000;-0.000} m to y={wanted:F3} " +
                      $"for a {startGapM:F3} m drop onto the pad.");
        }

        // ---------------------------------------------------------------- body

        void BuildBody()
        {
            _rb = gameObject.AddComponent<Rigidbody>();
            _rb.mass = massKg;
            _rb.useGravity = true;
            // Continuous: a 0.185 m box falling onto a 0.02 m pad that is itself rising on a wave
            // is exactly the contact discrete collision misses.
            _rb.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
            _rb.interpolation = RigidbodyInterpolation.Interpolate;
            _rb.linearDamping = 0.05f;
            _rb.angularDamping = 0.5f;

            Debug.Log($"[StationDeckPhysics] {name} is dynamic: {massKg} kg, box " +
                      $"{caseSizeM.x:F3} x {caseSizeM.y:F3} x {caseSizeM.z:F3} m, mu={frictionCoefficient}, " +
                      $"vessel = {deckBodyName}. " +
                      (weldWhenSettled
                          ? $"Weld once at rest, between {minSettleSeconds:F1} s and {maxSettleSeconds:F1} s."
                          : "Weld DISABLED — friction only, so expect it to walk in a seaway."));
        }

        // ---------------------------------------------------------------- settle and weld

        void FixedUpdate()
        {
            if (_inert || _rb == null || welded) return;

            _t += Time.fixedDeltaTime;

            bool atRest = _rb.linearVelocity.magnitude  < restLinearSpeed
                       && _rb.angularVelocity.magnitude < restAngularSpeed;

            if (!landed && atRest && _t >= minSettleSeconds)
            {
                landed = true;
                Debug.Log($"[StationDeckPhysics] Settled at t={_t:F2} s.");
            }

            if (!weldWhenSettled) return;
            if (landed) { Weld(false); return; }

            if (_t >= maxSettleSeconds)
            {
                Debug.LogWarning($"[StationDeckPhysics] Never came to rest within {maxSettleSeconds:F1} s " +
                                 $"(|v|={_rb.linearVelocity.magnitude:F3} m/s, " +
                                 $"|w|={_rb.angularVelocity.magnitude:F3} rad/s). Welding anyway so the " +
                                 "case does not end up in the water — but something is pushing it, and " +
                                 "welding hides that rather than fixing it.");
                Weld(true);
            }
        }

        void Weld(bool forced)
        {
            var joint = gameObject.AddComponent<FixedJoint>();
            // MixedBody.ConnectToJoint is the project's existing answer to "ArticulationBody or
            // Rigidbody?" — reused rather than re-branched here, because a second copy of that
            // decision is a second place for it to be wrong. It is also the whole reason this
            // works without re-parenting: a Rigidbody may be JOINTED to an articulation chain
            // even though it may not LIVE in one.
            var vessel = new MixedBody(deckArticulationBody, deckRigidbody);
            vessel.ConnectToJoint(joint);
            joint.enableCollision = false;
            joint.breakForce = Mathf.Infinity;
            joint.breakTorque = Mathf.Infinity;

            welded = true;

            var local = vessel.transform.InverseTransformPoint(transform.position);
            Debug.Log($"[StationDeckPhysics] Welded to {deckBodyName} at t={_t:F2} s" +
                      (forced ? " (FORCED, not settled)" : "") +
                      $". Case rests at ({local.x:F3}, {local.y:F3}, {local.z:F3}) in the vessel " +
                      "frame. From here it rides the vessel exactly — including the station GPS, " +
                      "which now reports a MOVING launch point (ADR-007).");
        }

        void Refuse(string why)
        {
            _inert = true;
            Debug.LogError("[StationDeckPhysics] REFUSED: " + why);
        }

        // ---------------------------------------------------------------- editor aid

        /// <summary>Neither collider exists outside Play, so draw where they will be. Cheap, and
        /// it means the case box and the deck plane can both be checked against the visual hull
        /// before anything moves.</summary>
        void OnDrawGizmosSelected()
        {
            var baseLink = transform.Find("base_link");
            if (baseLink == null) return;

            Gizmos.matrix = baseLink.localToWorldMatrix;
            Gizmos.color = new Color(0.94f, 0.62f, 0.06f, 0.9f);
            Gizmos.DrawWireCube(new Vector3(0f, caseSizeM.y * 0.5f, 0f), caseSizeM);

            Gizmos.matrix = Matrix4x4.identity;
            Gizmos.color = new Color(0.2f, 0.8f, 1f, 0.9f);
            float y = Application.isPlaying ? deckSurfaceWorldY : transform.position.y;
            Gizmos.DrawWireCube(new Vector3(transform.position.x, y - deckPadThicknessM * 0.5f, transform.position.z),
                                new Vector3(deckPadSizeM.x, deckPadThicknessM, deckPadSizeM.y));
        }
    }
}
