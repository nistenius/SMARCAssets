# Beckholmen dry dock — high-res, metric, georeferenced

Built 2026-08-10 from `_example_data_sets/2024-05-20 Scanning Drydock Beckholmen/Model/Meshroom/dockRaw2`.
Pipeline: `data-cube/scripts/beckholmen-hires/` · full write-up: `data-cube/docs/2026-08-10-beckholmen-hires-model-plan.md`

## Files

| file | verts | tris | use |
|---|---|---|---|
| **`beckholmen_hires_dropin.dae`** | 3,575,234 | 7,150,939 | textured visual mesh + **sonar collider mesh** (488 MB) |
| `beckholmen_hires_atlas.png` | — | — | its texture, 10240 × 8192 (85 MB) |
| `beckholmen_hires_preview.dae` | 321,458 | 645,481 | fast placement check / LOD (untextured) |
| `beckholmen_hires_collider.dae` | 80,428 | 161,630 | **physics contact collider** |

**All files share one format** (2026-08-10, after the drop-in was confirmed in place): same
coordinate space as `Drydock/beckholmen_boatless.dae`, same node matrix, and all take the same
local transform when parented under `Drydock` — position (0, 0.57, 0), rotation (0, 90, −90).
The earlier world-baked exports (`_dock`, `_dock_textured`) are deleted; the drop-in supersedes them.

## ⚠️ A COLLIDER CAN GO SILENTLY DEAD — AND "obst clear" IS THE SYMPTOM

**CORRECTION (2026-08-10, later the same day).** An earlier version of this file claimed a
7.15 M-tri MeshCollider *cannot* work and that PhysX's 2,097,152-triangle cap was the cause. **That
was wrong.** Ivan re-enabled the 7.15 M collider and it produces returns normally: hit particles,
`Alt 4.9 m`, `obst STOP 8.2 m`, with every other dock collider in the scene disabled.

What actually happened is almost certainly an **incomplete or interrupted PhysX cook**. The
collider was created, then `Use Fast Midphase` was toggled (forcing a re-cook of 7.15 M triangles)
and the editor was interrupted before it finished. It stayed in a broken state — returning nothing —
until a later property change (assigning the physics material) forced a clean re-cook.

So the honest lesson is not a triangle limit. It is:

1. **A MeshCollider can be silently dead.** No error, no exception — just no hits.
2. **`obst clear` is indistinguishable from `obst broken`.** An empty sonar cloud reads as "clear",
   so collision avoidance concludes there is no obstacle. This is the dangerous failure mode.
3. **Always verify a collider by seeing returns**, never by absence of errors. And after any change
   to a large collider, let the cook finish before judging it.

Unity's warning (*"Source mesh has over 2,097,152 triangles and is using the Fast Midphase option…
collisions may not be detected correctly"*) is still real, and is a reason to prefer a mesh under
the cap if you want certainty — but it does not mean the collider will do nothing.

The A/B that misled me (both readings are reproducible; the difference was cook state, not size):

| | 7.15 M collider | 162 k collider |
|---|---|---|
| `RayViewer` hit particles | none | rainbow fan, as before |
| `Alt` (DVL) | `-` | **4.7 m** |
| `obst` | `clear` (i.e. empty cloud) | **STOP 8.2 m** |

Note `obst clear` is *not* reassuring — an empty sonar cloud reads as "clear", which is the
dangerous failure mode: avoidance silently believes the dock is not there.

**Current working configuration — ONE object, the simplest thing that works:**

- `beckholmen_hires_dropin` — MeshFilter 7.15 M (visual), **MeshCollider enabled**, Convex off,
  Material `Rock`, layer Default. Serves both sonar raycasts and physics contacts.
- `beckholmen_hires_collider` (162 k) and `beckholmen_hires_preview` — **disabled**, kept as
  fallbacks.
- `beckholmen_boatless` — disabled.

This is exactly the old topology: one model, collider on it. No second object needed after all.

`beckholmen_hires_sonar.dae` (1.89 M tris, under the PhysX cap) exists as a **de-risked fallback**
if the full-res collider ever behaves oddly — it carries no Unity warning. The `SonarGeometry`
layer (User Layer 11, empty collision-matrix row) is defined but unused; harmless.

### Physics material — leave it empty, it makes no difference

The old collider had `Rock.physicMaterial`. Do **not** bother re-assigning it:
`Rock` is `dynamicFriction 0.6, staticFriction 0.6, bounciness 0, combine Average` — **identical to
Unity's built-in default** used when Material is None. Also note it is the legacy `PhysicMaterial`
type inside an immutable package, and Unity 6's object picker does not search Packages, so the
picker will only ever offer "None". Nothing is lost.

### Why the old scene needed one model and this one needs two meshes

The old `beckholmen_boatless` was **201 k triangles** — under the 2.1 M cap, so the same mesh could
be both the visual and the collider. One mesh, one object.

The new visual mesh is **7.15 M** — 3.4× over the cap, so it cannot be a collider at all. A second,
smaller mesh has to exist.

**But that means two _meshes_, not necessarily two _objects_.** A MeshCollider's `Mesh` field can
reference any mesh, not just the one in its own MeshFilter. Extra GameObjects are only genuinely
required if you want the sonar/contact **layer** split, because layer is a per-GameObject property.

#### CHOSEN SETUP — two objects (visual + collider)

| object | mesh | layer | MeshRenderer | MeshCollider | role |
|---|---|---|---|---|---|
| `beckholmen_hires_dropin` | 7.15 M | Default | **on** | **disabled** | what you see |
| `beckholmen_hires_sonar` | **1.89 M** | Default | **off** | **enabled**, Convex off | what you hit — sonar *and* contacts |
| `beckholmen_hires_collider` | 162 k | — | — | — | **disable it**, superseded |

Two objects doing one job each, and no mesh-field assignment needed — each object auto-fills its
own mesh on import. The `SonarGeometry` layer (User Layer 11) is not needed in this setup; it stays
defined but unused, harmless. Only adopt the three-object split if contact cost against 1.89 M ever
becomes a problem — it is a static mesh, so PhysX uses a BVH and this is not expected.

#### Alternative — one object

Possible but fiddlier: enable the drop-in's own MeshCollider and point its `Mesh` field at the
1.89 M mesh. Requires dragging a mesh sub-asset out of a package, which the object picker will not
list. Not recommended.

### Higher-resolution sonar collider — `beckholmen_hires_sonar.dae`

941,997 verts / **1,887,562 tris** — 90 % of the PhysX cap, with ~210 k triangles of headroom.
~12× the detail of the 162 k contact mesh. This makes the original two-collider plan viable after
all; it just has to live under 2,097,152 triangles rather than at the full 7.15 M.

Wiring (all objects are children of `Drydock`, local transform (0, 0.57, 0) / (90, 0, −90)):

| object | mesh | layer | MeshRenderer | role |
|---|---|---|---|---|
| `beckholmen_hires_dropin` | 7.15 M | Default | **on** | visual only — MeshCollider stays **disabled** |
| `beckholmen_hires_sonar` | 1.89 M | `SonarGeometry` | off | **sonar/altimeter raycasts** |
| `beckholmen_hires_collider` | 162 k | `Ignore Raycast` | off | **physics contacts** |

Both non-visual objects need Convex **off**. Cooking 1.89 M triangles freezes the editor for a
while on first import — once, then cached.

**Verify by seeing returns, not by absence of errors:** in Play, you must get `RayViewer` hit
particles and a numeric `Alt`. `obst clear` with no particles means the collider is silently dead
(see the 2.1 M warning above) — that is the failure mode to watch for.

## WIRED AND VERIFIED IN THE SCENE 2026-08-10

Done in `Beckholmen.unity` and saved:

- `SonarGeometry` = **User Layer 11**; its entire row in Physics → Settings → Layer Collision Matrix
  is unticked.
- `beckholmen_hires_dropin` (child of `Drydock`, pos (0, 0.57, 0), rot (90, 0, −90)) — MeshCollider,
  Convex off, mesh `node`, layer `SonarGeometry`, material `atlasMat`.
- `beckholmen_hires_collider` (child of `Drydock`, same transform) — MeshCollider, Convex off,
  MeshRenderer disabled, layer `Ignore Raycast`.
- `beckholmen_boatless` disabled (kept for A/B).

**PhysX Fast Midphase had to be turned off.** Unity warned: *"Source mesh has over 2,097,152
triangles and is using the Fast Midphase option. This might cause certain collisions to not be
detected correctly."* The 7.15 M-tri sonar collider is well over that cap, so `Use Fast Midphase`
is unticked in its Cooking Options (Cooking Options now reads "Mixed..."). Keep it off — otherwise
sonar rays can silently miss geometry.

**Play-mode verification passed:** HUD reported `Alt: 1.8 m` and `obst STOP 1.0 m`, action
`OBSTACLE STOP — holding depth`. Both the altimeter and the obstacle detector are getting returns
off the hi-res collider, which proves the layer setup works — raycasts hit `SonarGeometry` even
though its collision-matrix row is empty.

⚠️ **Open issue found during that test:** SAM triggers `OBSTACLE STOP` at 1.0 m *immediately at
spawn*. The spawn point (62.866, −0.153, 115.429) was set against the old, 5.8 % larger dock; the
walls have since moved ~0.3 m inward and the dock is ~4.3 m shorter, so the spawn now sits close to
geometry. Re-check the spawn point and the mission legs before reading anything into avoidance
behaviour — this is exactly the re-validation the scale change requires.

## Colliders — none are baked in; they are components you add

A `.dae` carries only geometry. Collision in Unity comes from a **MeshCollider component** that
references some mesh — so "does it have a collider" is decided in the prefab, not in the file.
Per the sonar-realism decision (sonar raycasts should see the hi-res surface; contacts can be
rough — `Sonar.cs` uses `QueryParameters.Default`, which hits everything except layer
*Ignore Raycast* and ignores the Layer Collision Matrix):

1. **Sonar collider = the drop-in's own mesh.** On the `beckholmen_hires_dropin` object, Add
   Component → MeshCollider (leave *Convex* off; it references the same 7.15 M-tri mesh — no
   separate file needed). Put the object on a dedicated layer (e.g. `SonarGeometry`, add it under
   Project Settings → Tags and Layers) and **untick that layer's entire row in Project Settings →
   Physics → Layer Collision Matrix**. Result: raycasts (sonar, altimeter) hit the full-detail
   surface; physics contacts never fire on it. First PhysX cook of 7 M tris is slow (once) and
   costs a few hundred MB — if that hurts, use the preview mesh for the sonar collider instead
   (cm-level deviation, still far beyond sonar resolution).
2. **Contact collider = `beckholmen_hires_collider.dae`.** Add it under `Drydock` with the same
   local transform, MeshCollider referencing its 162 k-tri mesh, MeshRenderer disabled, layer
   **Ignore Raycast**. Physics contacts work; sonar never sees it, so returns aren't polluted by
   the smoothed surface.
3. **Do not** use the 7.15 M mesh as the *contact* collider — PhysX contact generation against a
   non-convex mesh that dense is exactly the cost the two-collider split avoids, and detailed
   contacts aren't a goal.

### Textures

The 79 Meshroom UDIM tiles are repacked into **one 10 × 8 atlas** — no per-face material logic was
needed, because Meshroom already writes the UVs in global UDIM space (u ∈ [0,10), v ∈ [0,8)), so the
remap is just `u/10, v/8` and the mesh keeps a single material.

- Tiles downsampled 2048 → **1024** so the atlas fits Unity's 16384 limit in both axes.
  That is ~8 mm/texel over the dock — far finer than the geometry.
- EXR is linear; converted to sRGB properly (piecewise, not a naive gamma).
- The spare 80th atlas cell is filled flat grey and **all 2,812 gap-patch triangles point at it**,
  so the filled holes read as neutral concrete rather than smeared texture.
- `beckholmen_hires_atlas.png.meta` sets `maxTextureSize: 16384` — without it Unity would resample
  the atlas to 2048 and destroy it.
- Mapping verified before import by sampling the atlas through the UVs and rendering top-down:
  0.2 % of samples land on empty atlas, and the dock reads correctly (floor markings legible, not
  mirrored). If it looks vertically flipped in Unity, the fix is `v → 1−v`.

## USE THIS ONE: `beckholmen_hires_dropin.dae` (convention-proof)

Everything below about baked world coordinates and Unity's axis conversion was an attempt to
*infer* the importer's convention from how the model appeared. That approach failed — it was
inferring across reimports, which can reset the object's transform, so the observations weren't
taken under identical conditions, and it produced a **mirrored** model (a mirror is something no
rotation can produce, which is what finally exposed it).

`beckholmen_hires_dropin.dae` sidesteps the question entirely. Its vertices are written in the
**exact coordinate space of `Drydock/beckholmen_boatless.dae`** — same `up_axis`, same node matrix,
same raw array frame — so whatever Unity does on import, it does identically to both files.

### How to use it — exact steps

The old dock is **not** a MeshFilter on `Drydock`. `Drydock` has only a Transform; the dock is a
nested *model-prefab instance* named `beckholmen_boatless` sitting under it, with this local
transform (read out of `BeckholmenWorld.prefab`):

| | value |
|---|---|
| Position | **(0, 0.57, 0)** |
| Rotation | **(0, 90, −90)** — quaternion (0.5, 0.5, −0.5, 0.5) |
| Scale | (1, 1, 1) |

The drop-in is a structural clone of that file, so it needs the **same** local transform.

**1 — Clean up the earlier attempts.** In the Hierarchy, delete `beckholmen_hires_dock`,
`beckholmen_hires_preview` and `beckholmen_hires_dock_textured` wherever they ended up. Leave
`beckholmen_boatless` in place for now, just unchecked (disabled) so you can A/B against it.

**2 — Reimport.** Project window → right-click the `DrydockHiRes` folder → **Reimport**.
Wait for it; `beckholmen_hires_dropin.dae` is 488 MB.

**3 — Drag it in.** Drag `beckholmen_hires_dropin` from the Project window onto
**`BeckholmenWorld → Drydock`** in the Hierarchy, so it becomes a **child of `Drydock`** —
a sibling of `beckholmen_boatless` and `Water`, at the same level.

**4 — Copy the transform exactly.** Easiest and least error-prone:
select `beckholmen_boatless` → right-click its **Transform** component header → **Copy Component**
→ select the new `beckholmen_hires_dropin` object → right-click its Transform → **Paste Component
Values**. (Or type the table above in by hand.)

**5 — Check it.** The hi-res dock should sit exactly on the old one, ~4.5 % smaller, textured, and
**not mirrored** — the red house must be on the same side as in `beckholmen_boatless`. Toggle
`beckholmen_boatless` on/off to compare.

If the red house is still on the wrong side, stop and say so — that means the import mirrors and
the fix is a single sign, not another round of guessing.

Because it rides the existing, known-good transform, position, orientation and handedness are
correct **by construction** rather than by my inference.

What it does and does not include:

- ✅ metric scale — the 4.5 % shrink is baked, applied about the dock centre. Uniform scaling is
  isotropic so there is no sign ambiguity; this is safe.
- ✅ all 51 holes filled, textured via the atlas, 7.15 M tris.
- ❌ **the 2.21° heading correction is NOT applied.** Its sign depends on whether the import chain
  preserves handedness, which is exactly the thing that proved unreliable to infer. The new dock
  therefore inherits the old dock's heading — still ~2.2° off UTM grid north. Once the drop-in is
  confirmed in place, that becomes a single verifiable step on the `Drydock` transform.

---

## Older approach (baked world coordinates) — kept for reference

**The transform must be set to identity by hand.** Position, rotation and scale are baked into the
vertices, so the model georeferences itself against `GLOBALREF - Beckholmen`.

1. **Force a reimport** (right-click the `DrydockHiRes` folder → Reimport). Unity only re-reads
   changed files when the editor regains focus, and these were rewritten after the first attempt.
2. Drag `beckholmen_hires_preview.dae` into the scene as a child of **`BeckholmenWorld`**.
3. Set its Transform to **position (0,0,0), rotation (0,0,0), scale (1,1,1)**.
   Unity's COLLADA import leaves a **rotation of (−90, 0, 90)** and a non-zero position on the
   object — both must be cleared, or the model is 90° out and offset.
4. It should land on top of the existing `Drydock` object — same place, slightly smaller (below).

Start with the preview file (21 MB, reimports in seconds); swap to `beckholmen_hires_dock.dae`
once the placement looks right (250 MB / 7.15 M tris — slow import).

### Unity's axis convention for these files (measured, 2026-08-10)

Unity does **not** bake the COLLADA axis conversion into the vertices for these files — it puts it
in the imported root node's rotation. With that rotation cleared, the observed mapping is:

```
unity_vertex = ( −file_x , file_y , file_z )        # plain right-handed → left-handed X flip
```

**Consequence: triangle winding must be reversed in the file.** Negating X mirrors handedness, and
Unity does *not* re-order the indices to compensate. These files carry no normals (Unity computes
them from winding), so unreversed winding gives **inward-facing normals** — the model renders only
from the inside, looks shredded and see-through from outside, and the texture appears on the wrong
face. All exports now write triangles as `(v0, v2, v1)`.

### Parenting — put it under `BeckholmenWorld`, NOT under `Drydock`

The coordinates are baked in Unity **world** space. `BeckholmenWorld` is at identity so parenting
there is safe. `Drydock` is **not** — it carries position (30.87, 0, 78.63) and a −134.29° Y
rotation, which any child inherits, throwing the model ~85 m off and spinning it 134°.
Sun and sky do not depend on parenting: `Sun` is a directional light and `Sky and Fog Global Volume`
is a global volume, so both light the whole scene wherever the object sits.

**There is no axis permutation** — that false premise cost four iterations. Unity simply negates X.
The rule was recovered by fitting one hypothesis to four live observations; it reproduces all of
them, so it is overdetermined rather than guessed:

| export written as | predicted centre offset / bearing | observed in Unity | ✓ |
|---|---|---|---|
| `(W_z, −W_x, −W_y)` | unity-Y ← −W_x, span 87 m | 87 m tall **vertical slab** | ✓ |
| `(W_x, W_y, W_z)` | (−65, −1), bearing 131.6° | flat, **~90° rotated**, left | ✓ |
| `(W_z, W_y, −W_x)` | (−108, −112), bearing 41.6° | parallel, offset **down-left** | ✓ |
| `(−W_z, W_y, W_x)` | (+47, −44), bearing 41.6° | parallel, offset **down-right ~65 m** | ✓ |
| **`(−W_x, W_y, W_z)`** | **(+3.6, −0.6), bearing 48.4°** | ← current export | — |

That last row is the tell: the predicted residual offset is exactly the *intended* correction
(metric shrink + 2.2° heading + ~2 m position) and the bearing is exactly the true grid bearing.
A larger offset than ~4 m means something is still wrong; ~4 m at bearing 48.4° means it is right.

### Verify it landed correctly

With the object at identity, in the Unity scene:

| check | expected |
|---|---|
| dock basin centre | unity **X ≈ 32.0, Z ≈ 76.5** |
| dock floor | unity **Y ≈ −6.3 m** (water surface is Y = 0) |
| model bounding box | X −9.1…78.0, Y −6.9…7.0, Z 39.3…115.4 |
| dock inner width | **≈ 14.4 m** (old model: 15.1 m) |
| basin centre lat/lon | **59.3202584 N, 18.1007930 E** (UTM 34V 335016.97 / 6579304.91) |
| dock grid bearing | **48.39°** (old model: 46.18°) |

If the dock appears mirrored or 90° out, the importer's axis conversion differs from the one this
was built against — tell Claude, do not hand-rotate it (the georeference would silently break).

## What changed vs `Drydock/beckholmen_boatless.dae`

- **Scale: −4.5 %.** The old model was ~5.8 % oversized. Proven three independent ways: 651 camera
  GPS poses, the documented 11 m width of Västra dockan, and the ground survey in
  `_example_data_sets/2024-10-11_Drydock_isss/gps.txt` (94.38 m tail-corner→head; metric model
  93.68 m, old model 98.13 m).
- **Heading: +2.21°.** The old dock was aligned to *true* north; Unity's georeference is UTM *grid*
  north. The difference is meridian convergence (−2.494° here).
- **Each wall moves inward ~0.3 m; the dock is ~4.3 m shorter.** Mission margins change — the real
  dock is tighter than the old sim reported.
- 35× the triangles; keel blocks, wall steps and stair detail now resolved.
- All 51 holes closed (ear-clipped + subdivided + smoothed). Only the outer model rim remains open,
  which is correct — it is the edge of the survey, not a hole.

## Not done yet

- **No textures.** UVs exist in the source (79 UDIM tiles @ 2048²) but are not exported yet; the
  mesh imports untextured. Repack script is specified in the plan doc, Phase 4.
- No normals in the file — let Unity calculate them (the `.meta` is set for this).
- Prefab wiring (sonar layer vs contact layer) not applied; see plan doc Phase 5.
