# Kristineberg terrain — generated build inputs

**Nothing in this directory is hand-authored, and nothing in it should be hand-edited.**
Everything is produced by `data-cube/scripts/kristineberg-site/` from Per Fransson's source
rasters. Editing a file here silently decouples the Unity scene from the measured data — the
whole point of the pipeline is that the scene can be re-derived and re-checked.

| file | what it is |
|---|---|
| `kristineberg_heightmap.r16` | uint16 LE, 2049², no header. Row 0 = SOUTH, col 0 = WEST. |
| `kristineberg_heightmap.json` | provenance: source hashes, merge rule, datum evidence, statistics |
| `kristineberg_dock.smesh` | quay mesh, vertices already in Unity scene coordinates |
| `kristineberg_buildings.smesh` | station campus, same frame |
| `kristineberg_meshes.json` | provenance for the two meshes |
| `kristineberg_unity.json` | flat build contract the editor script reads |

## Frame

Unity **+X = UTM 32N easting, +Z = UTM 32N northing**, origin at K-berg origo
(58.249721 N, 11.44624 E). Water plane at **y = 0**. Terrain is 2048 × 2048 m at exactly
1.00 m/sample, positioned at (−1024, −59, −1024) with `size.y = 113`.

The source data is SWEREF99 TM (EPSG:3006), whose grid north is **5.104° from UTM 32's** here.
Every coordinate is reprojected **point-wise**; a constant offset would be 35–90 m wrong across
this site, and is exactly the defect in the shipped `(UTM)` mesh variants. See
`_test_sites/Kristineberg/KRISTINEBERG_SITE.md` §4.

## Regenerating

```bash
cd data-cube/scripts/kristineberg-site
python3 build_heightmap.py        # rasters  -> heightmap
python3 export_meshes.py          # dock/buildings -> .smesh, into the scene frame
python3 build_splatmap.py         # depth+slope -> terrain control map + placeholder textures
python3 make_unity_manifest.py    # -> kristineberg_unity.json
python3 verify_site.py            # acceptance checks; exit code = failures
```

`textures/` holds **procedural placeholders** — the surfacing *classification* is real
(measured depth and slope), the pixels are not. Replacing them is step 1 of
`data-cube/docs/2026-08-16-kristineberg-texturing-guide.md`; keep the filenames and the
builder picks them up. `Kristineberg_*.mat` and `Kristineberg_*.terrainlayer` are created once
and never overwritten, so hand-tuning survives a rebuild.

Then in Unity: **SMARC → Build Kristineberg Site**, and **SMARC → Verify Kristineberg Site**
with the scene open.

## When real multibeam arrives

Point `DEPTH_TIF` in `site_frame.py` at the new raster and re-run. Nothing downstream changes:
the merge rule, the frame, the scene and both verifiers are source-agnostic. Re-run
`verify_site.py` — check 3 (seabed vs the 1 m contours) will legitimately change, since the new
survey supersedes the contours rather than reproducing them; update that check's source rather
than loosening its tolerance.

Requires `git lfs install` — `.r16` and `.smesh` are LFS-tracked. Without it they arrive as
text pointers and the terrain is flat.
