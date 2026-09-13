using System;
using UnityEngine;
using VehicleComponents.Sensors;

namespace Visualizers
{
    /// <summary>
    /// Side-scan waterfall panel: openable, draggable, resizable, range-true across-track and
    /// DISTANCE-true along-track, in the copper palette every survey package uses.
    ///
    /// ── WHY THE RAW ECHO LOOKS WRONG, AND WHAT THIS DOES ABOUT IT ─────────────────────
    /// `SonarHit.GetIntensity()` is Lambert with no time-varied gain:
    ///
    ///     intensity = beamIntensity · (MaxRange−r)/MaxRange · |cos(incidence)| · reflectivity
    ///
    /// The first two terms collapse with range. At 2 m depth over 8 m of water the grazing
    /// angle at 100 m slant is ~3.5°, so |cos| ≈ 0.06 and the range ramp is ≈ 0 — together a
    /// >15× fade from nadir to the swath edge. **Reflectivity only spans 0.18 (clay) → 0.50
    /// (sand) → 0.80 (bedrock) → 0.95 (the car's steel), i.e. at most 5×.** Geometry
    /// therefore buries bottom type: sand and bedrock at the same range differ by 1.6× while
    /// the same sand differs by 15× across the swath, and a small bright target out at 30 m
    /// is dimmer than plain mud under the keel. That is exactly what a raw, un-gained side
    /// scan looks like — and it is why no survey instrument displays one.
    ///
    /// A real unit applies TVG. This panel does the processing side of that, in the DISPLAY
    /// only, leaving the sensor and `SSS_Pub`'s wire data untouched:
    ///
    ///   AGC (default) — an empirical per-range-column gain: a slow running mean of each
    ///                   column over recent pings, divided out. This cancels the range law
    ///                   AND the beam pattern together, whatever they are, without assuming
    ///                   a model — the standard empirical-gain normalisation. What survives
    ///                   is what varies BETWEEN pings at the same range: bottom type,
    ///                   texture, targets, shadows.
    ///   Raw           — the untouched bytes, for checking the sensor itself.
    ///
    /// ── VERIFYING IT IS REALLY THE MEASURED ECHO ─────────────────────────────────────
    /// The footer prints live per-ping statistics straight off `Sonar.Buckets` — peak byte,
    /// mean, and the fraction of bins with any return. Those are the same bytes `SSS_Pub`
    /// publishes; if they move with the seabed, the picture is the measurement.
    ///
    /// ── SCALES ───────────────────────────────────────────────────────────────────────
    /// Across-track: `Sonar.UpdateSidescan()` bins by SLANT range with
    /// bucketSize = MaxRange/NumBucketsPerBeam, so the horizontal axis is linear in range
    /// by construction; markers come from that same arithmetic.
    /// Along-track: each row is one PING, and the panel measures the sonar's own world
    /// displacement between pings, so the vertical axis is metres travelled — correct even
    /// when speed changes, which a rows×constant-speed assumption would get wrong.
    /// </summary>
    public class SSSWaterfallHUD : MonoBehaviour
    {
        public enum Gain { AGC, Raw }

        [Tooltip("Vehicle root to search for the SSS. Empty = this transform.")]
        public Transform vehicleRoot;

        [Tooltip("Open/close key on the LEGACY Input Manager path. This project runs the INPUT " +
                 "SYSTEM path (ENABLE_INPUT_SYSTEM is defined — CinematicDirector uses it " +
                 "unconditionally in this same assembly), so on the rig it is `inputToggleKey` " +
                 "below that decides. Kept and kept in step so a build without the Input System " +
                 "still has a key.")]
        public KeyCode toggleKey = KeyCode.F6;

        [Tooltip("Open/close key on the INPUT SYSTEM path — the one that actually runs here.\n\n" +
                 "FIXED 2026-09-01: Update() read `kb.f6Key` HARDCODED, so the serialized " +
                 "`toggleKey` above was a field that looked adjustable and was not. Rebinding the " +
                 "panel in the Inspector changed nothing, which is the class of defect where a " +
                 "readout and the thing it claims to describe drift apart silently.\n\n" +
                 "It stays F6 by default, because F6 is right when nothing else is competing for " +
                 "it. When a CinematicDirector is running, IT owns F6 (its HUD toggle) and gates " +
                 "this handler — see SetDirectorControl.")]
        public UnityEngine.InputSystem.Key inputToggleKey = UnityEngine.InputSystem.Key.F6;

        [Tooltip("While a CinematicDirector is driving this panel, ignore this component's own " +
                 "toggle key. ON is what resolves the F6 collision: during a take the director " +
                 "decides per shot whether the waterfall is up, and a stray key press must not be " +
                 "able to contradict the shot list on camera. Untick it only if you have first " +
                 "moved `inputToggleKey` to a key the director does not use (it uses F2–F10).")]
        public bool suppressOwnKeyWhileDirected = true;

        public int panelWidth = 900;
        public int panelHeight = 620;

        [Tooltip("Pings of history kept.")]
        public int historyRows = 1200;

        [Tooltip("Across-track display columns; range bins are combined into these by MAX, " +
                 "which preserves a small bright target instead of averaging it away.")]
        public int displayCols = 1024;

        [Tooltip("AGC divides out a slow running mean of each range column, cancelling the " +
                 "range law and beam pattern so bottom type and targets are what remain.")]
        public Gain gain = Gain.AGC;

        [Tooltip("Display gain applied to the AGC ratio before the dB conversion. " +
                 "1 = the local mean sits mid-palette; 2 = +6 dB brighter.")]
        [Range(0.2f, 4f)] public float brightness = 1f;

        [Tooltip("Palette dynamic range, dB peak-to-peak about the local mean. NARROW is the " +
                 "point: bedrock (0.80) over sand (0.50) is only 20·log10(0.8/0.5) = 4.1 dB, " +
                 "so a 60 dB window renders both as the same mid tone. 12 dB spends a third " +
                 "of the palette on exactly that difference — a contrast stretch, same as the " +
                 "one every survey processor puts on the raw record.")]
        [Range(6f, 48f)] public float dynamicRangeDb = 12f;

        [Tooltip("Longest run of unsampled display columns bridged by interpolation. Small on " +
                 "purpose: it closes raycast sampling gaps without erasing acoustic shadows.")]
        [Range(0, 12)] public int maxHoldColumns = 3;

        [Tooltip("Half-width, in display columns, of the window the AGC gain envelope is " +
                 "smoothed over. Must be much wider than a target and much narrower than the " +
                 "range law: too small and the correction eats the very contrast it exists " +
                 "to expose; too large and the swath edges stay dark.")]
        [Range(4, 256)] public int gainSmoothCols = 64;

        public bool open;

        Sonar sss;
        DeepVisionSSS deepVision;
        string sonarLabel = "SSS: none found";

        Texture2D waterfall, lineTex;
        Color32[] pixels;          // row 0 = TOP of the texture = newest ping
        float[] colMean;           // running mean per display column, for AGC
        float[] rowDist;           // along-track metres for each history row (row 0 = newest)
        float[] colRaw;            // this ping, binned to display columns
        float[] colGain;           // colMean smoothed along range: the AGC gain envelope
        Color32[] copper = new Color32[256];

        float lastPushTime = -1f;
        int rowsPushed;
        Vector3 lastPos;
        bool havePos;
        float speedMps, totalDist;
        int rawPeak; float rawMean, rawFill;

        Rect win;
        bool resizing;
        const int TitleH = 22, FooterH = 20, LeftAxis = 46;
        const int MinW = 380, MinH = 260;

        void Start()
        {
            var root = vehicleRoot != null ? vehicleRoot : transform;
            foreach (var s in root.GetComponentsInChildren<Sonar>(true))
                if (s.Type == SonarType.SSS) { sss = s; break; }
            if (sss == null)
            {
                Debug.LogWarning($"[SSSWaterfallHUD] no SSS-type Sonar under '{root.name}'.");
                return;
            }
            deepVision = sss.GetComponent<DeepVisionSSS>();
            sonarLabel = $"{sss.transform.parent.name}/{sss.name}";

            // COPPER, as on every survey plotter. Matplotlib's copper ramp:
            // r = 1.25v, g = 0.7812v, b = 0.4975v, each clamped.
            for (int i = 0; i < 256; i++)
            {
                float v = i / 255f;
                copper[i] = new Color32(
                    (byte)(Mathf.Clamp01(1.250f * v) * 255),
                    (byte)(Mathf.Clamp01(0.7812f * v) * 255),
                    (byte)(Mathf.Clamp01(0.4975f * v) * 255), 255);
            }

            // linear:false — the palette bytes ARE sRGB. Flagging them as linear data skips the
            // sRGB encode on display, which in this linear-space (HDRP) project lifted every
            // mid-tone towards white: the whole record read as saturated cream and the copper
            // ramp looked desaturated, even though the underlying values were mid-palette.
            waterfall = new Texture2D(displayCols, historyRows, TextureFormat.RGBA32, true, false);
            waterfall.filterMode = FilterMode.Trilinear;   // mips + trilinear: without them a
            waterfall.wrapMode = TextureWrapMode.Clamp;    // scrolling minified image shimmers
            pixels = new Color32[displayCols * historyRows];
            colMean = new float[displayCols];
            colRaw = new float[displayCols];
            colGain = new float[displayCols];
            rowDist = new float[historyRows];
            lineTex = new Texture2D(1, 1);
            lineTex.SetPixel(0, 0, Color.white);
            lineTex.Apply();

            win = new Rect(Screen.width - panelWidth - 8,
                           Screen.height - panelHeight - 8, panelWidth, panelHeight);
        }

        void FixedUpdate()
        {
            if (sss == null || waterfall == null || sss.Buckets == null) return;
            float period = sss.frequency > 0.01f ? 1f / sss.frequency : 0.2f;
            if (lastPushTime >= 0 && Time.fixedTime - lastPushTime < period) return;
            float dt = lastPushTime < 0 ? period : Time.fixedTime - lastPushTime;
            lastPushTime = Time.fixedTime;

            // --- along-track: MEASURE the movement, do not assume a speed ---------------
            var pos = sss.transform.position;
            float step = havePos ? Vector3.Distance(pos, lastPos) : 0f;
            lastPos = pos; havePos = true;
            speedMps = dt > 1e-4f ? step / dt : 0f;
            totalDist += step;

            int n = sss.NumBucketsPerBeam;
            int half = displayCols / 2;
            int perCol = Mathf.Max(1, n / half);
            int peak = 0; float sum = 0; int nz = 0;
            // With the sensor's reverberation floor on, every bin is nonzero, so "carries an
            // echo" must mean "above the floor" — 3x the Rayleigh mean clears ~99% of
            // noise-only bins. Without the floor, any nonzero bin is an echo.
            byte echoThresh = sss.UseReverbNoise ? (byte)Mathf.Min(255f, sss.ReverbNoiseMean * 3f) : (byte)0;

            // Port (beam 0) reversed so nadir is the centre; starboard (beam 1) to the right.
            // MAX within each bin, not mean: a 3 m car occupies a handful of 5 cm bins and
            // averaging it against its own shadow is how a target disappears.
            for (int c = 0; c < half; c++)
            {
                int b0 = c * perCol;
                byte mP = 0, mS = 0;
                for (int k = 0; k < perCol && b0 + k < n; k++)
                {
                    byte p = sss.Buckets[b0 + k];
                    byte s = sss.Buckets[n + b0 + k];
                    if (p > mP) mP = p;
                    if (s > mS) mS = s;
                    if (p > echoThresh) nz++;
                    if (s > echoThresh) nz++;
                    sum += p + s;
                    if (p > peak) peak = p;
                    if (s > peak) peak = s;
                }
                colRaw[half - 1 - c] = mP;      // port fans left from the centre
                colRaw[half + c] = mS;
            }

            // Close SHORT unsampled runs by holding the nearest sampled column. A raycast
            // sonar cannot put a ray in every range bin at grazing incidence (see the note in
            // DeepVisionSSS), so a one- or two-column gap means "not sampled", and painting it
            // black would be inventing an echo-free patch that the water never had. The limit
            // is what keeps this honest: a real acoustic shadow — behind the car, behind a
            // boulder — is metres long, many columns wide, and is left untouched and black.
            HoldShortGaps(colRaw, 0, half, maxHoldColumns);        // port
            HoldShortGaps(colRaw, half, displayCols, maxHoldColumns); // starboard
            rawPeak = peak;
            rawMean = sum / (2f * n);
            rawFill = nz / (2f * n);

            // --- AGC ---------------------------------------------------------------------
            // Two stages, and the second one is the whole point.
            //
            // (1) A slow running mean per range column, over pings (~50-ping memory).
            // (2) That mean SMOOTHED ALONG RANGE into a gain envelope.
            //
            // Stage 2 is not cosmetic. Dividing by the per-column mean directly — which is
            // what this did first — removes the range law, but it equally removes ANY signal
            // that is persistent at a given range. Fly a swath with sand at 20 m and bedrock
            // at 60 m and each column normalises to its own average, so both render as the
            // same mid tone: the correction erases precisely the bottom-type contrast it was
            // added to reveal, and a stationary vehicle images as a flat field.
            //
            // The range law and the beam pattern are SMOOTH functions of range; bottom type,
            // targets and shadows are not. Fitting the envelope over a wide range window
            // separates them: the smooth part is divided out, the local part survives.
            const float tau = 0.02f;
            for (int c = 0; c < displayCols; c++)
                colMean[c] = colMean[c] <= 0f ? colRaw[c]
                                              : colMean[c] * (1f - tau) + colRaw[c] * tau;
            SmoothEnvelope(colMean, colGain, 0, half, gainSmoothCols);         // port
            SmoothEnvelope(colMean, colGain, half, displayCols, gainSmoothCols); // starboard

            // --- scroll, newest ping at the TOP of the panel -----------------------------
            // Texture2D row 0 is the BOTTOM row, so "newest at top" means newest goes at the
            // END of the pixel array and history scrolls towards index 0. (v2 flipped a
            // second buffer every ping to achieve this; scrolling the right way costs nothing.)
            Array.Copy(pixels, displayCols, pixels, 0, (historyRows - 1) * displayCols);
            Array.Copy(rowDist, 0, rowDist, 1, historyRows - 1);
            rowDist[0] = step;                      // rowDist[0] = newest, matching the GUI loop
            int top = (historyRows - 1) * displayCols;
            for (int c = 0; c < displayCols; c++)
            {
                float v;
                if (gain == Gain.Raw) v = colRaw[c] / 255f;   // untouched bytes, linear
                else
                {
                    // dB about the running column mean. Linear ratio was wrong here: only
                    // ~10% of bins carry a return, so the mean sits near zero and every real
                    // echo clipped to white — the whole record saturated and bottom type
                    // vanished exactly where it was supposed to appear.
                    float ratio = colRaw[c] / Mathf.Max(colGain[c], 1e-3f) * brightness;
                    float db = 20f * Mathf.Log10(Mathf.Max(ratio, 1e-4f));
                    v = 0.5f + db / dynamicRangeDb;
                }
                pixels[top + c] = copper[(byte)(Mathf.Clamp01(v) * 255f)];
            }
            rowsPushed++;

            waterfall.SetPixels32(pixels);
            waterfall.Apply(true);       // regenerate mips — this is what stops the shimmer
        }

        // ── WHO OWNS THE KEY, AND WHO OWNS THE PANEL ─────────────────────────────────────
        // Not serialized: this is live state for the duration of a take, and an asset that
        // remembers "a director was driving me" would come back from a domain reload claiming
        // a director that no longer exists. Same rule as DeployedTransducer's three readouts
        // (SETTLED §3s8): a live measurement is never baked into a saved object.
        [System.NonSerialized] string directedBy;

        /// <summary>True while a CinematicDirector is driving this panel.</summary>
        public bool IsDirected => !string.IsNullOrEmpty(directedBy);

        /// <summary>
        /// Hand this panel over to a cinematic director, or take it back.
        ///
        /// THE COLLISION THIS RESOLVES, stated once: this panel's toggle key is F6 and
        /// `CinematicDirector.ToggleHudKey` is ALSO F6. In the AskoCurated scene both are in
        /// the same scene at the same time, so one press meant two things — the GUI canvases
        /// flipped AND the waterfall opened, on camera, with no way to tell which the operator
        /// had asked for. The director takes the key rather than the panel giving up its
        /// binding permanently: outside cinematic mode F6 still opens the waterfall, which is
        /// what it has always done and what everyone's fingers already know.
        ///
        /// It also hides the CLOSED-STATE BUTTON. Unity Recorder captures the Game view, so
        /// "▲ SSS waterfall (F6)" sitting in the corner is burned into every frame of every
        /// shot that did not ask for the panel. That is chrome, not instrumentation.
        /// </summary>
        public void SetDirectorControl(bool on, string who)
        {
            if (on == IsDirected && (!on || directedBy == who)) return;
            directedBy = on ? (string.IsNullOrEmpty(who) ? "a CinematicDirector" : who) : null;
            Debug.Log(on
                ? $"[SSSWaterfallHUD] '{directedBy}' has taken this panel: its visibility is now driven " +
                  $"PER SHOT (CameraShot.ShowSSSWaterfall), its own key ({inputToggleKey}) is " +
                  $"{(suppressOwnKeyWhileDirected ? "GATED so the director can own F6" : "STILL LIVE — you unticked suppressOwnKeyWhileDirected")}, " +
                  "and the closed-state button is hidden so it cannot be recorded into the video."
                : $"[SSSWaterfallHUD] released back to manual: {inputToggleKey} toggles it again and the " +
                  "closed-state button is back.");
        }

        void Update()
        {
            // A director that has taken the panel owns F6 for the duration. Checked before the
            // platform branch, so both input paths behave the same.
            if (IsDirected && suppressOwnKeyWhileDirected) return;
#if ENABLE_INPUT_SYSTEM
            // `kb[key]` and not `kb.f6Key`: the hardcoded control was the whole defect — the
            // serialized key field above did nothing on the path this project actually runs.
            var kb = UnityEngine.InputSystem.Keyboard.current;
            if (kb != null && kb[inputToggleKey].wasPressedThisFrame) Toggle();
#else
            if (Input.GetKeyDown(toggleKey)) Toggle();
#endif
        }

        void Toggle() { open = !open; }

        void OnGUI()
        {
            if (!open)
            {
                // No chrome in a take. Unity Recorder captures the Game view, so this button
                // would be burned into every frame of every shot that does not want the panel.
                if (IsDirected) return;
                if (GUI.Button(new Rect(Screen.width - 186, Screen.height - 30, 178, 22),
                               $"▲ SSS waterfall ({inputToggleKey})")) Toggle();
                return;
            }
            win = GUI.Window(GetInstanceID(), win, DrawWindow, GUIContent.none);
            win.x = Mathf.Clamp(win.x, -win.width + 120, Screen.width - 60);
            win.y = Mathf.Clamp(win.y, 0, Screen.height - 40);
        }

        void DrawWindow(int id)
        {
            float w = win.width, h = win.height;
            string mode = deepVision == null ? "raw Sonar"
                : deepVision.Mode == DeepVisionSSS.FrequencyMode.LF340
                    // 2026-08-28: the bin size is MEASURED off the sonar, not written into a
                    // string. It was hardcoded "5 cm", which stayed "5 cm" when the range
                    // became an operator setting and the bins became 4 cm -- a panel telling
                    // the operator the wrong thing about the instrument in front of them.
                    // Same rule as SSS_Pub's max_duration: send the fact, never a copy of it.
                    ? $"LF 340 kHz · {sss.MaxRange / sss.NumBucketsPerBeam * 100f:F0} cm bins"
                    : $"HF 680 kHz · {sss.MaxRange / sss.NumBucketsPerBeam * 100f:F0} cm bins";
            GUI.Label(new Rect(8, 3, w - 190, TitleH),
                sss == null ? sonarLabel
                : $"SSS · {sonarLabel}   {mode}   ±{sss.MaxRange:F0} m/side · {sss.frequency:F0} Hz");

            if (GUI.Button(new Rect(w - 176, 3, 46, 18), gain == Gain.AGC ? "AGC" : "RAW"))
                gain = gain == Gain.AGC ? Gain.Raw : Gain.AGC;
            if (GUI.Button(new Rect(w - 126, 3, 22, 18), "−"))
            { win.width = Mathf.Max(MinW, win.width * 0.8f); win.height = Mathf.Max(MinH, win.height * 0.8f); }
            if (GUI.Button(new Rect(w - 100, 3, 22, 18), "+"))
            { win.width = Mathf.Min(Screen.width, win.width * 1.25f); win.height = Mathf.Min(Screen.height, win.height * 1.25f); }
            if (GUI.Button(new Rect(w - 74, 3, 22, 18), "◐"))
                brightness = brightness >= 3.5f ? 0.4f : brightness * 1.35f;
            if (GUI.Button(new Rect(w - 26, 3, 20, 18), "✕")) open = false;

            // lineTex as well as waterfall: on a domain reload / Stop these are destroyed while
            // OnGUI is still being pumped, and a destroyed Texture2D is "fake null" — it passes
            // a plain reference check and then throws inside GUI.DrawTexture. That is exactly
            // the NRE the previous version threw on every Stop.
            if (sss == null || waterfall == null || lineTex == null)
            { GUI.DragWindow(new Rect(0, 0, w, TitleH)); return; }

            var img = new Rect(LeftAxis, TitleH + 2, w - LeftAxis - 8, h - TitleH - FooterH - 8);
            GUI.DrawTexture(img, waterfall, ScaleMode.StretchToFill, false);

            // ---- across-track range grid ------------------------------------------------
            float halfPx = img.width / 2f, maxR = sss.MaxRange;
            float rInt = PickInterval(maxR, halfPx, 60f);
            DrawV(img.x + halfPx, img.y, img.height, 2, new Color(1, 1, 1, .8f));
            GUI.Label(new Rect(img.x + halfPx + 3, img.yMax + 1, 40, 16), "0");
            for (float r = rInt; r <= maxR + .01f; r += rInt)
            {
                float dx = r / maxR * halfPx;
                DrawV(img.x + halfPx - dx, img.y, img.height, 1, new Color(1, 1, 1, .25f));
                DrawV(img.x + halfPx + dx, img.y, img.height, 1, new Color(1, 1, 1, .25f));
                GUI.Label(new Rect(img.x + halfPx - dx - 16, img.yMax + 1, 46, 16), $"{r:F0}");
                GUI.Label(new Rect(img.x + halfPx + dx - 12, img.yMax + 1, 46, 16), $"{r:F0}");
            }
            GUI.Label(new Rect(img.x + 3, img.y + 2, 90, 16), "◄ port");
            GUI.Label(new Rect(img.xMax - 56, img.y + 2, 56, 16), "stbd ►");

            // ---- ALONG-TRACK scale: metres travelled, from measured displacement ---------
            // Rows are pings, so metres-per-row varies with speed; walk the per-row distances
            // and put a tick wherever the running total crosses the next interval.
            // The whole texture is stretched into img.height, so the pitch is
            // img.height/historyRows — NOT per visible pixel. Getting that wrong put the
            // "1 m" tick at 40% of the panel when the vehicle had moved 1 m out of 2.
            float pxPerRow = img.height / historyRows;
            float spanM = 0;
            for (int r = 0; r < historyRows; r++) spanM += rowDist[r];
            float dInt = PickInterval(Mathf.Max(spanM, 1f), img.height, 55f);
            float acc = 0; float nextMark = dInt;
            for (int r = 0; r < historyRows; r++)
            {
                acc += rowDist[r];
                if (acc >= nextMark)
                {
                    float y = img.y + r * pxPerRow;
                    DrawH(img.x, y, img.width, 1, new Color(1, 1, 1, .18f));
                    GUI.Label(new Rect(2, y - 8, LeftAxis - 4, 16), $"{nextMark:F0} m");
                    nextMark += dInt;
                }
            }
            GUI.Label(new Rect(2, img.y + 2, LeftAxis - 4, 16), "0 m");

            // ---- footer: live evidence this is the measured echo -------------------------
            GUI.Label(new Rect(8, h - FooterH + 1, w - 16, 18),
                $"{speedMps:F2} m/s · {totalDist:F0} m track · {rowsPushed} pings   |   " +
                $"raw peak {rawPeak} · mean {rawMean:F1} · bins with echo {rawFill * 100:F0}%   |   " +
                $"{(gain == Gain.AGC ? $"AGC ±{dynamicRangeDb / 2f:F0} dB ×{brightness:F2}" : "RAW bytes")}");

            var grip = new Rect(w - 18, h - 18, 18, 18);
            GUI.Label(grip, "◢");
            var e = Event.current;
            if (e.type == EventType.MouseDown && grip.Contains(e.mousePosition)) resizing = true;
            if (e.type == EventType.MouseUp) resizing = false;
            if (resizing && e.type == EventType.MouseDrag)
            {
                win.width = Mathf.Clamp(win.width + e.delta.x, MinW, Screen.width);
                win.height = Mathf.Clamp(win.height + e.delta.y, MinH, Screen.height);
                e.Use();
            }
            GUI.DragWindow(new Rect(0, 0, w - 200, TitleH));
        }

        /// <summary>
        /// Fill runs of unsampled (zero) columns shorter than maxRun by linear interpolation
        /// between the sampled columns either side. Runs at the array ends, and runs longer
        /// than maxRun, are left alone — those are the shadows and the genuinely unsampled
        /// far field, and both should read as black.
        /// </summary>
        /// <summary>
        /// Boxcar-average src[lo,hi) into dst over a +/-half window, clipped at the side
        /// boundary so the port envelope never leaks across nadir into starboard. Wide enough
        /// to follow the range law and the beam pattern, far too wide to follow a target.
        /// </summary>
        static void SmoothEnvelope(float[] src, float[] dst, int lo, int hi, int half)
        {
            if (half < 1) { Array.Copy(src, lo, dst, lo, hi - lo); return; }
            double run = 0; int a = lo, b = lo;
            for (int c = lo; c < hi; c++)
            {
                int wLo = Mathf.Max(lo, c - half), wHi = Mathf.Min(hi, c + half + 1);
                while (b < wHi) run += src[b++];
                while (a < wLo) run -= src[a++];
                dst[c] = (float)(run / (b - a));
            }
        }

        static void HoldShortGaps(float[] a, int lo, int hi, int maxRun)
        {
            int i = lo;
            while (i < hi)
            {
                if (a[i] > 0f) { i++; continue; }
                int j = i;
                while (j < hi && a[j] <= 0f) j++;
                int run = j - i;
                if (i > lo && j < hi && run <= maxRun)
                {
                    float v0 = a[i - 1], v1 = a[j];
                    for (int k = 0; k < run; k++)
                        a[i + k] = Mathf.Lerp(v0, v1, (k + 1f) / (run + 1f));
                }
                i = j;
            }
        }

        static float PickInterval(float span, float px, float minPx)
        {
            foreach (float c in new[] { 1f, 2f, 5f, 10f, 20f, 25f, 50f, 100f, 200f, 500f })
                if (c / span * px >= minPx) return c;
            return 1000f;
        }

        void DrawV(float x, float y, float len, float t, Color c)
        {
            var o = GUI.color; GUI.color = c;
            GUI.DrawTexture(new Rect(x - t / 2f, y, t, len), lineTex);
            GUI.color = o;
        }

        void DrawH(float x, float y, float len, float t, Color c)
        {
            var o = GUI.color; GUI.color = c;
            GUI.DrawTexture(new Rect(x, y - t / 2f, len, t), lineTex);
            GUI.color = o;
        }
    }
}
