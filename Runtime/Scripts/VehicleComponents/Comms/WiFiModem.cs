using UnityEngine;

namespace VehicleComponents.Comms
{
    /// <summary>
    /// The surface WiFi link — SAM to the base station, used at the quayside and for anything
    /// close alongside.
    ///
    /// HARDWARE (2026-08-15). **Antenna: confirmed** — a **Taoglas SWDP.2438.A** ceramic patch on
    /// the potted three-antenna board (WiFi + GPS + 4G) under the clear dome on the nosecone top
    /// rail, legible in the 2024 Dialogdagarna deck, slide 9. **Radio: still unnamed**; on PI
    /// direction we build on the **Jetson AGX Orin dev-kit module (AzureWave AW-CB375NF /
    /// RTL8822CE, 802.11a/b/g/n/ac 2×2, BT 5.0)**, consistent with the IP convention that gives
    /// each Jetson a WiFi address while the NanoPi Neo3 — which has no WiFi — handles Ethernet
    /// and LTE. **Inferred, not a confirmed wiring fact.**
    ///
    /// **The link is 2.4 GHz.** The antenna is a 2.4 GHz patch, so the module's 5 GHz half is
    /// unavailable on this hull however the radio is specified — do not model 5 GHz throughput
    /// here. Shore end (Teltonika RUTX11 + Poynting PUCK-5) does offer both; 2.4 GHz is where
    /// they actually meet, at 23.18 dBm ERP shore-side.
    /// See `data-cube/docs/SAM_VEHICLE_SPECS.md` §2.3–2.5.
    ///
    /// RANGE: ~25 m working range from the base station, PI-stated 2026-08-15, with a sharp
    /// fall-off beyond it. That is an OPERATIONAL figure from running the vehicle, not a
    /// datasheet number, and the reason a datasheet number would be wrong is in
    /// SurfaceRadioModem's summary: at a few centimetres of antenna height over water the first
    /// Fresnel zone is obstructed and the two-ray null puts essentially the whole usable range
    /// into fourth-power path loss. Twenty-five metres over water is entirely consistent with a
    /// radio that would do two hundred in a field.
    ///
    /// Full status payload at ~1 Hz whenever it is up — this is the link everything else is
    /// measured against.
    /// </summary>
    [AddComponentMenu("Smarc/Comms/WiFi Modem (surface)")]
    public class WiFiModem : SurfaceRadioModem
    {
        public override string LinkName => "wifi";
        public override float ExpectedHz => 1.0f;
        // Nominal usable throughput. Any real WiFi vastly exceeds what this system sends; the
        // number exists so a consumer can reason about payload sizes uniformly across links,
        // not as a performance claim.
        public override float BitsPerSecond => 10e6f;

        void Reset()
        {
            // Defaults that match how SAM is actually operated, applied when the component is
            // first added so nobody has to remember them.
            operationalRangeM = 25f;
            marginalRangeM = 40f;
            antennaHeightM = 0.15f;
        }

        public override void Evaluate()
        {
            EvaluateSurfaceRadio();
        }
    }
}
