using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace SmarcGUI
{
    /// <summary>
    /// Transparent-black restyle for the SmarcGUI panels (Ivan, 2026-08-10).
    /// Runtime and name-based on purpose: disable this component and the GUI is
    /// back to stock — no prefab surgery to undo. Panels whose names are listed
    /// get PanelColor; Images living directly under a Button/Toggle/Dropdown get
    /// ControlColor so interactables stay visually distinct.
    /// </summary>
    public class GUIDarkTheme : MonoBehaviour
    {
        [Tooltip("Applied to panel/background Images (transparent black).")]
        public Color PanelColor = new Color(0f, 0f, 0f, 0.55f);

        [Tooltip("Applied to Images that belong to interactable controls.")]
        public Color ControlColor = new Color(0.08f, 0.08f, 0.08f, 0.75f);

        [Tooltip("GameObject names treated as panels/backgrounds.")]
        public List<string> PanelNames = new List<string> {
            "TopPanel", "ConnectionsPanel", "SettingsPanel", "ROSPanel",
            "ControllerPanel", "DashboardPanel", "BG", "Background",
            "Compass", "Altitude", "Depth", "Speed"
        };

        [Tooltip("Also darken any blue-tinted Image (catches panels the name list misses, e.g. MQTT block, robot rows, virtual controller).")]
        public bool HuntBlueBackgrounds = true;

        [Tooltip("Names never recolored (toggle innards, slider fills, text-input fields stay readable).")]
        public List<string> ExcludeNames = new List<string> {
            "Checkmark", "Handle", "Fill", "TextInput", "Placeholder", "Text",
            "PasswordField", "UsernameField", "Address", "Port"
        };

        void Start()
        {
            // Robot rows and controller panes are instantiated AFTER Start when a
            // vehicle registers — re-apply periodically so late arrivals get themed.
            InvokeRepeating(nameof(Apply), 0f, 2f);
        }

        static bool IsBlueish(Color c)
        {
            return c.a > 0.05f && c.b > c.r + 0.10f && c.b > c.g + 0.10f;
        }

        [UnityEngine.ContextMenu("Apply dark theme now")]
        public void Apply()
        {
            foreach (var img in GetComponentsInChildren<Image>(true))
            {
                string n = img.gameObject.name;
                if (ExcludeNames.Contains(n)) continue;

                bool isControl = img.GetComponent<Button>() != null
                              || img.GetComponent<Toggle>() != null
                              || img.GetComponentInParent<Button>() != null
                              || img.GetComponent<TMPro.TMP_Dropdown>() != null
                              || img.GetComponent<TMPro.TMP_InputField>() != null;

                if (PanelNames.Contains(n))
                    img.color = isControl ? ControlColor : PanelColor;
                else if (HuntBlueBackgrounds && IsBlueish(img.color))
                    img.color = isControl ? ControlColor : PanelColor;
                else if (isControl)
                    img.color = ControlColor;
            }
        }
    }
}
