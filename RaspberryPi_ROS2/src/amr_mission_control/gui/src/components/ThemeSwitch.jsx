import { Sun, Moon } from "lucide-react";
import { useTheme } from "../lib/theme";

/** Two-position selector, like the day / night switch on an instrument panel:
 *  the lever slides to the side that is on. */
export default function ThemeSwitch() {
  const [theme, setTheme] = useTheme();
  const dark = theme === "dark";
  return (
    <button
      type="button"
      className="themesw"
      role="switch"
      aria-checked={dark}
      aria-label="Night panel"
      title={dark ? "Night panel. Switch to day" : "Day panel. Switch to night"}
      onClick={() => setTheme(dark ? "light" : "dark")}
    >
      <Sun className="themesw__mark themesw__mark--day" size={13} strokeWidth={2} aria-hidden="true" />
      <Moon className="themesw__mark themesw__mark--night" size={13} strokeWidth={2} aria-hidden="true" />
      <span className="themesw__lever" aria-hidden="true" />
    </button>
  );
}
