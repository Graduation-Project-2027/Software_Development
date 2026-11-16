import { useCallback, useEffect, useState } from "react";

const KEY = "amr.theme";

/** Saved choice, else what the operating system asks for. */
export function initialTheme() {
  try {
    const saved = localStorage.getItem(KEY);
    if (saved === "light" || saved === "dark") return saved;
  } catch { /* private mode: fall through */ }
  return window.matchMedia?.("(prefers-color-scheme: dark)").matches ? "dark" : "light";
}

export function applyTheme(theme) {
  document.documentElement.dataset.theme = theme;
}

/** Day / night panel theme, shared by every mounted switch and kept across reloads. */
export function useTheme() {
  const [theme, setTheme] = useState(() => document.documentElement.dataset.theme || initialTheme());

  useEffect(() => {
    const onChange = (e) => setTheme(e.detail);
    window.addEventListener("amr-theme", onChange);
    return () => window.removeEventListener("amr-theme", onChange);
  }, []);

  const set = useCallback((next) => {
    applyTheme(next);
    try { localStorage.setItem(KEY, next); } catch { /* not persisted */ }
    window.dispatchEvent(new CustomEvent("amr-theme", { detail: next }));
  }, []);

  return [theme, set];
}
