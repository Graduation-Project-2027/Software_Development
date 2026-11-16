import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import "@fontsource-variable/archivo/wdth.css";
import "./styles/app.css";
import "./styles/fleet.css";
import "./styles/hmi.css";
import App from "./App.jsx";
import { applyTheme, initialTheme } from "./lib/theme";

applyTheme(initialTheme());

createRoot(document.getElementById("root")).render(
  <StrictMode>
    <App />
  </StrictMode>,
);
