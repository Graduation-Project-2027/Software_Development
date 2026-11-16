# Design — LEVERAGE Mission Control

Mode: **Operate**. The tool disappears into the task; it should feel like part of the machine.

## World

A machine controller. The screen is built like the pendant of a machine tool: a graphite frame around a white working drawing, aluminium panels at the sides, and colour used only the way the robot itself uses it.

- **The frame is graphite.** Top bar, event console and status strip are dark and surround the floor plan. The plan stays white paper with black ink, because that is what reads on a projector.
- **The stack light is on screen.** The three lamps in the top bar are the robot's own red / amber / green tower light, lit by the same rules as the firmware (`StatusBar.jsx`, `lampState`). Look at the robot or at the screen and you see the same thing.
- **The e-stop looks like the e-stop.** Red actuator on a yellow ground, as ISO 13850 has it on the machine. Nothing else on screen uses that yellow, and nothing may push the button off the bar.
- **A status strip, as on every HMI.** Bottom edge: link, stack, e-stop chain, drive power, battery, lift, pose, clock. A small square pilot light in front of a label is green, amber or red. Hardware state lives here and nowhere else.
- **Colour is the safety code.** Red is stop or fault. Amber is caution: paused, held, preview. Green is running normally. Everything else is ink on grey. The E-JUST red disc still marks the robot and the step it is on; gold is still lane tape.
- **Occupancy is shape, not colour.** Occupied rack = solid ink square; free = hollow outline.
- **Keys, not pills.** Controls have 3 px corners and a darker bottom edge, and they move down a pixel when pressed.
- **Data sits in rows.** Properties are label left, value right, one rule per row (`.kv`). State is a filled tag in the safety colour, not a dot. Tabs are a strip with the active tab joined to its panel. Keys in the side panels carry words; icons are kept for icon-only keys.
- **Day and night panels.** The selector in the top bar switches the whole screen (`lib/theme.js`, `data-theme` on the root, saved per browser, first visit follows the operating system). At night the frame is a step darker than the panels, and the floor plan and 3D floor are shown as a negative with the hue turned back, so red stays red and lane tape stays gold.

## Tokens

| Role | Value |
| --- | --- |
| Paper (drawing, fields, plates) | `#FFFFFF` |
| Panel (aluminium side panels) | `#ECEEF1` |
| Ground / active / line | `#E3E6EA` / `#DFE2E6` / `#CDD1D7` |
| Ink / muted | `#0F1114` / `#4F545C` |
| Frame (dark scope): surface / ground / line / ink | `#181A1E` / `#111316` / `#2F3339` / `#F2F3F5` |
| Lamp red / amber / green | `#FF3B42` / `#FFB020` / `#2FD07A` |
| E-stop yellow | `#F6C400` |
| Signal red (robot, danger buttons) | `#E11D27`; text `#C4161F` |
| Gold (lanes) | `#DE9242`; text `#8A5410` |

The frame is dark by re-scoping the same tokens on `.topbar`, `.sb` and `.log` (`styles/hmi.css`), so every component inside follows without a second set of rules.

Type: **Archivo** (variable, self-hosted) for everything, set slightly condensed (96 %) like a panel legend, numerals and small labels at 88 to 90 %. The LEVERAGE wordmark uses the expanded width. Tabular figures on every number. No all-caps labels. Scale 11 / 12 / 13 / 14 / 18 / 20 px.

Radius: 3 px panels and controls, 2 px chips. Edges are 1 px lines; shadows only on things that float (menus, tooltips, map overlays).

Spacing: 4 px base; one gap (12 px) inside panels, 16 px panel padding.

## Layout

```
┌ top bar: emblem LEVERAGE · connection · system ─────── mission controls · E-STOP ┐
├ Tasks | Locations ┬──────────── floor plan (map) ─────────────┬─ Robot ───────────┤
│ composer          │ tools         mode banner                 │ state + steps     │
│ queue             │                                           │ telemetry         │
│                   │ legend                    scale · cursor  │ drive · lift      │
│                   ├──────────── event log (collapsible) ──────┤ speed             │
└───────────────────┴───────────────────────────────────────────┴───────────────────┘
```

- ≥1280 px: three columns. 900–1279 px: Robot folds into the left tabs. <900 px: map on top, one tabbed panel below; E-stop stays in the top bar.

## Robot computer and RViz layers

- **System sheet** (right drawer from the top-bar stack button): target, mode, map, RViz, launch/restart/stop, live launcher output on an ink panel (the one place monospace is used: it is literal process output).
- **Maps tab**: rows with a real thumbnail of the map, tags (In use = ink, Default, Edited, No pose graph = gold), primary "Navigate with …", the rest in a menu. Delete is undoable via toast.
- **Overlays on the map** are paper cards with the float shadow, top-centre: the Mapping bar (live red sun, explored area, save) and the editor toolbar (tool, paint, size, undo/redo, save). Transient modes (pick, lane) use the ink banner.
- **Layers** popover mirrors RViz displays, grouped Map / Navigation / Sensors / Mission, each a labelled switch with its topic. Lanes also get a direct toggle in the map toolbar.
- **Sensor colours** stay inside the world: lidar = red points (rear lidar gold-ink), dynamic obstacles = red outlined boxes, costmap = gold ramp to red at inscribed/lethal, footprint = dashed ink, rack legs = gold-ink points.
- **Lanes are road-like**: straight floor-tape pieces plus rounded turning arcs at every bend and junction (same radius as the robot, from `lane_corner_radius`); junctions get a small paper node.

## Fleet

The laptop's view of every robot, in the same world: a white architectural model of the site on drafting paper.

- **Workspaces.** Top bar: `Fleet | <robot>`. Fleet shows every robot; the robot workspace is the single-robot mission control above, pointed at the chosen robot.
- **Layout.** Left: roster (in the fleet / found on the network / needs attention / set up a new Pi). Centre: 3D floor + fleet event log. Right: Robot · Dispatch · Layout · Maps. Below 1280 px all five share the left tabs; below 900 px the floor sits on top.
- **3D floor (React Three Fiber).** Paper floor printed from the map at true colours (unlit; shadows on a separate catcher, 3D view only), 1 m / 5 m rules. Only real walls (runs ≥ 0.4 m) are extruded in plaster `#D9DBE0`; rack-leg marks stay printed. Racks are ink frames, free sites hollow ink squares, zones dashed. Lanes are gold tape with rounded junctions; open ends carry a gold-ink stop bar, and while drawing a ring marks where an end will join. Robots are their black chassis with the red disc and a white heading bar; a carried rack rides on the robot. 3D for watching, Plan (orthographic) for drawing lanes.
- **Traffic language.** Each driving robot's reserved floor is a faint ink strip; the pair in conflict turns gold, the held robot gets a gold ring and a gold tag ("Holding for AMR-01"). Red stays reserved for E-stop and deadlocks. A live halo marks robots that are actually moving.
- **Robot glyph.** The chassis from above everywhere a robot is named: red sun = driving, gold ring = held, hollow = stopped or not answering, dashed = new robot.
- **Map sync cells.** Per robot: ink = same copy, gold = different, dashed = missing, red dot = running now.
- **Faces.** Every rack/zone draws an ink arrow from its front edge to a small ring on the lane: the one entry point. Hovering a site in the list turns its arrow red. A site not square to a lane is flagged in gold.
- **Schematic lanes.** In Edit nodes every node is a paper disc with an ink ring (gold ring = open end, ink fill = selected); while drawing, a gold-ink ring shows the node a click will link to (red = closes the loop). One-way lanes carry gold-ink chevrons.
- **Pi setup sheet.** The right drawer of the System sheet: Connect → Configure (facts card, Robot ID → hostname) → job (numbered steps with the live sun on the current one, ink log panel).
- E-stop all is never behind a confirmation.

## Motion

150–220 ms, ease-out. Only for state: tab change, inline editor open, queue reorder/remove, new log line, live halo on the moving robot. Respects `prefers-reduced-motion`.

## Components

Built on Radix primitives (`radix-ui`) styled with these tokens (`src/components/ui`): Button, IconButton, Tabs, Tooltip, Select, Segmented (ToggleGroup), Slider, Switch, Popover, ConfirmDialog, Dialog (sheet), DropdownMenu, Toast. Icons: lucide-react, 1.75 stroke.
