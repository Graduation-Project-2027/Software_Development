export const DEFAULT_STATE = {
  state: "IDLE",
  robot_pose: { x: 0, y: 0, yaw: 0 },
  current_task: null,
  current_step: "",
  progress: 0,
  carrying: false,
  speed: 0.4,
  queue: [],
  locations: { dock: { x: 0, y: 0, yaw: 0, name: "Dock" }, racks: [], drop_zones: [], lanes: [] },
  // Real robot only (null in simulation and preview):
  error: "", // why the state is ERROR: "estop" | "task"
  estop: false,
  safety: null, // e-stop controller: { online, active, cause, button, power }
  battery: null, // drive board: { online, voltage, percent, low, error_left, error_right }
  lift: null, // lift controller: { online, state, fault, homed, height, at_target }
};

/** Merge a raw /mission_control/state payload over safe defaults. */
export function normalizeState(data) {
  if (!data || typeof data !== "object") return DEFAULT_STATE;
  const loc = data.locations || {};
  return {
    ...DEFAULT_STATE,
    ...data,
    queue: Array.isArray(data.queue) ? data.queue : [],
    robot_pose: { ...DEFAULT_STATE.robot_pose, ...(data.robot_pose || {}) },
    locations: {
      dock: { ...DEFAULT_STATE.locations.dock, ...(loc.dock || {}) },
      racks: Array.isArray(loc.racks) ? loc.racks : [],
      drop_zones: Array.isArray(loc.drop_zones) ? loc.drop_zones : [],
      lanes: Array.isArray(loc.lanes) ? loc.lanes : [],
    },
  };
}

/** tone: idle | live | caution | fault | done */
export const STATE_META = {
  IDLE: { label: "Idle", tone: "idle", hint: "Ready for a mission" },
  RUNNING: { label: "Running", tone: "live", hint: "Working through the queue" },
  PAUSED: { label: "Paused", tone: "caution", hint: "Resume to continue the task" },
  NAVIGATING: { label: "Navigating", tone: "live", hint: "Driving to the next pose" },
  LIFTING: { label: "Lifting", tone: "live", hint: "Raising the rack" },
  DROPPING: { label: "Lowering", tone: "live", hint: "Setting the rack down" },
  RETURNING: { label: "Returning", tone: "live", hint: "Driving back to the dock" },
  ERROR: { label: "Emergency stop", tone: "fault", hint: "Check the event log, then release the E-stop" },
  COMPLETED: { label: "Completed", tone: "done", hint: "All tasks finished" },
};

export function stateMeta(state) {
  return STATE_META[state] || { label: state || "Unknown", tone: "idle", hint: "" };
}

/** SafetyStatus.cause of the e-stop controller, in the operator's words. */
export const ESTOP_CAUSES = {
  1: "Stopped from mission control",
  2: "E-stop button on the robot",
  3: "Controller lost the robot computer",
};

/** True when ERROR means a failed task, not an emergency stop. */
export function isTaskFault(ms) {
  return ms?.state === "ERROR" && ms.error === "task";
}

/** stateMeta, with ERROR told apart: failed task, pressed button, lost computer. */
export function missionMeta(ms) {
  const meta = stateMeta(ms?.state);
  if (ms?.state !== "ERROR") return meta;
  if (isTaskFault(ms)) return { label: "Task failed", tone: "fault", hint: "Check the event log, then clear the fault" };
  const s = ms.safety;
  if (s?.online && s.button) return { ...meta, hint: "The button on the robot is pressed. Twist it out, then release the E-stop here" };
  if (s?.online && s.active && ESTOP_CAUSES[s.cause]) return { ...meta, hint: `${ESTOP_CAUSES[s.cause]}. Check the robot, then release the E-stop` };
  return meta;
}

const TRANSPORT_STEPS = [
  { id: "navigating_to_rack", label: (t) => `Drive to ${t.from_name || "rack"}` },
  { id: "centering", label: () => "Align under rack" },
  { id: "lifting", label: () => "Lift rack" },
  { id: "navigating_to_dest", label: (t) => `Carry to ${t.to_name || "destination"}` },
  { id: "inserting", label: () => "Insert into slot" },
  { id: "dropping", label: () => "Lower rack" },
];

export function taskSteps(task) {
  if (!task) return [];
  if (task.type === "transport") {
    return TRANSPORT_STEPS.map((s) => ({ id: s.id, label: s.label(task) }));
  }
  return [{ id: "navigating", label: `Drive to ${task.to_name || "goal"}` }];
}

export function taskTitle(task) {
  if (!task) return "";
  if (task.type === "transport") return `${task.from_name || "?"} → ${task.to_name || "?"}`;
  return task.to_name || "Goal";
}

export function taskKind(task) {
  return task?.type === "transport" ? "Move rack" : "Go to";
}

export function allSites(locations) {
  const l = locations || {};
  return [
    ...(l.racks || []).map((r) => ({ ...r, kind: "rack" })),
    ...(l.drop_zones || []).map((z) => ({ ...z, kind: "zone" })),
  ];
}

export function siteState(site) {
  return site.state || (site.kind === "zone" ? "free" : "occupied");
}
