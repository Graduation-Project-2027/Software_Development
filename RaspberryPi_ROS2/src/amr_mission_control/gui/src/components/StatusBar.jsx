import { useEffect, useState, memo } from "react";
import { cx } from "../lib/cx";
import { fmt, rad2deg, num } from "../lib/geometry";
import { isTaskFault } from "../lib/mission";

const MOVING = new Set(["RUNNING", "NAVIGATING", "RETURNING", "LIFTING", "DROPPING"]);
const LATCH = { 1: "Latched by operator", 2: "Latched by button", 3: "Latched, link lost" };
const LIFT = ["Not homed", "Homing", null, "Moving", "Fault"];

/** What the robot's own stack light is showing: same rules as the body
 *  controller firmware (src/indicator.cpp) and the CAN bridge. */
function lampState(ms, online) {
  if (!online) return { red: "off", amber: "off", green: "off", text: "No robot" };
  if (ms.state === "ERROR" && !isTaskFault(ms)) return { red: "blink", amber: "off", green: "off", text: "E-stop latched" };
  if (ms.state === "ERROR") return { red: "on", amber: "off", green: "off", text: "Fault" };
  if (ms.state === "PAUSED" || ms.fleet?.held) return { red: "off", amber: "on", green: "off", text: "Paused" };
  if (MOVING.has(ms.state)) return { red: "off", amber: "blink", green: "on", text: "Moving" };
  return { red: "off", amber: "off", green: "on", text: "Idle" };
}

/** The robot's red / amber / green tower light, drawn as it is lit right now. */
export function StackLamp({ missionState, online }) {
  const s = lampState(missionState, online);
  return (
    <span className="lamps" role="img" aria-label={`Stack light: ${s.text}`}>
      {["red", "amber", "green"].map((c) => (
        <i key={c} className={cx("lamp", `lamp--${c}`, s[c] !== "off" && "is-lit", s[c] === "blink" && "is-blink")} />
      ))}
    </span>
  );
}

function Seg({ label, tone, children, grow }) {
  return (
    <div className={cx("sb__seg", tone && `sb__seg--${tone}`, grow && "sb__seg--grow")}>
      <span className="sb__label">{label}</span>
      <span className="sb__value num">{children}</span>
    </div>
  );
}

function Cells({ percent, low }) {
  const lit = Math.max(0, Math.min(10, Math.round(percent / 10)));
  return (
    <span className={cx("cells", low && "cells--low")} aria-hidden="true">
      {Array.from({ length: 10 }, (_, i) => <i key={i} className={i < lit ? "is-lit" : undefined} />)}
    </span>
  );
}

function Clock() {
  const [now, setNow] = useState(() => new Date());
  useEffect(() => {
    const id = setInterval(() => setNow(new Date()), 1000);
    return () => clearInterval(id);
  }, []);
  return <time className="sb__clock num" dateTime={now.toISOString()}>{now.toLocaleTimeString([], { hour12: false })}</time>;
}

/** Bottom strip of a robot console: link, stack, e-stop chain, drive power, battery, lift, pose. */
function StatusBar({ missionState: ms, online, status, mode, url, stack }) {
  const s = stack?.status;
  const link = mode === "preview" ? "Preview" : status === "connected" ? "Connected" : status === "connecting" ? "Connecting" : "Offline";
  const stackText = stack?.reachable === false ? "Launcher offline"
    : !s ? "Checking" : !s.running ? "Stopped" : s.starting ? "Starting" : s.mode === "mapping" ? "Mapping" : `Navigation, ${s.map}`;
  const safety = online ? ms.safety : null;
  const battery = online ? ms.battery : null;
  const lift = online ? ms.lift : null;
  const pose = ms.robot_pose;

  return (
    <footer className="sb" aria-label="Robot status">
      <Seg label="Link" tone={online ? "ok" : "off"}>
        <span title={mode === "preview" ? "Simulated robot in this browser" : `rosbridge ${url}`}>{link}</span>
      </Seg>
      <Seg label="Stack" tone={s?.running && !s.starting ? "ok" : "off"}>{stackText}</Seg>
      {safety && (
        <Seg label="E-stop" tone={!safety.online ? "off" : safety.active || safety.button ? "fault" : "ok"}>
          {!safety.online ? "Controller offline" : safety.button ? "Button pressed" : safety.active ? LATCH[safety.cause] || "Latched" : "Ready"}
        </Seg>
      )}
      {safety?.online && (
        <Seg label="Drive power" tone={safety.power ? "ok" : "fault"}>{safety.power ? "On" : "Cut"}</Seg>
      )}
      {battery && (
        <Seg label="Battery" tone={!battery.online ? "off" : battery.low ? "fault" : undefined}>
          {battery.online ? (
            <>
              <Cells percent={battery.percent} low={battery.low} />
              {battery.voltage.toFixed(1)} V<span className="sb__dim">{battery.percent}%</span>
            </>
          ) : "Drive board off"}
        </Seg>
      )}
      {lift && (
        <Seg label="Lift" tone={!lift.online ? "off" : lift.state === 4 ? "fault" : undefined}>
          {!lift.online ? "Offline" : LIFT[lift.state] || `${(lift.height * 1000).toFixed(0)} mm`}
        </Seg>
      )}
      <Seg label="Pose" grow>
        {online ? <>{fmt(pose.x)}, {fmt(pose.y)} m<span className="sb__dim">{fmt(rad2deg(num(pose.yaw)), 0)}°</span></> : "No data"}
      </Seg>
      <Clock />
    </footer>
  );
}

/** Bottom strip of the fleet workspace. */
export function FleetStatusBar({ fleet }) {
  const robots = fleet.state.robots.filter((r) => r.adopted);
  const linked = robots.filter((r) => r.linked);
  const holding = linked.filter((r) => r.hold).length;
  const faulted = linked.filter((r) => r.mission?.state === "ERROR").length;
  const layout = fleet.state.layout;
  const manager = fleet.preview ? "Preview" : fleet.status === "online" ? "Online" : fleet.status === "connecting" ? "Connecting" : "Offline";
  return (
    <footer className="sb" aria-label="Fleet status">
      <Seg label="Fleet manager" tone={fleet.preview || fleet.status === "online" ? "ok" : "off"}>{manager}</Seg>
      <Seg label="Robots" tone={linked.length ? "ok" : "off"}>{linked.length} of {robots.length} linked</Seg>
      <Seg label="Holding" tone={holding ? "caution" : undefined}>{holding}</Seg>
      <Seg label="Faults" tone={faulted ? "fault" : undefined}>{faulted}</Seg>
      <Seg label="Layout" grow>{layout?.map ? <>{layout.map}<span className="sb__dim">rev {layout.rev}</span></> : "No map"}</Seg>
      <Clock />
    </footer>
  );
}

export default memo(StatusBar);
