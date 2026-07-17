 
import argparse
import math
from pathlib import Path
 
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
 
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
 
 
# --------------------------------------------------------------------------
# Config -- keep these in sync with localization.launch.py
# --------------------------------------------------------------------------
 
CAMERA_PITCH = 0.611          # base_link -> realsense_link, rad
GRAVITY = 9.805               # Burlington, VT
EXPECTED_IMU_FRAME = "realsense_optical"
EXPECTED_GPS_FRAME = "gps_link"
EXPECTED_ODOM_FRAME = ("odom", "base_link")
 
FIX_STATUS = {-1: "NO FIX", 0: "unaugmented", 1: "SBAS", 2: "GBAS / RTK"}
 
 
# --------------------------------------------------------------------------
# Frame projection
# --------------------------------------------------------------------------
 
def base_z_in_optical(pitch: float) -> np.ndarray:
    """base_link's z-axis (the yaw axis) expressed in realsense_optical."""
    return np.array([0.0, -math.cos(pitch), -math.sin(pitch)])
 
 
def project_yaw_rate(imu: pd.DataFrame, pitch: float) -> np.ndarray:
    """Optical-frame gyro -> scalar yaw rate about base_link z, rad/s."""
    axis = base_z_in_optical(pitch)
    return imu[["wx", "wy", "wz"]].to_numpy() @ axis
 
 
def pitch_from_accel(ay: float, az: float) -> float:
    """Recover the camera's true pitch from the at-rest gravity vector.
    At rest: ay = -g*cos(p), az = -g*sin(p)  ->  p = atan2(-az, -ay)."""
    return math.atan2(-az, -ay)
 
 
# --------------------------------------------------------------------------
# Reading
# --------------------------------------------------------------------------
 
def stamp_to_sec(stamp) -> float:
    """Sensor capture time. Use this, not bag receive time -- it puts topics
    of wildly different rates on one honest axis."""
    return stamp.sec + stamp.nanosec * 1e-9
 
 
def read_topics(bagpath: Path) -> dict:
    imu, odom, fix, cmd = [], [], [], []
    found = set()
 
    # Foxy bags carry no embedded type definitions (that started in Humble),
    # so hand rosbags the Foxy typestore explicitly.
    typestore = get_typestore(Stores.ROS2_FOXY)
 
    with AnyReader([bagpath], default_typestore=typestore) as reader:
        for conn in reader.connections:
            found.add(conn.topic)
 
        for conn, bag_t, raw in reader.messages():
            topic = conn.topic
 
            if topic == "/imu/data":
                m = reader.deserialize(raw, conn.msgtype)
                imu.append({
                    "t": stamp_to_sec(m.header.stamp),
                    "t_bag": bag_t * 1e-9,
                    "frame": m.header.frame_id,
                    "wx": m.angular_velocity.x,
                    "wy": m.angular_velocity.y,
                    "wz": m.angular_velocity.z,
                    "ax": m.linear_acceleration.x,
                    "ay": m.linear_acceleration.y,
                    "az": m.linear_acceleration.z,
                })
 
            elif topic == "/odom":
                m = reader.deserialize(raw, conn.msgtype)
                odom.append({
                    "t": stamp_to_sec(m.header.stamp),
                    "frame": m.header.frame_id,
                    "child": m.child_frame_id,
                    "vx": m.twist.twist.linear.x,
                    "vyaw": m.twist.twist.angular.z,
                    "cov_vx": m.twist.covariance[0],
                    "cov_vyaw": m.twist.covariance[35],
                    "cov_x": m.pose.covariance[0],
                })
 
            elif topic == "/fix":
                m = reader.deserialize(raw, conn.msgtype)
                fix.append({
                    "t": stamp_to_sec(m.header.stamp),
                    "frame": m.header.frame_id,
                    "lat": m.latitude,
                    "lon": m.longitude,
                    "alt": m.altitude,
                    "status": m.status.status,
                    "cov_x": m.position_covariance[0],
                    "cov_y": m.position_covariance[4],
                })
 
            elif topic == "/cmd_vel":
                m = reader.deserialize(raw, conn.msgtype)
                cmd.append({
                    "t": bag_t * 1e-9,   # Twist has no header
                    "vx": m.linear.x,
                    "vyaw": m.angular.z,
                })
 
    return {
        "imu": pd.DataFrame(imu),
        "odom": pd.DataFrame(odom),
        "fix": pd.DataFrame(fix),
        "cmd": pd.DataFrame(cmd),
        "topics_in_bag": found,
    }
 
 
# --------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------
 
def rate_report(df: pd.DataFrame, name: str):
    if df.empty:
        print(f"  {name:<12} EMPTY")
        return
    t = df["t"].to_numpy()
    dt = np.diff(t)
    dt = dt[dt > 0]
    if len(dt) == 0:
        print(f"  {name:<12} {len(df)} msgs, timestamps do not advance (!)")
        return
    print(f"  {name:<12} {len(df):>7} msgs | {t[-1]-t[0]:6.1f} s | "
          f"mean {1/dt.mean():6.1f} Hz | jitter {dt.std()*1000:5.1f} ms | "
          f"worst gap {dt.max()*1000:6.1f} ms")
 
 
def check_frame(actual: str, expected: str, what: str):
    ok = actual == expected
    mark = "OK " if ok else "!! "
    print(f"  {mark}{what} frame_id: {actual!r}"
          + ("" if ok else f"   EXPECTED {expected!r}"))
    if not ok:
        print(f"      ^ this frame must exist in TF or robot_localization")
        print(f"        silently drops every message from this sensor.")
 
 
def latlon_to_local_m(lat, lon):
    """Equirectangular about the first fix. Sub-cm over a test-run footprint."""
    lat0, lon0 = lat[0], lon[0]
    R = 6378137.0
    x = np.radians(lon - lon0) * R * math.cos(math.radians(lat0))   # east
    y = np.radians(lat - lat0) * R                                   # north
    return x, y
 
 
def integrate_odom(odom: pd.DataFrame):
    """Dead-reckon vx/vyaw. This is what the robot THINKS it did -- if it was
    stuck or slipping, this reports motion that never happened."""
    t = odom["t"].to_numpy()
    vx = odom["vx"].to_numpy()
    vyaw = odom["vyaw"].to_numpy()
    dt = np.clip(np.diff(t, prepend=t[0]), 0, 1.0)
    yaw = np.cumsum(vyaw * dt)
    x = np.cumsum(vx * np.cos(yaw) * dt)
    y = np.cumsum(vx * np.sin(yaw) * dt)
    return x, y, yaw
 
 
# --------------------------------------------------------------------------
# Reports
# --------------------------------------------------------------------------
 
def report_imu(imu: pd.DataFrame, pitch: float, static: bool):
    if imu.empty:
        return
    print("\n=== IMU (/imu/data) ===")
    check_frame(imu["frame"].iloc[0], EXPECTED_IMU_FRAME, "IMU")
 
    skew = (imu["t_bag"] - imu["t"]).to_numpy()
    print(f"  header-vs-bagtime skew: mean {skew.mean()*1000:.1f} ms, "
          f"std {skew.std()*1000:.1f} ms")
    if skew.std() > 0.002:
        print("      ^ still stamping at publish time. frame.get_timestamp()")
        print("        (hardware capture time) would tighten this.")
 
    # ---- raw axes, for reference only -----------------------------------
    print("\n  Raw gyro in realsense_optical (rad/s) -- NOT robot axes:")
    for ax in ("wx", "wy", "wz"):
        v = imu[ax]
        print(f"    {ax}: mean {v.mean():+.6f}  std {v.std():.6f}")
 
    # ---- the number that actually matters -------------------------------
    wz_base = project_yaw_rate(imu, pitch)
    bias = wz_base.mean()
    var = wz_base.var()
 
    print(f"\n  >>> YAW RATE about base_link z (pitch={pitch:.3f} rad "
          f"= {math.degrees(pitch):.1f} deg) <<<")
    print(f"    bias : {bias:+.6f} rad/s = {math.degrees(bias):+.4f} deg/s "
          f"= {math.degrees(bias)*60:+.2f} deg/min")
    print(f"    noise: std {wz_base.std():.6f} rad/s | var {var:.3e}")
 
    if static:
        drift_10min = math.degrees(bias) * 600
        print(f"\n    --> unmitigated drift over a 10-min mission: "
              f"{drift_10min:+.1f} deg")
        if abs(drift_10min) > 5:
            print("        robot_localization has NO gyro-bias state -- it cannot")
            print("        estimate this away. Subtract it in realsense_node:")
            print("        average ~3 s of gyro at startup while stationary,")
            print("        then subtract from every sample before publishing.")
        else:
            print("        acceptable.")
        print(f"\n    --> imu0 angular_velocity_covariance (yaw): {var:.3e}")
 
    # ---- accel vs. TF ----------------------------------------------------
    ax_m, ay_m, az_m = imu.ax.mean(), imu.ay.mean(), imu.az.mean()
    mag = float(np.sqrt(imu.ax**2 + imu.ay**2 + imu.az**2).mean())
 
    exp_ay = -GRAVITY * math.cos(pitch)
    exp_az = -GRAVITY * math.sin(pitch)
    meas_pitch = pitch_from_accel(ay_m, az_m)
 
    print("\n  Accel (m/s^2), at rest should read gravity:")
    print(f"    measured: ax {ax_m:+.4f}  ay {ay_m:+.4f}  az {az_m:+.4f}")
    print(f"    expected: ax  0.0000  ay {exp_ay:+.4f}  az {exp_az:+.4f}")
    print(f"    |a| {mag:.3f}  (expect {GRAVITY:.3f}; "
          f"off by {abs(mag-GRAVITY):.3f} = {100*abs(mag-GRAVITY)/GRAVITY:.1f}%)")
 
    err_deg = math.degrees(meas_pitch - pitch)
    print(f"\n    pitch implied by accel: {math.degrees(meas_pitch):.1f} deg")
    print(f"    pitch declared in TF   : {math.degrees(pitch):.1f} deg")
    print(f"    DISAGREEMENT           : {err_deg:+.1f} deg")
    if abs(err_deg) > 2:
        print("      ^ either the camera is not physically mounted at the TF")
        print("        angle, or the D435i accel is uncalibrated (common --")
        print("        many units ship with nothing in flash).")
        print("      TEST: set the robot on a surface verified level with a")
        print("        bubble level. If accel reads (0, -9.81, 0) the sensor")
        print("        is honest and your MOUNT angle is wrong -> fix the TF")
        print(f"        (--pitch {meas_pitch:.3f}). If it reads skewed, run")
        print("        Intel's rs-imu-calibration.py (accel only; it does NOT")
        print("        calibrate the gyro, so you still owe the bias fix above).")
 
 
def report_odom(odom: pd.DataFrame, cmd: pd.DataFrame):
    if odom.empty:
        return
    print("\n=== Wheel odometry (/odom) ===")
    f, c = odom["frame"].iloc[0], odom["child"].iloc[0]
    ok = (f, c) == EXPECTED_ODOM_FRAME
    print(f"  {'OK ' if ok else '!! '}frame_id {f!r} / child_frame_id {c!r}"
          + ("" if ok else f"   EXPECTED {EXPECTED_ODOM_FRAME}"))
 
    print(f"  covariance -- pose x {odom['cov_x'].iloc[0]:.1e} (want ~1e6) | "
          f"twist vx {odom['cov_vx'].iloc[0]:.3f} | vyaw {odom['cov_vyaw'].iloc[0]:.3f}")
 
    print(f"\n  vx   : mean {odom.vx.mean():+.4f}  std {odom.vx.std():.4f}  "
          f"max |{odom.vx.abs().max():.3f}| m/s")
    print(f"  vyaw : mean {odom.vyaw.mean():+.4f}  std {odom.vyaw.std():.4f}  "
          f"max |{odom.vyaw.abs().max():.3f}| rad/s")
 
    if cmd.empty and odom.vx.abs().max() == 0:
        print("\n  /cmd_vel empty and /odom steady at zero -> wheel_odom_node is")
        print("  publishing on a TIMER, not from the cmd_vel callback. Correct:")
        print("  the EKF gets fed even when the robot is commanded nothing.")
 
    x, y, yaw = integrate_odom(odom)
    print(f"\n  dead-reckoned: ({x[-1]:+.2f}, {y[-1]:+.2f}) m, "
          f"net {math.hypot(x[-1], y[-1]):.2f} m | "
          f"yaw {math.degrees(yaw[-1]):+.1f} deg")
 
 
def report_fix(fix: pd.DataFrame, static: bool):
    print("\n=== GPS (/fix) ===")
    if fix.empty:
        print("  EMPTY")
        return
    check_frame(fix["frame"].iloc[0], EXPECTED_GPS_FRAME, "GPS")
 
    counts = fix["status"].value_counts()
    print("  fix quality:")
    for s, n in counts.items():
        print(f"    {FIX_STATUS.get(s, s):<14} {n:>6} ({100*n/len(fix):5.1f}%)")
    if 2 not in counts.index:
        print("    ^ NO RTK-fixed samples. This is plain GNSS -- metre-scale.")
        print("      Corrections (NTRIP / local base) are not reaching the rover.")
 
    x, y = latlon_to_local_m(fix["lat"].to_numpy(), fix["lon"].to_numpy())
    rms = float(np.sqrt(((x-x.mean())**2 + (y-y.mean())**2).mean()))
    print(f"\n  scatter (m, ENU): east std {x.std():.3f} | "
          f"north std {y.std():.3f} | 2D rms {rms:.3f}")
    if static:
        print("    ^ robot was still, so this spread is GPS PRECISION.")
        print("      Precision != accuracy: a short window hides the slow")
        print("      multipath/iono bias, so absolute error is worse than this.")
    print(f"  driver-reported cov: {fix['cov_x'].iloc[0]:.2f} m^2 "
          f"(std {math.sqrt(fix['cov_x'].iloc[0]):.2f} m) -- usually a hardcoded")
    print("      default. Conservative, which is currently protecting the EKF.")
    print(f"  altitude: mean {fix.alt.mean():.2f} m, std {fix.alt.std():.2f} m")
 
 
# --------------------------------------------------------------------------
# Plots
# --------------------------------------------------------------------------
 
def make_plots(d: dict, outdir: Path, pitch: float, static: bool):
    imu, odom, fix, cmd = d["imu"], d["odom"], d["fix"], d["cmd"]
    frames = [df for df in (imu, odom, fix) if not df.empty]
    if not frames:
        return
    t0 = min(df["t"].min() for df in frames)
 
    fig, axes = plt.subplots(4, 1, figsize=(13, 13), sharex=True)
 
    if not imu.empty:
        wz_base = project_yaw_rate(imu, pitch)
        axes[0].plot(imu.t - t0, imu.wz, lw=0.3, alpha=0.5,
                     label="raw wz (optical, NOT yaw)")
        axes[0].plot(imu.t - t0, wz_base, lw=0.4,
                     label="yaw rate about base_link z")
        axes[0].axhline(wz_base.mean(), color="r", ls="--", lw=1.2,
                        label=f"bias {math.degrees(wz_base.mean()):+.3f} deg/s")
        axes[0].set_ylabel("yaw rate\n(rad/s)")
        axes[0].legend(loc="upper right", fontsize=8)
        axes[0].set_title("Projected yaw rate -- the raw wz trace is the wrong axis")
 
    if not odom.empty:
        axes[1].plot(odom.t - t0, odom.vx, lw=0.8, label="odom vx")
        if not cmd.empty:
            axes[1].plot(cmd.t - t0, cmd.vx, lw=0.8, ls="--", label="cmd_vel vx")
        axes[1].set_ylabel("vx (m/s)")
        axes[1].legend(loc="upper right", fontsize=8)
 
        axes[2].plot(odom.t - t0, odom.vyaw, lw=0.8, label="odom vyaw")
        if not imu.empty:
            axes[2].plot(imu.t - t0, project_yaw_rate(imu, pitch), lw=0.3,
                         alpha=0.6, label="imu yaw rate (projected)")
        axes[2].set_ylabel("yaw rate\n(rad/s)")
        axes[2].legend(loc="upper right", fontsize=8)
        axes[2].set_title("odom vs IMU yaw rate -- should agree during turns")
 
    if not fix.empty:
        gx, gy = latlon_to_local_m(fix.lat.to_numpy(), fix.lon.to_numpy())
        axes[3].plot(fix.t - t0, gx, lw=1, label="east")
        axes[3].plot(fix.t - t0, gy, lw=1, label="north")
        axes[3].set_ylabel("GPS (m)")
        axes[3].legend(loc="upper right", fontsize=8)
    axes[3].set_xlabel("time since start (s)")
 
    fig.tight_layout()
    fig.savefig(outdir / "timeseries.png", dpi=130)
    print(f"  wrote {outdir/'timeseries.png'}")
 
    # Integrated yaw drift -- the headline number for a static run.
    if not imu.empty and static:
        wz_base = project_yaw_rate(imu, pitch)
        t = imu.t.to_numpy()
        dt = np.clip(np.diff(t, prepend=t[0]), 0, 1.0)
        yaw = np.degrees(np.cumsum(wz_base * dt))
        yaw_c = np.degrees(np.cumsum((wz_base - wz_base.mean()) * dt))
 
        fig2, ax = plt.subplots(figsize=(11, 5))
        ax.plot(t - t0, yaw, lw=1.2, label="raw gyro integration")
        ax.plot(t - t0, yaw_c, lw=1.2, label="bias-corrected")
        ax.axhline(0, color="k", lw=0.5)
        ax.set_xlabel("time (s)"); ax.set_ylabel("integrated yaw (deg)")
        ax.set_title("Yaw drift with the robot STATIONARY\n"
                     "(truth is a flat line at 0 -- the gap is the bias)")
        ax.legend(); ax.grid(alpha=0.3)
        fig2.tight_layout()
        fig2.savefig(outdir / "yaw_drift.png", dpi=130)
        print(f"  wrote {outdir/'yaw_drift.png'}")
 
    if not static and not odom.empty:
        fig3, ax = plt.subplots(figsize=(8, 8))
        ox, oy, _ = integrate_odom(odom)
        ax.plot(ox, oy, lw=1.5, label="odom (dead-reckoned)")
        ax.plot(ox[0], oy[0], "go", ms=9, label="start")
        ax.plot(ox[-1], oy[-1], "ro", ms=9, label="odom end")
        if not fix.empty:
            gx, gy = latlon_to_local_m(fix.lat.to_numpy(), fix.lon.to_numpy())
            ax.plot(gx, gy, lw=1.5, alpha=0.8, label="GPS")
        ax.set_aspect("equal"); ax.grid(alpha=0.3)
        ax.set_xlabel("east (m)"); ax.set_ylabel("north (m)")
        ax.set_title("Dead-reckoned odom vs GPS\n"
                     "(different frames -- expect a rotation until navsat "
                     "heading init works; compare SHAPES)")
        ax.legend()
        fig3.tight_layout()
        fig3.savefig(outdir / "paths.png", dpi=130)
        print(f"  wrote {outdir/'paths.png'}")
 
    if not fix.empty:
        fig4, ax = plt.subplots(figsize=(7, 7))
        gx, gy = latlon_to_local_m(fix.lat.to_numpy(), fix.lon.to_numpy())
        sc = ax.scatter(gx, gy, c=fix.t - fix.t.iloc[0], s=12, cmap="viridis")
        ax.set_aspect("equal"); ax.grid(alpha=0.3)
        ax.set_xlabel("east (m)"); ax.set_ylabel("north (m)")
        ax.set_title("GPS scatter (colour = time)")
        fig4.colorbar(sc, label="s")
        fig4.tight_layout()
        fig4.savefig(outdir / "gps_scatter.png", dpi=130)
        print(f"  wrote {outdir/'gps_scatter.png'}")
 
 
# --------------------------------------------------------------------------
 
def main():
    p = argparse.ArgumentParser()
    p.add_argument("bag", type=Path, help="path to the bag DIRECTORY")
    p.add_argument("--static", action="store_true",
                   help="robot was stationary: enable bias/noise conclusions")
    p.add_argument("--pitch", type=float, default=CAMERA_PITCH,
                   help=f"base_link->realsense_link pitch, rad "
                        f"(default {CAMERA_PITCH})")
    p.add_argument("--csv", action="store_true", help="also dump CSVs")
    p.add_argument("-o", "--outdir", type=Path, default=None)
    args = p.parse_args()
 
    outdir = args.outdir or (args.bag.parent / f"{args.bag.name}_analysis")
    outdir.mkdir(parents=True, exist_ok=True)
 
    print(f"Reading {args.bag} ...")
    d = read_topics(args.bag)
 
    print(f"\nTopics present: {sorted(d['topics_in_bag'])}")
    print("\n=== Rates (from header stamps) ===")
    rate_report(d["imu"], "/imu/data")
    rate_report(d["odom"], "/odom")
    rate_report(d["fix"], "/fix")
 
    report_imu(d["imu"], args.pitch, args.static)
    report_odom(d["odom"], d["cmd"])
    report_fix(d["fix"], args.static)
 
    print("\n=== Plots ===")
    make_plots(d, outdir, args.pitch, args.static)
 
    if args.csv:
        for name in ("imu", "odom", "fix", "cmd"):
            if not d[name].empty:
                d[name].to_csv(outdir / f"{name}.csv", index=False)
        print(f"  wrote CSVs to {outdir}")
 
    print(f"\nDone -> {outdir}")
 
 
if __name__ == "__main__":
    main()