from __future__ import annotations

import math
import os
import subprocess
import sys
import time
from typing import Optional

try:
    import optuna
    optuna.logging.set_verbosity(optuna.logging.WARNING)
except ImportError:
    print("[AUTOTUNE] Optuna kurulu değil! → pip install optuna")
    sys.exit(1)

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
    from nav_msgs.msg import Odometry, Path
    from std_msgs.msg import String
except ImportError:
    print("[AUTOTUNE] rclpy bulunamadı — ROS 2 ortamı kaynalandı mı?")
    sys.exit(1)

WP1_ARRIVAL_RADIUS: float = 2.0

MISSION_MANAGER_CMD = (
    "bash -c 'source /opt/ros/humble/setup.bash && "
    "source /home/aliomacc/sti_usv/src/usv_sim/workspace_nav/install/setup.bash && "
    "ros2 run workspace_nav mission_manager --ros-args "
    "-p use_sim_time:=true "
    "-p waypoints_file:=/home/aliomacc/sti_usv/src/usv_sim/workspace_nav/json/waypoints.json "
    "-p kamikaze_wp_id:=WP5 "
    "-p kamikaze_trigger_dist:=5.0 "
    "-p red_class_id:=0 "
    "-p green_class_id:=1 "
    "-p kp_yaw:=1.2 "
    "-p base_speed:=1.5 "
    "-p kamikaze_lost_timeout:=3.0 > /tmp/mission_autotune.log 2>&1'"
)

MISSION_MANAGER_WAIT_SEC: float = 8.0
TRIAL_TIMEOUT_SEC: float = 60.0

COLLISION_MARGIN_MIN: float = 0.00
COLLISION_MARGIN_MAX: float = 0.15
CRITICAL_WEIGHT_MIN: float = 8.0
CRITICAL_WEIGHT_MAX: float = 25.0

N_TRIALS: int = 30

def reset_mission_manager() -> None:

    print("[RESET] mission_manager öldürülüyor …")
    os.system("pkill -9 -f 'mission_manager' 2>/dev/null")
    time.sleep(2.0)

def verify_nav2_alive() -> bool:

    result = subprocess.run(
        "bash -c 'source /opt/ros/humble/setup.bash && "
        "source /home/aliomacc/sti_usv/src/usv_sim/workspace_nav/install/setup.bash 2>/dev/null; "
        "ros2 node list 2>/dev/null'",
        shell=True, capture_output=True, text=True, timeout=10.0
    )
    alive = "/controller_server" in result.stdout
    if not alive:
        print("[ERROR] /controller_server bulunamadı!")
        print("        start_all.sh mola ile sistemi başlatın ve tekrar deneyin.")
    return alive

def inject_params(collision_margin: float, critical_weight: float) -> bool:

    base_cmd = "ros2 param set /controller_server"

    params = {
        "FollowPath.ObstaclesCritic.collision_margin_distance": str(round(collision_margin, 4)),
        "FollowPath.ObstaclesCritic.critical_weight":           str(round(critical_weight, 2)),
    }

    for param_name, value in params.items():
        cmd = f"{base_cmd} {param_name} {value}"
        for attempt in range(3):
            try:
                result = subprocess.run(
                    cmd, shell=True, capture_output=True, text=True, timeout=5.0
                )
                if result.returncode == 0:
                    print(f"[PARAM] ✓ {param_name} = {value}")
                    break
                print(f"[PARAM] ✗ Deneme {attempt+1}/3: {result.stderr.strip()[:80]}")
            except subprocess.TimeoutExpired:
                print(f"[PARAM] ✗ Deneme {attempt+1}/3: timeout")
            time.sleep(2.0)
        else:
            print(f"[PARAM] ✗ {param_name} 3 denemede set edilemedi → score=0")
            return False

    return True

class WatchdogObserver(Node):

    def __init__(self):
        super().__init__('autotune_watchdog')

        self.arrived: bool              = False
        self.aborted: bool              = False
        self.arrival_time: Optional[float] = None
        self._start_time: float         = time.monotonic()

        self._wp1_x: Optional[float]    = None
        self._wp1_y: Optional[float]    = None
        self._wp1_discovered: bool      = False

        plan_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Path,     '/plan',               self._plan_cb, plan_qos)
        self.create_subscription(Odometry, '/odometry/filtered',  self._odom_cb, 10)
        self.create_subscription(String,   '/mission_state',      self._state_cb, 10)

    def _plan_cb(self, msg: Path) -> None:

        if self._wp1_discovered or not msg.poses:
            return
        last_pose    = msg.poses[-1].pose
        self._wp1_x  = last_pose.position.x
        self._wp1_y  = last_pose.position.y
        self._wp1_discovered = True
        self.get_logger().info(
            f'[WATCHDOG] 📍 WP1 keşfedildi: '
            f'x={self._wp1_x:.2f} y={self._wp1_y:.2f} (map frame)'
        )

    def _odom_cb(self, msg: Odometry) -> None:
        if self.arrived or self.aborted or not self._wp1_discovered:
            return
        x    = msg.pose.pose.position.x
        y    = msg.pose.pose.position.y
        dist = math.hypot(self._wp1_x - x, self._wp1_y - y)
        if dist <= WP1_ARRIVAL_RADIUS:
            self.arrived      = True
            self.arrival_time = time.monotonic() - self._start_time
            self.get_logger().info(
                f'[WATCHDOG] ✅ WP1\'e ulaşıldı! dist={dist:.2f} m | '
                f'süre={self.arrival_time:.1f} s'
            )

    def _state_cb(self, msg: String) -> None:
        if msg.data in ('ABORTED', 'ABORT', 'STATUS_6'):
            self.aborted = True
            elapsed = time.monotonic() - self._start_time
            self.get_logger().warn(
                f'[WATCHDOG] ❌ Mission state: {msg.data} @ t={elapsed:.1f} s'
            )

def run_watchdog(timeout: float) -> tuple[bool, float]:

    rclpy.init(args=None)
    observer = WatchdogObserver()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(observer)

    deadline = time.monotonic() + timeout
    _warned  = False
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.1)
        if observer.arrived or observer.aborted:
            break

        elapsed_so_far = time.monotonic() - (deadline - timeout)
        if elapsed_so_far > 15.0 and not observer._wp1_discovered and not _warned:
            _warned = True
            print(f"[WATCHDOG] ⚠ {elapsed_so_far:.0f}s geçti, /plan henüz alınmadı. "
                  f"mission_manager goal gönderiyor mu?")

    arrived = observer.arrived
    elapsed = observer.arrival_time if observer.arrival_time else timeout

    if not arrived:
        if observer.aborted:
            print("[WATCHDOG] Neden: MISSION ABORT")
        elif not observer._wp1_discovered:
            print("[WATCHDOG] Neden: /plan hiç gelmedi — mission_manager Nav2 goal gönderemiyor olabilir")
        else:
            print(f"[WATCHDOG] Neden: WP1'e yetişilemedi. "
                  f"WP1=({observer._wp1_x:.2f},{observer._wp1_y:.2f}) "
                  f"Timeout={timeout:.0f}s")

    executor.shutdown()
    observer.destroy_node()
    rclpy.shutdown()
    return arrived, elapsed

def objective(trial: optuna.Trial) -> float:

    collision_margin = trial.suggest_float(
        "collision_margin_distance", COLLISION_MARGIN_MIN, COLLISION_MARGIN_MAX, step=0.01
    )
    critical_weight = trial.suggest_float(
        "critical_weight", CRITICAL_WEIGHT_MIN, CRITICAL_WEIGHT_MAX, step=0.5
    )

    print(f"\n{'='*60}")
    print(f"[TRIAL {trial.number}] collision_margin={collision_margin:.3f}  "
          f"critical_weight={critical_weight:.1f}")
    print('='*60)

    mission_proc = None
    score        = 0.0

    try:

        if not verify_nav2_alive():
            print("[TRIAL] Nav2 çalışmıyor → score=0 (start_all.sh çalıştırın)")
            return 0.0

        reset_mission_manager()

        if not inject_params(collision_margin, critical_weight):
            return 0.0

        print("[LAUNCH] mission_manager başlatılıyor …")
        mission_proc = subprocess.Popen(
            MISSION_MANAGER_CMD,
            shell=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid
        )
        print(f"[LAUNCH] GPS dönüşümü bekleniyor ({MISSION_MANAGER_WAIT_SEC:.0f} s) …")
        time.sleep(MISSION_MANAGER_WAIT_SEC)

        arrived, elapsed = run_watchdog(timeout=TRIAL_TIMEOUT_SEC)

        if arrived:
            score = TRIAL_TIMEOUT_SEC - elapsed
            print(f"[TRIAL] ✅ BAŞARILI! süre={elapsed:.1f} s → score={score:.2f}")
        else:
            score = 0.0
            print(f"[TRIAL] ❌ BAŞARISIZ (timeout/abort) → score=0.0")

    except KeyboardInterrupt:
        print("\n[AUTOTUNE] Kullanıcı tarafından durduruldu.")
        raise

    except Exception as exc:
        print(f"[TRIAL] ✗ İstisna: {exc}")
        score = 0.0

    finally:
        if mission_proc and mission_proc.poll() is None:
            try:
                os.killpg(os.getpgid(mission_proc.pid), 9)
            except Exception:
                pass

    return score

def print_report(study: optuna.Study) -> None:
    best = study.best_trial
    print("\n" + "╔" + "═" * 56 + "╗")
    print("║  OPTUNA OPTİMİZASYON RAPORU                            ║")
    print("╠" + "═" * 56 + "╣")
    print(f"║  Toplam deneme      : {len(study.trials):<33}║")
    print(f"║  En iyi skor        : {best.value:<33.2f}║")
    print(f"║  collision_margin   : {best.params['collision_margin_distance']:<33.4f}║")
    print(f"║  critical_weight    : {best.params['critical_weight']:<33.2f}║")
    print("╚" + "═" * 56 + "╝")

    try:
        import csv
        csv_path = "/tmp/autotune_results.csv"
        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["trial", "collision_margin", "critical_weight", "score"])
            for t in study.trials:
                if t.state == optuna.trial.TrialState.COMPLETE:
                    writer.writerow([
                        t.number,
                        t.params["collision_margin_distance"],
                        t.params["critical_weight"],
                        t.value,
                    ])
        print(f"\n[REPORT] CSV → {csv_path}")
    except Exception as e:
        print(f"[REPORT] CSV hatası: {e}")

def main() -> None:
    print("╔" + "═" * 58 + "╗")
    print("║  STI USV — MPPI Otonom Parametre Avcısı (Optuna)   ║")
    print(f"║  Deneme sayısı: {N_TRIALS:<42}║")
    print(f"║  WP1 hedef: /plan topiğinden otomatik keşfedilir      ║")
    print(f"║  Timeout/deneme: {TRIAL_TIMEOUT_SEC:.0f} s{'':>38}║")
    print("╚" + "═" * 58 + "╝\n")
    print("⚠️  start_all.sh mola ile sistemi önce başlatın!\n")

    if not verify_nav2_alive():
        print("[AUTOTUNE] Nav2 çalışmıyor, çıkılıyor.")
        sys.exit(1)

    print("[AUTOTUNE] ✓ Nav2 sağlıklı — optimizasyon başlıyor\n")

    db_path = "sqlite:////tmp/autotune_mppi.db"
    study = optuna.create_study(
        study_name="usv_mppi_tune",
        direction="maximize",
        storage=db_path,
        load_if_exists=True,
        sampler=optuna.samplers.TPESampler(seed=42),
    )

    print(f"[OPTUNA] DB: {db_path}")
    print(f"[OPTUNA] Mevcut tamamlanmış deneme: {len(study.trials)}\n")

    try:
        study.optimize(objective, n_trials=N_TRIALS, catch=(Exception,))
    except KeyboardInterrupt:
        print("\n[AUTOTUNE] Ctrl+C — mevcut sonuçlarla rapor üretiliyor …")
    finally:
        if study.best_trial:
            print_report(study)
        else:
            print("[AUTOTUNE] Hiç başarılı deneme yok.")

if __name__ == "__main__":
    main()
