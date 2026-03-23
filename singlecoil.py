"""
Magnetic Sites Sequencer (32 coils)
✅ Travel sequencing uses SINGLE COIL (one coil ON at a time)
✅ Splitting uses DOUBLE COIL (2 adjacent coils) ONLY when you trigger Split
✅ Separate timing controls:
   - SEQUENCE delay (used for normal travel / sequence steps)
   - SPLIT delay (used only inside split patterns)
✅ When a split block is appended (Split Here / Split Sweep), those steps are tagged as SPLIT
   so the runner automatically uses the split delay for those steps.

Hardware:
- Modbus RTU, 32 registers (0..31) controlling coils 1..32 via FC=6
- Change cell_to_register if your device uses a different mapping.

NOTE:
- Motion planning assumes coil order is 1..32 (1D).
  If your physical layout is 4x8 neighbors, tell me the neighbor mapping.
"""

import tkinter as tk
from tkinter import messagebox
from tkinter import ttk
import threading
import time

# ------------------------
# SETTINGS
# ------------------------
NUM_CELLS = 32

# Start in TEST MODE so you can test without hardware.
TEST_MODE = False

DEFAULT_COM_PORT = "COM4"
SLAVE_ID = 1
BAUDRATE = 9600
TIMEOUT = 1

# Small overlap used when switching travel coils to avoid ALL-OFF
TRAVEL_SWITCH_OVERLAP_SEC = 0.02

# Step type tags
STEP_SEQ = "SEQ"     # normal sequence delay
STEP_SPLIT = "SPLIT" # split delay

# ------------------------
# Hardware control (shared)
# ------------------------
instrument = None
connected = False
lock = threading.Lock()


def list_com_ports():
    """Return a list like ['COM3','COM4', ...]."""
    try:
        from serial.tools import list_ports
        return [p.device for p in list_ports.comports()]
    except Exception:
        return []


def disconnect_device():
    """Close serial handle if it exists (prevents COM port from staying locked)."""
    global instrument, connected
    connected = False
    try:
        if instrument is not None and hasattr(instrument, "serial") and instrument.serial is not None:
            try:
                instrument.serial.close()
            except Exception:
                pass
    finally:
        instrument = None


def connect_device(port_name: str):
    """Connect to the Modbus device."""
    global instrument, connected

    if TEST_MODE:
        connected = True
        return True

    disconnect_device()

    try:
        import minimalmodbus
        instrument = minimalmodbus.Instrument(port_name, SLAVE_ID)
        instrument.mode = minimalmodbus.MODE_RTU
        instrument.serial.baudrate = BAUDRATE
        instrument.serial.bytesize = 8
        instrument.serial.parity = minimalmodbus.serial.PARITY_NONE
        instrument.serial.stopbits = 1
        instrument.serial.timeout = TIMEOUT

        # If the port sometimes "locks up", uncomment this (slower but safer):
        # instrument.close_port_after_each_call = True

        connected = True
        return True
    except Exception as e:
        connected = False
        instrument = None
        messagebox.showerror("Connect failed", str(e))
        return False


def cell_to_register(cell: int) -> int:
    """Map coil cell 1..32 -> register address 0..31."""
    return cell - 1


def hw_open_cell(cell: int):
    if not connected:
        raise RuntimeError("Not connected (click Connect first)")

    if TEST_MODE:
        print(f"[TEST] OPEN cell {cell}")
        return

    with lock:
        instrument.write_register(cell_to_register(cell), 1, functioncode=6)


def hw_close_cell(cell: int):
    if not connected:
        raise RuntimeError("Not connected (click Connect first)")

    if TEST_MODE:
        print(f"[TEST] CLOSE cell {cell}")
        return

    with lock:
        instrument.write_register(cell_to_register(cell), 0, functioncode=6)


def hw_close_all():
    for c in range(1, NUM_CELLS + 1):
        hw_close_cell(c)


def hw_open_cells_no_all_off(new_cells_iterable, overlap_sec=0.02):
    """
    Update coils WITHOUT EVER going through an all-off state:
      1) OPEN coils that need to be ON (but are currently OFF)
      2) tiny overlap delay
      3) CLOSE coils that need to be OFF (but are currently ON)

    Keeps a software-side state of currently-on coils:
      hw_open_cells_no_all_off.current_on
    """
    new_set = set(new_cells_iterable)
    cur_set = set(hw_open_cells_no_all_off.current_on)

    # If not connected, just update the state (so UI logic still works).
    if not connected:
        hw_open_cells_no_all_off.current_on = set(new_set)
        return

    to_open = sorted(new_set - cur_set)
    to_close = sorted(cur_set - new_set)

    for c in to_open:
        hw_open_cell(c)

    if overlap_sec and (to_open and to_close):
        time.sleep(overlap_sec)

    for c in to_close:
        hw_close_cell(c)

    hw_open_cells_no_all_off.current_on = set(new_set)


hw_open_cells_no_all_off.current_on = set()


def hw_set_single_travel_cell(next_cell: int, overlap_sec: float = TRAVEL_SWITCH_OVERLAP_SEC):
    """
    Travel mode: end state is SINGLE coil ON.
    During a switch we briefly allow 2 coils ON (prev + next) to avoid ALL-OFF.
    """
    next_cell = int(next_cell)
    if not (1 <= next_cell <= NUM_CELLS):
        raise ValueError("next_cell out of range")

    cur = sorted(hw_open_cells_no_all_off.current_on)
    prev = cur[0] if len(cur) == 1 else None

    if prev is None:
        hw_open_cells_no_all_off([next_cell], overlap_sec=0)
        return

    if prev == next_cell:
        hw_open_cells_no_all_off([next_cell], overlap_sec=0)
        return

    # Brief overlap (prev+next), then end in next only
    hw_open_cells_no_all_off([prev, next_cell], overlap_sec=0)
    if overlap_sec and overlap_sec > 0:
        time.sleep(overlap_sec)
    hw_open_cells_no_all_off([next_cell], overlap_sec=0)


# ------------------------
# Utilities
# ------------------------
def sleep_with_stop(stop_event: threading.Event, total_sec: float, check=0.02):
    elapsed = 0.0
    while elapsed < total_sec:
        if stop_event.is_set():
            return False
        step = min(check, total_sec - elapsed)
        time.sleep(step)
        elapsed += step
    return True


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def parse_int_in_range(s: str, lo: int, hi: int, label="Value") -> int:
    try:
        v = int(str(s).strip())
    except Exception:
        raise ValueError(f"{label} must be an integer")
    if not (lo <= v <= hi):
        raise ValueError(f"{label} must be in range {lo}..{hi}")
    return v


def parse_float_pos(s: str, label="Value") -> float:
    try:
        v = float(str(s).strip())
    except Exception:
        raise ValueError(f"{label} must be a number")
    if v <= 0:
        raise ValueError(f"{label} must be > 0")
    return v


def make_singlecoil_path(current_cell, target_cell, n_cells=32):
    """
    Return list of single-coil steps (tuples like (k,)) to move along 1D line.
    current_cell can be None.
    """
    target_cell = clamp(target_cell, 1, n_cells)

    if current_cell is None or not (1 <= current_cell <= n_cells):
        return [(target_cell,)]

    steps = []
    if current_cell < target_cell:
        for c in range(current_cell + 1, target_cell + 1):
            steps.append((c,))
    elif current_cell > target_cell:
        for c in range(current_cell - 1, target_cell - 1, -1):
            steps.append((c,))
    else:
        steps.append((target_cell,))
    return steps


def choose_split_ab(center_cell, n_cells=32):
    """Choose adjacent coils (A,B) around split location."""
    c = clamp(center_cell, 1, n_cells - 1)
    return c, c + 1


# ========================
# TAB 1: Single Travel + Double Split (separate timing)
# ========================
class SingleFluidTab:
    """
    Travel = single coil, uses SEQUENCE delay
    Split  = double coil pattern, uses SPLIT delay
    """

    def __init__(self, parent, conn_text_var):
        self.frame = ttk.Frame(parent)
        self.conn_text_var = conn_text_var

        self.status_var = tk.StringVar(value="Ready (TEST MODE)" if TEST_MODE else "Ready (Not connected)")

        # Separate delays
        self.seq_delay_var = tk.StringVar(value="0.25")   # travel / normal sequence
        self.split_delay_var = tk.StringVar(value="0.08") # split pattern speed

        # Go To / Split Here
        self.goto_var = tk.StringVar(value="16")
        self.splithere_var = tk.StringVar(value="16")
        self.goto_run_now_var = tk.BooleanVar(value=True)
        self.split_run_now_var = tk.BooleanVar(value=True)

        # Manual 2-coil split sweep settings
        self.split2_var = tk.StringVar(value="15,16")     # A,B (adjacent)
        self.split_cycles_var = tk.StringVar(value="30")  # cycles

        # Run appended blocks immediately?
        self.run_after_append_var = tk.BooleanVar(value=True)

        # Current travel selection (single)
        self.active_cell = None  # int or None

        # Recorded steps: list of (tag, tuple_of_coils)
        self.steps = []  # e.g. (STEP_SEQ, (10,)) or (STEP_SPLIT, (15,16))

        self.stop_run = threading.Event()
        self.is_running = False
        self.runner_thread = None

        self.buttons = {}
        self._build_ui()

    # ---------- UI ----------
    def _build_ui(self):
        main = ttk.Frame(self.frame)
        main.pack(padx=10, pady=10, fill="both", expand=True)

        left = ttk.Frame(main)
        left.grid(row=0, column=0, padx=10, pady=5)

        right = ttk.Frame(main)
        right.grid(row=0, column=1, padx=10, pady=5, sticky="n")

        # grid buttons (4x8)
        ROWS, COLS = 4, 8
        cell = 1
        for r in range(ROWS):
            for c in range(COLS):
                btn = tk.Button(
                    left,
                    text=f"{cell}\nOFF",
                    width=6,
                    height=3,
                    bg="lightgray",
                    command=lambda x=cell: self.select_single_cell(x),
                )
                btn.grid(row=r, column=c, padx=4, pady=4)
                self.buttons[cell] = btn
                cell += 1

        top_controls = ttk.Frame(right)
        top_controls.pack(fill="x", pady=5)
        ttk.Label(top_controls, text="(Shared connection on top of app)").grid(row=0, column=0, columnspan=2, sticky="w")
        ttk.Label(top_controls, textvariable=self.conn_text_var).grid(row=1, column=0, columnspan=2, sticky="w", pady=(4, 0))

        # Separate delay controls
        delay_box = ttk.LabelFrame(right, text="Timing")
        delay_box.pack(fill="x", pady=8)

        row1 = ttk.Frame(delay_box)
        row1.pack(fill="x", padx=8, pady=(6, 4))
        ttk.Label(row1, text="Sequence delay (sec):").pack(side="left")
        ttk.Entry(row1, textvariable=self.seq_delay_var, width=8).pack(side="left", padx=6)

        row2 = ttk.Frame(delay_box)
        row2.pack(fill="x", padx=8, pady=(0, 6))
        ttk.Label(row2, text="Split delay (sec):").pack(side="left")
        ttk.Entry(row2, textvariable=self.split_delay_var, width=8).pack(side="left", padx=22)

        runmode_row = ttk.Frame(right)
        runmode_row.pack(fill="x", pady=(2, 6))
        ttk.Checkbutton(
            runmode_row,
            text="Run appended blocks immediately",
            variable=self.run_after_append_var
        ).pack(side="left")

        # Go To / Split Here controls
        nav_row = ttk.Frame(right)
        nav_row.pack(fill="x", pady=(6, 2))
        ttk.Label(nav_row, text="Go To cell (travel single):").pack(side="left")
        ttk.Entry(nav_row, textvariable=self.goto_var, width=6).pack(side="left", padx=6)
        ttk.Button(nav_row, text="Go", command=self.append_go_to).pack(side="left", padx=4)
        ttk.Checkbutton(nav_row, text="Run now", variable=self.goto_run_now_var).pack(side="left", padx=(10, 0))

        split_at_row = ttk.Frame(right)
        split_at_row.pack(fill="x", pady=(2, 8))
        ttk.Label(split_at_row, text="Split at cell (double only):").pack(side="left")
        ttk.Entry(split_at_row, textvariable=self.splithere_var, width=6).pack(side="left", padx=6)
        ttk.Button(split_at_row, text="Split Here", command=self.append_split_here).pack(side="left", padx=4)
        ttk.Checkbutton(split_at_row, text="Run now", variable=self.split_run_now_var).pack(side="left", padx=(10, 0))

        # Manual split sweep controls
        split_row = ttk.Frame(right)
        split_row.pack(fill="x", pady=(2, 2))
        ttk.Label(split_row, text="Manual split coils (A,B):").pack(side="left")
        ttk.Entry(split_row, textvariable=self.split2_var, width=10).pack(side="left", padx=6)
        ttk.Label(split_row, text="Cycles:").pack(side="left", padx=(10, 0))
        ttk.Entry(split_row, textvariable=self.split_cycles_var, width=6).pack(side="left", padx=6)

        ttk.Label(right, text="Recorded commands preview:").pack(anchor="w")
        self.code_box = tk.Text(right, width=68, height=14)
        self.code_box.pack(pady=5)

        btn_row = ttk.Frame(right)
        btn_row.pack(fill="x", pady=5)
        ttk.Button(btn_row, text="Start", width=10, command=self.run_sequence).pack(side="left", padx=4)
        ttk.Button(btn_row, text="STOP", width=10, command=self.stop_running).pack(side="left", padx=4)
        ttk.Button(btn_row, text="Clear", width=10, command=self.clear_sequence).pack(side="left", padx=4)
        ttk.Button(btn_row, text="Delete Last", width=12, command=self.delete_last_step).pack(side="left", padx=4)

        preset_row = ttk.Frame(right)
        preset_row.pack(fill="x", pady=(8, 2))
        ttk.Button(
            preset_row,
            text="Travel Sweep 1→32→1 (SINGLE, APPEND)",
            command=self.append_single_sweep_1_to_32_to_1
        ).pack(side="left", padx=4)

        preset_row2 = ttk.Frame(right)
        preset_row2.pack(fill="x", pady=(4, 2))
        ttk.Button(
            preset_row2,
            text="Split Sweep (DOUBLE, APPEND)",
            command=self.append_split_sweep_2coil
        ).pack(side="left", padx=4)

        ttk.Label(
            right,
            text=(
                "Behavior:\n"
                "- Travel steps are tagged SEQ and use Sequence delay.\n"
                "- Split steps are tagged SPLIT and use Split delay.\n"
                "- You can mix them in one run and timing switches automatically."
            ),
            justify="left",
            wraplength=560
        ).pack(anchor="w", pady=(10, 6))

        ttk.Label(right, textvariable=self.status_var, wraplength=560, justify="left").pack(anchor="w", pady=4)

        self.redraw_all_buttons()

    def redraw_all_buttons(self):
        for cell in range(1, NUM_CELLS + 1):
            if self.active_cell == cell:
                self.buttons[cell].config(text=f"{cell}\nON", bg="lightgreen")
            else:
                self.buttons[cell].config(text=f"{cell}\nOFF", bg="lightgray")

    def _code_line_for(self, tag: str, cells_tuple):
        if len(cells_tuple) == 0:
            return f"[{tag}] open_cells()  # (none)\n"
        return f"[{tag}] open_cells(" + ", ".join(map(str, cells_tuple)) + ")\n"

    def _append_step(self, tag: str, cells_tuple):
        self.steps.append((tag, tuple(cells_tuple)))
        self.code_box.insert("end", self._code_line_for(tag, tuple(cells_tuple)))
        self.code_box.see("end")

    # ---------- Validation ----------
    def _get_seq_delay(self) -> float:
        return parse_float_pos(self.seq_delay_var.get(), label="Sequence delay")

    def _get_split_delay(self) -> float:
        return parse_float_pos(self.split_delay_var.get(), label="Split delay")

    def _parse_cycles(self) -> int:
        try:
            n = int(str(self.split_cycles_var.get()).strip())
        except Exception:
            raise ValueError("Cycles must be an integer")
        if n <= 0:
            raise ValueError("Cycles must be > 0")
        return n

    # ---------- Manual single-coil selection (SEQ) ----------
    def select_single_cell(self, cell: int):
        if self.is_running:
            self.status_var.set("Running... stop first to edit selection.")
            return

        # Toggle off if same
        if self.active_cell == cell:
            self.active_cell = None
            self._append_step(STEP_SEQ, tuple())  # none
            self.redraw_all_buttons()
            self.status_var.set(f"Cell {cell} turned OFF (none selected).")
            try:
                if connected:
                    hw_open_cells_no_all_off([], overlap_sec=0)
            except Exception as e:
                messagebox.showerror("Hardware Error", str(e))
            return

        self.active_cell = cell
        self._append_step(STEP_SEQ, (cell,))
        self.redraw_all_buttons()
        self.status_var.set(f"Travel select: Cell {cell} ON (single).")

        try:
            if connected:
                hw_set_single_travel_cell(cell, overlap_sec=TRAVEL_SWITCH_OVERLAP_SEC)
            else:
                print(f"[NO DEVICE] would set single travel cell: {cell}")
        except Exception as e:
            messagebox.showerror("Hardware Error", str(e))

    # ---------- Running ----------
    def _run_steps_list(self, steps_to_run, label="Running..."):
        if self.is_running:
            self.status_var.set("Already running.")
            return
        if not steps_to_run:
            self.status_var.set("No steps to run.")
            return

        try:
            seq_d = self._get_seq_delay()
            split_d = self._get_split_delay()
        except Exception as e:
            messagebox.showerror("Timing error", str(e))
            return

        self.stop_run.clear()

        def worker():
            self.is_running = True
            try:
                self.status_var.set(label)
                for i, (tag, step) in enumerate(steps_to_run, start=1):
                    if self.stop_run.is_set():
                        self.status_var.set("Stopped.")
                        return

                    # Choose delay based on tag
                    delay = split_d if tag == STEP_SPLIT else seq_d

                    if connected:
                        # Travel step: (k,) -> end with single coil k
                        if len(step) == 1 and tag == STEP_SEQ:
                            hw_set_single_travel_cell(step[0], overlap_sec=TRAVEL_SWITCH_OVERLAP_SEC)
                            self.active_cell = step[0]
                            self.redraw_all_buttons()
                        else:
                            # Split steps or any non-single step: set exactly those coils
                            hw_open_cells_no_all_off(step, overlap_sec=0.02)
                            if len(step) == 0:
                                self.active_cell = None
                                self.redraw_all_buttons()
                    else:
                        print(f"[NO DEVICE] would run step {i}: tag={tag} coils={step}")

                        if len(step) == 1 and tag == STEP_SEQ:
                            self.active_cell = step[0]
                            self.redraw_all_buttons()
                        elif len(step) == 0:
                            self.active_cell = None
                            self.redraw_all_buttons()

                    self.status_var.set(f"{label}  Step {i}/{len(steps_to_run)}  {tag}  delay={delay:.3f}s  coils={step}")

                    if not sleep_with_stop(self.stop_run, delay):
                        self.status_var.set("Stopped.")
                        return

                self.status_var.set("Finished.")
            except Exception as e:
                messagebox.showerror("Error", str(e))
                self.status_var.set("Error while running")
            finally:
                self.is_running = False

        self.runner_thread = threading.Thread(target=worker, daemon=True)
        self.runner_thread.start()

    def run_sequence(self):
        self._run_steps_list(self.steps, label="Running recorded sequence...")

    def stop_running(self):
        self.stop_run.set()
        self.status_var.set("Stop requested...")

        if connected:
            try:
                hw_close_all()
                hw_open_cells_no_all_off.current_on = set()
            except Exception as e:
                messagebox.showerror("Hardware Error", str(e))

        self.active_cell = None
        self.redraw_all_buttons()
        self.is_running = False
        self.status_var.set("Stopped and all cells closed.")

    # ---------- Sequence editing ----------
    def clear_sequence(self):
        if self.is_running:
            self.status_var.set("Stop first before clearing.")
            return
        self.steps.clear()
        self.code_box.delete("1.0", "end")
        self.status_var.set("Sequence cleared.")

    def delete_last_step(self):
        if self.is_running:
            self.status_var.set("Stop first before deleting.")
            return
        if not self.steps:
            self.status_var.set("Nothing to delete.")
            return

        self.steps.pop()
        lines = self.code_box.get("1.0", "end-1c").splitlines()
        if lines:
            lines.pop()
        self.code_box.delete("1.0", "end")
        if lines:
            self.code_box.insert("end", "\n".join(lines) + "\n")

        # Restore travel highlight from last SEQ single-coil step (if any)
        self.active_cell = None
        for tag, st in reversed(self.steps):
            if tag == STEP_SEQ and len(st) == 1:
                self.active_cell = st[0]
                break
        self.redraw_all_buttons()

        try:
            if connected:
                if self.active_cell is None:
                    hw_open_cells_no_all_off([], overlap_sec=0)
                else:
                    hw_set_single_travel_cell(self.active_cell, overlap_sec=TRAVEL_SWITCH_OVERLAP_SEC)
        except Exception as e:
            messagebox.showerror("Hardware Error", str(e))

        self.status_var.set("Deleted last step.")

    # ---------- Go To (auto single-coil travel path, tagged SEQ) ----------
    def _current_cell_for_path(self):
        if self.active_cell is not None:
            return self.active_cell
        for tag, st in reversed(self.steps):
            if tag == STEP_SEQ and len(st) == 1:
                return st[0]
        return None

    def append_go_to(self):
        if self.is_running:
            self.status_var.set("Already running.")
            return

        try:
            target = parse_int_in_range(self.goto_var.get(), 1, NUM_CELLS, label="Go To cell")
        except Exception as e:
            messagebox.showerror("Go To input error", str(e))
            return

        cur = self._current_cell_for_path()
        new_steps = make_singlecoil_path(cur, target, n_cells=NUM_CELLS)

        appended = []
        for st in new_steps:
            self._append_step(STEP_SEQ, st)
            appended.append((STEP_SEQ, st))

        if new_steps:
            self.active_cell = new_steps[-1][0]
            self.redraw_all_buttons()
            try:
                if connected:
                    hw_set_single_travel_cell(self.active_cell, overlap_sec=TRAVEL_SWITCH_OVERLAP_SEC)
            except Exception as e:
                messagebox.showerror("Hardware Error", str(e))

        self.status_var.set(f"Appended Go To travel path → cell {target} (single hold {self.active_cell})")

        if self.goto_run_now_var.get() and appended:
            self._run_steps_list(appended, label="Running appended Go To...")

    # ---------- Split Here (double-coil only, tagged SPLIT) ----------
    def append_split_here(self):
        if self.is_running:
            self.status_var.set("Already running.")
            return

        try:
            center = parse_int_in_range(self.splithere_var.get(), 1, NUM_CELLS, label="Split at cell")
            A, B = choose_split_ab(center, n_cells=NUM_CELLS)
            cycles = self._parse_cycles()
        except Exception as e:
            messagebox.showerror("Split Here input error", str(e))
            return

        pattern = [
            (A,),
            (A, B),
            (B,),
            (A, B),
        ]

        appended = []
        for _ in range(cycles):
            for st in pattern:
                self._append_step(STEP_SPLIT, st)
                appended.append((STEP_SPLIT, st))

        self.status_var.set(f"Appended SPLIT block @ {center} using DOUBLE coils ({A},{B}) for {cycles} cycles.")

        if self.split_run_now_var.get() and appended:
            self._run_steps_list(appended, label="Running appended SPLIT block...")

    # ---------- Travel Sweep (single, tagged SEQ) ----------
    def append_single_sweep_1_to_32_to_1(self):
        if self.is_running:
            self.status_var.set("Already running.")
            return

        sweep = [(i,) for i in range(1, NUM_CELLS + 1)] + [(i,) for i in range(NUM_CELLS - 1, 0, -1)]
        appended = []
        for st in sweep:
            self._append_step(STEP_SEQ, st)
            appended.append((STEP_SEQ, st))

        self.status_var.set("Appended SINGLE travel sweep 1→32→1 (SEQ).")
        if self.run_after_append_var.get():
            self._run_steps_list(appended, label="Running appended SINGLE sweep...")

    # ---------- Manual Split Sweep (double, tagged SPLIT) ----------
    def _parse_two_coils(self):
        raw = str(self.split2_var.get()).replace(" ", "")
        parts = [p for p in raw.split(",") if p]
        if len(parts) != 2:
            raise ValueError("Enter exactly 2 coils like: 15,16")

        A = int(parts[0])
        B = int(parts[1])

        if not (1 <= A <= NUM_CELLS) or not (1 <= B <= NUM_CELLS):
            raise ValueError(f"Coils must be in range 1..{NUM_CELLS}")
        if A == B:
            raise ValueError("The 2 coils must be different")

        A, B = sorted((A, B))
        if B != A + 1:
            raise ValueError("For DOUBLE-coil split, choose ADJACENT coils (e.g., 15,16)")
        return A, B

    def append_split_sweep_2coil(self):
        if self.is_running:
            self.status_var.set("Already running.")
            return

        try:
            A, B = self._parse_two_coils()
            cycles = self._parse_cycles()
        except Exception as e:
            messagebox.showerror("Split sweep input error", str(e))
            return

        pattern = [
            (A,),
            (A, B),
            (B,),
            (A, B),
        ]

        appended = []
        for _ in range(cycles):
            for st in pattern:
                self._append_step(STEP_SPLIT, st)
                appended.append((STEP_SPLIT, st))

        self.status_var.set(f"Appended DOUBLE split sweep (SPLIT) using ({A},{B}) for {cycles} cycles.")
        if self.run_after_append_var.get():
            self._run_steps_list(appended, label="Running appended SPLIT sweep...")


# ======================
# TAB 2: Dual Fluid Mode (same as before; uses one delay)
# ======================
class DualFluidTab:
    OVERLAP_SEC = 0.02

    def __init__(self, parent, conn_text_var):
        self.frame = ttk.Frame(parent)
        self.conn_text_var = conn_text_var

        self.status_var = tk.StringVar(value="Ready (TEST MODE)" if TEST_MODE else "Ready (Not connected)")
        self.delay_var = tk.StringVar(value="0.2")
        self.edit_var = tk.StringVar(value="A")  # A=Blue, B=Red

        self.active_A = []
        self.active_B = []
        self.steps_A = []
        self.steps_B = []
        self.running_union = set()

        self.stop_run = threading.Event()
        self.is_running = False
        self.runner_thread = None

        self.buttons = {}
        self._build_ui()

    def _build_ui(self):
        main = ttk.Frame(self.frame)
        main.pack(padx=10, pady=10, fill="both", expand=True)

        left = ttk.Frame(main)
        left.grid(row=0, column=0, padx=10, pady=5)

        right = ttk.Frame(main)
        right.grid(row=0, column=1, padx=10, pady=5, sticky="n")

        ROWS, COLS = 4, 8
        cell = 1
        for r in range(ROWS):
            for c in range(COLS):
                btn = tk.Button(
                    left,
                    text=f"{cell}\nOFF",
                    width=6,
                    height=3,
                    bg="lightgray",
                    command=lambda x=cell: self.click_cell(x)
                )
                btn.grid(row=r, column=c, padx=4, pady=4)
                self.buttons[cell] = btn
                cell += 1

        top_controls = ttk.Frame(right)
        top_controls.pack(fill="x", pady=5)
        ttk.Label(top_controls, text="(Shared connection on top of app)").grid(row=0, column=0, columnspan=2, sticky="w")
        ttk.Label(top_controls, textvariable=self.conn_text_var).grid(row=1, column=0, columnspan=2, sticky="w", pady=(4, 0))

        ttk.Label(right, text="Edit pathway:").pack(anchor="w", pady=(6, 0))
        mode_row = ttk.Frame(right)
        mode_row.pack(anchor="w", pady=2)
        ttk.Radiobutton(mode_row, text="Blue (A)", variable=self.edit_var, value="A").pack(side="left")
        ttk.Radiobutton(mode_row, text="Red (B)", variable=self.edit_var, value="B").pack(side="left")

        delay_row = ttk.Frame(right)
        delay_row.pack(fill="x", pady=6)
        ttk.Label(delay_row, text="Delay (sec):").pack(side="left")
        ttk.Entry(delay_row, textvariable=self.delay_var, width=8).pack(side="left", padx=6)

        ttk.Label(right, text="Recorded steps preview (Dual runs together):").pack(anchor="w")
        self.code_box = tk.Text(right, width=56, height=14)
        self.code_box.pack(pady=5)

        btn_row = ttk.Frame(right)
        btn_row.pack(fill="x", pady=5)
        ttk.Button(btn_row, text="Start", width=10, command=self.run_sequence).pack(side="left", padx=4)
        ttk.Button(btn_row, text="STOP", width=10, command=self.stop_running).pack(side="left", padx=4)
        ttk.Button(btn_row, text="Clear", width=10, command=self.clear_all).pack(side="left", padx=4)
        ttk.Button(btn_row, text="Delete Last", width=12, command=self.delete_last).pack(side="left", padx=4)

        ttk.Label(right, textvariable=self.status_var, wraplength=520, justify="left").pack(anchor="w", pady=8)

        self.refresh_preview()
        self.redraw_buttons()
        self.sync_hw_preview()

    def _apply_click_keep_closest(self, active_list, cell):
        if cell in active_list:
            active_list.remove(cell)
            active_list.sort()
            return

        if len(active_list) < 2:
            active_list.append(cell)
            active_list.sort()
            return

        a, b = active_list[0], active_list[1]
        da, db = abs(a - cell), abs(b - cell)
        keep = a if da <= db else b
        active_list[:] = sorted([keep, cell])

    def click_cell(self, cell: int):
        if self.is_running:
            self.status_var.set("Running... stop first to edit.")
            return

        which = self.edit_var.get()
        if which == "A":
            self._apply_click_keep_closest(self.active_A, cell)
            self.steps_A.append(tuple(self.active_A))
            self.status_var.set(f"Blue(A) active now: {tuple(self.active_A)}")
        else:
            self._apply_click_keep_closest(self.active_B, cell)
            self.steps_B.append(tuple(self.active_B))
            self.status_var.set(f"Red(B) active now: {tuple(self.active_B)}")

        self.redraw_buttons()
        self.refresh_preview()
        self.sync_hw_preview()

    def combined_preview_set(self):
        return set(self.active_A) | set(self.active_B)

    def redraw_buttons(self):
        A = set(self.active_A)
        B = set(self.active_B)
        running = set(self.running_union)

        for cell in range(1, NUM_CELLS + 1):
            inA = cell in A
            inB = cell in B

            if cell in running:
                self.buttons[cell].config(text=f"{cell}\nON", bg="lightgreen")
                continue

            if inA and inB:
                self.buttons[cell].config(text=f"{cell}\nA+B", bg="plum")
            elif inA:
                self.buttons[cell].config(text=f"{cell}\nA", bg="lightblue")
            elif inB:
                self.buttons[cell].config(text=f"{cell}\nB", bg="salmon")
            else:
                self.buttons[cell].config(text=f"{cell}\nOFF", bg="lightgray")

    def sync_hw_preview(self):
        if connected and not self.is_running:
            try:
                hw_open_cells_no_all_off(sorted(self.combined_preview_set()), overlap_sec=self.OVERLAP_SEC)
            except Exception as e:
                messagebox.showerror("Hardware Error", str(e))

    def build_run_steps(self):
        n = max(len(self.steps_A), len(self.steps_B))
        if n == 0:
            return []

        steps = []
        for i in range(n):
            A_step = self.steps_A[i] if i < len(self.steps_A) else (self.steps_A[-1] if self.steps_A else ())
            B_step = self.steps_B[i] if i < len(self.steps_B) else (self.steps_B[-1] if self.steps_B else ())
            comb = sorted(set(A_step) | set(B_step))
            steps.append((A_step, B_step, comb))
        return steps

    def refresh_preview(self):
        self.code_box.delete("1.0", "end")
        self.code_box.insert("end", f"Current Blue(A) active: {tuple(self.active_A)}\n")
        self.code_box.insert("end", f"Current Red(B)  active: {tuple(self.active_B)}\n\n")
        self.code_box.insert("end", f"A steps recorded: {len(self.steps_A)}\n")
        self.code_box.insert("end", f"B steps recorded: {len(self.steps_B)}\n\n")

        steps = self.build_run_steps()
        if not steps:
            self.code_box.insert("end", "(No steps yet — click cells in Blue and/or Red)\n")
            return

        self.code_box.insert("end", "Run steps (each opens A_step ∪ B_step):\n")
        for i, (A_step, B_step, comb) in enumerate(steps, start=1):
            self.code_box.insert(
                "end",
                f"{i:02d}: A={A_step}  B={B_step}  -> open_cells({', '.join(map(str, comb))})\n"
            )
        self.code_box.see("end")

    def clear_all(self):
        if self.is_running:
            self.status_var.set("Stop first before clearing.")
            return

        self.active_A.clear()
        self.active_B.clear()
        self.steps_A.clear()
        self.steps_B.clear()
        self.running_union.clear()

        self.redraw_buttons()
        self.refresh_preview()
        self.status_var.set("Cleared Dual A and B steps.")

        if connected:
            try:
                hw_close_all()
                hw_open_cells_no_all_off.current_on = set()
            except Exception as e:
                messagebox.showerror("Hardware Error", str(e))

    def delete_last(self):
        if self.is_running:
            self.status_var.set("Stop first before deleting.")
            return

        which = self.edit_var.get()
        if which == "A":
            if not self.steps_A:
                self.status_var.set("No Blue(A) steps to delete.")
                return
            self.steps_A.pop()
            self.active_A = list(self.steps_A[-1]) if self.steps_A else []
        else:
            if not self.steps_B:
                self.status_var.set("No Red(B) steps to delete.")
                return
            self.steps_B.pop()
            self.active_B = list(self.steps_B[-1]) if self.steps_B else []

        self.active_A.sort()
        self.active_B.sort()
        self.redraw_buttons()
        self.refresh_preview()
        self.sync_hw_preview()

    def run_sequence(self):
        if self.is_running:
            self.status_var.set("Already running.")
            return

        steps = self.build_run_steps()
        if not steps:
            self.status_var.set("No steps recorded yet.")
            return

        try:
            d = parse_float_pos(self.delay_var.get(), label="Delay")
        except Exception as e:
            messagebox.showerror("Error", str(e))
            return

        self.stop_run.clear()

        def worker():
            self.is_running = True
            try:
                self.status_var.set("Running dual pathways together...")
                for i, (_A_step, _B_step, comb) in enumerate(steps, start=1):
                    if self.stop_run.is_set():
                        self.status_var.set("Stopped.")
                        return

                    if connected:
                        hw_open_cells_no_all_off(comb, overlap_sec=self.OVERLAP_SEC)
                    else:
                        print(f"[NO DEVICE] would run step {i}: {comb}")

                    self.running_union = set(comb)
                    self.redraw_buttons()
                    self.status_var.set(f"Step {i}/{len(steps)}  ON={comb}")

                    if not sleep_with_stop(self.stop_run, d):
                        self.status_var.set("Stopped.")
                        return

                self.status_var.set("Finished.")
            except Exception as e:
                messagebox.showerror("Error", str(e))
                self.status_var.set("Error while running")
            finally:
                self.is_running = False
                self.running_union.clear()
                self.redraw_buttons()
                self.sync_hw_preview()

        self.runner_thread = threading.Thread(target=worker, daemon=True)
        self.runner_thread.start()

    def stop_running(self):
        self.stop_run.set()
        self.status_var.set("Stop requested...")

        if connected:
            try:
                hw_close_all()
                hw_open_cells_no_all_off.current_on = set()
            except Exception as e:
                messagebox.showerror("Hardware Error", str(e))

        self.running_union.clear()
        self.redraw_buttons()
        self.is_running = False
        self.status_var.set("Stopped and all cells closed.")
        self.sync_hw_preview()


# ------------------------
# Main App with Tabs + COM Port Picker
# ------------------------


if __name__ == "__main__":
    main()


