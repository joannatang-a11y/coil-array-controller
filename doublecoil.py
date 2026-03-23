"""
Magnetic Sites Sequencer (32 coils) — Single + Dual tabs
Purpose:
- Smooth travel using 2-coil “moving well” (overlap, no all-off gaps)
- Split attempt using ONLY 2 coils (no 3rd coil) via tug-of-war necking pattern
- Sweeps APPEND to your existing steps (do not delete)
- Optional “Go To” (auto-plans 2-coil slide path) + “Split Here” (auto 2-coil pair)

Hardware:
- Modbus RTU, 32 registers (0..31) controlling coils 1..32 via FC=6
- Change cell_to_register if your device uses a different mapping.

NOTE on geometry:
- This assumes coils are ordered linearly 1..32 for motion planning.
  If your physical neighbor layout is different (4x8 grid neighbors),
  tell me your wiring/neighbor order and I can swap in a grid path planner.
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

# ------------------------
# Utilities
# ------------------------
def sleep_with_stop(stop_event: threading.Event, total_sec: float, check=0.05):
    elapsed = 0.0
    while elapsed < total_sec:
        if stop_event.is_set():
            return False
        step = min(check, total_sec - elapsed)
        time.sleep(step)
        elapsed += step
    return True


def pick_two_keep_closest(active_two, new_cell):
    """
    When adding a 3rd cell in 2-coil mode, keep the one closest to the new click.
    Example: active=[15,16], new=14 -> keep 15, drop 16.
             active=[15,16], new=17 -> keep 16, drop 15.
    Tie-break: keep newer (active_two[1]).
    """
    a, b = active_two[0], active_two[1]
    da = abs(a - new_cell)
    db = abs(b - new_cell)
    if da < db:
        return a, b
    if db < da:
        return b, a
    return b, a


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


def make_doublecoil_path(current_pair, target_cell, n_cells=32):
    """
    Return list of 2-coil steps (tuples) to move along a 1D line of coils.

    current_pair: tuple like (i,i+1) or None
    target_cell: 1..n_cells

    End hold:
      - if target_cell < n_cells -> (target_cell, target_cell+1)
      - if target_cell == n_cells -> (n_cells-1, n_cells)
    """
    target_cell = clamp(target_cell, 1, n_cells)

    if target_cell == n_cells:
        final_pair = (n_cells - 1, n_cells)
    else:
        final_pair = (target_cell, target_cell + 1)

    if not current_pair or len(current_pair) != 2:
        return [final_pair]

    a, b = sorted(current_pair)
    if b != a + 1:
        return [final_pair]

    cur_left = a
    final_left = final_pair[0]

    steps = []
    if cur_left < final_left:
        for L in range(cur_left + 1, final_left + 1):
            steps.append((L, L + 1))
    elif cur_left > final_left:
        for L in range(cur_left - 1, final_left - 1, -1):
            steps.append((L, L + 1))
    else:
        steps.append(final_pair)

    return steps


def choose_split_ab(center_cell, n_cells=32):
    """
    Choose two adjacent coils (A,B) for a 2-coil split attempt.
    Default: (center, center+1), clamped so both are in range.
    """
    c = clamp(center_cell, 1, n_cells - 1)  # ensure room for +1
    A = c
    B = c + 1
    return A, B


# ========================
# TAB 1: Single Fluid Mode
# ========================
class SingleFluidTab:
    """
    - Click to build travel sequence (2-coil sliding rule).
    - Start runs your full recorded steps.
    - Sweep 1→32→1 APPENDS (does not delete).
    - Split Sweep (2 coils) APPENDS.
    - Go To: auto-plans a 2-coil slide path to a target cell and APPENDS.
    - Split Here: auto-selects (A,B) around a chosen cell and APPENDS (2-coil only).
    """

    OVERLAP_SEC = 0.02

    def __init__(self, parent, conn_text_var):
        self.frame = ttk.Frame(parent)
        self.conn_text_var = conn_text_var

        self.status_var = tk.StringVar(value="Ready (TEST MODE)" if TEST_MODE else "Ready (Not connected)")
        self.delay_var = tk.StringVar(value="0.2")

        # Go To / Split Here
        self.goto_var = tk.StringVar(value="16")
        self.splithere_var = tk.StringVar(value="16")
        self.goto_run_now_var = tk.BooleanVar(value=True)
        self.split_run_now_var = tk.BooleanVar(value=True)

        # Manual 2-coil split sweep settings
        self.split2_var = tk.StringVar(value="15,16")  # A,B (must be adjacent)
        self.split_cycles_var = tk.StringVar(value="20")  # 2-coil often needs more cycles

        # Behavior control: run appended sweeps immediately or just append
        self.run_after_append_var = tk.BooleanVar(value=True)

        self.active = []  # up to 2 for UI travel highlight
        self.steps = []   # list[tuple] like (2,) or (2,3)

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
                    command=lambda x=cell: self.toggle_site(x),
                )
                btn.grid(row=r, column=c, padx=4, pady=4)
                self.buttons[cell] = btn
                cell += 1

        # right controls
        top_controls = ttk.Frame(right)
        top_controls.pack(fill="x", pady=5)
        ttk.Label(top_controls, text="(Shared connection on top of app)").grid(row=0, column=0, columnspan=2, sticky="w")
        ttk.Label(top_controls, textvariable=self.conn_text_var).grid(row=1, column=0, columnspan=2, sticky="w", pady=(4, 0))

        delay_row = ttk.Frame(right)
        delay_row.pack(fill="x", pady=6)
        ttk.Label(delay_row, text="Delay (sec):").pack(side="left")
        ttk.Entry(delay_row, textvariable=self.delay_var, width=8).pack(side="left", padx=6)

        runmode_row = ttk.Frame(right)
        runmode_row.pack(fill="x", pady=(2, 6))
        ttk.Checkbutton(
            runmode_row,
            text="Run appended sweeps immediately",
            variable=self.run_after_append_var
        ).pack(side="left")

        # -------- Go To / Split Here controls --------
        nav_row = ttk.Frame(right)
        nav_row.pack(fill="x", pady=(6, 2))
        ttk.Label(nav_row, text="Go To cell:").pack(side="left")
        ttk.Entry(nav_row, textvariable=self.goto_var, width=6).pack(side="left", padx=6)
        ttk.Button(nav_row, text="Go", command=self.append_go_to).pack(side="left", padx=4)
        ttk.Checkbutton(nav_row, text="Run now", variable=self.goto_run_now_var).pack(side="left", padx=(10, 0))

        split_at_row = ttk.Frame(right)
        split_at_row.pack(fill="x", pady=(2, 8))
        ttk.Label(split_at_row, text="Split at cell:").pack(side="left")
        ttk.Entry(split_at_row, textvariable=self.splithere_var, width=6).pack(side="left", padx=6)
        ttk.Button(split_at_row, text="Split Here (2 coils)", command=self.append_split_here).pack(side="left", padx=4)
        ttk.Checkbutton(split_at_row, text="Run now", variable=self.split_run_now_var).pack(side="left", padx=(10, 0))

        # Manual split sweep controls (2 coils)
        split_row = ttk.Frame(right)
        split_row.pack(fill="x", pady=(2, 2))
        ttk.Label(split_row, text="Manual split coils (A,B):").pack(side="left")
        ttk.Entry(split_row, textvariable=self.split2_var, width=10).pack(side="left", padx=6)
        ttk.Label(split_row, text="Cycles:").pack(side="left", padx=(10, 0))
        ttk.Entry(split_row, textvariable=self.split_cycles_var, width=6).pack(side="left", padx=6)

        ttk.Label(right, text="Recorded commands preview (Single):").pack(anchor="w")
        self.code_box = tk.Text(right, width=56, height=14)
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
            text="Sweep 1→32→1 (APPEND)",
            command=self.append_sweep_1_to_32_to_1_two_coils
        ).pack(side="left", padx=4)

        preset_row2 = ttk.Frame(right)
        preset_row2.pack(fill="x", pady=(4, 2))
        ttk.Button(
            preset_row2,
            text="Split Sweep (2 coils, APPEND)",
            command=self.append_split_sweep_2coil
        ).pack(side="left", padx=4)

        ttk.Label(right, textvariable=self.status_var, wraplength=520, justify="left").pack(anchor="w", pady=8)

        self.redraw_all_buttons()

    def button_look(self, cell: int, on: bool):
        if on:
            self.buttons[cell].config(text=f"{cell}\nON", bg="lightgreen")
        else:
            self.buttons[cell].config(text=f"{cell}\nOFF", bg="lightgray")

    def redraw_all_buttons(self):
        active_set = set(self.active)
        for cell in range(1, NUM_CELLS + 1):
            self.button_look(cell, cell in active_set)

    def code_line_for(self, cells_tuple):
        if len(cells_tuple) == 0:
            return "open_cells()  # (none)\n"
        return "open_cells(" + ", ".join(map(str, cells_tuple)) + ")\n"

    def append_step(self, cells_tuple):
        self.steps.append(tuple(cells_tuple))
        self.code_box.insert("end", self.code_line_for(tuple(cells_tuple)))
        self.code_box.see("end")

    def sync_hw(self):
        """Mirror current selection to hardware WITHOUT all-off gaps."""
        if connected and not self.is_running:
            hw_open_cells_no_all_off(self.active, overlap_sec=self.OVERLAP_SEC)

    def toggle_site(self, cell: int):
        """2-coil sliding rule for travel control (manual clicking)."""
        if self.is_running:
            self.status_var.set("Running... stop first to edit selection.")
            return

        # turn OFF if already on
        if cell in self.active:
            self.active.remove(cell)
            self.active = sorted(self.active)
            self.redraw_all_buttons()
            self.status_var.set(f"Cell {cell} turned OFF (selected)")
            self.append_step(tuple(self.active))
            try:
                self.sync_hw()
            except Exception as e:
                messagebox.showerror("Hardware Error", str(e))
            return

        # turn ON
        if len(self.active) < 2:
            self.active.append(cell)
            self.active = sorted(self.active)
            self.status_var.set(f"Cell {cell} turned ON")
        else:
            keep_cell, drop_cell = pick_two_keep_closest(self.active, cell)
            self.active = sorted([keep_cell, cell])
            self.status_var.set(f"Cell {cell} ON, Cell {drop_cell} auto OFF (no all-off gap)")

        self.redraw_all_buttons()
        self.append_step(tuple(self.active))

        try:
            self.sync_hw()
        except Exception as e:
            messagebox.showerror("Hardware Error", str(e))

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

        self.active.clear()
        if self.steps:
            last = list(self.steps[-1])
            if len(last) <= 2:
                self.active = sorted(last)

        self.redraw_all_buttons()
        try:
            self.sync_hw()
        except Exception as e:
            messagebox.showerror("Hardware Error", str(e))

        self.status_var.set("Deleted last step.")

    def _validate_delay(self) -> float:
        try:
            d = float(self.delay_var.get())
            if d <= 0:
                raise ValueError
            return d
        except Exception:
            raise ValueError("Delay must be a positive number like 0.2")

    def _run_steps_list(self, steps_to_run, label="Running..."):
        """Run only a given list of steps (used for running appended blocks)."""
        if self.is_running:
            self.status_var.set("Already running.")
            return
        if not steps_to_run:
            self.status_var.set("No steps to run.")
            return

        try:
            d = self._validate_delay()
        except Exception as e:
            messagebox.showerror("Error", str(e))
            return

        self.stop_run.clear()

        def worker():
            self.is_running = True
            try:
                self.status_var.set(label)
                for i, step in enumerate(steps_to_run, start=1):
                    if self.stop_run.is_set():
                        self.status_var.set("Stopped.")
                        return

                    target_list = list(step)

                    if connected:
                        hw_open_cells_no_all_off(target_list, overlap_sec=self.OVERLAP_SEC)
                    else:
                        print(f"[NO DEVICE] would run: {target_list}")

                    # For UI: only highlight if <=2
                    if len(target_list) <= 2:
                        self.active[:] = sorted(target_list)
                        self.redraw_all_buttons()

                    self.status_var.set(f"{label}  Step {i}/{len(steps_to_run)} : {step}")

                    if not sleep_with_stop(self.stop_run, d):
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
        """Start runs the FULL recorded sequence."""
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

        self.active.clear()
        self.redraw_all_buttons()
        self.is_running = False
        self.status_var.set("Stopped and all cells closed.")

    # ---------- Go To (auto planned 2-coil path) ----------
    def _current_pair_for_path(self):
        # Prefer current active
        if len(self.active) == 2:
            a, b = sorted(self.active)
            if b == a + 1:
                return (a, b)

        # Fallback to last valid 2-coil step
        for st in reversed(self.steps):
            if len(st) == 2:
                a, b = sorted(st)
                if b == a + 1:
                    return (a, b)
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

        cur_pair = self._current_pair_for_path()
        new_steps = make_doublecoil_path(cur_pair, target, n_cells=NUM_CELLS)

        appended = []
        for st in new_steps:
            st = tuple(sorted(st))
            self.steps.append(st)
            appended.append(st)
            self.code_box.insert("end", self.code_line_for(st))
        self.code_box.see("end")

        if appended:
            self.active = list(appended[-1])
            self.active.sort()
            self.redraw_all_buttons()
            try:
                self.sync_hw()
            except Exception as e:
                messagebox.showerror("Hardware Error", str(e))

        self.status_var.set(f"Appended Go To path → cell {target} (hold {tuple(self.active)})")

        if self.goto_run_now_var.get() and appended:
            self._run_steps_list(appended, label="Running appended Go To...")

    # ---------- Split Here (2 coils only) ----------
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

        # 2-coil necking: anchor A -> stretch -> anchor B -> stretch
        pattern = [
            (A,),
            (A, B),
            (B,),
            (A, B),
        ]

        new_steps = []
        for _ in range(cycles):
            for st in pattern:
                st = tuple(sorted(st))
                self.steps.append(st)
                new_steps.append(st)
                self.code_box.insert("end", self.code_line_for(st))
        self.code_box.see("end")

        self.status_var.set(f"Appended 2-coil Split Here @ {center} using (A,B)=({A},{B}) for {cycles} cycles.")

        if self.split_run_now_var.get() and new_steps:
            self._run_steps_list(new_steps, label="Running appended 2-coil Split Here...")

    # ---------- APPEND Sweep 1→32→1 (2 coils) ----------
    def append_sweep_1_to_32_to_1_two_coils(self):
        if self.is_running:
            self.status_var.set("Already running.")
            return

        forward_pairs = [(i, i + 1) for i in range(1, NUM_CELLS)]
        backward_pairs = [(i, i + 1) for i in range(NUM_CELLS - 2, 0, -1)]
        sweep_pairs = forward_pairs + backward_pairs

        new_steps = []
        for a, b in sweep_pairs:
            st = (a, b)
            self.steps.append(st)
            new_steps.append(st)
            self.code_box.insert("end", self.code_line_for(st))
        self.code_box.see("end")

        self.status_var.set("Appended 1→32→1 sweep to your sequence.")
        if self.run_after_append_var.get():
            self._run_steps_list(new_steps, label="Running appended sweep...")

    # ---------- APPEND Split Sweep (2 coils) ----------
    def _parse_two_coils(self):
        raw = self.split2_var.get().replace(" ", "")
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
            raise ValueError("For 2-coil split, choose ADJACENT coils (e.g., 15,16)")
        return A, B

    def _parse_cycles(self):
        try:
            n = int(self.split_cycles_var.get())
            if n <= 0:
                raise ValueError
            return n
        except Exception:
            raise ValueError("Cycles must be a positive integer (e.g., 20)")

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

        new_steps = []
        for _ in range(cycles):
            for st in pattern:
                st = tuple(sorted(st))
                self.steps.append(st)
                new_steps.append(st)
                self.code_box.insert("end", self.code_line_for(st))
        self.code_box.see("end")

        self.status_var.set(f"Appended 2-coil split sweep using ({A},{B}) for {cycles} cycles.")
        if self.run_after_append_var.get():
            self._run_steps_list(new_steps, label="Running appended 2-coil split sweep...")


# ======================
# TAB 2: Dual Fluid Mode
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

        ttk.Label(
            right,
            text="Rule: Each pathway keeps 2 ON.\nIf adding a 3rd, it drops the one FARTHER from the new click.",
            justify="left"
        ).pack(anchor="w", pady=(6, 8))

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

        keep_cell, _drop_cell = pick_two_keep_closest(active_list, cell)
        active_list[:] = sorted([keep_cell, cell])

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
            self.status_var.set("Deleted last Blue(A) step.")
        else:
            if not self.steps_B:
                self.status_var.set("No Red(B) steps to delete.")
                return
            self.steps_B.pop()
            self.active_B = list(self.steps_B[-1]) if self.steps_B else []
            self.status_var.set("Deleted last Red(B) step.")

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
            d = float(self.delay_var.get())
            if d <= 0:
                raise ValueError
        except Exception:
            messagebox.showerror("Error", "Delay must be a positive number like 0.2")
            return

        self.stop_run.clear()

        def worker():
            self.is_running = True
            try:
                self.status_var.set("Running dual pathways together...")
                for i, (A_step, B_step, comb) in enumerate(steps, start=1):
                    if self.stop_run.is_set():
                        self.status_var.set("Stopped.")
                        return

                    if connected:
                        hw_open_cells_no_all_off(comb, overlap_sec=self.OVERLAP_SEC)
                    else:
                        print(f"[NO DEVICE] would run step {i}: {comb}")

                    self.running_union = set(comb)
                    self.redraw_buttons()
                    self.status_var.set(f"Step {i}/{len(steps)}  A={A_step}  B={B_step}  ON={comb}")

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


