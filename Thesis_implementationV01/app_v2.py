import tkinter as tk
from tkinter import ttk
import heapq

# ==========================================
# CONFIGURATION
# ==========================================
class Config:
    def __init__(self):
        # COMMUNICATION BASE VALUES
        self.link_rate_mbps = 11.22
        self.control_msg_kb = 2.0
        self.tensor1_mb = 2.5
        self.tensor2_mb = 2.0
        self.final_tensor_mb = 1.5
        self.raw_image_mb = 10.0

        # COMPUTATION BASE VALUES
        self.edge_plan_ms = 80.0
        self.capture_processing_ms = 180.0
        self.peer1_processing_ms = 140.0
        self.peer2_processing_ms = 140.0
        self.edge_finish_ms = 80.0

        # FAILOVER CONTROL VALUES
        self.fail_detect_ms = 50.0
        self.fail_replan_ms = 80.0

        # FALLBACK COMPUTE ASSUMPTIONS
        self.local_full_compute_ms = 540.0
        self.edge_only_full_compute_ms = 240.0
        self.reserve_margin_ms = 600.0

        # BATTERY MODEL DRAIN VALUES (%)
        self.image_capture_drain = 0.02
        self.capture_segment_processing_drain = 0.08
        self.peer1_segment_processing_drain = 0.06
        self.peer2_segment_processing_drain = 0.06
        self.tensor1_send_drain = 0.70
        self.tensor2_send_drain = 0.55
        self.final_tensor_send_drain = 0.55
        self.tensor_receive_drain = 0.03
        self.idle_drain_per_ms = (2.8 / 60.0) / 1000.0

def calc_data_ms(payload_mb, link_rate_mbps):
    if link_rate_mbps <= 0: return 0
    return (payload_mb * 8.0 / link_rate_mbps) * 1000.0

def calc_control_ms(payload_kb, link_rate_mbps):
    return calc_data_ms(payload_kb / 1024.0, link_rate_mbps)


# ==========================================
# ACTORS & PICTURES
# ==========================================
class Actor:
    def __init__(self, name):
        self.name = name
        self.state = "IDLE"
        self.current_pic = ""
        self.stage_name = ""
        self.busy_start = None
        self.idle_since = 0.0

class UAV(Actor):
    def __init__(self, name):
        super().__init__(name)
        self.battery = 100.0
        
    def drain(self, amount):
        self.battery = max(0.0, self.battery - amount)

class Picture:
    def __init__(self, pid):
        self.id = pid
        self.state = "INIT"
        self.roles = {}
        self.cum_time = 0.0
        self.step_counter = 0

# ==========================================
# SIMULATOR ENGINE (PIPELINE SCHEDULER)
# ==========================================
class SimulatorEngine:
    def __init__(self, app, config, scenario):
        self.app = app
        self.config = config
        self.scenario = scenario
        self.time = 0.0
        self.events = []
        self.event_id_seq = 0
        
        self.pictures = [Picture("Pic01"), Picture("Pic02"), Picture("Pic03")]
        self.uavs = {f"UAV{i}": UAV(f"UAV{i}") for i in range(1, 5)}
        self.edge = Actor("Edge")
        
        self.has_failed = False
        self.is_running = True
        self.blocks = {name: [] for name in list(self.uavs.keys()) + ["Edge"]}
        
        # Calculate Deadlines
        t_ctrl = calc_control_ms(config.control_msg_kb, config.link_rate_mbps)
        t_t1 = calc_data_ms(config.tensor1_mb, config.link_rate_mbps)
        t_t2 = calc_data_ms(config.tensor2_mb, config.link_rate_mbps)
        t_fin = calc_data_ms(config.final_tensor_mb, config.link_rate_mbps)
        norm = (t_ctrl + config.edge_plan_ms + t_ctrl + config.capture_processing_ms +
                t_t1 + config.peer1_processing_ms + t_t2 + config.peer2_processing_ms +
                t_fin + config.edge_finish_ms + t_ctrl)
        
        self.normal_deadline = norm + config.reserve_margin_ms
        if "Failover" in scenario: self.active_deadline = self.normal_deadline + 300
        elif "Edge-only" in scenario: self.active_deadline = self.normal_deadline + 500
        elif "Local-only" in scenario: self.active_deadline = self.normal_deadline + 200
        else: self.active_deadline = self.normal_deadline

    def add_event(self, delay, cb):
        self.event_id_seq += 1
        heapq.heappush(self.events, (self.time + delay, self.event_id_seq, cb))

    def set_busy(self, actor, state, pic_id, stage_name):
        if hasattr(actor, "battery"):
            if actor.state == "IDLE":
                idle_ms = self.time - actor.idle_since
                actor.drain(idle_ms * self.config.idle_drain_per_ms)
        actor.state = state
        actor.current_pic = pic_id
        actor.stage_name = stage_name
        actor.busy_start = self.time

    def set_idle(self, actor):
        if actor.busy_start is not None and self.time > actor.busy_start:
            if actor.state in ["PROCESSING", "TRANSMITTING", "RECEIVING", "CAPTURE"]:
                self.blocks[actor.name].append((actor.busy_start, self.time, actor.current_pic, actor.stage_name, actor.state))
        actor.state = "IDLE"
        actor.current_pic = ""
        actor.stage_name = ""
        actor.idle_since = self.time

    def get_best_uav(self, exclude=None):
        if exclude is None: exclude = []
        exclude_names = [u.name for u in exclude]
        candidates = []
        for u in self.uavs.values():
            if u.name in exclude_names or u.state == "FAILED": continue
            if u.state == "IDLE":
                temp_batt = u.battery - ((self.time - u.idle_since) * self.config.idle_drain_per_ms)
                candidates.append((temp_batt, u.idle_since, u.name, u))
        if not candidates:
            return None
        # Sort by: highest battery (-temp_batt), longest idle (smallest idle_since), lowest index (name)
        candidates.sort(key=lambda x: (-x[0], x[1], x[2]))
        return candidates[0][3]

    def get_display_battery(self, uav):
        if uav.state == "IDLE":
            idle_ms = self.time - uav.idle_since
            return max(0.0, uav.battery - (idle_ms * self.config.idle_drain_per_ms))
        return uav.battery

    # ------------------ EVENT LOOP ------------------
    def start(self):
        self.app.clear_ui()
        self.app.log_msg(f"Started scenario: {self.scenario}")
        self.try_schedule()
        self.run_next()

    def run_next(self):
        if not self.is_running: return
        if not self.events:
            self.app.update_ui_live(self)
            self.app.log_msg("All pictures completed. Pipeline finished.")
            return
            
        t, _, cb = heapq.heappop(self.events)
        delay_sim = t - self.time
        self.time = t
        cb()
        self.try_schedule()
        self.app.update_ui_live(self)
        
        real_delay = max(1, int(delay_sim / 3.0)) # 3x playback speed
        self.app.after(real_delay, self.run_next)

    # ------------------ SCHEDULER ------------------
    def try_schedule(self):
        # Prioritize Pic01 > Pic02 > Pic03 to prevent pipeline stall
        for pic in self.pictures:
            if pic.state == "DONE": continue
            
            if pic.state == "INIT":
                if "Local-only" in self.scenario:
                    cap = self.get_best_uav()
                    if cap:
                        self.set_busy(cap, "WAITING", pic.id, "FAIL_DETECT")
                        pic.roles["Capture"] = cap
                        pic.state = "LOCAL_DETECT"
                        self.app.log_msg(f"{pic.id} started locally on {cap.name} (Peers & Edge Unavailable)")
                        self.add_event(self.config.fail_detect_ms, lambda p=pic: self.ev_local_detect_done(p))
                else:
                    cap = self.get_best_uav()
                    if cap and self.edge.state == "IDLE":
                        self.set_busy(cap, "TRANSMITTING", pic.id, "REQ")
                        self.set_busy(self.edge, "RECEIVING", pic.id, "REQ")
                        pic.roles["Capture"] = cap
                        pic.state = "REQ"
                        self.app.log_msg(f"{pic.id} started. Capture assigned to {cap.name} (Battery: {cap.battery:.1f}%)")
                        delay = calc_control_ms(self.config.control_msg_kb, self.config.link_rate_mbps)
                        self.add_event(delay, lambda p=pic: self.ev_req_done(p))

            elif pic.state == "WAIT_PEER1":
                peer = self.get_best_uav(exclude=[pic.roles["Capture"]])
                if peer:
                    if "Failover" in self.scenario and not self.has_failed:
                        self.has_failed = True
                        peer.state = "FAILED"
                        self.app.log_msg(f"Peer {peer.name} selected for {pic.id} but FAILED. Rerouting pipeline...")
                        self.set_busy(pic.roles["Capture"], "WAITING", pic.id, "FAIL_DETECT")
                        self.set_busy(self.edge, "PROCESSING", pic.id, "FAIL_REPLAN")
                        pic.state = "REPLANNING"
                        self.add_event(self.config.fail_detect_ms + self.config.fail_replan_ms, lambda p=pic: self.ev_replan_done(p))
                    else:
                        cap = pic.roles["Capture"]
                        self.set_busy(cap, "TRANSMITTING", pic.id, "SEND_T1")
                        self.set_busy(peer, "RECEIVING", pic.id, "SEND_T1")
                        pic.roles["Peer1"] = peer
                        pic.state = "SEND_T1"
                        self.app.log_msg(f"{peer.name} selected as Peer 1 for {pic.id} (Battery: {peer.battery:.1f}%)")
                        delay = calc_data_ms(self.config.tensor1_mb, self.config.link_rate_mbps)
                        self.add_event(delay, lambda p=pic: self.ev_send_t1_done(p))
            
            elif pic.state == "WAIT_PEER2":
                peer = self.get_best_uav(exclude=[pic.roles["Peer1"]])
                if peer:
                    p1 = pic.roles["Peer1"]
                    self.set_busy(p1, "TRANSMITTING", pic.id, "SEND_T2")
                    self.set_busy(peer, "RECEIVING", pic.id, "SEND_T2")
                    pic.roles["Peer2"] = peer
                    pic.state = "SEND_T2"
                    self.app.log_msg(f"{peer.name} selected as Peer 2 for {pic.id} (Battery: {peer.battery:.1f}%)")
                    delay = calc_data_ms(self.config.tensor2_mb, self.config.link_rate_mbps)
                    self.add_event(delay, lambda p=pic: self.ev_send_t2_done(p))

            elif pic.state == "WAIT_EDGE":
                if self.edge.state == "IDLE":
                    p2 = pic.roles["Peer2"]
                    self.set_busy(p2, "TRANSMITTING", pic.id, "SEND_FINAL")
                    self.set_busy(self.edge, "RECEIVING", pic.id, "SEND_FINAL")
                    pic.state = "SEND_FINAL"
                    delay = calc_data_ms(self.config.final_tensor_mb, self.config.link_rate_mbps)
                    self.add_event(delay, lambda p=pic: self.ev_send_final_done(p))

    # ------------------ EVENT CALLBACKS ------------------
    def log_step(self, pic, step_name, sender, receiver, stype, payload, delay):
        pic.cum_time += delay
        pic.step_counter += 1
        self.app.add_tree_row(pic.id, pic.step_counter, step_name, sender, receiver, stype, payload, delay, pic.cum_time)

    def print_pic_summary(self, pic):
        margin = self.active_deadline - pic.cum_time
        status = "MET" if margin >= 0 else "MISSED"
        self.app.log_msg(f"=== {pic.id} COMPLETE ===")
        self.app.log_msg(f"Time: {pic.cum_time:.1f}ms | Deadline: {self.active_deadline:.1f}ms | Margin: {margin:.1f}ms ({status})")

    def ev_req_done(self, pic):
        cap = pic.roles["Capture"]
        cap.drain(self.config.image_capture_drain)
        delay = calc_control_ms(self.config.control_msg_kb, self.config.link_rate_mbps)
        self.log_step(pic, "Task Request", cap.name, "Edge", "Control", f"{self.config.control_msg_kb} KB", delay)
        
        if "Edge-only" in self.scenario:
            self.set_busy(cap, "WAITING", pic.id, "WAIT_PLAN")
            self.set_busy(self.edge, "PROCESSING", pic.id, "EDGE_PLAN")
            self.add_event(self.config.edge_plan_ms, lambda p=pic: self.ev_edge_only_plan_done(p))
        else:
            self.set_busy(cap, "WAITING", pic.id, "WAIT_PLAN")
            self.set_busy(self.edge, "PROCESSING", pic.id, "PLAN_GEN")
            self.add_event(self.config.edge_plan_ms, lambda p=pic: self.ev_plan_gen_done(p))

    def ev_plan_gen_done(self, pic):
        self.log_step(pic, "Global Plan Gen", "Edge", "Edge", "Compute", "-", self.config.edge_plan_ms)
        cap = pic.roles["Capture"]
        self.set_busy(self.edge, "TRANSMITTING", pic.id, "PLAN_DIST")
        self.set_busy(cap, "RECEIVING", pic.id, "PLAN_DIST")
        delay = calc_control_ms(self.config.control_msg_kb, self.config.link_rate_mbps)
        self.add_event(delay, lambda p=pic: self.ev_plan_dist_done(p))

    def ev_plan_dist_done(self, pic):
        delay = calc_control_ms(self.config.control_msg_kb, self.config.link_rate_mbps)
        self.log_step(pic, "Plan Dist", "Edge", "Nodes", "Control", f"{self.config.control_msg_kb} KB", delay)
        self.set_idle(self.edge)
        cap = pic.roles["Capture"]
        self.set_busy(cap, "PROCESSING", pic.id, "CAP_PROC")
        self.add_event(self.config.capture_processing_ms, lambda p=pic: self.ev_cap_proc_done(p))

    def ev_cap_proc_done(self, pic):
        cap = pic.roles["Capture"]
        cap.drain(self.config.capture_segment_processing_drain)
        self.log_step(pic, "Capture Process", cap.name, cap.name, "Compute", "-", self.config.capture_processing_ms)
        self.set_busy(cap, "WAITING", pic.id, "WAIT_PEER1")
        pic.state = "WAIT_PEER1"

    def ev_replan_done(self, pic):
        self.log_step(pic, "Failure Detect", "System", "System", "Control", "-", self.config.fail_detect_ms)
        self.log_step(pic, "Fail Reroute Plan", "Edge", "Edge", "Compute", "-", self.config.fail_replan_ms)
        self.set_idle(self.edge)
        pic.state = "WAIT_PEER1" # Retry peer 1 selection

    def ev_send_t1_done(self, pic):
        cap = pic.roles["Capture"]
        peer = pic.roles["Peer1"]
        cap.drain(self.config.tensor1_send_drain)
        peer.drain(self.config.tensor_receive_drain)
        delay = calc_data_ms(self.config.tensor1_mb, self.config.link_rate_mbps)
        self.log_step(pic, "Send Tensor 1", cap.name, peer.name, "Data", f"{self.config.tensor1_mb} MB", delay)
        self.set_idle(cap) # Cap is freed! Pipelining advantage here.
        self.set_busy(peer, "PROCESSING", pic.id, "PEER1_PROC")
        self.add_event(self.config.peer1_processing_ms, lambda p=pic: self.ev_peer1_proc_done(p))

    def ev_peer1_proc_done(self, pic):
        peer = pic.roles["Peer1"]
        peer.drain(self.config.peer1_segment_processing_drain)
        self.log_step(pic, "Peer 1 Process", peer.name, peer.name, "Compute", "-", self.config.peer1_processing_ms)
        self.set_busy(peer, "WAITING", pic.id, "WAIT_PEER2")
        pic.state = "WAIT_PEER2"

    def ev_send_t2_done(self, pic):
        p1 = pic.roles["Peer1"]
        p2 = pic.roles["Peer2"]
        p1.drain(self.config.tensor2_send_drain)
        p2.drain(self.config.tensor_receive_drain)
        delay = calc_data_ms(self.config.tensor2_mb, self.config.link_rate_mbps)
        self.log_step(pic, "Send Tensor 2", p1.name, p2.name, "Data", f"{self.config.tensor2_mb} MB", delay)
        self.set_idle(p1)
        self.set_busy(p2, "PROCESSING", pic.id, "PEER2_PROC")
        self.add_event(self.config.peer2_processing_ms, lambda p=pic: self.ev_peer2_proc_done(p))

    def ev_peer2_proc_done(self, pic):
        p2 = pic.roles["Peer2"]
        p2.drain(self.config.peer2_segment_processing_drain)
        self.log_step(pic, "Peer 2 Process", p2.name, p2.name, "Compute", "-", self.config.peer2_processing_ms)
        self.set_busy(p2, "WAITING", pic.id, "WAIT_EDGE")
        pic.state = "WAIT_EDGE"

    def ev_send_final_done(self, pic):
        p2 = pic.roles["Peer2"]
        p2.drain(self.config.final_tensor_send_drain)
        delay = calc_data_ms(self.config.final_tensor_mb, self.config.link_rate_mbps)
        self.log_step(pic, "Send Final", p2.name, "Edge", "Data", f"{self.config.final_tensor_mb} MB", delay)
        self.set_idle(p2)
        self.set_busy(self.edge, "PROCESSING", pic.id, "EDGE_FINISH")
        self.add_event(self.config.edge_finish_ms, lambda p=pic: self.ev_edge_finish_done(p))

    def ev_edge_finish_done(self, pic):
        self.log_step(pic, "Edge Finish", "Edge", "Edge", "Compute", "-", self.config.edge_finish_ms)
        self.set_busy(self.edge, "TRANSMITTING", pic.id, "ACK")
        delay = calc_control_ms(self.config.control_msg_kb, self.config.link_rate_mbps)
        self.add_event(delay, lambda p=pic: self.ev_ack_done(p))

    def ev_ack_done(self, pic):
        delay = calc_control_ms(self.config.control_msg_kb, self.config.link_rate_mbps)
        cap_name = pic.roles["Capture"].name
        self.log_step(pic, "Completion Ack", "Edge", cap_name, "Control", f"{self.config.control_msg_kb} KB", delay)
        self.set_idle(self.edge)
        pic.state = "DONE"
        self.print_pic_summary(pic)

    # Edge-Only & Local-Only Overrides
    def ev_edge_only_plan_done(self, pic):
        self.log_step(pic, "Edge Plan (Fallback)", "Edge", "Edge", "Compute", "-", self.config.edge_plan_ms)
        cap = pic.roles["Capture"]
        self.set_busy(cap, "TRANSMITTING", pic.id, "SEND_RAW")
        self.set_busy(self.edge, "RECEIVING", pic.id, "SEND_RAW")
        delay = calc_data_ms(self.config.raw_image_mb, self.config.link_rate_mbps)
        self.add_event(delay, lambda p=pic: self.ev_send_raw_done(p))

    def ev_send_raw_done(self, pic):
        cap = pic.roles["Capture"]
        cap.drain(self.config.tensor1_send_drain)
        delay = calc_data_ms(self.config.raw_image_mb, self.config.link_rate_mbps)
        self.log_step(pic, "Send Raw Image", cap.name, "Edge", "Data", f"{self.config.raw_image_mb} MB", delay)
        self.set_idle(cap)
        self.set_busy(self.edge, "PROCESSING", pic.id, "EDGE_FULL")
        self.add_event(self.config.edge_only_full_compute_ms, lambda p=pic: self.ev_edge_full_done(p))

    def ev_edge_full_done(self, pic):
        self.log_step(pic, "Edge Full Process", "Edge", "Edge", "Compute", "-", self.config.edge_only_full_compute_ms)
        self.set_busy(self.edge, "TRANSMITTING", pic.id, "ACK")
        delay = calc_control_ms(self.config.control_msg_kb, self.config.link_rate_mbps)
        self.add_event(delay, lambda p=pic: self.ev_ack_done(p))

    def ev_local_detect_done(self, pic):
        cap = pic.roles["Capture"]
        self.log_step(pic, "Detect Disconnect", cap.name, cap.name, "Control", "-", self.config.fail_detect_ms)
        self.set_busy(cap, "PROCESSING", pic.id, "LOCAL_PROC")
        self.add_event(self.config.local_full_compute_ms, lambda p=pic: self.ev_local_proc_done(p))

    def ev_local_proc_done(self, pic):
        cap = pic.roles["Capture"]
        cap.drain(self.config.capture_segment_processing_drain)
        self.log_step(pic, "Local Full Process", cap.name, cap.name, "Compute", "-", self.config.local_full_compute_ms)
        self.set_idle(cap)
        pic.state = "DONE"
        self.log_step(pic, "Local Complete", cap.name, cap.name, "Compute", "-", 0.0)
        self.print_pic_summary(pic)


# ==========================================
# TKINTER UI APPLICATION
# ==========================================
class UAVDemoAppV2(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Pipelined UAV Swarm Inference Demo V2")
        self.geometry("1400x900")
        self.configure(padx=10, pady=10)
        
        self.config = Config()
        self.vars = {}
        self.engine = None
        
        self._build_ui()
        self.reset_defaults()

    def _build_ui(self):
        self.rowconfigure(0, weight=0)
        self.rowconfigure(1, weight=1)
        
        # 1. TOP PANEL: Live Actors
        top_frame = ttk.LabelFrame(self, text="Live Actor States (Battery & Pipeline Tasks)", padding=5)
        top_frame.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        
        self.top_vars = {}
        actors = ["UAV1", "UAV2", "UAV3", "UAV4", "Edge"]
        for i, actor in enumerate(actors):
            f = ttk.Frame(top_frame, borderwidth=1, relief="solid")
            f.grid(row=0, column=i, padx=5, pady=5, sticky="nsew")
            top_frame.columnconfigure(i, weight=1)
            
            ttk.Label(f, text=actor, font=("Segoe UI", 10, "bold"), foreground="#005A9C").pack(anchor=tk.W, padx=5, pady=(5,0))
            self.top_vars[actor] = {
                "battery": tk.StringVar(value="Bat: 100.00%"),
                "state": tk.StringVar(value="State: IDLE"),
                "task": tk.StringVar(value="Task: None")
            }
            if actor != "Edge":
                ttk.Label(f, textvariable=self.top_vars[actor]["battery"]).pack(anchor=tk.W, padx=5)
            ttk.Label(f, textvariable=self.top_vars[actor]["state"]).pack(anchor=tk.W, padx=5)
            ttk.Label(f, textvariable=self.top_vars[actor]["task"]).pack(anchor=tk.W, padx=5, pady=(0,5))
            
        # 2. MID SECTION (Config + Data + Log)
        mid_frame = ttk.Frame(self)
        mid_frame.grid(row=1, column=0, sticky="nsew")
        mid_frame.rowconfigure(0, weight=1)
        mid_frame.rowconfigure(1, weight=0)
        mid_frame.columnconfigure(1, weight=1)
        
        # 2a. Config Panel
        config_frame = ttk.LabelFrame(mid_frame, text="Controls & Settings", padding=5)
        config_frame.grid(row=0, column=0, rowspan=2, sticky="ns", padx=(0, 10))
        
        ctrl_bar = ttk.Frame(config_frame)
        ctrl_bar.pack(fill=tk.X, pady=(0, 10))
        ttk.Label(ctrl_bar, text="Scenario:").pack(anchor=tk.W)
        self.scenario_var = tk.StringVar(value="Normal Distributed Inference")
        scenarios = ["Normal Distributed Inference", "Failover / Reroute Scenario", 
                     "Edge-only Fallback Scenario", "Local-only Fallback Scenario"]
        ttk.Combobox(ctrl_bar, textvariable=self.scenario_var, values=scenarios, state="readonly", width=30).pack(fill=tk.X, pady=2)
        ttk.Button(ctrl_bar, text="Run Pipeline", command=self.run_scenario).pack(fill=tk.X, pady=2)
        ttk.Button(ctrl_bar, text="Reset Defaults", command=self.reset_defaults).pack(fill=tk.X, pady=2)
        
        self._build_config_form(config_frame)
        
        # 2b. Treeview
        tree_frame = ttk.LabelFrame(mid_frame, text="Event Pipeline (All Pictures)", padding=5)
        tree_frame.grid(row=0, column=1, sticky="nsew", pady=(0, 5))
        
        columns = ("Pic ID", "Step", "Stage Name", "Sender", "Receiver", "Type", "Payload", "Delay (ms)", "Cum. Time (ms)")
        self.tree = ttk.Treeview(tree_frame, columns=columns, show="headings", selectmode="none")
        widths = [60, 40, 140, 80, 80, 60, 60, 80, 100]
        for col, w in zip(columns, widths):
            self.tree.heading(col, text=col)
            self.tree.column(col, width=w, anchor=tk.CENTER)
            
        scroll_y = ttk.Scrollbar(tree_frame, orient=tk.VERTICAL, command=self.tree.yview)
        self.tree.configure(yscrollcommand=scroll_y.set)
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scroll_y.pack(side=tk.RIGHT, fill=tk.Y)
        
        # 2c. Log Area
        log_frame = ttk.LabelFrame(mid_frame, text="Execution Log & Pipeline Reasoning", padding=5)
        log_frame.grid(row=1, column=1, sticky="ew")
        self.txt_log = tk.Text(log_frame, height=8, bg="#f9f9f9", font=("Consolas", 9))
        self.txt_log.pack(fill=tk.BOTH, expand=True)
        self.txt_log.config(state=tk.DISABLED)

        # 3. GANTT CHART BOTTOM
        gantt_frame = ttk.LabelFrame(self, text="Pipeline Timeline (Gantt Chart)", padding=5)
        gantt_frame.grid(row=2, column=0, sticky="nsew", pady=(10, 0))
        self.rowconfigure(2, weight=1)
        self.canvas = tk.Canvas(gantt_frame, bg="white", height=200)
        self.canvas.pack(fill=tk.BOTH, expand=True)

    def _build_config_form(self, parent):
        fields = [
            ("link_rate_mbps", "Link Rate (Mbps)"), ("control_msg_kb", "Control Msg (KB)"),
            ("tensor1_mb", "Tensor 1 (MB)"), ("tensor2_mb", "Tensor 2 (MB)"), ("final_tensor_mb", "Final Tensor (MB)"),
            ("edge_plan_ms", "Edge Plan (ms)"), ("capture_processing_ms", "Capture Proc (ms)"),
            ("peer1_processing_ms", "Peer 1 Proc (ms)"), ("peer2_processing_ms", "Peer 2 Proc (ms)"),
            ("fail_detect_ms", "Fail Detect (ms)"), ("fail_replan_ms", "Fail Replan (ms)")
        ]
        container = ttk.Frame(parent)
        container.pack(fill=tk.BOTH, expand=True)
        for i, (attr, label) in enumerate(fields):
            ttk.Label(container, text=label).grid(row=i, column=0, sticky="w", pady=2)
            var = tk.DoubleVar()
            self.vars[attr] = var
            ttk.Entry(container, textvariable=var, width=8).grid(row=i, column=1, sticky="e", pady=2)

    def reset_defaults(self):
        default_config = Config()
        for attr, var in self.vars.items():
            var.set(getattr(default_config, attr))

    def _sync_config_from_ui(self):
        for attr, var in self.vars.items():
            try: setattr(self.config, attr, float(var.get()))
            except ValueError: pass

    def run_scenario(self):
        if self.engine and self.engine.is_running:
            self.engine.is_running = False
        self._sync_config_from_ui()
        self.engine = SimulatorEngine(self, self.config, self.scenario_var.get())
        self.engine.start()

    def clear_ui(self):
        for item in self.tree.get_children(): self.tree.delete(item)
        self.txt_log.config(state=tk.NORMAL)
        self.txt_log.delete("1.0", tk.END)
        self.txt_log.config(state=tk.DISABLED)
        self.canvas.delete("all")

    def log_msg(self, msg):
        self.txt_log.config(state=tk.NORMAL)
        time_prefix = f"[{self.engine.time:6.1f}ms] " if self.engine else ""
        self.txt_log.insert(tk.END, time_prefix + msg + "\n")
        self.txt_log.see(tk.END)
        self.txt_log.config(state=tk.DISABLED)

    def add_tree_row(self, *values):
        vals = list(values)
        vals[7] = f"{vals[7]:.2f}"
        vals[8] = f"{vals[8]:.2f}"
        self.tree.insert("", tk.END, values=vals)

    def update_ui_live(self, engine):
        for name, uav in engine.uavs.items():
            batt = engine.get_display_battery(uav)
            self.top_vars[name]["battery"].set(f"Bat: {batt:.2f}%")
            self.top_vars[name]["state"].set(f"State: {uav.state}")
            task_str = f"{uav.current_pic} {uav.stage_name}".strip()
            self.top_vars[name]["task"].set(f"Task: {task_str if task_str else 'None'}")
        
        self.top_vars["Edge"]["state"].set(f"State: {engine.edge.state}")
        tstr = f"{engine.edge.current_pic} {engine.edge.stage_name}".strip()
        self.top_vars["Edge"]["task"].set(f"Task: {tstr if tstr else 'None'}")
        
        self.draw_gantt(engine)

    def draw_gantt(self, engine):
        self.canvas.delete("all")
        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()
        if w < 100 or h < 50: return
        
        actors = ["UAV1", "UAV2", "UAV3", "UAV4", "Edge"]
        row_h = h / len(actors)
        max_time = max(engine.time, 1000.0)
        scale = (w - 100) / max_time
        colors = {"Pic01": "#87CEFA", "Pic02": "#98FB98", "Pic03": "#F08080"}
        
        for i, a in enumerate(actors):
            y = i * row_h
            self.canvas.create_text(10, y + row_h/2, text=a, anchor="w", font=("Segoe UI", 10, "bold"))
            self.canvas.create_line(80, y + row_h, w, y + row_h, fill="#ccc")
            
            blks = list(engine.blocks[a])
            curr_actor = engine.uavs[a] if a in engine.uavs else engine.edge
            if curr_actor.state not in ["IDLE", "FAILED"] and curr_actor.busy_start is not None:
                blks.append((curr_actor.busy_start, engine.time, curr_actor.current_pic, curr_actor.stage_name, curr_actor.state))
                
            for start, end, pid, stage, state in blks:
                if state == "WAITING": continue # Don't draw idle block holding tensor
                x1 = 80 + start * scale
                x2 = 80 + end * scale
                if x2 - x1 < 2: x2 = x1 + 2
                self.canvas.create_rectangle(x1, y+5, x2, y+row_h-5, fill=colors.get(pid, "gray"), outline="black")
                if (x2 - x1) > 40:
                    self.canvas.create_text(x1 + (x2-x1)/2, y + row_h/2, text=stage, font=("Segoe UI", 8))

if __name__ == "__main__":
    app = UAVDemoAppV2()
    app.mainloop()
