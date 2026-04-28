import tkinter as tk
from tkinter import ttk
import copy

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

        # EXTRA FAILOVER CONTROL VALUES
        self.fail_detect_ms = 50.0
        self.fail_replan_ms = 80.0

        # FALLBACK COMPUTE ASSUMPTIONS (DEMO)
        self.local_full_compute_ms = 540.0
        self.edge_only_full_compute_ms = 240.0

        # DEADLINE
        self.reserve_margin_ms = 600.0

def calc_data_ms(payload_mb, link_rate_mbps):
    """Calculates communication time in milliseconds for MB payload."""
    if link_rate_mbps <= 0: return 0
    time_seconds = (payload_mb * 8.0) / link_rate_mbps
    return time_seconds * 1000.0

def calc_control_ms(payload_kb, link_rate_mbps):
    """Calculates communication time in milliseconds for KB control payload."""
    payload_mb = payload_kb / 1024.0
    return calc_data_ms(payload_mb, link_rate_mbps)

def get_normal_deadline(config):
    """Calculates the normal deadline to be used as a baseline for all scenarios."""
    ctrl_time = calc_control_ms(config.control_msg_kb, config.link_rate_mbps)
    t1_time = calc_data_ms(config.tensor1_mb, config.link_rate_mbps)
    t2_time = calc_data_ms(config.tensor2_mb, config.link_rate_mbps)
    final_time = calc_data_ms(config.final_tensor_mb, config.link_rate_mbps)
    
    normal_total_ms = (
        ctrl_time + config.edge_plan_ms + ctrl_time + 
        config.capture_processing_ms + t1_time + 
        config.peer1_processing_ms + t2_time + 
        config.peer2_processing_ms + final_time + 
        config.edge_finish_ms + ctrl_time
    )
    return normal_total_ms + config.reserve_margin_ms

class ScenarioEngine:
    @staticmethod
    def run_normal(config):
        steps = []
        c = config
        lr = c.link_rate_mbps
        
        steps.append(("Task Request", "Capture UAV", "Edge Coordinator", "Control", f"{c.control_msg_kb} KB", calc_control_ms(c.control_msg_kb, lr)))
        steps.append(("Global Plan Gen", "Edge Coordinator", "Edge Coordinator", "Compute", "-", c.edge_plan_ms))
        steps.append(("Plan Dist", "Edge Coordinator", "Nodes", "Control", f"{c.control_msg_kb} KB", calc_control_ms(c.control_msg_kb, lr)))
        steps.append(("Capture Process", "Capture UAV", "Capture UAV", "Compute", "-", c.capture_processing_ms))
        steps.append(("Send Tensor 1", "Capture UAV", "Peer UAV 1", "Data", f"{c.tensor1_mb} MB", calc_data_ms(c.tensor1_mb, lr)))
        steps.append(("Peer 1 Process", "Peer UAV 1", "Peer UAV 1", "Compute", "-", c.peer1_processing_ms))
        steps.append(("Send Tensor 2", "Peer UAV 1", "Peer UAV 2", "Data", f"{c.tensor2_mb} MB", calc_data_ms(c.tensor2_mb, lr)))
        steps.append(("Peer 2 Process", "Peer UAV 2", "Peer UAV 2", "Compute", "-", c.peer2_processing_ms))
        steps.append(("Send Final Tensor", "Peer UAV 2", "Edge Coordinator", "Data", f"{c.final_tensor_mb} MB", calc_data_ms(c.final_tensor_mb, lr)))
        steps.append(("Edge Finish", "Edge Coordinator", "Edge Coordinator", "Compute", "-", c.edge_finish_ms))
        steps.append(("Completion Ack", "Edge Coordinator", "Capture UAV", "Control", f"{c.control_msg_kb} KB", calc_control_ms(c.control_msg_kb, lr)))
        
        return steps

    @staticmethod
    def run_failover(config):
        steps = []
        c = config
        lr = c.link_rate_mbps
        
        steps.append(("Task Request", "Capture UAV", "Edge Coordinator", "Control", f"{c.control_msg_kb} KB", calc_control_ms(c.control_msg_kb, lr)))
        steps.append(("Global Plan Gen", "Edge Coordinator", "Edge Coordinator", "Compute", "-", c.edge_plan_ms))
        steps.append(("Plan Dist", "Edge Coordinator", "Nodes", "Control", f"{c.control_msg_kb} KB", calc_control_ms(c.control_msg_kb, lr)))
        steps.append(("Capture Process", "Capture UAV", "Capture UAV", "Compute", "-", c.capture_processing_ms))
        
        steps.append(("Failure Detect", "System", "System", "Control", "-", c.fail_detect_ms))
        steps.append(("Fail Reroute Plan", "Edge Coordinator", "Edge Coordinator", "Compute", "-", c.fail_replan_ms))
        
        steps.append(("Send Tensor 1", "Capture UAV", "Backup UAV 1", "Data", f"{c.tensor1_mb} MB", calc_data_ms(c.tensor1_mb, lr)))
        steps.append(("Backup 1 Process", "Backup UAV 1", "Backup UAV 1", "Compute", "-", c.peer1_processing_ms))
        steps.append(("Send Tensor 2", "Backup UAV 1", "Peer UAV 2", "Data", f"{c.tensor2_mb} MB", calc_data_ms(c.tensor2_mb, lr)))
        
        steps.append(("Peer 2 Process", "Peer UAV 2", "Peer UAV 2", "Compute", "-", c.peer2_processing_ms))
        steps.append(("Send Final Tensor", "Peer UAV 2", "Edge Coordinator", "Data", f"{c.final_tensor_mb} MB", calc_data_ms(c.final_tensor_mb, lr)))
        steps.append(("Edge Finish", "Edge Coordinator", "Edge Coordinator", "Compute", "-", c.edge_finish_ms))
        steps.append(("Completion Ack", "Edge Coordinator", "Capture UAV", "Control", f"{c.control_msg_kb} KB", calc_control_ms(c.control_msg_kb, lr)))
        
        return steps

    @staticmethod
    def run_edge_only(config):
        steps = []
        c = config
        lr = c.link_rate_mbps
        
        steps.append(("Task Request", "Capture UAV", "Edge Coordinator", "Control", f"{c.control_msg_kb} KB", calc_control_ms(c.control_msg_kb, lr)))
        steps.append(("Edge Plan (Fallback)", "Edge Coordinator", "Edge Coordinator", "Compute", "-", c.edge_plan_ms))
        steps.append(("Send Raw Image", "Capture UAV", "Edge Coordinator", "Data", f"{c.raw_image_mb} MB", calc_data_ms(c.raw_image_mb, lr)))
        steps.append(("Edge Full Process", "Edge Coordinator", "Edge Coordinator", "Compute", "-", c.edge_only_full_compute_ms))
        steps.append(("Completion Ack", "Edge Coordinator", "Capture UAV", "Control", f"{c.control_msg_kb} KB", calc_control_ms(c.control_msg_kb, lr)))
        
        return steps

    @staticmethod
    def run_local_only(config):
        steps = []
        c = config
        lr = c.link_rate_mbps
        
        steps.append(("Detect Disconnect", "Capture UAV", "Capture UAV", "Control", "-", c.fail_detect_ms))
        steps.append(("Local Full Process", "Capture UAV", "Capture UAV", "Compute", "-", c.local_full_compute_ms))
        steps.append(("Local Complete", "Capture UAV", "Capture UAV", "Compute", "-", 0.0))
        
        return steps


class UAVDemoApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("UAV Swarm Distributed CNN Inference Demo")
        self.geometry("1100x750")
        self.configure(padx=10, pady=10)
        
        self.config = Config()
        self.vars = {}
        
        self._build_ui()
        self.reset_defaults()
        self.run_scenario()

    def _build_ui(self):
        # Configure Grid
        self.columnconfigure(0, weight=0, minsize=300)
        self.columnconfigure(1, weight=1)
        self.rowconfigure(0, weight=1)
        
        # Left Panel (Config)
        left_frame = ttk.LabelFrame(self, text="Configuration & Timing Assumptions", padding=10)
        left_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        self._build_config_form(left_frame)
        
        # Right Panel
        right_frame = ttk.Frame(self)
        right_frame.grid(row=0, column=1, sticky="nsew")
        right_frame.rowconfigure(1, weight=1)
        right_frame.columnconfigure(0, weight=1)
        
        # Scenario Selection (Top of Right Panel)
        top_bar = ttk.Frame(right_frame)
        top_bar.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        
        ttk.Label(top_bar, text="Scenario:").pack(side=tk.LEFT, padx=(0, 5))
        self.scenario_var = tk.StringVar(value="Normal Distributed Inference")
        scenarios = [
            "Normal Distributed Inference",
            "Failover / Reroute Scenario",
            "Edge-only Fallback Scenario",
            "Local-only Fallback Scenario"
        ]
        self.combo_scenario = ttk.Combobox(top_bar, textvariable=self.scenario_var, values=scenarios, state="readonly", width=35)
        self.combo_scenario.pack(side=tk.LEFT, padx=(0, 10))
        
        btn_run = ttk.Button(top_bar, text="Run / Update", command=self.run_scenario)
        btn_run.pack(side=tk.LEFT, padx=5)
        
        btn_reset = ttk.Button(top_bar, text="Reset Defaults", command=self.reset_defaults)
        btn_reset.pack(side=tk.LEFT, padx=5)
        
        # Treeview (Middle of Right Panel)
        tree_frame = ttk.Frame(right_frame)
        tree_frame.grid(row=1, column=0, sticky="nsew")
        
        columns = ("Step", "Stage Name", "Sender", "Receiver", "Type", "Payload", "Delay (ms)", "Cumulative Time (ms)", "Status / Note")
        self.tree = ttk.Treeview(tree_frame, columns=columns, show="headings", selectmode="none")
        
        # Setup column headings and widths
        widths = [40, 120, 100, 100, 80, 70, 80, 130, 200]
        for col, w in zip(columns, widths):
            self.tree.heading(col, text=col)
            self.tree.column(col, width=w, anchor=tk.CENTER)
            
        scroll_y = ttk.Scrollbar(tree_frame, orient=tk.VERTICAL, command=self.tree.yview)
        self.tree.configure(yscrollcommand=scroll_y.set)
        
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scroll_y.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Summary Area (Bottom of Right Panel)
        summary_frame = ttk.LabelFrame(right_frame, text="Execution Summary", padding=10)
        summary_frame.grid(row=2, column=0, sticky="ew", pady=(10, 0))
        
        self.lbl_scenario_name = ttk.Label(summary_frame, text="Scenario: ", font=("Segoe UI", 10, "bold"))
        self.lbl_scenario_name.grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 5))
        
        # Metrics setup
        self.metrics_vars = {
            "total_time": tk.StringVar(),
            "comm_sub": tk.StringVar(),
            "comp_sub": tk.StringVar(),
            "ctrl_sub": tk.StringVar(),
            "data_sub": tk.StringVar(),
            "fail_sub": tk.StringVar(),
            "rec_dead": tk.StringVar(),
            "act_dead": tk.StringVar(),
            "margin": tk.StringVar(),
            "result": tk.StringVar()
        }
        
        ttk.Label(summary_frame, text="Total Time:").grid(row=1, column=0, sticky="e", padx=5)
        ttk.Label(summary_frame, textvariable=self.metrics_vars["total_time"]).grid(row=1, column=1, sticky="w", padx=5)
        
        ttk.Label(summary_frame, text="Communication:").grid(row=2, column=0, sticky="e", padx=5)
        ttk.Label(summary_frame, textvariable=self.metrics_vars["comm_sub"]).grid(row=2, column=1, sticky="w", padx=5)
        
        ttk.Label(summary_frame, text="Computation:").grid(row=3, column=0, sticky="e", padx=5)
        ttk.Label(summary_frame, textvariable=self.metrics_vars["comp_sub"]).grid(row=3, column=1, sticky="w", padx=5)
        
        ttk.Label(summary_frame, text="Control-Plane:").grid(row=1, column=2, sticky="e", padx=5)
        ttk.Label(summary_frame, textvariable=self.metrics_vars["ctrl_sub"]).grid(row=1, column=3, sticky="w", padx=5)
        
        ttk.Label(summary_frame, text="Data-Plane:").grid(row=2, column=2, sticky="e", padx=5)
        ttk.Label(summary_frame, textvariable=self.metrics_vars["data_sub"]).grid(row=2, column=3, sticky="w", padx=5)
        
        ttk.Label(summary_frame, text="Failover Overhead:").grid(row=3, column=2, sticky="e", padx=5)
        ttk.Label(summary_frame, textvariable=self.metrics_vars["fail_sub"]).grid(row=3, column=3, sticky="w", padx=5)
        
        ttk.Label(summary_frame, text="Recommended Deadline:").grid(row=1, column=4, sticky="e", padx=5)
        ttk.Label(summary_frame, textvariable=self.metrics_vars["rec_dead"]).grid(row=1, column=5, sticky="w", padx=5)
        
        ttk.Label(summary_frame, text="Active Deadline:").grid(row=2, column=4, sticky="e", padx=5)
        ttk.Label(summary_frame, textvariable=self.metrics_vars["act_dead"]).grid(row=2, column=5, sticky="w", padx=5)
        
        ttk.Label(summary_frame, text="Margin / Slack:").grid(row=3, column=4, sticky="e", padx=5)
        ttk.Label(summary_frame, textvariable=self.metrics_vars["margin"]).grid(row=3, column=5, sticky="w", padx=5)
        
        self.lbl_result = ttk.Label(summary_frame, textvariable=self.metrics_vars["result"], font=("Segoe UI", 12, "bold"))
        self.lbl_result.grid(row=1, column=6, rowspan=3, padx=20)
        
        # Log Area
        self.txt_log = tk.Text(summary_frame, height=3, width=80, state=tk.DISABLED, bg="#f0f0f0", font=("Segoe UI", 9, "italic"))
        self.txt_log.grid(row=4, column=0, columnspan=7, pady=(10, 0), sticky="ew")

    def _build_config_form(self, parent):
        fields = [
            ("link_rate_mbps", "Link Rate (Mbps)"),
            ("control_msg_kb", "Control Msg (KB)"),
            ("tensor1_mb", "Tensor 1 (MB)"),
            ("tensor2_mb", "Tensor 2 (MB)"),
            ("final_tensor_mb", "Final Tensor (MB)"),
            ("raw_image_mb", "Raw Image (MB)"),
            ("edge_plan_ms", "Edge Plan (ms)"),
            ("capture_processing_ms", "Capture Proc (ms)"),
            ("peer1_processing_ms", "Peer 1 Proc (ms)"),
            ("peer2_processing_ms", "Peer 2 Proc (ms)"),
            ("edge_finish_ms", "Edge Finish (ms)"),
            ("fail_detect_ms", "Fail Detect (ms)"),
            ("fail_replan_ms", "Fail Replan (ms)"),
            ("local_full_compute_ms", "Local Full (ms) [Demo]"),
            ("edge_only_full_compute_ms", "Edge Only Full (ms) [Demo]"),
            ("reserve_margin_ms", "Reserve Margin (ms)"),
        ]
        
        canvas = tk.Canvas(parent, borderwidth=0, highlightthickness=0)
        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        for i, (attr, label) in enumerate(fields):
            ttk.Label(scrollable_frame, text=label).grid(row=i, column=0, sticky="w", pady=2, padx=2)
            var = tk.DoubleVar()
            self.vars[attr] = var
            entry = ttk.Entry(scrollable_frame, textvariable=var, width=10)
            entry.grid(row=i, column=1, sticky="e", pady=2, padx=2)

    def reset_defaults(self):
        default_config = Config()
        for attr, var in self.vars.items():
            var.set(getattr(default_config, attr))
        self.run_scenario()

    def _sync_config_from_ui(self):
        for attr, var in self.vars.items():
            try:
                setattr(self.config, attr, float(var.get()))
            except ValueError:
                pass # Ignore invalid inputs temporarily

    def run_scenario(self):
        self._sync_config_from_ui()
        scenario_name = self.scenario_var.get()
        
        # Generate Steps
        if "Normal" in scenario_name:
            steps_data = ScenarioEngine.run_normal(self.config)
            active_deadline = get_normal_deadline(self.config)
            log_msg = "Normal distributed execution completed successfully."
        elif "Failover" in scenario_name:
            steps_data = ScenarioEngine.run_failover(self.config)
            active_deadline = get_normal_deadline(self.config) + 300.0
            log_msg = "Peer UAV 1 failed; rerouting to Backup UAV 1."
        elif "Edge-only" in scenario_name:
            steps_data = ScenarioEngine.run_edge_only(self.config)
            active_deadline = get_normal_deadline(self.config) + 500.0
            log_msg = "Edge-only fallback activated. Peer nodes unavailable."
        elif "Local-only" in scenario_name:
            steps_data = ScenarioEngine.run_local_only(self.config)
            active_deadline = get_normal_deadline(self.config) + 200.0
            log_msg = "Local-only fallback activated. No edge or peer help available."
            
        self._update_display(scenario_name, steps_data, active_deadline, log_msg)

    def _update_display(self, scenario_name, steps_data, active_deadline, log_msg):
        # Clear Treeview
        for item in self.tree.get_children():
            self.tree.delete(item)
            
        total_time = 0.0
        comm_time = 0.0
        comp_time = 0.0
        ctrl_time = 0.0
        data_time = 0.0
        fail_time = 0.0
        
        for i, data in enumerate(steps_data):
            stage, sender, receiver, stype, payload, delay = data
            total_time += delay
            
            # Subtotals
            if stype == "Control":
                comm_time += delay
                ctrl_time += delay
            elif stype == "Data":
                comm_time += delay
                data_time += delay
            elif stype == "Compute":
                comp_time += delay
                
            if stage in ("Failure Detect", "Fail Reroute Plan"):
                fail_time += delay
                
            status_note = "OK"
            if "Fail" in stage or "Detect" in stage:
                status_note = "Failover Triggered"
                
            step_num = i + 1
            self.tree.insert("", tk.END, values=(
                step_num, stage, sender, receiver, stype, payload, 
                f"{delay:.2f}", f"{total_time:.2f}", status_note
            ))

        # Update Summary
        self.lbl_scenario_name.config(text=f"Scenario: {scenario_name}")
        self.metrics_vars["total_time"].set(f"{total_time:.2f} ms")
        self.metrics_vars["comm_sub"].set(f"{comm_time:.2f} ms")
        self.metrics_vars["comp_sub"].set(f"{comp_time:.2f} ms")
        self.metrics_vars["ctrl_sub"].set(f"{ctrl_time:.2f} ms")
        self.metrics_vars["data_sub"].set(f"{data_time:.2f} ms")
        self.metrics_vars["fail_sub"].set(f"{fail_time:.2f} ms")
        
        rec_dead = get_normal_deadline(self.config)
        self.metrics_vars["rec_dead"].set(f"{rec_dead:.2f} ms")
        self.metrics_vars["act_dead"].set(f"{active_deadline:.2f} ms")
        
        margin = active_deadline - total_time
        self.metrics_vars["margin"].set(f"{margin:.2f} ms")
        
        if margin >= 0:
            self.metrics_vars["result"].set("MET")
            self.lbl_result.config(foreground="green")
        else:
            self.metrics_vars["result"].set("MISSED")
            self.lbl_result.config(foreground="red")
            
        # Update Log
        self.txt_log.config(state=tk.NORMAL)
        self.txt_log.delete("1.0", tk.END)
        self.txt_log.insert(tk.END, log_msg)
        self.txt_log.config(state=tk.DISABLED)

if __name__ == "__main__":
    app = UAVDemoApp()
    app.mainloop()
