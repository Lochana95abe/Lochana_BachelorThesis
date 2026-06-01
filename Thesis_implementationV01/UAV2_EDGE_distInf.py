import tkinter as tk
from tkinter import ttk
import heapq

# ==========================================
# CONFIGURATION
# ==========================================
class Config:
    def __init__(self):
        # COMMUNICATION BASE VALUES
        self.link_rate_mbps = 52.24
        self.control_msg_kb = 2.0
        self.tensor1_mb = 1.61
        self.final_tensor_mb = 0.80
        self.raw_image_mb = 10.0

        # COMPUTATION BASE VALUES
        self.edge_plan_ms = 80.0
        self.capture_processing_ms = 5.67
        self.peer1_processing_ms = 6.84
        self.edge_finish_ms = 80.0

        # FAILOVER CONTROL VALUES
        self.fail_detect_ms = 50.0
        self.fail_replan_ms = 80.0

        # FALLBACK COMPUTE ASSUMPTIONS
        self.local_full_compute_ms = 27.8
        self.edge_only_full_compute_ms = 2.0
        self.reserve_margin_ms = 600.0

        # BATTERY MODEL DRAIN VALUES (%)
        self.image_capture_drain = 0.02
        self.capture_segment_processing_drain = 0.08
        self.peer1_segment_processing_drain = 0.06
        self.tensor1_send_drain = 0.70
        self.final_tensor_send_drain = 0.55
        self.tensor_receive_drain = 0.03
        self.idle_drain_per_ms = (2.8 / 60.0) / 1000.0

        # LOW BATTERY / CHARGING VALUES
        self.low_battery_threshold = 10.0
        self.return_to_work_threshold = 80.0
        self.charging_rate_percent_per_ms = (1.4286 / 60.0) / 1000.0

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
        self.start_time_ms = 0.0
        self.finish_time_ms = 0.0
        self.comm_time_ms = 0.0
        self.comp_time_ms = 0.0
        self.failover_time_ms = 0.0
        self.margin_ms = 0.0
        self.status = ""
        self.battery_used_total_until_finish = 0.0

# ==========================================
# SIMULATOR ENGINE (PIPELINE SCHEDULER)
# ==========================================
class SimulatorEngine:
    def __init__(self, app, config, scenario, num_pictures=10):
        self.app = app
        self.config = config
        self.scenario = scenario
        self.time = 0.0
        self.events = []
        self.event_id_seq = 0
        
        self.pictures = [Picture(f"Pic{i:02d}") for i in range(1, num_pictures + 1)]
        self.uavs = {f"UAV{i}": UAV(f"UAV{i}") for i in range(1, 5)}
        self.edge = Actor("Edge")
        
        self.has_failed = False
        self.is_running = True
        self.evaluation_printed = False
        self.last_pic_start_time = -999.0
        self.blocks = {name: [] for name in list(self.uavs.keys()) + ["Edge"]}
        
        if "Edge-only" in scenario:
            for u in ["UAV2", "UAV3", "UAV4"]: self.uavs[u].state = "UNAVAILABLE"
        elif "Local-only" in scenario:
            for u in ["UAV2", "UAV3", "UAV4"]: self.uavs[u].state = "UNAVAILABLE"
            self.edge.state = "UNAVAILABLE"
            
        # Calculate Deadlines dynamically
        t_ctrl = calc_control_ms(config.control_msg_kb, config.link_rate_mbps)
        t_t1 = calc_data_ms(config.tensor1_mb, config.link_rate_mbps)
        t_fin = calc_data_ms(config.final_tensor_mb, config.link_rate_mbps)
        t_raw = calc_data_ms(config.raw_image_mb, config.link_rate_mbps)

        base_normal = (t_ctrl + config.edge_plan_ms + t_ctrl + config.capture_processing_ms +
                       t_t1 + config.peer1_processing_ms +
                       t_fin + config.edge_finish_ms + t_ctrl)
        
        if "Failover" in scenario:
            base_failover = base_normal + config.fail_detect_ms + config.fail_replan_ms + t_t1
            self.active_deadline = base_failover * 1.2
        elif "Edge-only" in scenario:
            base_edge_only = t_ctrl + config.edge_plan_ms + t_raw + config.edge_only_full_compute_ms + t_ctrl
            self.active_deadline = base_edge_only * 1.2
        elif "Local-only" in scenario:
            base_local = config.fail_detect_ms + config.local_full_compute_ms
            self.active_deadline = base_local * 1.2
        else: 
            self.active_deadline = base_normal * 1.2

    def add_event(self, delay, cb):
        self.event_id_seq += 1
        heapq.heappush(self.events, (self.time + delay, self.event_id_seq, cb))

    def set_busy(self, actor, state, pic_id, stage_name):
        if actor.state != "IDLE":
            if actor.busy_start is not None and self.time > actor.busy_start:
                if actor.state in ["PROCESSING", "TRANSMITTING", "RECEIVING", "CAPTURE", "CHARGING", "WAITING"]:
                    self.blocks[actor.name].append((actor.busy_start, self.time, actor.current_pic, actor.stage_name, actor.state))
        else:
            if hasattr(actor, "battery"):
                idle_ms = self.time - actor.idle_since
                actor.drain(idle_ms * self.config.idle_drain_per_ms)
                
        actor.state = state
        actor.current_pic = pic_id
        actor.stage_name = stage_name
        actor.busy_start = self.time

    def set_idle(self, actor):
        if actor.busy_start is not None and self.time > actor.busy_start:
            if actor.state in ["PROCESSING", "TRANSMITTING", "RECEIVING", "CAPTURE", "CHARGING", "WAITING"]:
                self.blocks[actor.name].append((actor.busy_start, self.time, actor.current_pic, actor.stage_name, actor.state))
        
        actor.state = "IDLE"
        actor.current_pic = ""
        actor.stage_name = ""
        actor.idle_since = self.time
        
        if hasattr(actor, "battery") and "Normal" in self.scenario:
            if actor.battery <= self.config.low_battery_threshold:
                actor.state = "CHARGING"
                actor.busy_start = self.time
                self.app.log_msg(f"{actor.name} battery dropped below {self.config.low_battery_threshold}%; entering charging state.")
                time_to_charge = (self.config.return_to_work_threshold - actor.battery) / self.config.charging_rate_percent_per_ms
                self.add_event(time_to_charge, lambda u=actor: self.ev_charge_done(u))

    def get_best_uav(self, exclude=None):
        if exclude is None: exclude = []
        exclude_names = [u.name for u in exclude]
        candidates = []
        for u in self.uavs.values():
            if u.name in exclude_names or u.state in ["FAILED", "UNAVAILABLE", "CHARGING"]: continue
            if u.state == "IDLE":
                temp_batt = u.battery - ((self.time - u.idle_since) * self.config.idle_drain_per_ms)
                candidates.append((temp_batt, u.idle_since, u.name, u))
        if not candidates:
            return None
        candidates.sort(key=lambda x: (-x[0], x[1], x[2]))
        return candidates[0][3]

    def get_display_battery(self, uav):
        if uav.state in ["FAILED", "UNAVAILABLE"]: return None
        if uav.state == "IDLE":
            idle_ms = self.time - uav.idle_since
            return max(0.0, uav.battery - (idle_ms * self.config.idle_drain_per_ms))
        if uav.state == "CHARGING":
            charge_ms = self.time - uav.busy_start
            return min(100.0, uav.battery + (charge_ms * self.config.charging_rate_percent_per_ms))
        return uav.battery

    # ------------------ EVENT LOOP ------------------
    def start(self):
        self.app.clear_ui()
        self.app.log_msg(f"Started scenario: {self.scenario}")
        if "Edge-only" in self.scenario:
            self.app.log_msg("edge_only_total_ms = task_request_ms + plan_ms + raw_image_transfer_ms + edge_only_full_compute_ms + completion_ack_ms")
        self.try_schedule()
        self.run_next()

    def run_next(self):
        if not self.is_running: return
        if not self.events:
            self.app.update_ui_live(self)
            self.app.log_msg("All pictures completed. Pipeline finished.")
            if not self.evaluation_printed:
                self.print_final_evaluation()
                self.evaluation_printed = True
            self.is_running = False
            return
            
        t, _, cb = heapq.heappop(self.events)
        delay_sim = t - self.time
        self.time = t
        cb()
        self.try_schedule()
        self.app.update_ui_live(self)
        
        if self.app.step_mode_var.get():
            return
            
        speed = self.app.speed_var.get()
        if speed == "Instant":
            while self.events and self.is_running and not self.app.step_mode_var.get():
                t, _, cb = heapq.heappop(self.events)
                self.time = t
                cb()
                self.try_schedule()
            self.app.update_ui_live(self)
            if not self.events:
                self.app.log_msg("All pictures completed. Pipeline finished.")
                if not self.evaluation_printed:
                    self.print_final_evaluation()
                    self.evaluation_printed = True
                self.is_running = False
        else:
            if speed == "Fast": real_delay = max(1, int(delay_sim / 10.0))
            elif speed == "Slow": real_delay = max(1, int(delay_sim * 2.0))
            else: real_delay = max(1, int(delay_sim / 3.0)) # Normal
            self.app.after(real_delay, self.run_next)

    # ------------------ SCHEDULER ------------------
    def check_idle_batteries(self):
        if "Normal" not in self.scenario: return
        for u in self.uavs.values():
            if u.state == "IDLE":
                temp_batt = u.battery - ((self.time - u.idle_since) * self.config.idle_drain_per_ms)
                if temp_batt <= self.config.low_battery_threshold:
                    u.battery = temp_batt
                    u.state = "CHARGING"
                    u.busy_start = self.time
                    self.app.log_msg(f"{u.name} battery dropped below {self.config.low_battery_threshold}%; entering charging state.")
                    time_to_charge = (self.config.return_to_work_threshold - u.battery) / self.config.charging_rate_percent_per_ms
                    self.add_event(time_to_charge, lambda actor=u: self.ev_charge_done(actor))

    def try_schedule(self):
        self.check_idle_batteries()
        
        usable_uavs = sum(1 for u in self.uavs.values() if u.state not in ["FAILED", "UNAVAILABLE", "CHARGING"])
        max_active = max(1, usable_uavs - 1)
        if "Local-only" in self.scenario or "Edge-only" in self.scenario:
            max_active = usable_uavs
            
        for pic in self.pictures:
            if pic.state == "DONE": continue
            
            if pic.state == "INIT":
                active_requiring = sum(1 for p in self.pictures if p.state not in [
                    "INIT", "DONE", 
                    "SEND_T1", "PEER1_PROC", "WAIT_EDGE", "SEND_FINAL", "EDGE_FINISH", "ACK",
                    "SEND_RAW", "EDGE_FULL", "LOCAL_DETECT", "LOCAL_PROC"
                ])
                
                if active_requiring >= max_active:
                    continue 
                
                if self.time < self.last_pic_start_time + 284.0:
                    continue
                
                if "Local-only" in self.scenario:
                    cap = self.get_best_uav()
                    if cap:
                        self.set_busy(cap, "WAITING", pic.id, "Fail Detect")
                        pic.roles["Capture"] = cap
                        pic.state = "LOCAL_DETECT"
                        pic.start_time_ms = self.time
                        self.last_pic_start_time = self.time
                        self.add_event(284.0, lambda: None)
                        self.app.log_msg(f"{pic.id} started locally on {cap.name} (Peers & Edge Unavailable)")
                        self.add_event(self.config.fail_detect_ms, lambda p=pic: self.ev_local_detect_done(p))
                else:
                    exclude_cap = []
                    if "Failover" in self.scenario and not self.has_failed:
                        exclude_cap.append(self.uavs["UAV2"])
                    
                    cap = self.get_best_uav(exclude=exclude_cap)
                    if cap and self.edge.state == "IDLE":
                        self.set_busy(cap, "TRANSMITTING", pic.id, "Req")
                        self.set_busy(self.edge, "RECEIVING", pic.id, "Req")
                        pic.roles["Capture"] = cap
                        pic.state = "REQ"
                        pic.start_time_ms = self.time
                        self.last_pic_start_time = self.time
                        self.add_event(284.0, lambda: None)
                        self.app.log_msg(f"{pic.id} selected {cap.name} as Capture UAV because it had the highest available battery ({cap.battery:.1f}%).")
                        delay = calc_control_ms(self.config.control_msg_kb, self.config.link_rate_mbps)
                        self.add_event(delay, lambda p=pic: self.ev_req_done(p))

            elif pic.state == "WAIT_PEER1":
                peer = self.get_best_uav(exclude=[pic.roles["Capture"]])
                if peer:
                    if "Failover" in self.scenario and peer.name == "UAV2" and not self.has_failed:
                        self.has_failed = True
                        pic.roles["FailedPeer"] = peer
                        cap = pic.roles["Capture"]
                        self.set_busy(cap, "TRANSMITTING", pic.id, "Send T1 (Fail)")
                        self.set_busy(peer, "RECEIVING", pic.id, "Recv T1 (Fail)")
                        pic.state = "ATTEMPTING_FAIL"
                        self.app.log_msg(f"{pic.id} attempted transfer to {peer.name}.")
                        delay = calc_data_ms(self.config.tensor1_mb, self.config.link_rate_mbps)
                        self.add_event(delay, lambda p=pic: self.ev_transfer_failed(p))
                    else:
                        cap = pic.roles["Capture"]
                        self.set_busy(cap, "TRANSMITTING", pic.id, "Send T1")
                        self.set_busy(peer, "RECEIVING", pic.id, "Recv T1")
                        pic.roles["Peer1"] = peer
                        pic.state = "SEND_T1"
                        self.app.log_msg(f"{pic.id} selected {peer.name} as Peer 1.")
                        delay = calc_data_ms(self.config.tensor1_mb, self.config.link_rate_mbps)
                        self.add_event(delay, lambda p=pic: self.ev_send_t1_done(p))

            elif pic.state == "WAIT_EDGE":
                if self.edge.state == "IDLE":
                    p1 = pic.roles["Peer1"]
                    self.set_busy(p1, "TRANSMITTING", pic.id, "Send Fin")
                    self.set_busy(self.edge, "RECEIVING", pic.id, "Recv Fin")
                    pic.state = "SEND_FINAL"
                    delay = calc_data_ms(self.config.final_tensor_mb, self.config.link_rate_mbps)
                    self.add_event(delay, lambda p=pic: self.ev_send_final_done(p))

    # ------------------ EVENT CALLBACKS ------------------
    def log_step(self, pic, step_name, sender, receiver, stype, payload, delay, category, note=""):
        pic.cum_time += delay
        pic.step_counter += 1
        
        if category == "comm": pic.comm_time_ms += delay
        elif category == "comp": pic.comp_time_ms += delay
        elif category == "failover": pic.failover_time_ms += delay
            
        self.app.add_tree_row(pic.id, pic.step_counter, step_name, sender, receiver, stype, payload, delay, pic.cum_time, note)

    def print_pic_summary(self, pic):
        pic.finish_time_ms = self.time
        total_time_ms = pic.finish_time_ms - pic.start_time_ms
        margin = self.active_deadline - total_time_ms
        status = "MET" if margin >= 0 else "MISSED"
        
        pic.margin_ms = margin
        pic.status = status
        pic.battery_used_total_until_finish = sum(100.0 - u.battery for u in self.uavs.values() if u.state != "UNAVAILABLE")
        
        self.app.log_msg(f"=== {pic.id} COMPLETE ===")
        self.app.log_msg(f"Time: {total_time_ms:.1f}ms | Deadline: {self.active_deadline:.1f}ms | Margin: {margin:.1f}ms ({status})")

    def ev_charge_done(self, uav):
        uav.battery = self.config.return_to_work_threshold
        self.set_idle(uav)
        self.app.log_msg(f"{uav.name} reached {self.config.return_to_work_threshold}%; rejoining candidate list.")

    def ev_transfer_failed(self, pic):
        cap = pic.roles["Capture"]
        failed_peer = pic.roles["FailedPeer"]
        delay = calc_data_ms(self.config.tensor1_mb, self.config.link_rate_mbps)
        self.log_step(pic, f"Tensor 1 ({cap.name}->{failed_peer.name})", cap.name, failed_peer.name, "Data", f"{self.config.tensor1_mb} MB", delay, "failover", f"{failed_peer.name} unresponsive")
        
        self.set_idle(failed_peer)
        failed_peer.state = "FAILED"
        self.app.log_msg(f"{failed_peer.name} did not respond. Starting failure detection.")
        
        self.set_busy(cap, "WAITING", pic.id, "Fail Detect")
        self.add_event(self.config.fail_detect_ms, lambda p=pic: self.ev_fail_detect_done(p))

    def ev_fail_detect_done(self, pic):
        cap = pic.roles["Capture"]
        self.log_step(pic, "Failure Detect", "System", "System", "Control", "-", self.config.fail_detect_ms, "failover", "Detected peer failure")
        self.set_busy(self.edge, "PROCESSING", pic.id, "Replan")
        self.set_busy(cap, "WAITING", pic.id, "Replan")
        self.app.log_msg(f"Failure detected. Edge is planning reroute for {pic.id}...")
        self.add_event(self.config.fail_replan_ms, lambda p=pic: self.ev_replan_done(p))

    def ev_replan_done(self, pic):
        self.log_step(pic, "Reroute Plan", "Edge", "Edge", "Compute", "-", self.config.fail_replan_ms, "failover", "Reroute planned")
        self.set_idle(self.edge)
        self.app.log_msg(f"Rerouting {pic.id}. Looking for replacement peer...")
        pic.state = "WAIT_PEER1"

    def ev_req_done(self, pic):
        cap = pic.roles["Capture"]
        cap.drain(self.config.image_capture_drain)
        delay = calc_control_ms(self.config.control_msg_kb, self.config.link_rate_mbps)
        self.log_step(pic, "Task Request", cap.name, "Edge", "Control", f"{self.config.control_msg_kb} KB", delay, "comm", "Requesting plan")
        
        if "Edge-only" in self.scenario:
            self.set_busy(cap, "WAITING", pic.id, "Wait Plan")
            self.set_busy(self.edge, "PROCESSING", pic.id, "Edge Plan")
            self.add_event(self.config.edge_plan_ms, lambda p=pic: self.ev_edge_only_plan_done(p))
        else:
            self.set_busy(cap, "WAITING", pic.id, "Wait Plan")
            self.set_busy(self.edge, "PROCESSING", pic.id, "Plan Gen")
            self.add_event(self.config.edge_plan_ms, lambda p=pic: self.ev_plan_gen_done(p))

    def ev_plan_gen_done(self, pic):
        self.log_step(pic, "Global Plan Gen", "Edge", "Edge", "Compute", "-", self.config.edge_plan_ms, "comp", "Plan generated")
        cap = pic.roles["Capture"]
        self.set_busy(self.edge, "TRANSMITTING", pic.id, "Plan Dist")
        self.set_busy(cap, "RECEIVING", pic.id, "Plan Dist")
        delay = calc_control_ms(self.config.control_msg_kb, self.config.link_rate_mbps)
        self.add_event(delay, lambda p=pic: self.ev_plan_dist_done(p))

    def ev_plan_dist_done(self, pic):
        delay = calc_control_ms(self.config.control_msg_kb, self.config.link_rate_mbps)
        self.log_step(pic, "Plan Dist", "Edge", "Nodes", "Control", f"{self.config.control_msg_kb} KB", delay, "comm", "Plan received")
        self.set_idle(self.edge)
        cap = pic.roles["Capture"]
        self.set_busy(cap, "CAPTURE", pic.id, "Seg1")
        self.app.log_msg(f"{pic.id} Segment 1 completed on {cap.name}.")
        self.add_event(self.config.capture_processing_ms, lambda p=pic: self.ev_cap_proc_done(p))

    def ev_cap_proc_done(self, pic):
        cap = pic.roles["Capture"]
        cap.drain(self.config.capture_segment_processing_drain)
        self.log_step(pic, f"Seg1 Proc ({cap.name})", cap.name, cap.name, "Compute", "-", self.config.capture_processing_ms, "comp", "Captured & processed")
        self.set_busy(cap, "WAITING", pic.id, "Wait P1")
        pic.state = "WAIT_PEER1"

    def ev_send_t1_done(self, pic):
        cap = pic.roles["Capture"]
        peer = pic.roles["Peer1"]
        cap.drain(self.config.tensor1_send_drain)
        peer.drain(self.config.tensor_receive_drain)
        delay = calc_data_ms(self.config.tensor1_mb, self.config.link_rate_mbps)
        self.log_step(pic, f"Tensor 1 ({cap.name}->{peer.name})", cap.name, peer.name, "Data", f"{self.config.tensor1_mb} MB", delay, "comm", "Sent onward")
        self.set_idle(cap)
        self.set_busy(peer, "PROCESSING", pic.id, "Seg2")
        self.app.log_msg(f"{pic.id} Tensor1 sent. {cap.name} is now available. {pic.id} Seg2 running on {peer.name}.")
        self.add_event(self.config.peer1_processing_ms, lambda p=pic: self.ev_peer1_proc_done(p))

    def ev_peer1_proc_done(self, pic):
        peer = pic.roles["Peer1"]
        peer.drain(self.config.peer1_segment_processing_drain)
        self.log_step(pic, f"Seg2 Proc ({peer.name})", peer.name, peer.name, "Compute", "-", self.config.peer1_processing_ms, "comp", "Segment 2 processed")
        self.set_busy(peer, "WAITING", pic.id, "Wait Edge")
        pic.state = "WAIT_EDGE"

    def ev_send_final_done(self, pic):
        p1 = pic.roles["Peer1"]
        p1.drain(self.config.final_tensor_send_drain)
        delay = calc_data_ms(self.config.final_tensor_mb, self.config.link_rate_mbps)
        self.log_step(pic, f"Send Final ({p1.name}->Edge)", p1.name, "Edge", "Data", f"{self.config.final_tensor_mb} MB", delay, "comm", "To coordinator")
        self.set_idle(p1)
        self.set_busy(self.edge, "PROCESSING", pic.id, "Edge Fin")
        self.app.log_msg(f"{pic.id} Final Tensor sent. {p1.name} is now available.")
        self.add_event(self.config.edge_finish_ms, lambda p=pic: self.ev_edge_finish_done(p))

    def ev_edge_finish_done(self, pic):
        self.log_step(pic, "Edge Finish", "Edge", "Edge", "Compute", "-", self.config.edge_finish_ms, "comp", "Model finished")
        self.set_busy(self.edge, "TRANSMITTING", pic.id, "Ack")
        delay = calc_control_ms(self.config.control_msg_kb, self.config.link_rate_mbps)
        self.add_event(delay, lambda p=pic: self.ev_ack_done(p))

    def ev_ack_done(self, pic):
        delay = calc_control_ms(self.config.control_msg_kb, self.config.link_rate_mbps)
        cap_name = pic.roles["Capture"].name
        self.log_step(pic, "Completion Ack", "Edge", cap_name, "Control", f"{self.config.control_msg_kb} KB", delay, "comm", "Done")
        self.set_idle(self.edge)
        pic.state = "DONE"
        self.print_pic_summary(pic)

    # Edge-Only & Local-Only Overrides
    def ev_edge_only_plan_done(self, pic):
        self.log_step(pic, "Edge Plan (Fallback)", "Edge", "Edge", "Compute", "-", self.config.edge_plan_ms, "comp", "Plan generated")
        cap = pic.roles["Capture"]
        self.set_busy(cap, "TRANSMITTING", pic.id, "Send Raw")
        self.set_busy(self.edge, "RECEIVING", pic.id, "Recv Raw")
        delay = calc_data_ms(self.config.raw_image_mb, self.config.link_rate_mbps)
        self.add_event(delay, lambda p=pic: self.ev_send_raw_done(p))

    def ev_send_raw_done(self, pic):
        cap = pic.roles["Capture"]
        cap.drain(self.config.tensor1_send_drain)
        delay = calc_data_ms(self.config.raw_image_mb, self.config.link_rate_mbps)
        self.log_step(pic, f"Send Raw Image ({cap.name})", cap.name, "Edge", "Data", f"{self.config.raw_image_mb} MB", delay, "comm", "Offloaded")
        self.set_idle(cap)
        self.set_busy(self.edge, "PROCESSING", pic.id, "Edge Full")
        self.add_event(self.config.edge_only_full_compute_ms, lambda p=pic: self.ev_edge_full_done(p))

    def ev_edge_full_done(self, pic):
        self.log_step(pic, "Edge Full Process", "Edge", "Edge", "Compute", "-", self.config.edge_only_full_compute_ms, "comp", "Finished locally at Edge")
        self.set_busy(self.edge, "TRANSMITTING", pic.id, "Ack")
        delay = calc_control_ms(self.config.control_msg_kb, self.config.link_rate_mbps)
        self.add_event(delay, lambda p=pic: self.ev_ack_done(p))

    def ev_local_detect_done(self, pic):
        cap = pic.roles["Capture"]
        self.log_step(pic, "Detect Disconnect", cap.name, cap.name, "Control", "-", self.config.fail_detect_ms, "failover", "No Edge/Peers")
        self.set_busy(cap, "PROCESSING", pic.id, "Local Proc")
        
        local_time = self.config.capture_processing_ms + self.config.peer1_processing_ms
        self.add_event(local_time, lambda p=pic: self.ev_local_proc_done(p, local_time))

    def ev_local_proc_done(self, pic, local_time):
        cap = pic.roles["Capture"]
        cap.drain(self.config.capture_segment_processing_drain + self.config.peer1_segment_processing_drain)
        self.log_step(pic, f"Local Full Process ({cap.name})", cap.name, cap.name, "Compute", "-", local_time, "comp", "Fully local execution")
        self.set_idle(cap)
        pic.state = "DONE"
        self.log_step(pic, "Local Complete", cap.name, cap.name, "Compute", "-", 0.0, "comp", "Done")
        self.print_pic_summary(pic)

    def print_final_evaluation(self):
        completed_pics = [p for p in self.pictures if p.state == "DONE"]
        if not completed_pics: return
        
        sim_time = self.time
        count = len(completed_pics)
        
        total_comm = sum(p.comm_time_ms for p in completed_pics)
        total_comp = sum(p.comp_time_ms for p in completed_pics)
        total_fail = sum(p.failover_time_ms for p in completed_pics)
        
        comp_times = [(p.finish_time_ms - p.start_time_ms) for p in completed_pics]
        avg_comp = sum(comp_times) / count
        min_comp = min(comp_times)
        max_comp = max(comp_times)
        
        met_count = sum(1 for p in completed_pics if p.status == "MET")
        missed_count = count - met_count
        success_rate = (met_count / count) * 100.0
        avg_margin = sum(p.margin_ms for p in completed_pics) / count
        throughput = count / (sim_time / 1000.0) if sim_time > 0 else 0.0
        
        battery_usage = {}
        battery_state = {}
        for name, uav in self.uavs.items():
            if uav.state in ["UNAVAILABLE"]:
                battery_usage[name] = 0.0
                battery_state[name] = "UNAVAILABLE"
            else:
                battery_usage[name] = 100.0 - uav.battery
                battery_state[name] = uav.state
                
        self.app.log_msg("\n" + "="*50)
        self.app.log_msg("FINAL EVALUATION SUMMARY")
        self.app.log_msg("="*50)
        self.app.log_msg(f"1. Scenario name: {self.scenario}")
        self.app.log_msg(f"2. Completed pictures: {count}")
        self.app.log_msg(f"3. Total simulation time: {sim_time:.2f} ms")
        self.app.log_msg(f"4. Avg completion time: {avg_comp:.2f} ms")
        self.app.log_msg(f"5. Min completion time: {min_comp:.2f} ms")
        self.app.log_msg(f"6. Max completion time: {max_comp:.2f} ms")
        self.app.log_msg(f"7. Deadline met: {met_count}")
        self.app.log_msg(f"8. Deadline missed: {missed_count}")
        self.app.log_msg(f"9. Success rate: {success_rate:.2f}%")
        self.app.log_msg(f"10. Avg deadline margin: {avg_margin:.2f} ms")
        self.app.log_msg(f"11. Total communication time: {total_comm:.2f} ms")
        self.app.log_msg(f"12. Total computation time: {total_comp:.2f} ms")
        self.app.log_msg(f"13. Total failover overhead: {total_fail:.2f} ms")
        self.app.log_msg(f"14. Pipeline throughput: {throughput:.2f} pics/sec")
        self.app.log_msg(f"(Note: Total Comm + Comp time may exceed Simulation Time due to pipelined concurrent execution)")
        
        self.app.log_msg("\n15. Per-UAV Battery Usage:")
        for k, v in battery_usage.items():
            self.app.log_msg(f"   {k}: {v:.2f}% used")
            
        self.app.log_msg("\n16. Per-UAV Final State:")
        for k, v in battery_state.items():
            self.app.log_msg(f"   {k}: {v}")
        self.app.log_msg("="*50 + "\n")
        
        self.export_csv(completed_pics, sim_time, avg_comp, min_comp, max_comp, met_count, missed_count, success_rate, avg_margin, total_comm, total_comp, total_fail, throughput)

    def export_csv(self, completed_pics, sim_time, avg_comp, min_comp, max_comp, met_count, missed_count, success_rate, avg_margin, total_comm, total_comp, total_fail, throughput):
        tag = self.scenario.replace(" ", "_").replace("/", "").lower()
        file_pics = f"results_{tag}_pictures.csv"
        file_summary = f"results_{tag}_summary.csv"
        import csv
        
        with open(file_pics, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["scenario", "picture_id", "start_time_ms", "finish_time_ms", "total_time_ms", "deadline_ms", "deadline_margin_ms", "deadline_result", "communication_time_ms", "computation_time_ms", "failover_overhead_ms", "capture_uav", "peer_uav", "edge_used", "local_only", "edge_only", "battery_used_total_until_picture_finish"])
            
            for p in completed_pics:
                tot = p.finish_time_ms - p.start_time_ms
                cap = p.roles.get("Capture").name if p.roles.get("Capture") else "None"
                peer = p.roles.get("Peer1").name if p.roles.get("Peer1") else "None"
                edge_used = "Yes" if "Edge-only" in self.scenario or "Distributed" in self.scenario or "Failover" in self.scenario else "No"
                local_only = "Yes" if "Local-only" in self.scenario else "No"
                edge_only = "Yes" if "Edge-only" in self.scenario else "No"
                
                writer.writerow([
                    self.scenario, p.id, f"{p.start_time_ms:.2f}", f"{p.finish_time_ms:.2f}", f"{tot:.2f}",
                    f"{self.active_deadline:.2f}", f"{p.margin_ms:.2f}", p.status,
                    f"{p.comm_time_ms:.2f}", f"{p.comp_time_ms:.2f}", f"{p.failover_time_ms:.2f}",
                    cap, peer, edge_used, local_only, edge_only, f"{p.battery_used_total_until_finish:.4f}"
                ])
                
        with open(file_summary, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["scenario", "completed_pictures", "total_simulation_time_ms", "average_completion_time_ms", "min_completion_time_ms", "max_completion_time_ms", "deadline_met_count", "deadline_missed_count", "deadline_success_rate_percent", "average_deadline_margin_ms", "total_communication_time_ms", "total_computation_time_ms", "total_failover_overhead_ms", "throughput_pictures_per_second"])
            writer.writerow([
                self.scenario, len(completed_pics), f"{sim_time:.2f}", f"{avg_comp:.2f}", f"{min_comp:.2f}", f"{max_comp:.2f}",
                met_count, missed_count, f"{success_rate:.2f}", f"{avg_margin:.2f}", f"{total_comm:.2f}", f"{total_comp:.2f}", f"{total_fail:.2f}", f"{throughput:.2f}"
            ])
            
        self.app.log_msg(f"CSV exports successfully saved to disk:\n -> {file_pics}\n -> {file_summary}")

# ==========================================
# TKINTER UI APPLICATION
# ==========================================
class UAVDemoAppV4_1(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Pipelined UAV Swarm Inference Demo V4.1")
        self.geometry("1450x950")
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
        
        row1 = ttk.Frame(ctrl_bar)
        row1.pack(fill=tk.X, pady=2)
        ttk.Label(row1, text="Scenario:").pack(side=tk.LEFT, padx=2)
        self.scenario_var = tk.StringVar(value="Normal Distributed Inference")
        scenarios = ["Normal Distributed Inference", "Failover / Reroute Scenario", 
                     "Edge-only Fallback Scenario", "Local-only Fallback Scenario"]
        ttk.Combobox(row1, textvariable=self.scenario_var, values=scenarios, state="readonly", width=30).pack(side=tk.LEFT, padx=2)
        
        row1b = ttk.Frame(ctrl_bar)
        row1b.pack(fill=tk.X, pady=2)
        ttk.Label(row1b, text="Pictures:").pack(side=tk.LEFT, padx=(2, 2))
        self.pic_count_var = tk.IntVar(value=10)
        ttk.Entry(row1b, textvariable=self.pic_count_var, width=5).pack(side=tk.LEFT)
        
        row2 = ttk.Frame(ctrl_bar)
        row2.pack(fill=tk.X, pady=(10, 2))
        ttk.Button(row2, text="Run Pipeline", command=self.run_scenario).pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)
        ttk.Button(row2, text="Reset", command=self.reset_defaults).pack(side=tk.LEFT, padx=2)
        
        row3 = ttk.Frame(ctrl_bar)
        row3.pack(fill=tk.X, pady=5)
        self.step_mode_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(row3, text="Step-by-Step", variable=self.step_mode_var).pack(side=tk.LEFT, padx=2)
        ttk.Button(row3, text="Next Step", command=self.do_next_step).pack(side=tk.LEFT, padx=5)
        
        row4 = ttk.Frame(ctrl_bar)
        row4.pack(fill=tk.X, pady=2)
        ttk.Label(row4, text="Speed:").pack(side=tk.LEFT, padx=2)
        self.speed_var = tk.StringVar(value="Normal")
        ttk.Combobox(row4, textvariable=self.speed_var, values=["Slow", "Normal", "Fast", "Instant"], state="readonly", width=10).pack(side=tk.LEFT)
        
        self._build_config_form(config_frame)
        
        # 2b. Treeview
        tree_frame = ttk.LabelFrame(mid_frame, text="Event Pipeline (All Pictures)", padding=5)
        tree_frame.grid(row=0, column=1, sticky="nsew", pady=(0, 5))
        
        columns = ("Pic ID", "Step", "Stage Name", "Sender", "Receiver", "Type", "Payload", "Delay (ms)", "Cum. Time (ms)", "Note")
        self.tree = ttk.Treeview(tree_frame, columns=columns, show="headings", selectmode="none")
        widths = [50, 40, 160, 70, 70, 60, 60, 70, 90, 180]
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
        self.txt_log = tk.Text(log_frame, height=9, bg="#f9f9f9", font=("Consolas", 9))
        self.txt_log.pack(fill=tk.BOTH, expand=True)
        self.txt_log.config(state=tk.DISABLED)

        # 3. GANTT CHART BOTTOM
        gantt_frame = ttk.LabelFrame(self, text="Pipeline Timeline (Gantt Chart)", padding=5)
        gantt_frame.grid(row=2, column=0, sticky="nsew", pady=(10, 0))
        self.rowconfigure(2, weight=1)
        
        progress_frame = ttk.Frame(gantt_frame)
        progress_frame.pack(side=tk.TOP, fill=tk.X, pady=2)
        self.lbl_progress = ttk.Label(progress_frame, text="Progress: 0/0 Completed | Active: 0 | Remaining: 0", font=("Segoe UI", 9, "bold"))
        self.lbl_progress.pack(side=tk.LEFT)
        
        self.canvas = tk.Canvas(gantt_frame, bg="white", height=200)
        self.canvas.pack(fill=tk.BOTH, expand=True)

    def _build_config_form(self, parent):
        fields = [
            ("link_rate_mbps", "Link Rate (Mbps)"), ("control_msg_kb", "Control Msg (KB)"),
            ("tensor1_mb", "Tensor 1 (MB)"), ("final_tensor_mb", "Final Tensor (MB)"),
            ("edge_plan_ms", "Edge Plan (ms)"), ("capture_processing_ms", "Capture Proc (ms)"),
            ("peer1_processing_ms", "Peer 1 Proc (ms)"),
            ("fail_detect_ms", "Fail Detect (ms)"), ("fail_replan_ms", "Fail Replan (ms)"),
            ("edge_only_full_compute_ms", "Edge-only Comp (ms)"), ("local_full_compute_ms", "Local Comp (ms)")
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
            if hasattr(default_config, attr):
                var.set(getattr(default_config, attr))

    def _sync_config_from_ui(self):
        for attr, var in self.vars.items():
            try: setattr(self.config, attr, float(var.get()))
            except ValueError: pass

    def run_scenario(self):
        if self.engine and self.engine.is_running:
            self.engine.is_running = False
        self._sync_config_from_ui()
        
        count = self.pic_count_var.get()
        if count < 1: count = 1
        elif count > 20: count = 20
        
        self.engine = SimulatorEngine(self, self.config, self.scenario_var.get(), num_pictures=count)
        self.engine.start()

    def do_next_step(self):
        if self.engine and self.engine.is_running:
            self.engine.run_next()

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
            if batt is None:
                self.top_vars[name]["battery"].set("Bat: N/A")
            else:
                self.top_vars[name]["battery"].set(f"Bat: {batt:.2f}%")
                
            self.top_vars[name]["state"].set(f"State: {uav.state}")
            task_str = f"{uav.current_pic} {uav.stage_name}".strip()
            self.top_vars[name]["task"].set(f"Task: {task_str if task_str else 'None'}")
        
        if engine.edge.state in ["UNAVAILABLE", "FAILED"]:
            self.top_vars["Edge"]["battery"].set("Bat: N/A")
        else:
            self.top_vars["Edge"]["battery"].set("Bat: 100.00%")
            
        self.top_vars["Edge"]["state"].set(f"State: {engine.edge.state}")
        tstr = f"{engine.edge.current_pic} {engine.edge.stage_name}".strip()
        self.top_vars["Edge"]["task"].set(f"Task: {tstr if tstr else 'None'}")
        
        completed = sum(1 for p in engine.pictures if p.state == "DONE")
        active = sum(1 for p in engine.pictures if p.state not in ["INIT", "DONE"])
        remaining = len(engine.pictures) - completed - active
        self.lbl_progress.config(text=f"Progress: {completed}/{len(engine.pictures)} Completed | Active: {active} | Remaining: {remaining}")
        
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
        
        colors = {
            "Pic01": "#87CEFA", "Pic02": "#98FB98", "Pic03": "#F08080",
            "Pic04": "#DDA0DD", "Pic05": "#F0E68C", "Pic06": "#FFB6C1",
            "Pic07": "#20B2AA", "Pic08": "#FFA07A", "Pic09": "#87CEEB",
            "Pic10": "#9370DB"
        }
        
        for i, a in enumerate(actors):
            y = i * row_h
            self.canvas.create_text(10, y + row_h/2, text=a, anchor="w", font=("Segoe UI", 10, "bold"))
            self.canvas.create_line(80, y + row_h, w, y + row_h, fill="#ccc")
            
            blks = list(engine.blocks[a])
            curr_actor = engine.uavs[a] if a in engine.uavs else engine.edge
            if curr_actor.state not in ["IDLE", "FAILED", "UNAVAILABLE"] and curr_actor.busy_start is not None:
                blks.append((curr_actor.busy_start, engine.time, curr_actor.current_pic, curr_actor.stage_name, curr_actor.state))
                
            for start, end, pid, stage, state in blks:
                x1 = 80 + start * scale
                x2 = 80 + end * scale
                if x2 - x1 < 2: x2 = x1 + 2
                
                color = colors.get(pid, "lightgray")
                outline = "black"
                dash = None
                
                if state == "WAITING":
                    color = "#F0F0F0"
                    outline = "#AAAAAA"
                    dash = (2,2)
                elif state == "CHARGING":
                    color = "#FFD700" # Gold for charging
                elif state == "FAILED" or "Fail" in stage:
                    color = "#FF4500" # Red for failed attempt
                
                self.canvas.create_rectangle(x1, y+5, x2, y+row_h-5, fill=color, outline=outline, dash=dash)
                
                if (x2 - x1) > 40 and state != "WAITING":
                    text = "Charging" if state == "CHARGING" else stage
                    self.canvas.create_text(x1 + (x2-x1)/2, y + row_h/2, text=text, font=("Segoe UI", 8))

if __name__ == "__main__":
    app = UAVDemoAppV4_1()
    app.mainloop()
