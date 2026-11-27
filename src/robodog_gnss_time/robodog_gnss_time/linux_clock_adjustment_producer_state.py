"""Pure-Python reference model for the proposed BPF producer state machine.

This model validates bounded stack and emission-accounting semantics only. It is
not a BPF implementation and does not model kernel scheduling or verifier rules.
"""


class ClockAdjustmentProducerState:
    def __init__(self, max_syscall_depth=2, max_handler_depth=2,
                 capture_session_id="capture-01", boot_id="boot-01",
                 task_instance_seq=1):
        if max_syscall_depth < 1 or max_handler_depth < 1:
            raise ValueError("stack depths must be positive")
        self.max_syscall_depth = max_syscall_depth
        self.max_handler_depth = max_handler_depth
        self.capture_session_id = capture_session_id
        self.boot_id = boot_id
        self.task_instance_seq = task_instance_seq
        self.next_invocation_seq = 1
        self.next_event_id = 1
        self.syscall_stack = []
        self.handler_stack = []
        self.syscall_overflow_depth = 0
        self.handler_overflow_depth = 0
        self.active_syscall_invocations = 0
        self.active_handler_invocations = 0
        self.inflight_bpf_callbacks = 0
        self.task_context_reservations = 1
        self.counters = self._zero_counters()
        self.counters.update({
            "task_context_admission_attempts": 1,
            "task_context_admitted": 1,
            "task_context_create_attempts": 1,
            "task_context_create_successes": 1,
            "task_context_init_attempts": 1,
            "task_context_init_successes": 1,
            "task_instance_id_allocation_attempts": 1,
            "task_instance_id_allocation_successes": 1,
        })
        self.sequence = {}
        self.per_cpu = {}
        self.emitted = []
        self.task_exited = False
        self.producer_detached = False

    @staticmethod
    def _zero_counters():
        names = (
            "event_id_allocation_attempts", "event_id_allocation_successes",
            "event_id_allocation_failures",
            "task_instance_id_allocation_attempts", "task_instance_id_allocation_successes",
            "task_instance_id_allocation_failures",
            "task_context_admission_attempts", "task_context_admitted",
            "task_context_capacity_rejected",
            "task_context_create_attempts", "task_context_create_successes",
            "task_context_create_failures",
            "task_context_init_attempts", "task_context_init_successes",
            "task_context_init_failures",
            "syscall_invocation_allocation_attempts",
            "syscall_invocation_allocation_successes",
            "syscall_invocation_allocation_failures",
            "handler_invocation_allocation_attempts",
            "handler_invocation_allocation_successes",
            "handler_invocation_allocation_failures",
            "cpu_sequence_allocation_failures",
            "syscall_context_stack_overflow", "syscall_context_pop_mismatch",
            "handler_context_stack_overflow", "handler_context_pop_mismatch",
            "handler_parent_context_unavailable", "task_exit_observed",
            "task_exit_with_pending_syscalls", "task_exit_with_pending_handlers",
        )
        return {name: 0 for name in names}

    def _allocate_invocation(self, kind, success=True):
        prefix = kind + "_invocation_allocation_"
        self.counters[prefix + "attempts"] += 1
        if not success or self.next_invocation_seq >= (1 << 64) - 1:
            self.counters[prefix + "failures"] += 1
            return None
        value = self.next_invocation_seq
        self.next_invocation_seq += 1
        self.counters[prefix + "successes"] += 1
        return f"{value:016x}"

    def _emit(self, stream_kind, event_type, name, invocation_id, cpu,
              monotonic_ns, payload, parent_id=None, parent_status=None,
              event_id_ok=True, sequence_ok=True, reserve_ok=True):
        self.counters["event_id_allocation_attempts"] += 1
        if not event_id_ok or self.next_event_id >= (1 << 64) - 1:
            self.counters["event_id_allocation_failures"] += 1
            return None
        event_id = self.next_event_id
        self.next_event_id += 1
        self.counters["event_id_allocation_successes"] += 1

        producer_id = "clock-step-syscall" if stream_kind == "syscall" else "clock-step-handler"
        sequence_key = (producer_id, cpu)
        self.sequence.setdefault(sequence_key, 0)
        if not sequence_ok:
            self.counters["cpu_sequence_allocation_failures"] += 1
            return None
        self.sequence[sequence_key] += 1
        sequence = self.sequence[sequence_key]
        cpu_counter = self.per_cpu.setdefault(sequence_key, {
            "producer_id": producer_id, "boot_id": self.boot_id, "cpu": cpu,
            "sequence_allocated": 0, "ringbuf_reserve_failures": 0,
            "ringbuf_submit_successes": 0,
        })
        cpu_counter["sequence_allocated"] += 1
        if not reserve_ok:
            cpu_counter["ringbuf_reserve_failures"] += 1
            return None
        cpu_counter["ringbuf_submit_successes"] += 1

        record = {
            "schema_version": 2,
            "capture_session_id": self.capture_session_id,
            "boot_id": self.boot_id,
            "producer_id": producer_id,
            "stream_id": stream_kind + "-stream",
            "stream_kind": stream_kind,
            "cpu": cpu,
            "sequence": sequence,
            "monotonic_ns": monotonic_ns,
            "task_instance_id": f"{self.task_instance_seq:016x}",
            "event_id": f"{event_id:016x}",
            "invocation_id": invocation_id,
            "payload": {"event_type": event_type, **payload},
        }
        if stream_kind == "kernel_handler":
            record["parent_context_status"] = parent_status
            if parent_status == "matched":
                record["parent_syscall_invocation_id"] = parent_id
        self.emitted.append(record)
        return record

    def syscall_enter(self, name, cpu, monotonic_ns, raw_clockid=None,
                      event_id_ok=True, sequence_ok=True, reserve_ok=True,
                      invocation_id_ok=True):
        if self.task_exited:
            raise RuntimeError("task context already exited")
        self.active_syscall_invocations += 1
        if self.syscall_overflow_depth or len(self.syscall_stack) >= self.max_syscall_depth:
            self.syscall_overflow_depth += 1
            self.counters["syscall_context_stack_overflow"] += 1
            return None
        invocation_id = self._allocate_invocation("syscall", invocation_id_ok)
        if invocation_id is None:
            self.syscall_overflow_depth += 1
            return None
        payload = {"pid_tgid": 4294967297, "syscall": name,
                   "modes_read_status": "not_requested"}
        if raw_clockid is not None:
            payload["raw_clockid"] = raw_clockid
        event = self._emit("syscall", "syscall_enter", name, invocation_id,
                           cpu, monotonic_ns, payload, event_id_ok=event_id_ok,
                           sequence_ok=sequence_ok, reserve_ok=reserve_ok)
        self.syscall_stack.append({
            "name": name, "invocation_id": invocation_id,
            "emit_enabled": event is not None,
        })
        return event

    def syscall_exit(self, name, cpu, monotonic_ns, return_value,
                     event_id_ok=True, sequence_ok=True, reserve_ok=True):
        if self.task_exited:
            raise RuntimeError("task context already exited")
        if self.syscall_overflow_depth:
            self.syscall_overflow_depth -= 1
            self.active_syscall_invocations -= 1
            return None
        if not self.syscall_stack:
            self.counters["syscall_context_pop_mismatch"] += 1
            return None
        frame = self.syscall_stack.pop()
        self.active_syscall_invocations -= 1
        if frame["name"] != name:
            self.counters["syscall_context_pop_mismatch"] += 1
            return None
        if not frame["emit_enabled"]:
            return None
        return self._emit(
            "syscall", "syscall_exit", name, frame["invocation_id"], cpu,
            monotonic_ns, {"pid_tgid": 4294967297, "syscall": name,
                           "return_value": return_value},
            event_id_ok=event_id_ok, sequence_ok=sequence_ok,
            reserve_ok=reserve_ok,
        )

    def handler_enter(self, name, cpu, monotonic_ns, raw_modes,
                      raw_clockid=None, event_id_ok=True, sequence_ok=True,
                      reserve_ok=True, invocation_id_ok=True):
        if self.task_exited:
            raise RuntimeError("task context already exited")
        self.active_handler_invocations += 1
        if self.handler_overflow_depth or len(self.handler_stack) >= self.max_handler_depth:
            self.handler_overflow_depth += 1
            self.counters["handler_context_stack_overflow"] += 1
            return None
        invocation_id = self._allocate_invocation("handler", invocation_id_ok)
        if invocation_id is None:
            self.handler_overflow_depth += 1
            return None
        if self.syscall_overflow_depth:
            parent_status, parent_id = "stack_overflow", None
        elif self.syscall_stack:
            parent_status = "matched"
            parent_id = self.syscall_stack[-1]["invocation_id"]
        else:
            parent_status, parent_id = "no_active_context", None
        if parent_status != "matched":
            self.counters["handler_parent_context_unavailable"] += 1
        payload = {"pid_tgid": 4294967297, "handler": name, "raw_modes": raw_modes}
        if name == "do_clock_adjtime":
            payload["raw_clockid"] = raw_clockid
        event = self._emit(
            "kernel_handler", "kernel_handler_enter", name, invocation_id,
            cpu, monotonic_ns, payload, parent_id, parent_status,
            event_id_ok, sequence_ok, reserve_ok,
        )
        self.handler_stack.append({
            "name": name, "invocation_id": invocation_id,
            "parent_id": parent_id, "parent_status": parent_status,
            "emit_enabled": event is not None,
        })
        return event

    def handler_exit(self, name, cpu, monotonic_ns, return_value,
                     event_id_ok=True, sequence_ok=True, reserve_ok=True):
        if self.task_exited:
            raise RuntimeError("task context already exited")
        if self.handler_overflow_depth:
            self.handler_overflow_depth -= 1
            self.active_handler_invocations -= 1
            return None
        if not self.handler_stack:
            self.counters["handler_context_pop_mismatch"] += 1
            return None
        frame = self.handler_stack.pop()
        self.active_handler_invocations -= 1
        if frame["name"] != name:
            self.counters["handler_context_pop_mismatch"] += 1
            return None
        if not frame["emit_enabled"]:
            return None
        return self._emit(
            "kernel_handler", "kernel_handler_exit", name,
            frame["invocation_id"], cpu, monotonic_ns,
            {"pid_tgid": 4294967297, "handler": name,
             "return_value": return_value},
            frame["parent_id"], frame["parent_status"],
            event_id_ok, sequence_ok, reserve_ok,
        )

    def task_exit(self):
        if self.task_exited:
            raise RuntimeError("task exit observed twice")
        pending_syscalls = len(self.syscall_stack) + self.syscall_overflow_depth
        pending_handlers = len(self.handler_stack) + self.handler_overflow_depth
        self.counters["task_exit_observed"] += 1
        if pending_syscalls:
            self.counters["task_exit_with_pending_syscalls"] += 1
        if pending_handlers:
            self.counters["task_exit_with_pending_handlers"] += 1
        self.active_syscall_invocations -= pending_syscalls
        self.active_handler_invocations -= pending_handlers
        self.syscall_stack.clear()
        self.handler_stack.clear()
        self.syscall_overflow_depth = 0
        self.handler_overflow_depth = 0
        self.task_context_reservations -= 1
        self.task_exited = True
        return {"pending_syscalls": pending_syscalls,
                "pending_handlers": pending_handlers}

    def detach(self):
        if self.active_syscall_invocations or self.active_handler_invocations:
            raise RuntimeError("cannot detach with active invocations")
        if self.inflight_bpf_callbacks:
            raise RuntimeError("cannot detach with in-flight callbacks")
        self.producer_detached = True

    def global_snapshot(self):
        return {
            "capture_session_id": self.capture_session_id,
            "boot_id": self.boot_id,
            "snapshot_after_producer_detach": self.producer_detached,
            "counters": dict(self.counters),
            "gauges": {
                "active_syscall_invocations": self.active_syscall_invocations,
                "active_handler_invocations": self.active_handler_invocations,
                "inflight_bpf_callbacks": self.inflight_bpf_callbacks,
                "task_context_reservations": self.task_context_reservations,
            },
        }

    def per_cpu_snapshot(self):
        return {"per_cpu": [dict(value) for _, value in sorted(self.per_cpu.items())]}


class ClockAdjustmentProducerSessionAllocator:
    """Session-wide reference allocator shared by multiple task contexts."""

    def __init__(self, max_tracked_tasks=128, max_cas_retries=8,
                 start_event_id=1, start_task_instance_seq=1):
        if max_tracked_tasks < 1 or max_cas_retries < 1:
            raise ValueError("capacity and CAS retry bound must be positive")
        self.max_tracked_tasks = max_tracked_tasks
        self.max_cas_retries = max_cas_retries
        self.next_event_id = start_event_id
        self.next_task_instance_seq = start_task_instance_seq
        self.next_invocation_seq = {}
        self.active_tasks = set()
        self.task_context_reservations = 0
        self.active_syscall_invocations = 0
        self.active_handler_invocations = 0
        self.inflight_bpf_callbacks = 0
        self.producer_detached = False
        self.counters = ClockAdjustmentProducerState._zero_counters()

    def allocate_event_id(self, cas_attempts=1):
        self.counters["event_id_allocation_attempts"] += 1
        if (cas_attempts > self.max_cas_retries
                or self.next_event_id <= 0
                or self.next_event_id >= (1 << 64) - 1):
            self.counters["event_id_allocation_failures"] += 1
            return None
        value = self.next_event_id
        self.next_event_id += 1
        self.counters["event_id_allocation_successes"] += 1
        return f"{value:016x}"

    def create_task_context(self, create_ok=True, task_id_ok=True,
                            task_id_cas_attempts=1):
        if self.producer_detached:
            raise RuntimeError("producer session is detached")
        self.counters["task_context_admission_attempts"] += 1
        if len(self.active_tasks) >= self.max_tracked_tasks:
            self.counters["task_context_capacity_rejected"] += 1
            return None
        self.counters["task_context_admitted"] += 1
        self.task_context_reservations += 1
        self.counters["task_context_create_attempts"] += 1
        if not create_ok:
            self.counters["task_context_create_failures"] += 1
            self.task_context_reservations -= 1
            return None
        self.counters["task_context_create_successes"] += 1
        self.counters["task_context_init_attempts"] += 1
        self.counters["task_instance_id_allocation_attempts"] += 1
        if (not task_id_ok or task_id_cas_attempts > self.max_cas_retries
                or self.next_task_instance_seq <= 0
                or self.next_task_instance_seq >= (1 << 64) - 1):
            self.counters["task_instance_id_allocation_failures"] += 1
            self.counters["task_context_init_failures"] += 1
            self.task_context_reservations -= 1
            return None
        value = self.next_task_instance_seq
        self.next_task_instance_seq += 1
        task_id = f"{value:016x}"
        self.counters["task_instance_id_allocation_successes"] += 1
        self.counters["task_context_init_successes"] += 1
        self.active_tasks.add(task_id)
        self.next_invocation_seq[task_id] = 1
        return task_id

    def allocate_invocation_id(self, task_instance_id, success=True):
        if task_instance_id not in self.active_tasks:
            raise KeyError("unknown or exited task context")
        self.counters["syscall_invocation_allocation_attempts"] += 1
        if not success or self.next_invocation_seq[task_instance_id] >= (1 << 64) - 1:
            self.counters["syscall_invocation_allocation_failures"] += 1
            return None
        value = self.next_invocation_seq[task_instance_id]
        self.next_invocation_seq[task_instance_id] += 1
        self.counters["syscall_invocation_allocation_successes"] += 1
        return f"{value:016x}"

    def begin_invocation(self, kind):
        if kind == "syscall":
            self.active_syscall_invocations += 1
        elif kind == "handler":
            self.active_handler_invocations += 1
        else:
            raise ValueError("kind must be syscall or handler")

    def end_invocation(self, kind):
        if kind == "syscall":
            if self.active_syscall_invocations <= 0:
                raise RuntimeError("syscall invocation gauge underflow")
            self.active_syscall_invocations -= 1
        elif kind == "handler":
            if self.active_handler_invocations <= 0:
                raise RuntimeError("handler invocation gauge underflow")
            self.active_handler_invocations -= 1
        else:
            raise ValueError("kind must be syscall or handler")

    def release_task_context(self, task_instance_id,
                             pending_syscalls=0, pending_handlers=0):
        if task_instance_id not in self.active_tasks:
            raise KeyError("task context already released or unknown")
        if (not isinstance(pending_syscalls, int) or isinstance(pending_syscalls, bool)
                or pending_syscalls < 0
                or not isinstance(pending_handlers, int) or isinstance(pending_handlers, bool)
                or pending_handlers < 0):
            raise ValueError("pending invocation counts must be nonnegative integers")
        if pending_syscalls > self.active_syscall_invocations:
            raise RuntimeError("pending syscall count exceeds active gauge")
        if pending_handlers > self.active_handler_invocations:
            raise RuntimeError("pending handler count exceeds active gauge")
        self.counters["task_exit_observed"] += 1
        if pending_syscalls:
            self.counters["task_exit_with_pending_syscalls"] += 1
            self.active_syscall_invocations -= pending_syscalls
        if pending_handlers:
            self.counters["task_exit_with_pending_handlers"] += 1
            self.active_handler_invocations -= pending_handlers
        self.active_tasks.remove(task_instance_id)
        self.next_invocation_seq.pop(task_instance_id, None)
        self.task_context_reservations -= 1

    def detach(self):
        if self.active_syscall_invocations or self.active_handler_invocations:
            raise RuntimeError("cannot detach with active invocations")
        if self.inflight_bpf_callbacks:
            raise RuntimeError("cannot detach with in-flight callbacks")
        self.producer_detached = True

    def global_snapshot(self, capture_session_id="capture-01", boot_id="boot-01"):
        return {
            "capture_session_id": capture_session_id,
            "boot_id": boot_id,
            "snapshot_after_producer_detach": self.producer_detached,
            "counters": dict(self.counters),
            "gauges": {
                "active_syscall_invocations": self.active_syscall_invocations,
                "active_handler_invocations": self.active_handler_invocations,
                "inflight_bpf_callbacks": self.inflight_bpf_callbacks,
                "task_context_reservations": self.task_context_reservations,
            },
        }




__all__ = [
    "ClockAdjustmentProducerState",
    "ClockAdjustmentProducerSessionAllocator",
]
