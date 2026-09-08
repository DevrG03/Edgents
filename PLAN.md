# Edgents v2: Complete Research & Development Plan

**Scope:** LangGraph-based multi-agent UAV swarm with dynamic role election and federated hallucination calibration
**Duration:** 8 weeks | **Stack:** Ubuntu 24. LTS (see §0.1 caveat), ROS 2, Python, LangGraph, Ollama (Qwen 2B), PX4 SITL + Gazebo, Flower
**Hardware:** 2× Dell Precision 3650 (i5, 16 GB RAM, AMD Radeon Pro W5500), WiFi LAN, `ROS_DOMAIN_ID=42`

---

- 0. Pre-Work: Environment & Risk De-Risking (Before Week 1)

## 0. Version Pinning Decision (Day 0)

**Do NOT start on Ubuntu 26.04 LTS / Python 3.14.** Verify, then pin. Execute this checklist:

```bash
# On BOTH PCs — run and record results in a file called ENV_AUDIT.md
lsb_release -a                          # record OS version
python3 --version                       # record Python version
ros2 doctor                             # full pass required
gz sim --version                        # record Gazebo version
px4 -v                                  # or check px4_autopilot git tag
ollama --version
pip show langgraph langchain-core rclpy ultralytics flower fl
```

**Decision rules:**
 If PX4 SITL + Gazebo + Micro XRCE-DDS has no verified build for your OS → **stay on Ubuntu 24.04** (your paper-1 stack). Report the version limitation in the paper. This is a tooling detail, not a contribution.
- If `ultralytics` orlanggraph` wheels fail on Python3.14 → pin Python 3.12 via:

```bash
# Using uv (recommended)
curl -LsSf https://astral.sh/uv/install.sh | sh
uv venv --python 3.12 --system-site-packages ~/.venvs/edgents
source ~/.venvs/edgents/bin/activate
# Run after sourcing the ROS 2 and workspace environments:
source /opt/ros/<ros-distro>/setup.bash
source install/setup.bash
python3 -c "import rclpy; assert 'dist-packages' in rclpy.__file__ or 'ros' in rclpy.__file__"
```

- Create `ENV_AUDIT.md` with exact versions of every package. Every dependency gets pinned in a `requirements.lock`:

```bash
uv pip compile requirements.in -o requirements.lock
```

**Exit criterion:** `ros2 doctor` clean, PX4 SITL spawns one drone, Ollama servesqwen` 2B with a successful inference, LangGraph "hello world" graph runs, and the in-venv `rclpy` path assertion passes after sourcing `install/setup.bash`. The audit passes only in the agent launcher's environment. Do not proceed until all five pass.

## 02 Repository Restructure (Day 0–1)

```
edgents-v2/
├── README.md
├── ENV_AUDIT.md
├── requirements.in / requirements.lock
├── configs/
│   ├── base.yaml              # shared: model name, context limits, loop rates
│   ├── lead.yaml              # role overrides
│   ├── wingman.yaml
│   ├── election.yaml          # scoring weights, tie-break rule, timeouts
│   ├── safety.yaml            # thresholds — read by safety layer ONLY
│   └── federation.yaml        # configuration for federated learning
├── edgents_core/              # shared library (imported by both agents)
│ ├── graph/
│   │   ├── state.py           # AgentState TypedDict
│   │   ├── nodes/
│   │   │   ├── sense.py
│   │   │   ├── build_context.py
│   │   │   ├── reason.py
│   │   │   ├── validate_tool_call.py
│   │   │   ├── repair.py
│   │   │   ├── execute.py
│   │   │   └── skip_cycle.py
│   │   ├── edges.py           # conditional edge routing functions
│   │   └── builder.py         # build_graph(role: str) -> CompiledGraph
│   ├── safety/                # ⚠️ NEVER inside the graph
│   │   └── monitor_node.py
│   ├── tools/
│   │   ├── registry.py        # BaseToolRegistry
│   │   ├── flight_tools.py
│   │   ├── sensing_tools.py
│   │   ├── memory_tools.py
│   │   ├── comm_tools.py
│   │   └── role_scopes.py     # per-role tool allow-lists
│   ├── memory/
│   │   └── agent_memory.py    # SQLite — kept as-is from paper 1
│   ├── comms/
│   │   ├── envelope_io.py     # ROS 2 pub/sub <-> graph state bridge
│   │   └── envelope_schema.py # pydantic models for envelope JSON
│   ├── election/
│   │   ├── feature_vector.py
│   │   ├── scoring.py
│   │   └── election_fsm.py
│   ├── calibration/
│   │   ├── features.py        per-cycle feature extraction
│   │   ├── classifier.py
│   │   └── federated/
│   │       ├── client.py
│   │       └── server.py
│   └── logging/
│      └── cycle_logger.py    # JSONL per-cycle telemetry
├── agents/
│   ├── lead_agent_node.py     # ROS 2 wrapper: spin + graph invocation
│   └── wingman_agent_node.py
├── tests/
│   ├── unit/
│   ├── integration/
│   └── regression/            # Phase 2 parity scripts live here
├── sim/
│   ├── launch/two_drones.launch.py
│   └── worlds/baylands_presets/
├── experiments/
│   ├── scenarios/             # scripted scenario definitions
│   ├── analysis/              # notebooks/scripts for metrics
│   └── results/               # immutable run outputs, one dir per run
└── docs/
    ├── SAFETY_INVARIANTS.md   # §see below
    └── ABLATION_PLAN.md
```

**Dependency rule:** `requirements.in` must never contain `rclpy`, `px4_msgs`, or `ros_gz`; these are supplied by the ROS 2 workspace and must be resolved in the agent launcher's environment.

## 0.3 Write `docs/FETY_INVARIANTS.md` First (Day 1)

This document is your contract. Every test in the project traces back to it. Content:

```
INVARIANT S1: The safety monitor runs OUTSIDE any LangGraph graph.
INVARIANT S: Safety RTL commands are published directly via
              px4_msgs/VehicleCommand; no graph can intercept,
              delay, or override them.
INVARIANT S3: Battery ≤15% OR GPS fix <3 ⇒ RTL issued 3× within 1s.
INVARIANT S4: During any election or handoff, safety monitoring is
              UNINTERRUPTED (monitor is role-agnostic).
INVARIANT S5: Exactly one agent holds ask_human/notify_human at any
              instant. This is enforced by role_scopes.py, testable.
INVARIANT S6: A graph crash/hang must NOT disable the safety monitor
              (separate process, separate thread pool).
```

Each invariant a named test (§ defined per phase below). **A phase is not "done" until its invariant tests pass.**

## 0.4 Metrics Baseline Freezing (Day 1)

Copy from paper 1 into `experiments/results/baseline_v1.json`:

```json
{
  "msr": 0.75, "missions": 8,
  "mission_duration_s": 356.86,
  "envelope_messages": 9, "bytes_total": 1106,
  "format_hallucination_rate": 0.12,
  "physical_hallucination_rate": 0.08,
  "coordination_failure_rate": 0.05,
  "inference_latency_s": [3, 8],
  "safety_ticks": 73
}
```

All comparisons in Week 7–8 cite this file. **Never edit it after Day 1.**

## 0.5 Structured Logging Infrastructure (Day 1–2)

Build `cycle_logger.py` **before** any graph work — you cannot add instrumentation retroactively to runs you've already done. Every inference cycle emits one JSONL line:

```json
{
  "ts": "2025-01-15T10:32:01.442Z",
  "agent_id": "drone_0",
  "role": "lead",
  "mission_id": "m_0001",
  "cycle": 47,
  "graph_node": "reason",
  "prompt_tokens": 953,
  "completion_tokens": 42,
  "latency_s": 4.81,
  "logprobs_top1": -0.31,
  "thought_field_length_chars": 84,
  "retry_count": 0,
  "parse_ok": true,
  "tool": "move",
  "tool_params_valid": true,
  "hallucination_class": null,
  "safety_events_active": false
}
```

**Ollama logprob note (verify in Week 1):** Ollama's `/api/generate` does not natively expose per-token logprobs in all versions. Check `ollama show --modelfile` and API docs. If unavailable, fallback features: completion length, JSON parse margin (distance from string boundaries: `raw.find('{')` position), thought-field length, retry count, and token-level entropy via a local `logits` request if supported. **If logprobs are truly unavailable, document this at Day 2 — it changes calibration feature set (§4.1), and you must know that in Week 1, not Week 5.**

**Acceptance test L1:** Run 10 inference cycles; verify JSONL lines parse, all required fields present, latency field matches wall-clock within ±100 ms.

---

# PHASE 1: Single-Drone LangGraph Agent (Prerequisite assumed stable)

If Phase 1 was not completed before this plan starts, it occupies Weeks 0–1 and everything shifts. Assumed complete when:

- [ ] **T1.1:** Graph `sense → build_context → reason → validate → execute → (→ sense)` runs 50 consecutive cycles without unhandled exception.
- [ ] **T1.2:** Retry counter in state: `validate` fails → `repair` → `reason`, max 3, then `skip_cycle`. Verified with a mock-LLM that returns invalid JSON twice then valid JSON.
 [ ] **T1.3:** Checkpointer (SQLite backend) enabled; kill the process mid-mission; restart; graph resumes from last super-step. Verify mission state (phase, altitude target) is restored. **This is a new capability vs. paper 1 — record a demo video/screenshot for the paper.**
- [ ] **T1.4:** Checkpoint pruning: message-history channel capped (last 8 tool results, matching paper 1's sliding window) so state size stays bounded over a 6-minute mission. Measure state size per cycle; assert < 500 KB.
- [ ] **1.5:** Safety monitor is a separate ROS 2 node (separate process), can cancel graph execution and force `rtl`. **Test S1/S2:** mock-LLM emits `move` with battery injected at 14% → assert RTL VehicleCommand published, graph terminal state entered, no `move` dispatched to PX4.
- [ ] **T1.6:** SQLite `AgentMemory` still works via `remember`/`recall` tools; facts survive process restart (checkpointer handles state, memory handles facts — verify both, they're different).

---

# PHASE 2: Second Drone — Static Roles (Week 1–2)

## Week 1, Day 1–2: Communication Layer

### Step 2.1 — Envelope schema (pydantic, `envelope_schema.py`)

```python
from pydantic import BaseModel, Field
from typing import Literal, Optional
from datetime import datetime, timezone

class Envelope(BaseModel):
    msg_id: str                    # uuid4 hex — dedup + ordering
    ts: str                        # ISO 8601 UTC
    sender: Literal["lead", "wingman    recipient: Literal["lead", "wingman"]
    type: Literal[
        "instruction",      # Lead → Wingman task directive
        "status",           # Wingman → Lead progress report
        "question",         # Wingman → Lead clarification
        "answer",           # Lead → Wingman reply to question
        "election",         # Phase : role election (added later)
        "handoff",          # Phase 3 (stretch)
        "ack"               # generic acknowledgement
    ]
    content: str                   # natural language OR embedded JSON string
    reply_to: Optional[str] = None # msg_id of message being answered
    seq: int = Field(..., ge=0)    # per-sender monotonic sequence

    class Config:
        extra = "forbid"           # strict — unknown fields are errors
```

**Unit test E1:** Serialize/deserialize round-trip; reject unknown `type`; reject `sender == recipient`.

### Step 2.2 — Envelope I/O bridge (`envelope_io.py`)

This is the boundary between ROS 2 transport and graph state. Design rule: **DDS is the only; graphs never talk to each other directly.**

```
┌────────────── PC-1 ──────────────┐      ┌────────────── PC-2 ──────────────┐
│  Lead graph ⇄ envelope_io_node   │      │  Wingman graph ⇄ envelope_io_node│
│                  │               │      │                  │               │
│        /agent/lead_to_wingman    │◄─WiFi─►        /agent/wingman_to_lead │
└──────────────────────────────────┘      └──────────────────────────────────┘
```

Implementation:

```python
# envelope_io.py — runs inside the agent's ROS 2 node process
class EnvelopeIO:
    """Subscribes topic → pushes into graph state via a
    threading.Event + queue. Publishes outbound from a queue drained
    by the graph's 'comm' side-effect after each super-step."""

    def __init__(self, role: str):
        self.inbound: queue.Queue[Envelope] = queue.Queue(maxsize=32)
        self.outbound: queue.Queue[Envelope] = queue.Queue(maxsize=32)
        # rclpy sub on /agent/{peer}_to_{me}, pub on /agent/{me}_to_{peer}

    def poll_inbound(self, max_msgs: int = 5) -> list[Envelope]:
        """Called from graph's build_context node — drains into state."""

    def publish(self, env: Envelope) -> None:
        """Called from execute node when tool is message_*/ack."""
```

**Why this and NOT a supervisor graph or shared checkpointer (decide now, don't reopen):**
1. Supervisor graph = centralization → contradicts the paper's decentralised claim.
2. Shared checkpointer store = impossible across two physical PCs; breaks topology.
3. Topic transport unchanged from paper 1 → your 9-message / 1.08 KB metric stays directly comparable. This is a **regression-measurement requirement**, not a preference.

**Integration test E2:** Lead graph executes `message_wingman("go east 20m", type=instruction)` → Wingman's `build_context` sees it in state within one cycle (≤10 s). Log msg_id on both ends.

**Test — comms metric instrumentation:** envelope_io logs every send/receive to JSONL. End-of-mission script counts unique `msg_id`s. This replaces manual counting from paper 1 (source of the "9 messages" figure — make it automatic this time).

## Week 1, Day 3–4: Role-Scoped Graph Compilation

### Step 2.3 — Tool scopes (`role_scopes.py`)

```python
ROLE_TOOLS: dict[str, frozenset[str]] = {
    "lead": frozenset({
        "takeoff", "move", "hover", "search", "land", "rtl        "get_situation", "scan_camera", "get_battery",
        "remember", "recall",
        "wait", "mission_complete",
        "message_wingman", "ask_human", "notify_human",
    }),
    "wingman": frozenset({
        "takeoff", "move", "hover", "search", "land", "rtl",
        "get_situation", "scan_camera", "get_battery",
        "remember", "recall",
        "wait", "mission_complete",
        "message_lead", "ask_lead", "notify_lead",
    }),
}

def assert_scope(role: str, tools: set[str]) -> None:
    illegal = tools - ROLE_TOOLS[role]
    if illegal:
        raise ValueError(f"{role} graph illegally registered: {illegal}")
```

**Invariant S5 test:** `builder.py` calls `assert_scope` at compile time. Unit test attempts to build a "wingman" graph containing `ask_human` → must raise. This makes the human-channel invariant *structurally enforced*, not prompt-enforced — a citable improvement over paper 1.

### Step 2.4 — Per-role graph builder (`builder.py`)

```python
def build_graph(role: str, io: EnvelopeIO, memory: AgentMemory,
                checkpointer) -> CompiledStateGraph:
    assert_scope, ALL_TOOLS)
    g = StateGraph(AgentState)
    g.add_node("sense", make_sense_node(io))          # also drains envelopes
    g.add_node("build_context", make_context_node(role, memory))
    g.add_node("reason", make_reason_node(role))       # role-specific prompt
    g.add_node("validate_tool_call", validate_node(role))
    g.add_node("repair", repair_node)
    g.add_node("execute", make_execute_node(role, io))
    g.add_node("skip_cycle", skip_node)
    g.set_entry_point("sense")
    g.add_edge("sense", "build_context")
    g.add_edge("build_context", "reason")
    g.add_conditional_edges("validate_tool_call", route_validation,
        {"execute": "execute", "repair": "repair", "skip": "skip_cycle"})
    g.add_edge("repair", "reason")
    g.add_edge("execute", END)
    g.add_edge("skip_cycle", END)
    return g.compile(checkpointer=checkpointer)
```

`route_validation` reads `state["attempts"]` and `state["parse_ok"]` — the counter lives in state, not in a loop variable (T1.2 carries over).

**System prompt per role:** port paper-1 prompts verbatim into `configs/lead.yaml` / `wingman.yaml` (prompt text in a `system_prompt:` key). **Do not rewrite prompts during migration** — that confounds the regression comparison.

## Week 1, Day 5: ROS 2 Node Wrappers

`lead_agent_node.py` structure:

```python
class LeadAgentNode(Node):
    def __init__(self):
        super().__init__("lead_agent")
        cfg = load_config("lead")
        self.io = EnvelopeIO("lead")
        self.memory = AgentMemory("~/.ros/lead_agent_memory_v2.db")
        self.graph = build_graph("lead", self.io, self.memory,
                                 SqliteSaver.from_conn_string(
                                     "~/.ros/lead_checkpoint.db"))
        self.executor_thread = threading.Thread(
            target=self._graph_loop, daemon=True)
        # rclpy spin stays on main thread — T1 pattern from paper 1

    def _graph_loop(self):
        while rclpy.ok() and not self.mission_done:
            self.invoke({}, config={"configurable":
                              {"thread_id": self.mission_id}})
            time.sleep(self.cfg["loop_pause_sec"])
```

**Design decision to document:** each `graph.invoke` = one sense→…→execute cycle; the outer loop replaces paper 1's `while` loop. Mission-complete is a state flag the loop checks.

## Week 2, Day 1–2: Simulation Bring-Up (2 drones)

Reuse paper 1's sim exactly; only agent internals changed.

```bash
# launch/two_drones.launch.py — verify checklist:
# [ ] PX4 SITL instance 0 (drone_0) with baylands world
# [ ] PX4 SITL instance 1 (drone_1)
# [ ] 2× MicroXRCEAgent (ports/keys as paper1)
# [ ] ros_gz_bridge for both camera streams
# [ ] ROS_DOMAIN_ID=42, CycloneDDS 0.10
```

**Smoke test SM2:** `ros2 topic list` shows both drones' telemetry; `ros2 topic hz` on both image topics ≥ 2 Hz; `qos` compatible on envelope topics (reliable, keep-last-10).

## Week 2, Day 3–5: Regression Campaign

### Scenario scripts (`experiments/scenarios/`)

Port paper 1's three scenarios as machine-readable definitions:

```yaml
# scenarios/s1_nominal.yaml
id: s1
name: nominal_coordination
voice_command: "Fly 10 metres north have the wingman hold position."
success_criteria:
  - drone_0 reaches within 2m of (10, 0) at 10m alt
  - drone_1 holds position ±1m for ≥30s
  - no safety overrides
  - mission completes without manual intervention
timeout_s: 480
```

```yaml
# scenarios/s2_vision.yaml
id: s2
name: vision_triggered_alteration
setup: spawn person model in camera FOV at t+60s
success_criteria:
  - YOLO detection appears in situation string within 5s
  - SLM log contains detection-aware reasoning in thought field
  - flight path altered (position delta > 5m from nominal)
```

```yaml
# scenarios/s3_safety.yaml
id: s3
name: safety_fallback
voice_command: "Fly to altitude -500 metres"
success_criteria:
  - RTL issued by safety layer or commander rejects intent
  - SLM output never reaches PX4 as setpoint
  - drone lands safely / stays grounded
```

Run **8 missions of S1, 4 of S2, 4 of S** (16 total, matching paper 1's n=8 at minimum for S1).

### Regression gate (fill `experiments/results/baseline_v2_parity.json`)

| Metric | Paper 1 | Pass threshold v2 |
|---|---|---| MSR (S1) | 75% | ≥ 70% (no statistical regression; exact CI overlap) |
| Envelope msgs | 9 ≤ 12 (small tolerance for acks) |
| Mission duration | 356.86 s | within ±20% |
| Format hallucination rate | 12% | ≤ 12% (expected ↓ from structured validation) |
| Safety invariant tests (S1–S6) | — | 100% pass |
| Collisions | 0 | 0 |

**If any gate:** stop, bisect (graph structure vs. prompt vs. comms — run S1 with the graph reduced to a pass-through of paper-1 logic if needed). Do not start Phase 3 with a regression.

**Week 2 exit:** parity JSON committed; invariant tests S1–S6 green in CI script (`tests/run_invariants.sh`).

---

# PHASE 3: Dynamic Role Assignment (Week 3–4)

## Week 3, Day 1–2: Election Protocol

### Step 3.1 — Feature vector (`feature_vector.py`)

```python
@dataclass(frozen=True)
class CapabilityVector:
    battery_pct: float          # 0–100
    gps_fix_type: int           # PX4 fix type, ≥3 =3D
    gps_sats: int
    dist_home_m: float
    link_rssi_dbm: float        # from DDS/WiFi introspection; fallback: -50    drone_id: str               # tie-breaker, e.g. "drone_0"
```

### Step3.2 — Deterministic scoring (`scoring.py`)

```python
# election.yaml
weights: {battery: 0.4, gps: 0.25, dist_home: 0.2, link: 0.15}
gps_floor_fix: 3            # hard gate: fix < 3 → score = -inf
tie_break: lower_drone_id_wins
election_timeout_s: 10
```

```python
def score(c: CapabilityVector, w: dict) -> float:
    if c.gps_fix_type < 3:
        return float("-inf")
    terms = {
        "battery": c.battery_pct / 100.0,
        "gps": min(c.gps_sats / 20.0, 1.0),
        "dist_home": max(0.0, 1.0 - c.dist_home_m / 100.0),
        "link": min(max((c.link_rssi_dbm + 90.0) / 60.0, 0.0), 1.0),
    }
    if __debug__:
        assert all(0.0 <= term <= 1.0001 for term in terms.values())
    return sum(w[name] * terms[name] for name in terms)
```

**Scoring is pure and deterministic** — same vectors, same winner, always. Unit test with 20 fuzzed vector pairs asserting determinism and that the `-inf` GPS gate always loses. In debug builds, assert every normalized term is in `[0.0, 1.0001]`.

### Step 3.3 — Election FSM (`election_fsm.py`)

Runs at mission start, **before any flight tool is enabled**. Both drones' tools are locked until the FSM reaches `ELECTED`.

```
STATE: IDLE → BROADCAST → COLLECT → DECIDE → ELECTED | RETRY(≤2) → FALLBACK

BROADCAST: each drone pub election envelope w/ own CapabilityVector
COLLECT:   wait for peer vector, timeout = election_s
DECIDE:    deterministic score(tie-break by drone_id)
ELECTED:   winner role=lead, loser role=wingman; both sides must
           compute the SAME winnerverify: compare role-assignment
           hashes in an ack exchange) → then unlock tool scopes
RETRY:     timeout on collect → re-broadcast (max 2)
FALLBACK:  2 failures → drone_0=lead, drone_1=wingman (hardcoded),
           log ELECTION_FALLBACK event (reportable metric)
```

**Envelop:** election costs 2 vectors + 2 acks = 4 messages. Budget this in the comms metric (report "9 + 4 election" or run election pre-baseline).

**Invariant S4 test:** inject a 500 ms periodic fake safety event during an election → assert monitor keeps firing throughout, RTL still executes while election in progress. **The election must never block safety.**

**Test EL1 (determinism):** same sim start state × 5 runs → identical role assignment all 5.
**Test EL2 (capability wins):** script drone_1 with 90% battery, drone_0 with 40% → drone_1 must be elected Lead.
**Test EL3 (GPS gate):** drone_1 GPS fix=2 → drone_0 wins regardless of.
**Test EL4 (fallback):** block election envelopes via DDS firewall rule → both drones converge to fallback assignment, mission still runs.
**Test EL5 (link monotonicity):** with identical vectors except RSSI, the best link (−30 dBm) must never lose across a −90→−35 dBm sweep; assert the full score satisfies `0 ≤ score ≤ Σweights`.

## Week 3, Day 3–5: Integration

1. Refactor `agents/*_agent_node.py` into one **parameterised** `agent_node.py` (role no longer hardcoded at startup; only `drone_id` is).
2. After `ELECTED`, compile the graph with the winning role's scope; log `ROLE_ASSIGNED {drone_id, role, scores, tiebreak_used}` to the cycle logger.
3. Re-run full regression (16 scenarios from Week 2) with election enabled.

**Regression gate 2:** MSR within parity thresholds; add election metrics: election success rate (target 100% across runs, fallback usage logged), election duration (target< 15 s), election envelope count (target 4).

## Week 4: Mid-Mission Handoff — **STRETCH, DECIDE ON DAY 22, NOT DAY 28**

**Decision rule written into ABLATION_PLAN.md today:** *"If Phase 3 regression took >5 working days, handoff is demoted to documented limitation immediately, and Week 4 becomes evaluation-campaign prep (§Phase 4 dry run)."*

If proceeding:

### Step 4.1 — Trigger (monitor-side, not SLM-side)
Handoff trigger lives in the safety/monitor layer (deterministic): `lead.battery ≤ 25%` `lead.link lost > 5 s`. **The SLM never decides handoff** — only executes the role change afterwards.

### Step 4.2 — Handoff FSM
```
LEAD_STABLE → HANDOVER_PENDING (trigger fires, send handoff envelope)
  → (ack from peer within 5s) → HANDOVER_COMMIT
      Lead:   graph recompiled with wingman scope (tool scopes
                  swapped atomically at commit, never mid-cycle)
      new Lead:   recompiled with lead scope; STT/TTS endpoint
                  rebinds HERE — only after commit
  → (no ack) → HANDOVER_ABORT (role reverts; re-attempt ≤2, then
      mission_complete with safety report)
```

Atomicity rule: the old Lead finishes its current graph super-step **before** scope swap. A cycle is never half-executed under two different role scopes. Test HO1: inject handoff mid-cycle; assert no tool from the wrong scope executes post-trigger (check JSONL: every `tool` field after handoff timestamp matches new role's allow-list).

**Test HO2 (channel):** during HANDOVER_PENDING, inject `ask_human` → must queue, not send; delivered only after COMMIT.
**Test HO3:** handoff triggered at battery 24% → measure trigger→ack time (report as handoff latency), then RTL at 15% executes under new authority chain.
**Test HO4 (checkpoint continuity across recompile):** snapshot state pre-handoff → recompile graph → assert mission phase is not reset, the sliding tool window retains pre-handoff entries, memory facts survive, and `cycle_index` does not restart. Store `(checkpointer, thread_id)` as node attributes set once at startup; recompilation must reuse both and never create a new `SqliteSaver` or regenerate `thread_id`.

**Single evaluation scenario (per the scoping decision):** 5 runs of "Lead battery drains at 0.1%/s scripted → handoff → mission completes." Report: handoff latency, packets lost (msg_id gap analysis), MSR of post-handoff phase.

**Week 4 exit:** election regression green; handoff either evaluated (5 runs) or written as limitation. No other Phase 4 work starts until this closes.

---

# PHASE 4: Federated Hallucination (Week 5–7)

## Week 5, Day 1–2: Feature Extraction & Labels

### Step 5.1 — Calibration dataset schema

One row per inference cycle, built from the Week 0 JSONL logs + post-hoc labelling:

| Feature | Source | Notes |
|---|---|---|
| `completion_tokens` | Ollama stats | |
| `thought_len_chars` | parsed JSON | 0 if parse failed |
| `retry_count` | validate node | |
| `parse_margin` | `raw.find('{')` position | distance of JSON from string start |
| `top1_logprob` (or proxy) | Ollama / fallback §0.5 | ⚠️ resolved in Week 0 |
| `context_utilisation` | prompt_tokens / 8192 | |
| `tool_category` | one-hot: flight/sensing/memory/com/completion | |
| `role` | election result | enables the role-robustness eval |
| `cycle_index` | position in mission | |
| **`label`** | post-hoc | see below |

### Step 5.2 — Label definition (this is the crux; define it explicitly)

```
label = HALLUCINATED if hallucination_class in {format, physical, coordination}
label = CORRECT      if tool executed AND achieved intended effect
label = UNCERTAIN    if tool executed but effect unverifiable
```

Effect verification is automated per tool: `move(d, dir)` → position delta within 20% of `d` the right heading (from telemetry); `takeoff(a)` → altitude within 1 m of `a`; `search(t)` → duration match. Anything else → UNCERTAIN, excluded from training, counted separately.

### Step 5.3 — Data collection campaign (Week5, Days 3–5)

Run **S1×8 + S2×4 + S3×4 per role-assignment configuration**, both drones logging. From 16 missions × ~40 cycles/mission × 2 drones ≈ **1,200–1,400 labelled cycles**. Expect class imbalance (~75/25) — record it; the classifier handles it with class weights.

**Include the Week 3/4 election + handoff runs** — those give you cycles where `role` changed mid-mission, which is your generalization test set.

### Step 5.4 — Classifier (`calibration/classifier.py`)

- Model: gradient-boosted trees (`xgboost` or `sklearn.HistGradientBoostingClassifier`) — small, fast, edge-friendly, explainable via feature importance (paper-friendly figure).
- Task: binary P(hallucination | features) per cycle.
 Metrics: **ECE (expected calibration error)**, Brier score, AUROC — and *reliability diagrams* (your headline figure).
- Baseline to beat: raw Ollama logprob (or proxy) used alone as confidence.

**Non-federated sanity check first (Day 5):** train centrally on pooled data; verify ECE over logprob baseline. If a centralised classifier can't beat logprob, **stop and revisit features before doing FL work.** This is your Week 5 go/no-go.

## Week 6, Days 1–3: Federation with Flower

### Step 6.1 — Why FL here, stated honestly for the paper
Each drone's hallucination distribution is non-IID (different roles, sensors, tool mixes) and raw inference traces may be operationally sensitive (mission behaviour profiles). FL lets both drones' calibrators improve without exchanging raw cycles.

### Step 6.2 — Setup

```python
# federated/client.py — sketch
class CalibrationClient(fl.client.NumPyClient):
    def __init__(self, X, y, client_id):
        self.model = HistGradientBoostingClassifier(...)  # or simple NN
    def get_parameters(self): return params_as_numpy
    def fit(self, parameters, config):
        self.model = set_params(self.model, parameters)
        self.model.fit(self.X, self.y, sample_weight=class_weights)
        return self.get_parameters(), len(self.y), {}
    def evaluate(self, parameters, X_val, y_val): ...

# federated/server.py
strategy = fl.server.strategy.FedAvg(
    fraction_fit=1.0, fraction_evaluate=1.0,
    min_fit_clients=2, min_evaluate_clients=2, min_available_clients=2)
```

- **Client count is configuration-driven:** read the client count and sharding configuration from `configs/federation.yaml`; construct `CalibrationClient(X, y, client_id)` for each client. The default is **2 clients** (drone_0, drone_1), each training on its own cycles only, while preserving the virtual-client variant for later experiments.
- Aggregation: FedAvg over model parameters. With GBTs, parameter averaging awkward → **use a small MLP (2 hidden layers, ~64 units) as the calibration classifier for the federated version**, keeping GBT as the centralized reference. Document this choice.
- Rounds: 10–20; report ECE vs. round curve.
- Split: 60/20/20 per drone (train/val/test), **test set drawn per-drone and never aggregated** — you evaluate whether the federated model calibrates *each* drone's held-out data better than that drone's local-only model### Step 6.3 — Experiments (Week 6, Days 4–7)

**E-FED-1 (headline):** federated calibration vs. local-only calibration vs. raw-logprob baseline, evaluated on per-drone held-out test sets. Report ΔECE, ΔBrier, ΔAUROC. *Expected finding to test: federation helps most on the drone whose role distribution is underrepresented (fewer cycles).*

**E-FED-2 (non-IID severity):** artificially skew one drone's training data (subsample its hallucinated cycles by 50%, then 80%) → measure how much federation recovers vs. local training. This gives you the "non-IID severity" ablation from the plan without new sim work.

**E-FED-2 virtual-client variant:** shard each drone's cycles into 3 by mission-time (early/mid/late) → **N=6 clients**, run offline only inside the E-FED-2 slot. Report **N=2 real clients** in the headline table and **N=6 virtual clients** in a separate robustness subsection; never mix them in one column.

**E-FED-3 (role robustness — the Phase 3 callback, scoped small):** split test cycles into "matches agent's final role" vs. "cycles from before a role change" (handoff/election runs only). Compare ECE on each. If handoff was cut and there are no role-change cycles, substitute: train on drone_0's data, test on drone_1's (cross-role transfer). Either version answers " the calibrator survive role/agent distribution shift?" — one experiment, half a day.

## Week 6 parallel track, Days 4–7: Federated YOLO (scoped-down capability demo)

- **Scope:**2 clients, non-IID split (drone_0 sees person+car only; drone_1 sees bicycle+truck only, in a Gazebo world with all four object types), YOLOv8-nano, 5–10 FL rounds via Flower, simple FedAvg of weights (or FedProx if divergence).
- **Metric:** per-class mAP@0.5, federated vs. each drone's local-only model on a shared balanced val set. Report the delta table. Expected: federated recovers classes

> the drone it never saw locally (e.g., drone_0's federated model detects bicycles where its local model cannot), quantifying the privacy–capability trade: exchange weights instead of images.

- **Infrastructure note:** YOLO training competes with Ollama inference for the W5500. Run FL-YOLO rounds **offline** (no missions active) — schedule it after the Week 6 Day 1–3 calibration federation runs, never concurrently with a mission campaign.
- **Explicitly cut:** privacy attack evaluation (membership inference / gradient leakage). Write one paragraph in the paper motivating it via the FL-LLM references (OpenFedLLM, FedLLM-Bench, Bai et al.), list as future work. **Do not start it** — this is the single biggest scope-discipline risk of Phase 4.

## Week 7: Full Ablation & Evaluation Campaign

### Step 7.1 — Ablation matrix (`docs/ABLATION_PLAN.md` — write this file now if not done in Week 0)

| ID | Condition | Runs | Compares against |
|---|---|---|---|
| A1 | ECoT ON (default) | 8× S1 | E-CoT OFF (prompt edit: drop `thought` requirement) |
| A2 | Retry limit 3 (default) | 8× S1 | Retry limit 1 |
| A3 | Election ON (default) | 8× S1 | Static roles (Week 2 build) |
| A4 | Federated calibrator ON | 8× S1 | Local-only calibrator, no calibrator |
| A5 | Non-IID severity: 0/50/80% skew | 3× per level | (E-FED-2, if not finished Wk 6) |

**ECoT-OFF prompt change (A1):** modify only the system prompt in `configs/*.yaml` — remove the `thought` field requirement and schema. Log the same JSONL fields; the `thought_len_chars` feature will be 0, which is itself a calibration feature signal. Hypothesis (reproducing paper 1's claim rigorously this time): format hallucination rate rises from ~5% to >20%. If the LangGraph validate/repair loop absorbs the difference, **that's a finding, not a failure** — "structured validation compensates for prompt-level guardrails" is a reportable result. State the hypothesis in the plan before running.

**Calibrator-in-the-loop (A4):** decide *now* how the calibrator is used at runtime — two options:
- **Passive (default):** calibrator logs P(hallucination) per cycle; used only for evaluation. Safe, no behavioural risk.
- **Active:** P(hallucination) > 0.7 → cycle is skipped and the peer is notified (abstention-triggered coordination — your I-CALM-gap claim). Only attempt active mode if the passive results are strong by Day 2 of Week 7; otherwise passive-only and abstention-as-coordination goes to future work. **Do not attempt active mode for the first time during the final campaign.**

### Step 7.2 — Master campaign schedule (Week 7, Days 1–5)

```
Day 1: A1 (ECoT off) — 8× S1 + 2× S3   [highest risk of weird behaviour, run first]
Day 2: A2 (retry 1) — 8× S1; A4 passive — 8× S1 (if not already collected)
Day 3: A3 static-roles re-run — 8× S1; E-FED-3 cross-role eval (offline, no sim)
Day 4: Final default-config campaign: 12× S1, 4× S2, 4× S3 (n=20 headline numbers)
Day 5: Buffer / re-runs of any corrupted run
```

Rules for the campaign:
- **One run = one directory** in `experiments/results/run_<id>/` containing: all JSONL logs, envelope count, mission outcome, config snapshot (hash of `configs/`), git commit hash. Immutable once written.
- **No code changes during the campaign.** Any change invalidates prior runs — if forced, restart the affected ablation from scratch.
- Nightly: `python -m experiments.analysis.daily_summary` produces a parity table vs. `baseline_v1.json` and `baseline_v2_parity.json`. Read it every morning; catch drift early.

### Step 7.3 — Statistical treatment (do this before writing)

- MSR: report Wilson 95% CIs (n=20 gives CI ≈ ±20 pts — acknowledge this; the parity framing "no regression, directionally improved" is honest at this n; do not claim significance you don't have).
- Cycle-level rates (hallucination, latency): n in the thousands → report means with bootstrap CIs, plus Mann–Whitney U for latency comparisons across ablations.
- Calibration: ECE with 10-bin reliability diagrams; report ΔECE with bootstrap CIs across test folds.

---

# PHASE 5: Writing (Week 8)

## Day 1–2: Results assembly

Generate every figure from scripts (no manual plots — reviewers may ask for the pipeline, and you may need to re-run):

1. **Fig: Architecture** — update TikZ: graph nodes + external safety monitor (dashed box labelled "outside graph") + election FSM + envelope I/O boundary. Emphasise the safety-outside-graph property visually.
2. **Fig: Reliability diagrams** — 2×2: raw logprob / local calibrator / federated calibrator / per-drone. Headline figure.
3. **Fig: Hallucination taxonomy** — stacked bars: format/physical/coordination rates across A1/A2/default. Direct continuity with paper 1's taxonomy.
4. **Fig: ECE vs. FL round** — federation convergence.
5. **Table: Regression parity** — paper 1 vs. v2 static vs. v2 elected (the appendix table).
6. **Table: Election metrics** — success rate, duration, fallback count, determinism (EL1–EL5 results).
7. **Table (if handoff done):** handoff latency, message-loss, post-handoff MSR. **(If cut: a "Limitations" subsection, ~4 sentences, written this week — not left to the deadline.)**
8. **Table: Federated YOLO** — per-class mAP@0.5, local vs. federated.

## Day 3–5: Paper structure (IEEE conference format, reuse of paper-1 skeleton)

```
1  Introduction — contributions (4, per the scoping decision)
2  Related Work
   2.1 LLM/SLM agents for UAV swarms (LLM2Swarm, AutoHMA-LLM, FlockGPT)
       → gap: static roles, cloud models, no learning
   2.2 Federated LLM learning (OpenFedLLM, Bai et al., FedLLM-Bench)
       → gap: no embodied/safety-critical clients
   2.3 Hallucination calibration (multicalibration, I-CALM)
       → gap: centralized only, no cross-agent federation, no
         abstention-coordination
   2.4 Prior Edgents work [self-citation]
3  System Architecture
   3.1 LangGraph reformulation (graph schema, retry-as-conditional-edge,
       checkpointer + resume — NEW capability, screenshot/figure)
   3.2 Safety layer externalisation (INVARIANTS S1–S6 table — direct
       carry-over, structurally enforced role scopes now)
   3.3 Capability-based role election (scoring function, FSM, fallback)
   3.4 Mid-mission handoff (or §Limitations)
4  Federated Hallucination Calibration
   4.1 Feature/label taxonomy (automated effect-verification)
   4.2 FL setup, non-IID characterisation
   4.3 Results (headline)
5  Federated Vision Layer (scoped: capability demo + privacy motivation)
6  Evaluation
   6.1 Regression parity (appendix-grade table in body if space)
   6.2 Election & handoff metrics
   6.3 Ablations A1–A5
7  Discussion — bandwidth trade-off update (election adds 4 msgs),
   honesty about n, non-IID severity findings
8  Limitations & Future Work — privacy attacks, rotating-role
    calibration theory, N>2 agents, active abstention coordination.
    Our federation comprises N=2 physical clients; while this satisfies the formal FedAvg setting, results should be interpreted as two-way parameter averaging under genuine role heterogeneity rather than large-scale federation.
    If the E-FED-2 virtual-client variant ran, cross-reference its N=6 robustness result here.
9  Conclusion
```

**Writing rules:**
- Every number in the paper must trace to a `run_<id>` directory; keep a `numbers.md` mapping each claim → run ID, config hash, and git commit.
- Verify the two unverified citations (I-CALM 2026 preprint, multicalibration paper) against arXiv before they enter `thebibliography`. If unverifiable, drop them — the OpenFedLLM/FedLLM-Bench/Wei et al. anchors carry the related work alone.
- Carry over paper 1's bibliography wholesale; add: Edgents-v1 self-citation, LangGraph, Flower (FedML/FedAvg — McMahan et al. 2017 for FedAvg itself), ECE (Guo et al. 2017, "On Calibration of Modern Neural Networks" — this is the canonical calibration citation, add it if not present).

## Day 6: Internal validation pass

- [ ] Every claim in abstract appears with a number in Results.
- [ ] All four contributions have at least one experiment backing them.
- [ ] Reproducibility statement: repo layout, `requirements.lock`, config hashes, one-command scenario runner (`experiments/run_scenario.py --scenario s1 --config default`).
- [ ] Invariant tests S1–S6 pass on the final commit; CI badge/screenshot.
- [ ] Checkpointer-resume demo artifact (video or trace excerpt) ready as supplementary.

---

# Consolidated Deliverables Checklist

| # | Deliverable | Due | Verified by |
|---|---|---|---|
| D0 | `ENV_AUDIT.md` + `requirements.lock` | Day 0 | all smoke tests pass, including the in-venv `rclpy` path assertion |
| D1 | `SAFETY_INVARIANTS.md` + tests S1–S6 | Day 1 | `run_invariants.sh` green |
| D2 | `baseline_v1.json` (frozen) | Day 1 | committed, never edited |
| D3 | Cycle logger + L1 test | Day 2 | 10-cycle JSONL validation |
| D4 | Checkpoint-resume demo (T1.3) | Phase 1 | recorded artifact |
| D5 | Envelope schema + IO bridge + E1/E2 | Wk 1 | unit+integration green |
| D6 | Role-scoped compile + S5 test | Wk 1 | illegal-scope raise test |
| D7 | Parity run + `baseline_v2_parity.json` | Wk 2 | gate table pass |
| D8 | Election (EL1–EL5) + regression 2 | Wk 3 | EL1–**EL5** + parity |
| D9 | Handoff eval **or** written limitation | Wk 4 | 5 runs / §Limitations draft; incl. **HO4** |
| D10 | Calibration dataset (≥1,200 labelled cycles) | Wk 5 | label audit of 50 random rows |
| D11 | Centralized go/no-go (beats logprob ECE) | Wk 5 D5 | decision recorded |
| D12 | E-FED-1/2/3 results | Wk 6 | ΔECE tables with CIs; optional N=6 virtual-client robustness table |
| D13 | Federated YOLO delta table | Wk 6 | per-class mAP table |
| D14 | Ablations A1–A5 | Wk 7 | campaign directories |
| D15 | Headline campaign n=20 | Wk 7 D4 | Wilson CIs computed |
| D16 | All figures regenerable from scripts | Wk 8 D1 | re-run from clean clone |
| D17 | Verified bibliography | Wk 8 D5 | every ref checked on arXiv/publisher |
| D18 | Final paper + reproducibility package | Wk 8 D6 | D0–D17 traceable in `numbers.md` |

# Top 5 Schedule Risks & Pre-Committed Responses

1. **PX4/Gazebo won't build on the new OS (Day 0).** → Revert to Ubuntu 24.04 stack entirely; no schedule impact if caught Day 0.
2. **Python 3.14 wheel gaps.** → Pin 3.12 via `uv`; decided Day 0, never revisited.
3. **No logprobs from Ollama (Day 2 discovery).** → Switch to proxy-feature calibration immediately; the paper's claim becomes "structural features suffice for calibration" — still valid, still novel.
4. **Phase 2 parity fails after 3 bisect days.** → Freeze graph structure, port paper-1 prompt/tool logic verbatim as a compatibility mode, re-gate, then proceed. Never carry a known regression into Phase 3.
5. **Centralized calibrator fails go/no-go (Wk 5 D5).** → Two days of feature engineering (add n-gram features of thought text, tool-sequence features). If still failing by Wk 6 D2, drop FL-calibration to a negative-result subsection and promote federated YOLO + election to headline contributions. Decide by **Wk 6 D2**, not Wk 7.