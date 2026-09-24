# plansys2_llm_examples

Bookstore demo: a simulated Kobuki robot detects a misplaced colored book in the AWS bookstore world, picks it up, and returns it to its correct shelf.

Part of the [`plansys2-llm`](https://github.com/plansys2-llm) project.

## What this package provides

- **`plan_bookstore`** — single ROS 2 package shipping:
  - PDDL domain and problem files.
  - Behavior trees for the `move`, `pick_book`, `place_book` actions.
  - The main launch file `bookstore_kobuki_launch.py`.
  - Maps (`bookstore_map.{yaml,pgm}`) and the four colored book models (`colored_book_{red,green,blue,yellow}`).
  - A perception node (`perception_yolo_node`) that wraps `yolo_ros`, configured to detect the colored book objects.

## Installation and usage

This is one of two repositories that compose the project. **The full installation and usage instructions live in the organization home:**

> https://github.com/plansys2-llm

The companion repository (`plansys2_llm_monitor`) provides the LLM replanner used by this demo. **`plan_bookstore` is the worked end-to-end reference for integrating that replanner** — see `plansys2_llm_monitor/INTEGRATION.md` (notably `Reception::step()` in `src/reception_controller_node.cpp`).

## Preparing the recovery prompt while executing

`reception_controller_node` optionally publishes coherent live snapshots through
`MonitorClient::precompute()` while in `WORKING`. Enable both the controller's
`precompute_enabled: true` and the monitor's `LLAMA.precompute_enabled: true`,
with `LLAMA.pre_launch: true`. `precompute_period` defaults to 5 seconds. The
same observation builder is used before and after failure; only the dynamic
tail changes. Defaults leave preparation disabled. Preparation uses the CPU thread and batch
settings from the model YAML. Rebuild the extended interfaces as documented in
the companion `plansys2_llm_monitor/INTEGRATION.md` §10. Preparation is best-effort;
`getProposal()` always supplies the authoritative failure state.

`reception_controller_node` also accepts `wait_for_perception: true` to hold its
initial planning step until its filtered perception log contains a book with a
known location. The default is `false`, preserving immediate startup.
`bookstore_launch.py perception_mode:=external` omits the synthetic
`perception_sim_node` and expects another ROS perception source; the default
`perception_mode:=sim` keeps the existing demo behavior.

## License

Apache 2.0
