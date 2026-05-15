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

The companion repository (`plansys2_llm_solver`) provides the LLM replanner used by this demo. **`plan_bookstore` is the worked end-to-end reference for integrating that replanner** — see `plansys2_llm_solver/INTEGRATION.md` (notably `Reception::step()` in `src/reception_controller_node.cpp`).

## License

Apache 2.0
