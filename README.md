# plansys2_llm_examples

Bookstore demo for the [`plansys2-llm`](https://github.com/plansys2-llm) project. A simulated Kobuki robot detects a misplaced colored book in the AWS bookstore world, picks it up, and returns it to its correct shelf — all driven by a PDDL plan generated with the help of an LLM solver.

## What this package provides

- `plan_bookstore` ROS 2 package, including:
  - PDDL domain and problem files for the bookstore task.
  - Behavior trees for the `move`, `pick_book`, `place_book` actions.
  - The main launch file `bookstore_kobuki_launch.py`.
  - Maps (`bookstore_map.{yaml,pgm}`) and the four colored book models (`colored_book_{red,green,blue,yellow}`).
  - A perception node (`perception_yolo_node`) that wraps `yolo_ros` for the bookstore-specific class set.

## Installation and usage

This is one of two repositories that compose the project. **The full installation and run instructions live in the organization home:**

> https://github.com/plansys2-llm

The companion repository (`plansys2_llm_solver`) is required for this demo to work.

## License

Apache 2.0
