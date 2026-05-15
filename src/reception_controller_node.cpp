// Copyright 2026 Álvaro Valencia
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <deque>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "plansys2_solver/SolverClient.hpp"
#include "plansys2_solver/SolverNode.hpp"

#include "std_msgs/msg/string.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class Reception : public rclcpp::Node
{
public:
  Reception()
  : rclcpp::Node("reception_controller"), state_(StateType::STARTING)
  {
  }

  bool init()
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();
    if (!has_parameter("solver_timeout")) {
      declare_parameter<double>("solver_timeout", 150.0);
    }
    set_parameter(rclcpp::Parameter("solver_timeout", 150.0));
    solver_client_ = std::make_shared<plansys2::SolverClient>();

    // Backup perception log used in solver prompt when /actions_hub dedup drops events.
    // Capped to bound LLM prompt size — the LLM only needs recent context, not the full plan history.
    perception_sub_ = create_subscription<std_msgs::msg::String>(
      "/perception_events", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        perception_log_.push_back(msg->data);
        if (perception_log_.size() > MAX_PERCEPTION_LOG) {
          perception_log_.pop_front();
        }
        RCLCPP_INFO(get_logger(), "[PERCEPTION] Logged: %s", msg->data.c_str());
      });

    init_knowledge();

    return true;
  }

  void init_knowledge()
  {
    problem_expert_->addInstance(plansys2::Instance{"shelf_red", "location"});
    problem_expert_->addInstance(plansys2::Instance{"shelf_green", "location"});
    problem_expert_->addInstance(plansys2::Instance{"shelf_yellow", "location"});
    problem_expert_->addInstance(plansys2::Instance{"shelf_blue", "location"});
    problem_expert_->addInstance(plansys2::Instance{"middle_path", "location"});
    problem_expert_->addInstance(plansys2::Instance{"deposit_table", "location"});
    problem_expert_->addInstance(plansys2::Instance{"reception", "location"});

    problem_expert_->addInstance(plansys2::Instance{"curiosity", "robot"});

    problem_expert_->addInstance(plansys2::Instance{"red_book", "item"});
    problem_expert_->addInstance(plansys2::Instance{"green_book", "item"});
    problem_expert_->addInstance(plansys2::Instance{"yellow_book", "item"});
    problem_expert_->addInstance(plansys2::Instance{"blue_book", "item"});

    problem_expert_->addPredicate(plansys2::Predicate("(is_book red_book)"));
    problem_expert_->addPredicate(plansys2::Predicate("(is_book green_book)"));
    problem_expert_->addPredicate(plansys2::Predicate("(is_book yellow_book)"));
    problem_expert_->addPredicate(plansys2::Predicate("(is_book blue_book)"));

    problem_expert_->addPredicate(plansys2::Predicate("(is_deposit deposit_table)"));
    problem_expert_->addPredicate(plansys2::Predicate("(is_base reception)"));

    problem_expert_->addPredicate(plansys2::Predicate("(robot_at curiosity reception)"));
    problem_expert_->addPredicate(plansys2::Predicate("(gripper_free curiosity)"));
    problem_expert_->addPredicate(plansys2::Predicate("(doing_nthg curiosity)"));

    problem_expert_->addPredicate(plansys2::Predicate("(object_at red_book shelf_red)"));
    problem_expert_->addPredicate(plansys2::Predicate("(object_at green_book shelf_green)"));
    problem_expert_->addPredicate(plansys2::Predicate("(object_at yellow_book shelf_yellow)"));
    problem_expert_->addPredicate(plansys2::Predicate("(object_at blue_book shelf_blue)"));

    choose_displaced_book();
  }

  void choose_displaced_book()
  {
    // Idempotent: declare the param only once, and keep the same displaced
    // book across replays so every loop runs the identical scenario.
    if (!has_parameter("displaced_book")) {
      declare_parameter<std::string>("displaced_book", "");
    }
    if (displaced_book_.empty()) {
      displaced_book_ = get_parameter("displaced_book").as_string();
    }

    if (displaced_book_.empty()) {
      std::vector<std::string> books = {
        "red_book", "green_book", "yellow_book", "blue_book"};

      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_int_distribution<> dist(0, books.size() - 1);

      displaced_book_ = books[dist(gen)];
    }

    RCLCPP_WARN(get_logger(),
      "[SCENARIO] Displaced book: %s (not physically at its shelf, "
      "but PDDL still believes it is)", displaced_book_.c_str());
  }

  void step()
  {
    switch (state_)
    {
      case StateType::STARTING:
        RCLCPP_INFO(get_logger(), "State: STARTING -> PLANNING");
        state_ = StateType::PLANNING;
        break;

      case StateType::PLANNING:
        {
          RCLCPP_INFO(get_logger(), "State: PLANNING");

          problem_expert_->setGoal(plansys2::Goal(
            "(and (book_deposited red_book) (book_deposited green_book) "
            "(book_deposited yellow_book) (book_deposited blue_book) "
            "(robot_at curiosity reception))"));

          auto domain = domain_expert_->getDomain();
          auto problem = problem_expert_->getProblem();
          auto plan = planner_client_->getPlan(domain, problem);

          if (!plan.has_value()) {
            RCLCPP_ERROR(get_logger(), "Could not find plan to reach goal %s",
              parser::pddl::toString(problem_expert_->getGoal()).c_str());
            state_ = StateType::FINISH;
            break;
          }

          RCLCPP_INFO(get_logger(), "Plan found, starting execution");
          if (executor_client_->start_plan_execution(plan.value())) {
            state_ = StateType::WORKING;
          }
          break;
        }

      case StateType::WORKING:
        {
          if (!executor_client_->execute_and_check_plan()) {
            auto result = executor_client_->getResult();

#ifdef PLANSYS2_HAS_RESULT_ENUM
            if (result.value().result ==
                plansys2_msgs::action::ExecutePlan::Result::SUCCESS)
#else
            if (result.value().success)
#endif
            {
              RCLCPP_WARN(get_logger(),
                "[LOOP] Plan successfully finished (run %zu). Resetting and "
                "replaying. Process stays alive so the solver / llama_node "
                "(KV-cache) stays warm. Ctrl-C to stop.", ++run_);

              // Reset PDDL belief to the initial scenario and replay. The
              // physical world (Gazebo) is NOT reset, so run >=2 still hits
              // the displaced-book mismatch -> the solver is queried again,
              // now against a warm KV-cache.
              problem_expert_->clearGoal();
              problem_expert_->clearKnowledge();
              perception_log_.clear();
              init_knowledge();
              state_ = StateType::PLANNING;
            } else {
              RCLCPP_ERROR(get_logger(),
                "[EXECUTION_FAILURE] Plan failed. Querying solver with perception log...");

              // pick_book's at-start effects remove doing_nthg+gripper_free; restore
              // on failure so replan preconditions hold.
              problem_expert_->addPredicate(plansys2::Predicate("(doing_nthg curiosity)"));
              problem_expert_->addPredicate(plansys2::Predicate("(gripper_free curiosity)"));
              RCLCPP_INFO(get_logger(),
                "[RECOVERY] Restored doing_nthg + gripper_free after execution failure");

              auto domain = domain_expert_->getDomain();
              auto problem = problem_expert_->getProblem();

              std::string perception_context = build_perception_context();

              // The failed action as a readable call, e.g.
              // "pick_book(curiosity, red_book, shelf_red)" - deliberately NOT
              // a PDDL S-expression so the LLM does not copy it as a predicate.
              std::string failed_action = "unknown";
              for (const auto & info : result.value().action_execution_status) {
                if (info.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                  failed_action = info.action + "(";
                  for (size_t i = 0; i < info.arguments.size(); ++i) {
                    failed_action += (i ? ", " : "") + info.arguments[i];
                  }
                  failed_action += ")";
                  break;
                }
              }

              std::string prompt =
                "A robot action just failed: the world is not as the PDDL "
                "problem believes. The perception list below is GROUND TRUTH "
                "about the objects the robot saw. Your only job: make the "
                "problem's facts about those objects match perception. This is "
                "belief correction, NOT planning - never reason about where an "
                "object should go or what it should be, only what perception "
                "says it IS now.\n"
                "For each perceived object whose problem fact disagrees: "
                "remove the stale fact and add the corrected one using the "
                "matching Domain predicate. Such a fact contains only the "
                "object and the perceived value (e.g. its location); it never "
                "contains the robot or an action name.\n\n"
                "The failed action was " + failed_action + ". The objects it "
                "names are the ones whose beliefs are most likely wrong - "
                "check those first against perception.\n\n"
                "Perception:\n" + perception_context;

              auto solver_result = solver_client_->getReplanificateSolve(
                domain, problem, prompt, "");

              if (!solver_result.has_value()) {
                RCLCPP_ERROR(get_logger(), "[EXECUTION_FAILURE] Solver did not respond");
                state_ = StateType::FINISH;
              } else if (solver_result->classification ==
                plansys2_solver_msgs::msg::Solver::CORRECT)
              {
                RCLCPP_INFO(get_logger(),
                  "[EXECUTION_FAILURE] LLM: no state changes needed, replanning anyway");
                state_ = StateType::PLANNING;
              } else {
                RCLCPP_INFO(get_logger(),
                  "[EXECUTION_FAILURE] Applying LLM state corrections...");
                for (const auto & pred_str : solver_result->remove_predicates) {
                  bool ok = problem_expert_->removePredicate(plansys2::Predicate(pred_str));
                  RCLCPP_INFO(get_logger(), "REMOVE %s: %s",
                    pred_str.c_str(), ok ? "ok" : "failed");
                }
                for (const auto & pred_str : solver_result->add_predicates) {
                  bool ok = problem_expert_->addPredicate(plansys2::Predicate(pred_str));
                  RCLCPP_INFO(get_logger(), "ADD %s: %s",
                    pred_str.c_str(), ok ? "ok" : "failed");
                }
                state_ = StateType::PLANNING;
              }
            }
          }
          break;
        }

      case StateType::FINISH:
        {
          RCLCPP_INFO(get_logger(), "State: FINISH");
          rclcpp::shutdown();
          break;
        }

      default:
        break;
    }
  }

  // The LLM is unreliable at filtering the raw event stream (mostly
  // location:null noise), so resolve "book color -> observed location" in
  // code and feed only that. Highest-confidence non-null sighting per color.
  std::string build_perception_context()
  {
    std::map<std::string, std::pair<std::string, double>> best;
    for (const auto & raw : perception_log_) {
      try {
        auto j = nlohmann::json::parse(raw);
        if (j.value("observation", "") != "object_detected") {continue;}
        if (j.value("object", "") != "book") {continue;}
        if (j["color"].is_null() || j["location"].is_null()) {continue;}
        const auto color = j["color"].get<std::string>();
        const auto loc = j["location"].get<std::string>();
        const double conf = j.value("confidence", 0.0);
        auto it = best.find(color);
        if (it == best.end() || conf > it->second.second) {
          best[color] = {loc, conf};
        }
      } catch (const nlohmann::json::exception &) {
        continue;
      }
    }

    if (best.empty()) {
      return "No observation reported any book at a known location.";
    }
    std::string ctx =
      "Each line is a book color and where it was observed "
      "(the PDDL item is named <color>_book):\n";
    for (const auto & [color, lc] : best) {
      ctx += "- " + color + " book -> " + lc.first + "\n";
    }
    return ctx;
  }



private:
  static constexpr size_t MAX_PERCEPTION_LOG = 8;

  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
  std::shared_ptr<plansys2::SolverClient> solver_client_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr perception_sub_;
  std::deque<std::string> perception_log_;

  std::string displaced_book_;
  size_t run_ = 0;  // completed-plan counter for the KV-cache replay loop

  enum class StateType { STARTING, PLANNING, WORKING, FINISH };
  StateType state_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto reception_node = std::make_shared<Reception>();

  // Ensure the reception node has solver_timeout so SolverClient picks it up
  if (!reception_node->has_parameter("solver_timeout")) {
    reception_node->declare_parameter<double>("solver_timeout", 150.0);
  }

  auto solver_node = std::make_shared<plansys2::SolverNode>();

  try {
    solver_node->configure();
  } catch (const std::exception & e) {
    std::cerr << "Error creating SolverNode: " << e.what() << std::endl;
    return 1;
  }

  if (!reception_node->init()) return 0;

  rclcpp::executors::SingleThreadedExecutor solver_executor;
  solver_executor.add_node(solver_node->get_node_base_interface());
  std::thread solver_thread([&solver_executor]() { solver_executor.spin(); });

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(reception_node);

  rclcpp::Rate rate(5);
  while (rclcpp::ok()) {
    reception_node->step();
    executor.spin_some();
    rate.sleep();
  }

  solver_executor.cancel();
  solver_thread.join();

  rclcpp::shutdown();
  return 0;
}
