from typing import List, Tuple

from pyvrp import CostEvaluator, ProblemData, Solution

def selective_route_exchange(
    parents: Tuple[Solution, Solution],
    data: ProblemData,
    cost_evaluator: CostEvaluator,
    start_indices: Tuple[int, int],
    num_moved_routes: int,
) -> Solution: ...
def heterogeneous_selective_route_exchange(
    parents: Tuple[Solution, Solution],
    data: ProblemData,
    cost_evaluator: CostEvaluator,
    start_indices_per_vehicle_type: List[int],
    num_moved_routes_per_vehicle_type: List[int],
) -> Solution: ...
def route_exchange(
    parents: Tuple[Solution, Solution],
    data: ProblemData,
    cost_evaluator: CostEvaluator,
    exchanges: List[int],
) -> Solution: ...
