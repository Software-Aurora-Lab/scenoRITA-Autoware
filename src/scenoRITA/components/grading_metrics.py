import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from scenoRITA.components.oracles.Violation import Violation
from scenoRITA.components.oracles import RecordAnalyzer
from scenoRITA.components.oracles.impl import (
    Comfort,
    Collision,
    Speeding,
    UnsafeLaneChange
)


@dataclass(slots=True)
class GradingResult:
    scenario_id: str
    record: Path
    fitnesses: Dict[int, Tuple[float, ...]]
    violations: List[Violation]


def registered_metrics():
    return [
        Collision(),
        Speeding(),
        UnsafeLaneChange(),
        Comfort(),
    ]


def grade_scenario(scenario_id: str, record: Path) -> Optional[GradingResult]:
    trial = 0
    while trial < 3:
        try:
            metrics = registered_metrics()
            record_file = RecordAnalyzer(str(record), metrics)
            violations = record_file.analyze()
            collision, speeding, unsafe_lane_change, comfort = metrics

            collision_fitness = collision.get_fitness()
            fitness: Dict[int, Tuple[float, ...]] = dict()
            for k in collision_fitness:
                fitness[k] = (
                    collision_fitness[k],  # collision
                    speeding.get_fitness(),  # speeding
                    unsafe_lane_change.get_fitness(),  # unsafe lane change
                    comfort.get_fitness()[0],  # fast accel
                    comfort.get_fitness()[1]  # hard brake
                )

            return GradingResult(
                scenario_id,
                record,
                fitness,
                violations,
            )
        except Exception:
            trial += 1
            time.sleep(1)
    return None
