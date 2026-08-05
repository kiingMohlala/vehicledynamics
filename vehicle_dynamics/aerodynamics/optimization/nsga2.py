"""Compact NSGA-II for aero multi-objective optimization."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .design_variables import DesignVector, DesignBounds, default_bounds
from .objective_functions import evaluate_objectives
from .constraints import evaluate_constraints, ConstraintSet
from .pareto import dominates


@dataclass
class NSGA2Config:
    pop_size: int = 40
    n_gen: int = 25
    crossover_eta: float = 15.0
    mutation_eta: float = 20.0
    mutation_rate: float = 0.2
    seed: int = 0
    speed: float = 50.0


@dataclass
class NSGA2Result:
    population: list[DesignVector]
    objectives: np.ndarray          # (n, 3) DF, drag, lap_time
    pareto_indices: np.ndarray
    history_best_lap: list[float]


def _obj_row(d: DesignVector, speed: float) -> np.ndarray:
    o = evaluate_objectives(d, speed=speed)
    return np.array([o.downforce, o.drag, o.lap_time])


def _feasible(d: DesignVector, speed: float) -> bool:
    o = evaluate_objectives(d, speed=speed)
    ok, _ = evaluate_constraints(d, o.drag, o.front_balance)
    return ok


def _nondominated_sort(objs: np.ndarray) -> list[list[int]]:
    """Return fronts as lists of indices. Maximize DF, minimize drag & lap."""
    n = objs.shape[0]
    max_mask = np.array([True, False, False])
    S = [[] for _ in range(n)]
    n_dom = np.zeros(n, dtype=int)
    ranks = np.zeros(n, dtype=int)
    fronts: list[list[int]] = [[]]
    for p in range(n):
        for q in range(n):
            if p == q:
                continue
            if dominates(objs[p], objs[q], max_mask):
                S[p].append(q)
            elif dominates(objs[q], objs[p], max_mask):
                n_dom[p] += 1
        if n_dom[p] == 0:
            ranks[p] = 0
            fronts[0].append(p)
    i = 0
    while fronts[i]:
        nxt = []
        for p in fronts[i]:
            for q in S[p]:
                n_dom[q] -= 1
                if n_dom[q] == 0:
                    ranks[q] = i + 1
                    nxt.append(q)
        i += 1
        fronts.append(nxt)
    return fronts[:-1]


def _crowding(objs: np.ndarray, front: list[int]) -> np.ndarray:
    n = len(front)
    if n == 0:
        return np.array([])
    crowd = np.zeros(n)
    if n <= 2:
        crowd[:] = np.inf
        return crowd
    F = objs[front]
    for m in range(F.shape[1]):
        order = np.argsort(F[:, m])
        crowd[order[0]] = np.inf
        crowd[order[-1]] = np.inf
        span = F[order[-1], m] - F[order[0], m]
        if span < 1e-12:
            continue
        for i in range(1, n - 1):
            crowd[order[i]] += (F[order[i + 1], m] - F[order[i - 1], m]) / span
    return crowd


def nsga2_optimize(
    bounds: DesignBounds | None = None,
    config: NSGA2Config | None = None,
) -> NSGA2Result:
    bounds = bounds or default_bounds()
    cfg = config or NSGA2Config()
    rng = np.random.default_rng(cfg.seed)
    lo, hi = bounds.low.as_array(), bounds.high.as_array()
    n, d = cfg.pop_size, bounds.n_dim()

    # Init
    pop = []
    for _ in range(n):
        for _try in range(50):
            cand = bounds.random(rng)
            if _feasible(cand, cfg.speed):
                pop.append(cand)
                break
        else:
            pop.append(bounds.clip(bounds.random(rng)))

    objs = np.vstack([_obj_row(p, cfg.speed) for p in pop])
    hist = [float(np.min(objs[:, 2]))]

    def sbx(p1, p2):
        u = rng.random(d)
        beta = np.where(
            u <= 0.5,
            (2 * u) ** (1 / (cfg.crossover_eta + 1)),
            (1 / (2 * (1 - u))) ** (1 / (cfg.crossover_eta + 1)),
        )
        c1 = 0.5 * ((p1 + p2) - beta * (p2 - p1))
        c2 = 0.5 * ((p1 + p2) + beta * (p2 - p1))
        return np.clip(c1, lo, hi), np.clip(c2, lo, hi)

    def mutate(x):
        x = x.copy()
        for i in range(d):
            if rng.random() < cfg.mutation_rate:
                u = rng.random()
                delta = (2 * u) ** (1 / (cfg.mutation_eta + 1)) - 1 if u < 0.5 else 1 - (2 * (1 - u)) ** (1 / (cfg.mutation_eta + 1))
                x[i] += delta * (hi[i] - lo[i])
        return np.clip(x, lo, hi)

    for _gen in range(cfg.n_gen):
        # Offspring
        offspring = []
        idx = rng.permutation(n)
        for i in range(0, n - 1, 2):
            p1 = pop[idx[i]].as_array()
            p2 = pop[idx[i + 1]].as_array()
            c1, c2 = sbx(p1, p2)
            offspring.append(DesignVector.from_array(mutate(c1)))
            offspring.append(DesignVector.from_array(mutate(c2)))
        if len(offspring) < n:
            offspring.append(bounds.random(rng))

        combined = pop + offspring[:n]
        comb_objs = np.vstack([_obj_row(p, cfg.speed) for p in combined])
        # Penalize infeasible
        for i, des in enumerate(combined):
            if not _feasible(des, cfg.speed):
                comb_objs[i, 2] += 50.0  # lap time penalty
                comb_objs[i, 1] += 500.0

        fronts = _nondominated_sort(comb_objs)
        new_pop = []
        new_objs = []
        for front in fronts:
            if len(new_pop) + len(front) <= n:
                for i in front:
                    new_pop.append(combined[i])
                    new_objs.append(comb_objs[i])
            else:
                crowd = _crowding(comb_objs, front)
                order = np.argsort(-crowd)
                for j in order:
                    if len(new_pop) >= n:
                        break
                    i = front[j]
                    new_pop.append(combined[i])
                    new_objs.append(comb_objs[i])
                break
        pop = new_pop[:n]
        objs = np.vstack(new_objs[:n])
        hist.append(float(np.min(objs[:, 2])))

    # Final Pareto
    from .pareto import pareto_front
    pidx = pareto_front(objs, maximize_mask=np.array([True, False, False]))
    return NSGA2Result(
        population=pop,
        objectives=objs,
        pareto_indices=pidx,
        history_best_lap=hist,
    )
