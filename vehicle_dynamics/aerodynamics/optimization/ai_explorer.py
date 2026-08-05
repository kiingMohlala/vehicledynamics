"""AI-assisted design exploration via surrogate + uncertainty sampling."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .design_variables import DesignVector, DesignBounds, default_bounds
from .objective_functions import evaluate_objectives
from .surrogate_model import SurrogateModel, train_surrogate
from .constraints import evaluate_constraints


@dataclass
class ExplorationResult:
    candidates: list[DesignVector]
    predicted_objs: np.ndarray
    true_objs: np.ndarray | None
    surrogate: SurrogateModel


class AIExplorer:
    """
    Active-learning style explorer:
    1. Evaluate seed designs
    2. Train surrogate
    3. Propose candidates maximizing predicted DF / L/D with diversity
    """

    def __init__(self, bounds: DesignBounds | None = None, seed: int = 0):
        self.bounds = bounds or default_bounds()
        self.rng = np.random.default_rng(seed)

    def explore(
        self,
        n_seed: int = 20,
        n_candidates: int = 10,
        evaluate_true: bool = True,
    ) -> ExplorationResult:
        seeds: list[DesignVector] = []
        objs = []
        for _ in range(n_seed * 3):
            if len(seeds) >= n_seed:
                break
            d = self.bounds.random(self.rng)
            o = evaluate_objectives(d, speed=50.0)
            ok, _ = evaluate_constraints(d, o.drag, o.front_balance)
            if ok:
                seeds.append(d)
                objs.append([o.downforce, o.drag, o.L_over_D, o.lap_time])
        objs_arr = np.asarray(objs)
        surr = train_surrogate(seeds, objs_arr)

        # Propose candidates: sample and rank by surrogate L/D and DF
        cands = []
        preds = []
        for _ in range(n_candidates * 15):
            if len(cands) >= n_candidates:
                break
            d = self.bounds.random(self.rng)
            # normalize for surrogate
            scale = surr.X.std(axis=0) * surr.length_scale + 1e-9
            # Surrogate was trained on X / scale_train — approximate with raw IDW on seeds
            pred = surr.predict(d.as_array() / (seeds[0].as_array() * 0 + 1))  # fallback
            # Better: distance in design space to seeds
            X = np.vstack([s.as_array() for s in seeds])
            sc = X.std(axis=0) + 1e-9
            dist = np.linalg.norm((X - d.as_array()) / sc, axis=1)
            w = 1.0 / np.power(np.maximum(dist, 1e-9), 2)
            w /= w.sum()
            pred = w @ objs_arr
            o_check = evaluate_objectives(d, speed=50.0)
            ok, _ = evaluate_constraints(d, o_check.drag, o_check.front_balance)
            if not ok:
                continue
            # Score: high DF, high L/D, low lap
            score = pred[0] / 1000.0 + pred[2] - pred[3] / 50.0
            cands.append((score, d, pred))
        cands.sort(key=lambda t: -t[0])
        cands = cands[:n_candidates]
        designs = [c[1] for c in cands]
        pred_mat = np.vstack([c[2] for c in cands]) if cands else np.zeros((0, 4))

        true_mat = None
        if evaluate_true and designs:
            true_mat = np.vstack([
                [
                    evaluate_objectives(d, speed=50.0).downforce,
                    evaluate_objectives(d, speed=50.0).drag,
                    evaluate_objectives(d, speed=50.0).L_over_D,
                    evaluate_objectives(d, speed=50.0).lap_time,
                ]
                for d in designs
            ])

        return ExplorationResult(
            candidates=designs,
            predicted_objs=pred_mat,
            true_objs=true_mat,
            surrogate=surr,
        )
