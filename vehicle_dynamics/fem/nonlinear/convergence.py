"""Newton–Raphson convergence logging."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class IterationRecord:
    iteration: int
    residual_norm: float
    correction_norm: float
    load_factor: float
    converged: bool = False


@dataclass
class ConvergenceLog:
    records: list[IterationRecord] = field(default_factory=list)

    def add(
        self,
        iteration: int,
        residual_norm: float,
        correction_norm: float,
        load_factor: float = 1.0,
        converged: bool = False,
    ) -> None:
        self.records.append(
            IterationRecord(
                iteration=iteration,
                residual_norm=residual_norm,
                correction_norm=correction_norm,
                load_factor=load_factor,
                converged=converged,
            )
        )

    @property
    def n_iter(self) -> int:
        return len(self.records)

    def summary(self) -> str:
        lines = ["Iter   Residual        Correction      λ      Conv"]
        for r in self.records:
            lines.append(
                f"{r.iteration:4d}  {r.residual_norm:12.4e}  {r.correction_norm:12.4e}  "
                f"{r.load_factor:5.2f}  {r.converged}"
            )
        return "\n".join(lines)
