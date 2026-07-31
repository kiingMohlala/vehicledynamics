from dataclasses import dataclass

@dataclass
class BicycleParameters:
    """Phase 4.0 default bicycle model parameters."""
    m: float = 1400.0          # mass [kg]
    Iz: float = 2500.0         # yaw inertia [kg·m²]
    a: float = 1.2             # CG to front axle [m]
    b: float = 1.5             # CG to rear axle [m]
    v_eps: float = 0.5         # speed regularization [m/s]

    @property
    def L(self) -> float:
        return self.a + self.b
