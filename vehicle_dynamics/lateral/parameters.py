
from dataclasses import dataclass
@dataclass
class BicycleParameters:
    m: float = 1400.0
    Iz: float = 2500.0
    a: float = 1.2
    b: float = 1.5
    v_eps: float = 0.5
    @property
    def L(self):
        return self.a + self.b
