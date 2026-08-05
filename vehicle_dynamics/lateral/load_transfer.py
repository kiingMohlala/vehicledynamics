from dataclasses import dataclass

@dataclass
class LoadTransferParameters:
    h_cg: float = 0.55
    track_f: float = 1.55
    track_r: float = 1.55
    chi_f: float = 0.55
    Fz_min: float = 50.0

@dataclass
class LoadTransferState:
    dFz_front: float
    dFz_rear: float
    Fz_fl: float
    Fz_fr: float
    Fz_rl: float
    Fz_rr: float
    wheel_lift_front: bool
    wheel_lift_rear: bool

def _clamp_axle(Fz_left, Fz_right, Fz_axle, Fz_min):
    lift = False
    if Fz_left < Fz_min:
        Fz_left = Fz_min
        Fz_right = Fz_axle - Fz_left
        lift = True
    if Fz_right < Fz_min:
        Fz_right = Fz_min
        Fz_left = Fz_axle - Fz_right
        lift = True
    if Fz_axle < 2.0 * Fz_min:
        Fz_left = Fz_min
        Fz_right = Fz_min
        lift = True
    return Fz_left, Fz_right, lift

def compute_load_transfer(ay, Fz_f_axle, Fz_r_axle, params=None, mass=1400.0):
    if params is None:
        params = LoadTransferParameters()
    dFz_f = (mass * ay * params.h_cg / params.track_f) * params.chi_f
    dFz_r = (mass * ay * params.h_cg / params.track_r) * (1.0 - params.chi_f)
    Fz_fl = Fz_f_axle / 2.0 - dFz_f
    Fz_fr = Fz_f_axle / 2.0 + dFz_f
    Fz_rl = Fz_r_axle / 2.0 - dFz_r
    Fz_rr = Fz_r_axle / 2.0 + dFz_r
    Fz_fl, Fz_fr, lift_f = _clamp_axle(Fz_fl, Fz_fr, Fz_f_axle, params.Fz_min)
    Fz_rl, Fz_rr, lift_r = _clamp_axle(Fz_rl, Fz_rr, Fz_r_axle, params.Fz_min)
    return LoadTransferState(dFz_f, dFz_r, Fz_fl, Fz_fr, Fz_rl, Fz_rr, lift_f, lift_r)
