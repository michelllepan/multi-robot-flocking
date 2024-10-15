from dataclasses import dataclass

@dataclass
class WeightMode:
    goal: float
    bounds_aversion: float
    cohesion: float
    separation: float
    alignment: float
    drive_at_human: float
    linear: float


_weight_modes = {
    "DEFAULT": 
        WeightMode(
            goal             =0.0,
            bounds_aversion  =0.2,
            cohesion         =0.5,
            separation       =1.0,
            alignment        =1.0,
            drive_at_human   =0.1,
            linear           =0.0,
        ),
    "FOLLOW": 
        WeightMode(
            goal             =0.0,
            bounds_aversion  =1.0,
            cohesion         =0.0,
            separation       =6.0,
            alignment        =0.0,
            drive_at_human   =6.0,
            linear           =0.0,
        ),
    "SPREAD": 
        WeightMode(
            goal             =0.0,
            bounds_aversion  =1.0,
            cohesion         =0.0,
            separation       =1.0,
            alignment        =0.0,
            drive_at_human   =0.0,
            linear           =0.0,
        ),
    "CIRCLE": 
        WeightMode(
            goal             =1.0,
            bounds_aversion  =0.0,
            cohesion         =0.0,
            separation       =1.0,
            alignment        =0.0,
            drive_at_human   =0.0,
            linear           =0.0,
        ),
    "ALIGN": 
        WeightMode(
            goal             =0.0,
            bounds_aversion  =1.0,
            cohesion         =0.0,
            separation       =2.0,
            alignment        =2.0,
            drive_at_human   =0.0,
            linear           =0.0,
        ),
    "COHERE": 
        WeightMode(
            goal             =0.0,
            bounds_aversion  =1.0,
            cohesion         =1.0,
            separation       =1.0,
            alignment        =0.0,
            drive_at_human   =0.0,
            linear           =0.0,
        ),
    "LINEAR_TRACKS": 
        WeightMode(
            goal             =0.0,
            bounds_aversion  =1.0,
            cohesion         =0.0,
            separation       =2.0,
            alignment        =0.0,
            drive_at_human   =0.0,
            linear           =1.0,
        ),
}

def get_weight_mode(mode: str) -> WeightMode:
    return _weight_modes[mode]