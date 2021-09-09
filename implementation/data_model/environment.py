#################### BEGIN LICENSE BLOCK ###############################
#
# Copyright (C) 2021 Fraunhofer IESE
#
# SPDX-License-Identifier: LGPL-2.1-only
#
#################### END LICENSE BLOCK #################################

from dataclasses import dataclass
from enum import Enum
from typing import Optional


@dataclass()
class DateTime:
    hour: int = 0
    minute: int = 0
    second: int = 0
    millisecond: int = 0


@dataclass()
class TimeOfDay:
    animation: bool = 0
    dateTime: Optional[DateTime] = None


@dataclass()
class Wind:
    direction: float = 0
    speed: float = 0


@dataclass()
class Sun:
    azimuth: float = 0
    elevation: float = 0
    intensity: float = 0


@dataclass()
class PrecipitationType(Enum):
    DRY = 1
    RAIN = 2
    SNOW = 3


@dataclass()
class Precipitation:
    intensity: float = 0
    precipitationType: Optional[PrecipitationType] = 0


@dataclass()
class CloudState(Enum):
    FREE = 1
    CLOUDY = 2
    OVERCAST = 3
    RAINY = 4
    SKY_OFF = 5


@dataclass()
class Fog:
    visualRange: float = 0


@dataclass()
class RoadCondition:
    # TODO add Properties
    frictionScaleFactor: float


@dataclass()
class Weather:
    athmosphericPressure: Optional[float] = None
    temperature: Optional[float] = None
    wind: Optional[Wind] = None
    sun: Optional[Sun] = None
    precipitation: Optional[Precipitation] = None
    cloudstate: Optional[CloudState] = None
    fog: Optional[Fog] = None


@dataclass()
class Environment:
    name: str = ""
    time_of_day: Optional[TimeOfDay] = None
    weather: Optional[Weather] = None
    road_condition: Optional[RoadCondition] = None
