import re
from abc import ABC, abstractmethod
from enum import Enum
from typing import Any, Type


class ControlChannel(ABC):
    def __init__(self, id: int, name: str, value: int):
        self.id = id
        self.name = name
        self.value = value

    @abstractmethod
    def update_value(self, raw_value: float) -> None:
        pass

    @abstractmethod
    def get_value(self) -> Any:
        pass


class Axis(ControlChannel, ABC):
    def __init__(self, channel_id: int, name: str, value: int, value_range: Type[Enum]):
        super().__init__(channel_id, name, 0)
        self.value_range = value_range
        self.update_value(value)

    def update_value(self, raw_value: int) -> None:
        if self.value_range.MIN.value <= raw_value <= self.value_range.MAX.value:
            self.value = int(raw_value)
        else:
            closest_value = min(self.value_range, key=lambda x: abs(x.value - raw_value))

            self.value = int(closest_value.value)

    def get_value(self) -> int:
        return self.value


class Switch(ControlChannel, ABC):
    def __init__(self, channel_id: int, name: str, value: int, value_range: Type[Enum]):
        super().__init__(channel_id, name, 0)
        self.value_range = value_range
        self.update_value(value)

    def update_value(self, raw_value: float) -> None:
        closest_value = min(self.value_range, key=lambda x: abs(x.value - raw_value))

        self.value = closest_value

    def get_value(self) -> Type[Enum]:
        return self.value


class AxisRange(Enum):
    MIN = 1000
    MAX = 2000


class ToggleSwitch(Enum):
    UP = 1000
    DOWN = 2000


class TernarySwitch(Enum):
    UP = 1000
    MIDDLE = 1500
    DOWN = 2000


class ChannelProcessor:
    def __init__(self, *watch_channel):
        self.channels = {}

        for channel in watch_channel:
            if isinstance(channel, ControlChannel):
                self.channels[channel.id] = channel
            else:
                raise ValueError(f"Invalid channel type: {type(channel)}")

    def update_channel(self, key, value):
        if key in self.channels:
            self.channels[key].update_value(value)

    def add_data(self, mavlink_message):
        pattern = r'chan(\d+)_raw'

        message_dict = mavlink_message.__dict__

        for key, value in message_dict.items():
            if re.match(pattern, key):
                key_id = int(re.match(pattern, key).group(1))
                self.update_channel(key_id, value)
