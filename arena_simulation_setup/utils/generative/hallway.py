import logging
import random

from . import (GeneratedWorld, Polygon, WorldGeneratorType, _BaseConfiguration,
               _WorldGenerator, _WorldGeneratorImpl)

logger = logging.getLogger(__name__)


@_WorldGenerator.register(WorldGeneratorType.HALLWAY)
class WorldGeneratorHallway(_WorldGeneratorImpl):

    class Configuration(_BaseConfiguration):
        width: float = 80.0
        height: float = 50.0

        # Hallway parameters (a horizontal band)
        hallway_height: float = 5.0

        # Room parameters (for each side)
        rooms_per_side: int = 7

        # For "big" rooms (first and last)
        big_min_width: float = 18.0
        big_max_width: float = 28.0
        big_min_height: float = 12.0
        big_max_height: float = 20.0

        # For "small" rooms (intermediate ones)
        small_min_width: float = 6.0
        small_max_width: float = 12.0
        small_min_height: float = 7.0
        small_max_height: float = 14.0

        # door width
        door_width: float = 2.5

        @property
        def hallway_top(self) -> float:
            return self.height / 2 + self.hallway_height / 2

        @property
        def hallway_bottom(self) -> float:
            return self.height / 2 - self.hallway_height / 2

    config: Configuration

    def configure(self, configuration: dict):
        self.config = self.Configuration.model_validate(configuration)
        logger.info(self.config)

    def compute(self) -> GeneratedWorld:
        top_rooms, top_doors = self._impl("top", self.config.rooms_per_side)
        bottom_rooms, bottom_doors = self._impl("bottom", self.config.rooms_per_side)

        return GeneratedWorld(
            rooms=top_rooms + bottom_rooms,
            doors=top_doors + bottom_doors,
            width=self.config.width,
            height=self.config.height,
            resolution=self.config.resolution
        )

    def _impl(self, side, num_rooms) -> tuple[list[Polygon], list[Polygon]]:
        rooms: list[Polygon] = []
        doors: list[Polygon] = []

        widths: list[float] = []
        heights: list[float] = []

        rooms.append([
            (0, self.config.hallway_bottom),
            (self.config.width, self.config.hallway_bottom),
            (self.config.width, self.config.hallway_top),
            (0, self.config.hallway_top)
        ])

        for i in range(num_rooms):
            if i == 0 or i == num_rooms - 1:
                w = random.uniform(self.config.big_min_width, self.config.big_max_width)
                h = random.uniform(self.config.big_min_height, self.config.big_max_height)
            else:
                w = random.uniform(self.config.small_min_width, self.config.small_max_width)
                h = random.uniform(self.config.small_min_height, self.config.small_max_height)
            widths.append(w)
            heights.append(h)
        total_width = sum(widths)
        norm_factor = self.config.width / total_width
        widths = [w * norm_factor for w in widths]

        current_x = 0.0
        for i in range(num_rooms):
            w = round(widths[i], 1)
            h = round(heights[i], 1)
            if side == "top":
                y = self.config.hallway_top
                door_y = self.config.hallway_top
            else:
                y = self.config.hallway_bottom - h
                door_y = self.config.hallway_bottom
            if w > self.config.door_width:
                door_start = random.uniform(current_x + self.config.door_width, current_x + w - self.config.door_width)
                door_end = door_start + self.config.door_width
            else:
                door_start, door_end = current_x, current_x + w
            x = round(current_x, 1)
            y = round(y, 1)

            door_start = round(door_start, 1)
            door_end = round(door_end, 1)

            rooms.append([
                (x, y),
                (x + w, y),
                (x + w, y + h),
                (x, y + h)
            ])
            doors.append([
                (door_start, door_y - self.config.resolution),
                (door_end, door_y - self.config.resolution),
                (door_end, door_y + self.config.resolution),
                (door_start, door_y + self.config.resolution)
            ])
            current_x += w

        return rooms, doors
