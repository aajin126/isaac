import io
import itertools
import os

import attrs
import PIL.Image
import PIL.ImageDraw
import shapely
import shapely.affinity
import yaml

import arena_simulation_setup.world

Point = tuple[float, float]
Line = tuple[Point, Point]
Polygon = list[Point]


@attrs.define
class GeneratedWorld:
    rooms: list[Polygon]  # m
    doors: list[Polygon]  # m
    width: float  # m
    height: float  # m
    resolution: float  # m/px

    padding: int = 50  # px

    def global_tf(self, geom):
        padding_world = self.padding * self.resolution
        return shapely.set_precision(shapely.affinity.translate(geom, padding_world, padding_world), 0.01)

    def to_zones_yaml(self) -> str:
        return yaml.safe_dump([
            {'polygon': [list(self.global_tf(shapely.Polygon(polygon)).exterior.coords)]}
            for polygon
            in self.rooms
        ])

    def to_walls(self, connect: tuple[float, float] | None = None) -> shapely.MultiLineString:
        doors = shapely.make_valid(shapely.MultiPolygon([shapely.Polygon(door) for door in self.doors]))

        all_walls: list[shapely.LineString] = []
        for room in self.rooms:
            walls = shapely.LineString(shapely.Polygon(room).exterior.coords)
            reduced = walls.difference(doors)

            if isinstance(reduced, shapely.LineString):
                reduced = shapely.MultiLineString([reduced])

            if isinstance(reduced, shapely.MultiLineString):
                for geom in reduced.geoms:
                    pts = list(geom.coords)
                    for i in range(1, len(pts)):
                        start = pts[i - 1]
                        end = pts[i]
                        all_walls.append(
                            shapely.LineString(
                                (
                                    (start[0], start[1]),
                                    (end[0], end[1])
                                )
                            )
                        )

        if connect is not None:
            all_pts = [pt for wall in all_walls for pt in wall]
            for pt_a, pt_b in itertools.combinations(all_pts, 2):
                dist = pt_a.distance(pt_b)
                if dist > connect[0] and dist < connect[1]:
                    # todo check if wall already exists
                    start = pt_a.coords[0]
                    end = pt_b.coords[0]
                    all_walls.append(
                        shapely.LineString(
                            (
                                (start[0], start[1]),
                                (end[0], end[1])
                            )
                        )
                    )

        world_padding = self.padding * self.resolution
        inner_width = self.width + world_padding
        inner_height = self.height + world_padding

        return self.global_tf(shapely.MultiLineString(all_walls)).union(
            shapely.MultiLineString([
                shapely.LineString(((world_padding, world_padding), (inner_width, world_padding))),
                shapely.LineString(((world_padding, inner_height), (inner_width, inner_height))),
                shapely.LineString(((world_padding, world_padding), (world_padding, inner_height))),
                shapely.LineString(((inner_width, world_padding), (inner_width, inner_height)))
            ])
        )

    def to_walls_yaml(self, walls: shapely.MultiLineString) -> str:
        return yaml.safe_dump({
            'walls': [
                list(wall.coords)
                for wall
                in walls.geoms
            ]
        })

    def to_map_yaml(self) -> str:
        return yaml.safe_dump({
            'free_thresh': 0.196,
            'image': 'map.png',
            'negate': 0,
            'occupied_thresh': 0.65,
            'origin': [0, 0, 0],
            'resolution': self.resolution,
        })

    def to_map_png(self) -> bytes:
        img = PIL.Image.new(
            'RGB',
            (
                int(self.width / self.resolution) + 2 * self.padding,
                int(self.height / self.resolution) + 2 * self.padding),
            color='black'
        )

        scaling_factor = 1 / self.resolution

        def tf(shape):
            shape = shapely.affinity.scale(shape, scaling_factor, -scaling_factor, origin=(0, 0))
            shape = shapely.affinity.translate(shape, 0, self.height * scaling_factor)
            shape = shapely.affinity.translate(shape, self.padding, self.padding)
            return shape

        draw = PIL.ImageDraw.Draw(img)
        for room in self.rooms:
            poly = tf(shapely.Polygon(room))
            draw.polygon(list(poly.exterior.coords), fill='white', outline='black')
        for door in self.doors:
            poly = tf(shapely.Polygon(door))
            draw.polygon(list(poly.exterior.coords), fill='white')

        img_bytes = io.BytesIO()
        img.save(img_bytes, format='PNG')
        img_bytes.seek(0)
        return img_bytes.getvalue()

    def save_to(self, world_name: str):
        world = arena_simulation_setup.world.World(world_name)
        os.makedirs(world.map.path, exist_ok=True)

        with open(world.map.zones, 'w') as f:
            f.write(self.to_zones_yaml())
        with open(world.map.walls, 'w') as f:
            f.write(self.to_walls_yaml(self.to_walls()))
        with open(world.map.map_yaml, 'w') as f:
            f.write(self.to_map_yaml())
        with open(os.path.join(world.map.path, 'map.png'), 'wb') as f:
            f.write(self.to_map_png())

        os.makedirs(world.scenario.base_dir(), exist_ok=True)
        with open(world.scenario('default.json').path, 'w') as f:
            f.write('{}')
