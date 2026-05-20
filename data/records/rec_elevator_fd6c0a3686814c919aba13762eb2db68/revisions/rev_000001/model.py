from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _annular_tube(
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    segments: int = 64,
) -> MeshGeometry:
    """Closed cylindrical tube shell centered on Z, with its bottom at z=0."""
    geom = MeshGeometry()
    outer_bottom: list[int] = []
    outer_top: list[int] = []
    inner_bottom: list[int] = []
    inner_top: list[int] = []

    for i in range(segments):
        theta = 2.0 * pi * i / segments
        c = cos(theta)
        s = sin(theta)
        outer_bottom.append(geom.add_vertex(outer_radius * c, outer_radius * s, 0.0))
        outer_top.append(geom.add_vertex(outer_radius * c, outer_radius * s, height))
        inner_bottom.append(geom.add_vertex(inner_radius * c, inner_radius * s, 0.0))
        inner_top.append(geom.add_vertex(inner_radius * c, inner_radius * s, height))

    for i in range(segments):
        j = (i + 1) % segments
        # Outer glass face.
        geom.add_face(outer_bottom[i], outer_bottom[j], outer_top[j])
        geom.add_face(outer_bottom[i], outer_top[j], outer_top[i])
        # Inner glass face.
        geom.add_face(inner_bottom[i], inner_top[i], inner_top[j])
        geom.add_face(inner_bottom[i], inner_top[j], inner_bottom[j])
        # Top annular cap.
        geom.add_face(outer_top[i], outer_top[j], inner_top[j])
        geom.add_face(outer_top[i], inner_top[j], inner_top[i])
        # Bottom annular cap.
        geom.add_face(outer_bottom[i], inner_bottom[i], inner_bottom[j])
        geom.add_face(outer_bottom[i], inner_bottom[j], outer_bottom[j])

    return geom


def _annular_sector(
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    start_deg: float,
    end_deg: float,
    segments: int = 44,
) -> MeshGeometry:
    """Closed extruded annular sector, useful for curved glass wall panels."""
    geom = MeshGeometry()
    outer_bottom: list[int] = []
    outer_top: list[int] = []
    inner_bottom: list[int] = []
    inner_top: list[int] = []

    start = start_deg * pi / 180.0
    end = end_deg * pi / 180.0
    for i in range(segments + 1):
        theta = start + (end - start) * i / segments
        c = cos(theta)
        s = sin(theta)
        outer_bottom.append(geom.add_vertex(outer_radius * c, outer_radius * s, 0.0))
        outer_top.append(geom.add_vertex(outer_radius * c, outer_radius * s, height))
        inner_bottom.append(geom.add_vertex(inner_radius * c, inner_radius * s, 0.0))
        inner_top.append(geom.add_vertex(inner_radius * c, inner_radius * s, height))

    for i in range(segments):
        j = i + 1
        geom.add_face(outer_bottom[i], outer_bottom[j], outer_top[j])
        geom.add_face(outer_bottom[i], outer_top[j], outer_top[i])
        geom.add_face(inner_bottom[i], inner_top[i], inner_top[j])
        geom.add_face(inner_bottom[i], inner_top[j], inner_bottom[j])
        geom.add_face(outer_top[i], outer_top[j], inner_top[j])
        geom.add_face(outer_top[i], inner_top[j], inner_top[i])
        geom.add_face(outer_bottom[i], inner_bottom[i], inner_bottom[j])
        geom.add_face(outer_bottom[i], inner_bottom[j], outer_bottom[j])

    # Radial end caps make the sector a true solid rather than a loose surface.
    for i in (0, segments):
        geom.add_face(outer_bottom[i], outer_top[i], inner_top[i])
        geom.add_face(outer_bottom[i], inner_top[i], inner_bottom[i])

    return geom


def _add_door_panel(part, *, material_glass, material_metal) -> None:
    part.visual(
        Box((0.34, 0.018, 1.48)),
        origin=Origin(xyz=(0.0, 0.0, 0.82)),
        material=material_glass,
        name="door_glass",
    )
    part.visual(
        Box((0.36, 0.030, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=material_metal,
        name="bottom_shoe",
    )
    part.visual(
        Box((0.36, 0.030, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 1.6175)),
        material=material_metal,
        name="top_hanger",
    )
    part.visual(
        Box((0.026, 0.028, 1.60)),
        origin=Origin(xyz=(-0.18, 0.0, 0.82)),
        material=material_metal,
        name="edge_stile_0",
    )
    part.visual(
        Box((0.026, 0.028, 1.60)),
        origin=Origin(xyz=(0.18, 0.0, 0.82)),
        material=material_metal,
        name="edge_stile_1",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="glass_observation_tower_elevator")

    glass = model.material("pale_blue_glass", rgba=(0.62, 0.88, 1.0, 0.26))
    car_glass = model.material("car_curved_glass", rgba=(0.70, 0.94, 1.0, 0.34))
    dark_metal = model.material("brushed_dark_metal", rgba=(0.08, 0.09, 0.10, 1.0))
    floor_metal = model.material("ribbed_steel", rgba=(0.35, 0.37, 0.38, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))

    column = model.part("column")
    column.visual(
        Cylinder(radius=1.06, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=dark_metal,
        name="round_plinth",
    )
    column.visual(
        mesh_from_geometry(
            _annular_tube(outer_radius=0.95, inner_radius=0.925, height=4.16),
            "outer_glass_tube",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=glass,
        name="outer_glass_tube",
    )
    column.visual(
        mesh_from_geometry(
            _annular_tube(outer_radius=0.97, inner_radius=0.91, height=0.08, segments=64),
            "base_clamp_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=dark_metal,
        name="base_clamp_ring",
    )
    column.visual(
        mesh_from_geometry(
            _annular_tube(outer_radius=0.97, inner_radius=0.91, height=0.08, segments=64),
            "top_clamp_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, 4.28)),
        material=dark_metal,
        name="top_clamp_ring",
    )
    column.visual(
        Cylinder(radius=1.00, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 4.37)),
        material=dark_metal,
        name="round_head_cap",
    )

    for idx, angle in enumerate((0, 60, 120, 180, 240, 300)):
        theta = angle * pi / 180.0
        column.visual(
            Cylinder(radius=0.018, length=4.26),
            origin=Origin(xyz=(0.95 * cos(theta), 0.95 * sin(theta), 2.25)),
            material=dark_metal,
            name=f"vertical_mullion_{idx}",
        )

    for idx, x in enumerate((-0.28, 0.28)):
        column.visual(
            Box((0.055, 0.035, 4.14)),
            origin=Origin(xyz=(x, 0.675, 2.19)),
            material=dark_metal,
            name=f"guide_rail_{idx}",
        )
    column.visual(
        Box((0.66, 0.030, 0.045)),
        origin=Origin(xyz=(0.0, 0.675, 0.16)),
        material=dark_metal,
        name="lower_rail_tie",
    )
    column.visual(
        Box((0.66, 0.030, 0.045)),
        origin=Origin(xyz=(0.0, 0.675, 4.21)),
        material=dark_metal,
        name="upper_rail_tie",
    )

    car = model.part("car")
    car.visual(
        Cylinder(radius=0.58, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=floor_metal,
        name="round_floor",
    )
    car.visual(
        mesh_from_geometry(
            _annular_sector(
                outer_radius=0.58,
                inner_radius=0.545,
                height=1.82,
                start_deg=-40.0,
                end_deg=220.0,
            ),
            "curved_three_side_glass",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=car_glass,
        name="curved_three_side_glass",
    )
    car.visual(
        mesh_from_geometry(
            _annular_sector(
                outer_radius=0.60,
                inner_radius=0.52,
                height=0.08,
                start_deg=-40.0,
                end_deg=220.0,
            ),
            "lower_car_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=dark_metal,
        name="lower_car_ring",
    )
    car.visual(
        mesh_from_geometry(
            _annular_sector(
                outer_radius=0.60,
                inner_radius=0.52,
                height=0.08,
                start_deg=-40.0,
                end_deg=220.0,
            ),
            "upper_car_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, 1.84)),
        material=dark_metal,
        name="upper_car_ring",
    )
    for name, x, y in (
        ("front_post_0", 0.44, -0.37),
        ("front_post_1", -0.44, -0.37),
        ("rear_post", 0.0, 0.59),
    ):
        car.visual(
            Cylinder(radius=0.024, length=1.90),
            origin=Origin(xyz=(x, y, 0.99)),
            material=dark_metal,
            name=name,
        )
    car.visual(
        Box((0.92, 0.10, 0.06)),
        origin=Origin(xyz=(0.0, -0.58, 0.10)),
        material=dark_metal,
        name="bottom_door_track",
    )
    car.visual(
        Box((0.92, 0.10, 0.06)),
        origin=Origin(xyz=(0.0, -0.58, 1.90)),
        material=dark_metal,
        name="top_door_track",
    )
    for idx, x in enumerate((-0.44, 0.44)):
        car.visual(
            Box((0.075, 0.22, 0.06)),
            origin=Origin(xyz=(x, -0.48, 1.90)),
            material=dark_metal,
            name=f"top_track_return_{idx}",
        )
    car.visual(
        Box((0.62, 0.045, 0.25)),
        origin=Origin(xyz=(0.0, 0.61, 1.02)),
        material=dark_metal,
        name="rear_guide_carriage",
    )
    for idx, x in enumerate((-0.25, 0.25)):
        car.visual(
            Box((0.10, 0.030, 0.12)),
            origin=Origin(xyz=(x, 0.6425, 1.02)),
            material=rubber,
            name=f"guide_shoe_{idx}",
        )

    left_door = model.part("left_door")
    _add_door_panel(left_door, material_glass=car_glass, material_metal=dark_metal)

    right_door = model.part("right_door")
    _add_door_panel(right_door, material_glass=car_glass, material_metal=dark_metal)

    model.articulation(
        "column_to_car",
        ArticulationType.PRISMATIC,
        parent=column,
        child=car,
        origin=Origin(xyz=(0.0, 0.0, 0.25)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4000.0, velocity=0.8, lower=0.0, upper=2.05),
    )
    model.articulation(
        "car_to_left_door",
        ArticulationType.PRISMATIC,
        parent=car,
        child=left_door,
        origin=Origin(xyz=(-0.185, -0.555, 0.13)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.55, lower=0.0, upper=0.25),
    )
    model.articulation(
        "car_to_right_door",
        ArticulationType.PRISMATIC,
        parent=car,
        child=right_door,
        origin=Origin(xyz=(0.185, -0.605, 0.13)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.55, lower=0.0, upper=0.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    column = object_model.get_part("column")
    car = object_model.get_part("car")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    lift = object_model.get_articulation("column_to_car")
    left_slide = object_model.get_articulation("car_to_left_door")
    right_slide = object_model.get_articulation("car_to_right_door")

    ctx.expect_within(
        car,
        column,
        axes="xy",
        inner_elem="curved_three_side_glass",
        outer_elem="outer_glass_tube",
        margin=0.0,
        name="curved car stays inside round glass column",
    )
    ctx.expect_gap(
        car,
        column,
        axis="z",
        positive_elem="round_floor",
        negative_elem="round_plinth",
        min_gap=0.10,
        name="car floor clears tower plinth",
    )
    ctx.expect_gap(
        right_door,
        left_door,
        axis="x",
        positive_elem="door_glass",
        negative_elem="door_glass",
        min_gap=0.010,
        max_gap=0.040,
        name="closed bi-parting doors meet at a narrow center gap",
    )
    ctx.expect_overlap(
        left_door,
        car,
        axes="z",
        elem_a="door_glass",
        elem_b="curved_three_side_glass",
        min_overlap=1.35,
        name="left door covers the elevator entrance height",
    )
    ctx.expect_overlap(
        right_door,
        car,
        axes="z",
        elem_a="door_glass",
        elem_b="curved_three_side_glass",
        min_overlap=1.35,
        name="right door covers the elevator entrance height",
    )

    rest_car = ctx.part_world_position(car)
    rest_left = ctx.part_world_position(left_door)
    rest_right = ctx.part_world_position(right_door)
    with ctx.pose({lift: 2.05, left_slide: 0.25, right_slide: 0.25}):
        raised_car = ctx.part_world_position(car)
        open_left = ctx.part_world_position(left_door)
        open_right = ctx.part_world_position(right_door)
        ctx.expect_within(
            car,
            column,
            axes="xy",
            inner_elem="curved_three_side_glass",
            outer_elem="outer_glass_tube",
            margin=0.0,
            name="raised car remains inside glass cylinder",
        )
        ctx.expect_gap(
            right_door,
            left_door,
            axis="x",
            positive_elem="door_glass",
            negative_elem="door_glass",
            min_gap=0.48,
            name="doors create a wide entrance when slid apart",
        )

    ctx.check(
        "vertical prismatic joint raises the observation car",
        rest_car is not None and raised_car is not None and raised_car[2] > rest_car[2] + 2.0,
        details=f"rest={rest_car}, raised={raised_car}",
    )
    ctx.check(
        "left door slides horizontally away from center",
        rest_left is not None and open_left is not None and open_left[0] < rest_left[0] - 0.22,
        details=f"rest={rest_left}, open={open_left}",
    )
    ctx.check(
        "right door slides horizontally away from center",
        rest_right is not None and open_right is not None and open_right[0] > rest_right[0] + 0.22,
        details=f"rest={rest_right}, open={open_right}",
    )

    return ctx.report()


object_model = build_object_model()
