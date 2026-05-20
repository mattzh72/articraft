from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _annular_flange(
    outer_diameter: float,
    inner_diameter: float,
    thickness: float,
    *,
    bolt_circle: float | None = None,
    bolt_diameter: float = 0.0,
    bolt_count: int = 0,
    boss_diameter: float | None = None,
    boss_height: float = 0.0,
):
    """CadQuery annular flange authored in meters with its bottom on z=0."""

    flange = (
        cq.Workplane("XY")
        .circle(outer_diameter / 2.0)
        .circle(inner_diameter / 2.0)
        .extrude(thickness)
    )
    if bolt_circle and bolt_count and bolt_diameter > 0.0:
        points = [
            (
                math.cos(2.0 * math.pi * i / bolt_count) * bolt_circle / 2.0,
                math.sin(2.0 * math.pi * i / bolt_count) * bolt_circle / 2.0,
            )
            for i in range(bolt_count)
        ]
        flange = flange.faces(">Z").workplane().pushPoints(points).hole(bolt_diameter)
    if boss_diameter and boss_height > 0.0:
        boss = (
            cq.Workplane("XY")
            .workplane(offset=thickness)
            .circle(boss_diameter / 2.0)
            .circle(inner_diameter / 2.0)
            .extrude(boss_height)
        )
        flange = flange.union(boss)
    return flange


def _rounded_cover(width: float, height: float, thickness: float, corner: float = 0.012):
    """Flat removable cover plate, front face normal along local +Z."""

    return (
        cq.Workplane("XY")
        .rect(width - 2.0 * corner, height)
        .rect(width, height - 2.0 * corner)
        .circle(corner)
        .extrude(thickness)
    )


def _mat(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]) -> Material:
    return model.material(name, rgba=rgba)


def _bolt_circle(
    part,
    *,
    radius: float,
    z: float,
    count: int,
    bolt_radius: float,
    height: float,
    material,
    name_prefix: str,
):
    for i in range(count):
        angle = 2.0 * math.pi * i / count
        part.visual(
            Cylinder(radius=bolt_radius, length=height),
            origin=Origin(
                xyz=(math.cos(angle) * radius, math.sin(angle) * radius, z)
            ),
            material=material,
            name=f"{name_prefix}_{i}",
        )


def _face_screws_y(
    part,
    *,
    xs: tuple[float, ...],
    zs: tuple[float, ...],
    y: float,
    radius: float,
    length: float,
    material,
    name_prefix: str,
):
    idx = 0
    for x in xs:
        for z in zs:
            part.visual(
                Cylinder(radius=radius, length=length),
                origin=Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=material,
                name=f"{name_prefix}_{idx}",
            )
            idx += 1


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotary_base_vertical_slide_study")

    dark_steel = _mat(model, "dark_phosphate_steel", (0.06, 0.065, 0.07, 1.0))
    blued_steel = _mat(model, "blued_machined_steel", (0.11, 0.13, 0.145, 1.0))
    ground_steel = _mat(model, "ground_linear_steel", (0.72, 0.73, 0.70, 1.0))
    black_oxide = _mat(model, "black_oxide_fasteners", (0.015, 0.015, 0.014, 1.0))
    cover_gray = _mat(model, "removable_cover_gray", (0.20, 0.215, 0.225, 1.0))
    stop_yellow = _mat(model, "safety_stop_collars", (0.78, 0.56, 0.12, 1.0))
    acetal = _mat(model, "black_acetal_bumpers", (0.02, 0.018, 0.016, 1.0))

    base = model.part("base_frame")
    base.visual(
        Box((0.72, 0.56, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_steel,
        name="floor_plate",
    )
    base.visual(
        Box((0.68, 0.026, 0.100)),
        origin=Origin(xyz=(0.0, -0.280, 0.100)),
        material=blued_steel,
        name="front_shear_web",
    )
    base.visual(
        Box((0.68, 0.026, 0.100)),
        origin=Origin(xyz=(0.0, 0.280, 0.100)),
        material=blued_steel,
        name="rear_shear_web",
    )
    base.visual(
        Box((0.026, 0.50, 0.095)),
        origin=Origin(xyz=(-0.335, 0.0, 0.097)),
        material=blued_steel,
        name="side_web_0",
    )
    base.visual(
        Box((0.026, 0.50, 0.095)),
        origin=Origin(xyz=(0.335, 0.0, 0.097)),
        material=blued_steel,
        name="side_web_1",
    )
    for i, x in enumerate((-0.230, 0.0, 0.230)):
        base.visual(
            Box((0.035, 0.44, 0.035)),
            origin=Origin(xyz=(x, 0.0, 0.067)),
            material=dark_steel,
            name=f"base_cross_rib_{i}",
        )
    for i, (x, y) in enumerate(
        ((-0.285, -0.205), (0.285, -0.205), (-0.285, 0.205), (0.285, 0.205))
    ):
        base.visual(
            Box((0.105, 0.075, 0.030)),
            origin=Origin(xyz=(x, y, 0.015)),
            material=dark_steel,
            name=f"leveling_foot_{i}",
        )
        base.visual(
            Cylinder(radius=0.014, length=0.010),
            origin=Origin(xyz=(x, y, 0.052)),
            material=black_oxide,
            name=f"foot_jam_nut_{i}",
        )

    outer_ring = _annular_flange(
        0.400,
        0.190,
        0.030,
        bolt_circle=0.370,
        bolt_diameter=0.018,
        bolt_count=12,
    )
    base.visual(
        mesh_from_cadquery(outer_ring, "outer_bearing_ring", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=ground_steel,
        name="outer_bearing_ring",
    )
    _bolt_circle(
        base,
        radius=0.185,
        z=0.083,
        count=12,
        bolt_radius=0.010,
        height=0.006,
        material=black_oxide,
        name_prefix="bearing_cap_screw",
    )
    for i, x in enumerate((-0.175, 0.175)):
        base.visual(
            Box((0.062, 0.030, 0.090)),
            origin=Origin(xyz=(x, 0.225, 0.095)),
            material=stop_yellow,
            name=f"rotary_stop_block_{i}",
        )
        base.visual(
            Box((0.050, 0.014, 0.034)),
            origin=Origin(xyz=(x, 0.205, 0.123)),
            material=acetal,
            name=f"rotary_stop_pad_{i}",
        )

    yaw = model.part("yaw_table")
    yaw.visual(
        mesh_from_cadquery(
            _annular_flange(
                0.345,
                0.105,
                0.035,
                bolt_circle=0.265,
                bolt_diameter=0.014,
                bolt_count=10,
                boss_diameter=0.170,
                boss_height=0.045,
            ),
            "rotary_hub_flange",
            tolerance=0.0008,
        ),
        material=ground_steel,
        name="rotary_hub_flange",
    )
    _bolt_circle(
        yaw,
        radius=0.1325,
        z=0.038,
        count=10,
        bolt_radius=0.008,
        height=0.006,
        material=black_oxide,
        name_prefix="hub_socket_screw",
    )
    yaw.visual(
        Cylinder(radius=0.083, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=blued_steel,
        name="central_bearing_hub",
    )
    yaw.visual(
        Box((0.300, 0.235, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=dark_steel,
        name="column_plinth",
    )
    yaw.visual(
        Box((0.205, 0.160, 0.720)),
        origin=Origin(xyz=(0.0, 0.0, 0.470)),
        material=dark_steel,
        name="fabricated_column",
    )
    yaw.visual(
        Box((0.235, 0.026, 0.045)),
        origin=Origin(xyz=(0.0, -0.095, 0.132)),
        material=blued_steel,
        name="lower_rail_tie",
    )
    yaw.visual(
        Box((0.235, 0.026, 0.045)),
        origin=Origin(xyz=(0.0, -0.095, 0.808)),
        material=blued_steel,
        name="upper_rail_tie",
    )
    rail_names = ("linear_guide_rail_0", "linear_guide_rail_1")
    for i, x in enumerate((-0.076, 0.076)):
        yaw.visual(
            Box((0.024, 0.022, 0.660)),
            origin=Origin(xyz=(x, -0.089, 0.470)),
            material=ground_steel,
            name=rail_names[i],
        )
        _face_screws_y(
            yaw,
            xs=(x,),
            zs=(0.185, 0.310, 0.435, 0.560, 0.685, 0.775),
            y=-0.103,
            radius=0.0048,
            length=0.006,
            material=black_oxide,
            name_prefix=f"rail_screw_{i}",
        )
        yaw.visual(
            Box((0.050, 0.034, 0.026)),
            origin=Origin(xyz=(x, -0.092, 0.145)),
            material=stop_yellow,
            name=f"lower_hard_stop_collar_{i}",
        )
        yaw.visual(
            Box((0.050, 0.034, 0.026)),
            origin=Origin(xyz=(x, -0.092, 0.795)),
            material=stop_yellow,
            name=f"upper_hard_stop_collar_{i}",
        )

    yaw.visual(
        Cylinder(radius=0.011, length=0.730),
        origin=Origin(xyz=(0.0, -0.137, 0.470)),
        material=ground_steel,
        name="exposed_lift_screw",
    )
    for prefix, z in (("lower", 0.128), ("upper", 0.812)):
        yaw.visual(
            Box((0.096, 0.045, 0.047)),
            origin=Origin(xyz=(0.0, -0.130, z)),
            material=blued_steel,
            name=f"{prefix}_screw_bearing",
        )
        yaw.visual(
            Box((0.110, 0.048, 0.043)),
            origin=Origin(xyz=(0.0, -0.096, z)),
            material=dark_steel,
            name=f"{prefix}_screw_bracket",
        )
    yaw.visual(
        Box((0.070, 0.070, 0.040)),
        origin=Origin(xyz=(0.0, 0.145, 0.105)),
        material=dark_steel,
        name="stop_dog_mount",
    )
    yaw.visual(
        Box((0.040, 0.050, 0.080)),
        origin=Origin(xyz=(0.0, 0.205, 0.120)),
        material=stop_yellow,
        name="rotary_stop_dog",
    )

    lift = model.part("lift_carriage")
    lift.visual(
        Box((0.310, 0.045, 0.220)),
        origin=Origin(xyz=(0.0, -0.055, 0.100)),
        material=blued_steel,
        name="machined_carriage_plate",
    )
    lift.visual(
        Box((0.230, 0.018, 0.165)),
        origin=Origin(xyz=(0.0, -0.086, 0.100)),
        material=dark_steel,
        name="front_stiffener_plate",
    )
    # U-shaped bearing shoes wrap around the fixed guide rails with a real open gap.
    shoe_specs = (
        (-0.076, 0.020, "guide_shoe_bridge_0"),
        (-0.076, 0.155, "guide_shoe_bridge_1"),
        (0.076, 0.020, "guide_shoe_bridge_2"),
        (0.076, 0.155, "guide_shoe_bridge_3"),
    )
    for shoe_index, (rail_x, zc, bridge_name) in enumerate(shoe_specs):
            lift.visual(
                Box((0.012, 0.031, 0.072)),
                origin=Origin(xyz=(rail_x - 0.018, 0.035, zc)),
                material=ground_steel,
                name=f"guide_shoe_cheek_{shoe_index}_0",
            )
            lift.visual(
                Box((0.012, 0.031, 0.072)),
                origin=Origin(xyz=(rail_x + 0.018, 0.035, zc)),
                material=ground_steel,
                name=f"guide_shoe_cheek_{shoe_index}_1",
            )
            lift.visual(
                Box((0.062, 0.012, 0.072)),
                origin=Origin(xyz=(rail_x, 0.019, zc)),
                material=ground_steel,
                name=bridge_name,
            )
            lift.visual(
                Box((0.094, 0.070, 0.024)),
                origin=Origin(xyz=(rail_x, -0.016, zc)),
                material=dark_steel,
                name=f"guide_shoe_rib_{shoe_index}",
            )
    for x in (-0.035, 0.035):
        lift.visual(
            Box((0.026, 0.040, 0.105)),
            origin=Origin(xyz=(x, 0.020, 0.090)),
            material=ground_steel,
            name=f"split_lift_nut_half_{0 if x < 0 else 1}",
        )
    for i, x in enumerate((-0.035, 0.035)):
        lift.visual(
            Box((0.026, 0.054, 0.030)),
            origin=Origin(xyz=(x, -0.013, 0.090)),
            material=dark_steel,
            name=f"lift_nut_tie_{i}",
        )
    lift.visual(
        Box((0.155, 0.020, 0.145)),
        origin=Origin(xyz=(0.0, -0.083, 0.100)),
        material=ground_steel,
        name="removable_front_adapter",
    )
    _face_screws_y(
        lift,
        xs=(-0.065, 0.065),
        zs=(0.025, 0.175),
        y=-0.083,
        radius=0.006,
        length=0.006,
        material=black_oxide,
        name_prefix="carriage_socket_screw",
    )

    base_cover = model.part("base_cover")
    base_cover.visual(
        mesh_from_cadquery(_rounded_cover(0.300, 0.075, 0.010), "base_access_cover"),
        origin=Origin(xyz=(0.0, -0.293, 0.100), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cover_gray,
        name="base_access_cover",
    )
    _face_screws_y(
        base_cover,
        xs=(-0.120, 0.120),
        zs=(0.075, 0.125),
        y=-0.306,
        radius=0.007,
        length=0.006,
        material=black_oxide,
        name_prefix="base_cover_screw",
    )
    base_cover.visual(
        Box((0.060, 0.012, 0.015)),
        origin=Origin(xyz=(0.0, -0.309, 0.100)),
        material=black_oxide,
        name="base_cover_pull_tab",
    )

    column_cover = model.part("column_cover")
    column_cover.visual(
        mesh_from_cadquery(_rounded_cover(0.132, 0.390, 0.010), "column_access_cover"),
        origin=Origin(xyz=(0.0, 0.080, 0.495), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=cover_gray,
        name="column_access_cover",
    )
    _face_screws_y(
        column_cover,
        xs=(-0.048, 0.048),
        zs=(0.330, 0.660),
        y=0.093,
        radius=0.006,
        length=0.006,
        material=black_oxide,
        name_prefix="column_cover_screw",
    )

    carriage_cover = model.part("carriage_cover")
    carriage_cover.visual(
        mesh_from_cadquery(_rounded_cover(0.170, 0.100, 0.008), "carriage_access_cover"),
        origin=Origin(xyz=(0.0, -0.095, 0.100), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cover_gray,
        name="carriage_access_cover",
    )
    _face_screws_y(
        carriage_cover,
        xs=(-0.065, 0.065),
        zs=(0.065, 0.135),
        y=-0.1055,
        radius=0.0055,
        length=0.005,
        material=black_oxide,
        name_prefix="carriage_cover_screw",
    )

    model.articulation(
        "base_to_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=yaw,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.75, lower=-1.30, upper=1.30),
    )
    model.articulation(
        "yaw_to_lift",
        ArticulationType.PRISMATIC,
        parent=yaw,
        child=lift,
        origin=Origin(xyz=(0.0, -0.137, 0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=650.0, velocity=0.18, lower=0.0, upper=0.340),
    )
    model.articulation(
        "base_to_cover",
        ArticulationType.FIXED,
        parent=base,
        child=base_cover,
        origin=Origin(),
    )
    model.articulation(
        "yaw_to_column_cover",
        ArticulationType.FIXED,
        parent=yaw,
        child=column_cover,
        origin=Origin(),
    )
    model.articulation(
        "lift_to_cover",
        ArticulationType.FIXED,
        parent=lift,
        child=carriage_cover,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    yaw = object_model.get_part("yaw_table")
    lift = object_model.get_part("lift_carriage")
    base_cover = object_model.get_part("base_cover")
    column_cover = object_model.get_part("column_cover")
    carriage_cover = object_model.get_part("carriage_cover")
    yaw_joint = object_model.get_articulation("base_to_yaw")
    lift_joint = object_model.get_articulation("yaw_to_lift")

    ctx.expect_gap(
        yaw,
        base,
        axis="z",
        positive_elem="rotary_hub_flange",
        negative_elem="outer_bearing_ring",
        max_gap=0.0015,
        max_penetration=0.0,
        name="yaw flange is seated on fixed bearing ring",
    )
    ctx.expect_overlap(
        lift,
        yaw,
        axes="z",
        elem_a="guide_shoe_bridge_0",
        elem_b="linear_guide_rail_0",
        min_overlap=0.060,
        name="lower carriage shoe remains on rail at rest",
    )
    ctx.expect_overlap(
        lift,
        yaw,
        axes="z",
        elem_a="guide_shoe_bridge_3",
        elem_b="linear_guide_rail_1",
        min_overlap=0.050,
        name="upper carriage shoe remains on rail at rest",
    )
    ctx.expect_contact(
        base_cover,
        base,
        elem_a="base_access_cover",
        elem_b="front_shear_web",
        contact_tol=0.003,
        name="base access cover is mounted to front web",
    )
    ctx.expect_contact(
        column_cover,
        yaw,
        elem_a="column_access_cover",
        elem_b="fabricated_column",
        contact_tol=0.004,
        name="column access cover is mounted to column",
    )
    ctx.expect_contact(
        carriage_cover,
        lift,
        elem_a="carriage_access_cover",
        elem_b="front_stiffener_plate",
        contact_tol=0.003,
        name="carriage cover is mounted to stiffener plate",
    )

    rest_pos = ctx.part_world_position(lift)
    with ctx.pose({lift_joint: 0.340}):
        raised_pos = ctx.part_world_position(lift)
        ctx.expect_overlap(
            lift,
            yaw,
            axes="z",
            elem_a="guide_shoe_bridge_0",
            elem_b="linear_guide_rail_0",
            min_overlap=0.050,
            name="lower carriage shoe retains rail engagement when raised",
        )
    ctx.check(
        "lift joint raises carriage upward",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.30,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    rest_yaw = ctx.part_world_position(yaw)
    with ctx.pose({yaw_joint: 0.75}):
        rotated_lift = ctx.part_world_position(lift)
    ctx.check(
        "yaw joint keeps lift stage on rotary axis",
        rest_yaw is not None and rotated_lift is not None and abs(rotated_lift[2] - 0.260) < 0.005,
        details=f"yaw={rest_yaw}, lift_at_yaw={rotated_lift}",
    )

    return ctx.report()


object_model = build_object_model()
