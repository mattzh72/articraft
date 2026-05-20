from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Mesh,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


SPINDLE_XS = (-0.28, 0.0, 0.28)
HEAD_REST_Z = 1.37
TABLE_JOINT_Z = 0.50
TABLE_TOP_Z = TABLE_JOINT_Z + 0.08 + 0.03
FEED_TRAVEL = 0.08


def bearing_sleeve_mesh(name: str) -> Mesh:
    """A hollow cast boss for one spindle bearing."""
    geom = LatheGeometry.from_shell_profiles(
        outer_profile=[(0.074, -0.060), (0.074, 0.060)],
        inner_profile=[(0.034, -0.060), (0.034, 0.060)],
        segments=40,
    )
    return mesh_from_geometry(geom, name)


def add_rotated_cylinder(part, *, radius, length, axis: str, origin, material, name):
    if axis == "x":
        rpy = (0.0, math.pi / 2.0, 0.0)
    elif axis == "y":
        rpy = (math.pi / 2.0, 0.0, 0.0)
    else:
        rpy = (0.0, 0.0, 0.0)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=origin, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="multi_spindle_gang_drill_press")

    cast = model.material("dark_green_cast_iron", rgba=(0.05, 0.11, 0.10, 1.0))
    worn_cast = model.material("worn_cast_iron", rgba=(0.10, 0.13, 0.13, 1.0))
    machined = model.material("machined_steel", rgba=(0.58, 0.58, 0.55, 1.0))
    dark_steel = model.material("dark_oiled_steel", rgba=(0.03, 0.035, 0.04, 1.0))
    brass = model.material("brass_nameplate", rgba=(0.75, 0.55, 0.22, 1.0))
    slot_black = model.material("blackened_t_slots", rgba=(0.01, 0.012, 0.014, 1.0))

    frame = model.part("frame")
    frame.visual(Box((1.18, 0.72, 0.10)), origin=Origin(xyz=(0.0, -0.05, 0.05)), material=cast, name="floor_base")
    frame.visual(Box((0.86, 0.22, 1.74)), origin=Origin(xyz=(0.0, 0.18, 0.92)), material=cast, name="wide_column")
    frame.visual(Box((0.95, 0.28, 0.08)), origin=Origin(xyz=(0.0, 0.12, 1.80)), material=cast, name="column_cap")
    frame.visual(Box((0.48, 0.08, 0.10)), origin=Origin(xyz=(0.0, -0.24, 0.14)), material=cast, name="front_foot")

    # A visible vertical rack on the column front face, with individually
    # raised teeth for the pinion that drives the head feed.
    frame.visual(Box((0.11, 0.028, 0.78)), origin=Origin(xyz=(-0.39, 0.105, 1.27)), material=dark_steel, name="feed_rack_rail")
    for i in range(24):
        z = 0.91 + i * 0.031
        frame.visual(
            Box((0.115, 0.032, 0.010)),
            origin=Origin(xyz=(-0.39, 0.104, z), rpy=(0.0, 0.0, 0.0)),
            material=machined,
            name=f"rack_tooth_{i}",
        )

    # Table trunnion support below the head: arm from the column, cross yoke,
    # and two cheeks that touch the table's axle ends.
    frame.visual(Box((0.25, 0.58, 0.09)), origin=Origin(xyz=(0.0, -0.18, TABLE_JOINT_Z - 0.12)), material=cast, name="table_support_arm")
    frame.visual(Box((1.10, 0.08, 0.08)), origin=Origin(xyz=(0.0, -0.38, TABLE_JOINT_Z - 0.10)), material=cast, name="tilt_cross_yoke")
    frame.visual(Box((0.06, 0.12, 0.20)), origin=Origin(xyz=(-0.52, -0.38, TABLE_JOINT_Z)), material=cast, name="tilt_bracket_0")
    frame.visual(Box((0.06, 0.12, 0.20)), origin=Origin(xyz=(0.52, -0.38, TABLE_JOINT_Z)), material=cast, name="tilt_bracket_1")
    for x in (-0.46, 0.46):
        for y in (-0.31, 0.19):
            frame.visual(Cylinder(radius=0.025, length=0.012), origin=Origin(xyz=(x, y, 0.106)), material=machined, name=f"base_bolt_{x}_{y}")

    head = model.part("gang_head")
    head.visual(Box((0.96, 0.32, 0.28)), origin=Origin(), material=worn_cast, name="head_casting")
    head.visual(Box((0.78, 0.25, 0.14)), origin=Origin(xyz=(0.0, -0.02, 0.21)), material=worn_cast, name="drive_cover")
    head.visual(Box((0.30, 0.04, 0.20)), origin=Origin(xyz=(0.22, 0.14, -0.01)), material=worn_cast, name="rear_slide_shoe")
    head.visual(Box((0.24, 0.04, 0.18)), origin=Origin(xyz=(-0.18, 0.14, -0.01)), material=worn_cast, name="aux_slide_shoe")
    head.visual(Box((0.26, 0.03, 0.06)), origin=Origin(xyz=(0.0, -0.174, 0.035)), material=brass, name="front_nameplate")
    for i, x in enumerate(SPINDLE_XS):
        head.visual(
            bearing_sleeve_mesh(f"bearing_sleeve_{i}"),
            origin=Origin(xyz=(x, -0.09, -0.20)),
            material=worn_cast,
            name=f"bearing_{i}",
        )
        head.visual(
            Box((0.050, 0.08, 0.05)),
            origin=Origin(xyz=(x + 0.075, -0.09, -0.145)),
            material=worn_cast,
            name=f"bearing_web_{i}",
        )
    add_rotated_cylinder(
        head,
        radius=0.030,
        length=0.030,
        axis="x",
        origin=(-0.495, 0.19, 0.025),
        material=machined,
        name="pinion_boss",
    )

    model.articulation(
        "head_feed",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=head,
        origin=Origin(xyz=(0.0, -0.09, HEAD_REST_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=550.0, velocity=0.10, lower=0.0, upper=FEED_TRAVEL),
    )

    for i, x in enumerate(SPINDLE_XS):
        spindle = model.part(f"spindle_{i}")
        spindle.visual(Cylinder(radius=0.034, length=0.120), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=machined, name="bearing_race")
        spindle.visual(Cylinder(radius=0.017, length=0.170), origin=Origin(xyz=(0.0, 0.0, -0.035)), material=machined, name="axle")
        spindle.visual(Cylinder(radius=0.026, length=0.050), origin=Origin(xyz=(0.0, 0.0, -0.145)), material=dark_steel, name="chuck_collar")
        spindle.visual(Cylinder(radius=0.041, length=0.090), origin=Origin(xyz=(0.0, 0.0, -0.215)), material=dark_steel, name="chuck_body")
        spindle.visual(Cylinder(radius=0.026, length=0.040), origin=Origin(xyz=(0.0, 0.0, -0.280)), material=dark_steel, name="chuck_nose")
        spindle.visual(Cylinder(radius=0.0075, length=0.105), origin=Origin(xyz=(0.0, 0.0, -0.352)), material=machined, name="drill_bit")
        spindle.visual(Cylinder(radius=0.003, length=0.020), origin=Origin(xyz=(0.0, 0.0, -0.414)), material=machined, name="drill_point")
        model.articulation(
            f"spindle_axle_{i}",
            ArticulationType.CONTINUOUS,
            parent=head,
            child=spindle,
            origin=Origin(xyz=(x, -0.09, -0.20)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=95.0),
        )

    pinion = model.part("feed_pinion")
    add_rotated_cylinder(
        pinion,
        radius=0.050,
        length=0.034,
        axis="x",
        origin=(-0.032, 0.0, 0.0),
        material=machined,
        name="pinion_web",
    )
    add_rotated_cylinder(
        pinion,
        radius=0.021,
        length=0.030,
        axis="x",
        origin=(-0.014, 0.0, 0.0),
        material=dark_steel,
        name="pinion_hub",
    )
    for i in range(16):
        theta = i * (2.0 * math.pi / 16.0)
        pinion.visual(
            Box((0.038, 0.010, 0.018)),
            origin=Origin(
                xyz=(-0.032, 0.055 * math.cos(theta), 0.055 * math.sin(theta)),
                rpy=(theta, 0.0, 0.0),
            ),
            material=machined,
            name=f"pinion_tooth_{i}",
        )
    model.articulation(
        "feed_pinion_axle",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=pinion,
        origin=Origin(xyz=(-0.510, 0.19, 0.025)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=4.0),
    )

    table = model.part("table")
    table.visual(Cylinder(radius=0.44, length=0.060), origin=Origin(xyz=(0.0, 0.0, 0.080)), material=worn_cast, name="round_table")
    table.visual(Cylinder(radius=0.090, length=0.120), origin=Origin(xyz=(0.0, 0.0, 0.035)), material=worn_cast, name="table_neck")
    add_rotated_cylinder(table, radius=0.052, length=0.980, axis="x", origin=(0.0, 0.0, 0.0), material=machined, name="trunnion")
    for j, y in enumerate((-0.18, 0.0, 0.18)):
        table.visual(Box((0.70, 0.030, 0.008)), origin=Origin(xyz=(0.0, y, 0.113)), material=slot_black, name=f"t_slot_{j}")
    table.visual(Cylinder(radius=0.035, length=0.010), origin=Origin(xyz=(0.0, 0.0, 0.114)), material=slot_black, name="center_bore")
    model.articulation(
        "table_tilt",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=table,
        origin=Origin(xyz=(0.0, -0.38, TABLE_JOINT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.55, lower=-0.45, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    head = object_model.get_part("gang_head")
    table = object_model.get_part("table")
    feed = object_model.get_articulation("head_feed")
    tilt = object_model.get_articulation("table_tilt")

    ctx.check(
        "three spindle axles are present",
        all(object_model.get_articulation(f"spindle_axle_{i}") for i in range(3)),
        details="The gang head should carry three rotary spindle joints.",
    )

    ctx.expect_gap(head, table, axis="z", min_gap=0.35, name="gang head stands above the tilting table")
    ctx.expect_overlap(head, table, axes="x", min_overlap=0.70, name="wide gang head spans the table width")
    ctx.expect_contact(head, frame, elem_a="rear_slide_shoe", elem_b="wide_column", contact_tol=0.002, name="head slide shoe bears on column face")
    ctx.expect_contact(table, frame, elem_a="trunnion", elem_b="tilt_bracket_0", contact_tol=0.003, name="trunnion touches one bracket cheek")
    ctx.expect_contact(table, frame, elem_a="trunnion", elem_b="tilt_bracket_1", contact_tol=0.003, name="trunnion touches other bracket cheek")
    for i in range(3):
        spindle = object_model.get_part(f"spindle_{i}")
        ctx.allow_overlap(
            head,
            spindle,
            elem_a=f"bearing_{i}",
            elem_b="bearing_race",
            reason="The spindle bearing race is intentionally captured inside the gang-head bearing sleeve proxy.",
        )
        ctx.expect_within(
            spindle,
            head,
            axes="xy",
            inner_elem="bearing_race",
            outer_elem=f"bearing_{i}",
            margin=0.001,
            name=f"spindle {i} bearing race is centered in its sleeve",
        )
        ctx.expect_overlap(
            spindle,
            head,
            axes="z",
            elem_a="bearing_race",
            elem_b=f"bearing_{i}",
            min_overlap=0.10,
            name=f"spindle {i} bearing race remains axially captured",
        )

    spindle_origins = [ctx.part_world_position(object_model.get_part(f"spindle_{i}")) for i in range(3)]
    equal_spacing = (
        all(p is not None for p in spindle_origins)
        and abs((spindle_origins[1][0] - spindle_origins[0][0]) - (spindle_origins[2][0] - spindle_origins[1][0])) < 1e-6
    )
    ctx.check("three spindles are equally spaced", equal_spacing, details=f"origins={spindle_origins}")

    rest_head = ctx.part_world_position(head)
    rest_table_aabb = ctx.part_world_aabb(table)
    with ctx.pose({feed: FEED_TRAVEL}):
        fed_head = ctx.part_world_position(head)
        ctx.expect_gap(object_model.get_part("spindle_1"), table, axis="z", min_gap=0.015, name="fed center drill clears the table top")
    ctx.check(
        "rack feed advances the gang head downward",
        rest_head is not None and fed_head is not None and fed_head[2] < rest_head[2] - 0.06,
        details=f"rest={rest_head}, fed={fed_head}",
    )

    with ctx.pose({tilt: 0.40}):
        tilted_table_aabb = ctx.part_world_aabb(table)
    ctx.check(
        "round table tilts on its bracket",
        rest_table_aabb is not None
        and tilted_table_aabb is not None
        and (tilted_table_aabb[1][2] - tilted_table_aabb[0][2]) > (rest_table_aabb[1][2] - rest_table_aabb[0][2]) + 0.10,
        details=f"rest_aabb={rest_table_aabb}, tilted_aabb={tilted_table_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
