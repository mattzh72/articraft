from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overbed_table")

    steel = model.material("steel", rgba=(0.67, 0.69, 0.72, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    tabletop = model.material("tabletop", rgba=(0.64, 0.60, 0.56, 1.0))
    black = model.material("black", rgba=(0.08, 0.08, 0.09, 1.0))
    lock_red = model.material("lock_red", rgba=(0.78, 0.18, 0.12, 1.0))

    base = model.part("base")
    rail_z = 0.070
    rail_h = 0.018
    corner_x = (0.110, -0.310)
    corner_y = (0.105, -0.105)
    caster_positions = (
        (corner_x[0], corner_y[0]),
        (corner_x[0], corner_y[1]),
        (corner_x[1], corner_y[0]),
        (corner_x[1], corner_y[1]),
    )

    base.visual(
        Box((0.460, 0.035, rail_h)),
        origin=Origin(xyz=(-0.100, 0.105, rail_z)),
        material=dark_steel,
        name="side_rail_0",
    )
    base.visual(
        Box((0.460, 0.035, rail_h)),
        origin=Origin(xyz=(-0.100, -0.105, rail_z)),
        material=dark_steel,
        name="side_rail_1",
    )
    base.visual(
        Box((0.035, 0.245, rail_h)),
        origin=Origin(xyz=(0.110, 0.0, rail_z)),
        material=dark_steel,
        name="front_rail",
    )
    base.visual(
        Box((0.035, 0.245, rail_h)),
        origin=Origin(xyz=(-0.310, 0.0, rail_z)),
        material=dark_steel,
        name="rear_rail",
    )
    base.visual(
        Box((0.250, 0.030, rail_h)),
        origin=Origin(xyz=(-0.175, 0.0, rail_z)),
        material=dark_steel,
        name="spine_rail",
    )
    base.visual(
        Box((0.060, 0.190, rail_h)),
        origin=Origin(xyz=(0.050, 0.0, rail_z)),
        material=dark_steel,
        name="column_foot_0",
    )
    base.visual(
        Box((0.060, 0.190, rail_h)),
        origin=Origin(xyz=(-0.050, 0.0, rail_z)),
        material=dark_steel,
        name="column_foot_1",
    )

    sleeve_bottom = rail_z + rail_h * 0.5 - 0.002
    sleeve_height = 0.300
    sleeve_outer_x = 0.050
    sleeve_outer_y = 0.036
    sleeve_wall = 0.005
    sleeve_inner_x = sleeve_outer_x - 2.0 * sleeve_wall
    sleeve_inner_y = sleeve_outer_y - 2.0 * sleeve_wall
    wall_height = sleeve_height
    wall_center_z = sleeve_bottom + wall_height * 0.5

    base.visual(
        Box((sleeve_outer_x, sleeve_wall, wall_height)),
        origin=Origin(xyz=(0.0, sleeve_outer_y * 0.5 - sleeve_wall * 0.5, wall_center_z)),
        material=steel,
        name="sleeve_front",
    )
    base.visual(
        Box((sleeve_outer_x, sleeve_wall, wall_height)),
        origin=Origin(xyz=(0.0, -sleeve_outer_y * 0.5 + sleeve_wall * 0.5, wall_center_z)),
        material=steel,
        name="sleeve_back",
    )
    base.visual(
        Box((sleeve_wall, sleeve_inner_y, wall_height)),
        origin=Origin(xyz=(sleeve_outer_x * 0.5 - sleeve_wall * 0.5, 0.0, wall_center_z)),
        material=steel,
        name="sleeve_right",
    )
    base.visual(
        Box((sleeve_wall, sleeve_inner_y, wall_height)),
        origin=Origin(xyz=(-sleeve_outer_x * 0.5 + sleeve_wall * 0.5, 0.0, wall_center_z)),
        material=steel,
        name="sleeve_left",
    )
    base.visual(
        Box((0.072, 0.006, 0.020)),
        origin=Origin(xyz=(0.0, 0.019, sleeve_bottom + 0.010)),
        material=steel,
        name="sleeve_collar_0",
    )
    base.visual(
        Box((0.072, 0.006, 0.020)),
        origin=Origin(xyz=(0.0, -0.019, sleeve_bottom + 0.010)),
        material=steel,
        name="sleeve_collar_1",
    )
    for index, (cx, cy) in enumerate(caster_positions):
        base.visual(
            Cylinder(radius=0.015, length=0.010),
            origin=Origin(xyz=(cx, cy, 0.057)),
            material=charcoal,
            name=f"caster_housing_{index}",
        )
        base.visual(
            Cylinder(radius=0.006, length=0.020),
            origin=Origin(xyz=(cx, cy, 0.065)),
            material=charcoal,
            name=f"caster_stem_{index}",
        )
        base.visual(
            Box((0.024, 0.028, 0.006)),
            origin=Origin(xyz=(cx, cy, 0.055)),
            material=charcoal,
            name=f"fork_bridge_{index}",
        )
        base.visual(
            Box((0.032, 0.004, 0.030)),
            origin=Origin(xyz=(cx, cy + 0.014, 0.037)),
            material=charcoal,
            name=f"fork_plate_a_{index}",
        )
        base.visual(
            Box((0.032, 0.004, 0.030)),
            origin=Origin(xyz=(cx, cy - 0.014, 0.037)),
            material=charcoal,
            name=f"fork_plate_b_{index}",
        )
        base.visual(
            Cylinder(radius=0.004, length=0.032),
            origin=Origin(xyz=(cx, cy, 0.025), rpy=(1.57079632679, 0.0, 0.0)),
            material=steel,
            name=f"axle_pin_{index}",
        )
        outboard = 1.0 if cy > 0.0 else -1.0
        base.visual(
            Cylinder(radius=0.0035, length=0.012),
            origin=Origin(
                xyz=(cx + 0.006, cy + outboard * 0.020, 0.045),
                rpy=(1.57079632679, 0.0, 0.0),
            ),
            material=steel,
            name=f"lock_pin_{index}",
        )

    column = model.part("column")
    mast_length = 0.680
    mast_center_z = 0.020
    column.visual(
        Box((0.036, 0.026, mast_length)),
        origin=Origin(xyz=(0.0, 0.0, mast_center_z)),
        material=steel,
        name="mast",
    )
    column.visual(
        Box((0.022, 0.030, 0.042)),
        origin=Origin(xyz=(0.0, 0.0, 0.380)),
        material=dark_steel,
        name="head_block",
    )
    column.visual(
        Box((0.028, 0.100, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.410)),
        material=dark_steel,
        name="head_crossbar",
    )
    column.visual(
        Box((0.050, 0.008, 0.060)),
        origin=Origin(xyz=(0.0, 0.052, 0.410)),
        material=dark_steel,
        name="head_cheek_0",
    )
    column.visual(
        Box((0.050, 0.008, 0.060)),
        origin=Origin(xyz=(0.0, -0.052, 0.410)),
        material=dark_steel,
        name="head_cheek_1",
    )
    model.articulation(
        "base_to_column",
        ArticulationType.PRISMATIC,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, sleeve_bottom + sleeve_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.18, lower=0.0, upper=0.220),
    )

    top = model.part("top")
    top.visual(
        Box((0.580, 0.360, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=tabletop,
        name="panel",
    )
    top.visual(
        Box((0.210, 0.086, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=dark_steel,
        name="under_plate",
    )
    top.visual(
        Box((0.032, 0.032, 0.032)),
        origin=Origin(xyz=(0.050, 0.0, -0.010)),
        material=dark_steel,
        name="trunnion_strut_0",
    )
    top.visual(
        Box((0.032, 0.032, 0.032)),
        origin=Origin(xyz=(-0.050, 0.0, -0.010)),
        material=dark_steel,
        name="trunnion_strut_1",
    )
    top.visual(
        Box((0.012, 0.330, 0.016)),
        origin=Origin(xyz=(0.284, 0.0, 0.041)),
        material=tabletop,
        name="lip",
    )
    top.visual(
        Box((0.520, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, 0.171, 0.019)),
        material=dark_steel,
        name="edge_band_0",
    )
    top.visual(
        Box((0.520, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, -0.171, 0.019)),
        material=dark_steel,
        name="edge_band_1",
    )

    model.articulation(
        "column_to_top",
        ArticulationType.REVOLUTE,
        parent=column,
        child=top,
        origin=Origin(xyz=(0.0, 0.0, 0.421)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=-0.55, upper=0.70),
    )

    for index, (cx, cy) in enumerate(caster_positions):
        wheel = model.part(f"wheel_{index}")
        wheel.visual(
            Cylinder(radius=0.025, length=0.018),
            origin=Origin(rpy=(1.57079632679, 0.0, 0.0)),
            material=black,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.011, length=0.022),
            origin=Origin(rpy=(1.57079632679, 0.0, 0.0)),
            material=steel,
            name="hub",
        )
        model.articulation(
            f"wheel_spin_{index}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=wheel,
            origin=Origin(xyz=(cx, cy, 0.025)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=18.0),
        )

        outboard = 1.0 if cy > 0.0 else -1.0
        lock = model.part(f"lock_{index}")
        lock.visual(
            Cylinder(radius=0.0045, length=0.010),
            origin=Origin(rpy=(1.57079632679, 0.0, 0.0)),
            material=charcoal,
            name="barrel",
        )
        lock.visual(
            Box((0.020, 0.006, 0.004)),
            origin=Origin(xyz=(0.010, 0.0, 0.004)),
            material=charcoal,
            name="arm",
        )
        lock.visual(
            Box((0.011, 0.018, 0.004)),
            origin=Origin(xyz=(0.018, -outboard * 0.004, 0.007)),
            material=lock_red,
            name="pedal",
        )
        model.articulation(
            f"lock_pivot_{index}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=lock,
            origin=Origin(xyz=(cx + 0.006, cy + outboard * 0.020, 0.045)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=-0.70, upper=0.25),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    column = object_model.get_part("column")
    top = object_model.get_part("top")
    slide = object_model.get_articulation("base_to_column")
    tilt = object_model.get_articulation("column_to_top")

    ctx.expect_origin_distance(
        top,
        column,
        axes="xy",
        max_dist=0.002,
        name="top stays centered over the column",
    )

    for index in range(4):
        ctx.allow_overlap(
            base,
            f"wheel_{index}",
            elem_a=f"axle_pin_{index}",
            elem_b="hub",
            reason="Each caster wheel spins around a visible axle pin carried by the fork.",
        )
        ctx.allow_overlap(
            base,
            f"wheel_{index}",
            elem_a=f"axle_pin_{index}",
            elem_b="tire",
            reason="The caster tire is represented as a solid wheel proxy around the visible axle pin.",
        )
        ctx.allow_overlap(
            base,
            f"lock_{index}",
            elem_a=f"lock_pin_{index}",
            elem_b="barrel",
            reason="Each caster brake tab rotates around a small pivot pin on the fork.",
        )

    slide_limits = slide.motion_limits
    if slide_limits is not None and slide_limits.upper is not None:
        rest_column_pos = ctx.part_world_position(column)
        with ctx.pose({slide: slide_limits.upper}):
            extended_column_pos = ctx.part_world_position(column)
            mast_aabb = ctx.part_element_world_aabb(column, elem="mast")
            sleeve_aabb = ctx.part_element_world_aabb(base, elem="sleeve_front")
            retained = None
            if mast_aabb is not None and sleeve_aabb is not None:
                retained = sleeve_aabb[1][2] - mast_aabb[0][2]
            ctx.check(
                "column extends upward",
                rest_column_pos is not None
                and extended_column_pos is not None
                and extended_column_pos[2] > rest_column_pos[2] + 0.10,
                details=f"rest={rest_column_pos}, extended={extended_column_pos}",
            )
            ctx.check(
                "column retains insertion at full extension",
                retained is not None and retained >= 0.080,
                details=f"retained_insertion={retained}",
            )

    tilt_limits = tilt.motion_limits
    if tilt_limits is not None and tilt_limits.upper is not None:
        rest_lip_aabb = ctx.part_element_world_aabb(top, elem="lip")
        with ctx.pose({tilt: tilt_limits.upper}):
            raised_lip_aabb = ctx.part_element_world_aabb(top, elem="lip")
        ctx.check(
            "positive tilt raises the front lip",
            rest_lip_aabb is not None
            and raised_lip_aabb is not None
            and raised_lip_aabb[0][2] > rest_lip_aabb[0][2] + 0.10,
            details=f"rest={rest_lip_aabb}, raised={raised_lip_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
