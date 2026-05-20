from __future__ import annotations

from math import pi

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


WHEEL_RADIUS = 0.026
WHEEL_WIDTH = 0.018
RAIL_Y = 0.185
WHEEL_XS = (0.10, 0.58)


def _add_caster_support(base, x: float, y: float, material: str) -> None:
    sign_y = 1.0 if y >= 0.0 else -1.0
    base.visual(
        Box((0.028, 0.028, 0.028)),
        origin=Origin(xyz=(x, y, 0.066)),
        material=material,
    )
    base.visual(
        Box((0.020, 0.056, 0.008)),
        origin=Origin(xyz=(x, y, 0.052)),
        material=material,
    )
    for dy in (-0.013, 0.013):
        base.visual(
            Box((0.010, 0.006, 0.052)),
            origin=Origin(xyz=(x, y + dy, 0.026)),
            material=material,
        )
    base.visual(
        Box((0.020, 0.016, 0.020)),
        origin=Origin(xyz=(x - 0.016, y, 0.066)),
        material=material,
    )
    base.visual(
        Box((0.020, 0.020, 0.020)),
        origin=Origin(xyz=(x - 0.030, y, 0.066)),
        material=material,
    )
    base.visual(
        Box((0.022, 0.016, 0.042)),
        origin=Origin(xyz=(0.560, sign_y * 0.159, 0.081)),
        material=material,
        name=f"brake_tab_{'front' if sign_y > 0.0 else 'rear'}",
    )


def _add_caster_wheel(part, wheel_material: str, hub_material: str) -> None:
    wheel_axis = Origin(rpy=(pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        origin=wheel_axis,
        material=wheel_material,
        name="tire",
    )
    part.visual(
        Cylinder(radius=0.024, length=0.022),
        origin=wheel_axis,
        material=hub_material,
        name="hub_shell",
    )
    part.visual(
        Cylinder(radius=0.014, length=0.028),
        origin=wheel_axis,
        material=hub_material,
        name="hub_core",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pneumatic_overbed_table")

    steel = model.material("steel", rgba=(0.69, 0.71, 0.73, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.34, 0.36, 0.39, 1.0))
    tray_white = model.material("tray_white", rgba=(0.93, 0.94, 0.92, 1.0))
    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    plastic = model.material("plastic", rgba=(0.19, 0.21, 0.22, 1.0))

    base_frame = model.part("base_frame")
    base_frame.visual(
        Box((0.64, 0.045, 0.032)),
        origin=Origin(xyz=(0.34, RAIL_Y, 0.094)),
        material=dark_steel,
        name="front_rail",
    )
    base_frame.visual(
        Box((0.64, 0.045, 0.032)),
        origin=Origin(xyz=(0.34, -RAIL_Y, 0.094)),
        material=dark_steel,
        name="rear_rail",
    )
    base_frame.visual(
        Box((0.10, 0.41, 0.045)),
        origin=Origin(xyz=(0.05, 0.0, 0.100)),
        material=dark_steel,
        name="spine",
    )
    base_frame.visual(
        Box((0.12, 0.14, 0.080)),
        origin=Origin(xyz=(0.10, 0.0, 0.126)),
        material=steel,
        name="mast_pedestal",
    )
    base_frame.visual(
        Box((0.080, 0.220, 0.032)),
        origin=Origin(xyz=(0.120, 0.0, 0.142)),
        material=steel,
        name="upper_bridge",
    )

    for wheel_x in WHEEL_XS:
        _add_caster_support(base_frame, wheel_x, RAIL_Y, dark_steel)
        _add_caster_support(base_frame, wheel_x, -RAIL_Y, dark_steel)

    outer_sleeve = model.part("outer_sleeve")
    outer_sleeve.visual(
        Box((0.100, 0.080, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=steel,
        name="collar_base",
    )
    outer_sleeve.visual(
        Box((0.006, 0.080, 0.045)),
        origin=Origin(xyz=(-0.047, 0.0, 0.0225)),
        material=steel,
        name="collar_left",
    )
    outer_sleeve.visual(
        Box((0.006, 0.080, 0.045)),
        origin=Origin(xyz=(0.047, 0.0, 0.0225)),
        material=steel,
        name="collar_right",
    )
    outer_sleeve.visual(
        Box((0.088, 0.006, 0.045)),
        origin=Origin(xyz=(0.0, -0.037, 0.0225)),
        material=steel,
        name="collar_back",
    )
    outer_sleeve.visual(
        Box((0.088, 0.006, 0.045)),
        origin=Origin(xyz=(0.0, 0.037, 0.0225)),
        material=steel,
        name="collar_front",
    )
    outer_sleeve.visual(
        Box((0.012, 0.062, 0.020)),
        origin=Origin(xyz=(-0.043, 0.0, 0.049)),
        material=steel,
        name="collar_web_left",
    )
    outer_sleeve.visual(
        Box((0.012, 0.062, 0.020)),
        origin=Origin(xyz=(0.043, 0.0, 0.049)),
        material=steel,
        name="collar_web_right",
    )
    outer_sleeve.visual(
        Box((0.080, 0.012, 0.020)),
        origin=Origin(xyz=(0.0, -0.031, 0.049)),
        material=steel,
        name="collar_web_back",
    )
    outer_sleeve.visual(
        Box((0.080, 0.012, 0.020)),
        origin=Origin(xyz=(0.0, 0.031, 0.049)),
        material=steel,
        name="collar_web_front",
    )
    outer_sleeve.visual(
        Box((0.006, 0.056, 0.254)),
        origin=Origin(xyz=(-0.039, 0.0, 0.171)),
        material=steel,
        name="sleeve_left",
    )
    outer_sleeve.visual(
        Box((0.006, 0.056, 0.254)),
        origin=Origin(xyz=(0.039, 0.0, 0.171)),
        material=steel,
        name="sleeve_right",
    )
    outer_sleeve.visual(
        Box((0.072, 0.006, 0.254)),
        origin=Origin(xyz=(0.0, -0.025, 0.171)),
        material=steel,
        name="sleeve_back",
    )
    outer_sleeve.visual(
        Box((0.072, 0.006, 0.254)),
        origin=Origin(xyz=(0.0, 0.025, 0.171)),
        material=steel,
        name="sleeve_front",
    )
    outer_sleeve.visual(
        Box((0.010, 0.065, 0.020)),
        origin=Origin(xyz=(-0.0425, 0.0, 0.305)),
        material=steel,
        name="top_left",
    )
    outer_sleeve.visual(
        Box((0.010, 0.065, 0.020)),
        origin=Origin(xyz=(0.0425, 0.0, 0.305)),
        material=steel,
        name="top_right",
    )
    outer_sleeve.visual(
        Box((0.075, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, -0.0275, 0.305)),
        material=steel,
        name="top_back",
    )
    outer_sleeve.visual(
        Box((0.075, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, 0.0275, 0.305)),
        material=steel,
        name="top_front",
    )

    lift_column = model.part("lift_column")
    lift_column.visual(
        Box((0.055, 0.044, 0.600)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=charcoal,
        name="mast",
    )
    lift_column.visual(
        Box((0.120, 0.080, 0.036)),
        origin=Origin(xyz=(0.020, 0.0, 0.318)),
        material=steel,
        name="head_plate",
    )
    lift_column.visual(
        Box((0.016, 0.012, 0.070)),
        origin=Origin(xyz=(-0.010, -0.020, 0.350)),
        material=steel,
        name="hinge_ear_0",
    )
    lift_column.visual(
        Box((0.016, 0.012, 0.070)),
        origin=Origin(xyz=(-0.010, 0.020, 0.350)),
        material=steel,
        name="hinge_ear_1",
    )

    tray = model.part("tray")
    tray.visual(
        Cylinder(radius=0.009, length=0.028),
        origin=Origin(xyz=(-0.002, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )
    tray.visual(
        Box((0.780, 0.400, 0.016)),
        origin=Origin(xyz=(0.390, 0.0, 0.032)),
        material=tray_white,
        name="deck",
    )
    tray.visual(
        Box((0.030, 0.400, 0.050)),
        origin=Origin(xyz=(0.015, 0.0, 0.025)),
        material=tray_white,
        name="left_rim",
    )
    tray.visual(
        Box((0.030, 0.400, 0.040)),
        origin=Origin(xyz=(0.765, 0.0, 0.020)),
        material=tray_white,
        name="right_rim",
    )
    tray.visual(
        Box((0.720, 0.030, 0.040)),
        origin=Origin(xyz=(0.405, 0.185, 0.020)),
        material=tray_white,
        name="front_rim",
    )
    tray.visual(
        Box((0.720, 0.030, 0.040)),
        origin=Origin(xyz=(0.405, -0.185, 0.020)),
        material=tray_white,
        name="rear_rim",
    )
    tray.visual(
        Box((0.180, 0.160, 0.020)),
        origin=Origin(xyz=(0.120, 0.0, 0.014)),
        material=steel,
        name="underside_plate",
    )
    tray.visual(
        Box((0.016, 0.012, 0.026)),
        origin=Origin(xyz=(0.742, -0.020, 0.013)),
        material=steel,
        name="release_bracket_0",
    )
    tray.visual(
        Box((0.016, 0.012, 0.026)),
        origin=Origin(xyz=(0.742, 0.020, 0.013)),
        material=steel,
        name="release_bracket_1",
    )

    release_paddle = model.part("release_paddle")
    release_paddle.visual(
        Cylinder(radius=0.007, length=0.028),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=plastic,
        name="pivot_barrel",
    )
    release_paddle.visual(
        Box((0.026, 0.026, 0.020)),
        origin=Origin(xyz=(-0.013, 0.0, -0.010)),
        material=plastic,
        name="pivot_arm",
    )
    release_paddle.visual(
        Box((0.014, 0.050, 0.030)),
        origin=Origin(xyz=(-0.033, 0.0, -0.020)),
        material=plastic,
        name="stem",
    )
    release_paddle.visual(
        Box((0.100, 0.050, 0.010)),
        origin=Origin(xyz=(-0.083, 0.0, -0.028)),
        material=charcoal,
        name="paddle",
    )

    brake_bar = model.part("brake_bar")
    brake_bar.visual(
        Box((0.028, 0.260, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=charcoal,
        name="cross_bar",
    )
    brake_bar.visual(
        Box((0.018, 0.050, 0.040)),
        origin=Origin(xyz=(0.0, -0.125, -0.010)),
        material=charcoal,
        name="arm_0",
    )
    brake_bar.visual(
        Box((0.018, 0.050, 0.040)),
        origin=Origin(xyz=(0.0, 0.125, -0.010)),
        material=charcoal,
        name="arm_1",
    )
    brake_bar.visual(
        Cylinder(radius=0.006, length=0.016),
        origin=Origin(xyz=(0.0, -0.145, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="pivot_0",
    )
    brake_bar.visual(
        Cylinder(radius=0.006, length=0.016),
        origin=Origin(xyz=(0.0, 0.145, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="pivot_1",
    )
    brake_bar.visual(
        Box((0.018, 0.070, 0.020)),
        origin=Origin(xyz=(0.0, 0.090, -0.024)),
        material=charcoal,
        name="pedal_web",
    )
    brake_bar.visual(
        Box((0.045, 0.080, 0.010)),
        origin=Origin(xyz=(0.0, 0.090, -0.035)),
        material=charcoal,
        name="pedal_pad",
    )

    caster_0 = model.part("caster_0")
    _add_caster_wheel(caster_0, rubber, steel)
    caster_1 = model.part("caster_1")
    _add_caster_wheel(caster_1, rubber, steel)
    caster_2 = model.part("caster_2")
    _add_caster_wheel(caster_2, rubber, steel)
    caster_3 = model.part("caster_3")
    _add_caster_wheel(caster_3, rubber, steel)

    model.articulation(
        "base_to_sleeve",
        ArticulationType.FIXED,
        parent=base_frame,
        child=outer_sleeve,
        origin=Origin(xyz=(0.100, 0.0, 0.166)),
    )
    model.articulation(
        "sleeve_lift",
        ArticulationType.PRISMATIC,
        parent=outer_sleeve,
        child=lift_column,
        origin=Origin(xyz=(0.0, 0.0, 0.315)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.25, lower=0.0, upper=0.22),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=lift_column,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, 0.350)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.2, lower=0.0, upper=0.50),
    )
    model.articulation(
        "release_pivot",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=release_paddle,
        origin=Origin(xyz=(0.742, 0.0, 0.013)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=0.0, upper=0.55),
    )
    model.articulation(
        "brake_pivot",
        ArticulationType.REVOLUTE,
        parent=base_frame,
        child=brake_bar,
        origin=Origin(xyz=(0.560, 0.0, 0.081)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=-0.20, upper=0.45),
    )

    caster_positions = (
        (caster_0, WHEEL_XS[0], RAIL_Y),
        (caster_1, WHEEL_XS[1], RAIL_Y),
        (caster_2, WHEEL_XS[0], -RAIL_Y),
        (caster_3, WHEEL_XS[1], -RAIL_Y),
    )
    for index, (caster, x_pos, y_pos) in enumerate(caster_positions):
        model.articulation(
            f"caster_{index}_spin",
            ArticulationType.CONTINUOUS,
            parent=base_frame,
            child=caster,
            origin=Origin(xyz=(x_pos, y_pos, WHEEL_RADIUS)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.5, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    outer_sleeve = object_model.get_part("outer_sleeve")
    lift_column = object_model.get_part("lift_column")
    tray = object_model.get_part("tray")
    release_paddle = object_model.get_part("release_paddle")
    brake_bar = object_model.get_part("brake_bar")

    sleeve_lift = object_model.get_articulation("sleeve_lift")
    head_tilt = object_model.get_articulation("head_tilt")
    release_pivot = object_model.get_articulation("release_pivot")
    brake_pivot = object_model.get_articulation("brake_pivot")

    lift_limits = sleeve_lift.motion_limits
    tilt_limits = head_tilt.motion_limits
    release_limits = release_pivot.motion_limits
    brake_limits = brake_pivot.motion_limits

    if lift_limits is not None and lift_limits.upper is not None:
        with ctx.pose({sleeve_lift: 0.0}):
            ctx.expect_within(
                lift_column,
                outer_sleeve,
                axes="xy",
                inner_elem="mast",
                margin=0.020,
                name="mast stays centered inside sleeve at rest",
            )
            ctx.expect_overlap(
                lift_column,
                outer_sleeve,
                axes="z",
                elem_a="mast",
                elem_b="sleeve_left",
                min_overlap=0.25,
                name="mast remains deeply nested at rest",
            )
            rest_column_pos = ctx.part_world_position(lift_column)

        with ctx.pose({sleeve_lift: lift_limits.upper}):
            ctx.expect_within(
                lift_column,
                outer_sleeve,
                axes="xy",
                inner_elem="mast",
                margin=0.020,
                name="mast stays centered at full lift",
            )
            ctx.expect_overlap(
                lift_column,
                outer_sleeve,
                axes="z",
                elem_a="mast",
                elem_b="sleeve_left",
                min_overlap=0.05,
                name="mast retains insertion at full lift",
            )
            extended_column_pos = ctx.part_world_position(lift_column)

        ctx.check(
            "column rises upward",
            rest_column_pos is not None
            and extended_column_pos is not None
            and extended_column_pos[2] > rest_column_pos[2] + 0.18,
            details=f"rest={rest_column_pos}, extended={extended_column_pos}",
        )

    if tilt_limits is not None and tilt_limits.upper is not None:
        with ctx.pose({sleeve_lift: 0.0, head_tilt: 0.0}):
            ctx.expect_gap(
                tray,
                lift_column,
                axis="z",
                positive_elem="deck",
                negative_elem="head_plate",
                min_gap=0.0,
                max_gap=0.08,
                name="tray deck sits just above the head plate",
            )
            closed_edge = ctx.part_element_world_aabb(tray, elem="right_rim")

        with ctx.pose({sleeve_lift: 0.0, head_tilt: tilt_limits.upper}):
            tilted_edge = ctx.part_element_world_aabb(tray, elem="right_rim")

        closed_edge_z = closed_edge[1][2] if closed_edge is not None else None
        tilted_edge_z = tilted_edge[1][2] if tilted_edge is not None else None
        ctx.check(
            "tray right edge tilts upward",
            closed_edge_z is not None
            and tilted_edge_z is not None
            and tilted_edge_z > closed_edge_z + 0.12,
            details=f"closed={closed_edge_z}, tilted={tilted_edge_z}",
        )

    if release_limits is not None and release_limits.upper is not None:
        with ctx.pose({release_pivot: 0.0}):
            rest_paddle = ctx.part_element_world_aabb(release_paddle, elem="paddle")
        with ctx.pose({release_pivot: release_limits.upper}):
            actuated_paddle = ctx.part_element_world_aabb(release_paddle, elem="paddle")

        rest_paddle_z = rest_paddle[1][2] if rest_paddle is not None else None
        actuated_paddle_z = actuated_paddle[1][2] if actuated_paddle is not None else None
        ctx.check(
            "release paddle rotates upward",
            rest_paddle_z is not None
            and actuated_paddle_z is not None
            and actuated_paddle_z > rest_paddle_z + 0.02,
            details=f"rest={rest_paddle_z}, actuated={actuated_paddle_z}",
        )

    if brake_limits is not None and brake_limits.upper is not None:
        with ctx.pose({brake_pivot: 0.0}):
            rest_brake = ctx.part_world_aabb(brake_bar)
        with ctx.pose({brake_pivot: brake_limits.upper}):
            actuated_brake = ctx.part_world_aabb(brake_bar)

        rest_brake_z = rest_brake[1][2] if rest_brake is not None else None
        actuated_brake_z = actuated_brake[1][2] if actuated_brake is not None else None
        ctx.check(
            "brake bar rocks on its pivots",
            rest_brake_z is not None
            and actuated_brake_z is not None
            and actuated_brake_z > rest_brake_z + 0.03,
            details=f"rest={rest_brake_z}, actuated={actuated_brake_z}",
        )

    return ctx.report()


object_model = build_object_model()
