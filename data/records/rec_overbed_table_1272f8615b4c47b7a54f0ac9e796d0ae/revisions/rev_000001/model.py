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


def _add_caster_mount(
    frame,
    *,
    x: float,
    y: float,
    prefix: str,
    steel,
) -> None:
    frame.visual(
        Cylinder(radius=0.011, length=0.010),
        origin=Origin(xyz=(x, y, 0.041)),
        material=steel,
        name=f"{prefix}_swivel_housing",
    )
    frame.visual(
        Box((0.034, 0.018, 0.006)),
        origin=Origin(xyz=(x, y, 0.038)),
        material=steel,
        name=f"{prefix}_fork_bridge",
    )
    for side_index, side_sign in enumerate((-1.0, 1.0)):
        frame.visual(
            Box((0.006, 0.018, 0.026)),
            origin=Origin(xyz=(x + 0.013 * side_sign, y, 0.022)),
            material=steel,
            name=f"{prefix}_fork_arm_{side_index}",
        )


def _add_wheel_visuals(wheel_part, *, rubber, hub_color) -> None:
    axle_origin = Origin(rpy=(0.0, 1.5707963267948966, 0.0))
    wheel_part.visual(
        Cylinder(radius=0.017, length=0.016),
        origin=axle_origin,
        material=rubber,
        name="tire",
    )
    wheel_part.visual(
        Cylinder(radius=0.011, length=0.018),
        origin=axle_origin,
        material=hub_color,
        name="hub",
    )
    wheel_part.visual(
        Cylinder(radius=0.006, length=0.022),
        origin=axle_origin,
        material=hub_color,
        name="axle_boss",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overbed_table")

    frame_paint = model.material("frame_paint", rgba=(0.78, 0.80, 0.82, 1.0))
    head_paint = model.material("head_paint", rgba=(0.70, 0.72, 0.75, 1.0))
    top_finish = model.material("top_finish", rgba=(0.88, 0.80, 0.67, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.17, 0.18, 0.20, 1.0))
    steel = model.material("steel", rgba=(0.54, 0.57, 0.60, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.045, 0.310, 0.022)),
        origin=Origin(xyz=(-0.135, -0.060, 0.050)),
        material=frame_paint,
        name="rail_0",
    )
    frame.visual(
        Box((0.045, 0.310, 0.022)),
        origin=Origin(xyz=(0.135, -0.060, 0.050)),
        material=frame_paint,
        name="rail_1",
    )
    frame.visual(
        Box((0.285, 0.045, 0.018)),
        origin=Origin(xyz=(0.000, -0.205, 0.061)),
        material=frame_paint,
        name="rear_crossmember",
    )
    frame.visual(
        Box((0.240, 0.055, 0.016)),
        origin=Origin(xyz=(0.000, -0.005, 0.064)),
        material=frame_paint,
        name="center_bridge",
    )
    frame.visual(
        Box((0.020, 0.055, 0.038)),
        origin=Origin(xyz=(-0.052, -0.005, 0.080)),
        material=frame_paint,
        name="pedestal_web_0",
    )
    frame.visual(
        Box((0.020, 0.055, 0.038)),
        origin=Origin(xyz=(0.052, -0.005, 0.080)),
        material=frame_paint,
        name="pedestal_web_1",
    )
    frame.visual(
        Box((0.008, 0.018, 0.018)),
        origin=Origin(xyz=(-0.113, 0.048, 0.064)),
        material=steel,
        name="brake_bracket_0",
    )
    frame.visual(
        Box((0.008, 0.018, 0.018)),
        origin=Origin(xyz=(0.113, 0.048, 0.064)),
        material=steel,
        name="brake_bracket_1",
    )

    caster_locations = (
        ("front_caster_0", -0.135, 0.085),
        ("front_caster_1", 0.135, 0.085),
        ("rear_caster_0", -0.135, -0.205),
        ("rear_caster_1", 0.135, -0.205),
    )
    for prefix, x, y in caster_locations:
        _add_caster_mount(frame, x=x, y=y, prefix=prefix, steel=steel)

    sleeve = model.part("sleeve")
    sleeve.visual(
        Box((0.078, 0.074, 0.022)),
        origin=Origin(xyz=(0.000, 0.000, 0.011)),
        material=frame_paint,
        name="lower_block",
    )
    sleeve.visual(
        Box((0.012, 0.040, 0.070)),
        origin=Origin(xyz=(-0.026, -0.010, 0.045)),
        material=frame_paint,
        name="gusset_0",
    )
    sleeve.visual(
        Box((0.012, 0.040, 0.070)),
        origin=Origin(xyz=(0.026, -0.010, 0.045)),
        material=frame_paint,
        name="gusset_1",
    )
    sleeve.visual(
        Box((0.004, 0.046, 0.338)),
        origin=Origin(xyz=(-0.028, 0.000, 0.191)),
        material=frame_paint,
        name="wall_0",
    )
    sleeve.visual(
        Box((0.004, 0.046, 0.338)),
        origin=Origin(xyz=(0.028, 0.000, 0.191)),
        material=frame_paint,
        name="wall_1",
    )
    sleeve.visual(
        Box((0.060, 0.004, 0.338)),
        origin=Origin(xyz=(0.000, 0.021, 0.191)),
        material=frame_paint,
        name="wall_2",
    )
    sleeve.visual(
        Box((0.060, 0.004, 0.338)),
        origin=Origin(xyz=(0.000, -0.021, 0.191)),
        material=frame_paint,
        name="wall_3",
    )

    column = model.part("column")
    column.visual(
        Box((0.052, 0.032, 0.560)),
        origin=Origin(xyz=(0.000, 0.000, 0.020)),
        material=head_paint,
        name="mast",
    )
    column.visual(
        Box((0.056, 0.042, 0.018)),
        origin=Origin(xyz=(0.000, 0.000, 0.309)),
        material=head_paint,
        name="mast_collar",
    )
    column.visual(
        Box((0.040, 0.030, 0.040)),
        origin=Origin(xyz=(0.000, 0.000, 0.338)),
        material=head_paint,
        name="mast_neck",
    )

    support_head = model.part("support_head")
    support_head.visual(
        Box((0.092, 0.060, 0.018)),
        origin=Origin(xyz=(0.000, 0.000, 0.009)),
        material=head_paint,
        name="base_block",
    )
    support_head.visual(
        Box((0.012, 0.090, 0.028)),
        origin=Origin(xyz=(-0.043, 0.000, 0.023)),
        material=head_paint,
        name="cheek_0",
    )
    support_head.visual(
        Box((0.012, 0.090, 0.028)),
        origin=Origin(xyz=(0.043, 0.000, 0.023)),
        material=head_paint,
        name="cheek_1",
    )
    support_head.visual(
        Box((0.060, 0.018, 0.022)),
        origin=Origin(xyz=(0.000, -0.028, 0.020)),
        material=head_paint,
        name="rear_rib",
    )

    top = model.part("top")
    top.visual(
        Box((0.560, 0.360, 0.016)),
        origin=Origin(xyz=(0.000, 0.000, 0.018)),
        material=top_finish,
        name="panel",
    )
    top.visual(
        Cylinder(radius=0.010, length=0.068),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(0.0, 1.5707963267948966, 0.0)),
        material=dark_trim,
        name="tilt_barrel",
    )
    top.visual(
        Box((0.500, 0.010, 0.020)),
        origin=Origin(xyz=(0.000, 0.174, 0.036)),
        material=dark_trim,
        name="retaining_lip",
    )

    brake_bar = model.part("brake_bar")
    brake_bar.visual(
        Cylinder(radius=0.009, length=0.194),
        origin=Origin(rpy=(0.0, 1.5707963267948966, 0.0)),
        material=dark_trim,
        name="pedal_tube",
    )
    brake_bar.visual(
        Cylinder(radius=0.0045, length=0.012),
        origin=Origin(xyz=(-0.103, 0.000, 0.000), rpy=(0.0, 1.5707963267948966, 0.0)),
        material=steel,
        name="pivot_0",
    )
    brake_bar.visual(
        Cylinder(radius=0.0045, length=0.012),
        origin=Origin(xyz=(0.103, 0.000, 0.000), rpy=(0.0, 1.5707963267948966, 0.0)),
        material=steel,
        name="pivot_1",
    )
    brake_bar.visual(
        Box((0.180, 0.022, 0.010)),
        origin=Origin(xyz=(0.000, 0.014, -0.013)),
        material=dark_trim,
        name="pedal_pad",
    )
    brake_bar.visual(
        Box((0.028, 0.024, 0.010)),
        origin=Origin(xyz=(-0.060, 0.022, -0.022)),
        material=dark_trim,
        name="toe_tab_0",
    )
    brake_bar.visual(
        Box((0.028, 0.024, 0.010)),
        origin=Origin(xyz=(0.060, 0.022, -0.022)),
        material=dark_trim,
        name="toe_tab_1",
    )

    for wheel_name in ("front_caster_0", "front_caster_1", "rear_caster_0", "rear_caster_1"):
        wheel = model.part(wheel_name)
        _add_wheel_visuals(wheel, rubber=rubber, hub_color=steel)

    model.articulation(
        "frame_to_sleeve",
        ArticulationType.FIXED,
        parent=frame,
        child=sleeve,
        origin=Origin(xyz=(0.000, 0.000, 0.072)),
    )
    model.articulation(
        "sleeve_to_column",
        ArticulationType.PRISMATIC,
        parent=sleeve,
        child=column,
        origin=Origin(xyz=(0.000, 0.000, 0.360)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.20,
            lower=0.0,
            upper=0.170,
        ),
    )
    model.articulation(
        "column_to_support_head",
        ArticulationType.FIXED,
        parent=column,
        child=support_head,
        origin=Origin(xyz=(0.000, 0.000, 0.358)),
    )
    model.articulation(
        "head_to_top",
        ArticulationType.REVOLUTE,
        parent=support_head,
        child=top,
        origin=Origin(xyz=(0.000, 0.000, 0.028)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=-0.12,
            upper=0.95,
        ),
    )
    model.articulation(
        "frame_to_brake_bar",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=brake_bar,
        origin=Origin(xyz=(0.000, 0.048, 0.064)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-0.45,
            upper=0.20,
        ),
    )

    for wheel_name, x, y in (
        ("front_caster_0", -0.135, 0.085),
        ("front_caster_1", 0.135, 0.085),
        ("rear_caster_0", -0.135, -0.205),
        ("rear_caster_1", 0.135, -0.205),
    ):
        model.articulation(
            f"{wheel_name}_spin",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel_name,
            origin=Origin(xyz=(x, y, 0.017)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=20.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    sleeve = object_model.get_part("sleeve")
    column = object_model.get_part("column")
    top = object_model.get_part("top")
    lift = object_model.get_articulation("sleeve_to_column")
    tilt = object_model.get_articulation("head_to_top")

    ctx.expect_origin_distance(
        top,
        column,
        axes="xy",
        max_dist=0.001,
        name="top stays centered over the column",
    )
    ctx.expect_within(
        column,
        sleeve,
        axes="xy",
        inner_elem="mast",
        margin=0.004,
        name="mast stays centered inside the sleeve at rest",
    )
    ctx.expect_overlap(
        column,
        sleeve,
        axes="z",
        elem_a="mast",
        min_overlap=0.250,
        name="collapsed mast keeps deep insertion in the sleeve",
    )

    rest_top_pos = ctx.part_world_position(top)
    rest_lip_aabb = ctx.part_element_world_aabb(top, elem="retaining_lip")

    lift_limits = lift.motion_limits
    if lift_limits is not None and lift_limits.upper is not None:
        with ctx.pose({lift: lift_limits.upper}):
            ctx.expect_within(
                column,
                sleeve,
                axes="xy",
                inner_elem="mast",
                margin=0.004,
                name="mast stays centered inside the sleeve when extended",
            )
            ctx.expect_overlap(
                column,
                sleeve,
                axes="z",
                elem_a="mast",
                min_overlap=0.080,
                name="extended mast still remains inserted in the sleeve",
            )
            extended_top_pos = ctx.part_world_position(top)
        ctx.check(
            "column raises the work surface",
            rest_top_pos is not None
            and extended_top_pos is not None
            and extended_top_pos[2] > rest_top_pos[2] + 0.15,
            details=f"rest={rest_top_pos}, extended={extended_top_pos}",
        )

    tilt_limits = tilt.motion_limits
    if tilt_limits is not None and tilt_limits.upper is not None:
        with ctx.pose({tilt: tilt_limits.upper}):
            tilted_lip_aabb = ctx.part_element_world_aabb(top, elem="retaining_lip")
        ctx.check(
            "tilt raises the front retaining lip",
            rest_lip_aabb is not None
            and tilted_lip_aabb is not None
            and tilted_lip_aabb[1][2] > rest_lip_aabb[1][2] + 0.10,
            details=f"rest={rest_lip_aabb}, tilted={tilted_lip_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
