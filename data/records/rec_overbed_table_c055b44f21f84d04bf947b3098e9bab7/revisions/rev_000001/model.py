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


BASE_WIDTH = 0.48
BASE_DEPTH = 0.59
BASE_RAIL_X = 0.06
BASE_RAIL_Z = 0.035
BASE_BOTTOM_Z = 0.110
BASE_TOP_Z = BASE_BOTTOM_Z + BASE_RAIL_Z
BASE_CENTER_Z = BASE_BOTTOM_Z + BASE_RAIL_Z * 0.5
REAR_CROSS_Y = -0.267
SIDE_RAIL_X = 0.210
CASTER_AXLE_Z = 0.045
OUTER_POST_BASE = (0.0, -0.215, 0.155)


def _spin_x_origin() -> Origin:
    return Origin(rpy=(0.0, pi / 2.0, 0.0))


def _add_wheel(part, *, tire_material: str, hub_material: str) -> None:
    spin_origin = _spin_x_origin()
    part.visual(
        Cylinder(radius=0.040, length=0.022),
        origin=spin_origin,
        material=tire_material,
        name="tire",
    )
    part.visual(
        Cylinder(radius=0.031, length=0.028),
        origin=spin_origin,
        material=hub_material,
        name="rim",
    )
    part.visual(
        Cylinder(radius=0.015, length=0.030),
        origin=spin_origin,
        material=hub_material,
        name="hub",
    )


def _add_fork(part, *, material: str) -> None:
    part.visual(
        Box((0.004, 0.018, 0.045)),
        origin=Origin(xyz=(0.017, 0.0, 0.0225)),
        material=material,
        name="left_leg",
    )
    part.visual(
        Box((0.004, 0.018, 0.045)),
        origin=Origin(xyz=(-0.017, 0.0, 0.0225)),
        material=material,
        name="right_leg",
    )
    part.visual(
        Box((0.042, 0.024, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=material,
        name="crown",
    )
    part.visual(
        Box((0.020, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, 0.018, 0.055)),
        material=material,
        name="lock_boss",
    )
    part.visual(
        Cylinder(radius=0.007, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=material,
        name="stem",
    )
    part.visual(
        Box((0.050, 0.028, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.063)),
        material=material,
        name="mount_plate",
    )


def _add_lock_tab(part, *, material: str) -> None:
    part.visual(
        Box((0.012, 0.008, 0.006)),
        origin=Origin(xyz=(0.0, 0.004, -0.003)),
        material=material,
        name="pivot_block",
    )
    part.visual(
        Box((0.018, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, 0.010, -0.007)),
        material=material,
        name="arm",
    )
    part.visual(
        Box((0.030, 0.014, 0.004)),
        origin=Origin(xyz=(0.0, 0.020, -0.016)),
        material=material,
        name="pedal",
    )


def _add_caster(
    model: ArticulatedObject,
    *,
    base,
    prefix: str,
    xyz: tuple[float, float, float],
    fork_material: str,
    tire_material: str,
    hub_material: str,
    lock_material: str,
) -> None:
    fork = model.part(f"{prefix}_fork")
    _add_fork(fork, material=fork_material)

    wheel = model.part(f"{prefix}_wheel")
    _add_wheel(wheel, tire_material=tire_material, hub_material=hub_material)

    lock = model.part(f"{prefix}_lock")
    _add_lock_tab(lock, material=lock_material)

    model.articulation(
        f"{prefix}_fork_mount",
        ArticulationType.FIXED,
        parent=base,
        child=fork,
        origin=Origin(xyz=xyz),
    )
    model.articulation(
        f"{prefix}_spin",
        ArticulationType.CONTINUOUS,
        parent=fork,
        child=wheel,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=24.0),
    )
    model.articulation(
        f"{prefix}_lock_pivot",
        ArticulationType.REVOLUTE,
        parent=fork,
        child=lock,
        origin=Origin(xyz=(0.0, 0.024, 0.050)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=2.5,
            lower=-0.55,
            upper=0.35,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hospital_overbed_table")

    frame_paint = model.material("frame_paint", rgba=(0.71, 0.74, 0.77, 1.0))
    column_paint = model.material("column_paint", rgba=(0.83, 0.85, 0.87, 1.0))
    tray_beige = model.material("tray_beige", rgba=(0.90, 0.88, 0.80, 1.0))
    tray_trim = model.material("tray_trim", rgba=(0.82, 0.79, 0.70, 1.0))
    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    lock_red = model.material("lock_red", rgba=(0.73, 0.18, 0.14, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_RAIL_X, BASE_DEPTH + 0.002, BASE_RAIL_Z)),
        origin=Origin(xyz=(SIDE_RAIL_X, 0.0, BASE_CENTER_Z)),
        material=frame_paint,
        name="left_rail",
    )
    base.visual(
        Box((BASE_RAIL_X, BASE_DEPTH + 0.002, BASE_RAIL_Z)),
        origin=Origin(xyz=(-SIDE_RAIL_X, 0.0, BASE_CENTER_Z)),
        material=frame_paint,
        name="right_rail",
    )
    base.visual(
        Box((BASE_WIDTH, 0.058, BASE_RAIL_Z)),
        origin=Origin(xyz=(0.0, REAR_CROSS_Y, BASE_CENTER_Z)),
        material=frame_paint,
        name="rear_crossbar",
    )
    base.visual(
        Box((0.140, 0.120, 0.010)),
        origin=Origin(xyz=(0.0, -0.215, 0.150)),
        material=frame_paint,
        name="post_plate",
    )

    outer_post = model.part("outer_post")
    outer_post.visual(
        Box((0.090, 0.070, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=column_paint,
        name="base_cap",
    )
    outer_post.visual(
        Box((0.092, 0.012, 0.310)),
        origin=Origin(xyz=(0.0, -0.029, 0.175)),
        material=column_paint,
        name="rear_wall",
    )
    outer_post.visual(
        Box((0.092, 0.012, 0.310)),
        origin=Origin(xyz=(0.0, 0.029, 0.175)),
        material=column_paint,
        name="front_wall",
    )
    outer_post.visual(
        Box((0.012, 0.070, 0.310)),
        origin=Origin(xyz=(0.039, 0.0, 0.175)),
        material=column_paint,
        name="left_wall",
    )
    outer_post.visual(
        Box((0.012, 0.070, 0.310)),
        origin=Origin(xyz=(-0.039, 0.0, 0.175)),
        material=column_paint,
        name="right_wall",
    )
    outer_post.visual(
        Box((0.004, 0.030, 0.050)),
        origin=Origin(xyz=(0.031, 0.0, 0.280)),
        material=charcoal,
        name="left_guide",
    )
    outer_post.visual(
        Box((0.004, 0.030, 0.050)),
        origin=Origin(xyz=(-0.031, 0.0, 0.280)),
        material=charcoal,
        name="right_guide",
    )
    outer_post.visual(
        Box((0.050, 0.004, 0.050)),
        origin=Origin(xyz=(0.0, 0.021, 0.280)),
        material=charcoal,
        name="front_guide",
    )
    outer_post.visual(
        Box((0.050, 0.004, 0.050)),
        origin=Origin(xyz=(0.0, -0.021, 0.280)),
        material=charcoal,
        name="rear_guide",
    )

    inner_column = model.part("inner_column")
    inner_column.visual(
        Box((0.058, 0.038, 0.600)),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=column_paint,
        name="column_body",
    )
    inner_column.visual(
        Box((0.070, 0.050, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.309)),
        material=column_paint,
        name="top_cap",
    )

    support_head = model.part("support_head")
    support_head.visual(
        Box((0.060, 0.050, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=frame_paint,
        name="neck",
    )
    support_head.visual(
        Box((0.120, 0.050, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=frame_paint,
        name="saddle",
    )
    support_head.visual(
        Box((0.022, 0.030, 0.028)),
        origin=Origin(xyz=(0.051, 0.0, 0.064)),
        material=frame_paint,
        name="left_ear",
    )
    support_head.visual(
        Box((0.022, 0.030, 0.028)),
        origin=Origin(xyz=(-0.051, 0.0, 0.064)),
        material=frame_paint,
        name="right_ear",
    )
    support_head.visual(
        Box((0.034, 0.018, 0.030)),
        origin=Origin(xyz=(0.0, -0.030, 0.045)),
        material=frame_paint,
        name="rear_brace",
    )

    tray = model.part("tray")
    tray.visual(
        Cylinder(radius=0.010, length=0.080),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_paint,
        name="hinge_barrel",
    )
    tray.visual(
        Box((0.074, 0.090, 0.024)),
        origin=Origin(xyz=(0.0, 0.045, 0.009)),
        material=frame_paint,
        name="underside_mount",
    )
    tray.visual(
        Box((0.780, 0.400, 0.012)),
        origin=Origin(xyz=(0.0, 0.240, 0.021)),
        material=tray_beige,
        name="tray_deck",
    )
    tray.visual(
        Box((0.780, 0.016, 0.020)),
        origin=Origin(xyz=(0.0, 0.432, 0.037)),
        material=tray_trim,
        name="front_lip",
    )
    tray.visual(
        Box((0.780, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, 0.047, 0.033)),
        material=tray_trim,
        name="rear_lip",
    )
    tray.visual(
        Box((0.016, 0.370, 0.020)),
        origin=Origin(xyz=(0.382, 0.239, 0.037)),
        material=tray_trim,
        name="left_lip",
    )
    tray.visual(
        Box((0.016, 0.370, 0.020)),
        origin=Origin(xyz=(-0.382, 0.239, 0.037)),
        material=tray_trim,
        name="right_lip",
    )

    lever = model.part("front_lever")
    lever.visual(
        Cylinder(radius=0.006, length=0.100),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=charcoal,
        name="pivot_bar",
    )
    lever.visual(
        Box((0.012, 0.028, 0.038)),
        origin=Origin(xyz=(0.032, 0.014, -0.019)),
        material=charcoal,
        name="left_arm",
    )
    lever.visual(
        Box((0.012, 0.028, 0.038)),
        origin=Origin(xyz=(-0.032, 0.014, -0.019)),
        material=charcoal,
        name="right_arm",
    )
    lever.visual(
        Cylinder(radius=0.008, length=0.140),
        origin=Origin(xyz=(0.0, 0.032, -0.034), rpy=(0.0, pi / 2.0, 0.0)),
        material=charcoal,
        name="lever_grip",
    )

    model.articulation(
        "post_mount",
        ArticulationType.FIXED,
        parent=base,
        child=outer_post,
        origin=Origin(xyz=OUTER_POST_BASE),
    )
    model.articulation(
        "column_lift",
        ArticulationType.PRISMATIC,
        parent=outer_post,
        child=inner_column,
        origin=Origin(xyz=(0.0, 0.0, 0.330)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.12,
            lower=0.0,
            upper=0.180,
        ),
    )
    model.articulation(
        "head_mount",
        ArticulationType.FIXED,
        parent=inner_column,
        child=support_head,
        origin=Origin(xyz=(0.0, 0.0, 0.318)),
    )
    model.articulation(
        "tray_tilt",
        ArticulationType.REVOLUTE,
        parent=support_head,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=-0.18,
            upper=0.55,
        ),
    )
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=lever,
        origin=Origin(xyz=(0.0, 0.378, 0.009)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=0.0,
            upper=0.55,
        ),
    )

    fork_material = frame_paint.name
    _add_caster(
        model,
        base=base,
        prefix="front_left",
        xyz=(SIDE_RAIL_X, 0.270, CASTER_AXLE_Z),
        fork_material=fork_material,
        tire_material=charcoal.name,
        hub_material=frame_paint.name,
        lock_material=lock_red.name,
    )
    _add_caster(
        model,
        base=base,
        prefix="front_right",
        xyz=(-SIDE_RAIL_X, 0.270, CASTER_AXLE_Z),
        fork_material=fork_material,
        tire_material=charcoal.name,
        hub_material=frame_paint.name,
        lock_material=lock_red.name,
    )
    _add_caster(
        model,
        base=base,
        prefix="rear_left",
        xyz=(SIDE_RAIL_X, -0.270, CASTER_AXLE_Z),
        fork_material=fork_material,
        tire_material=charcoal.name,
        hub_material=frame_paint.name,
        lock_material=lock_red.name,
    )
    _add_caster(
        model,
        base=base,
        prefix="rear_right",
        xyz=(-SIDE_RAIL_X, -0.270, CASTER_AXLE_Z),
        fork_material=fork_material,
        tire_material=charcoal.name,
        hub_material=frame_paint.name,
        lock_material=lock_red.name,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    outer_post = object_model.get_part("outer_post")
    inner_column = object_model.get_part("inner_column")
    support_head = object_model.get_part("support_head")
    tray = object_model.get_part("tray")
    lever = object_model.get_part("front_lever")
    front_left_wheel = object_model.get_part("front_left_wheel")

    lift = object_model.get_articulation("column_lift")
    tray_tilt = object_model.get_articulation("tray_tilt")
    lever_pivot = object_model.get_articulation("lever_pivot")

    lift_limits = lift.motion_limits
    tilt_limits = tray_tilt.motion_limits
    lever_limits = lever_pivot.motion_limits

    ctx.expect_within(
        inner_column,
        outer_post,
        axes="xy",
        inner_elem="column_body",
        margin=0.0,
        name="inner column stays centered in the outer post at rest",
    )
    ctx.expect_overlap(
        inner_column,
        outer_post,
        axes="z",
        elem_a="column_body",
        min_overlap=0.28,
        name="inner column remains deeply inserted at rest",
    )
    ctx.expect_contact(
        tray,
        support_head,
        elem_a="hinge_barrel",
        elem_b="left_ear",
        name="tray barrel seats against the left support ear",
    )
    ctx.expect_gap(
        base,
        front_left_wheel,
        axis="z",
        min_gap=0.018,
        max_gap=0.030,
        positive_elem="left_rail",
        negative_elem="tire",
        name="base rail clears the front left caster wheel",
    )
    ctx.expect_gap(
        tray,
        lever,
        axis="z",
        min_gap=0.010,
        max_gap=0.050,
        positive_elem="tray_deck",
        negative_elem="lever_grip",
        name="release lever hangs beneath the tray front edge",
    )

    if lift_limits is not None and lift_limits.upper is not None:
        rest_column_pos = ctx.part_world_position(inner_column)
        with ctx.pose({lift: lift_limits.upper}):
            ctx.expect_within(
                inner_column,
                outer_post,
                axes="xy",
                inner_elem="column_body",
                margin=0.0,
                name="inner column stays centered at full lift",
            )
            ctx.expect_overlap(
                inner_column,
                outer_post,
                axes="z",
                elem_a="column_body",
                min_overlap=0.11,
                name="inner column keeps retained insertion at full lift",
            )
            extended_column_pos = ctx.part_world_position(inner_column)

        ctx.check(
            "column lift raises the tray support head",
            rest_column_pos is not None
            and extended_column_pos is not None
            and extended_column_pos[2] > rest_column_pos[2] + 0.15,
            details=f"rest={rest_column_pos}, extended={extended_column_pos}",
        )

    if tilt_limits is not None and tilt_limits.upper is not None:
        rest_front = ctx.part_element_world_aabb(tray, elem="front_lip")
        with ctx.pose({tray_tilt: tilt_limits.upper}):
            tilted_front = ctx.part_element_world_aabb(tray, elem="front_lip")
        ctx.check(
            "tray front edge rises when tilted",
            rest_front is not None
            and tilted_front is not None
            and tilted_front[1][2] > rest_front[1][2] + 0.15,
            details=f"rest={rest_front}, tilted={tilted_front}",
        )

    if lever_limits is not None and lever_limits.upper is not None:
        rest_grip = ctx.part_element_world_aabb(lever, elem="lever_grip")
        with ctx.pose({lever_pivot: lever_limits.upper}):
            squeezed_grip = ctx.part_element_world_aabb(lever, elem="lever_grip")
        ctx.check(
            "release lever squeezes upward",
            rest_grip is not None
            and squeezed_grip is not None
            and squeezed_grip[1][2] > rest_grip[1][2] + 0.012,
            details=f"rest={rest_grip}, squeezed={squeezed_grip}",
        )

    return ctx.report()


object_model = build_object_model()
