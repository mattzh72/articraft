from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _aabb_center(aabb):
    if aabb is None:
        return None
    min_corner, max_corner = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(min_corner, max_corner))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_tripod_spotlight")

    frame = model.material("frame", rgba=(0.16, 0.17, 0.18, 1.0))
    body = model.material("body", rgba=(0.20, 0.21, 0.22, 1.0))
    aluminum = model.material("aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    lens = model.material("lens", rgba=(0.58, 0.64, 0.70, 0.55))

    crown = model.part("crown")
    crown.visual(
        Cylinder(radius=0.075, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=frame,
        name="crown_skirt",
    )
    crown.visual(
        Cylinder(radius=0.060, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=frame,
        name="crown_hub",
    )
    crown.visual(
        Box((0.090, 0.090, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=frame,
        name="lower_collar",
    )

    sleeve_height = 0.290
    sleeve_center_z = 0.215
    crown.visual(
        Box((0.068, 0.008, sleeve_height)),
        origin=Origin(xyz=(0.0, 0.030, sleeve_center_z)),
        material=frame,
        name="sleeve_front",
    )
    crown.visual(
        Box((0.068, 0.008, sleeve_height)),
        origin=Origin(xyz=(0.0, -0.030, sleeve_center_z)),
        material=frame,
        name="sleeve_rear",
    )
    crown.visual(
        Box((0.008, 0.052, sleeve_height)),
        origin=Origin(xyz=(0.030, 0.0, sleeve_center_z)),
        material=frame,
        name="sleeve_right",
    )
    crown.visual(
        Box((0.008, 0.052, sleeve_height)),
        origin=Origin(xyz=(-0.030, 0.0, sleeve_center_z)),
        material=frame,
        name="sleeve_left",
    )
    for block_name, xyz, size in (
        ("clamp_front", (0.0, 0.035, 0.385), (0.082, 0.012, 0.050)),
        ("clamp_rear", (0.0, -0.035, 0.385), (0.082, 0.012, 0.050)),
        ("clamp_right", (0.035, 0.0, 0.385), (0.012, 0.058, 0.050)),
        ("clamp_left", (-0.035, 0.0, 0.385), (0.012, 0.058, 0.050)),
    ):
        crown.visual(Box(size), origin=Origin(xyz=xyz), material=frame, name=block_name)
    crown.visual(
        Cylinder(radius=0.006, length=0.024),
        origin=Origin(xyz=(0.050, 0.0, 0.384), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="clamp_pin",
    )
    crown.visual(
        Cylinder(radius=0.014, length=0.018),
        origin=Origin(xyz=(0.068, 0.0, 0.384), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="clamp_knob",
    )

    leg_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    for index, angle in enumerate(leg_angles):
        c = math.cos(angle)
        s = math.sin(angle)
        hinge_xyz = (0.088 * c, 0.088 * s, -0.012)

        crown.visual(
            Box((0.0256, 0.032, 0.026)),
            origin=Origin(
                xyz=((0.068 * c), (0.068 * s), hinge_xyz[2]),
                rpy=(0.0, 0.0, angle),
            ),
            material=frame,
            name=f"hinge_block_{index}",
        )
        leg = model.part(f"leg_{index}")
        leg.visual(
            Cylinder(radius=0.0072, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=aluminum,
            name="hinge_barrel",
        )
        leg.visual(
            Box((0.042, 0.022, 0.022)),
            origin=Origin(xyz=(0.023, 0.0, -0.004)),
            material=frame,
            name="shoulder",
        )
        leg.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    [
                        (0.008, 0.0, -0.002),
                        (0.110, 0.0, -0.015),
                        (0.235, 0.0, -0.060),
                        (0.348, 0.0, -0.146),
                    ],
                    radius=0.012,
                    samples_per_segment=18,
                    radial_segments=18,
                    cap_ends=True,
                ),
                f"leg_tube_{index}",
            ),
            material=frame,
            name="tube",
        )
        leg.visual(
            Box((0.042, 0.050, 0.038)),
            origin=Origin(xyz=(0.348, 0.0, -0.157)),
            material=frame,
            name="fork_head",
        )
        leg.visual(
            Box((0.012, 0.008, 0.064)),
            origin=Origin(xyz=(0.356, 0.022, -0.205)),
            material=frame,
            name="fork_arm_0",
        )
        leg.visual(
            Box((0.012, 0.008, 0.064)),
            origin=Origin(xyz=(0.356, -0.022, -0.205)),
            material=frame,
            name="fork_arm_1",
        )
        leg.visual(
            Cylinder(radius=0.004, length=0.050),
            origin=Origin(xyz=(0.356, 0.0, -0.215), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=aluminum,
            name="axle",
        )
        leg.visual(
            Cylinder(radius=0.030, length=0.018),
            origin=Origin(xyz=(0.356, 0.0, -0.215), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name="wheel",
        )
        leg.visual(
            Cylinder(radius=0.014, length=0.022),
            origin=Origin(xyz=(0.356, 0.0, -0.215), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=aluminum,
            name="hub",
        )

        model.articulation(
            f"crown_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=crown,
            child=leg,
            origin=Origin(xyz=hinge_xyz, rpy=(0.0, 0.0, angle)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=30.0,
                velocity=1.2,
                lower=0.0,
                upper=1.05,
            ),
        )

    mast = model.part("mast")
    mast.visual(
        Box((0.046, 0.046, 0.920)),
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        material=aluminum,
        name="inner_tube",
    )
    mast.visual(
        Box((0.030, 0.004, 0.160)),
        origin=Origin(xyz=(0.0, 0.024, -0.140)),
        material=rubber,
        name="guide_front",
    )
    mast.visual(
        Box((0.030, 0.004, 0.160)),
        origin=Origin(xyz=(0.0, -0.024, -0.140)),
        material=rubber,
        name="guide_rear",
    )
    mast.visual(
        Box((0.004, 0.030, 0.160)),
        origin=Origin(xyz=(0.024, 0.0, -0.140)),
        material=rubber,
        name="guide_right",
    )
    mast.visual(
        Box((0.004, 0.030, 0.160)),
        origin=Origin(xyz=(-0.024, 0.0, -0.140)),
        material=rubber,
        name="guide_left",
    )
    mast.visual(
        Box((0.066, 0.066, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.690)),
        material=frame,
        name="top_cap",
    )
    mast.visual(
        Cylinder(radius=0.036, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.707)),
        material=frame,
        name="pan_seat",
    )
    mast.visual(
        Cylinder(radius=0.007, length=0.026),
        origin=Origin(xyz=(0.028, 0.0, 0.300), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body,
        name="lock_pin",
    )

    model.articulation(
        "crown_to_mast",
        ArticulationType.PRISMATIC,
        parent=crown,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.18,
            lower=0.0,
            upper=0.160,
        ),
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.050, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=frame,
        name="pan_base",
    )
    yoke.visual(
        Box((0.060, 0.050, 0.090)),
        origin=Origin(xyz=(0.0, 0.0, 0.061)),
        material=frame,
        name="pedestal",
    )
    yoke.visual(
        Box((0.110, 0.212, 0.018)),
        origin=Origin(xyz=(0.025, 0.0, 0.101)),
        material=frame,
        name="bridge",
    )
    yoke.visual(
        Box((0.110, 0.012, 0.132)),
        origin=Origin(xyz=(0.030, 0.100, 0.176)),
        material=frame,
        name="arm_0",
    )
    yoke.visual(
        Box((0.110, 0.012, 0.132)),
        origin=Origin(xyz=(0.030, -0.100, 0.176)),
        material=frame,
        name="arm_1",
    )

    model.articulation(
        "mast_to_yoke",
        ArticulationType.CONTINUOUS,
        parent=mast,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.714)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.5),
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.0, 0.085, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="trunnion_0",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.0, -0.085, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="trunnion_1",
    )
    head.visual(
        Cylinder(radius=0.082, length=0.180),
        origin=Origin(xyz=(0.035, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body,
        name="barrel",
    )
    head.visual(
        Cylinder(radius=0.090, length=0.028),
        origin=Origin(xyz=(0.139, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame,
        name="bezel",
    )
    head.visual(
        Cylinder(radius=0.078, length=0.006),
        origin=Origin(xyz=(0.156, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens,
        name="lens",
    )
    head.visual(
        Box((0.130, 0.160, 0.140)),
        origin=Origin(xyz=(-0.065, 0.0, 0.005)),
        material=body,
        name="rear_housing",
    )
    head.visual(
        Cylinder(radius=0.058, length=0.030),
        origin=Origin(xyz=(-0.145, 0.0, 0.005), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body,
        name="rear_cap",
    )
    for fin_index, fin_x in enumerate((-0.110, -0.086, -0.062)):
        head.visual(
            Box((0.012, 0.146, 0.028)),
            origin=Origin(xyz=(fin_x, 0.0, 0.080)),
            material=frame,
            name=f"fin_{fin_index}",
        )
    head.visual(
        Box((0.022, 0.014, 0.022)),
        origin=Origin(xyz=(-0.122, 0.046, 0.077)),
        material=frame,
        name="handle_mount_0",
    )
    head.visual(
        Box((0.022, 0.014, 0.022)),
        origin=Origin(xyz=(-0.122, -0.046, 0.077)),
        material=frame,
        name="handle_mount_1",
    )

    model.articulation(
        "yoke_to_head",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=head,
        origin=Origin(xyz=(0.040, 0.0, 0.198)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.5,
            lower=-0.90,
            upper=1.05,
        ),
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.007, length=0.014),
        origin=Origin(xyz=(0.0, 0.060, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="pivot_0",
    )
    handle.visual(
        Cylinder(radius=0.007, length=0.014),
        origin=Origin(xyz=(0.0, -0.060, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="pivot_1",
    )
    for side_index, side_y in enumerate((-0.060, 0.060)):
        handle.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    [
                        (0.0, side_y, 0.0),
                        (-0.012, side_y, 0.026),
                        (-0.018, side_y, 0.062),
                        (-0.012, side_y, 0.112),
                    ],
                    radius=0.006,
                    samples_per_segment=18,
                    radial_segments=18,
                    cap_ends=True,
                ),
                f"handle_side_{side_index}",
            ),
            material=frame,
            name=f"side_{side_index}",
        )
    handle.visual(
        Cylinder(radius=0.008, length=0.120),
        origin=Origin(xyz=(-0.012, 0.0, 0.112), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="crossbar",
    )
    handle.visual(
        Cylinder(radius=0.011, length=0.070),
        origin=Origin(xyz=(-0.012, 0.0, 0.112), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="grip",
    )

    model.articulation(
        "head_to_handle",
        ArticulationType.REVOLUTE,
        parent=head,
        child=handle,
        origin=Origin(xyz=(-0.122, 0.0, 0.077)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.0,
            lower=-1.0,
            upper=0.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    crown = object_model.get_part("crown")
    mast = object_model.get_part("mast")
    leg_0 = object_model.get_part("leg_0")
    head = object_model.get_part("head")
    handle = object_model.get_part("handle")

    mast_slide = object_model.get_articulation("crown_to_mast")
    leg_fold = object_model.get_articulation("crown_to_leg_0")
    pan = object_model.get_articulation("mast_to_yoke")
    tilt = object_model.get_articulation("yoke_to_head")
    handle_fold = object_model.get_articulation("head_to_handle")

    mast_limits = mast_slide.motion_limits
    leg_limits = leg_fold.motion_limits
    tilt_limits = tilt.motion_limits
    handle_limits = handle_fold.motion_limits

    mast_upper = 0.0 if mast_limits is None or mast_limits.upper is None else mast_limits.upper
    leg_upper = 0.0 if leg_limits is None or leg_limits.upper is None else leg_limits.upper
    tilt_upper = 0.0 if tilt_limits is None or tilt_limits.upper is None else tilt_limits.upper
    handle_lower = 0.0 if handle_limits is None or handle_limits.lower is None else handle_limits.lower

    ctx.expect_within(
        mast,
        crown,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="lower_collar",
        margin=0.0,
        name="mast stays centered in the crown collar",
    )

    rest_mast_pos = ctx.part_world_position(mast)
    with ctx.pose({mast_slide: mast_upper}):
        ctx.expect_within(
            mast,
            crown,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="lower_collar",
            margin=0.0,
            name="extended mast stays centered in the crown collar",
        )
        ctx.expect_overlap(
            mast,
            crown,
            axes="z",
            elem_a="inner_tube",
            elem_b="sleeve_front",
            min_overlap=0.080,
            name="mast keeps retained insertion at full extension",
        )
        extended_mast_pos = ctx.part_world_position(mast)

    ctx.check(
        "mast extends upward",
        rest_mast_pos is not None
        and extended_mast_pos is not None
        and extended_mast_pos[2] > rest_mast_pos[2] + 0.12,
        details=f"rest={rest_mast_pos}, extended={extended_mast_pos}",
    )

    rest_lens_center = _aabb_center(ctx.part_element_world_aabb(head, elem="lens"))
    with ctx.pose({pan: math.pi / 2.0}):
        panned_lens_center = _aabb_center(ctx.part_element_world_aabb(head, elem="lens"))
    ctx.check(
        "yoke pan swings the light sideways",
        rest_lens_center is not None
        and panned_lens_center is not None
        and panned_lens_center[1] > rest_lens_center[1] + 0.10,
        details=f"rest={rest_lens_center}, panned={panned_lens_center}",
    )

    with ctx.pose({tilt: tilt_upper}):
        tilted_lens_center = _aabb_center(ctx.part_element_world_aabb(head, elem="lens"))
    ctx.check(
        "head tilt lifts the beam",
        rest_lens_center is not None
        and tilted_lens_center is not None
        and tilted_lens_center[2] > rest_lens_center[2] + 0.08,
        details=f"rest={rest_lens_center}, tilted={tilted_lens_center}",
    )

    rest_handle_center = _aabb_center(ctx.part_element_world_aabb(handle, elem="crossbar"))
    with ctx.pose({handle_fold: handle_lower}):
        folded_handle_center = _aabb_center(ctx.part_element_world_aabb(handle, elem="crossbar"))
    ctx.check(
        "handle folds down toward the housing",
        rest_handle_center is not None
        and folded_handle_center is not None
        and folded_handle_center[2] < rest_handle_center[2] - 0.05,
        details=f"rest={rest_handle_center}, folded={folded_handle_center}",
    )

    rest_wheel_center = _aabb_center(ctx.part_element_world_aabb(leg_0, elem="wheel"))
    with ctx.pose({leg_fold: leg_upper}):
        folded_wheel_center = _aabb_center(ctx.part_element_world_aabb(leg_0, elem="wheel"))
    ctx.check(
        "tripod leg folds upward at the crown hinge",
        rest_wheel_center is not None
        and folded_wheel_center is not None
        and folded_wheel_center[2] > rest_wheel_center[2] + 0.12,
        details=f"rest={rest_wheel_center}, folded={folded_wheel_center}",
    )

    return ctx.report()


object_model = build_object_model()
