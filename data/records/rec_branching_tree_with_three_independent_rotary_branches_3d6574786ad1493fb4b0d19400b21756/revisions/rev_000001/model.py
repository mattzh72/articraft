from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


SPINE_SIZE = (0.09, 0.07, 0.42)
LOWER_JOINT_ORIGIN = (0.0, -0.118, 0.108)
FORWARD_JOINT_ORIGIN = (0.082, 0.0, 0.226)
UPPER_JOINT_ORIGIN = (0.0, 0.128, 0.318)


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _x_cylinder(
    radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return (
        cq.Workplane("YZ", origin=(center[0] - (length / 2.0), center[1], center[2]))
        .circle(radius)
        .extrude(length)
    )


def _y_cylinder(
    radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return (
        cq.Workplane("XY", origin=(0.0, 0.0, -(length / 2.0)))
        .circle(radius)
        .extrude(length)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate(center)
    )


def _z_cylinder(
    radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return (
        cq.Workplane("XY", origin=(center[0], center[1], center[2] - (length / 2.0)))
        .circle(radius)
        .extrude(length)
    )


def _spine_column() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(*SPINE_SIZE)
        .edges("|Z")
        .fillet(0.008)
        .edges(">Z")
        .fillet(0.003)
        .translate((0.0, 0.0, SPINE_SIZE[2] / 2.0))
    )


def _lower_housing() -> cq.Workplane:
    strap = _box((0.086, 0.038, 0.088), (0.0, -0.054, 0.108))
    trunnion = _x_cylinder(0.024, 0.074, (0.0, -0.094, 0.108))
    rear_web = _box((0.052, 0.020, 0.056), (0.0, -0.074, 0.108))
    return strap.union(trunnion).union(rear_web)


def _forward_housing() -> cq.Workplane:
    collar = _box((0.050, 0.078, 0.058), (0.070, 0.0, 0.175))
    pedestal = _box((0.032, 0.052, 0.028), (0.082, 0.0, 0.204))
    stub = _z_cylinder(0.024, 0.022, (0.082, 0.0, 0.215))
    return collar.union(pedestal).union(stub)


def _upper_housing() -> cq.Workplane:
    strap = _box((0.080, 0.044, 0.072), (0.0, 0.052, 0.318))
    shaft_boss = _y_cylinder(0.022, 0.060, (0.0, 0.098, 0.318))
    lower_web = _box((0.046, 0.032, 0.048), (0.0, 0.061, 0.300))
    return strap.union(shaft_boss).union(lower_web)


def _lower_pivot() -> cq.Workplane:
    return _x_cylinder(0.018, 0.062, (0.0, -0.018, 0.0))


def _lower_beam() -> cq.Workplane:
    beam = _box((0.034, 0.150, 0.026), (0.0, -0.095, 0.0))
    rib = _box((0.024, 0.090, 0.040), (0.0, -0.092, -0.012))
    return beam.union(rib)


def _slot_pad() -> cq.Workplane:
    neck = _box((0.050, 0.024, 0.036), (0.0, -0.174, 0.0))
    plate = _box((0.092, 0.014, 0.064), (0.0, -0.194, 0.0))
    pad = neck.union(plate)
    for x_offset in (-0.026, 0.026):
        slot = (
            cq.Workplane("XZ", origin=(x_offset, -0.203, 0.0))
            .slot2D(0.024, 0.010, angle=90)
            .extrude(0.018)
        )
        pad = pad.cut(slot)
    return pad


def _forward_base() -> cq.Workplane:
    return _z_cylinder(0.028, 0.018, (0.0, 0.0, 0.009))


def _forward_beam() -> cq.Workplane:
    root_block = _box((0.040, 0.056, 0.020), (0.020, 0.0, 0.012))
    main_beam = _box((0.148, 0.034, 0.024), (0.094, 0.0, 0.020))
    rib_profile = (
        cq.Workplane("XZ", origin=(0.008, -0.009, 0.0))
        .polyline([(0.0, 0.0), (0.076, 0.0), (0.0, 0.028)])
        .close()
        .extrude(0.018)
    )
    return root_block.union(main_beam).union(rib_profile)


def _round_pad() -> cq.Workplane:
    neck = _box((0.038, 0.028, 0.030), (0.177, 0.0, 0.022))
    flange = _x_cylinder(0.042, 0.014, (0.198, 0.0, 0.022))
    pad = neck.union(flange)
    pad = pad.cut(_x_cylinder(0.011, 0.018, (0.198, 0.0, 0.022)))
    for angle in (0.0, (2.0 * math.pi) / 3.0, (4.0 * math.pi) / 3.0):
        y = 0.026 * math.cos(angle)
        z = 0.022 + (0.026 * math.sin(angle))
        pad = pad.cut(_x_cylinder(0.004, 0.018, (0.198, y, z)))
    return pad


def _upper_shaft_base() -> cq.Workplane:
    return _y_cylinder(0.018, 0.026, (0.0, 0.013, 0.0))


def _upper_shaft() -> cq.Workplane:
    return _y_cylinder(0.014, 0.144, (0.0, 0.098, 0.0))


def _fork_pad() -> cq.Workplane:
    riser = _box((0.020, 0.030, 0.052), (0.0, 0.158, 0.026))
    bridge = _box((0.030, 0.022, 0.024), (0.0, 0.182, 0.052))
    jaw_left = _box((0.014, 0.026, 0.056), (-0.025, 0.194, 0.052))
    jaw_right = _box((0.014, 0.026, 0.056), (0.025, 0.194, 0.052))
    pad = riser.union(bridge).union(jaw_left).union(jaw_right)
    pad = pad.cut(_x_cylinder(0.004, 0.020, (-0.025, 0.194, 0.052)))
    pad = pad.cut(_x_cylinder(0.004, 0.020, (0.025, 0.194, 0.052)))
    return pad


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inspection_fixture_head")

    frame_mat = model.material("frame_graphite", rgba=(0.24, 0.26, 0.29, 1.0))
    arm_mat = model.material("machined_aluminum", rgba=(0.63, 0.66, 0.70, 1.0))
    pad_mat = model.material("tool_steel", rgba=(0.72, 0.74, 0.76, 1.0))

    spine = model.part("spine")
    spine.visual(
        mesh_from_cadquery(_spine_column(), "spine_column"),
        material=frame_mat,
        name="spine_column",
    )
    spine.visual(
        mesh_from_cadquery(_lower_housing(), "lower_housing"),
        material=frame_mat,
        name="lower_housing",
    )
    spine.visual(
        mesh_from_cadquery(_forward_housing(), "forward_housing"),
        material=frame_mat,
        name="forward_housing",
    )
    spine.visual(
        mesh_from_cadquery(_upper_housing(), "upper_housing"),
        material=frame_mat,
        name="upper_housing",
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        mesh_from_cadquery(_lower_pivot(), "lower_pivot"),
        material=arm_mat,
        name="lower_pivot",
    )
    lower_arm.visual(
        mesh_from_cadquery(_lower_beam(), "lower_beam"),
        material=arm_mat,
        name="lower_beam",
    )
    lower_arm.visual(
        mesh_from_cadquery(_slot_pad(), "slot_pad"),
        material=pad_mat,
        name="slot_pad",
    )

    forward_arm = model.part("forward_arm")
    forward_arm.visual(
        mesh_from_cadquery(_forward_base(), "forward_base"),
        material=arm_mat,
        name="forward_base",
    )
    forward_arm.visual(
        mesh_from_cadquery(_forward_beam(), "forward_beam"),
        material=arm_mat,
        name="forward_beam",
    )
    forward_arm.visual(
        mesh_from_cadquery(_round_pad(), "round_pad"),
        material=pad_mat,
        name="round_pad",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(_upper_shaft_base(), "upper_shaft_base"),
        material=arm_mat,
        name="upper_shaft_base",
    )
    upper_arm.visual(
        mesh_from_cadquery(_upper_shaft(), "upper_shaft"),
        material=arm_mat,
        name="upper_shaft",
    )
    upper_arm.visual(
        mesh_from_cadquery(_fork_pad(), "fork_pad"),
        material=pad_mat,
        name="fork_pad",
    )

    model.articulation(
        "spine_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=lower_arm,
        origin=Origin(xyz=LOWER_JOINT_ORIGIN),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=-0.75,
            upper=0.55,
        ),
    )
    model.articulation(
        "spine_to_forward_arm",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=forward_arm,
        origin=Origin(xyz=FORWARD_JOINT_ORIGIN),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.6,
            lower=-1.05,
            upper=1.05,
        ),
    )
    model.articulation(
        "spine_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=upper_arm,
        origin=Origin(xyz=UPPER_JOINT_ORIGIN),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.4,
            lower=-1.35,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    spine = object_model.get_part("spine")
    lower_arm = object_model.get_part("lower_arm")
    forward_arm = object_model.get_part("forward_arm")
    upper_arm = object_model.get_part("upper_arm")

    lower_joint = object_model.get_articulation("spine_to_lower_arm")
    forward_joint = object_model.get_articulation("spine_to_forward_arm")
    upper_joint = object_model.get_articulation("spine_to_upper_arm")

    def _aabb_center(aabb):
        (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
        return (
            (min_x + max_x) / 2.0,
            (min_y + max_y) / 2.0,
            (min_z + max_z) / 2.0,
        )

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "part set present",
        {part.name for part in object_model.parts}
        == {"spine", "lower_arm", "forward_arm", "upper_arm"},
        details=f"found {[part.name for part in object_model.parts]}",
    )
    ctx.check(
        "fixture visuals present",
        {"spine_column", "lower_housing", "forward_housing", "upper_housing"}
        .issubset({visual.name for visual in spine.visuals})
        and {"lower_pivot", "lower_beam", "slot_pad"}.issubset(
            {visual.name for visual in lower_arm.visuals}
        )
        and {"forward_base", "forward_beam", "round_pad"}.issubset(
            {visual.name for visual in forward_arm.visuals}
        )
        and {"upper_shaft_base", "upper_shaft", "fork_pad"}.issubset(
            {visual.name for visual in upper_arm.visuals}
        ),
        details="missing one or more named visuals used to define housings or pads",
    )

    ctx.check(
        "joint axes match branch mechanisms",
        lower_joint.axis == (1.0, 0.0, 0.0)
        and forward_joint.axis == (0.0, 0.0, 1.0)
        and upper_joint.axis == (0.0, 1.0, 0.0),
        details=(
            f"lower={lower_joint.axis}, forward={forward_joint.axis}, "
            f"upper={upper_joint.axis}"
        ),
    )
    ctx.check(
        "joint limits are bounded and usable",
        lower_joint.motion_limits is not None
        and forward_joint.motion_limits is not None
        and upper_joint.motion_limits is not None
        and lower_joint.motion_limits.lower < 0.0 < lower_joint.motion_limits.upper
        and forward_joint.motion_limits.lower < 0.0 < forward_joint.motion_limits.upper
        and upper_joint.motion_limits.lower < 0.0 < upper_joint.motion_limits.upper,
        details="one or more joints lack realistic bidirectional working travel",
    )

    ctx.expect_contact(
        lower_arm,
        spine,
        elem_a="lower_pivot",
        elem_b="lower_housing",
        name="lower arm seats on trunnion housing",
    )
    ctx.expect_overlap(
        lower_arm,
        spine,
        axes="xz",
        elem_a="lower_pivot",
        elem_b="lower_housing",
        min_overlap=0.035,
        name="lower pivot aligns with lower housing",
    )
    ctx.expect_contact(
        forward_arm,
        spine,
        elem_a="forward_base",
        elem_b="forward_housing",
        name="forward arm seats on vertical stub housing",
    )
    ctx.expect_overlap(
        forward_arm,
        spine,
        axes="xy",
        elem_a="forward_base",
        elem_b="forward_housing",
        min_overlap=0.040,
        name="forward base overlaps its stub housing footprint",
    )
    ctx.expect_contact(
        upper_arm,
        spine,
        elem_a="upper_shaft_base",
        elem_b="upper_housing",
        name="upper arm seats on longitudinal shaft housing",
    )
    ctx.expect_overlap(
        upper_arm,
        spine,
        axes="xz",
        elem_a="upper_shaft_base",
        elem_b="upper_housing",
        min_overlap=0.028,
        name="upper shaft aligns with upper housing",
    )

    slot_center = _aabb_center(ctx.part_element_world_aabb(lower_arm, elem="slot_pad"))
    round_center = _aabb_center(ctx.part_element_world_aabb(forward_arm, elem="round_pad"))
    fork_center = _aabb_center(ctx.part_element_world_aabb(upper_arm, elem="fork_pad"))
    ctx.check(
        "pads occupy distinct low forward and high branches",
        slot_center[1] < -0.17
        and round_center[0] > 0.18
        and fork_center[1] > 0.16
        and fork_center[2] > round_center[2] + 0.06,
        details=(
            f"slot_center={slot_center}, round_center={round_center}, "
            f"fork_center={fork_center}"
        ),
    )

    with ctx.pose({lower_joint: -0.45}):
        lowered_slot_center = _aabb_center(
            ctx.part_element_world_aabb(lower_arm, elem="slot_pad")
        )
    ctx.check(
        "lower arm pitches about transverse axis",
        lowered_slot_center[2] > slot_center[2] + 0.03,
        details=f"rest={slot_center}, posed={lowered_slot_center}",
    )

    with ctx.pose({forward_joint: 0.80}):
        swept_round_center = _aabb_center(
            ctx.part_element_world_aabb(forward_arm, elem="round_pad")
        )
    ctx.check(
        "forward arm yaws about vertical stub axis",
        swept_round_center[1] > round_center[1] + 0.12
        and swept_round_center[0] < round_center[0] - 0.02,
        details=f"rest={round_center}, posed={swept_round_center}",
    )

    with ctx.pose({upper_joint: 1.00}):
        rolled_fork_center = _aabb_center(
            ctx.part_element_world_aabb(upper_arm, elem="fork_pad")
        )
    ctx.check(
        "upper arm rolls about longitudinal shaft axis",
        abs(rolled_fork_center[0] - fork_center[0]) > 0.03
        and abs(rolled_fork_center[2] - fork_center[2]) > 0.01,
        details=f"rest={fork_center}, posed={rolled_fork_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
