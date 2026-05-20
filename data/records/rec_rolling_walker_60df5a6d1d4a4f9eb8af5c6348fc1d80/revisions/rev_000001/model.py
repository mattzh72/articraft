from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
    mesh_from_geometry,
    wire_from_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_walker")

    frame_mat = model.material("frame_aluminum", rgba=(0.78, 0.80, 0.82, 1.0))
    hardware_mat = model.material("hardware_gray", rgba=(0.40, 0.43, 0.46, 1.0))
    fork_mat = model.material("fork_graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    tire_mat = model.material("tire_black", rgba=(0.10, 0.10, 0.10, 1.0))
    grip_mat = model.material("grip_black", rgba=(0.12, 0.12, 0.12, 1.0))
    tip_mat = model.material("rubber_tip", rgba=(0.08, 0.08, 0.08, 1.0))

    tube_radius = 0.0125
    frame_half_width = 0.255
    hinge_z = 0.31
    socket_y = 0.225
    socket_z = 0.196
    shoulder_y = 0.07
    shoulder_z = 0.79
    rear_foot_y = -0.29
    rear_foot_z = 0.04
    rear_shoulder_y = -0.05
    rear_shoulder_z = 0.83

    def tube_mesh(points, name, *, corner_radius=0.055):
        return mesh_from_geometry(
            wire_from_points(
                points,
                radius=tube_radius,
                radial_segments=18,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=corner_radius,
                corner_segments=10,
            ),
            name,
        )

    front_frame = model.part("front_frame")
    front_frame.visual(
        tube_mesh(
            [
                (-frame_half_width, 0.02, hinge_z + 0.05),
                (-frame_half_width, shoulder_y, shoulder_z),
                (-frame_half_width, 0.16, 0.54),
                (-frame_half_width, socket_y, 0.255),
                (-frame_half_width, socket_y, socket_z),
            ],
            "left_front_side_tube",
            corner_radius=0.06,
        ),
        material=frame_mat,
        name="left_side_tube",
    )
    front_frame.visual(
        tube_mesh(
            [
                (frame_half_width, 0.02, hinge_z + 0.05),
                (frame_half_width, shoulder_y, shoulder_z),
                (frame_half_width, 0.16, 0.54),
                (frame_half_width, socket_y, 0.255),
                (frame_half_width, socket_y, socket_z),
            ],
            "right_front_side_tube",
            corner_radius=0.06,
        ),
        material=frame_mat,
        name="right_side_tube",
    )
    front_frame.visual(
        Cylinder(radius=tube_radius, length=frame_half_width * 2.0),
        origin=Origin(xyz=(0.0, shoulder_y, shoulder_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_mat,
        name="top_rail",
    )
    front_frame.visual(
        Cylinder(radius=0.014, length=0.55),
        origin=Origin(xyz=(0.0, 0.0, hinge_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_mat,
        name="hinge_shaft",
    )
    front_frame.visual(
        Box((0.032, 0.026, 0.068)),
        origin=Origin(xyz=(-frame_half_width, 0.013, hinge_z + 0.034)),
        material=hardware_mat,
        name="left_hinge_lug",
    )
    front_frame.visual(
        Box((0.032, 0.026, 0.068)),
        origin=Origin(xyz=(frame_half_width, 0.013, hinge_z + 0.034)),
        material=hardware_mat,
        name="right_hinge_lug",
    )
    front_frame.visual(
        Cylinder(radius=tube_radius, length=0.485),
        origin=Origin(xyz=(0.0, socket_y, 0.226), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_mat,
        name="caster_bridge",
    )

    rear_frame = model.part("rear_frame")
    rear_frame.visual(
        tube_mesh(
            [
                (-frame_half_width, -0.035, 0.015),
                (-frame_half_width, rear_foot_y, rear_foot_z - hinge_z),
                (-frame_half_width, rear_shoulder_y, rear_shoulder_z - hinge_z),
            ],
            "left_rear_side_tube",
            corner_radius=0.065,
        ),
        material=frame_mat,
        name="left_side_tube",
    )
    rear_frame.visual(
        tube_mesh(
            [
                (frame_half_width, -0.035, 0.015),
                (frame_half_width, rear_foot_y, rear_foot_z - hinge_z),
                (frame_half_width, rear_shoulder_y, rear_shoulder_z - hinge_z),
            ],
            "right_rear_side_tube",
            corner_radius=0.065,
        ),
        material=frame_mat,
        name="right_side_tube",
    )
    rear_frame.visual(
        Cylinder(radius=tube_radius, length=frame_half_width * 2.0),
        origin=Origin(
            xyz=(0.0, rear_shoulder_y, rear_shoulder_z - hinge_z),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=frame_mat,
        name="top_crossbar",
    )
    rear_frame.visual(
        Box((0.55, 0.010, 0.050)),
        origin=Origin(xyz=(0.0, -0.023, 0.0)),
        material=hardware_mat,
        name="hinge_plate",
    )
    rear_frame.visual(
        Cylinder(radius=0.018, length=0.04),
        origin=Origin(xyz=(-0.16, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_mat,
        name="left_hinge_sleeve",
    )
    rear_frame.visual(
        Cylinder(radius=0.018, length=0.04),
        origin=Origin(xyz=(0.16, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_mat,
        name="right_hinge_sleeve",
    )
    rear_frame.visual(
        Cylinder(radius=tube_radius, length=0.16),
        origin=Origin(
            xyz=(-frame_half_width, 0.03, rear_shoulder_z - hinge_z + 0.005),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=frame_mat,
        name="left_handle_stub",
    )
    rear_frame.visual(
        Cylinder(radius=tube_radius, length=0.16),
        origin=Origin(
            xyz=(frame_half_width, 0.03, rear_shoulder_z - hinge_z + 0.005),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=frame_mat,
        name="right_handle_stub",
    )
    rear_frame.visual(
        Cylinder(radius=0.018, length=0.085),
        origin=Origin(
            xyz=(-frame_half_width, -0.002, rear_shoulder_z - hinge_z + 0.005),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=grip_mat,
        name="left_grip",
    )
    rear_frame.visual(
        Cylinder(radius=0.018, length=0.085),
        origin=Origin(
            xyz=(frame_half_width, -0.002, rear_shoulder_z - hinge_z + 0.005),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=grip_mat,
        name="right_grip",
    )
    rear_frame.visual(
        Cylinder(radius=0.020, length=0.045),
        origin=Origin(xyz=(-frame_half_width, rear_foot_y, rear_foot_z - hinge_z - 0.0175)),
        material=tip_mat,
        name="left_rear_tip",
    )
    rear_frame.visual(
        Cylinder(radius=0.020, length=0.045),
        origin=Origin(xyz=(frame_half_width, rear_foot_y, rear_foot_z - hinge_z - 0.0175)),
        material=tip_mat,
        name="right_rear_tip",
    )

    model.articulation(
        "front_to_rear_fold",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=rear_frame,
        origin=Origin(xyz=(0.0, 0.0, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=2.0, lower=0.0, upper=1.10),
    )

    def add_caster(side: str, x_sign: float) -> None:
        fork = model.part(f"{side}_caster_fork")
        fork.visual(
            Cylinder(radius=0.0105, length=0.07),
            origin=Origin(xyz=(0.0, 0.0, 0.005)),
            material=fork_mat,
            name="stem",
        )
        fork.visual(
            Box((0.052, 0.024, 0.016)),
            origin=Origin(xyz=(0.0, 0.0, -0.038)),
            material=fork_mat,
            name="crown",
        )
        fork.visual(
            Box((0.007, 0.022, 0.086)),
            origin=Origin(xyz=(-0.0175, 0.0, -0.076)),
            material=fork_mat,
            name="left_yoke",
        )
        fork.visual(
            Box((0.007, 0.022, 0.086)),
            origin=Origin(xyz=(0.0175, 0.0, -0.076)),
            material=fork_mat,
            name="right_yoke",
        )

        model.articulation(
            f"{side}_caster_swivel",
            ArticulationType.CONTINUOUS,
            parent=front_frame,
            child=fork,
            origin=Origin(xyz=(x_sign * frame_half_width, socket_y, socket_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=8.0),
        )

        wheel = model.part(f"{side}_front_wheel")
        wheel.visual(
            Cylinder(radius=0.076, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=tire_mat,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.031, length=0.028),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=hardware_mat,
            name="hub",
        )

        model.articulation(
            f"{side}_wheel_roll",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.119)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=20.0),
        )

    add_caster("left", -1.0)
    add_caster("right", 1.0)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front_frame = object_model.get_part("front_frame")
    rear_frame = object_model.get_part("rear_frame")
    left_fork = object_model.get_part("left_caster_fork")
    right_fork = object_model.get_part("right_caster_fork")
    left_wheel = object_model.get_part("left_front_wheel")
    right_wheel = object_model.get_part("right_front_wheel")

    fold = object_model.get_articulation("front_to_rear_fold")
    left_swivel = object_model.get_articulation("left_caster_swivel")
    right_swivel = object_model.get_articulation("right_caster_swivel")
    left_roll = object_model.get_articulation("left_wheel_roll")
    right_roll = object_model.get_articulation("right_wheel_roll")

    hinge_shaft = front_frame.get_visual("hinge_shaft")
    left_front_leg = front_frame.get_visual("left_side_tube")
    right_front_leg = front_frame.get_visual("right_side_tube")
    left_hinge_sleeve = rear_frame.get_visual("left_hinge_sleeve")
    right_hinge_sleeve = rear_frame.get_visual("right_hinge_sleeve")
    top_crossbar = rear_frame.get_visual("top_crossbar")
    left_rear_tip = rear_frame.get_visual("left_rear_tip")
    right_rear_tip = rear_frame.get_visual("right_rear_tip")
    left_stem = left_fork.get_visual("stem")
    right_stem = right_fork.get_visual("stem")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.allow_overlap(
        rear_frame,
        front_frame,
        reason="rear frame sleeves rotate on the transverse hinge shaft",
        elem_a=left_hinge_sleeve,
        elem_b=hinge_shaft,
    )
    ctx.allow_overlap(
        rear_frame,
        front_frame,
        reason="rear frame sleeves rotate on the transverse hinge shaft",
        elem_a=right_hinge_sleeve,
        elem_b=hinge_shaft,
    )
    ctx.allow_overlap(
        left_fork,
        front_frame,
        reason="left caster stem telescopes into the formed front leg receiver",
        elem_a=left_stem,
        elem_b=left_front_leg,
    )
    ctx.allow_overlap(
        right_fork,
        front_frame,
        reason="right caster stem telescopes into the formed front leg receiver",
        elem_a=right_stem,
        elem_b=right_front_leg,
    )

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
        "fold axis is transverse",
        fold.axis == (-1.0, 0.0, 0.0),
        details=f"unexpected fold axis: {fold.axis}",
    )
    ctx.check(
        "caster swivel axes are vertical",
        left_swivel.axis == (0.0, 0.0, 1.0) and right_swivel.axis == (0.0, 0.0, 1.0),
        details=f"left={left_swivel.axis}, right={right_swivel.axis}",
    )
    ctx.check(
        "wheel roll axes are lateral",
        left_roll.axis == (1.0, 0.0, 0.0) and right_roll.axis == (1.0, 0.0, 0.0),
        details=f"left={left_roll.axis}, right={right_roll.axis}",
    )

    with ctx.pose({fold: 0.0}):
        ctx.expect_contact(
            rear_frame,
            front_frame,
            elem_a=left_hinge_sleeve,
            elem_b=hinge_shaft,
            name="left hinge sleeve mounted on shaft",
        )
        ctx.expect_contact(
            rear_frame,
            front_frame,
            elem_a=right_hinge_sleeve,
            elem_b=hinge_shaft,
            name="right hinge sleeve mounted on shaft",
        )
        ctx.expect_contact(left_fork, left_wheel, name="left wheel captured by fork")
        ctx.expect_contact(right_fork, right_wheel, name="right wheel captured by fork")
        ctx.expect_gap(
            left_fork,
            rear_frame,
            axis="y",
            positive_elem=left_stem,
            negative_elem=left_rear_tip,
            min_gap=0.45,
            name="left side open stance depth",
        )
        ctx.expect_gap(
            right_fork,
            rear_frame,
            axis="y",
            positive_elem=right_stem,
            negative_elem=right_rear_tip,
            min_gap=0.45,
            name="right side open stance depth",
        )

    open_crossbar = ctx.part_element_world_aabb(rear_frame, elem=top_crossbar)
    open_tip = ctx.part_element_world_aabb(rear_frame, elem=left_rear_tip)
    with ctx.pose({fold: 1.0}):
        folded_crossbar = ctx.part_element_world_aabb(rear_frame, elem=top_crossbar)
        folded_tip = ctx.part_element_world_aabb(rear_frame, elem=left_rear_tip)

    ctx.check(
        "rear frame folds forward",
        open_crossbar is not None
        and folded_crossbar is not None
        and folded_crossbar[0][1] > open_crossbar[1][1] + 0.12,
        details=f"open={open_crossbar}, folded={folded_crossbar}",
    )
    ctx.check(
        "rear tips lift when folding",
        open_tip is not None and folded_tip is not None and folded_tip[0][2] > open_tip[1][2] + 0.04,
        details=f"open={open_tip}, folded={folded_tip}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
