from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _build_side_frame_geometry(side_sign: float):
    tube_radius = 0.014
    frame = tube_from_spline_points(
        [
            (0.00, side_sign * 0.055, -0.34),
            (0.00, side_sign * 0.055, -0.06),
            (0.00, side_sign * 0.060, 0.26),
            (-0.08, side_sign * 0.095, 0.31),
            (-0.24, side_sign * 0.145, 0.35),
            (-0.36, side_sign * 0.160, 0.38),
            (-0.40, side_sign * 0.170, 0.18),
            (-0.40, side_sign * 0.185, -0.45),
        ],
        radius=tube_radius,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )
    frame.merge(
        CylinderGeometry(0.015, 0.11, radial_segments=24).translate(
            0.0,
            side_sign * 0.055,
            0.020,
        )
    )
    frame.merge(
        CylinderGeometry(0.012, 0.040, radial_segments=20)
        .rotate_x(pi / 2.0)
        .translate(0.0, side_sign * 0.040, 0.000)
    )
    frame.merge(
        CylinderGeometry(0.011, 0.040, radial_segments=20)
        .rotate_x(pi / 2.0)
        .translate(0.0, side_sign * 0.045, 0.160)
    )
    frame.merge(
        CylinderGeometry(0.016, 0.050, radial_segments=24).translate(
            0.0,
            side_sign * 0.055,
            -0.310,
        )
    )
    frame.merge(
        CylinderGeometry(0.011, 0.260, radial_segments=20).translate(
            -0.40,
            side_sign * 0.170,
            0.240,
        )
    )
    frame.merge(
        BoxGeometry((0.036, 0.038, 0.040)).translate(
            -0.40,
            side_sign * 0.172,
            0.080,
        )
    )
    return frame


def _add_side_frame(
    model: ArticulatedObject,
    *,
    name: str,
    side_sign: float,
    frame_material,
    grip_material,
    datum_material,
    accent_material,
    rubber_material,
):
    frame = model.part(name)
    frame.visual(
        Cylinder(radius=0.014, length=0.640),
        origin=Origin(xyz=(0.0, side_sign * 0.055, -0.015)),
        material=frame_material,
        name="front_upright",
    )
    frame.visual(
        Cylinder(radius=0.014, length=0.660),
        origin=Origin(xyz=(-0.400, side_sign * 0.185, -0.115)),
        material=frame_material,
        name="rear_leg",
    )
    frame.visual(
        _mesh(
            f"{name}_top_rail",
            tube_from_spline_points(
                [
                    (0.0, side_sign * 0.055, 0.250),
                    (-0.10, side_sign * 0.090, 0.300),
                    (-0.24, side_sign * 0.145, 0.345),
                    (-0.36, side_sign * 0.168, 0.375),
                    (-0.40, side_sign * 0.170, 0.385),
                ],
                radius=0.014,
                samples_per_segment=12,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=frame_material,
        name="top_rail",
    )
    frame.visual(
        _mesh(
            f"{name}_lower_rail",
            tube_from_spline_points(
                [
                    (0.0, side_sign * 0.055, -0.080),
                    (-0.12, side_sign * 0.090, -0.030),
                    (-0.26, side_sign * 0.145, 0.020),
                    (-0.40, side_sign * 0.175, 0.085),
                ],
                radius=0.012,
                samples_per_segment=10,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=frame_material,
        name="lower_rail",
    )
    frame.visual(
        _mesh(
            f"{name}_rear_brace",
            tube_from_spline_points(
                [
                    (-0.18, side_sign * 0.120, 0.305),
                    (-0.28, side_sign * 0.150, 0.205),
                    (-0.36, side_sign * 0.170, 0.085),
                    (-0.40, side_sign * 0.185, -0.035),
                ],
                radius=0.011,
                samples_per_segment=10,
                radial_segments=16,
                cap_ends=True,
            ),
        ),
        material=frame_material,
        name="rear_brace",
    )
    frame.visual(
        Box((0.020, 0.060, 0.220)),
        origin=Origin(xyz=(0.0, side_sign * 0.028, 0.000)),
        material=frame_material,
        name="hinge_spine",
    )
    frame.visual(
        Box((0.020, 0.060, 0.018)),
        origin=Origin(xyz=(0.0, side_sign * 0.028, -0.105)),
        material=frame_material,
        name="lower_hinge_web",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.099)),
        material=frame_material,
        name="lower_hinge_pad",
    )
    frame.visual(
        Box((0.020, 0.060, 0.018)),
        origin=Origin(xyz=(0.0, side_sign * 0.028, 0.105)),
        material=frame_material,
        name="upper_hinge_web",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.111)),
        material=frame_material,
        name="upper_hinge_pad",
    )
    frame.visual(
        Cylinder(radius=0.016, length=0.012),
        origin=Origin(xyz=(0.0, side_sign * 0.055, -0.329)),
        material=frame_material,
        name="caster_bearing_housing",
    )
    frame.visual(
        Box((0.020, 0.040, 0.018)),
        origin=Origin(xyz=(0.0, side_sign * 0.055, -0.320)),
        material=frame_material,
        name="caster_mount_block",
    )
    frame.visual(
        Box((0.026, 0.022, 0.030)),
        origin=Origin(xyz=(-0.316, side_sign * 0.178, 0.392)),
        material=frame_material,
        name="brake_mount",
    )
    frame.visual(
        Box((0.085, 0.026, 0.012)),
        origin=Origin(xyz=(-0.355, side_sign * 0.158, 0.385)),
        material=datum_material,
        name="alignment_pad",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.110),
        origin=Origin(
            xyz=(-0.355, side_sign * 0.160, 0.400),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=grip_material,
        name="hand_grip",
    )
    frame.visual(
        Box((0.004, 0.010, 0.160)),
        origin=Origin(xyz=(-0.386, side_sign * 0.184, 0.200)),
        material=accent_material,
        name="index_strip",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.070),
        origin=Origin(xyz=(-0.400, side_sign * 0.185, -0.480)),
        material=rubber_material,
        name="rear_foot",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.44, 0.22, 0.98)),
        mass=2.8,
        origin=Origin(xyz=(-0.20, side_sign * 0.11, -0.03)),
    )
    return frame


def _add_caster_fork(
    model: ArticulatedObject,
    *,
    name: str,
    frame_material,
):
    fork = model.part(name)
    fork.visual(
        Cylinder(radius=0.009, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=frame_material,
        name="stem",
    )
    fork.visual(
        Box((0.046, 0.052, 0.012)),
        origin=Origin(xyz=(-0.018, 0.0, -0.028)),
        material=frame_material,
        name="crown",
    )
    fork.visual(
        Box((0.036, 0.004, 0.090)),
        origin=Origin(xyz=(-0.020, 0.018, -0.079)),
        material=frame_material,
        name="left_arm",
    )
    fork.visual(
        Box((0.036, 0.004, 0.090)),
        origin=Origin(xyz=(-0.020, -0.018, -0.079)),
        material=frame_material,
        name="right_arm",
    )
    fork.inertial = Inertial.from_geometry(
        Box((0.060, 0.040, 0.140)),
        mass=0.28,
        origin=Origin(xyz=(-0.015, 0.0, -0.070)),
    )
    return fork


def _add_caster_wheel(
    model: ArticulatedObject,
    *,
    name: str,
    tire_material,
    hub_material,
):
    wheel = model.part(name)
    wheel.visual(
        Cylinder(radius=0.065, length=0.020),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=tire_material,
        name="tire",
    )
    wheel.visual(
        Cylinder(radius=0.012, length=0.032),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=hub_material,
        name="hub_core",
    )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.065, length=0.020),
        mass=0.34,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    return wheel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_rolling_walker")

    frame_satin = model.material("frame_satin", rgba=(0.79, 0.81, 0.83, 1.0))
    grip_black = model.material("grip_black", rgba=(0.10, 0.10, 0.11, 1.0))
    datum_blue = model.material("datum_blue", rgba=(0.72, 0.82, 0.88, 1.0))
    accent_orange = model.material("accent_orange", rgba=(0.92, 0.48, 0.12, 1.0))
    tire_black = model.material("tire_black", rgba=(0.09, 0.09, 0.10, 1.0))
    hub_gray = model.material("hub_gray", rgba=(0.50, 0.53, 0.56, 1.0))
    foot_black = model.material("foot_black", rgba=(0.08, 0.08, 0.08, 1.0))

    center_bridge = model.part("center_bridge")
    center_bridge.visual(
        Box((0.040, 0.120, 0.040)),
        origin=Origin(xyz=(0.258, 0.0, 0.630)),
        material=datum_blue,
        name="upper_datum_bar",
    )
    center_bridge.visual(
        Box((0.038, 0.110, 0.035)),
        origin=Origin(xyz=(0.252, 0.0, 0.430)),
        material=datum_blue,
        name="lower_datum_bar",
    )
    center_bridge.visual(
        Box((0.018, 0.060, 0.310)),
        origin=Origin(xyz=(0.236, 0.0, 0.510)),
        material=frame_satin,
        name="datum_spine",
    )
    for side_sign, side_name in ((1.0, "left"), (-1.0, "right")):
        center_bridge.visual(
            Box((0.050, 0.050, 0.018)),
            origin=Origin(xyz=(0.219, side_sign * 0.028, 0.405)),
            material=frame_satin,
            name=f"{side_name}_lower_hinge_connector",
        )
        center_bridge.visual(
            Box((0.016, 0.060, 0.018)),
            origin=Origin(xyz=(0.202, side_sign * 0.057, 0.405)),
            material=frame_satin,
            name=f"{side_name}_lower_hinge_web",
        )
        center_bridge.visual(
            Cylinder(radius=0.010, length=0.085),
            origin=Origin(xyz=(0.202, side_sign * 0.084, 0.405)),
            material=frame_satin,
            name=f"{side_name}_lower_hinge",
        )
        center_bridge.visual(
            Box((0.050, 0.050, 0.018)),
            origin=Origin(xyz=(0.219, side_sign * 0.028, 0.615)),
            material=frame_satin,
            name=f"{side_name}_upper_hinge_connector",
        )
        center_bridge.visual(
            Box((0.016, 0.060, 0.018)),
            origin=Origin(xyz=(0.202, side_sign * 0.057, 0.615)),
            material=frame_satin,
            name=f"{side_name}_upper_hinge_web",
        )
        center_bridge.visual(
            Cylinder(radius=0.010, length=0.085),
            origin=Origin(xyz=(0.202, side_sign * 0.084, 0.615)),
            material=frame_satin,
            name=f"{side_name}_upper_hinge",
        )
    center_bridge.inertial = Inertial.from_geometry(
        Box((0.090, 0.150, 0.360)),
        mass=1.8,
        origin=Origin(xyz=(0.235, 0.0, 0.510)),
    )

    left_frame = _add_side_frame(
        model,
        name="left_side_frame",
        side_sign=1.0,
        frame_material=frame_satin,
        grip_material=grip_black,
        datum_material=datum_blue,
        accent_material=accent_orange,
        rubber_material=foot_black,
    )
    right_frame = _add_side_frame(
        model,
        name="right_side_frame",
        side_sign=-1.0,
        frame_material=frame_satin,
        grip_material=grip_black,
        datum_material=datum_blue,
        accent_material=accent_orange,
        rubber_material=foot_black,
    )

    left_fork = _add_caster_fork(model, name="left_front_caster_fork", frame_material=frame_satin)
    right_fork = _add_caster_fork(model, name="right_front_caster_fork", frame_material=frame_satin)
    left_wheel = _add_caster_wheel(
        model,
        name="left_front_caster_wheel",
        tire_material=tire_black,
        hub_material=hub_gray,
    )
    right_wheel = _add_caster_wheel(
        model,
        name="right_front_caster_wheel",
        tire_material=tire_black,
        hub_material=hub_gray,
    )

    left_fold = model.articulation(
        "center_to_left_frame",
        ArticulationType.REVOLUTE,
        parent=center_bridge,
        child=left_frame,
        origin=Origin(xyz=(0.200, 0.110, 0.510)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=0.0,
            upper=0.62,
        ),
    )
    right_fold = model.articulation(
        "center_to_right_frame",
        ArticulationType.REVOLUTE,
        parent=center_bridge,
        child=right_frame,
        origin=Origin(xyz=(0.200, -0.110, 0.510)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=0.0,
            upper=0.62,
        ),
    )

    left_swivel = model.articulation(
        "left_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=left_frame,
        child=left_fork,
        origin=Origin(xyz=(0.000, 0.055, -0.335)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=8.0),
    )
    right_swivel = model.articulation(
        "right_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=right_frame,
        child=right_fork,
        origin=Origin(xyz=(0.000, -0.055, -0.335)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=8.0),
    )
    left_spin = model.articulation(
        "left_caster_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=left_fork,
        child=left_wheel,
        origin=Origin(xyz=(-0.020, 0.0, -0.105)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=22.0),
    )
    right_spin = model.articulation(
        "right_caster_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=right_fork,
        child=right_wheel,
        origin=Origin(xyz=(-0.020, 0.0, -0.105)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=22.0),
    )

    model.meta["articulations"] = {
        "folds": (left_fold.name, right_fold.name),
        "casters": (left_swivel.name, right_swivel.name, left_spin.name, right_spin.name),
    }
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    center_bridge = object_model.get_part("center_bridge")
    left_frame = object_model.get_part("left_side_frame")
    right_frame = object_model.get_part("right_side_frame")
    left_fork = object_model.get_part("left_front_caster_fork")
    right_fork = object_model.get_part("right_front_caster_fork")
    left_wheel = object_model.get_part("left_front_caster_wheel")
    right_wheel = object_model.get_part("right_front_caster_wheel")

    left_pad = left_frame.get_visual("alignment_pad")
    right_pad = right_frame.get_visual("alignment_pad")
    left_index = left_frame.get_visual("index_strip")
    right_index = right_frame.get_visual("index_strip")
    left_arm = left_fork.get_visual("left_arm")
    right_arm = right_fork.get_visual("right_arm")
    left_tire = left_wheel.get_visual("tire")
    right_tire = right_wheel.get_visual("tire")
    left_hub = left_wheel.get_visual("hub_core")
    right_hub = right_wheel.get_visual("hub_core")
    left_lower_hinge = center_bridge.get_visual("left_lower_hinge")
    right_lower_hinge = center_bridge.get_visual("right_lower_hinge")
    left_upper_hinge = center_bridge.get_visual("left_upper_hinge")
    right_upper_hinge = center_bridge.get_visual("right_upper_hinge")
    left_lower_pad = left_frame.get_visual("lower_hinge_pad")
    right_lower_pad = right_frame.get_visual("lower_hinge_pad")
    left_upper_pad = left_frame.get_visual("upper_hinge_pad")
    right_upper_pad = right_frame.get_visual("upper_hinge_pad")

    left_fold = object_model.get_articulation("center_to_left_frame")
    right_fold = object_model.get_articulation("center_to_right_frame")
    left_swivel = object_model.get_articulation("left_caster_swivel")
    right_swivel = object_model.get_articulation("right_caster_swivel")
    left_spin = object_model.get_articulation("left_caster_wheel_spin")
    right_spin = object_model.get_articulation("right_caster_wheel_spin")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        center_bridge,
        left_frame,
        elem_a=left_lower_hinge,
        elem_b=left_lower_pad,
        reason="Captured lower fold hinge pin remains engaged inside the left hinge pad through the fold arc.",
    )
    ctx.allow_overlap(
        center_bridge,
        right_frame,
        elem_a=right_lower_hinge,
        elem_b=right_lower_pad,
        reason="Captured lower fold hinge pin remains engaged inside the right hinge pad through the fold arc.",
    )
    ctx.allow_overlap(
        center_bridge,
        left_frame,
        elem_a=left_upper_hinge,
        elem_b=left_upper_pad,
        reason="Captured upper fold hinge pin remains engaged inside the left hinge pad through the fold arc.",
    )
    ctx.allow_overlap(
        center_bridge,
        right_frame,
        elem_a=right_upper_hinge,
        elem_b=right_upper_pad,
        reason="Captured upper fold hinge pin remains engaged inside the right hinge pad through the fold arc.",
    )
    ctx.allow_overlap(
        left_fork,
        left_wheel,
        elem_a=left_arm,
        elem_b=left_hub,
        reason="The simplified left caster hub cylinder stands in for a bearing boss nested tightly between the fork cheeks.",
    )
    ctx.allow_overlap(
        left_fork,
        left_wheel,
        elem_a=left_fork.get_visual("right_arm"),
        elem_b=left_hub,
        reason="The simplified left caster hub cylinder stands in for a bearing boss nested tightly between the fork cheeks.",
    )
    ctx.allow_overlap(
        right_fork,
        right_wheel,
        elem_a=right_fork.get_visual("left_arm"),
        elem_b=right_hub,
        reason="The simplified right caster hub cylinder stands in for a bearing boss nested tightly between the fork cheeks.",
    )
    ctx.allow_overlap(
        right_fork,
        right_wheel,
        elem_a=right_arm,
        elem_b=right_hub,
        reason="The simplified right caster hub cylinder stands in for a bearing boss nested tightly between the fork cheeks.",
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

    ctx.expect_contact(left_fork, left_frame, name="left_caster_fork_supported_by_frame")
    ctx.expect_contact(right_fork, right_frame, name="right_caster_fork_supported_by_frame")
    ctx.expect_contact(left_wheel, left_fork, name="left_caster_wheel_supported_by_fork")
    ctx.expect_contact(right_wheel, right_fork, name="right_caster_wheel_supported_by_fork")

    ctx.expect_gap(
        left_fork,
        left_wheel,
        axis="y",
        positive_elem=left_arm,
        negative_elem=left_tire,
        min_gap=0.0045,
        max_gap=0.0075,
        name="left_caster_tire_to_arm_gap",
    )
    ctx.expect_gap(
        right_fork,
        right_wheel,
        axis="y",
        positive_elem=right_fork.get_visual("left_arm"),
        negative_elem=right_tire,
        min_gap=0.0040,
        max_gap=0.0080,
        name="right_caster_tire_to_arm_gap",
    )

    with ctx.pose({left_fold: 0.0, right_fold: 0.0}):
        ctx.expect_gap(
            left_frame,
            right_frame,
            axis="y",
            positive_elem=left_pad,
            negative_elem=right_pad,
            min_gap=0.48,
            max_gap=0.54,
            name="open_pose_handle_alignment_gap",
        )

    with ctx.pose({left_fold: 0.58, right_fold: 0.58}):
        ctx.expect_gap(
            left_frame,
            right_frame,
            axis="y",
            positive_elem=left_pad,
            negative_elem=right_pad,
            min_gap=0.025,
            max_gap=0.11,
            name="folded_pose_controlled_handle_gap",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="folded_pose_no_overlaps")

    ctx.check(
        "fold_joints_use_opposed_vertical_axes",
        left_fold.axis == (0.0, 0.0, 1.0) and right_fold.axis == (0.0, 0.0, -1.0),
        details=f"left={left_fold.axis} right={right_fold.axis}",
    )
    ctx.check(
        "caster_swivel_and_roll_axes_are_explicit",
        left_swivel.axis == (0.0, 0.0, 1.0)
        and right_swivel.axis == (0.0, 0.0, 1.0)
        and left_spin.axis == (0.0, 1.0, 0.0)
        and right_spin.axis == (0.0, 1.0, 0.0),
        details=(
            f"swivel=({left_swivel.axis},{right_swivel.axis}) "
            f"spin=({left_spin.axis},{right_spin.axis})"
        ),
    )
    ctx.check(
        "caster_wheels_have_positive_trail",
        left_spin.origin.xyz[0] < 0.0 and right_spin.origin.xyz[0] < 0.0,
        details=f"left={left_spin.origin.xyz} right={right_spin.origin.xyz}",
    )
    ctx.check(
        "repeatable_alignment_features_present",
        left_index.name == "index_strip"
        and right_index.name == "index_strip"
        and left_pad.name == "alignment_pad"
        and right_pad.name == "alignment_pad",
        details="Missing index strip or alignment pad visual",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
