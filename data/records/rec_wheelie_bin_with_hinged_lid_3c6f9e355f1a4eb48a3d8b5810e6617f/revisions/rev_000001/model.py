from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi, radians

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_wheelie_bin")

    body_plastic = model.material("body_plastic", rgba=(0.20, 0.27, 0.22, 1.0))
    lid_plastic = model.material("lid_plastic", rgba=(0.23, 0.31, 0.25, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    wheel_grey = model.material("wheel_grey", rgba=(0.17, 0.18, 0.19, 1.0))
    stainless = model.material("stainless", rgba=(0.66, 0.69, 0.72, 1.0))

    body_shell = model.part("body_shell")
    body_shell.visual(
        Box((0.516, 0.642, 0.018)),
        origin=Origin(xyz=(0.0, -0.001, 0.026)),
        material=body_plastic,
        name="floor_pan",
    )
    body_shell.visual(
        Box((0.54, 0.012, 0.87)),
        origin=Origin(xyz=(0.0, -0.328, 0.467)),
        material=body_plastic,
        name="front_wall",
    )
    body_shell.visual(
        Box((0.54, 0.012, 0.88)),
        origin=Origin(xyz=(0.0, 0.326, 0.470)),
        material=body_plastic,
        name="back_wall",
    )
    body_shell.visual(
        Box((0.012, 0.66, 0.90)),
        origin=Origin(xyz=(-0.264, -0.006, 0.480)),
        material=body_plastic,
        name="left_wall",
    )
    body_shell.visual(
        Box((0.012, 0.66, 0.90)),
        origin=Origin(xyz=(0.264, -0.006, 0.480)),
        material=body_plastic,
        name="right_wall",
    )
    body_shell.visual(
        Box((0.58, 0.040, 0.035)),
        origin=Origin(xyz=(0.0, -0.352, 0.932)),
        material=body_plastic,
        name="front_rim",
    )
    body_shell.visual(
        Box((0.040, 0.68, 0.035)),
        origin=Origin(xyz=(-0.286, -0.010, 0.932)),
        material=body_plastic,
        name="left_rim",
    )
    body_shell.visual(
        Box((0.040, 0.68, 0.035)),
        origin=Origin(xyz=(0.286, -0.010, 0.932)),
        material=body_plastic,
        name="right_rim",
    )
    body_shell.visual(
        Box((0.44, 0.030, 0.020)),
        origin=Origin(xyz=(0.0, -0.315, 0.520)),
        material=body_plastic,
        name="front_stiffener",
    )
    body_shell.visual(
        Box((0.028, 0.26, 0.020)),
        origin=Origin(xyz=(-0.270, -0.010, 0.560)),
        material=body_plastic,
        name="left_side_rib",
    )
    body_shell.visual(
        Box((0.028, 0.26, 0.020)),
        origin=Origin(xyz=(0.270, -0.010, 0.560)),
        material=body_plastic,
        name="right_side_rib",
    )
    body_shell.visual(
        Box((0.085, 0.10, 0.024)),
        origin=Origin(xyz=(-0.150, -0.230, 0.012)),
        material=body_plastic,
        name="left_front_skid",
    )
    body_shell.visual(
        Box((0.085, 0.10, 0.024)),
        origin=Origin(xyz=(0.150, -0.230, 0.012)),
        material=body_plastic,
        name="right_front_skid",
    )
    body_shell.visual(
        Box((0.36, 0.050, 0.050)),
        origin=Origin(xyz=(0.0, 0.300, 0.840)),
        material=body_plastic,
        name="rear_handle_beam",
    )
    body_shell.visual(
        Box((0.58, 0.050, 0.056)),
        origin=Origin(xyz=(0.0, 0.351, 0.900)),
        material=body_plastic,
        name="hinge_shroud",
    )
    body_shell.visual(
        Box((0.064, 0.028, 0.020)),
        origin=Origin(xyz=(-0.225, 0.338, 0.920)),
        material=body_plastic,
        name="left_hinge_support",
    )
    body_shell.visual(
        Box((0.070, 0.028, 0.020)),
        origin=Origin(xyz=(0.000, 0.338, 0.920)),
        material=body_plastic,
        name="center_hinge_support",
    )
    body_shell.visual(
        Box((0.064, 0.028, 0.020)),
        origin=Origin(xyz=(0.225, 0.338, 0.920)),
        material=body_plastic,
        name="right_hinge_support",
    )
    body_shell.visual(
        Box((0.060, 0.060, 0.060)),
        origin=Origin(xyz=(-0.200, 0.345, 0.220)),
        material=body_plastic,
        name="left_axle_pod",
    )
    body_shell.visual(
        Box((0.060, 0.060, 0.060)),
        origin=Origin(xyz=(0.200, 0.345, 0.220)),
        material=body_plastic,
        name="right_axle_pod",
    )
    body_shell.visual(
        Cylinder(radius=0.014, length=0.110),
        origin=Origin(xyz=(-0.225, 0.338, 0.944), rpy=(0.0, pi / 2.0, 0.0)),
        material=stainless,
        name="left_hinge_knuckle",
    )
    body_shell.visual(
        Cylinder(radius=0.014, length=0.120),
        origin=Origin(xyz=(0.000, 0.338, 0.944), rpy=(0.0, pi / 2.0, 0.0)),
        material=stainless,
        name="center_hinge_knuckle",
    )
    body_shell.visual(
        Cylinder(radius=0.014, length=0.110),
        origin=Origin(xyz=(0.225, 0.338, 0.944), rpy=(0.0, pi / 2.0, 0.0)),
        material=stainless,
        name="right_hinge_knuckle",
    )
    body_shell.inertial = Inertial.from_geometry(
        Box((0.58, 0.72, 0.98)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 0.49)),
    )

    rear_axle = model.part("rear_axle")
    rear_axle.visual(
        Cylinder(radius=0.012, length=0.567),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=stainless,
        name="axle_tube",
    )
    rear_axle.visual(
        Box((0.040, 0.040, 0.060)),
        origin=Origin(xyz=(-0.200, 0.0, 0.030)),
        material=stainless,
        name="left_bracket",
    )
    rear_axle.visual(
        Box((0.040, 0.040, 0.060)),
        origin=Origin(xyz=(0.200, 0.0, 0.030)),
        material=stainless,
        name="right_bracket",
    )
    rear_axle.visual(
        Box((0.060, 0.050, 0.020)),
        origin=Origin(xyz=(-0.200, 0.0, 0.070)),
        material=stainless,
        name="left_mount_pad",
    )
    rear_axle.visual(
        Box((0.060, 0.050, 0.020)),
        origin=Origin(xyz=(0.200, 0.0, 0.070)),
        material=stainless,
        name="right_mount_pad",
    )
    rear_axle.visual(
        Cylinder(radius=0.032, length=0.012),
        origin=Origin(xyz=(-0.2895, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=stainless,
        name="left_retainer",
    )
    rear_axle.visual(
        Cylinder(radius=0.032, length=0.012),
        origin=Origin(xyz=(0.2895, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=stainless,
        name="right_retainer",
    )
    rear_axle.inertial = Inertial.from_geometry(
        Box((0.60, 0.06, 0.16)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
    )

    def add_wheel(part_name: str, *, inner_sign: float) -> None:
        wheel = model.part(part_name)
        wheel.visual(
            Cylinder(radius=0.110, length=0.055),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=black_rubber,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.080, length=0.040),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=wheel_grey,
            name="rim",
        )
        wheel.visual(
            Cylinder(radius=0.034, length=0.010),
            origin=Origin(xyz=(inner_sign * 0.0225, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=wheel_grey,
            name="inner_boss",
        )
        wheel.visual(
            Cylinder(radius=0.054, length=0.012),
            origin=Origin(xyz=(-inner_sign * 0.020, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=wheel_grey,
            name="hub_cap",
        )
        wheel.inertial = Inertial.from_geometry(
            Box((0.22, 0.055, 0.22)),
            mass=1.6,
            origin=Origin(),
        )

    add_wheel("left_wheel", inner_sign=1.0)
    add_wheel("right_wheel", inner_sign=-1.0)

    lid = model.part("lid")
    lid.visual(
        Box((0.62, 0.74, 0.024)),
        origin=Origin(xyz=(0.0, -0.350, 0.028)),
        material=lid_plastic,
        name="lid_panel",
    )
    lid.visual(
        Box((0.54, 0.62, 0.010)),
        origin=Origin(xyz=(0.0, -0.340, 0.011)),
        material=lid_plastic,
        name="inner_liner",
    )
    lid.visual(
        Box((0.48, 0.030, 0.010)),
        origin=Origin(xyz=(0.0, -0.690, 0.011)),
        material=black_rubber,
        name="front_seal",
    )
    lid.visual(
        Box((0.60, 0.050, 0.090)),
        origin=Origin(xyz=(0.0, -0.735, -0.025)),
        material=lid_plastic,
        name="front_drip",
    )
    lid.visual(
        Box((0.034, 0.66, 0.088)),
        origin=Origin(xyz=(-0.327, -0.350, -0.023)),
        material=lid_plastic,
        name="left_skirt",
    )
    lid.visual(
        Box((0.034, 0.66, 0.088)),
        origin=Origin(xyz=(0.327, -0.350, -0.023)),
        material=lid_plastic,
        name="right_skirt",
    )
    lid.visual(
        Box((0.100, 0.020, 0.030)),
        origin=Origin(xyz=(-0.112, 0.002, 0.020)),
        material=lid_plastic,
        name="left_hinge_bridge",
    )
    lid.visual(
        Box((0.100, 0.020, 0.030)),
        origin=Origin(xyz=(0.112, 0.002, 0.020)),
        material=lid_plastic,
        name="right_hinge_bridge",
    )
    lid.visual(
        Box((0.62, 0.044, 0.060)),
        origin=Origin(xyz=(0.0, 0.022, 0.070)),
        material=lid_plastic,
        name="rear_shroud",
    )
    lid.visual(
        Cylinder(radius=0.014, length=0.090),
        origin=Origin(xyz=(-0.115, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=stainless,
        name="left_lid_knuckle",
    )
    lid.visual(
        Cylinder(radius=0.014, length=0.090),
        origin=Origin(xyz=(0.115, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=stainless,
        name="right_lid_knuckle",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.62, 0.78, 0.10)),
        mass=2.0,
        origin=Origin(xyz=(0.0, -0.340, 0.020)),
    )

    model.articulation(
        "body_to_rear_axle",
        ArticulationType.FIXED,
        parent=body_shell,
        child=rear_axle,
        origin=Origin(xyz=(0.0, 0.355, 0.110)),
    )
    model.articulation(
        "rear_axle_to_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=rear_axle,
        child="left_wheel",
        origin=Origin(xyz=(-0.323, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=20.0),
    )
    model.articulation(
        "rear_axle_to_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=rear_axle,
        child="right_wheel",
        origin=Origin(xyz=(0.323, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=20.0),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body_shell,
        child=lid,
        origin=Origin(xyz=(0.0, 0.338, 0.944)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=0.0,
            upper=radians(105.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body_shell")
    lid = object_model.get_part("lid")
    axle = object_model.get_part("rear_axle")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    lid_hinge = object_model.get_articulation("body_to_lid")
    left_spin = object_model.get_articulation("rear_axle_to_left_wheel")
    right_spin = object_model.get_articulation("rear_axle_to_right_wheel")

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
        "lid_joint_axis_is_protected_rear_hinge",
        tuple(lid_hinge.axis) == (-1.0, 0.0, 0.0),
        details=f"unexpected lid axis {lid_hinge.axis}",
    )
    ctx.check(
        "wheel_spin_axes_match_rear_axle",
        tuple(left_spin.axis) == (1.0, 0.0, 0.0) and tuple(right_spin.axis) == (1.0, 0.0, 0.0),
        details=f"left axis={left_spin.axis}, right axis={right_spin.axis}",
    )
    ctx.expect_contact(
        axle,
        body,
        elem_a="left_mount_pad",
        elem_b="left_axle_pod",
        name="left_axle_mount_is_hard_connected",
    )
    ctx.expect_contact(
        axle,
        body,
        elem_a="right_mount_pad",
        elem_b="right_axle_pod",
        name="right_axle_mount_is_hard_connected",
    )
    ctx.expect_contact(
        left_wheel,
        axle,
        elem_a="inner_boss",
        elem_b="left_retainer",
        name="left_wheel_is_supported_on_axle",
    )
    ctx.expect_contact(
        right_wheel,
        axle,
        elem_a="inner_boss",
        elem_b="right_retainer",
        name="right_wheel_is_supported_on_axle",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="front_seal",
            negative_elem="front_rim",
            max_gap=0.002,
            max_penetration=0.0,
            name="closed_lid_seal_sits_on_front_rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.50,
            elem_a="lid_panel",
            name="lid_panel_covers_body_opening",
        )

    with ctx.pose({lid_hinge: radians(75.0)}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="front_seal",
            negative_elem="hinge_shroud",
            min_gap=0.20,
            name="lid_front_edge_lifts_clear_when_open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
