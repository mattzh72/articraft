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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


def _tube_mesh(
    name: str,
    points: list[tuple[float, float, float]],
    *,
    radius: float,
    corner_radius: float,
    radial_segments: int = 18,
) -> object:
    return mesh_from_geometry(
        wire_from_points(
            points,
            radius=radius,
            radial_segments=radial_segments,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=corner_radius,
            corner_segments=10,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_walker")

    aluminum = model.material("aluminum", rgba=(0.82, 0.84, 0.86, 1.0))
    grip_black = model.material("grip_black", rgba=(0.14, 0.14, 0.15, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.23, 0.24, 0.26, 1.0))
    wheel_gray = model.material("wheel_gray", rgba=(0.62, 0.65, 0.68, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.58, 0.42, 0.92)),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.0, 0.46)),
    )

    frame_tube_radius = 0.014
    side_x = 0.255
    front_y = 0.18
    handle_y = 0.12
    rear_y = -0.18
    handle_z = 0.87
    front_mount_z = 0.18

    for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
        x_pos = side_x * side_sign
        side_loop = [
            (x_pos, front_y, front_mount_z),
            (x_pos, front_y, 0.76),
            (x_pos, handle_y, handle_z),
            (x_pos, -0.10, handle_z),
            (x_pos, rear_y, 0.06),
        ]
        frame.visual(
            _tube_mesh(
                f"{side_name}_walker_side",
                side_loop,
                radius=frame_tube_radius,
                corner_radius=0.055,
            ),
            material=aluminum,
            name=f"{side_name}_side_frame",
        )
        frame.visual(
            Cylinder(radius=0.0185, length=0.13),
            origin=Origin(
                xyz=(x_pos, -0.045, handle_z),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=grip_black,
            name=f"{side_name}_handle_grip",
        )
        frame.visual(
            Cylinder(radius=0.017, length=0.060),
            origin=Origin(xyz=(x_pos, rear_y, 0.030)),
            material=rubber,
            name=f"{side_name}_rear_tip",
        )
        frame.visual(
            Cylinder(radius=0.019, length=0.040),
            origin=Origin(xyz=(x_pos, front_y, 0.160)),
            material=dark_gray,
            name=f"{side_name}_caster_socket",
        )

    frame.visual(
        Cylinder(radius=0.012, length=0.50),
        origin=Origin(
            xyz=(0.0, 0.01, handle_z),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=aluminum,
        name="upper_crossbar",
    )
    frame.visual(
        Cylinder(radius=0.011, length=0.482),
        origin=Origin(
            xyz=(0.0, 0.18, 0.34),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=aluminum,
        name="lower_crossbar",
    )

    lower_brace = model.part("lower_brace")
    lower_brace.inertial = Inertial.from_geometry(
        Cylinder(radius=0.010, length=0.482),
        mass=0.45,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    lower_brace.visual(
        Cylinder(radius=0.010, length=0.482),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_gray,
        name="brace_bar",
    )

    caster_radius = 0.050
    wheel_width = 0.024
    fork_inner_gap = 0.036
    fork_side_thickness = 0.006
    fork_side_depth = 0.090
    fork_side_height = 0.074
    fork_half_offset = fork_inner_gap * 0.5 + fork_side_thickness * 0.5
    axle_z = -0.094

    for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
        caster = model.part(f"{side_name}_caster")
        caster.inertial = Inertial.from_geometry(
            Box((0.055, 0.050, 0.160)),
            mass=0.35,
            origin=Origin(xyz=(0.0, 0.0, -0.080)),
        )
        caster.visual(
            Cylinder(radius=0.007, length=0.034),
            origin=Origin(xyz=(0.0, 0.0, -0.017)),
            material=dark_gray,
            name="stem",
        )
        caster.visual(
            Box((0.048, 0.054, 0.012)),
            origin=Origin(xyz=(0.0, 0.0, -0.028)),
            material=dark_gray,
            name="fork_crown",
        )
        caster.visual(
            Box((fork_side_thickness, fork_side_depth, fork_side_height)),
            origin=Origin(xyz=(fork_half_offset, 0.0, -0.065)),
            material=dark_gray,
            name="outer_blade",
        )
        caster.visual(
            Box((fork_side_thickness, fork_side_depth, fork_side_height)),
            origin=Origin(xyz=(-fork_half_offset, 0.0, -0.065)),
            material=dark_gray,
            name="inner_blade",
        )
        caster.visual(
            Cylinder(radius=0.005, length=0.006),
            origin=Origin(xyz=(fork_half_offset, 0.0, axle_z), rpy=(0.0, pi / 2.0, 0.0)),
            material=wheel_gray,
            name="outer_stub",
        )
        caster.visual(
            Cylinder(radius=0.005, length=0.006),
            origin=Origin(xyz=(-fork_half_offset, 0.0, axle_z), rpy=(0.0, pi / 2.0, 0.0)),
            material=wheel_gray,
            name="inner_stub",
        )

        wheel = model.part(f"{side_name}_front_wheel")
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=caster_radius, length=wheel_width),
            mass=0.18,
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        )
        wheel.visual(
            Cylinder(radius=caster_radius, length=wheel_width),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=rubber,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.025, length=wheel_width + 0.004),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=wheel_gray,
            name="hub",
        )
        wheel.visual(
            Cylinder(radius=0.010, length=0.004),
            origin=Origin(xyz=(0.016, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=wheel_gray,
            name="outer_bearing_cap",
        )
        wheel.visual(
            Cylinder(radius=0.010, length=0.004),
            origin=Origin(xyz=(-0.016, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=wheel_gray,
            name="inner_bearing_cap",
        )

        model.articulation(
            f"frame_to_{side_name}_caster_swivel",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=caster,
            origin=Origin(xyz=(side_x * side_sign, front_y, 0.140)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=2.5,
                velocity=4.0,
                lower=-pi,
                upper=pi,
            ),
        )
        model.articulation(
            f"{side_name}_caster_to_wheel_spin",
            ArticulationType.CONTINUOUS,
            parent=caster,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, axle_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.2, velocity=18.0),
        )

    model.articulation(
        "frame_to_lower_brace",
        ArticulationType.FIXED,
        parent=frame,
        child=lower_brace,
        origin=Origin(xyz=(0.0, -0.162, 0.22)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    frame = object_model.get_part("frame")
    brace = object_model.get_part("lower_brace")
    left_caster = object_model.get_part("left_caster")
    right_caster = object_model.get_part("right_caster")
    left_wheel = object_model.get_part("left_front_wheel")
    right_wheel = object_model.get_part("right_front_wheel")
    left_swivel = object_model.get_articulation("frame_to_left_caster_swivel")
    right_swivel = object_model.get_articulation("frame_to_right_caster_swivel")
    left_spin = object_model.get_articulation("left_caster_to_wheel_spin")
    right_spin = object_model.get_articulation("right_caster_to_wheel_spin")

    ctx.expect_contact(
        frame,
        brace,
        elem_a="left_side_frame",
        elem_b="brace_bar",
        name="brace ties into the walker frame",
    )
    ctx.expect_gap(
        frame,
        frame,
        axis="z",
        positive_elem="left_handle_grip",
        negative_elem="left_rear_tip",
        min_gap=0.76,
        name="rear support sits well below the hand grip",
    )
    ctx.expect_contact(
        frame,
        brace,
        elem_a="right_side_frame",
        elem_b="brace_bar",
        name="brace ties into the right support",
    )
    ctx.expect_gap(
        frame,
        left_wheel,
        axis="z",
        positive_elem="left_caster_socket",
        negative_elem="tire",
        min_gap=0.005,
        max_gap=0.060,
        name="left caster hangs below the front socket",
    )
    ctx.expect_gap(
        frame,
        right_wheel,
        axis="z",
        positive_elem="right_caster_socket",
        negative_elem="tire",
        min_gap=0.005,
        max_gap=0.060,
        name="right caster hangs below the front socket",
    )
    ctx.expect_gap(
        left_caster,
        left_wheel,
        axis="z",
        positive_elem="fork_crown",
        negative_elem="tire",
        min_gap=0.006,
        max_gap=0.040,
        name="left fork crown stays above the wheel",
    )
    ctx.expect_gap(
        right_caster,
        right_wheel,
        axis="z",
        positive_elem="fork_crown",
        negative_elem="tire",
        min_gap=0.006,
        max_gap=0.040,
        name="right fork crown stays above the wheel",
    )
    ctx.check(
        "left caster uses vertical swivel and rolling wheel joints",
        left_swivel.axis == (0.0, 0.0, 1.0)
        and right_swivel.axis == (0.0, 0.0, 1.0)
        and left_spin.joint_type == ArticulationType.CONTINUOUS
        and right_spin.joint_type == ArticulationType.CONTINUOUS,
        details=(
            f"left_swivel_axis={left_swivel.axis}, right_swivel_axis={right_swivel.axis}, "
            f"left_spin_type={left_spin.joint_type}, right_spin_type={right_spin.joint_type}"
        ),
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
