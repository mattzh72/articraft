from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, pi, sin, sqrt

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


def _add_x_cylinder(part, name, radius, length, xyz, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _rail_center(length: float, angle: float) -> tuple[float, float]:
    return (0.5 * length * sin(angle), -0.5 * length * cos(angle))


def _point_down_rail(distance: float, angle: float) -> tuple[float, float]:
    return (distance * sin(angle), -distance * cos(angle))


def _brace_setup(
    pivot_y: float,
    pivot_z: float,
    center_y: float,
    center_z: float,
) -> tuple[float, float, float, float]:
    dy = center_y - pivot_y
    dz = center_z - pivot_z
    length = sqrt(dy * dy + dz * dz)
    angle = atan2(dy, -dz)
    return dy, dz, length, angle


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_legacy_step_ladder")

    frame_metal = model.material("frame_metal", rgba=(0.72, 0.74, 0.76, 1.0))
    brace_metal = model.material("brace_metal", rgba=(0.28, 0.30, 0.33, 1.0))
    hatch_amber = model.material("hatch_amber", rgba=(0.76, 0.61, 0.23, 1.0))
    bolt_zinc = model.material("bolt_zinc", rgba=(0.86, 0.86, 0.84, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.08, 1.0))

    rail_x = 0.205
    front_rail_size = (0.050, 0.028, 1.240)
    rear_rail_size = (0.046, 0.026, 1.120)
    front_angle = -0.180
    rear_open_angle = 0.550

    top_tray_size = (0.400, 0.140, 0.040)
    top_tray_center = (0.0, -0.055, -0.045)

    front_step_depth = 0.090
    front_step_thickness = 0.024
    front_step_width = 0.360
    front_step_positions = (0.240, 0.480, 0.720, 0.960)

    front_pivot_distance = 0.540
    rear_pivot_distance = 0.560
    brace_center_y = 0.080
    brace_center_z = -0.620
    rear_frame_offset_y = 0.032

    front_pivot_y, front_pivot_z = _point_down_rail(front_pivot_distance, front_angle)
    rear_pivot_y, rear_pivot_z = _point_down_rail(rear_pivot_distance, rear_open_angle)
    front_brace_dy, front_brace_dz, front_brace_len, front_brace_angle = _brace_setup(
        front_pivot_y,
        front_pivot_z,
        brace_center_y,
        brace_center_z,
    )
    rear_brace_center_y = brace_center_y - rear_frame_offset_y
    rear_brace_dy, rear_brace_dz, rear_brace_len, rear_brace_angle = _brace_setup(
        rear_pivot_y,
        rear_pivot_z,
        rear_brace_center_y,
        brace_center_z,
    )

    front_frame = model.part("front_frame")
    rear_frame = model.part("rear_frame")
    left_front_brace = model.part("left_front_brace")
    right_front_brace = model.part("right_front_brace")
    left_rear_brace = model.part("left_rear_brace")
    right_rear_brace = model.part("right_rear_brace")

    front_frame.visual(
        Box(top_tray_size),
        origin=Origin(xyz=top_tray_center),
        material=frame_metal,
        name="top_tray",
    )
    front_frame.visual(
        Box((0.110, 0.050, 0.006)),
        origin=Origin(xyz=(-0.105, top_tray_center[1], -0.022)),
        material=hatch_amber,
        name="left_service_hatch",
    )
    front_frame.visual(
        Box((0.110, 0.050, 0.006)),
        origin=Origin(xyz=(0.105, top_tray_center[1], -0.022)),
        material=hatch_amber,
        name="right_service_hatch",
    )

    front_center_y, front_center_z = _rail_center(front_rail_size[2], front_angle)
    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        x = side_sign * rail_x
        front_frame.visual(
            Box(front_rail_size),
            origin=Origin(xyz=(x, front_center_y, front_center_z), rpy=(front_angle, 0.0, 0.0)),
            material=frame_metal,
            name=f"{side_name}_front_leg",
        )
        front_frame.visual(
            Box((0.072, 0.092, 0.020)),
            origin=Origin(
                xyz=(
                    x,
                    front_rail_size[2] * sin(front_angle),
                    -front_rail_size[2] * cos(front_angle) - 0.010,
                )
            ),
            material=rubber_black,
            name=f"{side_name}_front_foot",
        )
        front_frame.visual(
            Box((0.030, 0.024, 0.048)),
            origin=Origin(xyz=(side_sign * 0.245, 0.020, -0.023)),
            material=frame_metal,
            name=f"{side_name}_hinge_adapter",
        )
        _add_x_cylinder(
            front_frame,
            f"{side_name}_hinge_bolt",
            radius=0.010,
            length=0.080,
            xyz=(side_sign * 0.236, 0.020, -0.035),
            material=bolt_zinc,
        )
        front_frame.visual(
            Box((0.014, 0.032, 0.052)),
            origin=Origin(xyz=(side_sign * 0.173, front_pivot_y, front_pivot_z)),
            material=frame_metal,
            name=f"{side_name}_front_brace_mount",
        )

    for index, distance_down in enumerate(front_step_positions, start=1):
        step_y, step_z = _point_down_rail(distance_down, front_angle)
        front_frame.visual(
            Box((front_step_width, front_step_depth, front_step_thickness)),
            origin=Origin(xyz=(0.0, step_y - 0.004, step_z)),
            material=frame_metal,
            name=f"tread_{index}",
        )
        for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
            front_frame.visual(
                Box((0.030, 0.060, 0.042)),
                origin=Origin(xyz=(side_sign * 0.168, step_y - 0.004, step_z - 0.020)),
                material=brace_metal,
                name=f"{side_name}_tread_reinforcement_{index}",
            )

    rear_center_y, rear_center_z = _rail_center(rear_rail_size[2], rear_open_angle)
    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        x = side_sign * rail_x
        rear_frame.visual(
            Box(rear_rail_size),
            origin=Origin(xyz=(x, rear_center_y, rear_center_z), rpy=(rear_open_angle, 0.0, 0.0)),
            material=frame_metal,
            name=f"{side_name}_rear_leg",
        )
        rear_frame.visual(
            Box((0.058, 0.036, 0.082)),
            origin=Origin(xyz=(x, 0.018, -0.044)),
            material=frame_metal,
            name=f"{side_name}_hinge_clevis",
        )
        _add_x_cylinder(
            rear_frame,
            f"{side_name}_hinge_sleeve",
            radius=0.009,
            length=0.064,
            xyz=(x, 0.018, -0.035),
            material=bolt_zinc,
        )
        rear_frame.visual(
            Box((0.020, 0.022, 0.048)),
            origin=Origin(xyz=(side_sign * 0.193, rear_pivot_y, rear_pivot_z)),
            material=frame_metal,
            name=f"{side_name}_rear_brace_gusset",
        )
        _add_x_cylinder(
            rear_frame,
            f"{side_name}_rear_brace_mount",
            radius=0.010,
            length=0.010,
            xyz=(side_sign * 0.183, rear_pivot_y, rear_pivot_z),
            material=bolt_zinc,
        )
        rear_frame.visual(
            Box((0.068, 0.090, 0.020)),
            origin=Origin(
                xyz=(
                    x,
                    rear_rail_size[2] * sin(rear_open_angle),
                    -rear_rail_size[2] * cos(rear_open_angle) - 0.010,
                )
            ),
            material=rubber_black,
            name=f"{side_name}_rear_foot",
        )

    rear_frame.visual(
        Box((0.420, 0.024, 0.048)),
        origin=Origin(xyz=(0.0, 0.018, -0.044)),
        material=brace_metal,
        name="rear_top_bridge",
    )
    for name, distance_down, size in (
        ("retrofit_backstrap", 0.260, (0.432, 0.020, 0.070)),
        ("upper_rear_spanner", 0.760, (0.438, 0.028, 0.044)),
        ("lower_rear_spanner", 0.980, (0.446, 0.024, 0.038)),
    ):
        support_y, support_z = _point_down_rail(distance_down, rear_open_angle)
        rear_frame.visual(
            Box(size),
            origin=Origin(xyz=(0.0, support_y, support_z)),
            material=brace_metal,
            name=name,
        )
        for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
            _add_x_cylinder(
                rear_frame,
                f"{name}_{side_name}_bolt",
                radius=0.008,
                length=0.026,
                xyz=(side_sign * 0.195, support_y, support_z),
                material=bolt_zinc,
            )

    def add_front_brace(part, side_name: str, side_sign: float) -> None:
        outer_x = -side_sign * 0.005
        strap_x = -side_sign * 0.006
        center_x = side_sign * 0.002
        strap_len = front_brace_len - 0.030
        start_offset = 0.016
        outer_lug_len = 0.034
        center_lug_len = 0.018
        strap_center_factor = (start_offset + 0.5 * strap_len) / front_brace_len
        outer_lug_factor = 0.5 * outer_lug_len / front_brace_len
        center_lug_factor = (front_brace_len - 0.5 * center_lug_len) / front_brace_len
        part.visual(
            Box((0.004, 0.018, strap_len)),
            origin=Origin(
                xyz=(
                    strap_x,
                    strap_center_factor * front_brace_dy,
                    strap_center_factor * front_brace_dz,
                ),
                rpy=(front_brace_angle, 0.0, 0.0),
            ),
            material=brace_metal,
            name="brace_bar",
        )
        _add_x_cylinder(
            part,
            "outer_collar",
            radius=0.010,
            length=0.010,
            xyz=(outer_x, 0.0, 0.0),
            material=bolt_zinc,
        )
        part.visual(
            Box((abs(strap_x - outer_x) + 0.004, 0.018, outer_lug_len)),
            origin=Origin(
                xyz=(
                    0.5 * (strap_x + outer_x),
                    outer_lug_factor * front_brace_dy,
                    outer_lug_factor * front_brace_dz,
                ),
                rpy=(front_brace_angle, 0.0, 0.0),
            ),
            material=brace_metal,
            name="outer_lug",
        )
        _add_x_cylinder(
            part,
            "center_collar",
            radius=0.006,
            length=0.012,
            xyz=(center_x, front_brace_dy, front_brace_dz),
            material=bolt_zinc,
        )
        part.visual(
            Box((0.004, 0.016, center_lug_len)),
            origin=Origin(
                xyz=(
                    strap_x,
                    center_lug_factor * front_brace_dy,
                    center_lug_factor * front_brace_dz,
                ),
                rpy=(front_brace_angle, 0.0, 0.0),
            ),
            material=brace_metal,
            name=f"{side_name}_brace_adapter",
        )

    def add_rear_brace(part, side_name: str, side_sign: float) -> None:
        outer_x = -side_sign * 0.005
        strap_x = -side_sign * 0.006
        center_x = 0.0
        strap_len = rear_brace_len - 0.030
        start_offset = 0.016
        outer_lug_len = 0.034
        center_lug_len = 0.018
        strap_center_factor = (start_offset + 0.5 * strap_len) / rear_brace_len
        center_lug_factor = (rear_brace_len - 0.5 * center_lug_len) / rear_brace_len
        outer_lug_factor = 0.5 * outer_lug_len / rear_brace_len
        part.visual(
            Box((0.004, 0.018, strap_len)),
            origin=Origin(
                xyz=(
                    strap_x,
                    strap_center_factor * rear_brace_dy,
                    strap_center_factor * rear_brace_dz,
                ),
                rpy=(rear_brace_angle, 0.0, 0.0),
            ),
            material=brace_metal,
            name="brace_bar",
        )
        _add_x_cylinder(
            part,
            "outer_collar",
            radius=0.010,
            length=0.010,
            xyz=(outer_x, 0.0, 0.0),
            material=bolt_zinc,
        )
        part.visual(
            Box((abs(strap_x - outer_x) + 0.004, 0.018, outer_lug_len)),
            origin=Origin(
                xyz=(
                    0.5 * (strap_x + outer_x),
                    outer_lug_factor * rear_brace_dy,
                    outer_lug_factor * rear_brace_dz,
                ),
                rpy=(rear_brace_angle, 0.0, 0.0),
            ),
            material=brace_metal,
            name="outer_lug",
        )
        _add_x_cylinder(
            part,
            "center_collar",
            radius=0.006,
            length=0.008,
            xyz=(center_x, rear_brace_dy, rear_brace_dz),
            material=bolt_zinc,
        )
        part.visual(
            Box((0.004, 0.016, center_lug_len)),
            origin=Origin(
                xyz=(
                    strap_x,
                    center_lug_factor * rear_brace_dy,
                    center_lug_factor * rear_brace_dz,
                ),
                rpy=(rear_brace_angle, 0.0, 0.0),
            ),
            material=brace_metal,
            name=f"{side_name}_brace_adapter",
        )

    add_front_brace(left_front_brace, "left", -1.0)
    add_front_brace(right_front_brace, "right", 1.0)
    add_rear_brace(left_rear_brace, "left", -1.0)
    add_rear_brace(right_rear_brace, "right", 1.0)

    model.articulation(
        "rear_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=rear_frame,
        origin=Origin(xyz=(0.0, rear_frame_offset_y, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.4,
            lower=-0.700,
            upper=0.100,
        ),
    )
    model.articulation(
        "left_front_brace_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=left_front_brace,
        origin=Origin(xyz=(-0.166, front_pivot_y, front_pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=-1.750,
            upper=0.150,
        ),
    )
    model.articulation(
        "right_front_brace_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=right_front_brace,
        origin=Origin(xyz=(0.166, front_pivot_y, front_pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=-1.750,
            upper=0.150,
        ),
    )
    model.articulation(
        "left_rear_brace_hinge",
        ArticulationType.REVOLUTE,
        parent=rear_frame,
        child=left_rear_brace,
        origin=Origin(xyz=(-0.178, rear_pivot_y, rear_pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=-0.150,
            upper=1.250,
        ),
    )
    model.articulation(
        "right_rear_brace_hinge",
        ArticulationType.REVOLUTE,
        parent=rear_frame,
        child=right_rear_brace,
        origin=Origin(xyz=(0.178, rear_pivot_y, rear_pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=-0.150,
            upper=1.250,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front_frame = object_model.get_part("front_frame")
    rear_frame = object_model.get_part("rear_frame")
    left_front_brace = object_model.get_part("left_front_brace")
    right_front_brace = object_model.get_part("right_front_brace")
    left_rear_brace = object_model.get_part("left_rear_brace")
    right_rear_brace = object_model.get_part("right_rear_brace")
    rear_hinge = object_model.get_articulation("rear_hinge")
    left_front_brace_hinge = object_model.get_articulation("left_front_brace_hinge")
    right_front_brace_hinge = object_model.get_articulation("right_front_brace_hinge")
    left_rear_brace_hinge = object_model.get_articulation("left_rear_brace_hinge")
    right_rear_brace_hinge = object_model.get_articulation("right_rear_brace_hinge")

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
        "all_ladder_parts_present",
        all(
            part is not None
            for part in (
                front_frame,
                rear_frame,
                left_front_brace,
                right_front_brace,
                left_rear_brace,
                right_rear_brace,
            )
        ),
        "front frame, rear frame, and all four spread-brace members must exist",
    )
    ctx.check(
        "hinge_axes_run_across_ladder_width",
        rear_hinge.axis == (1.0, 0.0, 0.0)
        and left_front_brace_hinge.axis == (1.0, 0.0, 0.0)
        and right_front_brace_hinge.axis == (1.0, 0.0, 0.0)
        and left_rear_brace_hinge.axis == (1.0, 0.0, 0.0)
        and right_rear_brace_hinge.axis == (1.0, 0.0, 0.0),
        "leg and spread-brace pivots should rotate in the side plane around the ladder width axis",
    )

    ctx.expect_contact(
        front_frame,
        rear_frame,
        elem_a="left_hinge_adapter",
        elem_b="left_hinge_clevis",
        name="left_top_hinge_supported",
    )
    ctx.expect_contact(
        front_frame,
        rear_frame,
        elem_a="right_hinge_adapter",
        elem_b="right_hinge_clevis",
        name="right_top_hinge_supported",
    )

    ctx.expect_contact(
        left_front_brace,
        front_frame,
        elem_a="outer_collar",
        elem_b="left_front_brace_mount",
        name="left_front_brace_mounted",
    )
    ctx.expect_contact(
        right_front_brace,
        front_frame,
        elem_a="outer_collar",
        elem_b="right_front_brace_mount",
        name="right_front_brace_mounted",
    )
    ctx.expect_contact(
        left_rear_brace,
        rear_frame,
        elem_a="outer_collar",
        elem_b="left_rear_brace_mount",
        name="left_rear_brace_mounted",
    )
    ctx.expect_contact(
        right_rear_brace,
        rear_frame,
        elem_a="outer_collar",
        elem_b="right_rear_brace_mount",
        name="right_rear_brace_mounted",
    )
    ctx.expect_contact(
        left_front_brace,
        left_rear_brace,
        elem_a="center_collar",
        elem_b="center_collar",
        name="left_spread_linkage_engaged",
    )
    ctx.expect_contact(
        right_front_brace,
        right_rear_brace,
        elem_a="center_collar",
        elem_b="center_collar",
        name="right_spread_linkage_engaged",
    )

    ctx.expect_gap(
        rear_frame,
        front_frame,
        axis="y",
        positive_elem="left_rear_foot",
        negative_elem="left_front_foot",
        min_gap=0.70,
        name="left_open_stance_spread",
    )
    ctx.expect_gap(
        rear_frame,
        front_frame,
        axis="y",
        positive_elem="right_rear_foot",
        negative_elem="right_front_foot",
        min_gap=0.70,
        name="right_open_stance_spread",
    )

    with ctx.pose(
        {
            rear_hinge: -0.670,
            left_front_brace_hinge: -1.450,
            right_front_brace_hinge: -1.450,
            left_rear_brace_hinge: 0.950,
            right_rear_brace_hinge: 0.950,
        }
    ):
        ctx.expect_gap(
            rear_frame,
            front_frame,
            axis="y",
            positive_elem="left_rear_foot",
            negative_elem="left_front_foot",
            max_gap=0.080,
            max_penetration=0.015,
            name="left_closed_stance_stacks_compactly",
        )
        ctx.expect_gap(
            rear_frame,
            front_frame,
            axis="y",
            positive_elem="right_rear_foot",
            negative_elem="right_front_foot",
            max_gap=0.080,
            max_penetration=0.015,
            name="right_closed_stance_stacks_compactly",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
