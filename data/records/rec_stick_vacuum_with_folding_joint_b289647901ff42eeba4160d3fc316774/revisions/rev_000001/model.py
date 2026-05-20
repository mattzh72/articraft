from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_stick_vacuum")

    body_red = model.material("body_red", rgba=(0.74, 0.16, 0.14, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.18, 0.19, 0.20, 1.0))
    black = model.material("black", rgba=(0.08, 0.08, 0.09, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.74, 1.0))
    clear_bin = model.material("clear_bin", rgba=(0.72, 0.82, 0.90, 0.45))

    def yz_section(
        width: float,
        height: float,
        radius: float,
        x: float,
        *,
        z_center: float,
    ) -> list[tuple[float, float, float]]:
        return [(x, y, z + z_center) for z, y in rounded_rect_profile(height, width, radius)]

    motor_body = model.part("motor_body")

    body_shell = section_loft(
        [
            yz_section(0.070, 0.120, 0.022, -0.090, z_center=0.235),
            yz_section(0.100, 0.155, 0.030, 0.015, z_center=0.215),
            yz_section(0.078, 0.110, 0.024, 0.155, z_center=0.175),
        ]
    )
    motor_body.visual(
        mesh_from_geometry(body_shell, "motor_body_shell"),
        material=body_red,
        name="body_shell",
    )
    motor_body.visual(
        Box((0.110, 0.078, 0.085)),
        origin=Origin(xyz=(-0.060, 0.0, 0.250)),
        material=dark_gray,
        name="battery_pack",
    )
    motor_body.visual(
        Cylinder(radius=0.043, length=0.205),
        origin=Origin(xyz=(0.085, 0.0, 0.130), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=clear_bin,
        name="dust_bin",
    )
    motor_body.visual(
        Box((0.120, 0.070, 0.055)),
        origin=Origin(xyz=(0.060, 0.0, 0.155)),
        material=dark_gray,
        name="cyclone_bridge",
    )
    motor_body.visual(
        Box((0.052, 0.080, 0.070)),
        origin=Origin(xyz=(0.010, 0.0, 0.058)),
        material=dark_gray,
        name="joint_support",
    )
    motor_body.visual(
        Box((0.056, 0.082, 0.100)),
        origin=Origin(xyz=(0.012, 0.0, 0.122)),
        material=dark_gray,
        name="neck_spine",
    )
    motor_body.visual(
        Box((0.026, 0.014, 0.048)),
        origin=Origin(xyz=(0.000, 0.031, 0.012)),
        material=black,
        name="left_hinge_lug",
    )
    motor_body.visual(
        Box((0.026, 0.014, 0.048)),
        origin=Origin(xyz=(0.000, -0.031, 0.012)),
        material=black,
        name="right_hinge_lug",
    )
    handle_geom = tube_from_spline_points(
        [
            (-0.105, 0.0, 0.165),
            (-0.145, 0.0, 0.250),
            (-0.110, 0.0, 0.360),
            (-0.025, 0.0, 0.395),
            (0.015, 0.0, 0.315),
        ],
        radius=0.015,
        samples_per_segment=18,
        radial_segments=18,
    )
    motor_body.visual(
        mesh_from_geometry(handle_geom, "motor_body_handle"),
        material=dark_gray,
        name="carry_handle",
    )
    motor_body.inertial = Inertial.from_geometry(
        Box((0.330, 0.140, 0.420)),
        mass=2.6,
        origin=Origin(xyz=(0.010, 0.0, 0.210)),
    )

    wand = model.part("wand")
    wand.visual(
        Cylinder(radius=0.021, length=0.052),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="upper_knuckle",
    )
    wand.visual(
        Cylinder(radius=0.017, length=0.640),
        origin=Origin(xyz=(0.0, 0.0, -0.320)),
        material=steel,
        name="wand_tube",
    )
    wand.visual(
        Box((0.030, 0.040, 0.048)),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=black,
        name="upper_socket",
    )
    wand.visual(
        Box((0.028, 0.050, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.627)),
        material=black,
        name="lower_bridge",
    )
    wand.visual(
        Box((0.024, 0.012, 0.055)),
        origin=Origin(xyz=(0.0, 0.028, -0.650)),
        material=black,
        name="lower_left_fork",
    )
    wand.visual(
        Box((0.024, 0.012, 0.055)),
        origin=Origin(xyz=(0.0, -0.028, -0.650)),
        material=black,
        name="lower_right_fork",
    )
    wand.inertial = Inertial.from_geometry(
        Cylinder(radius=0.018, length=0.660),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.0, -0.320)),
    )

    model.articulation(
        "body_to_wand_fold",
        ArticulationType.REVOLUTE,
        parent=motor_body,
        child=wand,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.4,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )

    floor_head = model.part("floor_head")
    floor_head.visual(
        Cylinder(radius=0.015, length=0.044),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="pitch_barrel",
    )
    floor_head.visual(
        Box((0.042, 0.038, 0.056)),
        origin=Origin(xyz=(0.006, 0.0, -0.028)),
        material=black,
        name="neck_tower",
    )
    floor_head.visual(
        Box((0.285, 0.115, 0.034)),
        origin=Origin(xyz=(0.025, 0.0, -0.058)),
        material=dark_gray,
        name="head_shell",
    )
    floor_head.visual(
        Box((0.255, 0.095, 0.008)),
        origin=Origin(xyz=(0.028, 0.0, -0.076)),
        material=body_red,
        name="soleplate",
    )
    floor_head.visual(
        Cylinder(radius=0.014, length=0.095),
        origin=Origin(xyz=(0.055, 0.0, -0.058), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="front_roller",
    )
    floor_head.inertial = Inertial.from_geometry(
        Box((0.285, 0.115, 0.085)),
        mass=0.9,
        origin=Origin(xyz=(0.025, 0.0, -0.042)),
    )

    model.articulation(
        "wand_to_floor_head_pitch",
        ArticulationType.REVOLUTE,
        parent=wand,
        child=floor_head,
        origin=Origin(xyz=(0.0, 0.0, -0.665)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=math.radians(-40.0),
            upper=math.radians(22.0),
        ),
    )

    fold_guard = model.part("fold_guard")
    guard_geom = tube_from_spline_points(
        [
            (0.010, 0.050, 0.065),
            (-0.040, 0.058, 0.090),
            (-0.072, 0.035, 0.085),
            (-0.082, 0.000, 0.045),
            (-0.072, -0.035, 0.085),
            (-0.040, -0.058, 0.090),
            (0.010, -0.050, 0.065),
        ],
        radius=0.008,
        samples_per_segment=14,
        radial_segments=14,
    )
    fold_guard.visual(
        mesh_from_geometry(guard_geom, "fold_guard_hoop"),
        material=steel,
        name="guard_hoop",
    )
    fold_guard.visual(
        Box((0.028, 0.018, 0.028)),
        origin=Origin(xyz=(0.010, 0.050, 0.067)),
        material=black,
        name="left_guard_mount",
    )
    fold_guard.visual(
        Box((0.028, 0.018, 0.028)),
        origin=Origin(xyz=(0.010, -0.050, 0.067)),
        material=black,
        name="right_guard_mount",
    )
    fold_guard.inertial = Inertial.from_geometry(
        Box((0.120, 0.150, 0.080)),
        mass=0.2,
        origin=Origin(xyz=(-0.025, 0.0, 0.070)),
    )

    model.articulation(
        "body_to_fold_guard",
        ArticulationType.FIXED,
        parent=motor_body,
        child=fold_guard,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    motor_body = object_model.get_part("motor_body")
    wand = object_model.get_part("wand")
    floor_head = object_model.get_part("floor_head")
    fold_guard = object_model.get_part("fold_guard")
    fold_joint = object_model.get_articulation("body_to_wand_fold")
    pitch_joint = object_model.get_articulation("wand_to_floor_head_pitch")

    def aabb_center(aabb):
        if aabb is None:
            return None
        min_corner, max_corner = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(min_corner, max_corner))

    ctx.expect_contact(
        wand,
        motor_body,
        elem_a="upper_knuckle",
        elem_b="left_hinge_lug",
        name="left fold lug captures the upper knuckle",
    )
    ctx.expect_contact(
        wand,
        motor_body,
        elem_a="upper_knuckle",
        elem_b="right_hinge_lug",
        name="right fold lug captures the upper knuckle",
    )
    ctx.expect_contact(
        floor_head,
        wand,
        elem_a="pitch_barrel",
        elem_b="lower_left_fork",
        name="left fork supports the floor head barrel",
    )
    ctx.expect_contact(
        floor_head,
        wand,
        elem_a="pitch_barrel",
        elem_b="lower_right_fork",
        name="right fork supports the floor head barrel",
    )
    ctx.expect_overlap(
        fold_guard,
        wand,
        axes="xy",
        elem_a="guard_hoop",
        elem_b="upper_knuckle",
        min_overlap=0.02,
        name="guard hoop wraps laterally around the fold stage",
    )
    ctx.expect_gap(
        fold_guard,
        wand,
        axis="z",
        positive_elem="guard_hoop",
        negative_elem="upper_knuckle",
        min_gap=0.01,
        max_gap=0.03,
        name="guard hoop clears the folding knuckle from above",
    )

    rest_head_pos = ctx.part_world_position(floor_head)
    with ctx.pose({fold_joint: fold_joint.motion_limits.upper}):
        folded_head_pos = ctx.part_world_position(floor_head)
        ctx.expect_gap(
            floor_head,
            motor_body,
            axis="x",
            min_gap=0.35,
            name="folded wand swings the nozzle clear of the motor body",
        )
    ctx.check(
        "fold joint tucks the wand forward and upward",
        rest_head_pos is not None
        and folded_head_pos is not None
        and folded_head_pos[0] > rest_head_pos[0] + 0.45
        and folded_head_pos[2] > rest_head_pos[2] + 0.45,
        details=f"rest={rest_head_pos}, folded={folded_head_pos}",
    )

    with ctx.pose({pitch_joint: pitch_joint.motion_limits.lower}):
        roller_low = aabb_center(ctx.part_element_world_aabb(floor_head, elem="front_roller"))
    with ctx.pose({pitch_joint: pitch_joint.motion_limits.upper}):
        roller_high = aabb_center(ctx.part_element_world_aabb(floor_head, elem="front_roller"))
    ctx.check(
        "floor head pitch sweeps the front roller through a clear arc",
        roller_low is not None
        and roller_high is not None
        and abs(roller_low[2] - roller_high[2]) > 0.05
        and abs(roller_low[0] - roller_high[0]) > 0.03,
        details=f"roller_low={roller_low}, roller_high={roller_high}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
