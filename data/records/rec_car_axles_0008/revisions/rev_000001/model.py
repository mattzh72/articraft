from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os
import pathlib

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

_REAL_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _REAL_GETCWD()
    except FileNotFoundError:
        os.chdir("/")
        return "/"


os.getcwd = _safe_getcwd
pathlib.Path.cwd = classmethod(lambda cls: cls(_safe_getcwd()))


def _spin_origin(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0))


def _ensure_valid_cwd() -> None:
    _safe_getcwd()


def build_object_model() -> ArticulatedObject:
    _ensure_valid_cwd()
    model = ArticulatedObject(name="drive_axle")

    housing_steel = model.material("housing_steel", rgba=(0.45, 0.47, 0.50, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.23, 0.24, 0.26, 1.0))
    wheel_steel = model.material("wheel_steel", rgba=(0.67, 0.69, 0.72, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    def add_halfshaft(part_name: str, sign: float) -> None:
        part = model.part(part_name)
        part.visual(
            Cylinder(radius=0.058, length=0.070),
            origin=_spin_origin((sign * 0.020, 0.0, 0.0)),
            material=rubber,
            name="inner_boot",
        )
        part.visual(
            Cylinder(radius=0.050, length=0.030),
            origin=_spin_origin((sign * 0.070, 0.0, 0.0)),
            material=rubber,
            name="inner_boot_rib_1",
        )
        part.visual(
            Cylinder(radius=0.056, length=0.034),
            origin=_spin_origin((sign * 0.102, 0.0, 0.0)),
            material=rubber,
            name="inner_boot_rib_2",
        )
        part.visual(
            Cylinder(radius=0.018, length=0.220),
            origin=_spin_origin((sign * 0.225, 0.0, 0.0)),
            material=dark_steel,
            name="plunge_shaft",
        )
        part.visual(
            Cylinder(radius=0.060, length=0.050),
            origin=_spin_origin((sign * 0.335, 0.0, 0.0)),
            material=rubber,
            name="outer_boot",
        )
        part.visual(
            Cylinder(radius=0.048, length=0.036),
            origin=_spin_origin((sign * 0.372, 0.0, 0.0)),
            material=rubber,
            name="outer_boot_rib_1",
        )
        part.visual(
            Cylinder(radius=0.062, length=0.040),
            origin=_spin_origin((sign * 0.410, 0.0, 0.0)),
            material=rubber,
            name="outer_boot_rib_2",
        )
        part.visual(
            Cylinder(radius=0.046, length=0.076),
            origin=_spin_origin((sign * 0.450, 0.0, 0.0)),
            material=dark_steel,
            name="outer_cv_barrel",
        )
        part.visual(
            Cylinder(radius=0.028, length=0.060),
            origin=_spin_origin((sign * 0.470, 0.0, 0.0)),
            material=dark_steel,
            name="stub_axle",
        )
        part.visual(
            Cylinder(radius=0.054, length=0.010),
            origin=_spin_origin((sign * 0.495, 0.0, 0.0)),
            material=dark_steel,
            name="stub_flange",
        )
        part.inertial = Inertial.from_geometry(
            Box((0.52, 0.14, 0.14)),
            mass=7.0,
            origin=Origin(xyz=(sign * 0.250, 0.0, 0.0)),
        )

    def add_wheel(part_name: str, outboard_sign: float) -> None:
        part = model.part(part_name)
        part.visual(
            Cylinder(radius=0.310, length=0.225),
            origin=_spin_origin((outboard_sign * 0.1125, 0.0, 0.0)),
            material=rubber,
            name="tire",
        )
        part.visual(
            Cylinder(radius=0.215, length=0.170),
            origin=_spin_origin((outboard_sign * 0.085, 0.0, 0.0)),
            material=wheel_steel,
            name="inner_rim",
        )
        part.visual(
            Cylinder(radius=0.188, length=0.045),
            origin=_spin_origin((outboard_sign * 0.170, 0.0, 0.0)),
            material=wheel_steel,
            name="outer_rim_face",
        )
        part.visual(
            Cylinder(radius=0.075, length=0.012),
            origin=_spin_origin((outboard_sign * 0.006, 0.0, 0.0)),
            material=wheel_steel,
            name="mount_face",
        )
        part.visual(
            Cylinder(radius=0.095, length=0.065),
            origin=_spin_origin((outboard_sign * 0.0325, 0.0, 0.0)),
            material=dark_steel,
            name="hub_body",
        )
        part.visual(
            Cylinder(radius=0.055, length=0.030),
            origin=_spin_origin((outboard_sign * 0.155, 0.0, 0.0)),
            material=wheel_steel,
            name="center_cap",
        )
        part.inertial = Inertial.from_geometry(
            Cylinder(radius=0.310, length=0.225),
            mass=16.0,
            origin=Origin(xyz=(outboard_sign * 0.1125, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        )

    central_housing = model.part("central_housing")
    central_housing.visual(
        Cylinder(radius=0.085, length=0.140),
        origin=_spin_origin((0.0, 0.0, 0.0)),
        material=housing_steel,
        name="housing_shell",
    )
    central_housing.visual(
        Cylinder(radius=0.072, length=0.060),
        origin=_spin_origin((-0.088, 0.0, 0.0)),
        material=housing_steel,
        name="left_shoulder",
    )
    central_housing.visual(
        Cylinder(radius=0.072, length=0.060),
        origin=_spin_origin((0.088, 0.0, 0.0)),
        material=housing_steel,
        name="right_shoulder",
    )
    central_housing.visual(
        Cylinder(radius=0.064, length=0.090),
        origin=_spin_origin((0.0, 0.0, 0.0)),
        material=housing_steel,
        name="housing_band",
    )
    central_housing.visual(
        Cylinder(radius=0.046, length=0.085),
        origin=_spin_origin((-0.1475, 0.0, 0.0)),
        material=dark_steel,
        name="left_output_cup",
    )
    central_housing.visual(
        Cylinder(radius=0.046, length=0.085),
        origin=_spin_origin((0.1475, 0.0, 0.0)),
        material=dark_steel,
        name="right_output_cup",
    )
    central_housing.inertial = Inertial.from_geometry(
        Box((0.38, 0.19, 0.19)),
        mass=22.0,
        origin=Origin(),
    )

    add_halfshaft("left_plunge_shaft", -1.0)
    add_halfshaft("right_plunge_shaft", 1.0)
    add_wheel("left_wheel", -1.0)
    add_wheel("right_wheel", 1.0)

    model.articulation(
        "housing_to_left_shaft",
        ArticulationType.PRISMATIC,
        parent="central_housing",
        child="left_plunge_shaft",
        origin=Origin(xyz=(-0.190, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12000.0, velocity=0.25, lower=0.0, upper=0.012),
    )
    model.articulation(
        "housing_to_right_shaft",
        ArticulationType.PRISMATIC,
        parent="central_housing",
        child="right_plunge_shaft",
        origin=Origin(xyz=(0.190, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12000.0, velocity=0.25, lower=0.0, upper=0.012),
    )
    model.articulation(
        "left_wheel_spin",
        ArticulationType.REVOLUTE,
        parent="left_plunge_shaft",
        child="left_wheel",
        origin=Origin(xyz=(-0.500, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3500.0, velocity=30.0, lower=-(2.0 * math.pi), upper=2.0 * math.pi),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.REVOLUTE,
        parent="right_plunge_shaft",
        child="right_wheel",
        origin=Origin(xyz=(0.500, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3500.0, velocity=30.0, lower=-(2.0 * math.pi), upper=2.0 * math.pi),
    )
    return model


def run_tests() -> TestReport:
    _ensure_valid_cwd()
    ctx = TestContext(object_model, seed=0)
    central_housing = object_model.get_part("central_housing")
    left_shaft = object_model.get_part("left_plunge_shaft")
    right_shaft = object_model.get_part("right_plunge_shaft")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    left_plunge = object_model.get_articulation("housing_to_left_shaft")
    right_plunge = object_model.get_articulation("housing_to_right_shaft")
    left_wheel_spin = object_model.get_articulation("left_wheel_spin")
    right_wheel_spin = object_model.get_articulation("right_wheel_spin")

    left_output_cup = central_housing.get_visual("left_output_cup")
    right_output_cup = central_housing.get_visual("right_output_cup")
    left_inner_boot = left_shaft.get_visual("inner_boot")
    right_inner_boot = right_shaft.get_visual("inner_boot")
    left_flange = left_shaft.get_visual("stub_flange")
    right_flange = right_shaft.get_visual("stub_flange")
    left_mount_face = left_wheel.get_visual("mount_face")
    right_mount_face = right_wheel.get_visual("mount_face")

    ctx.allow_overlap(
        central_housing,
        left_shaft,
        elem_a=left_output_cup,
        elem_b=left_inner_boot,
        reason="The flexible rubber boot is simplified as a solid sleeve clamped over the left output cup.",
    )
    ctx.allow_overlap(
        central_housing,
        right_shaft,
        elem_a=right_output_cup,
        elem_b=right_inner_boot,
        reason="The flexible rubber boot is simplified as a solid sleeve clamped over the right output cup.",
    )

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    def expect_boot_engagement(prefix: str) -> None:
        ctx.expect_gap(
            central_housing,
            left_shaft,
            axis="x",
            max_gap=0.001,
            max_penetration=0.016,
            positive_elem=left_output_cup,
            negative_elem=left_inner_boot,
            name=f"{prefix}_left_inner_boot_clamps_to_output_cup",
        )
        ctx.expect_overlap(
            central_housing,
            left_shaft,
            axes="yz",
            min_overlap=0.090,
            elem_a=left_output_cup,
            elem_b=left_inner_boot,
            name=f"{prefix}_left_inner_boot_is_concentric_with_output_cup",
        )
        ctx.expect_gap(
            right_shaft,
            central_housing,
            axis="x",
            max_gap=0.001,
            max_penetration=0.016,
            positive_elem=right_inner_boot,
            negative_elem=right_output_cup,
            name=f"{prefix}_right_inner_boot_clamps_to_output_cup",
        )
        ctx.expect_overlap(
            right_shaft,
            central_housing,
            axes="yz",
            min_overlap=0.090,
            elem_a=right_inner_boot,
            elem_b=right_output_cup,
            name=f"{prefix}_right_inner_boot_is_concentric_with_output_cup",
        )

    def expect_wheel_mounts(prefix: str) -> None:
        ctx.expect_gap(
            left_shaft,
            left_wheel,
            axis="x",
            max_gap=0.001,
            max_penetration=1e-6,
            positive_elem=left_flange,
            negative_elem=left_mount_face,
            name=f"{prefix}_left_wheel_seats_on_stub_flange",
        )
        ctx.expect_overlap(
            left_shaft,
            left_wheel,
            axes="yz",
            min_overlap=0.108,
            elem_a=left_flange,
            elem_b=left_mount_face,
            name=f"{prefix}_left_wheel_mount_face_is_centered_on_stub_axle",
        )
        ctx.expect_gap(
            right_wheel,
            right_shaft,
            axis="x",
            max_gap=0.001,
            max_penetration=1e-6,
            positive_elem=right_mount_face,
            negative_elem=right_flange,
            name=f"{prefix}_right_wheel_seats_on_stub_flange",
        )
        ctx.expect_overlap(
            right_wheel,
            right_shaft,
            axes="yz",
            min_overlap=0.108,
            elem_a=right_mount_face,
            elem_b=right_flange,
            name=f"{prefix}_right_wheel_mount_face_is_centered_on_stub_axle",
        )

    expect_boot_engagement("rest")
    expect_wheel_mounts("rest")

    ctx.expect_origin_distance(left_wheel, left_shaft, axes="yz", max_dist=0.001, name="left_wheel_axis_matches_stub_axle")
    ctx.expect_origin_distance(right_wheel, right_shaft, axes="yz", max_dist=0.001, name="right_wheel_axis_matches_stub_axle")
    ctx.expect_origin_distance(left_wheel, right_wheel, axes="yz", max_dist=0.001, name="wheels_share_common_axle_height")

    ctx.check(
        "left_shaft_is_prismatic",
        left_plunge.articulation_type == ArticulationType.PRISMATIC and tuple(left_plunge.axis) == (-1.0, 0.0, 0.0),
        details=f"Expected left shaft plunge articulation on -x, got type={left_plunge.articulation_type} axis={left_plunge.axis}.",
    )
    ctx.check(
        "right_shaft_is_prismatic",
        right_plunge.articulation_type == ArticulationType.PRISMATIC and tuple(right_plunge.axis) == (1.0, 0.0, 0.0),
        details=f"Expected right shaft plunge articulation on +x, got type={right_plunge.articulation_type} axis={right_plunge.axis}.",
    )
    ctx.check(
        "wheel_spin_axes_are_stub_aligned",
        left_wheel_spin.articulation_type == ArticulationType.REVOLUTE
        and right_wheel_spin.articulation_type == ArticulationType.REVOLUTE
        and tuple(left_wheel_spin.axis) == (1.0, 0.0, 0.0)
        and tuple(right_wheel_spin.axis) == (1.0, 0.0, 0.0),
        details=(
            "Both wheel joints should revolve about the axle line; "
            f"left={left_wheel_spin.articulation_type}/{left_wheel_spin.axis}, "
            f"right={right_wheel_spin.articulation_type}/{right_wheel_spin.axis}."
        ),
    )

    left_travel = left_plunge.motion_limits.upper if left_plunge.motion_limits is not None and left_plunge.motion_limits.upper is not None else None
    right_travel = right_plunge.motion_limits.upper if right_plunge.motion_limits is not None and right_plunge.motion_limits.upper is not None else None
    ctx.check(
        "plunge_travel_is_realistic",
        left_travel is not None
        and right_travel is not None
        and 0.008 <= left_travel <= 0.020
        and 0.008 <= right_travel <= 0.020,
        details=f"Expected 8-20 mm plunge travel, got left={left_travel} right={right_travel}.",
    )

    housing_aabb = ctx.part_world_aabb(central_housing)
    left_wheel_aabb = ctx.part_world_aabb(left_wheel)
    right_wheel_aabb = ctx.part_world_aabb(right_wheel)
    if housing_aabb is None or left_wheel_aabb is None or right_wheel_aabb is None:
        ctx.fail("part_aabbs_available", "Expected world AABBs for the housing and both wheels.")
    else:
        overall_width = right_wheel_aabb[1][0] - left_wheel_aabb[0][0]
        wheel_diameter = left_wheel_aabb[1][1] - left_wheel_aabb[0][1]
        housing_diameter = housing_aabb[1][2] - housing_aabb[0][2]
        ctx.check(
            "overall_axle_width_is_realistic",
            1.75 <= overall_width <= 1.95,
            details=f"Expected overall axle width near passenger-car scale, got {overall_width:.3f} m.",
        )
        ctx.check(
            "wheel_diameter_is_realistic",
            0.58 <= wheel_diameter <= 0.66,
            details=f"Expected wheel diameter around 0.6 m, got {wheel_diameter:.3f} m.",
        )
        ctx.check(
            "cv_housing_diameter_is_realistic",
            0.15 <= housing_diameter <= 0.19,
            details=f"Expected central CV housing diameter near 0.17 m, got {housing_diameter:.3f} m.",
        )

    for articulation in (left_plunge, right_plunge, left_wheel_spin, right_wheel_spin):
        limits = articulation.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({articulation: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{articulation.name}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{articulation.name}_lower_no_floating")
            with ctx.pose({articulation: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{articulation.name}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{articulation.name}_upper_no_floating")

    with ctx.pose(
        {
            left_plunge: left_travel if left_travel is not None else 0.0,
            right_plunge: right_travel if right_travel is not None else 0.0,
            left_wheel_spin: math.pi,
            right_wheel_spin: -math.pi,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="combined_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="combined_pose_no_floating")
        expect_boot_engagement("combined_pose")
        expect_wheel_mounts("combined_pose")
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
