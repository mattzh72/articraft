from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, pi, sin, sqrt, tau

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

def _add_box(
    part,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    *,
    material,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
):
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_cylinder(
    part,
    name: str,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    *,
    material,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_ring_segments(
    part,
    *,
    prefix: str,
    representative_name: str,
    radius: float,
    y_center: float,
    radial_thickness: float,
    y_length: float,
    count: int,
    material,
    angle_offset: float = 0.0,
):
    tangent_length = 2.0 * radius * sin(pi / count) * 0.96
    for index in range(count):
        theta = (tau * index / count) + angle_offset
        name = representative_name if index == 0 else f"{prefix}_{index:02d}"
        _add_box(
            part,
            name,
            (tangent_length, y_length, radial_thickness),
            (radius * cos(theta), y_center, radius * sin(theta)),
            material=material,
            rpy=(0.0, (pi / 2.0) - theta, 0.0),
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_undershot_waterwheel", assets=ASSETS)

    oak_matte = model.material("oak_matte", rgba=(0.44, 0.31, 0.20, 1.0))
    oak_dark = model.material("oak_dark", rgba=(0.31, 0.23, 0.16, 1.0))
    iron_matte = model.material("iron_matte", rgba=(0.19, 0.20, 0.22, 1.0))
    steel_satin = model.material("steel_satin", rgba=(0.53, 0.56, 0.59, 1.0))
    graphite_satin = model.material("graphite_satin", rgba=(0.28, 0.30, 0.32, 1.0))
    bronze_satin = model.material("bronze_satin", rgba=(0.63, 0.47, 0.27, 1.0))

    axle_z = 1.54
    wheel_outer_radius = 1.48
    wheel_width = 0.84
    rim_center_y = 0.37
    paddle_radius = 1.31
    paddle_count = 12
    axle_radius = 0.09

    frame = model.part("frame")

    _add_box(
        frame,
        "left_shoe",
        (3.00, 0.18, 0.12),
        (0.0, -0.92, 0.06),
        material=oak_dark,
    )
    _add_box(
        frame,
        "right_shoe",
        (3.00, 0.18, 0.12),
        (0.0, 0.92, 0.06),
        material=oak_dark,
    )
    _add_box(
        frame,
        "lower_cross_front",
        (0.22, 1.88, 0.12),
        (-1.14, 0.0, 0.12),
        material=oak_matte,
    )
    _add_box(
        frame,
        "lower_cross_rear",
        (0.22, 1.88, 0.12),
        (1.14, 0.0, 0.12),
        material=oak_matte,
    )

    for side_name, side_y in (("left", -0.92), ("right", 0.92)):
        for post_name, post_x in (("front", -1.14), ("rear", 1.14)):
            _add_box(
                frame,
                f"{side_name}_{post_name}_post",
                (0.16, 0.16, 1.90),
                (post_x, side_y, 1.07),
                material=oak_matte,
            )

        _add_box(
            frame,
            f"{side_name}_ledger",
            (2.30, 0.16, 0.16),
            (0.0, side_y, 1.42),
            material=oak_dark,
        )
        _add_box(
            frame,
            f"{side_name}_cap_beam",
            (2.62, 0.16, 0.12),
            (0.0, side_y, 2.08),
            material=oak_dark,
        )

    brace_length = sqrt(0.72**2 + 1.40**2)
    brace_angle = atan2(1.40, 0.72)
    for side_name, side_y in (("left", -0.92), ("right", 0.92)):
        _add_box(
            frame,
            f"{side_name}_brace_forward",
            (brace_length, 0.05, 0.10),
            (-0.74, side_y, 0.82),
            material=oak_dark,
            rpy=(0.0, -brace_angle, 0.0),
        )
        _add_box(
            frame,
            f"{side_name}_brace_rear",
            (brace_length, 0.05, 0.10),
            (0.74, side_y, 0.82),
            material=oak_dark,
            rpy=(0.0, brace_angle, 0.0),
        )

    for side_name, side_y, housing_y, bolt_y in (
        ("left", -0.92, -0.72, -0.72),
        ("right", 0.92, 0.72, 0.72),
    ):
        _add_box(
            frame,
            f"{side_name}_bearing_arm",
            (0.46, 0.12, 0.14),
            (0.0, -0.82 if side_name == "left" else 0.82, axle_z - 0.22),
            material=oak_dark,
        )
        _add_box(
            frame,
            f"{side_name}_bearing_lower",
            (0.34, 0.18, 0.16),
            (0.0, housing_y, axle_z - 0.27),
            material=graphite_satin,
        )
        _add_box(
            frame,
            f"{side_name}_bearing_saddle",
            (0.22, 0.14, 0.16),
            (0.0, -0.67 if side_name == "left" else 0.67, axle_z - 0.17),
            material=bronze_satin,
        )
        _add_box(
            frame,
            f"{side_name}_bearing_cap",
            (0.26, 0.18, 0.06),
            (0.0, housing_y, axle_z + 0.13),
            material=steel_satin,
        )
        for strap_name, strap_x in (("front", -0.14), ("rear", 0.14)):
            _add_box(
                frame,
                f"{side_name}_bearing_{strap_name}_strap",
                (0.04, 0.18, 0.32),
                (strap_x, housing_y, axle_z + 0.01),
                material=steel_satin,
            )
        for bolt_name, bolt_x in (("front", -0.08), ("rear", 0.08)):
            _add_cylinder(
                frame,
                f"{side_name}_bolt_{bolt_name}",
                0.015,
                0.04,
                (bolt_x, bolt_y, axle_z + 0.14),
                material=graphite_satin,
            )

    wheel = model.part("wheel")

    _add_cylinder(
        wheel,
        "axle_shaft",
        axle_radius,
        1.46,
        (0.0, 0.0, 0.0),
        material=steel_satin,
        rpy=(pi / 2.0, 0.0, 0.0),
    )
    _add_cylinder(
        wheel,
        "hub_core",
        0.20,
        0.58,
        (0.0, 0.0, 0.0),
        material=oak_dark,
        rpy=(pi / 2.0, 0.0, 0.0),
    )
    for side_name, side_y in (("left", -0.32), ("right", 0.32)):
        _add_cylinder(
            wheel,
            f"{side_name}_spider_plate",
            0.28,
            0.12,
            (0.0, -0.22 if side_name == "left" else 0.22, 0.0),
            material=iron_matte,
            rpy=(pi / 2.0, 0.0, 0.0),
        )
        _add_cylinder(
            wheel,
            f"{side_name}_spindle_sleeve",
            0.11,
            0.12,
            (0.0, -0.50 if side_name == "left" else 0.50, 0.0),
            material=graphite_satin,
            rpy=(pi / 2.0, 0.0, 0.0),
        )
        _add_cylinder(
            wheel,
            f"{side_name}_bearing_collar",
            0.12,
            0.04,
            (0.0, -0.56 if side_name == "left" else 0.56, 0.0),
            material=steel_satin,
            rpy=(pi / 2.0, 0.0, 0.0),
        )

    _add_ring_segments(
        wheel,
        prefix="left_rim_segment",
        representative_name="left_rim",
        radius=1.41,
        y_center=-rim_center_y,
        radial_thickness=0.14,
        y_length=0.14,
        count=paddle_count,
        material=oak_matte,
        angle_offset=-pi / 2.0,
    )
    _add_ring_segments(
        wheel,
        prefix="right_rim_segment",
        representative_name="right_rim",
        radius=1.41,
        y_center=rim_center_y,
        radial_thickness=0.14,
        y_length=0.14,
        count=paddle_count,
        material=oak_matte,
        angle_offset=-pi / 2.0,
    )
    _add_ring_segments(
        wheel,
        prefix="left_tire_segment",
        representative_name="left_tire_band",
        radius=1.445,
        y_center=-0.44,
        radial_thickness=0.05,
        y_length=0.05,
        count=paddle_count,
        material=iron_matte,
        angle_offset=-pi / 2.0,
    )
    _add_ring_segments(
        wheel,
        prefix="right_tire_segment",
        representative_name="right_tire_band",
        radius=1.445,
        y_center=0.44,
        radial_thickness=0.05,
        y_length=0.05,
        count=paddle_count,
        material=iron_matte,
        angle_offset=-pi / 2.0,
    )

    for index in range(paddle_count):
        theta = (tau * index / paddle_count) - (pi / 2.0)
        radial_mid = 0.78
        x_mid = radial_mid * cos(theta)
        z_mid = radial_mid * sin(theta)
        x_paddle = paddle_radius * cos(theta)
        z_paddle = paddle_radius * sin(theta)

        for side_name, side_y in (("left", -0.25), ("right", 0.25)):
            _add_box(
                wheel,
                f"{side_name}_spoke_{index:02d}",
                (1.18, 0.18, 0.10),
                (x_mid, side_y, z_mid),
                material=oak_dark,
                rpy=(0.0, -theta, 0.0),
            )

        _add_box(
            wheel,
            f"paddle_{index:02d}",
            (0.22, wheel_width, 0.06),
            (x_paddle, 0.0, z_paddle),
            material=oak_matte,
            rpy=(0.0, -theta, 0.0),
        )
        _add_box(
            wheel,
            f"paddle_cleat_{index:02d}",
            (0.06, 0.78, 0.06),
            ((paddle_radius + 0.07) * cos(theta), 0.0, (paddle_radius + 0.07) * sin(theta)),
            material=oak_dark,
            rpy=(0.0, -theta, 0.0),
        )

    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, axle_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8000.0, velocity=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    wheel_spin = object_model.get_articulation("wheel_spin")

    axle_shaft = wheel.get_visual("axle_shaft")
    left_rim = wheel.get_visual("left_rim")
    right_rim = wheel.get_visual("right_rim")
    bottom_paddle = wheel.get_visual("paddle_00")
    left_saddle = frame.get_visual("left_bearing_saddle")
    right_saddle = frame.get_visual("right_bearing_saddle")
    left_housing = frame.get_visual("left_bearing_lower")
    right_housing = frame.get_visual("right_bearing_lower")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
    ctx.fail_if_isolated_parts(
        max_pose_samples=8,
        contact_tol=0.002,
        name="sampled_spin_support",
    )
    ctx.fail_if_articulation_overlaps(
        max_pose_samples=20,
        name="wheel_spin_clearance_sweep",
    )

    limits = wheel_spin.motion_limits
    axis_ok = tuple(round(value, 4) for value in wheel_spin.axis) == (0.0, 1.0, 0.0)
    limits_ok = limits is not None and limits.lower is None and limits.upper is None
    ctx.check("wheel_spin_axis", axis_ok, details=f"axis={wheel_spin.axis}")
    ctx.check(
        "wheel_spin_continuous_limits",
        limits_ok,
        details=f"limits={limits}",
    )

    with ctx.pose({wheel_spin: 0.0}):
        ctx.expect_contact(
            wheel,
            frame,
            elem_a=axle_shaft,
            elem_b=left_saddle,
            contact_tol=0.003,
            name="left_bearing_support_contact",
        )
        ctx.expect_contact(
            wheel,
            frame,
            elem_a=axle_shaft,
            elem_b=right_saddle,
            contact_tol=0.003,
            name="right_bearing_support_contact",
        )
        ctx.expect_overlap(
            wheel,
            frame,
            axes="x",
            elem_a=axle_shaft,
            elem_b=left_saddle,
            min_overlap=0.16,
            name="left_saddle_axle_x_alignment",
        )
        ctx.expect_overlap(
            wheel,
            frame,
            axes="x",
            elem_a=axle_shaft,
            elem_b=right_saddle,
            min_overlap=0.16,
            name="right_saddle_axle_x_alignment",
        )
        ctx.expect_gap(
            wheel,
            frame,
            axis="y",
            positive_elem=left_rim,
            negative_elem=left_housing,
            min_gap=0.10,
            max_gap=0.22,
            name="left_rim_to_housing_clearance",
        )
        ctx.expect_gap(
            frame,
            wheel,
            axis="y",
            positive_elem=right_housing,
            negative_elem=right_rim,
            min_gap=0.10,
            max_gap=0.22,
            name="right_rim_to_housing_clearance",
        )

        wheel_aabb = ctx.part_world_aabb(wheel)
        wheel_origin = ctx.part_world_position(wheel)
        paddle_aabb = ctx.part_element_world_aabb(wheel, elem=bottom_paddle)
        if wheel_aabb is None or wheel_origin is None or paddle_aabb is None:
            ctx.fail("wheel_pose_measurements", "could not evaluate wheel world measurements")
        else:
            wheel_bottom = wheel_aabb[0][2]
            axle_height = wheel_origin[2]
            paddle_bottom = paddle_aabb[0][2]
            ctx.check(
                "undershot_bottom_sits_low",
                0.02 <= wheel_bottom <= 0.14,
                details=f"wheel_bottom={wheel_bottom:.4f}",
            )
            ctx.check(
                "axle_height_is_low_flow_plausible",
                1.48 <= axle_height <= 1.60,
                details=f"axle_height={axle_height:.4f}",
            )
            ctx.check(
                "bottom_paddle_skims_low",
                0.10 <= paddle_bottom <= 0.28,
                details=f"paddle_bottom={paddle_bottom:.4f}",
            )

    for pose_index, angle in enumerate((0.0, pi / 6.0, pi / 3.0, pi / 2.0), start=1):
        with ctx.pose({wheel_spin: angle}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"wheel_pose_{pose_index}_no_overlap")
            ctx.fail_if_isolated_parts(
                contact_tol=0.002,
                name=f"wheel_pose_{pose_index}_supported",
            )
            ctx.expect_contact(
                wheel,
                frame,
                elem_a=axle_shaft,
                elem_b=left_saddle,
                contact_tol=0.003,
                name=f"left_contact_pose_{pose_index}",
            )
            ctx.expect_contact(
                wheel,
                frame,
                elem_a=axle_shaft,
                elem_b=right_saddle,
                contact_tol=0.003,
                name=f"right_contact_pose_{pose_index}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
