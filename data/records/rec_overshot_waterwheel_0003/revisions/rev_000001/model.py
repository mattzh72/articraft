from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="timber_mill_waterwheel", assets=ASSETS)

    timber_dark = model.material("timber_dark", rgba=(0.39, 0.27, 0.15, 1.0))
    timber_light = model.material("timber_light", rgba=(0.54, 0.39, 0.22, 1.0))
    wet_wood = model.material("wet_wood", rgba=(0.31, 0.24, 0.15, 1.0))
    stone = model.material("stone", rgba=(0.56, 0.56, 0.54, 1.0))
    iron = model.material("iron", rgba=(0.23, 0.24, 0.26, 1.0))

    axle_radius = 0.018
    wheel_radius = 0.30
    rim_inner_radius = 0.23
    wheel_width = 0.22
    side_ring_thickness = 0.024
    side_ring_offset = 0.098
    hub_radius = 0.060
    hub_length = 0.180
    axle_length = 0.600
    axle_height = 0.390

    def _save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(name))

    def _annular_ring_mesh(
        *,
        outer_radius: float,
        inner_radius: float,
        thickness: float,
        radial_segments: int = 56,
    ):
        outer = CylinderGeometry(
            radius=outer_radius,
            height=thickness,
            radial_segments=radial_segments,
        )
        inner = CylinderGeometry(
            radius=inner_radius,
            height=thickness + 0.006,
            radial_segments=radial_segments,
        )
        return boolean_difference(outer, inner).rotate_y(math.pi / 2.0)

    ring_mesh = _save_mesh(
        "waterwheel_side_ring.obj",
        _annular_ring_mesh(
            outer_radius=wheel_radius,
            inner_radius=rim_inner_radius,
            thickness=side_ring_thickness,
        ),
    )

    base = model.part("base")
    base.visual(
        Box((0.840, 0.740, 0.050)),
        origin=Origin(xyz=(0.000, 0.000, 0.025)),
        material=stone,
        name="foundation",
    )
    base.visual(
        Box((0.560, 0.220, 0.020)),
        origin=Origin(xyz=(0.000, 0.000, 0.060)),
        material=stone,
        name="trough_floor",
    )
    base.visual(
        Box((0.560, 0.025, 0.050)),
        origin=Origin(xyz=(0.000, 0.130, 0.075)),
        material=stone,
        name="trough_wall_front",
    )
    base.visual(
        Box((0.560, 0.025, 0.050)),
        origin=Origin(xyz=(0.000, -0.130, 0.075)),
        material=stone,
        name="trough_wall_back",
    )
    base.visual(
        Box((0.030, 0.235, 0.050)),
        origin=Origin(xyz=(0.265, 0.000, 0.075)),
        material=stone,
        name="trough_end_right",
    )
    base.visual(
        Box((0.030, 0.235, 0.050)),
        origin=Origin(xyz=(-0.265, 0.000, 0.075)),
        material=stone,
        name="trough_end_left",
    )
    base.visual(
        Box((0.080, 0.160, 0.280)),
        origin=Origin(xyz=(-0.280, 0.000, 0.190)),
        material=timber_dark,
        name="left_upright",
    )
    base.visual(
        Box((0.080, 0.160, 0.280)),
        origin=Origin(xyz=(0.280, 0.000, 0.190)),
        material=timber_dark,
        name="right_upright",
    )
    base.visual(
        Box((0.070, 0.110, 0.042)),
        origin=Origin(xyz=(-0.280, 0.000, axle_height - axle_radius - 0.021)),
        material=timber_light,
        name="left_bearing_saddle",
    )
    base.visual(
        Box((0.070, 0.110, 0.042)),
        origin=Origin(xyz=(0.280, 0.000, axle_height - axle_radius - 0.021)),
        material=timber_light,
        name="right_bearing_saddle",
    )
    base.visual(
        Box((0.070, 0.022, 0.120)),
        origin=Origin(xyz=(-0.280, 0.044, 0.390)),
        material=timber_light,
        name="left_bearing_cheek_front",
    )
    base.visual(
        Box((0.070, 0.022, 0.120)),
        origin=Origin(xyz=(-0.280, -0.044, 0.390)),
        material=timber_light,
        name="left_bearing_cheek_back",
    )
    base.visual(
        Box((0.070, 0.022, 0.120)),
        origin=Origin(xyz=(0.280, 0.044, 0.390)),
        material=timber_light,
        name="right_bearing_cheek_front",
    )
    base.visual(
        Box((0.070, 0.022, 0.120)),
        origin=Origin(xyz=(0.280, -0.044, 0.390)),
        material=timber_light,
        name="right_bearing_cheek_back",
    )
    base.visual(
        Box((0.560, 0.100, 0.020)),
        origin=Origin(xyz=(0.000, 0.000, 0.060)),
        material=wet_wood,
        name="water_channel",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.840, 0.740, 0.460)),
        mass=36.0,
        origin=Origin(xyz=(0.000, 0.000, 0.230)),
    )

    wheel = model.part("wheel")
    wheel.visual(
        Cylinder(radius=axle_radius, length=axle_length),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="axle",
    )
    wheel.visual(
        Cylinder(radius=hub_radius, length=hub_length),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=timber_dark,
        name="hub",
    )
    wheel.visual(
        ring_mesh,
        origin=Origin(xyz=(-side_ring_offset, 0.000, 0.000)),
        material=timber_dark,
        name="left_ring",
    )
    wheel.visual(
        ring_mesh,
        origin=Origin(xyz=(side_ring_offset, 0.000, 0.000)),
        material=timber_dark,
        name="right_ring",
    )

    spoke_length = rim_inner_radius - hub_radius
    spoke_center_radius = hub_radius + (spoke_length * 0.5)
    for side_name, x_pos in (("left", -side_ring_offset), ("right", side_ring_offset)):
        for index in range(8):
            angle = (math.tau * index) / 8.0
            wheel.visual(
                Box((0.020, 0.028, spoke_length)),
                origin=Origin(
                    xyz=(
                        x_pos,
                        spoke_center_radius * math.cos(angle),
                        spoke_center_radius * math.sin(angle),
                    ),
                    rpy=(angle - (math.pi / 2.0), 0.0, 0.0),
                ),
                material=timber_light,
                name=f"{side_name}_spoke_{index:02d}",
            )

    paddle_length_x = (2.0 * side_ring_offset) - side_ring_thickness
    paddle_tangent = 0.040
    paddle_depth = wheel_radius - rim_inner_radius
    paddle_center_radius = rim_inner_radius + (paddle_depth * 0.5)
    for index in range(12):
        angle = (math.tau * index) / 12.0 + (math.pi / 2.0)
        wheel.visual(
            Box((paddle_length_x, paddle_tangent, paddle_depth)),
            origin=Origin(
                xyz=(
                    0.000,
                    paddle_center_radius * math.cos(angle),
                    paddle_center_radius * math.sin(angle),
                ),
                rpy=(angle - (math.pi / 2.0), 0.0, 0.0),
            ),
            material=wet_wood,
            name=f"paddle_{index:02d}",
        )

    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=axle_length),
        mass=18.0,
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "wheel_axle",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=wheel,
        origin=Origin(xyz=(0.000, 0.000, axle_height)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=4.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    wheel = object_model.get_part("wheel")
    axle_joint = object_model.get_articulation("wheel_axle")

    foundation = base.get_visual("foundation")
    trough_floor = base.get_visual("trough_floor")
    left_saddle = base.get_visual("left_bearing_saddle")
    right_saddle = base.get_visual("right_bearing_saddle")
    axle = wheel.get_visual("axle")
    left_ring = wheel.get_visual("left_ring")
    right_ring = wheel.get_visual("right_ring")
    paddle_00 = wheel.get_visual("paddle_00")

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

    ctx.check(
        "waterwheel_parts_present",
        base is not None and wheel is not None and axle_joint is not None,
        "Expected base part, wheel part, and wheel_axle articulation.",
    )
    ctx.check(
        "wheel_axle_is_continuous_horizontal",
        axle_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(axle_joint.axis) == (1.0, 0.0, 0.0)
        and getattr(axle_joint.motion_limits, "lower", None) is None
        and getattr(axle_joint.motion_limits, "upper", None) is None,
        "Wheel should rotate continuously about a horizontal X-axis axle.",
    )

    ctx.expect_origin_distance(
        wheel,
        base,
        axes="xy",
        max_dist=0.001,
        name="wheel_centered_over_trough",
    )
    ctx.expect_within(
        wheel,
        base,
        axes="xy",
        inner_elem=left_ring,
        outer_elem=foundation,
        name="left_ring_within_foundation_plan",
    )
    ctx.expect_within(
        wheel,
        base,
        axes="xy",
        inner_elem=right_ring,
        outer_elem=foundation,
        name="right_ring_within_foundation_plan",
    )
    ctx.expect_gap(
        wheel,
        base,
        axis="z",
        positive_elem=left_ring,
        negative_elem=trough_floor,
        min_gap=0.015,
        max_gap=0.030,
        name="wheel_clears_trough_floor",
    )
    ctx.expect_gap(
        wheel,
        base,
        axis="x",
        positive_elem=left_ring,
        negative_elem=left_saddle,
        min_gap=0.120,
        max_gap=0.150,
        name="left_ring_clear_of_left_support",
    )
    ctx.expect_gap(
        base,
        wheel,
        axis="x",
        positive_elem=right_saddle,
        negative_elem=right_ring,
        min_gap=0.120,
        max_gap=0.150,
        name="right_ring_clear_of_right_support",
    )
    ctx.expect_contact(
        wheel,
        base,
        elem_a=axle,
        elem_b=left_saddle,
        name="axle_seated_on_left_saddle",
    )
    ctx.expect_contact(
        wheel,
        base,
        elem_a=axle,
        elem_b=right_saddle,
        name="axle_seated_on_right_saddle",
    )

    def _aabb_center(bounds):
        if bounds is None:
            return None
        lower, upper = bounds
        return tuple((lo + hi) * 0.5 for lo, hi in zip(lower, upper))

    rest_center = _aabb_center(ctx.part_element_world_aabb(wheel, elem=paddle_00))
    with ctx.pose({axle_joint: math.pi / 2.0}):
        ctx.expect_gap(
            wheel,
            base,
            axis="z",
            positive_elem=left_ring,
            negative_elem=trough_floor,
            min_gap=0.015,
            max_gap=0.030,
            name="wheel_clears_trough_floor_quarter_turn",
        )
        ctx.expect_contact(
            wheel,
            base,
            elem_a=axle,
            elem_b=left_saddle,
            name="axle_stays_on_left_saddle_quarter_turn",
        )
        ctx.expect_contact(
            wheel,
            base,
            elem_a=axle,
            elem_b=right_saddle,
            name="axle_stays_on_right_saddle_quarter_turn",
        )
        quarter_center = _aabb_center(ctx.part_element_world_aabb(wheel, elem=paddle_00))

    ctx.check(
        "paddle_moves_in_yz_when_wheel_turns",
        rest_center is not None
        and quarter_center is not None
        and abs(rest_center[1] - quarter_center[1]) > 0.20
        and abs(rest_center[2] - quarter_center[2]) > 0.20,
        "A bucket paddle should sweep through the wheel plane when the axle rotates.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
