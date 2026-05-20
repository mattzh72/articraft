from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi, sin, cos

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


AXLE_Z = 1.15
WHEEL_HALF_WIDTH = 0.18
WHEEL_RIM_RADIUS = 0.86
WHEEL_RIM_TUBE = 0.03
SHAFT_RADIUS = 0.05


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _yz_polar(radius: float, angle: float) -> tuple[float, float, float]:
    return (0.0, radius * sin(angle), radius * cos(angle))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="masonry_overshot_waterwheel")

    stone = model.material("stone", rgba=(0.60, 0.58, 0.54, 1.0))
    dark_stone = model.material("dark_stone", rgba=(0.47, 0.45, 0.42, 1.0))
    aged_timber = model.material("aged_timber", rgba=(0.44, 0.31, 0.18, 1.0))
    wet_timber = model.material("wet_timber", rgba=(0.30, 0.22, 0.14, 1.0))
    iron = model.material("iron", rgba=(0.22, 0.23, 0.24, 1.0))
    steel = model.material("steel", rgba=(0.57, 0.59, 0.61, 1.0))
    gate_steel = model.material("gate_steel", rgba=(0.34, 0.37, 0.40, 1.0))

    outer_rim_mesh = _save_mesh(
        "waterwheel_outer_rim",
        TorusGeometry(
            radius=WHEEL_RIM_RADIUS,
            tube=WHEEL_RIM_TUBE,
            radial_segments=18,
            tubular_segments=72,
        ).rotate_y(pi / 2.0),
    )
    inner_rim_mesh = _save_mesh(
        "waterwheel_inner_rim",
        TorusGeometry(
            radius=0.54,
            tube=0.02,
            radial_segments=14,
            tubular_segments=56,
        ).rotate_y(pi / 2.0),
    )
    pulley_mesh = _save_mesh(
        "drive_pulley_body",
        LatheGeometry(
            [
                (0.045, -0.055),
                (0.075, -0.055),
                (0.135, -0.036),
                (0.104, -0.010),
                (0.090, 0.0),
                (0.104, 0.010),
                (0.135, 0.036),
                (0.075, 0.055),
                (0.045, 0.055),
            ],
            segments=56,
        ).rotate_y(pi / 2.0),
    )

    support = model.part("masonry_support")
    support.visual(
        Box((1.26, 1.94, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=stone,
        name="foundation_slab",
    )
    for side, x_sign in (("left", -1.0), ("right", 1.0)):
        x_pos = x_sign * 0.34
        support.visual(
            Box((0.24, 0.82, 0.88)),
            origin=Origin(xyz=(x_pos, 0.0, 0.62)),
            material=stone,
            name=f"{side}_pier_lower",
        )
        support.visual(
            Box((0.20, 0.22, 0.22)),
            origin=Origin(xyz=(x_pos, 0.24, 1.09)),
            material=dark_stone,
            name=f"{side}_pier_upper_front",
        )
        support.visual(
            Box((0.20, 0.22, 0.22)),
            origin=Origin(xyz=(x_pos, -0.24, 1.09)),
            material=dark_stone,
            name=f"{side}_pier_upper_rear",
        )
        support.visual(
            Box((0.16, 0.24, 0.08)),
            origin=Origin(xyz=(x_sign * 0.30, 0.0, 1.06)),
            material=dark_stone,
            name=f"{side}_bearing_pedestal",
        )
        support.visual(
            Box((0.12, 0.26, 0.22)),
            origin=Origin(xyz=(x_sign * 0.34, 0.28, 0.68)),
            material=stone,
            name=f"{side}_front_buttress",
        )
        support.visual(
            Box((0.12, 0.26, 0.22)),
            origin=Origin(xyz=(x_sign * 0.34, -0.28, 0.68)),
            material=stone,
            name=f"{side}_rear_buttress",
        )
    support.visual(
        Box((0.92, 0.36, 0.92)),
        origin=Origin(xyz=(0.0, 1.11, 0.64)),
        material=stone,
        name="headrace_wall_lower",
    )
    support.visual(
        Box((0.72, 0.36, 0.98)),
        origin=Origin(xyz=(0.0, 1.11, 1.59)),
        material=dark_stone,
        name="headrace_wall_upper",
    )
    support.inertial = Inertial.from_geometry(
        Box((1.26, 1.94, 2.08)),
        mass=8200.0,
        origin=Origin(xyz=(0.0, 0.0, 1.04)),
    )

    wheel = model.part("waterwheel")
    wheel.visual(
        Cylinder(radius=SHAFT_RADIUS, length=0.82),
        origin=Origin(xyz=(0.05, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=iron,
        name="shaft",
    )
    wheel.visual(
        Cylinder(radius=0.14, length=0.40),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=iron,
        name="hub",
    )
    for side, x_pos in (("left", -WHEEL_HALF_WIDTH), ("right", WHEEL_HALF_WIDTH)):
        wheel.visual(outer_rim_mesh, origin=Origin(xyz=(x_pos, 0.0, 0.0)), material=aged_timber, name=f"{side}_outer_rim")
        wheel.visual(inner_rim_mesh, origin=Origin(xyz=(x_pos, 0.0, 0.0)), material=wet_timber, name=f"{side}_inner_rim")
    for spoke_index in range(8):
        angle = (2.0 * pi * spoke_index) / 8.0
        _, y_pos, z_pos = _yz_polar(0.46, angle)
        wheel.visual(
            Box((0.34, 0.05, 0.82)),
            origin=Origin(xyz=(0.0, y_pos, z_pos), rpy=(angle, 0.0, 0.0)),
            material=aged_timber,
            name=f"spoke_{spoke_index:02d}",
        )
    for bucket_index in range(16):
        angle = (2.0 * pi * bucket_index) / 16.0
        _, tread_y, tread_z = _yz_polar(0.84, angle)
        _, back_y, back_z = _yz_polar(0.76, angle + 0.12)
        wheel.visual(
            Box((0.36, 0.18, 0.03)),
            origin=Origin(xyz=(0.0, tread_y, tread_z), rpy=(angle, 0.0, 0.0)),
            material=wet_timber,
            name=f"bucket_tread_{bucket_index:02d}",
        )
        wheel.visual(
            Box((0.36, 0.04, 0.32)),
            origin=Origin(xyz=(0.0, back_y, back_z), rpy=(angle + 0.12, 0.0, 0.0)),
            material=aged_timber,
            name=f"bucket_back_{bucket_index:02d}",
        )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.92, length=0.82),
        mass=540.0,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    pulley = model.part("drive_pulley")
    pulley.visual(
        Cylinder(radius=SHAFT_RADIUS, length=0.15),
        origin=Origin(xyz=(0.075, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=iron,
        name="pulley_stub",
    )
    pulley.visual(
        pulley_mesh,
        origin=Origin(xyz=(0.15, 0.0, 0.0)),
        material=aged_timber,
        name="pulley_wheel",
    )
    pulley.inertial = Inertial.from_geometry(
        Cylinder(radius=0.14, length=0.22),
        mass=28.0,
        origin=Origin(xyz=(0.11, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
    )

    chute = model.part("feed_chute")
    chute.visual(
        Box((0.26, 0.84, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=aged_timber,
        name="chute_floor",
    )
    chute.visual(
        Box((0.03, 0.84, 0.22)),
        origin=Origin(xyz=(-0.115, 0.0, 0.11)),
        material=aged_timber,
        name="left_wall",
    )
    chute.visual(
        Box((0.03, 0.84, 0.22)),
        origin=Origin(xyz=(0.115, 0.0, 0.11)),
        material=aged_timber,
        name="right_wall",
    )
    chute.visual(
        Box((0.26, 0.10, 0.24)),
        origin=Origin(xyz=(0.0, 0.37, 0.12)),
        material=aged_timber,
        name="rear_header",
    )
    chute.visual(
        Box((0.03, 0.08, 0.46)),
        origin=Origin(xyz=(-0.115, -0.35, 0.24)),
        material=iron,
        name="left_gate_guide",
    )
    chute.visual(
        Box((0.03, 0.08, 0.46)),
        origin=Origin(xyz=(0.115, -0.35, 0.24)),
        material=iron,
        name="right_gate_guide",
    )
    chute.visual(
        Box((0.26, 0.08, 0.05)),
        origin=Origin(xyz=(0.0, -0.35, 0.025)),
        material=dark_stone,
        name="gate_sill",
    )
    chute.visual(
        Box((0.26, 0.08, 0.05)),
        origin=Origin(xyz=(0.0, -0.35, 0.455)),
        material=iron,
        name="gate_cap",
    )
    chute.inertial = Inertial.from_geometry(
        Box((0.26, 0.84, 0.48)),
        mass=72.0,
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
    )

    gate = model.part("chute_gate")
    gate.visual(
        Box((0.20, 0.014, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=gate_steel,
        name="gate_panel",
    )
    gate.visual(
        Cylinder(radius=0.009, length=0.10),
        origin=Origin(xyz=(0.0, -0.012, 0.165), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="gate_handle",
    )
    gate.inertial = Inertial.from_geometry(
        Box((0.20, 0.03, 0.22)),
        mass=16.0,
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
    )

    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, AXLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2200.0, velocity=2.2),
    )
    model.articulation(
        "wheel_to_pulley",
        ArticulationType.FIXED,
        parent=wheel,
        child=pulley,
        origin=Origin(xyz=(0.46, 0.0, 0.0)),
    )
    model.articulation(
        "support_to_chute",
        ArticulationType.FIXED,
        parent=support,
        child=chute,
        origin=Origin(xyz=(0.0, 0.87, 2.08)),
    )
    model.articulation(
        "gate_slide",
        ArticulationType.PRISMATIC,
        parent=chute,
        child=gate,
        origin=Origin(xyz=(0.0, -0.35, 0.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.20, lower=0.0, upper=0.16),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("masonry_support")
    wheel = object_model.get_part("waterwheel")
    pulley = object_model.get_part("drive_pulley")
    chute = object_model.get_part("feed_chute")
    gate = object_model.get_part("chute_gate")

    wheel_spin = object_model.get_articulation("wheel_spin")
    gate_slide = object_model.get_articulation("gate_slide")
    wheel_to_pulley = object_model.get_articulation("wheel_to_pulley")

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

    ctx.expect_contact(wheel, support, name="wheel_shaft_supported_by_masonry_bearings")
    ctx.expect_contact(pulley, wheel, name="pulley_fixed_to_outboard_shaft_end")
    ctx.expect_contact(chute, support, name="chute_resting_on_headdrace_wall")
    ctx.expect_contact(gate, chute, name="gate_seated_in_lower_guide_stop")

    ctx.expect_origin_gap(chute, wheel, axis="z", min_gap=0.80, max_gap=1.05, name="chute_above_wheel_axis")
    ctx.expect_origin_gap(pulley, wheel, axis="x", min_gap=0.44, max_gap=0.48, name="pulley_is_outboard_of_wheel")

    ctx.check(
        "wheel_joint_is_horizontal_continuous_spin",
        wheel_spin.articulation_type == ArticulationType.CONTINUOUS and wheel_spin.axis == (1.0, 0.0, 0.0),
        details=f"type={wheel_spin.articulation_type}, axis={wheel_spin.axis}",
    )
    ctx.check(
        "pulley_moves_with_wheel_via_fixed_joint",
        wheel_to_pulley.articulation_type == ArticulationType.FIXED,
        details=f"type={wheel_to_pulley.articulation_type}",
    )
    ctx.check(
        "gate_joint_is_vertical_prismatic",
        gate_slide.articulation_type == ArticulationType.PRISMATIC and gate_slide.axis == (0.0, 0.0, 1.0),
        details=f"type={gate_slide.articulation_type}, axis={gate_slide.axis}",
    )

    gate_rest = ctx.part_world_position(gate)
    if gate_rest is None:
        ctx.fail("gate_world_position_available", "Could not resolve chute_gate world position in rest pose.")
    else:
        with ctx.pose({gate_slide: 0.16}):
            gate_open = ctx.part_world_position(gate)
            if gate_open is None:
                ctx.fail("gate_world_position_available_open", "Could not resolve chute_gate world position in open pose.")
            else:
                ctx.check(
                    "gate_rises_vertically",
                    gate_open[2] > gate_rest[2] + 0.15,
                    details=f"rest={gate_rest}, open={gate_open}",
                )
            ctx.expect_contact(gate, chute, name="gate_remains_captured_at_upper_stop")
            ctx.expect_within(gate, chute, axes="xy", margin=0.0, name="gate_stays_within_guide_envelope")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
