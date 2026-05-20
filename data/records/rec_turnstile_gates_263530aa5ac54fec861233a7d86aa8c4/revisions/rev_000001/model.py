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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _element_center(aabb):
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (
        0.5 * (min_x + max_x),
        0.5 * (min_y + max_y),
        0.5 * (min_z + max_z),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_turnstile")

    painted_steel = model.material("painted_steel", rgba=(0.33, 0.36, 0.40, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    stainless = model.material("stainless", rgba=(0.74, 0.76, 0.79, 1.0))
    polymer = model.material("polymer", rgba=(0.74, 0.62, 0.22, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.34, 0.30, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=dark_steel,
        name="floor_plate",
    )
    base.visual(
        Box((0.018, 0.22, 0.88)),
        origin=Origin(xyz=(-0.161, 0.0, 0.46)),
        material=painted_steel,
        name="left_shell_wall",
    )
    base.visual(
        Box((0.018, 0.22, 0.88)),
        origin=Origin(xyz=(0.161, 0.0, 0.46)),
        material=painted_steel,
        name="right_shell_wall",
    )
    base.visual(
        Box((0.304, 0.018, 0.88)),
        origin=Origin(xyz=(0.0, -0.101, 0.46)),
        material=painted_steel,
        name="rear_shell_wall",
    )
    base.visual(
        Box((0.03, 0.012, 0.88)),
        origin=Origin(xyz=(-0.103, 0.104, 0.46)),
        material=painted_steel,
        name="front_left_stile",
    )
    base.visual(
        Box((0.03, 0.012, 0.88)),
        origin=Origin(xyz=(0.103, 0.104, 0.46)),
        material=painted_steel,
        name="front_right_stile",
    )
    base.visual(
        Box((0.176, 0.012, 0.16)),
        origin=Origin(xyz=(0.0, 0.104, 0.10)),
        material=painted_steel,
        name="front_lower_rail",
    )
    base.visual(
        Box((0.176, 0.012, 0.16)),
        origin=Origin(xyz=(0.0, 0.104, 0.80)),
        material=painted_steel,
        name="front_upper_rail",
    )
    base.visual(
        Box((0.074, 0.20, 0.05)),
        origin=Origin(xyz=(-0.125, 0.0, 0.875)),
        material=dark_steel,
        name="left_upper_frame",
    )
    base.visual(
        Box((0.074, 0.20, 0.05)),
        origin=Origin(xyz=(0.125, 0.0, 0.875)),
        material=dark_steel,
        name="right_upper_frame",
    )
    base.visual(
        Box((0.034, 0.08, 0.10)),
        origin=Origin(xyz=(-0.074, 0.0, 0.85)),
        material=dark_steel,
        name="left_support_rib",
    )
    base.visual(
        Box((0.034, 0.08, 0.10)),
        origin=Origin(xyz=(0.074, 0.0, 0.85)),
        material=dark_steel,
        name="right_support_rib",
    )
    base.visual(
        Box((0.032, 0.08, 0.05)),
        origin=Origin(xyz=(-0.041, 0.0, 0.80)),
        material=dark_steel,
        name="left_support_block",
    )
    base.visual(
        Box((0.032, 0.08, 0.05)),
        origin=Origin(xyz=(0.041, 0.0, 0.80)),
        material=dark_steel,
        name="right_support_block",
    )
    for sx in (-0.11, 0.11):
        for sy in (-0.09, 0.09):
            lug_name = f"anchor_lug_{'p' if sx > 0 else 'n'}x_{'p' if sy > 0 else 'n'}y"
            bolt_name = f"anchor_bolt_{'p' if sx > 0 else 'n'}x_{'p' if sy > 0 else 'n'}y"
            base.visual(
                Box((0.06, 0.05, 0.01)),
                origin=Origin(xyz=(sx, sy, 0.025)),
                material=dark_steel,
                name=lug_name,
            )
            base.visual(
                Cylinder(radius=0.012, length=0.03),
                origin=Origin(xyz=(sx, sy, 0.035)),
                material=stainless,
                name=bolt_name,
            )
    base.inertial = Inertial.from_geometry(
        Box((0.34, 0.30, 1.02)),
        mass=145.0,
        origin=Origin(xyz=(0.0, 0.0, 0.51)),
    )

    service_door = model.part("service_door")
    service_door.visual(
        Box((0.17, 0.01, 0.52)),
        origin=Origin(xyz=(0.0, 0.005, 0.0)),
        material=painted_steel,
        name="door_panel",
    )
    service_door.visual(
        Box((0.018, 0.012, 0.42)),
        origin=Origin(xyz=(-0.076, 0.006, 0.0)),
        material=dark_steel,
        name="hinge_strap",
    )
    service_door.visual(
        Cylinder(radius=0.010, length=0.08),
        origin=Origin(xyz=(-0.078, 0.006, -0.18)),
        material=stainless,
        name="hinge_barrel_lower",
    )
    service_door.visual(
        Cylinder(radius=0.010, length=0.08),
        origin=Origin(xyz=(-0.078, 0.006, 0.18)),
        material=stainless,
        name="hinge_barrel_upper",
    )
    service_door.visual(
        Box((0.018, 0.020, 0.14)),
        origin=Origin(xyz=(0.053, 0.018, 0.0)),
        material=dark_steel,
        name="pull_handle",
    )
    service_door.visual(
        Cylinder(radius=0.011, length=0.028),
        origin=Origin(xyz=(0.053, 0.033, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="handle_grip",
    )
    service_door.inertial = Inertial.from_geometry(
        Box((0.17, 0.04, 0.52)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.02, 0.0)),
    )

    bearing_core = model.part("bearing_core")
    bearing_core.visual(
        Cylinder(radius=0.024, length=0.30),
        origin=Origin(),
        material=stainless,
        name="spindle",
    )
    bearing_core.visual(
        Cylinder(radius=0.058, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, -0.14)),
        material=dark_steel,
        name="lower_support_collar",
    )
    bearing_core.visual(
        Cylinder(radius=0.046, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=dark_steel,
        name="upper_retainer",
    )
    bearing_core.visual(
        Cylinder(radius=0.034, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, -0.095)),
        material=stainless,
        name="lower_thrust_face",
    )
    bearing_core.inertial = Inertial.from_geometry(
        Cylinder(radius=0.060, length=0.32),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, -0.02)),
    )

    wear_ring_lower = model.part("wear_ring_lower")
    wear_ring_lower.visual(
        _mesh(
            "lower_wear_ring",
            LatheGeometry.from_shell_profiles(
                [
                    (0.030, -0.015),
                    (0.032, -0.012),
                    (0.032, 0.012),
                    (0.030, 0.015),
                ],
                [
                    (0.025, -0.015),
                    (0.025, 0.015),
                ],
                segments=48,
                start_cap="flat",
                end_cap="flat",
            ),
        ),
        material=polymer,
        name="ring_shell",
    )
    wear_ring_lower.inertial = Inertial.from_geometry(
        Cylinder(radius=0.032, length=0.030),
        mass=0.15,
    )

    wear_ring_upper = model.part("wear_ring_upper")
    wear_ring_upper.visual(
        _mesh(
            "upper_wear_ring",
            LatheGeometry.from_shell_profiles(
                [
                    (0.030, -0.015),
                    (0.032, -0.012),
                    (0.032, 0.012),
                    (0.030, 0.015),
                ],
                [
                    (0.025, -0.015),
                    (0.025, 0.015),
                ],
                segments=48,
                start_cap="flat",
                end_cap="flat",
            ),
        ),
        material=polymer,
        name="ring_shell",
    )
    wear_ring_upper.inertial = Inertial.from_geometry(
        Cylinder(radius=0.032, length=0.030),
        mass=0.15,
    )

    rotor = model.part("rotor")
    rotor.visual(
        _mesh(
            "rotor_hub_shell",
            LatheGeometry.from_shell_profiles(
                [
                    (0.056, -0.098),
                    (0.079, -0.090),
                    (0.090, -0.062),
                    (0.094, -0.020),
                    (0.094, 0.020),
                    (0.090, 0.062),
                    (0.079, 0.090),
                    (0.056, 0.098),
                ],
                [
                    (0.037, -0.094),
                    (0.037, 0.094),
                ],
                segments=72,
                start_cap="flat",
                end_cap="flat",
            ),
        ),
        material=painted_steel,
        name="hub_shell",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        arm_name = ("front", "left_rear", "right_rear")[index]
        rotor.visual(
            Box((0.085, 0.060, 0.110)),
            origin=Origin(
                xyz=(0.122 * cos_a, 0.122 * sin_a, 0.0),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_steel,
            name=f"clamp_{arm_name}",
        )
        rotor.visual(
            Cylinder(radius=0.020, length=0.48),
            origin=Origin(
                xyz=(0.335 * cos_a, 0.335 * sin_a, 0.0),
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=stainless,
            name=f"arm_{arm_name}",
        )
        rotor.visual(
            Cylinder(radius=0.028, length=0.05),
            origin=Origin(
                xyz=(0.600 * cos_a, 0.600 * sin_a, 0.0),
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=rubber,
            name=f"bumper_{arm_name}",
        )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.62, length=0.20),
        mass=18.0,
        origin=Origin(),
    )

    model.articulation(
        "base_to_service_door",
        ArticulationType.FIXED,
        parent=base,
        child=service_door,
        origin=Origin(xyz=(-0.003, 0.110, 0.44)),
    )
    model.articulation(
        "base_to_bearing_core",
        ArticulationType.FIXED,
        parent=base,
        child=bearing_core,
        origin=Origin(xyz=(0.0, 0.0, 0.995)),
    )
    model.articulation(
        "bearing_core_to_lower_wear_ring",
        ArticulationType.FIXED,
        parent=bearing_core,
        child=wear_ring_lower,
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
    )
    model.articulation(
        "bearing_core_to_upper_wear_ring",
        ArticulationType.FIXED,
        parent=bearing_core,
        child=wear_ring_upper,
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
    )
    model.articulation(
        "base_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.995)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    service_door = object_model.get_part("service_door")
    bearing_core = object_model.get_part("bearing_core")
    wear_ring_lower = object_model.get_part("wear_ring_lower")
    wear_ring_upper = object_model.get_part("wear_ring_upper")
    rotor = object_model.get_part("rotor")
    rotor_joint = object_model.get_articulation("base_to_rotor")
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

    ctx.expect_contact(service_door, base, elem_a="door_panel", elem_b="front_lower_rail", name="door_seats_on_frame")
    ctx.expect_contact(
        bearing_core,
        base,
        elem_a="lower_support_collar",
        elem_b="left_support_block",
        name="bearing_core_supported_on_left_block",
    )
    ctx.expect_contact(
        bearing_core,
        base,
        elem_a="lower_support_collar",
        elem_b="right_support_block",
        name="bearing_core_supported_on_right_block",
    )
    ctx.expect_contact(
        wear_ring_lower,
        bearing_core,
        elem_a="ring_shell",
        elem_b="lower_thrust_face",
        name="lower_wear_ring_mounted",
    )
    ctx.expect_contact(
        wear_ring_upper,
        bearing_core,
        elem_a="ring_shell",
        elem_b="upper_retainer",
        name="upper_wear_ring_mounted",
    )
    ctx.expect_within(
        bearing_core,
        rotor,
        axes="xy",
        inner_elem="spindle",
        outer_elem="hub_shell",
        margin=0.012,
        name="rotor_hub_wraps_supported_spindle",
    )
    ctx.check(
        "rotor_joint_is_vertical_continuous",
        rotor_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(rotor_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={rotor_joint.articulation_type} axis={rotor_joint.axis}",
    )

    arm_front_rest = ctx.part_element_world_aabb(rotor, elem="arm_front")
    rest_center = _element_center(arm_front_rest)
    ctx.check(
        "front_arm_faces_entry_lane_at_rest",
        rest_center is not None and rest_center[0] > 0.30 and abs(rest_center[1]) < 0.03,
        details=f"center={rest_center}",
    )

    with ctx.pose({rotor_joint: math.pi / 2.0}):
        arm_front_quarter = ctx.part_element_world_aabb(rotor, elem="arm_front")
        quarter_center = _element_center(arm_front_quarter)
        ctx.check(
            "front_arm_rotates_around_spindle",
            quarter_center is not None and quarter_center[1] > 0.30 and abs(quarter_center[0]) < 0.04,
            details=f"center={quarter_center}",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_quarter_turn")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
