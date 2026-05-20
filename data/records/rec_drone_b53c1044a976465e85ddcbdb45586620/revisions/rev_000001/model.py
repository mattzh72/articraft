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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilt_rotor_quadplane")

    airframe = model.material("airframe", rgba=(0.76, 0.78, 0.81, 1.0))
    canopy = model.material("canopy", rgba=(0.22, 0.31, 0.43, 1.0))
    motor = model.material("motor", rgba=(0.20, 0.21, 0.23, 1.0))
    prop = model.material("prop", rgba=(0.07, 0.07, 0.08, 1.0))
    accent = model.material("accent", rgba=(0.86, 0.41, 0.12, 1.0))

    wing_span = 1.46
    wing_chord = 0.34
    wing_thickness = 0.04

    front_nacelle_y = 0.46
    rear_pod_x = -0.08
    rear_pod_y = 0.39
    rear_pod_top_z = 0.20
    rudder_hinge_x = -0.79
    rudder_hinge_z = 0.08

    body = model.part("body")
    body.visual(
        Box((wing_chord, wing_span, wing_thickness)),
        origin=Origin(xyz=(0.0, 0.0, wing_thickness / 2.0)),
        material=airframe,
        name="wing",
    )
    body.visual(
        Box((0.38, 0.18, 0.08)),
        origin=Origin(xyz=(-0.02, 0.0, 0.06)),
        material=airframe,
        name="center_fuselage",
    )
    body.visual(
        Box((0.15, 0.14, 0.05)),
        origin=Origin(xyz=(0.02, 0.0, 0.085)),
        material=canopy,
        name="canopy",
    )
    body.visual(
        Box((0.62, 0.10, 0.08)),
        origin=Origin(xyz=(-0.48, 0.0, 0.04)),
        material=airframe,
        name="tail_boom",
    )

    for side_name, sign in (("left", 1.0), ("right", -1.0)):
        y_center = sign * front_nacelle_y
        for bracket_name, y_offset in (("inboard", -0.046), ("outboard", 0.046)):
            body.visual(
                Box((0.034, 0.012, 0.05)),
                origin=Origin(xyz=(0.152, y_center + y_offset, 0.015)),
                material=motor,
                name=f"front_{side_name}_hinge_{bracket_name}",
            )

    for side_name, sign in (("left", 1.0), ("right", -1.0)):
        pod_y = sign * rear_pod_y
        body.visual(
            Box((0.08, 0.07, 0.10)),
            origin=Origin(xyz=(rear_pod_x, pod_y, 0.09)),
            material=airframe,
            name=f"rear_{side_name}_pod_stalk",
        )
        body.visual(
            Cylinder(radius=0.032, length=0.10),
            origin=Origin(xyz=(rear_pod_x, pod_y, 0.15)),
            material=motor,
            name=f"rear_{side_name}_pod_shell",
        )

    body.visual(
        Box((0.03, 0.03, 0.13)),
        origin=Origin(xyz=(-0.75, 0.0, 0.145)),
        material=airframe,
        name="rudder_cap_strut",
    )
    body.visual(
        Box((0.07, 0.04, 0.01)),
        origin=Origin(xyz=(-0.77, 0.0, 0.215)),
        material=airframe,
        name="rudder_upper_cap",
    )

    front_left_nacelle = model.part("front_left_nacelle")
    front_left_nacelle.visual(
        Cylinder(radius=0.015, length=0.08),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=motor,
        name="hinge_barrel",
    )
    front_left_nacelle.visual(
        Cylinder(radius=0.034, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, -0.07)),
        material=motor,
        name="nacelle_body",
    )
    front_left_nacelle.visual(
        Cylinder(radius=0.012, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.147)),
        material=motor,
        name="motor_shaft",
    )
    front_left_nacelle.visual(
        Cylinder(radius=0.019, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.168)),
        material=motor,
        name="prop_hub",
    )
    front_left_nacelle.visual(
        Box((0.32, 0.024, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.168), rpy=(0.0, 0.0, 0.25)),
        material=prop,
        name="prop_blade",
    )
    front_left_nacelle.visual(
        Box((0.32, 0.024, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.168), rpy=(0.0, 0.0, 0.25 + (pi / 2.0))),
        material=prop,
        name="prop_blade_cross",
    )

    front_right_nacelle = model.part("front_right_nacelle")
    front_right_nacelle.visual(
        Cylinder(radius=0.015, length=0.08),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=motor,
        name="hinge_barrel",
    )
    front_right_nacelle.visual(
        Cylinder(radius=0.034, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, -0.07)),
        material=motor,
        name="nacelle_body",
    )
    front_right_nacelle.visual(
        Cylinder(radius=0.012, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.147)),
        material=motor,
        name="motor_shaft",
    )
    front_right_nacelle.visual(
        Cylinder(radius=0.019, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.168)),
        material=motor,
        name="prop_hub",
    )
    front_right_nacelle.visual(
        Box((0.32, 0.024, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.168), rpy=(0.0, 0.0, -0.25)),
        material=prop,
        name="prop_blade",
    )
    front_right_nacelle.visual(
        Box((0.32, 0.024, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.168), rpy=(0.0, 0.0, -0.25 + (pi / 2.0))),
        material=prop,
        name="prop_blade_cross",
    )

    rear_left_rotor = model.part("rear_left_rotor")
    rear_left_rotor.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=motor,
        name="rotor_hub",
    )
    rear_left_rotor.visual(
        Box((0.30, 0.026, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.012), rpy=(0.0, 0.0, 0.18)),
        material=prop,
        name="rotor_blade",
    )
    rear_left_rotor.visual(
        Box((0.30, 0.026, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.012), rpy=(0.0, 0.0, 0.18 + (pi / 2.0))),
        material=prop,
        name="rotor_blade_cross",
    )

    rear_right_rotor = model.part("rear_right_rotor")
    rear_right_rotor.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=motor,
        name="rotor_hub",
    )
    rear_right_rotor.visual(
        Box((0.30, 0.026, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.012), rpy=(0.0, 0.0, -0.18)),
        material=prop,
        name="rotor_blade",
    )
    rear_right_rotor.visual(
        Box((0.30, 0.026, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.012), rpy=(0.0, 0.0, -0.18 + (pi / 2.0))),
        material=prop,
        name="rotor_blade_cross",
    )

    rudder_fin = model.part("rudder_fin")
    rudder_fin.visual(
        Cylinder(radius=0.014, length=0.13),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=motor,
        name="hinge_boss",
    )
    rudder_fin.visual(
        Box((0.05, 0.02, 0.10)),
        origin=Origin(xyz=(-0.025, 0.0, 0.06)),
        material=accent,
        name="fin_root",
    )
    rudder_fin.visual(
        Box((0.13, 0.014, 0.20)),
        origin=Origin(xyz=(-0.09, 0.0, 0.20)),
        material=accent,
        name="fin_surface",
    )

    model.articulation(
        "body_to_front_left_nacelle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=front_left_nacelle,
        origin=Origin(xyz=(0.17, front_nacelle_y, -0.015)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=0.0, upper=pi / 2.0),
    )
    model.articulation(
        "body_to_front_right_nacelle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=front_right_nacelle,
        origin=Origin(xyz=(0.17, -front_nacelle_y, -0.015)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=0.0, upper=pi / 2.0),
    )
    model.articulation(
        "body_to_rear_left_rotor",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=rear_left_rotor,
        origin=Origin(xyz=(rear_pod_x, rear_pod_y, rear_pod_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=80.0),
    )
    model.articulation(
        "body_to_rear_right_rotor",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=rear_right_rotor,
        origin=Origin(xyz=(rear_pod_x, -rear_pod_y, rear_pod_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=80.0),
    )
    model.articulation(
        "body_to_rudder_fin",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rudder_fin,
        origin=Origin(xyz=(rudder_hinge_x, 0.0, rudder_hinge_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.2, lower=-0.45, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    front_left_nacelle = object_model.get_part("front_left_nacelle")
    front_right_nacelle = object_model.get_part("front_right_nacelle")
    rear_left_rotor = object_model.get_part("rear_left_rotor")
    rear_right_rotor = object_model.get_part("rear_right_rotor")
    rudder_fin = object_model.get_part("rudder_fin")

    left_tilt = object_model.get_articulation("body_to_front_left_nacelle")
    right_tilt = object_model.get_articulation("body_to_front_right_nacelle")
    left_rear_spin = object_model.get_articulation("body_to_rear_left_rotor")
    right_rear_spin = object_model.get_articulation("body_to_rear_right_rotor")
    rudder_joint = object_model.get_articulation("body_to_rudder_fin")

    def aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

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

    ctx.expect_contact(
        front_left_nacelle,
        body,
        elem_a="hinge_barrel",
        name="left front nacelle hinge remains mounted to the wing brackets",
    )
    ctx.expect_contact(
        front_right_nacelle,
        body,
        elem_a="hinge_barrel",
        name="right front nacelle hinge remains mounted to the wing brackets",
    )
    ctx.expect_contact(
        rear_left_rotor,
        body,
        elem_a="rotor_hub",
        name="left rear rotor hub sits on the left rear motor pod",
    )
    ctx.expect_contact(
        rear_right_rotor,
        body,
        elem_a="rotor_hub",
        name="right rear rotor hub sits on the right rear motor pod",
    )
    ctx.expect_contact(
        rudder_fin,
        body,
        elem_a="hinge_boss",
        name="rudder hinge boss is supported by the tail boom and cap",
    )

    ctx.check(
        "front nacelles use leading-edge spanwise tilt hinges",
        left_tilt.articulation_type == ArticulationType.REVOLUTE
        and right_tilt.articulation_type == ArticulationType.REVOLUTE
        and left_tilt.axis == (0.0, -1.0, 0.0)
        and right_tilt.axis == (0.0, -1.0, 0.0)
        and left_tilt.motion_limits is not None
        and right_tilt.motion_limits is not None
        and left_tilt.motion_limits.lower == 0.0
        and right_tilt.motion_limits.lower == 0.0
        and left_tilt.motion_limits.upper is not None
        and right_tilt.motion_limits.upper is not None
        and abs(left_tilt.motion_limits.upper - (pi / 2.0)) < 1e-6
        and abs(right_tilt.motion_limits.upper - (pi / 2.0)) < 1e-6,
        details=f"left={left_tilt.axis, left_tilt.motion_limits}, right={right_tilt.axis, right_tilt.motion_limits}",
    )
    ctx.check(
        "rear pods carry vertical continuous propeller axles",
        left_rear_spin.articulation_type == ArticulationType.CONTINUOUS
        and right_rear_spin.articulation_type == ArticulationType.CONTINUOUS
        and left_rear_spin.axis == (0.0, 0.0, 1.0)
        and right_rear_spin.axis == (0.0, 0.0, 1.0)
        and left_rear_spin.motion_limits is not None
        and right_rear_spin.motion_limits is not None
        and left_rear_spin.motion_limits.lower is None
        and left_rear_spin.motion_limits.upper is None
        and right_rear_spin.motion_limits.lower is None
        and right_rear_spin.motion_limits.upper is None,
        details=f"left={left_rear_spin.axis, left_rear_spin.motion_limits}, right={right_rear_spin.axis, right_rear_spin.motion_limits}",
    )
    ctx.check(
        "rudder fin yaws on a vertical tail hinge",
        rudder_joint.articulation_type == ArticulationType.REVOLUTE
        and rudder_joint.axis == (0.0, 0.0, 1.0)
        and rudder_joint.motion_limits is not None
        and rudder_joint.motion_limits.lower is not None
        and rudder_joint.motion_limits.upper is not None
        and rudder_joint.motion_limits.lower < 0.0 < rudder_joint.motion_limits.upper,
        details=f"rudder axis={rudder_joint.axis}, limits={rudder_joint.motion_limits}",
    )

    left_blade_rest = ctx.part_element_world_aabb(front_left_nacelle, elem="prop_blade")
    right_blade_rest = ctx.part_element_world_aabb(front_right_nacelle, elem="prop_blade")
    fin_rest = ctx.part_element_world_aabb(rudder_fin, elem="fin_surface")
    with ctx.pose(
        {
            left_tilt: left_tilt.motion_limits.upper if left_tilt.motion_limits is not None else 0.0,
            right_tilt: right_tilt.motion_limits.upper if right_tilt.motion_limits is not None else 0.0,
            rudder_joint: 0.35,
        }
    ):
        left_blade_cruise = ctx.part_element_world_aabb(front_left_nacelle, elem="prop_blade")
        right_blade_cruise = ctx.part_element_world_aabb(front_right_nacelle, elem="prop_blade")
        fin_deflected = ctx.part_element_world_aabb(rudder_fin, elem="fin_surface")

    left_rest_center = aabb_center(left_blade_rest)
    left_cruise_center = aabb_center(left_blade_cruise)
    right_rest_center = aabb_center(right_blade_rest)
    right_cruise_center = aabb_center(right_blade_cruise)
    fin_rest_center = aabb_center(fin_rest)
    fin_deflected_center = aabb_center(fin_deflected)

    ctx.check(
        "front left nacelle tips from vertical-lift to forward-flight orientation",
        left_rest_center is not None
        and left_cruise_center is not None
        and left_cruise_center[0] > left_rest_center[0] + 0.13
        and left_cruise_center[2] > left_rest_center[2] + 0.13,
        details=f"rest={left_rest_center}, cruise={left_cruise_center}",
    )
    ctx.check(
        "front right nacelle tips from vertical-lift to forward-flight orientation",
        right_rest_center is not None
        and right_cruise_center is not None
        and right_cruise_center[0] > right_rest_center[0] + 0.13
        and right_cruise_center[2] > right_rest_center[2] + 0.13,
        details=f"rest={right_rest_center}, cruise={right_cruise_center}",
    )
    ctx.check(
        "rudder fin deflects sideways about the vertical tail axis",
        fin_rest_center is not None
        and fin_deflected_center is not None
        and abs(fin_deflected_center[1] - fin_rest_center[1]) > 0.02,
        details=f"rest={fin_rest_center}, deflected={fin_deflected_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
