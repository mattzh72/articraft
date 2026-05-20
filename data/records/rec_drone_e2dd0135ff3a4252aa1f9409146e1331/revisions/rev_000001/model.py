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
    model = ArticulatedObject(name="tilt_rotor_bicopter")

    carbon = model.material("carbon_fiber", rgba=(0.09, 0.09, 0.11, 1.0))
    composite = model.material("composite_wing", rgba=(0.18, 0.18, 0.20, 1.0))
    nacelle_gray = model.material("nacelle_gray", rgba=(0.55, 0.57, 0.60, 1.0))
    rotor_black = model.material("rotor_black", rgba=(0.05, 0.05, 0.05, 1.0))
    skid_black = model.material("skid_black", rgba=(0.12, 0.12, 0.12, 1.0))

    body = model.part("body")

    fuselage_length = 1.22
    fuselage_width = 0.14
    fuselage_height = 0.14
    wing_span = 1.42
    wing_chord = 0.18
    wing_thickness = 0.024
    hinge_z = 0.045
    hinge_y = 0.75
    hinge_plate_thickness = 0.015
    hinge_plate_span = 0.07

    body.visual(
        Box((fuselage_length, fuselage_width, fuselage_height)),
        material=carbon,
        name="fuselage_shell",
    )
    body.visual(
        Box((0.24, 0.11, 0.09)),
        origin=Origin(xyz=(0.45, 0.0, 0.005)),
        material=carbon,
        name="nose_fairing",
    )
    body.visual(
        Box((wing_chord, wing_span, wing_thickness)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=composite,
        name="center_wing",
    )

    for side_name, side in (("left", 1.0), ("right", -1.0)):
        inner_plate_y = side * 0.7175
        outer_plate_y = side * 0.7825
        brace_y = side * hinge_y

        body.visual(
            Box((0.05, hinge_plate_thickness, hinge_plate_span)),
            origin=Origin(xyz=(0.0, inner_plate_y, hinge_z)),
            material=nacelle_gray,
            name=f"{side_name}_hinge_inner_plate",
        )
        body.visual(
            Box((0.05, hinge_plate_thickness, hinge_plate_span)),
            origin=Origin(xyz=(0.0, outer_plate_y, hinge_z)),
            material=nacelle_gray,
            name=f"{side_name}_hinge_outer_plate",
        )
        body.visual(
            Box((0.054, 0.08, 0.018)),
            origin=Origin(xyz=(0.05, brace_y, 0.085)),
            material=nacelle_gray,
            name=f"{side_name}_hinge_forward_brace",
        )

    body.visual(
        Box((0.18, 0.02, 0.18)),
        origin=Origin(xyz=(-0.53, 0.0, 0.09)),
        material=composite,
        name="vertical_tail",
    )
    body.visual(
        Box((0.24, 0.44, 0.018)),
        origin=Origin(xyz=(-0.53, 0.0, 0.03)),
        material=composite,
        name="tailplane",
    )

    skid_z = -0.16
    skid_y = 0.09
    for side_name, side in (("left", 1.0), ("right", -1.0)):
        body.visual(
            Box((0.72, 0.02, 0.02)),
            origin=Origin(xyz=(0.0, side * skid_y, skid_z)),
            material=skid_black,
            name=f"{side_name}_skid",
        )
        for station_name, x in (("front", 0.28), ("rear", -0.28)):
            body.visual(
                Box((0.02, 0.06, 0.18)),
                origin=Origin(xyz=(x, side * skid_y, -0.085)),
                material=skid_black,
                name=f"{side_name}_{station_name}_strut",
            )

    nacelle_specs = (
        ("left_nacelle", "left_rotor", 1.0, (0.0, 0.0, 1.0)),
        ("right_nacelle", "right_rotor", -1.0, (0.0, 0.0, -1.0)),
    )

    for nacelle_name, rotor_name, side, rotor_axis in nacelle_specs:
        nacelle = model.part(nacelle_name)
        nacelle.visual(
            Cylinder(radius=0.028, length=0.05),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=nacelle_gray,
            name="hinge_barrel",
        )
        nacelle.visual(
            Cylinder(radius=0.022, length=0.20),
            origin=Origin(xyz=(0.02, 0.0, -0.078), rpy=(0.0, pi / 2.0, 0.0)),
            material=nacelle_gray,
            name="motor_pod",
        )
        nacelle.visual(
            Box((0.18, 0.045, 0.10)),
            origin=Origin(xyz=(0.01, 0.0, -0.073)),
            material=nacelle_gray,
            name="pod_fairing",
        )
        nacelle.visual(
            Cylinder(radius=0.018, length=0.10),
            origin=Origin(xyz=(0.0, 0.0, 0.05)),
            material=nacelle_gray,
            name="mast",
        )

        rotor = model.part(rotor_name)
        rotor.visual(
            Cylinder(radius=0.024, length=0.02),
            origin=Origin(xyz=(0.0, 0.0, 0.01)),
            material=rotor_black,
            name="hub",
        )
        rotor.visual(
            Box((0.58, 0.045, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, 0.015)),
            material=rotor_black,
            name="blade_pair",
        )

        model.articulation(
            f"body_to_{nacelle_name}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=nacelle,
            origin=Origin(xyz=(0.0, side * hinge_y, hinge_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=40.0,
                velocity=1.6,
                lower=0.0,
                upper=pi / 2.0,
            ),
        )
        model.articulation(
            f"{nacelle_name}_to_{rotor_name}",
            ArticulationType.CONTINUOUS,
            parent=nacelle,
            child=rotor,
            origin=Origin(xyz=(0.0, 0.0, 0.10)),
            axis=rotor_axis,
            motion_limits=MotionLimits(effort=10.0, velocity=120.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    left_nacelle = object_model.get_part("left_nacelle")
    right_nacelle = object_model.get_part("right_nacelle")
    left_rotor = object_model.get_part("left_rotor")
    right_rotor = object_model.get_part("right_rotor")

    left_tilt = object_model.get_articulation("body_to_left_nacelle")
    right_tilt = object_model.get_articulation("body_to_right_nacelle")
    left_spin = object_model.get_articulation("left_nacelle_to_left_rotor")
    right_spin = object_model.get_articulation("right_nacelle_to_right_rotor")

    ctx.check("left tilt joint axis is spanwise", left_tilt.axis == (0.0, 1.0, 0.0), details=str(left_tilt.axis))
    ctx.check("right tilt joint axis is spanwise", right_tilt.axis == (0.0, 1.0, 0.0), details=str(right_tilt.axis))

    left_limits = left_tilt.motion_limits
    right_limits = right_tilt.motion_limits
    ctx.check(
        "left nacelle tilts from vertical to horizontal",
        left_limits is not None
        and left_limits.lower == 0.0
        and left_limits.upper is not None
        and abs(left_limits.upper - pi / 2.0) < 1e-6,
        details=str(left_limits),
    )
    ctx.check(
        "right nacelle tilts from vertical to horizontal",
        right_limits is not None
        and right_limits.lower == 0.0
        and right_limits.upper is not None
        and abs(right_limits.upper - pi / 2.0) < 1e-6,
        details=str(right_limits),
    )

    ctx.check(
        "left rotor spins continuously",
        left_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=str(left_spin.articulation_type),
    )
    ctx.check(
        "right rotor spins continuously",
        right_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=str(right_spin.articulation_type),
    )

    ctx.expect_contact(
        left_nacelle,
        body,
        name="left nacelle is physically mounted to the wingtip yoke",
    )
    ctx.expect_contact(
        right_nacelle,
        body,
        name="right nacelle is physically mounted to the wingtip yoke",
    )
    ctx.expect_contact(
        left_rotor,
        left_nacelle,
        name="left rotor sits on the left nacelle mast",
    )
    ctx.expect_contact(
        right_rotor,
        right_nacelle,
        name="right rotor sits on the right nacelle mast",
    )

    left_rest = ctx.part_world_position(left_rotor)
    right_rest = ctx.part_world_position(right_rotor)
    with ctx.pose({left_tilt: pi / 2.0, right_tilt: pi / 2.0}):
        left_forward = ctx.part_world_position(left_rotor)
        right_forward = ctx.part_world_position(right_rotor)

    ctx.check(
        "left rotor moves forward when the nacelle tilts horizontal",
        left_rest is not None
        and left_forward is not None
        and left_forward[0] > left_rest[0] + 0.08
        and left_forward[2] < left_rest[2] - 0.08,
        details=f"rest={left_rest}, forward={left_forward}",
    )
    ctx.check(
        "right rotor moves forward when the nacelle tilts horizontal",
        right_rest is not None
        and right_forward is not None
        and right_forward[0] > right_rest[0] + 0.08
        and right_forward[2] < right_rest[2] - 0.08,
        details=f"rest={right_rest}, forward={right_forward}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
