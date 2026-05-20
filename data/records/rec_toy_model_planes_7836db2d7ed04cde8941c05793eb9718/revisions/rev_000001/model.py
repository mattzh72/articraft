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
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="propeller_model_airplane")

    center_len = 0.28
    center_rad = 0.06
    nose_len = 0.16
    nose_rad = 0.047
    tail_len = 0.22
    tail_rad = 0.028
    prop_hub_len = 0.03
    wing_profile = [
        (-0.09, -0.36),
        (0.03, -0.36),
        (0.12, -0.02),
        (0.12, 0.02),
        (0.03, 0.36),
        (-0.09, 0.36),
    ]
    stabilizer_profile = [
        (-0.08, -0.13),
        (0.0, -0.13),
        (0.04, -0.02),
        (0.04, 0.02),
        (0.0, 0.13),
        (-0.08, 0.13),
    ]
    fin_profile = [
        (-0.055, 0.0),
        (-0.01, 0.0),
        (0.015, 0.065),
        (-0.005, 0.135),
        (-0.05, 0.10),
    ]

    body_white = model.material("body_white", rgba=(0.94, 0.95, 0.97, 1.0))
    nose_red = model.material("nose_red", rgba=(0.76, 0.14, 0.12, 1.0))
    tail_blue = model.material("tail_blue", rgba=(0.18, 0.31, 0.62, 1.0))
    prop_black = model.material("prop_black", rgba=(0.12, 0.12, 0.12, 1.0))
    canopy_tint = model.material("canopy_tint", rgba=(0.52, 0.66, 0.80, 0.70))
    stand_gray = model.material("stand_gray", rgba=(0.58, 0.60, 0.64, 1.0))

    wing_mesh = mesh_from_geometry(
        ExtrudeGeometry(wing_profile, 0.014, center=True),
        "main_wing",
    )
    stabilizer_mesh = mesh_from_geometry(
        ExtrudeGeometry(stabilizer_profile, 0.009, center=True),
        "horizontal_stabilizer",
    )
    fin_mesh = mesh_from_geometry(
        ExtrudeGeometry(fin_profile, 0.012, center=True),
        "vertical_fin",
    )

    fuselage_center = model.part("fuselage_center")
    fuselage_center.visual(
        Cylinder(radius=center_rad, length=center_len),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=body_white,
        name="center_body",
    )
    fuselage_center.visual(
        wing_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=body_white,
        name="main_wing",
    )
    fuselage_center.visual(
        Sphere(radius=0.040),
        origin=Origin(xyz=(0.02, 0.0, 0.045)),
        material=canopy_tint,
        name="canopy",
    )
    fuselage_center.visual(
        Cylinder(radius=0.012, length=0.11),
        origin=Origin(xyz=(-0.01, 0.0, -0.105)),
        material=stand_gray,
        name="display_peg",
    )
    fuselage_center.visual(
        Cylinder(radius=0.046, length=0.022),
        origin=Origin(xyz=(-0.01, 0.0, -0.169)),
        material=stand_gray,
        name="display_base",
    )

    fuselage_nose = model.part("fuselage_nose")
    fuselage_nose.visual(
        Cylinder(radius=nose_rad, length=nose_len),
        origin=Origin(xyz=(nose_len / 2.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=nose_red,
        name="nose_cylinder",
    )
    fuselage_nose.visual(
        Sphere(radius=nose_rad),
        origin=Origin(xyz=(nose_len + nose_rad * 0.85, 0.0, 0.0)),
        material=nose_red,
        name="nose_cowling",
    )

    fuselage_tail = model.part("fuselage_tail")
    fuselage_tail.visual(
        Cylinder(radius=tail_rad, length=tail_len),
        origin=Origin(xyz=(-tail_len / 2.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=tail_blue,
        name="tail_boom",
    )
    fuselage_tail.visual(
        Sphere(radius=tail_rad * 0.78),
        origin=Origin(xyz=(-tail_len, 0.0, 0.0)),
        material=tail_blue,
        name="tail_cap",
    )
    fuselage_tail.visual(
        stabilizer_mesh,
        origin=Origin(xyz=(-0.15, 0.0, 0.0)),
        material=tail_blue,
        name="horizontal_stabilizer",
    )
    fuselage_tail.visual(
        fin_mesh,
        origin=Origin(xyz=(-0.145, 0.0, 0.01), rpy=(pi / 2.0, 0.0, 0.0)),
        material=tail_blue,
        name="vertical_fin",
    )

    propeller = model.part("propeller")
    propeller.visual(
        Cylinder(radius=0.012, length=prop_hub_len),
        origin=Origin(xyz=(prop_hub_len / 2.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=prop_black,
        name="prop_hub",
    )
    propeller.visual(
        Sphere(radius=0.015),
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
        material=prop_black,
        name="spinner",
    )
    propeller.visual(
        Box((0.008, 0.22, 0.028)),
        origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(0.0, 0.20, 0.0)),
        material=prop_black,
        name="prop_blade_pair",
    )

    model.articulation(
        "center_to_nose",
        ArticulationType.FIXED,
        parent=fuselage_center,
        child=fuselage_nose,
        origin=Origin(xyz=(center_len / 2.0, 0.0, 0.0)),
    )
    model.articulation(
        "center_to_tail",
        ArticulationType.FIXED,
        parent=fuselage_center,
        child=fuselage_tail,
        origin=Origin(xyz=(-center_len / 2.0, 0.0, 0.0)),
    )
    model.articulation(
        "nose_to_propeller",
        ArticulationType.CONTINUOUS,
        parent=fuselage_nose,
        child=propeller,
        origin=Origin(xyz=(nose_len + nose_rad * 1.85, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=50.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fuselage_center = object_model.get_part("fuselage_center")
    fuselage_nose = object_model.get_part("fuselage_nose")
    fuselage_tail = object_model.get_part("fuselage_tail")
    propeller = object_model.get_part("propeller")
    propeller_spin = object_model.get_articulation("nose_to_propeller")

    ctx.expect_gap(
        fuselage_nose,
        fuselage_center,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        name="nose section meets center fuselage cleanly",
    )
    ctx.expect_overlap(
        fuselage_nose,
        fuselage_center,
        axes="yz",
        min_overlap=0.08,
        name="nose section stays coaxial with center fuselage",
    )
    ctx.expect_gap(
        fuselage_center,
        fuselage_tail,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        name="tail boom meets center fuselage cleanly",
    )
    ctx.expect_overlap(
        fuselage_center,
        fuselage_tail,
        axes="yz",
        min_overlap=0.04,
        name="tail boom stays coaxial with center fuselage",
    )
    ctx.expect_gap(
        propeller,
        fuselage_nose,
        axis="x",
        max_gap=0.002,
        max_penetration=0.0,
        name="propeller mounts directly on the nose",
    )
    ctx.expect_overlap(
        propeller,
        fuselage_nose,
        axes="yz",
        min_overlap=0.024,
        name="propeller stays centered on the nose axis",
    )

    rest_pos = ctx.part_world_position(propeller)
    with ctx.pose({propeller_spin: 2.1}):
        spun_pos = ctx.part_world_position(propeller)
        ctx.expect_gap(
            propeller,
            fuselage_nose,
            axis="x",
            max_gap=0.002,
            max_penetration=0.0,
            name="propeller remains seated while spinning",
        )

    ctx.check(
        "propeller spins about a fixed nose axis",
        rest_pos is not None
        and spun_pos is not None
        and abs(rest_pos[0] - spun_pos[0]) < 1e-6
        and abs(rest_pos[1] - spun_pos[1]) < 1e-6
        and abs(rest_pos[2] - spun_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, spun={spun_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
