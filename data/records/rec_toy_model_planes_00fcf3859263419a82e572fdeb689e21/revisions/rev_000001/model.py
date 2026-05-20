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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="propeller_model_airplane")

    painted_red = model.material("painted_red", rgba=(0.78, 0.16, 0.14, 1.0))
    cream = model.material("cream", rgba=(0.95, 0.93, 0.84, 1.0))
    canopy_tint = model.material("canopy_tint", rgba=(0.22, 0.36, 0.48, 0.72))
    tire_black = model.material("tire_black", rgba=(0.07, 0.07, 0.08, 1.0))
    aluminum = model.material("aluminum", rgba=(0.76, 0.78, 0.82, 1.0))

    airframe = model.part("airframe")
    airframe.inertial = Inertial.from_geometry(
        Box((0.44, 0.56, 0.18)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )

    fuselage_profile = [
        (0.0, -0.165),
        (0.014, -0.150),
        (0.022, -0.120),
        (0.032, -0.070),
        (0.040, -0.015),
        (0.043, 0.055),
        (0.042, 0.115),
        (0.036, 0.145),
    ]
    fuselage_mesh = mesh_from_geometry(
        LatheGeometry(fuselage_profile, segments=48).rotate_y(pi / 2.0),
        "airframe_fuselage_body",
    )
    airframe.visual(fuselage_mesh, material=painted_red, name="fuselage_body")
    airframe.visual(
        Cylinder(radius=0.041, length=0.028),
        origin=Origin(xyz=(0.158, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=cream,
        name="nose_cowl",
    )

    wing_outline = [
        (-0.095, -0.025),
        (-0.055, -0.240),
        (0.070, -0.220),
        (0.112, -0.040),
        (0.120, 0.0),
        (0.112, 0.040),
        (0.070, 0.220),
        (-0.055, 0.240),
        (-0.095, 0.025),
    ]
    main_wing_mesh = mesh_from_geometry(
        ExtrudeGeometry(wing_outline, 0.012, center=True),
        "airframe_main_wing",
    )
    airframe.visual(
        main_wing_mesh,
        origin=Origin(xyz=(-0.006, 0.0, 0.008), rpy=(0.09, 0.0, 0.0)),
        material=cream,
        name="main_wing",
    )

    tail_outline = [
        (-0.052, -0.018),
        (-0.035, -0.105),
        (0.028, -0.090),
        (0.055, -0.020),
        (0.055, 0.020),
        (0.028, 0.090),
        (-0.035, 0.105),
        (-0.052, 0.018),
    ]
    horizontal_tail_mesh = mesh_from_geometry(
        ExtrudeGeometry(tail_outline, 0.008, center=True),
        "airframe_horizontal_tail",
    )
    airframe.visual(
        horizontal_tail_mesh,
        origin=Origin(xyz=(-0.135, 0.0, 0.030), rpy=(0.04, 0.0, 0.0)),
        material=cream,
        name="horizontal_tail",
    )

    fin_outline = [
        (-0.048, 0.000),
        (-0.020, 0.000),
        (0.000, 0.040),
        (0.014, 0.084),
        (-0.008, 0.096),
        (-0.040, 0.056),
    ]
    vertical_tail_mesh = mesh_from_geometry(
        ExtrudeGeometry(fin_outline, 0.008, center=True),
        "airframe_vertical_tail",
    )
    airframe.visual(
        vertical_tail_mesh,
        origin=Origin(xyz=(-0.148, 0.0, 0.034), rpy=(pi / 2.0, 0.0, 0.0)),
        material=painted_red,
        name="vertical_tail",
    )

    canopy_geom = SphereGeometry(1.0).scale(0.036, 0.023, 0.018).translate(-0.010, 0.0, 0.040)
    airframe.visual(
        mesh_from_geometry(canopy_geom, "airframe_canopy"),
        material=canopy_tint,
        name="canopy",
    )

    airframe.visual(
        Cylinder(radius=0.010, length=0.085),
        origin=Origin(xyz=(-0.018, 0.0, -0.072)),
        material=aluminum,
        name="display_peg",
    )
    airframe.visual(
        Cylinder(radius=0.032, length=0.008),
        origin=Origin(xyz=(-0.018, 0.0, -0.118)),
        material=tire_black,
        name="display_base",
    )

    propeller = model.part("propeller")
    propeller.inertial = Inertial.from_geometry(
        Box((0.07, 0.04, 0.20)),
        mass=0.08,
        origin=Origin(xyz=(0.024, 0.0, 0.0)),
    )
    spinner_geom = SphereGeometry(1.0).scale(0.024, 0.021, 0.021).translate(0.024, 0.0, 0.0)
    propeller.visual(
        mesh_from_geometry(spinner_geom, "propeller_spinner"),
        material=painted_red,
        name="spinner",
    )
    propeller.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.008, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=aluminum,
        name="hub",
    )
    propeller.visual(
        Box((0.008, 0.024, 0.196)),
        origin=Origin(xyz=(0.016, 0.0, 0.0)),
        material=cream,
        name="blade_bar",
    )
    propeller.visual(
        Box((0.006, 0.034, 0.080)),
        origin=Origin(xyz=(0.020, 0.0, 0.058)),
        material=cream,
        name="upper_blade_paddle",
    )
    propeller.visual(
        Box((0.006, 0.034, 0.080)),
        origin=Origin(xyz=(0.020, 0.0, -0.058)),
        material=cream,
        name="lower_blade_paddle",
    )

    model.articulation(
        "nose_propeller_spin",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=propeller,
        origin=Origin(xyz=(0.171, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=40.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    airframe = object_model.get_part("airframe")
    propeller = object_model.get_part("propeller")
    prop_joint = object_model.get_articulation("nose_propeller_spin")

    ctx.expect_gap(
        propeller,
        airframe,
        axis="x",
        positive_elem="hub",
        negative_elem="nose_cowl",
        min_gap=0.0,
        max_gap=0.004,
        name="propeller hub stays seated against the cowl",
    )
    wing_box = ctx.part_element_world_aabb(airframe, elem="main_wing")
    fuselage_box = ctx.part_element_world_aabb(airframe, elem="fuselage_body")
    cowl_box = ctx.part_element_world_aabb(airframe, elem="nose_cowl")
    blade_box = ctx.part_element_world_aabb(propeller, elem="blade_bar")
    if wing_box is not None and fuselage_box is not None:
        wing_span = wing_box[1][1] - wing_box[0][1]
        fuselage_length = fuselage_box[1][0] - fuselage_box[0][0]
        ctx.check(
            "short wide airplane proportion",
            wing_span > fuselage_length * 1.35,
            details=f"wing_span={wing_span:.4f}, fuselage_length={fuselage_length:.4f}",
        )
    else:
        ctx.fail("short wide airplane proportion", "Missing wing or fuselage AABB.")

    if cowl_box is not None and blade_box is not None:
        cowl_y = cowl_box[1][1] - cowl_box[0][1]
        cowl_z = cowl_box[1][2] - cowl_box[0][2]
        blade_z = blade_box[1][2] - blade_box[0][2]
        ctx.check(
            "propeller visually dominates the nose face",
            blade_z > cowl_z * 2.1 and blade_z > cowl_y * 2.1,
            details=f"blade_z={blade_z:.4f}, cowl_y={cowl_y:.4f}, cowl_z={cowl_z:.4f}",
        )
    else:
        ctx.fail("propeller visually dominates the nose face", "Missing blade or cowl AABB.")

    rest_blade = blade_box
    with ctx.pose({prop_joint: pi / 2.0}):
        spun_blade = ctx.part_element_world_aabb(propeller, elem="blade_bar")
    if rest_blade is not None and spun_blade is not None:
        rest_y = rest_blade[1][1] - rest_blade[0][1]
        rest_z = rest_blade[1][2] - rest_blade[0][2]
        spun_y = spun_blade[1][1] - spun_blade[0][1]
        spun_z = spun_blade[1][2] - spun_blade[0][2]
        ctx.check(
            "propeller rotates about the nose axis",
            rest_z > rest_y * 3.0 and spun_y > spun_z * 3.0,
            details=(
                f"rest_y={rest_y:.4f}, rest_z={rest_z:.4f}, "
                f"spun_y={spun_y:.4f}, spun_z={spun_z:.4f}"
            ),
        )
    else:
        ctx.fail("propeller rotates about the nose axis", "Missing blade AABB.")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
