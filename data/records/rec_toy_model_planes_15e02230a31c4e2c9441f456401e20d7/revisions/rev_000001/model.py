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
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    rounded_rect_profile,
    section_loft,
)


def _yz_rounded_section(
    x_pos: float,
    width: float,
    height: float,
    radius: float,
) -> list[tuple[float, float, float]]:
    return [(x_pos, y, z) for y, z in rounded_rect_profile(width, height, radius)]


def _wing_section(
    y_pos: float,
    *,
    chord: float,
    thickness: float,
    x_leading: float,
    z_center: float,
) -> list[tuple[float, float, float]]:
    x_trailing = x_leading - chord
    return [
        (x_trailing, y_pos, z_center + 0.08 * thickness),
        (x_trailing + 0.35 * chord, y_pos, z_center + 0.55 * thickness),
        (x_leading, y_pos, z_center + 0.12 * thickness),
        (x_leading - 0.03 * chord, y_pos, z_center - 0.10 * thickness),
        (x_trailing + 0.42 * chord, y_pos, z_center - 0.56 * thickness),
        (x_trailing, y_pos, z_center - 0.08 * thickness),
    ]


def _fin_section(
    y_pos: float,
    *,
    thickness_scale: float,
) -> list[tuple[float, float, float]]:
    return [
        (-0.286, y_pos, 0.012),
        (-0.248, y_pos, 0.012),
        (-0.234, y_pos, 0.042 * thickness_scale + 0.014),
        (-0.220, y_pos, 0.112),
        (-0.270, y_pos, 0.062),
    ]


def _propeller_blade_sections() -> list[list[tuple[float, float, float]]]:
    return [
        [
            (0.0035, 0.007, -0.014),
            (0.0050, 0.007, -0.003),
            (0.0010, 0.007, 0.014),
            (-0.0035, 0.007, 0.003),
        ],
        [
            (0.0025, 0.062, -0.015),
            (0.0040, 0.062, -0.003),
            (0.0000, 0.062, 0.015),
            (-0.0035, 0.062, 0.003),
        ],
        [
            (0.0010, 0.108, -0.009),
            (0.0018, 0.108, -0.002),
            (-0.0004, 0.108, 0.009),
            (-0.0020, 0.108, 0.002),
        ],
    ]


def _build_airframe_meshes() -> dict[str, MeshGeometry]:
    fuselage = repair_loft(
        section_loft(
            [
                _yz_rounded_section(0.198, 0.006, 0.008, 0.002),
                _yz_rounded_section(0.168, 0.038, 0.048, 0.010),
                _yz_rounded_section(0.080, 0.060, 0.074, 0.018),
                _yz_rounded_section(-0.010, 0.070, 0.088, 0.022),
                _yz_rounded_section(-0.105, 0.056, 0.072, 0.016),
                _yz_rounded_section(-0.210, 0.030, 0.042, 0.010),
                _yz_rounded_section(-0.286, 0.012, 0.018, 0.004),
            ]
        )
    )

    wing = repair_loft(
        section_loft(
            [
                _wing_section(-0.275, chord=0.085, thickness=0.006, x_leading=-0.018, z_center=0.032),
                _wing_section(-0.155, chord=0.122, thickness=0.010, x_leading=0.000, z_center=0.024),
                _wing_section(0.000, chord=0.172, thickness=0.013, x_leading=0.018, z_center=0.016),
                _wing_section(0.155, chord=0.122, thickness=0.010, x_leading=0.000, z_center=0.024),
                _wing_section(0.275, chord=0.085, thickness=0.006, x_leading=-0.018, z_center=0.032),
            ]
        )
    )

    tailplane = repair_loft(
        section_loft(
            [
                _wing_section(-0.112, chord=0.050, thickness=0.004, x_leading=-0.235, z_center=0.040),
                _wing_section(0.000, chord=0.084, thickness=0.006, x_leading=-0.214, z_center=0.018),
                _wing_section(0.112, chord=0.050, thickness=0.004, x_leading=-0.235, z_center=0.040),
            ]
        )
    )

    fin = repair_loft(
        section_loft(
            [
                _fin_section(-0.006, thickness_scale=0.80),
                _fin_section(0.000, thickness_scale=1.00),
                _fin_section(0.006, thickness_scale=0.80),
            ]
        )
    )

    return {
        "fuselage": fuselage,
        "wing": wing,
        "tailplane": tailplane,
        "fin": fin,
    }


def _build_spinner_mesh() -> MeshGeometry:
    return LatheGeometry(
        [
            (0.000, -0.002),
            (0.014, 0.000),
            (0.018, 0.008),
            (0.014, 0.022),
            (0.007, 0.032),
            (0.000, 0.038),
        ],
        segments=40,
    ).rotate_y(math.pi / 2.0)


def _build_propeller_blades_mesh() -> MeshGeometry:
    blade = repair_loft(section_loft(_propeller_blade_sections()))
    pair = MeshGeometry()
    pair.merge(blade)
    pair.merge(blade.copy().rotate_x(math.pi))
    return pair


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="model_airplane")

    fuselage_paint = model.material("fuselage_paint", rgba=(0.78, 0.18, 0.16, 1.0))
    wing_paint = model.material("wing_paint", rgba=(0.94, 0.94, 0.89, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.12, 0.14, 0.18, 1.0))
    canopy_tint = model.material("canopy_tint", rgba=(0.24, 0.34, 0.42, 0.78))

    airframe_meshes = _build_airframe_meshes()

    airframe = model.part("airframe")
    airframe.visual(
        mesh_from_geometry(airframe_meshes["fuselage"], "airframe_fuselage"),
        material=fuselage_paint,
        name="fuselage_shell",
    )
    airframe.visual(
        mesh_from_geometry(airframe_meshes["wing"], "airframe_wing"),
        material=wing_paint,
        name="main_wing",
    )
    airframe.visual(
        mesh_from_geometry(airframe_meshes["tailplane"], "airframe_tailplane"),
        material=wing_paint,
        name="horizontal_tail",
    )
    airframe.visual(
        mesh_from_geometry(airframe_meshes["fin"], "airframe_fin"),
        material=fuselage_paint,
        name="vertical_tail",
    )
    airframe.visual(
        Cylinder(radius=0.006, length=0.068),
        origin=Origin(xyz=(-0.028, 0.0, -0.022), rpy=(0.0, 0.0, 0.0)),
        material=trim_dark,
        name="display_peg",
    )
    airframe.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(-0.028, 0.0, -0.059), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_dark,
        name="display_base",
    )
    airframe.visual(
        Cylinder(radius=0.017, length=0.066),
        origin=Origin(xyz=(0.038, 0.0, 0.052), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=canopy_tint,
        name="canopy",
    )
    airframe.visual(
        Cylinder(radius=0.004, length=0.010),
        origin=Origin(xyz=(0.198, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_dark,
        name="nose_shaft",
    )
    airframe.inertial = Inertial.from_geometry(
        Box((0.49, 0.56, 0.18)),
        mass=0.9,
        origin=Origin(xyz=(-0.040, 0.0, 0.020)),
    )

    propeller = model.part("propeller")
    propeller.visual(
        mesh_from_geometry(_build_propeller_blades_mesh(), "propeller_blades"),
        material=trim_dark,
        name="blade_pair",
    )
    propeller.visual(
        Cylinder(radius=0.011, length=0.014),
        origin=Origin(xyz=(0.005, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_dark,
        name="hub",
    )
    propeller.visual(
        Cylinder(radius=0.004, length=0.010),
        origin=Origin(xyz=(0.002, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_dark,
        name="shaft_stub",
    )
    propeller.visual(
        mesh_from_geometry(_build_spinner_mesh(), "propeller_spinner"),
        material=wing_paint,
        name="spinner",
    )
    propeller.inertial = Inertial.from_geometry(
        Cylinder(radius=0.110, length=0.042),
        mass=0.08,
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "airframe_to_propeller",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=propeller,
        origin=Origin(xyz=(0.206, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=40.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    airframe = object_model.get_part("airframe")
    propeller = object_model.get_part("propeller")
    propeller_joint = object_model.get_articulation("airframe_to_propeller")

    ctx.expect_origin_distance(
        propeller,
        airframe,
        axes="yz",
        max_dist=0.0001,
        name="propeller stays centered on fuselage midline",
    )
    ctx.expect_origin_gap(
        propeller,
        airframe,
        axis="x",
        min_gap=0.18,
        max_gap=0.24,
        name="propeller joint sits ahead of the airframe origin",
    )
    ctx.expect_contact(
        propeller,
        airframe,
        contact_tol=1e-6,
        elem_a="shaft_stub",
        elem_b="nose_shaft",
        name="propeller shaft seats against the nose bearing",
    )

    rest_position = ctx.part_world_position(propeller)
    with ctx.pose({propeller_joint: math.pi / 2.0}):
        spun_position = ctx.part_world_position(propeller)
        ctx.expect_origin_distance(
            propeller,
            airframe,
            axes="yz",
            max_dist=0.0001,
            name="propeller remains centered while spinning",
        )
    ctx.check(
        "propeller spin preserves hub location",
        rest_position is not None
        and spun_position is not None
        and abs(rest_position[0] - spun_position[0]) < 1e-6
        and abs(rest_position[1] - spun_position[1]) < 1e-6
        and abs(rest_position[2] - spun_position[2]) < 1e-6,
        details=f"rest={rest_position}, spun={spun_position}",
    )
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
