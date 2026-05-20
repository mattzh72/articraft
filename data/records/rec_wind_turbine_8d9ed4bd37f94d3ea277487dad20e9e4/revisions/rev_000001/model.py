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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="horizontal_axis_wind_turbine")

    concrete = model.material("concrete", rgba=(0.72, 0.72, 0.72, 1.0))
    tower_white = model.material("tower_white", rgba=(0.92, 0.93, 0.94, 1.0))
    nacelle_white = model.material("nacelle_white", rgba=(0.90, 0.91, 0.92, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.34, 0.36, 0.39, 1.0))
    blade_white = model.material("blade_white", rgba=(0.95, 0.95, 0.96, 1.0))
    safety_red = model.material("safety_red", rgba=(0.72, 0.12, 0.10, 1.0))

    tower = model.part("tower")
    tower.visual(
        Box((4.6, 4.6, 0.9)),
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
        material=concrete,
        name="foundation_pad",
    )
    tower.visual(
        Cylinder(radius=0.95, length=21.6),
        origin=Origin(xyz=(0.0, 0.0, 11.7)),
        material=tower_white,
        name="tower_shell",
    )
    tower.visual(
        Cylinder(radius=1.15, length=1.0),
        origin=Origin(xyz=(0.0, 0.0, 1.4)),
        material=tower_white,
        name="tower_base_flare",
    )
    tower.visual(
        Cylinder(radius=0.78, length=0.42),
        origin=Origin(xyz=(0.0, 0.0, 22.29)),
        material=steel_dark,
        name="yaw_bearing_ring",
    )
    tower.inertial = Inertial.from_geometry(
        Box((4.6, 4.6, 23.3)),
        mass=1800.0,
        origin=Origin(xyz=(0.0, 0.0, 11.65)),
    )

    nacelle = model.part("nacelle")
    nacelle.visual(
        Cylinder(radius=0.72, length=0.62),
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
        material=steel_dark,
        name="yaw_drive_pedestal",
    )
    nacelle.visual(
        Box((4.4, 2.3, 2.1)),
        origin=Origin(xyz=(1.0, 0.0, 1.55)),
        material=nacelle_white,
        name="nacelle_body",
    )
    nacelle.visual(
        Cylinder(radius=0.55, length=1.3),
        origin=Origin(xyz=(2.8, 0.0, 1.55), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_dark,
        name="main_bearing_housing",
    )
    nacelle.visual(
        Box((1.4, 1.8, 1.5)),
        origin=Origin(xyz=(-1.1, 0.0, 1.45)),
        material=nacelle_white,
        name="rear_equipment_block",
    )
    nacelle.inertial = Inertial.from_geometry(
        Box((4.8, 2.4, 2.4)),
        mass=240.0,
        origin=Origin(xyz=(1.0, 0.0, 1.5)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.58, length=1.1),
        origin=Origin(xyz=(0.20, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_dark,
        name="hub_shell",
    )
    rotor.visual(
        Cylinder(radius=0.28, length=0.62),
        origin=Origin(xyz=(-0.09, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_dark,
        name="main_shaft_stub",
    )
    rotor.visual(
        Cylinder(radius=0.30, length=0.9),
        origin=Origin(xyz=(0.68, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=safety_red,
        name="spinner",
    )
    blade_radius = 6.8
    blade_origin_x = 0.78
    blade_size = (0.55, 0.30, blade_radius)
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        rotor.visual(
            Box(blade_size),
            origin=Origin(
                xyz=(blade_origin_x, 0.0, blade_radius * 0.5),
                rpy=(0.0, 0.06, angle),
            ),
            material=blade_white,
            name=f"blade_{index}",
        )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=7.0, length=2.0),
        mass=120.0,
        origin=Origin(xyz=(0.2, 0.0, 0.0)),
    )

    model.articulation(
        "tower_to_nacelle_yaw",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=nacelle,
        origin=Origin(xyz=(0.0, 0.0, 22.5)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40000.0, velocity=0.25),
    )
    model.articulation(
        "nacelle_to_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=rotor,
        origin=Origin(xyz=(3.85, 0.0, 1.55)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50000.0, velocity=1.5),
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
    tower = object_model.get_part("tower")
    nacelle = object_model.get_part("nacelle")
    rotor = object_model.get_part("rotor")
    yaw = object_model.get_articulation("tower_to_nacelle_yaw")
    spin = object_model.get_articulation("nacelle_to_rotor_spin")

    ctx.expect_origin_gap(
        rotor,
        tower,
        axis="z",
        min_gap=22.0,
        max_gap=24.5,
        name="rotor hub sits high above the tower base",
    )
    ctx.expect_origin_gap(
        rotor,
        nacelle,
        axis="x",
        min_gap=3.6,
        max_gap=4.2,
        name="rotor hub is mounted at the front of the nacelle",
    )

    blade0_rest = ctx.part_element_world_aabb(rotor, elem="blade_0")
    with ctx.pose({spin: math.pi / 2.0}):
        blade0_quarter_turn = ctx.part_element_world_aabb(rotor, elem="blade_0")
    ctx.check(
        "rotor spin swings a blade around the main shaft axis",
        blade0_rest is not None
        and blade0_quarter_turn is not None
        and blade0_rest[1][2] > 6.0
        and blade0_quarter_turn[0][1] < -5.5,
        details=f"rest={blade0_rest}, quarter_turn={blade0_quarter_turn}",
    )

    rotor_pos_rest = ctx.part_world_position(rotor)
    with ctx.pose({yaw: math.pi / 2.0}):
        rotor_pos_yawed = ctx.part_world_position(rotor)
    ctx.check(
        "yaw rotates the nacelle and rotor around the tower axis",
        rotor_pos_rest is not None
        and rotor_pos_yawed is not None
        and rotor_pos_rest[0] > 3.0
        and abs(rotor_pos_rest[1]) < 0.05
        and rotor_pos_yawed[1] > 3.0
        and abs(rotor_pos_yawed[0]) < 0.05,
        details=f"rest={rotor_pos_rest}, yawed={rotor_pos_yawed}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
