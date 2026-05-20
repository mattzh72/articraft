from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    LatheGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="horizontal_axis_wind_turbine")

    galvanized = model.material("galvanized_steel", rgba=(0.62, 0.66, 0.68, 1.0))
    off_white = model.material("off_white_composite", rgba=(0.92, 0.94, 0.92, 1.0))
    dark_glass = model.material("dark_service_panel", rgba=(0.08, 0.10, 0.12, 1.0))
    rubber = model.material("dark_rubber", rgba=(0.025, 0.025, 0.023, 1.0))

    # Fixed support: a broad base, one tapered column, and a large yaw deck.
    tower = model.part("tower")
    tower.visual(
        Cylinder(radius=0.48, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=galvanized,
        name="base_pad",
    )
    tower_column = LatheGeometry(
        [
            (0.0, 0.12),
            (0.22, 0.12),
            (0.145, 2.50),
            (0.0, 2.50),
        ],
        segments=48,
    )
    tower.visual(
        mesh_from_geometry(tower_column, "tapered_tower_column"),
        material=galvanized,
        name="tower_column",
    )
    tower.visual(
        Cylinder(radius=0.27, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 2.46)),
        material=galvanized,
        name="yaw_deck",
    )

    # The nacelle part frame is on the yaw axis at the tower top.
    nacelle = model.part("nacelle")
    nacelle.visual(
        Cylinder(radius=0.23, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=galvanized,
        name="yaw_bearing",
    )
    nacelle.visual(
        Box((0.86, 0.42, 0.34)),
        origin=Origin(xyz=(0.26, 0.0, 0.24)),
        material=off_white,
        name="main_housing",
    )
    nacelle.visual(
        Cylinder(radius=0.18, length=0.24),
        origin=Origin(xyz=(0.76, 0.0, 0.24), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="shaft_collar",
    )
    nacelle.visual(
        Box((0.30, 0.010, 0.18)),
        origin=Origin(xyz=(0.22, -0.215, 0.26)),
        material=dark_glass,
        name="service_panel",
    )
    nacelle.visual(
        Cylinder(radius=0.075, length=0.16),
        origin=Origin(xyz=(-0.18, 0.0, 0.24), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="rear_cap",
    )

    rotor = model.part("rotor")
    rotor_mesh = FanRotorGeometry(
        outer_radius=1.05,
        hub_radius=0.18,
        blade_count=3,
        thickness=0.12,
        blade_pitch_deg=12.0,
        blade_sweep_deg=6.0,
        blade_root_chord=0.24,
        blade_tip_chord=0.07,
        blade=FanRotorBlade(shape="narrow", tip_pitch_deg=5.0, camber=0.08),
        hub=FanRotorHub(style="spinner", rear_collar_height=0.035, rear_collar_radius=0.14),
        center=True,
    )
    rotor.visual(
        mesh_from_geometry(rotor_mesh, "three_blade_rotor"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=off_white,
        name="rotor_mesh",
    )
    rotor.visual(
        Cylinder(radius=0.085, length=0.16),
        origin=Origin(xyz=(-0.10, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="rotor_shaft",
    )

    model.articulation(
        "tower_to_nacelle",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=nacelle,
        origin=Origin(xyz=(0.0, 0.0, 2.50)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.6),
        motion_properties=MotionProperties(damping=0.08, friction=0.02),
    )
    model.articulation(
        "nacelle_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=rotor,
        origin=Origin(xyz=(0.94, 0.0, 0.24)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=18.0),
        motion_properties=MotionProperties(damping=0.015, friction=0.004),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    nacelle = object_model.get_part("nacelle")
    rotor = object_model.get_part("rotor")
    yaw = object_model.get_articulation("tower_to_nacelle")
    spin = object_model.get_articulation("nacelle_to_rotor")

    ctx.allow_overlap(
        nacelle,
        rotor,
        elem_a="shaft_collar",
        elem_b="rotor_shaft",
        reason="The rotating main shaft is intentionally captured inside the nacelle bearing collar.",
    )

    ctx.check(
        "nacelle yaws continuously about the tower axis",
        yaw.articulation_type == ArticulationType.CONTINUOUS and tuple(yaw.axis) == (0.0, 0.0, 1.0),
        details=f"type={yaw.articulation_type}, axis={yaw.axis}",
    )
    ctx.check(
        "rotor spins continuously about the horizontal main shaft",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.expect_contact(
        tower,
        nacelle,
        elem_a="yaw_deck",
        elem_b="yaw_bearing",
        contact_tol=0.001,
        name="yaw bearing sits directly on the tower deck",
    )
    ctx.expect_overlap(
        nacelle,
        tower,
        axes="xy",
        elem_a="yaw_bearing",
        elem_b="yaw_deck",
        min_overlap=0.36,
        name="yaw bearing is centered over the tower deck",
    )
    ctx.expect_within(
        rotor,
        nacelle,
        axes="yz",
        inner_elem="rotor_shaft",
        outer_elem="shaft_collar",
        margin=0.001,
        name="rotor shaft stays centered inside the bearing collar",
    )
    ctx.expect_overlap(
        rotor,
        nacelle,
        axes="x",
        elem_a="rotor_shaft",
        elem_b="shaft_collar",
        min_overlap=0.10,
        name="rotor shaft remains inserted in the bearing collar",
    )

    rest_rotor_position = ctx.part_world_position(rotor)
    with ctx.pose({yaw: math.pi / 2.0}):
        yawed_rotor_position = ctx.part_world_position(rotor)
    ctx.check(
        "yaw joint swings the rotor around the vertical tower axis",
        rest_rotor_position is not None
        and yawed_rotor_position is not None
        and yawed_rotor_position[1] > rest_rotor_position[0] - 0.01
        and abs(yawed_rotor_position[0]) < 0.05,
        details=f"rest={rest_rotor_position}, yawed={yawed_rotor_position}",
    )

    return ctx.report()


object_model = build_object_model()
