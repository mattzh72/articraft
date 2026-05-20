from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelFace,
    BezelGeometry,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mini_itx_graphics_card")

    pcb_mat = model.material("matte_black_pcb", rgba=(0.015, 0.025, 0.020, 1.0))
    solder_mat = model.material("dark_solder_mask", rgba=(0.025, 0.13, 0.075, 1.0))
    shroud_mat = model.material("satin_black_shroud", rgba=(0.018, 0.018, 0.020, 1.0))
    fan_mat = model.material("smoked_fan_plastic", rgba=(0.030, 0.032, 0.036, 1.0))
    aluminum_mat = model.material("brushed_aluminum", rgba=(0.62, 0.64, 0.62, 1.0))
    steel_mat = model.material("io_bracket_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    gold_mat = model.material("edge_gold", rgba=(0.95, 0.68, 0.20, 1.0))
    port_mat = model.material("dark_port_insets", rgba=(0.006, 0.007, 0.008, 1.0))
    copper_mat = model.material("copper_heatpipes", rgba=(0.78, 0.33, 0.12, 1.0))

    body = model.part("card_body")

    # Mini-ITX class board: short, flat PCB with the cooler occupying most of it.
    body.visual(
        Box((0.170, 0.105, 0.0020)),
        origin=Origin(xyz=(0.0, 0.0, 0.0010)),
        material=pcb_mat,
        name="pcb",
    )
    body.visual(
        Box((0.104, 0.006, 0.0024)),
        origin=Origin(xyz=(0.004, -0.0550, 0.0021)),
        material=gold_mat,
        name="pcie_contacts",
    )
    body.visual(
        Box((0.004, 0.115, 0.057)),
        origin=Origin(xyz=(-0.0855, 0.0, 0.0275)),
        material=steel_mat,
        name="io_bracket",
    )
    body.visual(
        Box((0.0010, 0.025, 0.010)),
        origin=Origin(xyz=(-0.0878, -0.022, 0.030)),
        material=port_mat,
        name="display_port",
    )
    body.visual(
        Box((0.0010, 0.021, 0.008)),
        origin=Origin(xyz=(-0.0878, 0.018, 0.029)),
        material=port_mat,
        name="hdmi_port",
    )
    for i, z in enumerate((0.010, 0.017, 0.044, 0.051)):
        body.visual(
            Box((0.0010, 0.053, 0.0022)),
            origin=Origin(xyz=(-0.0878, 0.0, z)),
            material=port_mat,
            name=f"bracket_vent_{i}",
        )

    # Aluminum heatsink and real-looking fin stack below the fan shroud.
    body.visual(
        Box((0.143, 0.090, 0.0090)),
        origin=Origin(xyz=(0.010, 0.0, 0.0064)),
        material=aluminum_mat,
        name="heatsink_base",
    )
    for i, y in enumerate([-0.039, -0.031, -0.023, -0.015, -0.007, 0.001, 0.009, 0.017, 0.025, 0.033, 0.041]):
        body.visual(
            Box((0.136, 0.0021, 0.018)),
            origin=Origin(xyz=(0.012, y, 0.0198)),
            material=aluminum_mat,
            name=f"fin_{i}",
        )
    for i, y in enumerate((-0.027, 0.027)):
        body.visual(
            Cylinder(radius=0.0032, length=0.132),
            origin=Origin(xyz=(0.012, y, 0.0152), rpy=(0.0, pi / 2.0, 0.0)),
            material=copper_mat,
            name=f"heatpipe_{i}",
        )

    fan_center_x = 0.012
    fan_center_z = 0.043

    shroud = BezelGeometry(
        opening_size=(0.088, 0.088),
        outer_size=(0.150, 0.100),
        depth=0.018,
        opening_shape="circle",
        outer_shape="rounded_rect",
        outer_corner_radius=0.010,
        face=BezelFace(style="radiused_step", front_lip=0.0025, fillet=0.0015),
    )
    body.visual(
        mesh_from_geometry(shroud, "short_fan_shroud"),
        origin=Origin(xyz=(fan_center_x, 0.0, 0.0355)),
        material=shroud_mat,
        name="fan_shroud",
    )

    # Fixed stator cross and bearing pin: these make the fan read as retained in
    # the circular opening rather than hovering above the board.
    body.visual(
        Box((0.090, 0.0040, 0.0030)),
        origin=Origin(xyz=(fan_center_x, 0.0, 0.0305)),
        material=shroud_mat,
        name="stator_spoke_x",
    )
    body.visual(
        Box((0.0040, 0.090, 0.0030)),
        origin=Origin(xyz=(fan_center_x, 0.0, 0.0305)),
        material=shroud_mat,
        name="stator_spoke_y",
    )
    body.visual(
        Cylinder(radius=0.0135, length=0.0070),
        origin=Origin(xyz=(fan_center_x, 0.0, 0.0335)),
        material=shroud_mat,
        name="stator_hub",
    )
    body.visual(
        Cylinder(radius=0.0030, length=0.025),
        origin=Origin(xyz=(fan_center_x, 0.0, 0.0415)),
        material=steel_mat,
        name="bearing_pin",
    )
    body.visual(
        Cylinder(radius=0.0070, length=0.0022),
        origin=Origin(xyz=(fan_center_x, 0.0, 0.0520)),
        material=steel_mat,
        name="retainer_clip",
    )
    for i, (x, y) in enumerate(
        (
            (-0.045, -0.038),
            (-0.045, 0.038),
            (0.070, -0.038),
            (0.070, 0.038),
        )
    ):
        body.visual(
            Cylinder(radius=0.0030, length=0.0024),
            origin=Origin(xyz=(x, y, 0.0435)),
            material=steel_mat,
            name=f"shroud_screw_{i}",
        )

    rotor = model.part("fan_rotor")
    rotor_mesh = FanRotorGeometry(
        outer_radius=0.039,
        hub_radius=0.014,
        blade_count=9,
        thickness=0.010,
        blade_pitch_deg=32.0,
        blade_sweep_deg=26.0,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=13.0, camber=0.14, tip_clearance=0.0015),
        hub=FanRotorHub(style="capped"),
    )
    rotor.visual(
        mesh_from_geometry(rotor_mesh, "large_axial_fan_rotor"),
        origin=Origin(),
        material=fan_mat,
        name="rotor_mesh",
    )

    model.articulation(
        "fan_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=rotor,
        origin=Origin(xyz=(fan_center_x, 0.0, fan_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.03, velocity=180.0),
        motion_properties=MotionProperties(damping=0.001, friction=0.0001),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("card_body")
    rotor = object_model.get_part("fan_rotor")
    spin = object_model.get_articulation("fan_spin")

    ctx.allow_overlap(
        body,
        rotor,
        elem_a="bearing_pin",
        elem_b="rotor_mesh",
        reason="The stationary fan pin intentionally passes through the rotor hub as the retained spin bearing.",
    )

    ctx.check(
        "fan uses a continuous spin joint",
        spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={spin.articulation_type}",
    )
    ctx.expect_within(
        rotor,
        body,
        axes="xy",
        inner_elem="rotor_mesh",
        outer_elem="fan_shroud",
        margin=0.0,
        name="rotor stays inside shroud footprint",
    )
    ctx.expect_overlap(
        body,
        rotor,
        axes="z",
        elem_a="bearing_pin",
        elem_b="rotor_mesh",
        min_overlap=0.006,
        name="bearing pin passes through fan hub height",
    )
    ctx.expect_overlap(
        body,
        rotor,
        axes="xy",
        elem_a="bearing_pin",
        elem_b="rotor_mesh",
        min_overlap=0.004,
        name="bearing pin is centered in fan hub",
    )

    rest_position = ctx.part_world_position(rotor)
    with ctx.pose({spin: pi / 2.0}):
        spun_position = ctx.part_world_position(rotor)
        ctx.expect_within(
            rotor,
            body,
            axes="xy",
            inner_elem="rotor_mesh",
            outer_elem="fan_shroud",
            margin=0.0,
            name="spun rotor remains inside shroud footprint",
        )

    ctx.check(
        "spin keeps rotor clipped to hub",
        rest_position is not None
        and spun_position is not None
        and abs(rest_position[0] - spun_position[0]) < 1e-6
        and abs(rest_position[1] - spun_position[1]) < 1e-6
        and abs(rest_position[2] - spun_position[2]) < 1e-6,
        details=f"rest={rest_position}, spun={spun_position}",
    )

    return ctx.report()


object_model = build_object_model()
