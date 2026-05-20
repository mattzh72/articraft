from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="freestanding_candy_vending_machine")

    red_enamel = Material("red_enamel", rgba=(0.72, 0.03, 0.025, 1.0))
    dark_red = Material("dark_red_shadow", rgba=(0.36, 0.01, 0.01, 1.0))
    chrome = Material("polished_chrome", rgba=(0.78, 0.78, 0.74, 1.0))
    brass = Material("brushed_brass", rgba=(0.95, 0.67, 0.28, 1.0))
    black = Material("black_recess", rgba=(0.01, 0.01, 0.012, 1.0))
    glass = Material("clear_glass", rgba=(0.68, 0.90, 1.0, 0.30))
    label_white = Material("white_label", rgba=(0.96, 0.94, 0.84, 1.0))
    candy_mats = (
        Material("candy_red", rgba=(0.95, 0.03, 0.04, 1.0)),
        Material("candy_yellow", rgba=(1.0, 0.82, 0.05, 1.0)),
        Material("candy_blue", rgba=(0.05, 0.25, 0.95, 1.0)),
        Material("candy_green", rgba=(0.04, 0.68, 0.20, 1.0)),
        Material("candy_orange", rgba=(1.0, 0.42, 0.02, 1.0)),
        Material("candy_purple", rgba=(0.50, 0.12, 0.72, 1.0)),
    )

    machine = model.part("machine")

    # Heavy freestanding pedestal and column.
    machine.visual(
        Cylinder(radius=0.29, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=red_enamel,
        name="round_floor_base",
    )
    machine.visual(
        Cylinder(radius=0.225, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.092)),
        material=chrome,
        name="base_trim_ring",
    )
    machine.visual(
        Cylinder(radius=0.075, length=0.560),
        origin=Origin(xyz=(0.0, 0.0, 0.370)),
        material=red_enamel,
        name="pedestal_column",
    )
    machine.visual(
        Cylinder(radius=0.115, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.635)),
        material=chrome,
        name="column_collar",
    )

    # Coin-box housing under the globe.
    machine.visual(
        Box((0.420, 0.320, 0.320)),
        origin=Origin(xyz=(0.0, 0.0, 0.785)),
        material=red_enamel,
        name="coin_box_body",
    )
    machine.visual(
        Box((0.360, 0.020, 0.245)),
        origin=Origin(xyz=(0.0, -0.169, 0.805)),
        material=brass,
        name="coin_panel",
    )
    machine.visual(
        Box((0.115, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, -0.182, 0.885)),
        material=black,
        name="coin_slot",
    )
    machine.visual(
        Box((0.150, 0.008, 0.045)),
        origin=Origin(xyz=(0.0, -0.178, 0.715)),
        material=black,
        name="coin_return_recess",
    )
    machine.visual(
        Box((0.115, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, -0.190, 0.940)),
        material=label_white,
        name="price_label",
    )

    # Front dispensing chute: a dark opening with a protruding collection tray.
    machine.visual(
        Box((0.185, 0.014, 0.065)),
        origin=Origin(xyz=(0.0, -0.183, 0.610)),
        material=black,
        name="chute_opening",
    )
    machine.visual(
        Box((0.230, 0.030, 0.075)),
        origin=Origin(xyz=(0.0, -0.166, 0.603)),
        material=chrome,
        name="chute_back_mount",
    )
    machine.visual(
        Box((0.230, 0.130, 0.018)),
        origin=Origin(xyz=(0.0, -0.220, 0.584)),
        material=chrome,
        name="chute_floor",
    )
    machine.visual(
        Box((0.018, 0.125, 0.070)),
        origin=Origin(xyz=(-0.124, -0.221, 0.618)),
        material=chrome,
        name="chute_side_0",
    )
    machine.visual(
        Box((0.018, 0.125, 0.070)),
        origin=Origin(xyz=(0.124, -0.221, 0.618)),
        material=chrome,
        name="chute_side_1",
    )
    machine.visual(
        Box((0.248, 0.018, 0.045)),
        origin=Origin(xyz=(0.0, -0.290, 0.600)),
        material=chrome,
        name="chute_front_lip",
    )

    # Reservoir support, clear hollow globe, and lid.
    machine.visual(
        Cylinder(radius=0.245, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.968)),
        material=red_enamel,
        name="reservoir_base_cap",
    )
    machine.visual(
        Cylinder(radius=0.202, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.997)),
        material=chrome,
        name="lower_glass_seat",
    )

    reservoir_shell = LatheGeometry.from_shell_profiles(
        outer_profile=(
            (0.158, 0.000),
            (0.212, 0.055),
            (0.226, 0.180),
            (0.205, 0.310),
            (0.150, 0.365),
        ),
        inner_profile=(
            (0.145, 0.010),
            (0.199, 0.062),
            (0.213, 0.180),
            (0.192, 0.302),
            (0.137, 0.355),
        ),
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=5,
    )
    machine.visual(
        mesh_from_geometry(reservoir_shell, "clear_product_reservoir"),
        origin=Origin(xyz=(0.0, 0.0, 0.990)),
        material=glass,
        name="reservoir_shell",
    )
    machine.visual(
        Cylinder(radius=0.168, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 1.373)),
        material=red_enamel,
        name="reservoir_lid",
    )
    machine.visual(
        Cylinder(radius=0.055, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 1.420)),
        material=chrome,
        name="lid_knob",
    )

    # Visible product pile inside the transparent reservoir.  The gumballs are
    # slightly nested into each other and the metal seat so the pile reads as a
    # single supported mass rather than loose floating islands.
    candy_positions = (
        (0.000, 0.000, 1.017),
        (0.060, 0.010, 1.020),
        (-0.058, 0.018, 1.022),
        (0.010, 0.066, 1.020),
        (-0.004, -0.064, 1.022),
        (0.112, 0.040, 1.024),
        (-0.105, -0.045, 1.026),
        (0.050, -0.075, 1.078),
        (-0.058, 0.078, 1.080),
        (0.000, 0.000, 1.088),
        (0.096, -0.018, 1.092),
        (-0.094, 0.000, 1.095),
        (0.026, 0.055, 1.150),
        (-0.036, -0.048, 1.154),
        (0.066, 0.018, 1.160),
    )
    for idx, xyz in enumerate(candy_positions):
        machine.visual(
            Sphere(radius=0.045),
            origin=Origin(xyz=xyz),
            material=candy_mats[idx % len(candy_mats)],
            name=f"gumball_{idx}",
        )

    # The front rotary dispense knob is a separate articulated part.
    knob = model.part("dispense_knob")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.122,
            0.044,
            body_style="faceted",
            base_diameter=0.130,
            top_diameter=0.104,
            edge_radius=0.002,
            grip=KnobGrip(style="ribbed", count=18, depth=0.0025, width=0.006),
            indicator=KnobIndicator(style="wedge", mode="raised", angle_deg=0.0),
        ),
        "front_dispense_knob",
    )
    knob.visual(
        Cylinder(radius=0.036, length=0.012),
        origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="knob_collar",
    )
    knob.visual(
        Cylinder(radius=0.018, length=0.032),
        origin=Origin(xyz=(0.0, -0.016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="knob_shaft",
    )
    knob.visual(
        knob_mesh,
        origin=Origin(xyz=(0.0, -0.052, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="knob_cap",
    )
    knob.visual(
        Cylinder(radius=0.026, length=0.020),
        origin=Origin(xyz=(0.055, -0.077, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="handle_boss",
    )
    knob.visual(
        Cylinder(radius=0.016, length=0.040),
        origin=Origin(xyz=(0.055, -0.096, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_red,
        name="handle_grip",
    )

    model.articulation(
        "machine_to_dispense_knob",
        ArticulationType.REVOLUTE,
        parent=machine,
        child=knob,
        origin=Origin(xyz=(0.0, -0.179, 0.805)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=3.0, lower=0.0, upper=math.tau),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    machine = object_model.get_part("machine")
    knob = object_model.get_part("dispense_knob")
    joint = object_model.get_articulation("machine_to_dispense_knob")

    ctx.expect_contact(
        knob,
        machine,
        elem_a="knob_collar",
        elem_b="coin_panel",
        contact_tol=0.002,
        name="dispense knob collar seats on coin panel",
    )

    rest_aabb = ctx.part_element_world_aabb(knob, elem="handle_grip")
    with ctx.pose({joint: math.pi / 2.0}):
        turned_aabb = ctx.part_element_world_aabb(knob, elem="handle_grip")

    rest_z = None if rest_aabb is None else (rest_aabb[0][2] + rest_aabb[1][2]) * 0.5
    turned_z = None if turned_aabb is None else (turned_aabb[0][2] + turned_aabb[1][2]) * 0.5
    ctx.check(
        "front handle rotates upward with dispense knob",
        rest_z is not None and turned_z is not None and turned_z > rest_z + 0.045,
        details=f"rest_z={rest_z}, turned_z={turned_z}",
    )

    return ctx.report()


object_model = build_object_model()
