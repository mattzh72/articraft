from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TireTread,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_W = 0.320
BODY_D = 0.220
BODY_H = 0.460
BODY_CENTER_Z = 0.290
BODY_TOP_Z = BODY_CENTER_Z + BODY_H / 2.0
DECK_TOP_Z = 0.535
REAR_Y = BODY_D / 2.0
SLEEVE_Y = REAR_Y + 0.012
RAIL_X = 0.105
HANDLE_ORIGIN_Z = BODY_TOP_Z
HANDLE_TRAVEL = 0.280


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """CadQuery rounded rectangular solid in metres."""

    return cq.Workplane("XY").box(*size).edges("|Z").fillet(radius)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_karaoke_speaker")

    shell_mat = model.material("charcoal_plastic", rgba=(0.045, 0.050, 0.055, 1.0))
    trim_mat = model.material("black_rubber", rgba=(0.010, 0.010, 0.012, 1.0))
    grille_mat = model.material("perforated_black_metal", rgba=(0.005, 0.006, 0.007, 1.0))
    deck_mat = model.material("satin_control_deck", rgba=(0.075, 0.082, 0.090, 1.0))
    chrome_mat = model.material("brushed_metal", rgba=(0.62, 0.66, 0.68, 1.0))
    button_mat = model.material("dark_button_caps", rgba=(0.12, 0.14, 0.16, 1.0))
    mode_mat = model.material("blue_mode_buttons", rgba=(0.05, 0.20, 0.42, 1.0))
    rocker_mat = model.material("red_power_rocker", rgba=(0.55, 0.035, 0.025, 1.0))
    tire_mat = model.material("soft_black_tire", rgba=(0.012, 0.012, 0.014, 1.0))
    wheel_mat = model.material("molded_wheel_hub", rgba=(0.36, 0.38, 0.40, 1.0))

    chassis = model.part("chassis")

    body_shape = _rounded_box((BODY_W, BODY_D, BODY_H), 0.030)
    chassis.visual(
        mesh_from_cadquery(body_shape, "rounded_speaker_shell", tolerance=0.0012),
        origin=Origin(xyz=(0.0, 0.0, BODY_CENTER_Z)),
        material=shell_mat,
        name="rounded_shell",
    )

    # A real perforated front grille, slightly proud of the plastic shell.
    grille = PerforatedPanelGeometry(
        (0.255, 0.315),
        0.005,
        hole_diameter=0.0065,
        pitch=(0.013, 0.013),
        frame=0.014,
        corner_radius=0.018,
        stagger=True,
    )
    chassis.visual(
        mesh_from_geometry(grille, "front_perforated_grille"),
        origin=Origin(xyz=(0.0, -REAR_Y - 0.0018, 0.285), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grille_mat,
        name="front_grille",
    )
    # Dark speaker drivers sit behind the holes so the front reads as audio hardware.
    chassis.visual(
        Cylinder(radius=0.092, length=0.004),
        origin=Origin(xyz=(0.0, -REAR_Y - 0.0045, 0.255), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_mat,
        name="woofer_shadow",
    )
    chassis.visual(
        Cylinder(radius=0.038, length=0.004),
        origin=Origin(xyz=(0.0, -REAR_Y - 0.0045, 0.405), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_mat,
        name="tweeter_shadow",
    )

    deck_shape = _rounded_box((0.300, 0.150, 0.020), 0.016)
    chassis.visual(
        mesh_from_cadquery(deck_shape, "top_control_deck", tolerance=0.0008),
        origin=Origin(xyz=(0.0, -0.012, DECK_TOP_Z - 0.010)),
        material=deck_mat,
        name="top_deck",
    )

    # Rear trolley sleeves are fixed to the shell and visibly carry the sliding rails.
    for i, x in enumerate((-RAIL_X, RAIL_X)):
        chassis.visual(
            Cylinder(radius=0.014, length=0.320),
            origin=Origin(xyz=(x, SLEEVE_Y, 0.360)),
            material=chrome_mat,
            name=f"rear_sleeve_{i}",
        )
        chassis.visual(
            Cylinder(radius=0.019, length=0.018),
            origin=Origin(xyz=(x, SLEEVE_Y, BODY_TOP_Z - 0.004)),
            material=trim_mat,
            name=f"sleeve_collar_{i}",
        )

    # Wheel axle and fork-like brackets are part of the fixed chassis.
    chassis.visual(
        Cylinder(radius=0.006, length=0.390),
        origin=Origin(xyz=(0.0, REAR_Y + 0.024, 0.065), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome_mat,
        name="rear_axle",
    )
    for i, x in enumerate((-0.150, 0.150)):
        chassis.visual(
            Box((0.020, 0.055, 0.056)),
            origin=Origin(xyz=(x, REAR_Y + 0.010, 0.066)),
            material=shell_mat,
            name=f"axle_bracket_{i}",
        )
    for i, x in enumerate((-0.095, 0.095)):
        chassis.visual(
            Box((0.060, 0.032, 0.020)),
            origin=Origin(xyz=(x, -0.060, 0.050)),
            material=trim_mat,
            name=f"front_foot_{i}",
        )

    # Telescoping trolley handle: twin sliding rails joined by a rounded grip.
    handle = model.part("trolley_handle")
    for i, x in enumerate((-RAIL_X, RAIL_X)):
        handle.visual(
            Cylinder(radius=0.008, length=0.380),
            origin=Origin(xyz=(x, 0.0, -0.130)),
            material=chrome_mat,
            name=f"handle_rail_{i}",
        )
    handle.visual(
        Cylinder(radius=0.015, length=0.255),
        origin=Origin(xyz=(0.0, 0.0, 0.074), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_mat,
        name="handle_grip",
    )
    for i, x in enumerate((-RAIL_X, RAIL_X)):
        handle.visual(
            Cylinder(radius=0.014, length=0.035),
            origin=Origin(xyz=(x, 0.0, 0.058)),
            material=trim_mat,
            name=f"grip_socket_{i}",
        )
    model.articulation(
        "handle_slide",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=handle,
        origin=Origin(xyz=(0.0, SLEEVE_Y, HANDLE_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=HANDLE_TRAVEL),
    )

    # Rear transport wheels.  The tire helper supplies a real treaded rubber
    # carcass; the molded hub and caps are centered on the continuous spin axis.
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.055,
            0.034,
            inner_radius=0.037,
            tread=TireTread(style="block", depth=0.004, count=16, land_ratio=0.58),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.004, radius=0.002),
        ),
        "transport_tire",
    )
    for i, x in enumerate((-0.185, 0.185)):
        wheel = model.part(f"rear_wheel_{i}")
        wheel.visual(tire_mesh, material=tire_mat, name="tire")
        wheel.visual(
            Cylinder(radius=0.038, length=0.030),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=wheel_mat,
            name="wheel_core",
        )
        model.articulation(
            f"wheel_spin_{i}",
            ArticulationType.CONTINUOUS,
            parent=chassis,
            child=wheel,
            origin=Origin(xyz=(x, REAR_Y + 0.024, 0.065)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=20.0),
        )

    # Main rotary control on the top deck.
    knob = model.part("main_knob")
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.072,
                0.038,
                body_style="skirted",
                top_diameter=0.058,
                skirt=KnobSkirt(0.083, 0.006, flare=0.06, chamfer=0.001),
                grip=KnobGrip(style="ribbed", count=22, depth=0.0012, width=0.002),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "large_volume_knob",
        ),
        material=trim_mat,
        name="knob_body",
    )
    model.articulation(
        "knob_turn",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=knob,
        origin=Origin(xyz=(-0.075, -0.010, DECK_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=8.0),
    )

    # Two mode buttons on the left side of the deck.
    for i, x in enumerate((-0.113, -0.038)):
        mode = model.part(f"mode_button_{i}")
        mode.visual(
            Box((0.048, 0.024, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, 0.005)),
            material=mode_mat,
            name="button_cap",
        )
        model.articulation(
            f"mode_button_press_{i}",
            ArticulationType.PRISMATIC,
            parent=chassis,
            child=mode,
            origin=Origin(xyz=(x, -0.064, DECK_TOP_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.20, lower=0.0, upper=0.006),
        )

    # Power rocker is a larger red seesaw; two smaller menu buttons sit beside it.
    rocker = model.part("power_rocker")
    rocker_shape = _rounded_box((0.046, 0.030, 0.012), 0.004)
    rocker.visual(
        mesh_from_cadquery(rocker_shape, "power_rocker_cap", tolerance=0.0005),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=rocker_mat,
        name="rocker_cap",
    )
    rocker.visual(
        Cylinder(radius=0.005, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.001), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_mat,
        name="rocker_pivot_bar",
    )
    model.articulation(
        "rocker_pivot",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=rocker,
        origin=Origin(xyz=(0.057, 0.036, DECK_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=3.0, lower=-0.28, upper=0.28),
    )

    for i, x in enumerate((0.107, 0.137)):
        menu = model.part(f"menu_button_{i}")
        menu.visual(
            Cylinder(radius=0.010, length=0.009),
            origin=Origin(xyz=(0.0, 0.0, 0.0045)),
            material=button_mat,
            name="button_cap",
        )
        model.articulation(
            f"menu_button_press_{i}",
            ArticulationType.PRISMATIC,
            parent=chassis,
            child=menu,
            origin=Origin(xyz=(x, 0.036, DECK_TOP_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=1.5, velocity=0.18, lower=0.0, upper=0.0055),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    handle = object_model.get_part("trolley_handle")
    slide = object_model.get_articulation("handle_slide")

    # The sliding rails are intentionally nested inside visible rear sleeves.
    for i in (0, 1):
        ctx.allow_overlap(
            chassis,
            handle,
            elem_a=f"rear_sleeve_{i}",
            elem_b=f"handle_rail_{i}",
            reason="The trolley rail is intentionally represented as a telescoping member retained inside its rear sleeve.",
        )
        ctx.expect_within(
            handle,
            chassis,
            axes="xy",
            inner_elem=f"handle_rail_{i}",
            outer_elem=f"rear_sleeve_{i}",
            margin=0.001,
            name=f"handle rail {i} stays centered in sleeve",
        )
        ctx.expect_overlap(
            handle,
            chassis,
            axes="z",
            elem_a=f"handle_rail_{i}",
            elem_b=f"rear_sleeve_{i}",
            min_overlap=0.250,
            name=f"collapsed handle rail {i} remains deeply inserted",
        )
        ctx.allow_overlap(
            chassis,
            handle,
            elem_a=f"sleeve_collar_{i}",
            elem_b=f"handle_rail_{i}",
            reason="The rail passes through the solid simplified collar at the sleeve mouth.",
        )
        ctx.expect_within(
            handle,
            chassis,
            axes="xy",
            inner_elem=f"handle_rail_{i}",
            outer_elem=f"sleeve_collar_{i}",
            margin=0.001,
            name=f"handle rail {i} passes through collar center",
        )
        ctx.expect_overlap(
            handle,
            chassis,
            axes="z",
            elem_a=f"handle_rail_{i}",
            elem_b=f"sleeve_collar_{i}",
            min_overlap=0.015,
            name=f"handle rail {i} is captured by collar",
        )
        with ctx.pose({slide: HANDLE_TRAVEL}):
            ctx.expect_within(
                handle,
                chassis,
                axes="xy",
                inner_elem=f"handle_rail_{i}",
                outer_elem=f"rear_sleeve_{i}",
                margin=0.001,
                name=f"extended handle rail {i} stays centered",
            )
            ctx.expect_overlap(
                handle,
                chassis,
                axes="z",
                elem_a=f"handle_rail_{i}",
                elem_b=f"rear_sleeve_{i}",
                min_overlap=0.035,
                name=f"extended handle rail {i} retains insertion",
            )

    for i in (0, 1):
        wheel = object_model.get_part(f"rear_wheel_{i}")
        ctx.allow_overlap(
            chassis,
            wheel,
            elem_a="rear_axle",
            elem_b="wheel_core",
            reason="The wheel core is intentionally represented as spinning around the captured axle.",
        )
        ctx.expect_within(
            chassis,
            wheel,
            axes="yz",
            inner_elem="rear_axle",
            outer_elem="wheel_core",
            margin=0.001,
            name=f"wheel {i} core surrounds axle centerline",
        )
        ctx.expect_overlap(
            chassis,
            wheel,
            axes="x",
            elem_a="rear_axle",
            elem_b="wheel_core",
            min_overlap=0.025,
            name=f"wheel {i} has axle engagement",
        )

    rest_handle_pos = ctx.part_world_position(handle)
    with ctx.pose({slide: HANDLE_TRAVEL}):
        extended_handle_pos = ctx.part_world_position(handle)
    ctx.check(
        "trolley handle slides upward",
        rest_handle_pos is not None
        and extended_handle_pos is not None
        and extended_handle_pos[2] > rest_handle_pos[2] + 0.250,
        details=f"rest={rest_handle_pos}, extended={extended_handle_pos}",
    )

    # Prompt-critical controls and wheels have the requested articulation types.
    expected_types = {
        "knob_turn": ArticulationType.CONTINUOUS,
        "wheel_spin_0": ArticulationType.CONTINUOUS,
        "wheel_spin_1": ArticulationType.CONTINUOUS,
        "rocker_pivot": ArticulationType.REVOLUTE,
        "menu_button_press_0": ArticulationType.PRISMATIC,
        "menu_button_press_1": ArticulationType.PRISMATIC,
        "mode_button_press_0": ArticulationType.PRISMATIC,
        "mode_button_press_1": ArticulationType.PRISMATIC,
    }
    for joint_name, joint_type in expected_types.items():
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} has correct motion type",
            joint.articulation_type == joint_type,
            details=f"expected {joint_type}, got {joint.articulation_type}",
        )

    # Buttons sit on the top deck at rest and move down independently when pressed.
    for joint_name, part_name, travel in (
        ("menu_button_press_0", "menu_button_0", 0.0055),
        ("menu_button_press_1", "menu_button_1", 0.0055),
        ("mode_button_press_0", "mode_button_0", 0.0060),
        ("mode_button_press_1", "mode_button_1", 0.0060),
    ):
        button = object_model.get_part(part_name)
        joint = object_model.get_articulation(joint_name)
        rest = ctx.part_world_position(button)
        with ctx.pose({joint: travel}):
            pressed = ctx.part_world_position(button)
        ctx.check(
            f"{part_name} depresses downward",
            rest is not None and pressed is not None and pressed[2] < rest[2] - travel * 0.8,
            details=f"rest={rest}, pressed={pressed}",
        )

    ctx.expect_gap(
        object_model.get_part("menu_button_0"),
        object_model.get_part("power_rocker"),
        axis="x",
        min_gap=0.008,
        name="rocker is distinct from adjacent menu buttons",
    )

    return ctx.report()


object_model = build_object_model()
