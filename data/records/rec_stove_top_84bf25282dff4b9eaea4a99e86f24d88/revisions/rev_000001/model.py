from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


COOKTOP_WIDTH = 0.76
COOKTOP_DEPTH = 0.52
FRAME_BAND = 0.03
TOP_FRAME_THICKNESS = 0.008
INSET_PANEL_THICKNESS = 0.006
INSET_PANEL_TOP_Z = -0.002
CONTROL_Z = -0.031
CONTROL_Y = -(COOKTOP_DEPTH / 2.0) + 0.0005


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gas_cooktop")

    stainless = model.material("stainless", rgba=(0.74, 0.75, 0.77, 1.0))
    enamel = model.material("enamel_black", rgba=(0.08, 0.08, 0.09, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.12, 0.12, 0.13, 1.0))
    burner_metal = model.material("burner_metal", rgba=(0.34, 0.35, 0.37, 1.0))
    knob_finish = model.material("knob_finish", rgba=(0.11, 0.11, 0.12, 1.0))

    body = model.part("body")

    body.visual(
        Box((FRAME_BAND, COOKTOP_DEPTH, TOP_FRAME_THICKNESS)),
        origin=Origin(xyz=(-(COOKTOP_WIDTH - FRAME_BAND) / 2.0, 0.0, -TOP_FRAME_THICKNESS / 2.0)),
        material=stainless,
        name="left_trim",
    )
    body.visual(
        Box((FRAME_BAND, COOKTOP_DEPTH, TOP_FRAME_THICKNESS)),
        origin=Origin(xyz=((COOKTOP_WIDTH - FRAME_BAND) / 2.0, 0.0, -TOP_FRAME_THICKNESS / 2.0)),
        material=stainless,
        name="right_trim",
    )
    body.visual(
        Box((COOKTOP_WIDTH - (2.0 * FRAME_BAND), FRAME_BAND, TOP_FRAME_THICKNESS)),
        origin=Origin(xyz=(0.0, -(COOKTOP_DEPTH - FRAME_BAND) / 2.0, -TOP_FRAME_THICKNESS / 2.0)),
        material=stainless,
        name="front_trim",
    )
    body.visual(
        Box((COOKTOP_WIDTH - (2.0 * FRAME_BAND), FRAME_BAND, TOP_FRAME_THICKNESS)),
        origin=Origin(xyz=(0.0, (COOKTOP_DEPTH - FRAME_BAND) / 2.0, -TOP_FRAME_THICKNESS / 2.0)),
        material=stainless,
        name="rear_trim",
    )

    inset_width = COOKTOP_WIDTH - (2.0 * FRAME_BAND)
    inset_depth = COOKTOP_DEPTH - (2.0 * FRAME_BAND)
    body.visual(
        Box((inset_width, inset_depth, INSET_PANEL_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, INSET_PANEL_TOP_Z - (INSET_PANEL_THICKNESS / 2.0))),
        material=enamel,
        name="inset_panel",
    )
    body.visual(
        Box((0.68, 0.40, 0.040)),
        origin=Origin(xyz=(0.0, 0.025, -0.028)),
        material=enamel,
        name="chassis",
    )
    body.visual(
        Box((0.72, 0.025, 0.047)),
        origin=Origin(xyz=(0.0, -0.247, -0.0315)),
        material=enamel,
        name="front_fascia",
    )

    burner_specs = {
        "front_left_burner": {
            "pos": (-0.170, -0.075),
            "seat_radius": 0.065,
            "support_radius": 0.060,
            "head_radius": 0.044,
            "cap_radius": 0.026,
            "grate_outer": 0.160,
        },
        "front_right_burner": {
            "pos": (0.170, -0.075),
            "seat_radius": 0.058,
            "support_radius": 0.053,
            "head_radius": 0.037,
            "cap_radius": 0.022,
            "grate_outer": 0.146,
        },
        "rear_left_burner": {
            "pos": (-0.170, 0.120),
            "seat_radius": 0.058,
            "support_radius": 0.053,
            "head_radius": 0.037,
            "cap_radius": 0.022,
            "grate_outer": 0.146,
        },
        "rear_right_burner": {
            "pos": (0.170, 0.120),
            "seat_radius": 0.065,
            "support_radius": 0.060,
            "head_radius": 0.044,
            "cap_radius": 0.026,
            "grate_outer": 0.160,
        },
    }

    for burner_name, spec in burner_specs.items():
        x_pos, y_pos = spec["pos"]
        body.visual(
            Cylinder(radius=spec["seat_radius"], length=0.003),
            origin=Origin(xyz=(x_pos, y_pos, -0.0005)),
            material=burner_metal,
            name=f"{burner_name}_seat",
        )

        burner = model.part(burner_name)
        burner.visual(
            Cylinder(radius=spec["support_radius"], length=0.004),
            origin=Origin(xyz=(0.0, 0.0, 0.002)),
            material=burner_metal,
            name="support_ring",
        )
        burner.visual(
            Cylinder(radius=spec["head_radius"], length=0.008),
            origin=Origin(xyz=(0.0, 0.0, 0.008)),
            material=burner_metal,
            name="burner_head",
        )
        burner.visual(
            Cylinder(radius=spec["cap_radius"], length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.015)),
            material=cast_iron,
            name="burner_cap",
        )

        grate_outer = spec["grate_outer"]
        grate_offset = (grate_outer / 2.0) - 0.021
        frame_run = grate_outer - 0.042
        cross_run = (2.0 * grate_offset) - 0.012

        burner.visual(
            Box((frame_run, 0.012, 0.012)),
            origin=Origin(xyz=(0.0, -grate_offset, 0.032)),
            material=cast_iron,
            name="front_grate",
        )
        burner.visual(
            Box((frame_run, 0.012, 0.012)),
            origin=Origin(xyz=(0.0, grate_offset, 0.032)),
            material=cast_iron,
            name="rear_grate",
        )
        burner.visual(
            Box((0.012, frame_run, 0.012)),
            origin=Origin(xyz=(-grate_offset, 0.0, 0.032)),
            material=cast_iron,
            name="left_grate",
        )
        burner.visual(
            Box((0.012, frame_run, 0.012)),
            origin=Origin(xyz=(grate_offset, 0.0, 0.032)),
            material=cast_iron,
            name="right_grate",
        )
        burner.visual(
            Box((cross_run, 0.010, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, 0.029)),
            material=cast_iron,
            name="cross_x",
        )
        burner.visual(
            Box((0.010, cross_run, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, 0.029)),
            material=cast_iron,
            name="cross_y",
        )

        leg_positions = (
            (0.0, -grate_offset),
            (0.0, grate_offset),
            (-grate_offset, 0.0),
            (grate_offset, 0.0),
        )
        for leg_index, (x_leg, y_leg) in enumerate(leg_positions):
            burner.visual(
                Cylinder(radius=0.006, length=0.022),
                origin=Origin(xyz=(x_leg, y_leg, 0.015)),
                material=cast_iron,
                name=f"leg_{leg_index}",
            )

        model.articulation(
            f"body_to_{burner_name}",
            ArticulationType.FIXED,
            parent=body,
            child=burner,
            origin=Origin(xyz=(x_pos, y_pos, 0.001)),
        )

    knob_x_positions = (-0.264, -0.132, 0.0, 0.132, 0.264)
    for knob_index, x_pos in enumerate(knob_x_positions):
        knob = model.part(f"knob_{knob_index}")
        knob.visual(
            mesh_from_geometry(
                KnobGeometry(
                    0.042,
                    0.024,
                    body_style="skirted",
                    top_diameter=0.034,
                    skirt=KnobSkirt(0.055, 0.006, flare=0.07),
                    grip=KnobGrip(style="fluted", count=16, depth=0.0013),
                    indicator=KnobIndicator(
                        style="line",
                        mode="engraved",
                        depth=0.0008,
                        angle_deg=0.0,
                    ),
                    bore=KnobBore(style="d_shaft", diameter=0.006, flat_depth=0.001),
                    center=False,
                ),
                f"cooktop_knob_{knob_index}",
            ),
            material=knob_finish,
            name="knob_shell",
        )

        model.articulation(
            f"body_to_knob_{knob_index}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob,
            origin=Origin(xyz=(x_pos, CONTROL_Y, CONTROL_Z), rpy=(pi / 2.0, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.6, velocity=10.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    inset_panel = body.get_visual("inset_panel")
    front_fascia = body.get_visual("front_fascia")

    burner_names = (
        "front_left_burner",
        "front_right_burner",
        "rear_left_burner",
        "rear_right_burner",
    )
    knob_names = tuple(f"knob_{index}" for index in range(5))

    for burner_name in burner_names:
        burner = object_model.get_part(burner_name)
        ctx.expect_overlap(
            burner,
            body,
            axes="xy",
            elem_b=inset_panel,
            min_overlap=0.10,
            name=f"{burner_name}_over_inset_panel",
        )
        ctx.expect_gap(
            burner,
            body,
            axis="z",
            positive_elem="support_ring",
            negative_elem=f"{burner_name}_seat",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"{burner_name}_seats_on_trim_ring",
        )

    front_left_burner = object_model.get_part("front_left_burner")
    rear_left_burner = object_model.get_part("rear_left_burner")
    front_right_burner = object_model.get_part("front_right_burner")
    rear_right_burner = object_model.get_part("rear_right_burner")

    ctx.expect_origin_gap(
        rear_left_burner,
        front_left_burner,
        axis="y",
        min_gap=0.16,
        name="left_burners_form_front_and_rear_rows",
    )
    ctx.expect_origin_gap(
        rear_right_burner,
        front_right_burner,
        axis="y",
        min_gap=0.16,
        name="right_burners_form_front_and_rear_rows",
    )

    for knob_name in knob_names:
        knob = object_model.get_part(knob_name)
        joint = object_model.get_articulation(f"body_to_{knob_name}")
        ctx.check(
            f"{knob_name}_is_continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"type={joint.articulation_type!r}",
        )
        ctx.expect_gap(
            body,
            knob,
            axis="y",
            positive_elem=front_fascia,
            negative_elem="knob_shell",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"{knob_name}_mounts_flush_to_fascia",
        )
        ctx.expect_origin_gap(
            front_left_burner,
            knob,
            axis="y",
            min_gap=0.16,
            name=f"{knob_name}_sits_ahead_of_burner_field",
        )

    knob_positions = [ctx.part_world_position(object_model.get_part(knob_name)) for knob_name in knob_names]
    knobs_aligned = all(pos is not None for pos in knob_positions)
    if knobs_aligned:
        z_values = [pos[2] for pos in knob_positions if pos is not None]
        y_values = [pos[1] for pos in knob_positions if pos is not None]
        x_values = [pos[0] for pos in knob_positions if pos is not None]
        ctx.check(
            "front_knobs_share_common_axis_height",
            max(z_values) - min(z_values) <= 1e-6 and max(y_values) - min(y_values) <= 1e-6,
            details=f"positions={knob_positions!r}",
        )
        ctx.check(
            "front_knobs_span_the_lower_edge",
            all(x_values[index] < x_values[index + 1] for index in range(len(x_values) - 1))
            and (x_values[-1] - x_values[0]) >= 0.50,
            details=f"positions={knob_positions!r}",
        )
    else:
        ctx.fail("front_knob_positions_available", f"positions={knob_positions!r}")

    center_knob = object_model.get_part("knob_2")
    center_joint = object_model.get_articulation("body_to_knob_2")
    rest_pos = ctx.part_world_position(center_knob)
    with ctx.pose({center_joint: pi / 2.0}):
        turned_pos = ctx.part_world_position(center_knob)
    ctx.check(
        "center_knob_rotates_without_translation",
        rest_pos is not None
        and turned_pos is not None
        and all(abs(rest_pos[index] - turned_pos[index]) <= 1e-6 for index in range(3)),
        details=f"rest={rest_pos!r}, turned={turned_pos!r}",
    )

    return ctx.report()


object_model = build_object_model()
