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
    Material,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_W = 0.300
BODY_D = 0.220
BODY_H = 0.550
FRONT_Y = -BODY_D / 2.0
TOP_Z = BODY_H

RECESS_W = 0.250
RECESS_D = 0.150
RECESS_CENTER_Y = -0.015
RECESS_DEPTH = 0.040
RECESS_FLOOR_Z = TOP_Z - RECESS_DEPTH
HANDLE_HINGE_Y = 0.020
HANDLE_HINGE_Z = RECESS_FLOOR_Z + 0.018


def _speaker_enclosure_shape() -> cq.Workplane:
    """Rounded enclosure with real top and front recess cuts."""
    body = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H)
        .translate((0.0, 0.0, BODY_H / 2.0))
        .edges("|Z")
        .fillet(0.020)
    )

    top_cutter = (
        cq.Workplane("XY")
        .box(RECESS_W, RECESS_D, RECESS_DEPTH + 0.025)
        .edges("|Z")
        .fillet(0.018)
        .translate(
            (
                0.0,
                RECESS_CENTER_Y,
                TOP_Z - RECESS_DEPTH / 2.0 + 0.012,
            )
        )
    )
    body = body.cut(top_cutter)

    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_party_speaker")

    body_plastic = model.material("charcoal_molded_plastic", rgba=(0.035, 0.038, 0.042, 1.0))
    black_rubber = model.material("soft_black_rubber", rgba=(0.005, 0.005, 0.006, 1.0))
    dark_panel = model.material("satin_black_panel", rgba=(0.012, 0.014, 0.016, 1.0))
    grille_metal = model.material("black_perforated_metal", rgba=(0.018, 0.019, 0.020, 1.0))
    button_blue = model.material("blue_silicone_buttons", rgba=(0.05, 0.28, 0.78, 1.0))
    knob_gray = model.material("dark_gray_encoder_knob", rgba=(0.11, 0.115, 0.12, 1.0))
    hinge_pin_mat = model.material("brushed_hinge_pin", rgba=(0.55, 0.55, 0.50, 1.0))

    enclosure = model.part("enclosure")
    enclosure.visual(
        mesh_from_cadquery(_speaker_enclosure_shape(), "rounded_enclosure", tolerance=0.0015),
        material=body_plastic,
        name="rounded_shell",
    )

    # Dark pocket liner on the real cut recess floor so the folded handle reads
    # as sitting in a molded trough, not on top of the speaker.
    enclosure.visual(
        Box((RECESS_W - 0.020, RECESS_D - 0.018, 0.003)),
        origin=Origin(xyz=(0.0, RECESS_CENTER_Y, RECESS_FLOOR_Z + 0.001)),
        material=dark_panel,
        name="recess_floor_liner",
    )

    enclosure.visual(
        Box((0.214, 0.007, 0.098)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.0025, 0.430)),
        material=dark_panel,
        name="control_panel",
    )

    grille = PerforatedPanelGeometry(
        (0.232, 0.292),
        0.004,
        hole_diameter=0.006,
        pitch=(0.012, 0.012),
        frame=0.012,
        corner_radius=0.010,
        stagger=True,
        center=False,
    )
    enclosure.visual(
        mesh_from_geometry(grille, "front_speaker_grille"),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.001, 0.235), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grille_metal,
        name="speaker_grille",
    )

    # Rubber feet are slightly proud below the cabinet, touching the molded base.
    for ix, x in enumerate((-0.105, 0.105)):
        for iy, y in enumerate((-0.065, 0.065)):
            enclosure.visual(
                Cylinder(radius=0.018, length=0.012),
                origin=Origin(xyz=(x, y, -0.006)),
                material=black_rubber,
                name=f"foot_{ix}_{iy}",
            )

    # Fixed hinge cheek blocks in the top recess. Their inner faces touch the
    # rotating handle barrels and their bottoms are molded into the recess floor.
    for suffix, x in (("0", -0.096), ("1", 0.096)):
        enclosure.visual(
            Box((0.024, 0.036, 0.036)),
            origin=Origin(xyz=(x, HANDLE_HINGE_Y, HANDLE_HINGE_Z)),
            material=body_plastic,
            name=f"handle_socket_{suffix}",
        )
        enclosure.visual(
            Cylinder(radius=0.0065, length=0.026),
            origin=Origin(xyz=(x, HANDLE_HINGE_Y, HANDLE_HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hinge_pin_mat,
            name=f"hinge_pin_{suffix}",
        )

    handle = model.part("handle")
    handle.visual(
        Box((0.016, 0.070, 0.010)),
        origin=Origin(xyz=(-0.075, -0.034, 0.0)),
        material=black_rubber,
        name="handle_arm_0",
    )
    handle.visual(
        Box((0.016, 0.070, 0.010)),
        origin=Origin(xyz=(0.075, -0.034, 0.0)),
        material=black_rubber,
        name="handle_arm_1",
    )
    handle.visual(
        Box((0.166, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, -0.071, 0.0)),
        material=black_rubber,
        name="front_grip",
    )
    for suffix, x in (("0", -0.075), ("1", 0.075)):
        handle.visual(
            Cylinder(radius=0.010, length=0.018),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black_rubber,
            name=f"handle_barrel_{suffix}",
        )

    model.articulation(
        "enclosure_to_handle",
        ArticulationType.REVOLUTE,
        parent=enclosure,
        child=handle,
        origin=Origin(xyz=(0.0, HANDLE_HINGE_Y, HANDLE_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.30),
    )

    knob = model.part("knob")
    knob_shape = KnobGeometry(
        0.050,
        0.026,
        body_style="faceted",
        base_diameter=0.052,
        top_diameter=0.042,
        edge_radius=0.0012,
        grip=KnobGrip(style="ribbed", count=20, depth=0.0010, width=0.0020),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        center=False,
    )
    knob.visual(
        mesh_from_geometry(knob_shape, "rotary_knob"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_gray,
        name="knob_cap",
    )
    model.articulation(
        "enclosure_to_knob",
        ArticulationType.CONTINUOUS,
        parent=enclosure,
        child=knob,
        origin=Origin(xyz=(-0.060, FRONT_Y - 0.006, 0.448)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.45, velocity=8.0),
    )

    for index, x in enumerate((0.035, 0.086)):
        button = model.part(f"button_{index}")
        button.visual(
            Cylinder(radius=0.014, length=0.011),
            origin=Origin(xyz=(0.0, -0.0055, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=button_blue,
            name="button_cap",
        )
        button.visual(
            Cylinder(radius=0.006, length=0.008),
            origin=Origin(xyz=(0.0, 0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black_rubber,
            name="plunger_stem",
        )
        model.articulation(
            f"enclosure_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=enclosure,
            child=button,
            origin=Origin(xyz=(x, FRONT_Y - 0.006, 0.415)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.08, lower=0.0, upper=0.008),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    enclosure = object_model.get_part("enclosure")
    handle = object_model.get_part("handle")
    knob = object_model.get_part("knob")
    handle_joint = object_model.get_articulation("enclosure_to_handle")
    knob_joint = object_model.get_articulation("enclosure_to_knob")

    ctx.expect_within(
        handle,
        enclosure,
        axes="xy",
        inner_elem="front_grip",
        outer_elem="recess_floor_liner",
        margin=0.003,
        name="folded handle fits inside top recess footprint",
    )
    folded_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({handle_joint: 1.30}):
        raised_aabb = ctx.part_world_aabb(handle)
    ctx.check(
        "handle rotates upward above top surface",
        folded_aabb is not None
        and raised_aabb is not None
        and raised_aabb[1][2] > TOP_Z + 0.040
        and raised_aabb[1][2] > folded_aabb[1][2] + 0.050,
        details=f"folded={folded_aabb}, raised={raised_aabb}",
    )

    ctx.expect_contact(
        knob,
        enclosure,
        elem_a="knob_cap",
        elem_b="control_panel",
        contact_tol=0.0015,
        name="rotary knob mounts on control panel face",
    )
    ctx.check(
        "knob rotation axis faces forward",
        abs(knob_joint.axis[1]) > 0.95 and abs(knob_joint.axis[0]) < 0.05 and abs(knob_joint.axis[2]) < 0.05,
        details=f"axis={knob_joint.axis}",
    )

    for index in (0, 1):
        button = object_model.get_part(f"button_{index}")
        button_joint = object_model.get_articulation(f"enclosure_to_button_{index}")
        ctx.allow_overlap(
            button,
            enclosure,
            elem_a="plunger_stem",
            elem_b="control_panel",
            reason="The push button stem is intentionally represented as a short plunger captured inside the control panel socket.",
        )
        ctx.expect_within(
            button,
            enclosure,
            axes="xz",
            inner_elem="plunger_stem",
            outer_elem="control_panel",
            margin=0.001,
            name=f"button_{index} plunger is centered in panel socket",
        )
        ctx.expect_overlap(
            button,
            enclosure,
            axes="y",
            elem_a="plunger_stem",
            elem_b="control_panel",
            min_overlap=0.004,
            name=f"button_{index} plunger remains seated in panel",
        )
        ctx.expect_contact(
            button,
            enclosure,
            elem_a="button_cap",
            elem_b="control_panel",
            contact_tol=0.0015,
            name=f"button_{index} cap seats on panel face",
        )

        rest_pos = ctx.part_world_position(button)
        with ctx.pose({button_joint: 0.008}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button_{index} plunges inward",
            rest_pos is not None and pressed_pos is not None and pressed_pos[1] > rest_pos[1] + 0.006,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
