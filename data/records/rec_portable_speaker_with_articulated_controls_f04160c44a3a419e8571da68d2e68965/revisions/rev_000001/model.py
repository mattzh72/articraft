from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


WIDTH = 0.34
DEPTH = 0.26
HEIGHT = 0.72
FRONT_Y = -DEPTH / 2.0


def _speaker_body_shape() -> ExtrudeGeometry:
    """Tall rounded portable enclosure body."""
    return ExtrudeGeometry(rounded_rect_profile(WIDTH, DEPTH, 0.034, corner_segments=10), HEIGHT)


def _rounded_button_shape(width: float, height: float, thickness: float) -> ExtrudeGeometry:
    return ExtrudeGeometry(
        rounded_rect_profile(width, height, min(width, height) * 0.28, corner_segments=8),
        thickness,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_party_speaker")

    model.material("molded_black", rgba=(0.015, 0.016, 0.018, 1.0))
    model.material("charcoal_panel", rgba=(0.045, 0.048, 0.052, 1.0))
    model.material("perforated_steel", rgba=(0.010, 0.011, 0.012, 1.0))
    model.material("dark_rubber", rgba=(0.025, 0.025, 0.026, 1.0))
    model.material("button_gray", rgba=(0.33, 0.34, 0.35, 1.0))
    model.material("knob_silver", rgba=(0.72, 0.72, 0.68, 1.0))
    model.material("hinge_graphite", rgba=(0.10, 0.105, 0.11, 1.0))

    enclosure = model.part("enclosure")
    enclosure.visual(
        mesh_from_geometry(_speaker_body_shape(), "rounded_enclosure"),
        origin=Origin(xyz=(0.0, 0.0, HEIGHT / 2.0)),
        material="molded_black",
        name="shell",
    )

    # Flush, dark control face above the grille.  Controls mount to this face as
    # independent articulated parts instead of being fused decorations.
    control_face_y = FRONT_Y - 0.003
    enclosure.visual(
        Box((0.270, 0.006, 0.118)),
        origin=Origin(xyz=(0.0, control_face_y, 0.565)),
        material="charcoal_panel",
        name="control_face",
    )

    grille = PerforatedPanelGeometry(
        (0.286, 0.390),
        0.006,
        hole_diameter=0.008,
        pitch=(0.0155, 0.0155),
        frame=0.014,
        corner_radius=0.020,
        stagger=True,
    )
    enclosure.visual(
        mesh_from_geometry(grille, "front_grille"),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.0015, 0.288), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="perforated_steel",
        name="front_grille",
    )

    # Top pocket floor and side hinge sockets are fixed to the enclosure.  The
    # sockets intentionally capture the moving handle sleeves.
    enclosure.visual(
        Box((0.255, 0.130, 0.005)),
        origin=Origin(xyz=(0.0, -0.020, HEIGHT + 0.001)),
        material="charcoal_panel",
        name="handle_pocket",
    )
    for i, (x, y, sx, sy) in enumerate(
        (
            (0.0, -0.095, 0.285, 0.016),
            (0.0, 0.086, 0.285, 0.016),
            (-0.142, -0.020, 0.016, 0.130),
            (0.142, -0.020, 0.016, 0.130),
        )
    ):
        enclosure.visual(
            Box((sx, sy, 0.010)),
            origin=Origin(xyz=(x, y, HEIGHT + 0.005)),
            material="molded_black",
            name=f"pocket_rim_{i}",
        )
    for i, x in enumerate((-0.132, 0.132)):
        enclosure.visual(
            Box((0.058, 0.040, 0.012)),
            origin=Origin(xyz=(x, 0.058, HEIGHT - 0.003)),
            material="hinge_graphite",
            name=f"hinge_base_{i}",
        )
        enclosure.visual(
            Cylinder(radius=0.019, length=0.044),
            origin=Origin(xyz=(x, 0.058, HEIGHT + 0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="hinge_graphite",
            name=f"hinge_socket_{i}",
        )

    for i, (x, z) in enumerate(
        ((-0.123, 0.117), (0.123, 0.117), (-0.123, 0.459), (0.123, 0.459))
    ):
        enclosure.visual(
            Box((0.026, 0.010, 0.020)),
            origin=Origin(xyz=(x, FRONT_Y - 0.0015, z)),
            material="hinge_graphite",
            name=f"grille_mount_{i}",
        )

    # Four rubber feet keep the tall portable cabinet visibly freestanding.
    for i, (x, y) in enumerate(
        ((-0.118, -0.083), (0.118, -0.083), (-0.118, 0.083), (0.118, 0.083))
    ):
        enclosure.visual(
            Box((0.070, 0.046, 0.018)),
            origin=Origin(xyz=(x, y, -0.006)),
            material="dark_rubber",
            name=f"foot_{i}",
        )

    # Rotating top handle, authored in its hinge frame.  At q=0 it nests inside
    # the top recess; positive rotation raises the grip.
    handle = model.part("handle")
    for i, x in enumerate((-0.132, 0.132)):
        handle.visual(
            Cylinder(radius=0.014, length=0.055),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="dark_rubber",
            name=f"sleeve_{i}",
        )
    for i, x in enumerate((-0.094, 0.094)):
        handle.visual(
            Box((0.026, 0.150, 0.022)),
            origin=Origin(xyz=(x, -0.074, 0.014)),
            material="dark_rubber",
            name=f"arm_{i}",
        )
    handle.visual(
        Box((0.250, 0.034, 0.028)),
        origin=Origin(xyz=(0.0, -0.150, 0.015)),
        material="dark_rubber",
        name="grip",
    )
    handle.visual(
        Box((0.185, 0.036, 0.006)),
        origin=Origin(xyz=(0.0, -0.151, 0.032)),
        material="button_gray",
        name="grip_pad",
    )
    model.articulation(
        "enclosure_to_handle",
        ArticulationType.REVOLUTE,
        parent=enclosure,
        child=handle,
        origin=Origin(xyz=(0.0, 0.058, HEIGHT + 0.014)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.35, effort=10.0, velocity=2.0),
    )

    knob = model.part("volume_knob")
    knob.visual(
        Cylinder(radius=0.032, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material="knob_silver",
        name="cap",
    )
    knob.visual(
        Box((0.005, 0.026, 0.003)),
        origin=Origin(xyz=(0.0, 0.009, 0.0315)),
        material="molded_black",
        name="indicator",
    )
    model.articulation(
        "enclosure_to_volume_knob",
        ArticulationType.CONTINUOUS,
        parent=enclosure,
        child=knob,
        origin=Origin(xyz=(0.0, FRONT_Y - 0.006, 0.618), rpy=(math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=6.0),
    )

    button_xs = (-0.074, 0.0, 0.074)
    for i, x in enumerate(button_xs):
        button = model.part(f"button_{i}")
        button.visual(
            mesh_from_geometry(_rounded_button_shape(0.048, 0.022, 0.012), f"button_cap_{i}"),
            origin=Origin(xyz=(0.0, 0.0, 0.006)),
            material="button_gray",
            name="cap",
        )
        model.articulation(
            f"enclosure_to_button_{i}",
            ArticulationType.PRISMATIC,
            parent=enclosure,
            child=button,
            origin=Origin(xyz=(x, FRONT_Y - 0.006, 0.532), rpy=(math.pi / 2.0, 0.0, 0.0)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(lower=0.0, upper=0.008, effort=3.0, velocity=0.06),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    enclosure = object_model.get_part("enclosure")
    handle = object_model.get_part("handle")
    knob = object_model.get_part("volume_knob")
    handle_joint = object_model.get_articulation("enclosure_to_handle")
    knob_joint = object_model.get_articulation("enclosure_to_volume_knob")

    for i in range(2):
        ctx.allow_overlap(
            enclosure,
            handle,
            elem_a=f"hinge_socket_{i}",
            elem_b=f"sleeve_{i}",
            reason="The handle sleeve is intentionally captured inside the fixed hinge socket proxy.",
        )
        ctx.expect_overlap(
            enclosure,
            handle,
            axes="xyz",
            elem_a=f"hinge_socket_{i}",
            elem_b=f"sleeve_{i}",
            min_overlap=0.004,
            name=f"handle hinge {i} remains captured",
        )

    ctx.expect_gap(
        enclosure,
        knob,
        axis="y",
        positive_elem="control_face",
        negative_elem="cap",
        max_gap=0.001,
        max_penetration=0.001,
        name="volume knob seats on control face",
    )
    ctx.check(
        "volume knob is continuous",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_joint.articulation_type}",
    )

    rest_handle_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({handle_joint: 1.20}):
        raised_handle_aabb = ctx.part_world_aabb(handle)
    ctx.check(
        "handle rotates upward",
        rest_handle_aabb is not None
        and raised_handle_aabb is not None
        and raised_handle_aabb[1][2] > rest_handle_aabb[1][2] + 0.070,
        details=f"rest={rest_handle_aabb}, raised={raised_handle_aabb}",
    )

    grille_aabb = ctx.part_element_world_aabb(enclosure, elem="front_grille")
    for i in range(3):
        button = object_model.get_part(f"button_{i}")
        button_joint = object_model.get_articulation(f"enclosure_to_button_{i}")
        ctx.expect_gap(
            enclosure,
            button,
            axis="y",
            positive_elem="control_face",
            negative_elem="cap",
            max_gap=0.001,
            max_penetration=0.001,
            name=f"button {i} is a separate face-mounted cap",
        )
        rest_pos = ctx.part_world_position(button)
        button_aabb = ctx.part_world_aabb(button)
        with ctx.pose({button_joint: 0.008}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button {i} depresses inward",
            rest_pos is not None and pressed_pos is not None and pressed_pos[1] > rest_pos[1] + 0.006,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )
        ctx.check(
            f"button {i} sits above grille",
            grille_aabb is not None and button_aabb is not None and button_aabb[0][2] > grille_aabb[1][2] + 0.010,
            details=f"button={button_aabb}, grille={grille_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
