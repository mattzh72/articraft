from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    place_on_face,
    rounded_rect_profile,
)

COOKTOP_WIDTH = 0.91
COOKTOP_DEPTH = 0.52
GLASS_THICKNESS = 0.006
HOUSING_WIDTH = 0.85
HOUSING_DEPTH = 0.45
HOUSING_HEIGHT = 0.048
FRONT_TRIM_DEPTH = 0.018
FRONT_TRIM_HEIGHT = 0.032
BUTTON_WIDTH = 0.046
BUTTON_HEIGHT = 0.014
BUTTON_THICKNESS = 0.004
BUTTON_TRAVEL = 0.003
BUTTON_PROUD = 0.0005
BUTTON_SLOT_WIDTH = BUTTON_WIDTH
BUTTON_SLOT_HEIGHT = BUTTON_HEIGHT
BUTTON_CENTER_Z = -0.021
BUTTON_XS = (-0.090, -0.030, 0.030, 0.090)
KNOB_X = 0.332
KNOB_Y = -0.188
KNOB_SHAFT_HEIGHT = 0.008


def _build_glass_deck() -> object:
    deck = (
        cq.Workplane("XY")
        .box(
            COOKTOP_WIDTH,
            COOKTOP_DEPTH,
            GLASS_THICKNESS,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, -GLASS_THICKNESS))
    )

    for center_x, center_y, radius in (
        (-0.175, 0.105, 0.094),
        (0.180, 0.112, 0.086),
        (-0.188, -0.102, 0.092),
        (0.172, -0.092, 0.084),
    ):
        deck = (
            deck.faces(">Z")
            .workplane()
            .center(center_x, center_y)
            .circle(radius)
            .circle(radius - 0.0035)
            .cutBlind(-0.0007)
        )

    return deck


def _build_front_trim() -> object:
    trim_center_y = -COOKTOP_DEPTH / 2.0 + FRONT_TRIM_DEPTH / 2.0
    trim = (
        cq.Workplane("XY")
        .box(
            COOKTOP_WIDTH,
            FRONT_TRIM_DEPTH,
            FRONT_TRIM_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, trim_center_y, -GLASS_THICKNESS - FRONT_TRIM_HEIGHT))
    )

    slot_center_y = trim_center_y
    slot_center_z = BUTTON_CENTER_Z
    for button_x in BUTTON_XS:
        slot = (
            cq.Workplane("XY")
            .box(
                BUTTON_SLOT_WIDTH,
                FRONT_TRIM_DEPTH + 0.004,
                BUTTON_SLOT_HEIGHT,
                centered=(True, True, True),
            )
            .translate((button_x, slot_center_y, slot_center_z))
        )
        trim = trim.cut(slot)

    return trim


def _build_button_mesh(name: str):
    button_geom = ExtrudeGeometry.from_z0(
        rounded_rect_profile(BUTTON_WIDTH, BUTTON_HEIGHT, 0.0035),
        BUTTON_THICKNESS,
    )
    button_geom.translate(0.0, 0.0, -BUTTON_THICKNESS)
    return mesh_from_geometry(button_geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="induction_stovetop")

    glass_black = model.material("glass_black", rgba=(0.08, 0.09, 0.10, 1.0))
    chassis_black = model.material("chassis_black", rgba=(0.15, 0.16, 0.17, 1.0))
    trim_steel = model.material("trim_steel", rgba=(0.66, 0.67, 0.69, 1.0))
    knob_black = model.material("knob_black", rgba=(0.14, 0.14, 0.15, 1.0))
    shaft_steel = model.material("shaft_steel", rgba=(0.73, 0.74, 0.76, 1.0))
    button_black = model.material("button_black", rgba=(0.11, 0.12, 0.13, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_glass_deck(), "glass_deck"),
        material=glass_black,
        name="glass_deck",
    )
    body.visual(
        Box((HOUSING_WIDTH, HOUSING_DEPTH, HOUSING_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -0.017,
                -GLASS_THICKNESS - HOUSING_HEIGHT / 2.0,
            )
        ),
        material=chassis_black,
        name="housing",
    )
    body.visual(
        mesh_from_cadquery(_build_front_trim(), "front_trim"),
        material=trim_steel,
        name="front_trim",
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.009, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=shaft_steel,
        name="knob_bushing",
    )
    knob.visual(
        Cylinder(radius=0.004, length=KNOB_SHAFT_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, KNOB_SHAFT_HEIGHT / 2.0)),
        material=shaft_steel,
        name="knob_shaft",
    )
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.046,
                0.021,
                body_style="skirted",
                center=False,
                top_diameter=0.038,
                skirt=KnobSkirt(0.056, 0.004, flare=0.05),
                grip=KnobGrip(style="fluted", count=16, depth=0.0012),
                indicator=KnobIndicator(
                    style="line",
                    mode="engraved",
                    depth=0.0007,
                    angle_deg=0.0,
                ),
            ),
            "selector_knob",
        ),
        origin=Origin(xyz=(0.0, 0.0, KNOB_SHAFT_HEIGHT)),
        material=knob_black,
        name="knob_shell",
    )
    model.articulation(
        "body_to_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=place_on_face(
            body,
            "+z",
            face_pos=(KNOB_X, KNOB_Y),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.35,
            velocity=6.0,
        ),
    )

    button_mesh = _build_button_mesh("touch_button")
    for button_index, button_x in enumerate(BUTTON_XS):
        button = model.part(f"button_{button_index}")
        button.visual(
            button_mesh,
            material=button_black,
            name="button_cap",
        )
        model.articulation(
            f"body_to_button_{button_index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=place_on_face(
                body,
                "-y",
                face_pos=(button_x, BUTTON_CENTER_Z),
                proud=BUTTON_PROUD,
            ),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=0.08,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    knob = object_model.get_part("knob")
    knob_joint = object_model.get_articulation("body_to_knob")

    ctx.expect_overlap(
        knob,
        body,
        axes="xy",
        min_overlap=0.030,
        name="knob footprint sits on the glass deck",
    )
    ctx.expect_gap(
        knob,
        body,
        axis="z",
        min_gap=0.0,
        max_gap=0.010,
        name="knob stack rises from the deck without floating",
    )

    knob_pos = ctx.part_world_position(knob)
    ctx.check(
        "knob sits in the right front corner region",
        knob_pos is not None and knob_pos[0] > 0.28 and knob_pos[1] < -0.14,
        details=f"knob_pos={knob_pos}",
    )
    ctx.check(
        "selector knob uses continuous rotation",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and knob_joint.motion_limits is not None
        and knob_joint.motion_limits.lower is None
        and knob_joint.motion_limits.upper is None,
        details=f"joint_type={knob_joint.articulation_type}, limits={knob_joint.motion_limits}",
    )

    button_parts = [object_model.get_part(f"button_{index}") for index in range(4)]
    button_joints = [object_model.get_articulation(f"body_to_button_{index}") for index in range(4)]
    rest_positions = [ctx.part_world_position(button) for button in button_parts]

    for index, (button, button_joint, rest_pos) in enumerate(zip(button_parts, button_joints, rest_positions)):
        ctx.expect_overlap(
            button,
            body,
            axes="xz",
            min_overlap=0.012,
            name=f"button_{index} stays within the front control band",
        )
        ctx.check(
            f"button_{index} is centered on the front trim",
            rest_pos is not None and abs(rest_pos[0]) < 0.12 and rest_pos[1] < -0.255 and -0.030 < rest_pos[2] < -0.010,
            details=f"rest_pos={rest_pos}",
        )

        upper = button_joint.motion_limits.upper if button_joint.motion_limits is not None else None
        with ctx.pose({button_joint: upper if upper is not None else 0.0}):
            moved_pos = ctx.part_world_position(button)
            other_positions = [
                ctx.part_world_position(other_button)
                for other_button in button_parts
            ]

        moved_ok = (
            rest_pos is not None
            and moved_pos is not None
            and moved_pos[1] > rest_pos[1] + 0.0024
        )
        others_ok = True
        if rest_pos is not None:
            for other_index, (other_rest, other_pos) in enumerate(zip(rest_positions, other_positions)):
                if other_index == index:
                    continue
                if other_rest is None or other_pos is None or abs(other_pos[1] - other_rest[1]) > 1e-6:
                    others_ok = False
                    break

        ctx.check(
            f"button_{index} depresses inward",
            moved_ok,
            details=f"rest_pos={rest_pos}, moved_pos={moved_pos}",
        )
        ctx.check(
            f"button_{index} moves independently",
            others_ok,
            details=f"rest_positions={rest_positions}, posed_positions={other_positions}",
        )

    return ctx.report()


object_model = build_object_model()
