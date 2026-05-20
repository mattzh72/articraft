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
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


GLASS_WIDTH = 0.90
GLASS_DEPTH = 0.56
GLASS_THICKNESS = 0.008
PAN_WIDTH = 0.82
PAN_DEPTH = 0.46
PAN_HEIGHT = 0.028
GLASS_TOP_Z = PAN_HEIGHT + GLASS_THICKNESS

POD_WIDTH = 0.46
POD_DEPTH = 0.115
POD_HEIGHT = 0.042
POD_CENTER_Y = -(GLASS_DEPTH * 0.5) + (POD_DEPTH * 0.5) - 0.004
POD_TOP_Z = GLASS_TOP_Z + POD_HEIGHT

HINGE_AXIS_Y = POD_CENTER_Y + (POD_DEPTH * 0.5) - 0.006
HINGE_AXIS_Z = POD_TOP_Z + 0.018

COVER_WIDTH = 0.44
COVER_DEPTH = 0.092
COVER_INTERNAL_HEIGHT = 0.021
COVER_TOP_THICKNESS = 0.003
COVER_SIDE_THICKNESS = 0.004

ZONE_THICKNESS = 0.0006

DIAL_CLEARANCE = 0.001
DIAL_HEIGHT = 0.013
DIAL_SHAFT_LENGTH = 0.004
BUTTON_TRAVEL = 0.0018
BUTTON_HEIGHT = 0.004


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="induction_stovetop")

    glass_black = model.material("glass_black", rgba=(0.05, 0.05, 0.06, 1.0))
    glass_mark = model.material("glass_mark", rgba=(0.28, 0.29, 0.31, 0.55))
    pan_graphite = model.material("pan_graphite", rgba=(0.22, 0.23, 0.24, 1.0))
    pod_satin = model.material("pod_satin", rgba=(0.66, 0.67, 0.70, 1.0))
    cover_smoke = model.material("cover_smoke", rgba=(0.16, 0.18, 0.20, 0.42))
    hinge_dark = model.material("hinge_dark", rgba=(0.19, 0.20, 0.21, 1.0))

    body = model.part("body")
    body.visual(
        Box((PAN_WIDTH, PAN_DEPTH, PAN_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, PAN_HEIGHT * 0.5)),
        material=pan_graphite,
        name="underside_pan",
    )
    body.visual(
        Box((GLASS_WIDTH, GLASS_DEPTH, GLASS_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, PAN_HEIGHT + GLASS_THICKNESS * 0.5)),
        material=glass_black,
        name="glass_top",
    )

    for x_pos, y_pos, diameter in (
        (-0.21, 0.12, 0.215),
        (0.21, 0.12, 0.215),
        (-0.19, -0.10, 0.185),
        (0.19, -0.10, 0.185),
        (0.0, 0.01, 0.245),
    ):
        body.visual(
            Cylinder(radius=diameter * 0.5, length=ZONE_THICKNESS),
            origin=Origin(xyz=(x_pos, y_pos, GLASS_TOP_Z - (ZONE_THICKNESS * 0.5))),
            material=glass_mark,
            name=f"zone_{int((x_pos + 0.45) * 1000):03d}_{int((y_pos + 0.30) * 1000):03d}",
        )

    body.visual(
        Box((POD_WIDTH, POD_DEPTH, POD_HEIGHT)),
        origin=Origin(xyz=(0.0, POD_CENTER_Y, GLASS_TOP_Z + POD_HEIGHT * 0.5)),
        material=pod_satin,
        name="pod_shell",
    )
    body.visual(
        Box((POD_WIDTH * 0.93, POD_DEPTH * 0.84, 0.006)),
        origin=Origin(xyz=(0.0, POD_CENTER_Y, POD_TOP_Z - 0.003)),
        material=glass_black,
        name="pod_roof",
    )
    body.visual(
        Box((POD_WIDTH * 1.02, 0.016, 0.018)),
        origin=Origin(
            xyz=(0.0, POD_CENTER_Y - (POD_DEPTH * 0.5) + 0.008, GLASS_TOP_Z + 0.010)
        ),
        material=pod_satin,
        name="pod_front_trim",
    )
    body.visual(
        Box((COVER_WIDTH * 0.86, 0.014, 0.014)),
        origin=Origin(xyz=(0.0, HINGE_AXIS_Y - 0.006, HINGE_AXIS_Z - 0.011)),
        material=hinge_dark,
        name="hinge_spine",
    )

    for x_pos in (-0.17, 0.0, 0.17):
        body.visual(
            Cylinder(radius=0.004, length=0.050),
            origin=Origin(
                xyz=(x_pos, HINGE_AXIS_Y, HINGE_AXIS_Z),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=hinge_dark,
            name=f"body_hinge_{'mid' if abs(x_pos) < 1e-6 else ('outer' if x_pos > 0 else 'inner')}_{int(abs(x_pos) * 1000):03d}",
        )

    cover = model.part("cover")
    cover.visual(
        Box((COVER_WIDTH, COVER_DEPTH - 0.008, COVER_TOP_THICKNESS)),
        origin=Origin(
            xyz=(0.0, -(COVER_DEPTH * 0.5) + 0.004, 0.0065)
        ),
        material=cover_smoke,
        name="top_plate",
    )
    cover.visual(
        Box((COVER_WIDTH, 0.008, COVER_INTERNAL_HEIGHT + COVER_TOP_THICKNESS)),
        origin=Origin(
            xyz=(0.0, -COVER_DEPTH + 0.004, -0.006)
        ),
        material=cover_smoke,
        name="front_lip",
    )
    for side_x, side_name in ((-(COVER_WIDTH * 0.5) + 0.002, "side_lip_0"), ((COVER_WIDTH * 0.5) - 0.002, "side_lip_1")):
        cover.visual(
            Box((COVER_SIDE_THICKNESS, COVER_DEPTH - 0.008, COVER_INTERNAL_HEIGHT + COVER_TOP_THICKNESS)),
            origin=Origin(
                xyz=(
                    side_x,
                    -(COVER_DEPTH * 0.5) + 0.004,
                    -0.006,
                )
            ),
            material=cover_smoke,
            name=side_name,
        )
    for x_pos, hinge_name in ((-0.085, "cover_hinge_0"), (0.085, "cover_hinge_1")):
        cover.visual(
            Cylinder(radius=0.004, length=0.090),
            origin=Origin(
                xyz=(x_pos, 0.0, 0.0),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=hinge_dark,
            name=hinge_name,
        )
        cover.visual(
            Box((0.100, 0.010, 0.008)),
            origin=Origin(xyz=(x_pos, -0.005, 0.003)),
            material=hinge_dark,
            name=f"hinge_tab_{hinge_name[-1]}",
        )

    model.articulation(
        "body_to_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(0.0, HINGE_AXIS_Y, HINGE_AXIS_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=1.35),
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.006, length=DIAL_SHAFT_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, DIAL_SHAFT_LENGTH * 0.5)),
        material=hinge_dark,
        name="shaft",
    )
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.054,
                DIAL_HEIGHT,
                body_style="skirted",
                top_diameter=0.044,
                skirt=KnobSkirt(0.062, 0.0035, flare=0.05),
                grip=KnobGrip(style="fluted", count=24, depth=0.0011),
                indicator=KnobIndicator(
                    style="line",
                    mode="engraved",
                    depth=0.0006,
                    angle_deg=0.0,
                ),
                center=False,
            ),
            "stove_dial_shell",
        ),
        origin=Origin(xyz=(0.0, 0.0, DIAL_CLEARANCE + DIAL_SHAFT_LENGTH - 0.001)),
        material=glass_black,
        name="dial_shell",
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, POD_CENTER_Y, POD_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=5.0),
    )

    button_x_positions = (-0.165, -0.113, -0.061, 0.061, 0.113, 0.165)
    for index, x_pos in enumerate(button_x_positions):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.036, 0.017, BUTTON_HEIGHT)),
            origin=Origin(xyz=(0.0, 0.0, BUTTON_HEIGHT * 0.5)),
            material=glass_black,
            name="button_cap",
        )
        button.visual(
            Box((0.022, 0.009, 0.001)),
            origin=Origin(xyz=(0.0, 0.0, BUTTON_HEIGHT + 0.0005)),
            material=cover_smoke,
            name="button_label",
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, POD_CENTER_Y, POD_TOP_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=10.0,
                velocity=0.06,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    cover = object_model.get_part("cover")
    cover_hinge = object_model.get_articulation("body_to_cover")
    dial = object_model.get_part("dial")
    dial_joint = object_model.get_articulation("body_to_dial")
    buttons = [object_model.get_part(f"button_{index}") for index in range(6)]
    button_joints = [object_model.get_articulation(f"body_to_button_{index}") for index in range(6)]

    ctx.expect_overlap(
        cover,
        body,
        axes="xy",
        elem_a="top_plate",
        elem_b="pod_roof",
        min_overlap=0.075,
        name="cover spans the pod roof footprint",
    )

    closed_cover_aabb = ctx.part_world_aabb(cover)
    with ctx.pose({cover_hinge: 1.1}):
        opened_cover_aabb = ctx.part_world_aabb(cover)

    ctx.check(
        "cover lifts upward",
        closed_cover_aabb is not None
        and opened_cover_aabb is not None
        and float(opened_cover_aabb[1][2]) > float(closed_cover_aabb[1][2]) + 0.06,
        details=f"closed={closed_cover_aabb}, opened={opened_cover_aabb}",
    )

    ctx.expect_gap(
        cover,
        dial,
        axis="z",
        positive_elem="top_plate",
        negative_elem="dial_shell",
        min_gap=0.002,
        max_gap=0.010,
        name="cover clears the central dial when closed",
    )
    ctx.expect_within(
        dial,
        cover,
        axes="xy",
        inner_elem="dial_shell",
        outer_elem="top_plate",
        margin=0.0,
        name="dial sits under the control cover footprint",
    )
    ctx.expect_gap(
        dial,
        body,
        axis="z",
        positive_elem="shaft",
        negative_elem="pod_roof",
        max_penetration=0.0,
        max_gap=0.0005,
        name="dial shaft seats onto the pod roof",
    )

    dial_rest = ctx.part_world_position(dial)
    with ctx.pose({dial_joint: 1.7}):
        dial_spun = ctx.part_world_position(dial)
    ctx.check(
        "dial rotates in place",
        dial_rest is not None
        and dial_spun is not None
        and abs(float(dial_rest[0]) - float(dial_spun[0])) < 1e-6
        and abs(float(dial_rest[1]) - float(dial_spun[1])) < 1e-6
        and abs(float(dial_rest[2]) - float(dial_spun[2])) < 1e-6,
        details=f"rest={dial_rest}, spun={dial_spun}",
    )

    for index, (button, button_joint) in enumerate(zip(buttons, button_joints)):
        ctx.expect_within(
            button,
            cover,
            axes="xy",
            inner_elem="button_cap",
            outer_elem="top_plate",
            margin=0.0,
            name=f"button_{index} stays under the cover footprint",
        )
        ctx.expect_gap(
            button,
            body,
            axis="z",
            positive_elem="button_cap",
            negative_elem="pod_roof",
            max_penetration=0.0,
            max_gap=0.0002,
            name=f"button_{index} is seated on the pod roof",
        )

        rest_pos = ctx.part_world_position(button)
        with ctx.pose({button_joint: BUTTON_TRAVEL}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button_{index} depresses downward",
            rest_pos is not None
            and pressed_pos is not None
            and float(pressed_pos[2]) < float(rest_pos[2]) - 0.001,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
