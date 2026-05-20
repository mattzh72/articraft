from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    VentGrilleGeometry,
    VentGrilleSlats,
    VentGrilleSleeve,
    mesh_from_geometry,
)


BODY_WIDTH = 0.420
BODY_DEPTH = 0.330
BODY_BOTTOM = 0.048
LOWER_BODY_HEIGHT = 0.512
UPPER_BODY_WIDTH = 0.392
UPPER_BODY_DEPTH = 0.304
UPPER_BODY_HEIGHT = 0.120
TOP_WIDTH = 0.400
TOP_DEPTH = 0.310
TOP_RING_HEIGHT = 0.048
TOP_Z0 = BODY_BOTTOM + LOWER_BODY_HEIGHT + UPPER_BODY_HEIGHT
TOP_Z1 = TOP_Z0 + TOP_RING_HEIGHT

OUTLET_WIDTH = 0.260
OUTLET_DEPTH = 0.110
OUTLET_CENTER_Y = 0.020
OUTLET_FRONT_Y = OUTLET_CENTER_Y + OUTLET_DEPTH * 0.5

CONTROL_PANEL_WIDTH = 0.240
CONTROL_PANEL_HEIGHT = 0.120
CONTROL_PANEL_DEPTH = 0.020
CONTROL_PANEL_CENTER_Z = 0.615
CONTROL_PANEL_FRONT_Y = UPPER_BODY_DEPTH * 0.5 + CONTROL_PANEL_DEPTH


def _add_caster(body, *, x: float, y: float, wheel_material: str, fork_material: str) -> None:
    body.visual(
        Cylinder(radius=0.005, length=0.014),
        origin=Origin(xyz=(x, y, 0.041)),
        material=fork_material,
        name=f"stem_{'p' if x >= 0 else 'n'}{'p' if y >= 0 else 'n'}",
    )
    body.visual(
        Box((0.018, 0.012, 0.010)),
        origin=Origin(xyz=(x, y, 0.029)),
        material=fork_material,
        name=f"fork_{'p' if x >= 0 else 'n'}{'p' if y >= 0 else 'n'}",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(
            xyz=(x, y, 0.018),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=wheel_material,
        name=f"wheel_{'p' if x >= 0 else 'n'}{'p' if y >= 0 else 'n'}",
    )


def _build_button_part(model: ArticulatedObject, name: str, material: str) -> None:
    button = model.part(name)
    button.visual(
        Box((0.026, 0.008, 0.014)),
        origin=Origin(xyz=(0.0, 0.004, 0.007)),
        material=material,
        name="base",
    )
    button.visual(
        Box((0.022, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, 0.011, 0.007)),
        material=material,
        name="cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_room_ac")

    shell = model.material("shell", rgba=(0.90, 0.91, 0.92, 1.0))
    trim = model.material("trim", rgba=(0.79, 0.81, 0.83, 1.0))
    panel = model.material("panel", rgba=(0.20, 0.24, 0.28, 1.0))
    vent = model.material("vent", rgba=(0.18, 0.20, 0.22, 1.0))
    wheel = model.material("wheel", rgba=(0.12, 0.12, 0.13, 1.0))
    fork = model.material("fork", rgba=(0.55, 0.57, 0.60, 1.0))
    control = model.material("control", rgba=(0.83, 0.85, 0.87, 1.0))
    dial_finish = model.material("dial_finish", rgba=(0.16, 0.17, 0.18, 1.0))
    indicator = model.material("indicator", rgba=(0.90, 0.92, 0.95, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_WIDTH, BODY_DEPTH, LOWER_BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BODY_BOTTOM + LOWER_BODY_HEIGHT * 0.5)),
        material=shell,
        name="lower_shell",
    )
    body.visual(
        Box((UPPER_BODY_WIDTH, UPPER_BODY_DEPTH, UPPER_BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BODY_BOTTOM + LOWER_BODY_HEIGHT + UPPER_BODY_HEIGHT * 0.5)),
        material=shell,
        name="upper_shell",
    )
    body.visual(
        Box((TOP_WIDTH, (TOP_DEPTH - OUTLET_DEPTH) * 0.5, TOP_RING_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                (OUTLET_FRONT_Y + TOP_DEPTH * 0.5) * 0.5,
                TOP_Z0 + TOP_RING_HEIGHT * 0.5,
            )
        ),
        material=trim,
        name="front_top",
    )
    body.visual(
        Box((TOP_WIDTH, 0.120, TOP_RING_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.095, TOP_Z0 + TOP_RING_HEIGHT * 0.5)),
        material=trim,
        name="rear_top",
    )
    body.visual(
        Box(((TOP_WIDTH - OUTLET_WIDTH) * 0.5, TOP_DEPTH, TOP_RING_HEIGHT)),
        origin=Origin(xyz=(-0.165, 0.0, TOP_Z0 + TOP_RING_HEIGHT * 0.5)),
        material=trim,
        name="left_top",
    )
    body.visual(
        Box(((TOP_WIDTH - OUTLET_WIDTH) * 0.5, TOP_DEPTH, TOP_RING_HEIGHT)),
        origin=Origin(xyz=(0.165, 0.0, TOP_Z0 + TOP_RING_HEIGHT * 0.5)),
        material=trim,
        name="right_top",
    )
    body.visual(
        mesh_from_geometry(
            VentGrilleGeometry(
                (OUTLET_WIDTH, OUTLET_DEPTH),
                frame=0.010,
                face_thickness=0.004,
                duct_depth=0.018,
                slat_pitch=0.018,
                slat_width=0.008,
                slat_angle_deg=28.0,
                slats=VentGrilleSlats(profile="airfoil", direction="down"),
                sleeve=VentGrilleSleeve(style="none"),
            ),
            "portable_ac_top_grille",
        ),
        origin=Origin(xyz=(0.0, OUTLET_CENTER_Y, TOP_Z0 + 0.024)),
        material=vent,
        name="grille",
    )
    body.visual(
        Box((CONTROL_PANEL_WIDTH, CONTROL_PANEL_DEPTH, CONTROL_PANEL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                UPPER_BODY_DEPTH * 0.5 + CONTROL_PANEL_DEPTH * 0.5,
                CONTROL_PANEL_CENTER_Z,
            )
        ),
        material=panel,
        name="panel",
    )
    body.visual(
        Box((0.080, 0.006, 0.020)),
        origin=Origin(
            xyz=(
                0.0,
                CONTROL_PANEL_FRONT_Y - 0.003,
                CONTROL_PANEL_CENTER_Z + 0.042,
            )
        ),
        material=trim,
        name="display_strip",
    )

    for x_sign in (-1.0, 1.0):
        body.visual(
            Box((0.012, 0.045, 0.090)),
            origin=Origin(
                xyz=(
                    x_sign * 0.176,
                    0.0,
                    BODY_BOTTOM + LOWER_BODY_HEIGHT - 0.010,
                )
            ),
            material=trim,
            name=f"side_cap_{0 if x_sign < 0 else 1}",
        )

    _add_caster(body, x=-0.155, y=-0.112, wheel_material=wheel, fork_material=fork)
    _add_caster(body, x=0.155, y=-0.112, wheel_material=wheel, fork_material=fork)
    _add_caster(body, x=-0.155, y=0.112, wheel_material=wheel, fork_material=fork)
    _add_caster(body, x=0.155, y=0.112, wheel_material=wheel, fork_material=fork)

    flap = model.part("flap")
    flap.visual(
        Box((0.244, 0.106, 0.005)),
        origin=Origin(xyz=(0.0, -0.053, 0.0025)),
        material=trim,
        name="panel",
    )
    flap.visual(
        Box((0.244, 0.006, 0.014)),
        origin=Origin(xyz=(0.0, -0.003, 0.007)),
        material=trim,
        name="nose",
    )
    flap.visual(
        Box((0.244, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, -0.102, 0.006)),
        material=trim,
        name="rear_lip",
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.025, length=0.008),
        origin=Origin(
            xyz=(0.0, 0.004, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=panel,
        name="skirt",
    )
    dial.visual(
        Cylinder(radius=0.020, length=0.020),
        origin=Origin(
            xyz=(0.0, 0.014, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=dial_finish,
        name="knob",
    )
    dial.visual(
        Box((0.003, 0.003, 0.014)),
        origin=Origin(xyz=(0.0, 0.0245, 0.011)),
        material=indicator,
        name="indicator",
    )

    button_positions = (-0.055, 0.0, 0.055)
    for index in range(3):
        _build_button_part(model, f"button_{index}", control)

    model.articulation(
        "body_to_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flap,
        origin=Origin(xyz=(0.0, OUTLET_FRONT_Y, TOP_Z1)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.5,
            lower=0.0,
            upper=1.20,
        ),
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(
            xyz=(
                0.0,
                CONTROL_PANEL_FRONT_Y,
                CONTROL_PANEL_CENTER_Z - 0.008,
            )
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=6.0),
    )

    for index, button_x in enumerate(button_positions):
        button = model.get_part(f"button_{index}")
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(
                xyz=(
                    button_x,
                    CONTROL_PANEL_FRONT_Y,
                    CONTROL_PANEL_CENTER_Z + 0.036,
                )
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=5.0,
                velocity=0.08,
                lower=0.0,
                upper=0.004,
            ),
        )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((float(mins[i]) + float(maxs[i])) * 0.5 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    flap = object_model.get_part("flap")
    dial = object_model.get_part("dial")
    flap_joint = object_model.get_articulation("body_to_flap")
    dial_joint = object_model.get_articulation("body_to_dial")

    ctx.expect_overlap(
        flap,
        body,
        axes="xy",
        elem_a="panel",
        elem_b="grille",
        min_overlap=0.09,
        name="flap covers the discharge opening at rest",
    )
    ctx.expect_gap(
        flap,
        body,
        axis="z",
        positive_elem="panel",
        negative_elem="grille",
        min_gap=0.008,
        max_gap=0.028,
        name="closed flap sits just above the top grille",
    )
    ctx.expect_overlap(
        dial,
        body,
        axes="xz",
        elem_a="knob",
        elem_b="panel",
        min_overlap=0.035,
        name="dial stays centered within the control panel footprint",
    )
    ctx.expect_gap(
        dial,
        body,
        axis="y",
        positive_elem="knob",
        negative_elem="panel",
        min_gap=0.0,
        max_gap=0.020,
        name="dial sits proud of the front control panel",
    )

    rest_flap_aabb = ctx.part_element_world_aabb(flap, elem="panel")
    with ctx.pose({flap_joint: 1.05}):
        open_flap_aabb = ctx.part_element_world_aabb(flap, elem="panel")
    ctx.check(
        "flap opens upward",
        rest_flap_aabb is not None
        and open_flap_aabb is not None
        and float(open_flap_aabb[1][2]) > float(rest_flap_aabb[1][2]) + 0.060,
        details=f"rest={rest_flap_aabb}, open={open_flap_aabb}",
    )

    rest_indicator = _aabb_center(ctx.part_element_world_aabb(dial, elem="indicator"))
    with ctx.pose({dial_joint: math.pi / 2.0}):
        turned_indicator = _aabb_center(ctx.part_element_world_aabb(dial, elem="indicator"))
    ctx.check(
        "dial indicator sweeps around the knob axis",
        rest_indicator is not None
        and turned_indicator is not None
        and abs(float(turned_indicator[0]) - float(rest_indicator[0])) > 0.008
        and abs(float(turned_indicator[2]) - float(rest_indicator[2])) > 0.008,
        details=f"rest={rest_indicator}, turned={turned_indicator}",
    )

    for index in range(3):
        button = object_model.get_part(f"button_{index}")
        button_joint = object_model.get_articulation(f"body_to_button_{index}")
        ctx.expect_overlap(
            button,
            body,
            axes="xz",
            elem_a="cap",
            elem_b="panel",
            min_overlap=0.009,
            name=f"button_{index} stays within the panel footprint",
        )
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({button_joint: 0.004}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button_{index} depresses inward",
            rest_pos is not None
            and pressed_pos is not None
            and float(pressed_pos[1]) < float(rest_pos[1]) - 0.003,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
