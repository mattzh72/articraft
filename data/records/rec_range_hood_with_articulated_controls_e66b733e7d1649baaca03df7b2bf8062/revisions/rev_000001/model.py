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
    MeshGeometry,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


FRONT_PANEL_Y = -0.602


def _quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _add_loop(geom: MeshGeometry, points: list[tuple[float, float, float]]) -> list[int]:
    return [geom.add_vertex(x, y, z) for x, y, z in points]


def _rect_loop(x_half: float, y_front: float, y_back: float, z: float) -> list[tuple[float, float, float]]:
    return [
        (-x_half, y_front, z),
        (x_half, y_front, z),
        (x_half, y_back, z),
        (-x_half, y_back, z),
    ]


def _connect_loops(geom: MeshGeometry, loop_a: list[int], loop_b: list[int], *, reverse: bool = False) -> None:
    count = len(loop_a)
    for i in range(count):
        j = (i + 1) % count
        if reverse:
            _quad(geom, loop_a[i], loop_b[i], loop_b[j], loop_a[j])
        else:
            _quad(geom, loop_a[i], loop_a[j], loop_b[j], loop_b[i])


def _canopy_shell_geometry() -> MeshGeometry:
    """Thin hollow trapezoid canopy with a real lower intake opening."""
    geom = MeshGeometry()

    outer_bottom = _add_loop(geom, _rect_loop(0.48, -0.58, 0.06, 0.000))
    outer_top = _add_loop(geom, _rect_loop(0.19, -0.20, 0.055, 0.225))
    inner_bottom = _add_loop(geom, _rect_loop(0.425, -0.515, 0.020, 0.016))
    inner_top = _add_loop(geom, _rect_loop(0.145, -0.165, 0.028, 0.198))

    _connect_loops(geom, outer_bottom, outer_top)
    _connect_loops(geom, inner_top, inner_bottom, reverse=True)
    _connect_loops(geom, outer_bottom, inner_bottom, reverse=True)
    _connect_loops(geom, inner_top, outer_top)
    return geom


def _chimney_shell_geometry() -> MeshGeometry:
    """Short rectangular upper chimney cover, modeled as an open metal sleeve."""
    geom = MeshGeometry()

    outer_bottom = _add_loop(geom, _rect_loop(0.155, -0.170, 0.060, 0.205))
    outer_top = _add_loop(geom, _rect_loop(0.155, -0.170, 0.060, 0.505))
    inner_bottom = _add_loop(geom, _rect_loop(0.128, -0.143, 0.033, 0.225))
    inner_top = _add_loop(geom, _rect_loop(0.128, -0.143, 0.033, 0.485))

    _connect_loops(geom, outer_bottom, outer_top)
    _connect_loops(geom, inner_top, inner_bottom, reverse=True)
    _connect_loops(geom, outer_bottom, inner_bottom, reverse=True)
    _connect_loops(geom, inner_top, outer_top)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood")

    stainless = model.material("brushed_stainless", rgba=(0.70, 0.70, 0.66, 1.0))
    dark_filter = model.material("dark_filter_mesh", rgba=(0.16, 0.17, 0.17, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.015, 0.015, 0.014, 1.0))
    button_plastic = model.material("button_plastic", rgba=(0.06, 0.065, 0.07, 1.0))

    hood = model.part("hood")
    hood.visual(
        mesh_from_geometry(_canopy_shell_geometry(), "deep_canopy_shell"),
        material=stainless,
        name="canopy_shell",
    )
    hood.visual(
        mesh_from_geometry(_chimney_shell_geometry(), "short_chimney_shell"),
        material=stainless,
        name="chimney_shell",
    )
    hood.visual(
        Box((0.94, 0.024, 0.120)),
        origin=Origin(xyz=(0.0, -0.590, 0.060)),
        material=stainless,
        name="front_panel",
    )
    hood.visual(
        mesh_from_geometry(
            SlotPatternPanelGeometry(
                (0.90, 0.58),
                0.004,
                slot_size=(0.055, 0.007),
                pitch=(0.075, 0.025),
                frame=0.026,
                corner_radius=0.004,
                slot_angle_deg=0.0,
                stagger=True,
            ),
            "slotted_grease_filter",
        ),
        origin=Origin(xyz=(0.0, -0.260, 0.010)),
        material=dark_filter,
        name="underside_filter",
    )
    hood.visual(
        Box((0.90, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, -0.578, 0.130)),
        material=stainless,
        name="front_lip",
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.056,
                0.026,
                body_style="skirted",
                top_diameter=0.040,
                skirt=KnobSkirt(0.066, 0.006, flare=0.07, chamfer=0.001),
                grip=KnobGrip(style="fluted", count=20, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
                center=False,
            ),
            "selector_knob_cap",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_plastic,
        name="knob_cap",
    )

    button_0 = model.part("button_0")
    button_0.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(xyz=(0.0, -0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=button_plastic,
        name="button_cap",
    )

    button_1 = model.part("button_1")
    button_1.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(xyz=(0.0, -0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=button_plastic,
        name="button_cap",
    )

    model.articulation(
        "hood_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=hood,
        child=selector_knob,
        origin=Origin(xyz=(-0.345, FRONT_PANEL_Y, 0.073)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0),
    )
    model.articulation(
        "hood_to_button_0",
        ArticulationType.PRISMATIC,
        parent=hood,
        child=button_0,
        origin=Origin(xyz=(-0.255, FRONT_PANEL_Y, 0.073)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=7.0, velocity=0.08, lower=0.0, upper=0.010),
    )
    model.articulation(
        "hood_to_button_1",
        ArticulationType.PRISMATIC,
        parent=hood,
        child=button_1,
        origin=Origin(xyz=(-0.205, FRONT_PANEL_Y, 0.073)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=7.0, velocity=0.08, lower=0.0, upper=0.010),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hood = object_model.get_part("hood")
    knob = object_model.get_part("selector_knob")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")
    knob_joint = object_model.get_articulation("hood_to_selector_knob")
    button_0_joint = object_model.get_articulation("hood_to_button_0")
    button_1_joint = object_model.get_articulation("hood_to_button_1")

    ctx.check(
        "only the three controls articulate",
        len(object_model.articulations) == 3,
        details=f"articulations={[joint.name for joint in object_model.articulations]}",
    )
    ctx.check(
        "selector knob is continuous",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_joint.articulation_type}",
    )
    ctx.check(
        "buttons are prismatic plungers",
        button_0_joint.articulation_type == ArticulationType.PRISMATIC
        and button_1_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"types={[button_0_joint.articulation_type, button_1_joint.articulation_type]}",
    )

    knob_pos = ctx.part_world_position(knob)
    button_0_pos = ctx.part_world_position(button_0)
    button_1_pos = ctx.part_world_position(button_1)
    ctx.check(
        "controls occupy the left end of the front panel",
        knob_pos is not None
        and button_0_pos is not None
        and button_1_pos is not None
        and knob_pos[0] < -0.30
        and button_0_pos[0] > knob_pos[0] + 0.070
        and button_1_pos[0] > button_0_pos[0] + 0.035
        and abs(knob_pos[2] - button_0_pos[2]) < 0.002
        and abs(button_0_pos[2] - button_1_pos[2]) < 0.002,
        details=f"knob={knob_pos}, button_0={button_0_pos}, button_1={button_1_pos}",
    )

    ctx.expect_gap(
        hood,
        knob,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="front_panel",
        negative_elem="knob_cap",
        name="selector knob seats on front panel",
    )
    ctx.expect_overlap(
        knob,
        hood,
        axes="xz",
        min_overlap=0.035,
        elem_a="knob_cap",
        elem_b="front_panel",
        name="selector knob footprint is on the panel",
    )

    for button, joint, label in (
        (button_0, button_0_joint, "button_0"),
        (button_1, button_1_joint, "button_1"),
    ):
        ctx.allow_overlap(
            button,
            hood,
            elem_a="button_cap",
            elem_b="front_panel",
            reason="A real push-button plunger travels a short distance into the switch panel when pressed.",
        )
        ctx.expect_gap(
            hood,
            button,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="front_panel",
            negative_elem="button_cap",
            name=f"{label} rests proud of the panel",
        )
        ctx.expect_overlap(
            button,
            hood,
            axes="xz",
            min_overlap=0.020,
            elem_a="button_cap",
            elem_b="front_panel",
            name=f"{label} footprint is on the panel",
        )
        rest_pos = ctx.part_world_position(button)
        upper = joint.motion_limits.upper if joint.motion_limits is not None else 0.010
        with ctx.pose({joint: upper}):
            pressed_pos = ctx.part_world_position(button)
            ctx.expect_gap(
                hood,
                button,
                axis="y",
                max_penetration=0.011,
                positive_elem="front_panel",
                negative_elem="button_cap",
                name=f"{label} plunger has short inward travel",
            )
        ctx.check(
            f"{label} moves inward along the panel normal",
            rest_pos is not None and pressed_pos is not None and pressed_pos[1] > rest_pos[1] + 0.009,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
