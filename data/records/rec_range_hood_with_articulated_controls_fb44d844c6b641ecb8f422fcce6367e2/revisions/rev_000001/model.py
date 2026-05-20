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
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


FRONT_Y = -0.320
CONTROL_Z = 0.075
KNOB_X = 0.330
BUTTON_TRAVEL = 0.006


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _loop(
    geom: MeshGeometry,
    half_x: float,
    half_y: float,
    z: float,
) -> list[int]:
    return [
        geom.add_vertex(-half_x, -half_y, z),
        geom.add_vertex(half_x, -half_y, z),
        geom.add_vertex(half_x, half_y, z),
        geom.add_vertex(-half_x, half_y, z),
    ]


def _connect_loops(geom: MeshGeometry, lower: list[int], upper: list[int]) -> None:
    for i in range(4):
        j = (i + 1) % 4
        _add_quad(geom, lower[i], lower[j], upper[j], upper[i])


def _hood_shell_geometry() -> MeshGeometry:
    """One connected thin-sheet mesh for the flared canopy and chimney sleeve."""
    geom = MeshGeometry()

    # Canopy: broad open lower rim, gently tapering to the chimney footprint.
    outer_bottom = _loop(geom, 0.470, 0.280, 0.000)
    inner_bottom = _loop(geom, 0.440, 0.250, 0.018)
    outer_neck = _loop(geom, 0.170, 0.120, 0.280)
    inner_neck = _loop(geom, 0.142, 0.092, 0.280)

    _connect_loops(geom, outer_bottom, outer_neck)
    _connect_loops(geom, inner_neck, inner_bottom)

    # Rolled lower rim and the top landing lip around the chimney opening.
    for i in range(4):
        j = (i + 1) % 4
        _add_quad(geom, outer_bottom[j], outer_bottom[i], inner_bottom[i], inner_bottom[j])
        _add_quad(geom, outer_neck[i], outer_neck[j], inner_neck[j], inner_neck[i])

    # Straight box chimney above the canopy.  It shares the lower neck loops so
    # the static hood body is a single physically connected sheet-metal shell.
    outer_top = _loop(geom, 0.170, 0.120, 0.950)
    inner_top = _loop(geom, 0.142, 0.092, 0.950)
    _connect_loops(geom, outer_neck, outer_top)
    _connect_loops(geom, inner_top, inner_neck)
    for i in range(4):
        j = (i + 1) % 4
        _add_quad(geom, outer_top[i], outer_top[j], inner_top[j], inner_top[i])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood")

    stainless = model.material("brushed_stainless", rgba=(0.72, 0.72, 0.68, 1.0))
    shadow = model.material("dark_filter_shadow", rgba=(0.055, 0.058, 0.060, 1.0))
    black = model.material("satin_black", rgba=(0.015, 0.014, 0.013, 1.0))
    white = model.material("white_indicator", rgba=(0.93, 0.92, 0.86, 1.0))

    hood = model.part("hood")
    hood.visual(
        mesh_from_geometry(_hood_shell_geometry(), "hood_shell"),
        material=stainless,
        name="hood_shell",
    )
    hood.visual(
        Box((0.900, 0.070, 0.105)),
        origin=Origin(xyz=(0.0, -0.285, 0.066)),
        material=stainless,
        name="front_strip",
    )
    hood.visual(
        mesh_from_geometry(
            SlotPatternPanelGeometry(
                (0.800, 0.430),
                0.006,
                slot_size=(0.050, 0.010),
                pitch=(0.075, 0.040),
                frame=0.025,
                corner_radius=0.006,
                slot_angle_deg=0.0,
            ),
            "grease_filter",
        ),
        origin=Origin(xyz=(0.0, -0.010, 0.015)),
        material=shadow,
        name="grease_filter",
    )
    # Narrow rails make the filter visibly retained by the lower canopy rim.
    for name, x in (("filter_rail_0", -0.425), ("filter_rail_1", 0.425)):
        hood.visual(
            Box((0.080, 0.450, 0.018)),
            origin=Origin(xyz=(x, -0.010, 0.018)),
            material=stainless,
            name=name,
        )
    hood.visual(
        mesh_from_geometry(TorusGeometry(0.023, 0.003, radial_segments=24, tubular_segments=10), "button_bezel"),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.001, CONTROL_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="button_bezel",
    )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.072,
            0.030,
            body_style="skirted",
            top_diameter=0.058,
            skirt=KnobSkirt(0.088, 0.007, flare=0.06, chamfer=0.001),
            grip=KnobGrip(style="fluted", count=18, depth=0.0013),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=0.0),
            center=False,
        ),
        "rotary_knob",
    )

    for part_name, x in (("left_knob", -KNOB_X), ("right_knob", KNOB_X)):
        knob = model.part(part_name)
        knob.visual(
            knob_mesh,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black,
            name="knob_shell",
        )
        knob.visual(
            Box((0.006, 0.0025, 0.026)),
            origin=Origin(xyz=(0.0, -0.032, 0.016)),
            material=white,
            name="indicator_mark",
        )
        model.articulation(
            f"hood_to_{part_name}",
            ArticulationType.CONTINUOUS,
            parent=hood,
            child=knob,
            origin=Origin(xyz=(x, FRONT_Y, CONTROL_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=0.35, velocity=8.0),
        )

    button = model.part("button")
    button.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(0.0, -0.007, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="button_cap",
    )
    model.articulation(
        "hood_to_button",
        ArticulationType.PRISMATIC,
        parent=hood,
        child=button,
        origin=Origin(xyz=(0.0, FRONT_Y, CONTROL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.08, lower=0.0, upper=BUTTON_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hood = object_model.get_part("hood")
    left_knob = object_model.get_part("left_knob")
    right_knob = object_model.get_part("right_knob")
    button = object_model.get_part("button")
    left_joint = object_model.get_articulation("hood_to_left_knob")
    right_joint = object_model.get_articulation("hood_to_right_knob")
    button_joint = object_model.get_articulation("hood_to_button")

    ctx.check(
        "only_controls_articulate",
        len(object_model.articulations) == 3,
        details=f"articulations={[joint.name for joint in object_model.articulations]!r}",
    )
    for joint, label in ((left_joint, "left_knob"), (right_joint, "right_knob")):
        ctx.check(
            f"{label}_continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS and tuple(joint.axis) == (0.0, -1.0, 0.0),
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )
    ctx.check(
        "button_short_plunger",
        button_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(button_joint.axis) == (0.0, 1.0, 0.0)
        and button_joint.motion_limits is not None
        and 0.004 <= float(button_joint.motion_limits.upper) <= 0.010,
        details=f"type={button_joint.articulation_type}, axis={button_joint.axis}, limits={button_joint.motion_limits}",
    )

    ctx.expect_gap(
        hood,
        left_knob,
        axis="y",
        positive_elem="front_strip",
        negative_elem="knob_shell",
        max_gap=0.002,
        max_penetration=0.001,
        name="left knob seats on front strip",
    )
    ctx.expect_gap(
        hood,
        right_knob,
        axis="y",
        positive_elem="front_strip",
        negative_elem="knob_shell",
        max_gap=0.002,
        max_penetration=0.001,
        name="right knob seats on front strip",
    )
    ctx.expect_gap(
        hood,
        button,
        axis="y",
        positive_elem="front_strip",
        negative_elem="button_cap",
        max_gap=0.002,
        max_penetration=0.001,
        name="button starts proud of front strip",
    )

    # The small button is intentionally allowed to enter its simplified panel
    # socket at full depression; this is local to the plunger and front strip.
    ctx.allow_overlap(
        hood,
        button,
        elem_a="front_strip",
        elem_b="button_cap",
        reason="The push-button plunger travels into a simplified socket in the front control strip.",
    )
    rest_button_pos = ctx.part_world_position(button)
    with ctx.pose({button_joint: BUTTON_TRAVEL}):
        pressed_button_pos = ctx.part_world_position(button)
        ctx.expect_gap(
            hood,
            button,
            axis="y",
            positive_elem="front_strip",
            negative_elem="button_cap",
            max_penetration=BUTTON_TRAVEL + 0.001,
            name="pressed button remains a short local plunger insertion",
        )
    ctx.check(
        "button moves inward",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[1] > rest_button_pos[1] + 0.004,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    mark_rest = ctx.part_element_world_aabb(left_knob, elem="indicator_mark")
    with ctx.pose({left_joint: math.pi / 2.0}):
        mark_turned = ctx.part_element_world_aabb(left_knob, elem="indicator_mark")
    if mark_rest is not None and mark_turned is not None:
        rest_center = tuple((mark_rest[0][i] + mark_rest[1][i]) * 0.5 for i in range(3))
        turned_center = tuple((mark_turned[0][i] + mark_turned[1][i]) * 0.5 for i in range(3))
        ctx.check(
            "knob indicator rotates about front axis",
            abs(turned_center[0] - rest_center[0]) > 0.010
            and abs(turned_center[1] - rest_center[1]) < 0.003,
            details=f"rest={rest_center}, turned={turned_center}",
        )
    else:
        ctx.fail("knob indicator rotates about front axis", "Missing indicator AABB.")

    return ctx.report()


object_model = build_object_model()
