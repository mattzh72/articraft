from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
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


def _add_quad(mesh: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    mesh.add_face(a, b, c)
    mesh.add_face(a, c, d)


def _rect_loop(width: float, depth: float, z: float) -> list[tuple[float, float, float]]:
    return [
        (-width / 2.0, -depth / 2.0, z),
        (width / 2.0, -depth / 2.0, z),
        (width / 2.0, depth / 2.0, z),
        (-width / 2.0, depth / 2.0, z),
    ]


def _rect_shell(
    *,
    bottom_outer: tuple[float, float],
    top_outer: tuple[float, float],
    bottom_inner: tuple[float, float],
    top_inner: tuple[float, float],
    z_bottom: float,
    z_top: float,
) -> MeshGeometry:
    """A closed thin rectangular frustum/tube shell with open center."""

    mesh = MeshGeometry()
    loops = [
        _rect_loop(bottom_outer[0], bottom_outer[1], z_bottom),
        _rect_loop(top_outer[0], top_outer[1], z_top),
        _rect_loop(bottom_inner[0], bottom_inner[1], z_bottom),
        _rect_loop(top_inner[0], top_inner[1], z_top),
    ]
    indices: list[list[int]] = []
    for loop in loops:
        indices.append([mesh.add_vertex(x, y, z) for x, y, z in loop])

    outer_bottom, outer_top, inner_bottom, inner_top = indices
    for i in range(4):
        j = (i + 1) % 4
        _add_quad(mesh, outer_bottom[i], outer_bottom[j], outer_top[j], outer_top[i])
        _add_quad(mesh, inner_bottom[j], inner_bottom[i], inner_top[i], inner_top[j])
        _add_quad(mesh, outer_bottom[j], outer_bottom[i], inner_bottom[i], inner_bottom[j])
        _add_quad(mesh, outer_top[i], outer_top[j], inner_top[j], inner_top[i])
    return mesh


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood")

    stainless = model.material("brushed_stainless", rgba=(0.70, 0.70, 0.66, 1.0))
    shadow = model.material("dark_filter", rgba=(0.055, 0.055, 0.055, 1.0))
    dark_trim = model.material("shadowed_control_strip", rgba=(0.12, 0.12, 0.12, 1.0))
    knob_black = model.material("satin_black_knobs", rgba=(0.025, 0.025, 0.025, 1.0))
    white_mark = model.material("white_indicator_marks", rgba=(0.94, 0.92, 0.84, 1.0))

    hood = model.part("hood")

    canopy_shell = _rect_shell(
        bottom_outer=(0.92, 0.52),
        top_outer=(0.36, 0.22),
        bottom_inner=(0.80, 0.40),
        top_inner=(0.25, 0.11),
        z_bottom=0.0,
        z_top=0.24,
    )
    hood.visual(
        mesh_from_geometry(canopy_shell, "canopy_shell"),
        material=stainless,
        name="canopy_shell",
    )

    # A thin panel closes the underside around the two grease-filter openings and
    # carries the front-edge controls.
    hood.visual(
        Box((0.84, 0.43, 0.010)),
        origin=Origin(xyz=(0.0, -0.005, -0.005)),
        material=stainless,
        name="underside_panel",
    )
    hood.visual(
        Box((0.84, 0.065, 0.012)),
        origin=Origin(xyz=(0.0, -0.225, -0.006)),
        material=dark_trim,
        name="control_strip",
    )
    hood.visual(
        Box((0.91, 0.035, 0.040)),
        origin=Origin(xyz=(0.0, -0.260, 0.020)),
        material=stainless,
        name="front_lip",
    )

    filter_mesh = mesh_from_geometry(
        SlotPatternPanelGeometry(
            (0.32, 0.18),
            0.004,
            slot_size=(0.055, 0.007),
            pitch=(0.075, 0.020),
            frame=0.014,
            corner_radius=0.004,
            slot_angle_deg=0.0,
            stagger=True,
        ),
        "grease_filter_panel",
    )
    for x, name in [(-0.19, "filter_0"), (0.19, "filter_1")]:
        hood.visual(
            filter_mesh,
            origin=Origin(xyz=(x, 0.035, -0.012)),
            material=shadow,
            name=name,
        )

    lower_chimney = _rect_shell(
        bottom_outer=(0.32, 0.20),
        top_outer=(0.32, 0.20),
        bottom_inner=(0.294, 0.174),
        top_inner=(0.294, 0.174),
        z_bottom=0.238,
        z_top=0.80,
    )
    hood.visual(
        mesh_from_geometry(lower_chimney, "lower_chimney_shell"),
        material=stainless,
        name="lower_chimney",
    )

    upper_chimney = _rect_shell(
        bottom_outer=(0.294, 0.174),
        top_outer=(0.294, 0.174),
        bottom_inner=(0.274, 0.154),
        top_inner=(0.274, 0.154),
        z_bottom=0.56,
        z_top=1.20,
    )
    hood.visual(
        mesh_from_geometry(upper_chimney, "upper_chimney_shell"),
        material=stainless,
        name="upper_chimney",
    )
    hood.visual(
        Box((0.32, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -0.105, 0.80)),
        material=dark_trim,
        name="telescoping_seam",
    )

    knob_geometry = KnobGeometry(
        0.042,
        0.024,
        body_style="skirted",
        top_diameter=0.034,
        skirt=KnobSkirt(0.052, 0.006, flare=0.08, chamfer=0.0012),
        grip=KnobGrip(style="fluted", count=18, depth=0.0012),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
        center=False,
    )
    knob_mesh = mesh_from_geometry(knob_geometry, "rotary_knob")
    knob_x_positions = (-0.27, -0.09, 0.09, 0.27)
    for idx, x in enumerate(knob_x_positions):
        knob = model.part(f"knob_{idx}")
        knob.visual(
            knob_mesh,
            origin=Origin(rpy=(math.pi, 0.0, 0.0)),
            material=knob_black,
            name="knob_body",
        )
        knob.visual(
            Box((0.004, 0.020, 0.0008)),
            origin=Origin(xyz=(0.0, 0.006, -0.0244)),
            material=white_mark,
            name="indicator_mark",
        )
        model.articulation(
            f"hood_to_knob_{idx}",
            ArticulationType.CONTINUOUS,
            parent=hood,
            child=knob,
            origin=Origin(xyz=(x, -0.225, -0.012)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.35, velocity=6.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hood = object_model.get_part("hood")
    knobs = [object_model.get_part(f"knob_{idx}") for idx in range(4)]
    joints = [object_model.get_articulation(f"hood_to_knob_{idx}") for idx in range(4)]

    ctx.check(
        "only four knob articulations",
        len(object_model.articulations) == 4
        and all(joint.articulation_type == ArticulationType.CONTINUOUS for joint in joints),
        details=f"articulations={[joint.name for joint in object_model.articulations]}",
    )
    ctx.check(
        "knob axes are vertical",
        all(tuple(round(v, 6) for v in joint.axis) == (0.0, 0.0, 1.0) for joint in joints),
        details=f"axes={[joint.axis for joint in joints]}",
    )

    positions = [ctx.part_world_position(knob) for knob in knobs]
    row_ok = all(pos is not None for pos in positions)
    if row_ok:
        ys = [pos[1] for pos in positions if pos is not None]
        zs = [pos[2] for pos in positions if pos is not None]
        xs = [pos[0] for pos in positions if pos is not None]
        row_ok = (
            max(ys) - min(ys) < 1e-6
            and max(zs) - min(zs) < 1e-6
            and all(xs[i] < xs[i + 1] for i in range(3))
        )
    ctx.check("knobs form a straight underside row", row_ok, details=f"positions={positions}")

    for idx, knob in enumerate(knobs):
        ctx.expect_gap(
            hood,
            knob,
            axis="z",
            positive_elem="control_strip",
            negative_elem="knob_body",
            max_gap=0.001,
            max_penetration=0.00001,
            name=f"knob_{idx} seats against underside control strip",
        )

    with ctx.pose({joints[0]: math.pi * 1.5}):
        rotated_position = ctx.part_world_position(knobs[0])
    ctx.check(
        "knob rotation keeps vertical axis fixed",
        positions[0] is not None and rotated_position == positions[0],
        details=f"rest={positions[0]}, rotated={rotated_position}",
    )

    return ctx.report()


object_model = build_object_model()
