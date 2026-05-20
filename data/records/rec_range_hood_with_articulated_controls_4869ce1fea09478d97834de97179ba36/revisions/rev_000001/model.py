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
    MotionProperties,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _rounded_loop(width: float, depth: float, radius: float, y_center: float, z: float) -> list[tuple[float, float, float]]:
    return [(x, y + y_center, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=5)]


def _tapered_canopy_shell() -> MeshGeometry:
    """Thin, open-ended sheet-metal frustum for the visible hood canopy."""
    geom = MeshGeometry()

    outer_bottom = _rounded_loop(0.92, 0.54, 0.030, 0.020, 0.100)
    outer_top = _rounded_loop(0.42, 0.22, 0.020, -0.130, 0.455)
    inner_bottom = _rounded_loop(0.870, 0.490, 0.022, 0.020, 0.102)
    inner_top = _rounded_loop(0.370, 0.170, 0.014, -0.130, 0.453)

    loops = [outer_bottom, outer_top, inner_bottom, inner_top]
    ids: list[list[int]] = []
    for loop in loops:
        ids.append([geom.add_vertex(x, y, z) for x, y, z in loop])

    ob, ot, ib, it = ids
    count = len(ob)
    for i in range(count):
        j = (i + 1) % count
        # Outer sloped sheet.
        _quad(geom, ob[i], ob[j], ot[j], ot[i])
        # Inner return surface, reversed so the shell has a real thickness.
        _quad(geom, ib[j], ib[i], it[i], it[j])
        # Lower and upper rolled lips leave the bottom and chimney throat open.
        _quad(geom, ob[j], ob[i], ib[i], ib[j])
        _quad(geom, ot[i], ot[j], it[j], it[i])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_range_hood")

    stainless = model.material("brushed_stainless", rgba=(0.70, 0.72, 0.70, 1.0))
    darker_steel = model.material("shadowed_stainless", rgba=(0.38, 0.40, 0.39, 1.0))
    black_glass = model.material("black_control_glass", rgba=(0.02, 0.022, 0.024, 1.0))
    filter_dark = model.material("dark_filter_mesh", rgba=(0.12, 0.13, 0.13, 1.0))
    wall_paint = model.material("warm_kitchen_wall", rgba=(0.86, 0.84, 0.78, 1.0))

    hood = model.part("hood")
    hood.visual(
        Box((1.06, 0.030, 0.92)),
        origin=Origin(xyz=(0.0, -0.266, 0.435)),
        material=wall_paint,
        name="wall_mount_panel",
    )
    hood.visual(
        Box((0.34, 0.20, 0.60)),
        origin=Origin(xyz=(0.0, -0.165, 0.755)),
        material=stainless,
        name="chimney_duct",
    )
    hood.visual(
        mesh_from_geometry(_tapered_canopy_shell(), "tapered_canopy_shell"),
        material=stainless,
        name="tapered_canopy",
    )

    # Rectangular intake rim below the tapered cover.
    hood.visual(
        Box((0.94, 0.060, 0.100)),
        origin=Origin(xyz=(0.0, 0.290, 0.055)),
        material=stainless,
        name="front_intake_lip",
    )
    hood.visual(
        Box((0.94, 0.055, 0.090)),
        origin=Origin(xyz=(0.0, -0.245, 0.055)),
        material=stainless,
        name="rear_intake_lip",
    )
    for x, name in [(-0.445, "side_intake_lip_0"), (0.445, "side_intake_lip_1")]:
        hood.visual(
            Box((0.060, 0.530, 0.095)),
            origin=Origin(xyz=(x, 0.020, 0.054)),
            material=stainless,
            name=name,
        )

    filter_panel = SlotPatternPanelGeometry(
        (0.440, 0.500),
        0.006,
        slot_size=(0.055, 0.010),
        pitch=(0.075, 0.030),
        frame=0.018,
        corner_radius=0.010,
        slot_angle_deg=0.0,
        stagger=True,
    )
    hood.visual(
        mesh_from_geometry(filter_panel, "filter_panel_0"),
        origin=Origin(xyz=(-0.205, 0.020, 0.010)),
        material=filter_dark,
        name="filter_panel_0",
    )
    hood.visual(
        mesh_from_geometry(filter_panel, "filter_panel_1"),
        origin=Origin(xyz=(0.205, 0.020, 0.010)),
        material=filter_dark,
        name="filter_panel_1",
    )
    hood.visual(
        Box((0.030, 0.500, 0.014)),
        origin=Origin(xyz=(0.0, 0.020, 0.014)),
        material=stainless,
        name="center_filter_seam",
    )

    # A proud, dark front control strip spans the leading face.
    control_y = 0.326
    control_depth = 0.035
    knob_axis_y = control_y + control_depth / 2.0
    hood.visual(
        Box((0.720, control_depth, 0.104)),
        origin=Origin(xyz=(0.0, control_y, 0.120)),
        material=black_glass,
        name="control_strip",
    )
    for x in (-0.220, 0.0, 0.220):
        hood.visual(
            Box((0.050, 0.004, 0.010)),
            origin=Origin(xyz=(x, knob_axis_y + 0.002, 0.177)),
            material=Material("white_tick", rgba=(0.93, 0.93, 0.88, 1.0)),
            name=f"control_tick_{x:+.2f}",
        )

    knob_meshes = []
    for i in range(3):
        knob_geom = KnobGeometry(
            0.060,
            0.030,
            body_style="skirted",
            top_diameter=0.044,
            skirt=KnobSkirt(0.068, 0.006, flare=0.05, chamfer=0.0012),
            grip=KnobGrip(style="fluted", count=20, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="raised", depth=0.0009, angle_deg=90.0),
            center=False,
        )
        knob_meshes.append(mesh_from_geometry(knob_geom, f"rotary_knob_{i}"))

    for i, x in enumerate((-0.220, 0.0, 0.220)):
        knob = model.part(f"knob_{i}")
        knob.visual(
            knob_meshes[i],
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=darker_steel,
            name="cap",
        )
        knob.visual(
            Cylinder(radius=0.008, length=0.026),
            origin=Origin(xyz=(0.0, -0.007, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=darker_steel,
            name="shaft",
        )
        model.articulation(
            f"hood_to_knob_{i}",
            ArticulationType.CONTINUOUS,
            parent=hood,
            child=knob,
            origin=Origin(xyz=(x, knob_axis_y, 0.120)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=6.0),
            motion_properties=MotionProperties(damping=0.01, friction=0.02),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hood = object_model.get_part("hood")

    ctx.check("three rotary knobs", all(object_model.get_part(f"knob_{i}") is not None for i in range(3)))

    for i in range(3):
        knob = object_model.get_part(f"knob_{i}")
        joint = object_model.get_articulation(f"hood_to_knob_{i}")
        ctx.check(
            f"knob_{i} is continuous rotary control",
            joint.articulation_type == ArticulationType.CONTINUOUS and tuple(joint.axis) == (0.0, 1.0, 0.0),
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )
        ctx.allow_overlap(
            hood,
            knob,
            elem_a="control_strip",
            elem_b="shaft",
            reason="The rotary control shaft is intentionally captured through the front control strip.",
        )
        ctx.expect_within(
            knob,
            hood,
            axes="xz",
            inner_elem="shaft",
            outer_elem="control_strip",
            margin=0.002,
            name=f"knob_{i} shaft is centered in control strip",
        )
        ctx.expect_overlap(
            knob,
            hood,
            axes="y",
            elem_a="shaft",
            elem_b="control_strip",
            min_overlap=0.010,
            name=f"knob_{i} shaft penetrates retained panel bushing",
        )
        rest_pos = ctx.part_world_position(knob)
        with ctx.pose({joint: math.pi / 2.0}):
            turned_pos = ctx.part_world_position(knob)
        ctx.check(
            f"knob_{i} rotation does not translate",
            rest_pos is not None
            and turned_pos is not None
            and max(abs(a - b) for a, b in zip(rest_pos, turned_pos)) < 1e-6,
            details=f"rest={rest_pos}, turned={turned_pos}",
        )

    return ctx.report()


object_model = build_object_model()
