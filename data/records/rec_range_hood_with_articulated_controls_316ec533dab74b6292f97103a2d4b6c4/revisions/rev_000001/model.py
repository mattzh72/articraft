from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(radius: float, segments: int = 32) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _canopy_shell_geometry() -> MeshGeometry:
    """Open, thin visual shell for the tapered stainless canopy."""
    geom = MeshGeometry()

    lower = [
        (-0.46, -0.34, 0.13),
        (0.46, -0.34, 0.13),
        (0.46, 0.22, 0.13),
        (-0.46, 0.22, 0.13),
    ]
    upper = [
        (-0.19, -0.05, 0.30),
        (0.19, -0.05, 0.30),
        (0.19, 0.18, 0.30),
        (-0.19, 0.18, 0.30),
    ]

    verts = [geom.add_vertex(*p) for p in lower + upper]

    def add_quad(a: int, b: int, c: int, d: int) -> None:
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)

    add_quad(verts[0], verts[1], verts[5], verts[4])
    add_quad(verts[1], verts[2], verts[6], verts[5])
    add_quad(verts[2], verts[3], verts[7], verts[6])
    add_quad(verts[3], verts[0], verts[4], verts[7])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood")

    stainless = model.material("brushed_stainless", rgba=(0.72, 0.72, 0.68, 1.0))
    dark_glass = model.material("black_glass", rgba=(0.02, 0.022, 0.025, 1.0))
    shadow = model.material("dark_filter", rgba=(0.05, 0.055, 0.055, 1.0))
    control_black = model.material("satin_black_controls", rgba=(0.01, 0.01, 0.012, 1.0))
    control_chrome = model.material("soft_chrome_controls", rgba=(0.82, 0.82, 0.78, 1.0))

    hood = model.part("hood")

    # Tall rectangular chimney chase and the tapered rectangular canopy.
    hood.visual(
        mesh_from_geometry(_canopy_shell_geometry(), "canopy_shell"),
        material=stainless,
        name="canopy_shell",
    )
    hood.visual(
        Box((0.32, 0.20, 0.82)),
        origin=Origin(xyz=(0.0, 0.06, 0.72)),
        material=stainless,
        name="chimney_chase",
    )
    hood.visual(
        Box((0.39, 0.25, 0.035)),
        origin=Origin(xyz=(0.0, 0.065, 0.3175)),
        material=stainless,
        name="chimney_collar",
    )

    # A real hood has a hollow underside.  The dark filter panels and ribs make
    # the underside read as an intake cavity rather than a sealed solid block.
    for i, x in enumerate((-0.205, 0.205)):
        hood.visual(
            Box((0.39, 0.54, 0.006)),
            origin=Origin(xyz=(x, -0.07, 0.003)),
            material=shadow,
            name=f"filter_panel_{i}",
        )
        for rib in range(6):
            hood.visual(
                Box((0.35, 0.008, 0.004)),
                origin=Origin(xyz=(x, -0.23 + 0.055 * rib, 0.008)),
                material=stainless,
                name=f"filter_rib_{i}_{rib}",
            )

    # Front control fascia: stainless frame around a black insert plate with
    # through-holes for the five separate controls.
    hood.visual(
        Box((0.92, 0.044, 0.0275)),
        origin=Origin(xyz=(0.0, -0.362, 0.01375)),
        material=stainless,
        name="front_lower_rail",
    )
    hood.visual(
        Box((0.92, 0.044, 0.0275)),
        origin=Origin(xyz=(0.0, -0.362, 0.11625)),
        material=stainless,
        name="front_upper_rail",
    )
    for x, name in ((-0.40, "front_side_post_0"), (0.40, "front_side_post_1")):
        hood.visual(
            Box((0.12, 0.044, 0.075)),
            origin=Origin(xyz=(x, -0.362, 0.065)),
            material=stainless,
            name=name,
        )

    control_xs = (-0.18, -0.11, 0.0, 0.11, 0.18)
    panel_holes = [
        [(x + px, py) for px, py in _circle_profile(0.021, 28)]
        for x in (control_xs[0], control_xs[1], control_xs[3], control_xs[4])
    ]
    panel_holes.append(
        [(control_xs[2] + px, py) for px, py in _circle_profile(0.029, 36)]
    )
    panel_geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.62, 0.075, 0.006, corner_segments=5),
        panel_holes,
        0.004,
        center=True,
    )
    hood.visual(
        mesh_from_geometry(panel_geom, "control_panel"),
        origin=Origin(xyz=(0.0, -0.382, 0.065), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_glass,
        name="control_panel",
    )

    # Central continuous rotary knob with a skirt, fluting, and indicator mark.
    knob = model.part("knob")
    knob_geom = KnobGeometry(
        0.062,
        0.028,
        body_style="skirted",
        top_diameter=0.048,
        skirt=KnobSkirt(0.071, 0.006, flare=0.06, chamfer=0.001),
        grip=KnobGrip(style="fluted", count=22, depth=0.0011),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
        center=False,
    )
    knob.visual(
        mesh_from_geometry(knob_geom, "knob_cap"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=control_chrome,
        name="knob_cap",
    )
    knob.visual(
        Cylinder(radius=0.011, length=0.026),
        origin=Origin(xyz=(0.0, 0.013, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=control_black,
        name="knob_shaft",
    )
    model.articulation(
        "knob_spin",
        ArticulationType.CONTINUOUS,
        parent=hood,
        child=knob,
        origin=Origin(xyz=(0.0, -0.384, 0.065)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0),
    )

    button_positions = (control_xs[0], control_xs[1], control_xs[3], control_xs[4])
    for index, x in enumerate(button_positions):
        button = model.part(f"button_{index}")
        button.visual(
            Cylinder(radius=0.017, length=0.014),
            origin=Origin(xyz=(0.0, -0.007, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=control_black,
            name="button_cap",
        )
        button.visual(
            Cylinder(radius=0.0075, length=0.026),
            origin=Origin(xyz=(0.0, 0.013, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=control_black,
            name="button_stem",
        )
        model.articulation(
            f"button_{index}_press",
            ArticulationType.PRISMATIC,
            parent=hood,
            child=button,
            origin=Origin(xyz=(x, -0.384, 0.065)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.05, lower=0.0, upper=0.008),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hood = object_model.get_part("hood")
    knob = object_model.get_part("knob")
    knob_joint = object_model.get_articulation("knob_spin")

    ctx.check(
        "five articulated controls only",
        len(object_model.articulations) == 5 and len(object_model.parts) == 6,
        details=f"parts={len(object_model.parts)}, joints={len(object_model.articulations)}",
    )
    ctx.check(
        "central control is continuous rotary knob",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=str(knob_joint.articulation_type),
    )
    ctx.expect_overlap(
        knob,
        hood,
        axes="xz",
        elem_a="knob_cap",
        elem_b="control_panel",
        min_overlap=0.045,
        name="knob is centered on the control insert",
    )

    rest_positions = {}
    pressed_positions = {}
    for index in range(4):
        button = object_model.get_part(f"button_{index}")
        joint = object_model.get_articulation(f"button_{index}_press")
        rest_positions[index] = ctx.part_world_position(button)
        ctx.check(
            f"button_{index} is prismatic",
            joint.articulation_type == ArticulationType.PRISMATIC,
            details=str(joint.articulation_type),
        )
        ctx.expect_overlap(
            button,
            hood,
            axes="xz",
            elem_a="button_cap",
            elem_b="control_panel",
            min_overlap=0.020,
            name=f"button_{index} sits in its panel opening",
        )
        with ctx.pose({joint: 0.008}):
            pressed_positions[index] = ctx.part_world_position(button)

        rest = rest_positions[index]
        pressed = pressed_positions[index]
        ctx.check(
            f"button_{index} moves inward",
            rest is not None and pressed is not None and pressed[1] > rest[1] + 0.006,
            details=f"rest={rest}, pressed={pressed}",
        )

    return ctx.report()


object_model = build_object_model()
