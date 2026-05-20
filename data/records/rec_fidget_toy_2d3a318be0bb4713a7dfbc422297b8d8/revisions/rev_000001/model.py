from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def annular_tube(outer_radius: float, inner_radius: float, length: float, z_min: float):
    """CadQuery tube with a real through bore along the pen axis."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((0.0, 0.0, z_min))
    )


def conical_socket(
    base_radius: float,
    tip_radius: float,
    bore_radius: float,
    length: float,
    z_min: float,
):
    """Hollow tapered nose shell, revolved around the +Z pen axis."""
    return LatheGeometry.from_shell_profiles(
        outer_profile=((tip_radius, z_min), (base_radius, z_min + length)),
        inner_profile=((bore_radius, z_min), (bore_radius, z_min + length)),
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retractable_ballpoint_pen")

    blue_resin = Material("blue_resin", rgba=(0.04, 0.10, 0.34, 1.0))
    black_plastic = Material("black_plastic", rgba=(0.01, 0.01, 0.012, 1.0))
    brushed_steel = Material("brushed_steel", rgba=(0.72, 0.70, 0.66, 1.0))
    spring_steel = Material("spring_steel", rgba=(0.55, 0.56, 0.58, 1.0))
    brass = Material("brass_tip", rgba=(0.92, 0.68, 0.26, 1.0))
    ink_black = Material("black_ink", rgba=(0.0, 0.0, 0.0, 1.0))

    barrel = model.part("barrel")

    barrel_shell = annular_tube(
        outer_radius=0.0055,
        inner_radius=0.0038,
        length=0.112,
        z_min=-0.054,
    )
    barrel.visual(
        mesh_from_cadquery(barrel_shell, "barrel_shell", tolerance=0.00012),
        material=blue_resin,
        name="barrel_shell",
    )

    top_collar = annular_tube(
        outer_radius=0.0059,
        inner_radius=0.0042,
        length=0.006,
        z_min=0.058,
    )
    barrel.visual(
        mesh_from_cadquery(top_collar, "top_collar", tolerance=0.0001),
        material=black_plastic,
        name="top_collar",
    )

    button_bushing = annular_tube(
        outer_radius=0.0039,
        inner_radius=0.0016,
        length=0.018,
        z_min=0.040,
    )
    barrel.visual(
        mesh_from_cadquery(button_bushing, "button_bushing", tolerance=0.00008),
        material=black_plastic,
        name="button_bushing",
    )

    nose_shell = conical_socket(
        base_radius=0.0055,
        tip_radius=0.00235,
        bore_radius=0.00072,
        length=0.018,
        z_min=-0.072,
    )
    barrel.visual(
        mesh_from_geometry(nose_shell, "nose_socket"),
        material=brushed_steel,
        name="nose_socket",
    )

    # Fixed flat pocket clip: a spring strip joined to the upper barrel by a
    # broad saddle and a small crimped lower lip.
    barrel.visual(
        Box((0.0032, 0.00085, 0.054)),
        origin=Origin(xyz=(0.0, 0.00635, 0.021)),
        material=spring_steel,
        name="clip_strip",
    )
    barrel.visual(
        Box((0.0052, 0.0018, 0.010)),
        origin=Origin(xyz=(0.0, 0.00595, 0.053)),
        material=spring_steel,
        name="clip_anchor",
    )
    barrel.visual(
        Box((0.0035, 0.0012, 0.004)),
        origin=Origin(xyz=(0.0, 0.00655, -0.008)),
        material=spring_steel,
        name="clip_lip",
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.0037, length=0.007),
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),
        material=black_plastic,
        name="button_cap",
    )
    plunger.visual(
        Cylinder(radius=0.0017, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=black_plastic,
        name="button_stem",
    )

    tip = model.part("tip")
    tip.visual(
        Cylinder(radius=0.00075, length=0.011),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=brass,
        name="tip_tube",
    )
    tip.visual(
        Sphere(radius=0.00045),
        origin=Origin(xyz=(0.0, 0.0, -0.00035)),
        material=ink_black,
        name="ball_point",
    )

    model.articulation(
        "barrel_to_plunger",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.08, lower=0.0, upper=0.004),
    )

    model.articulation(
        "barrel_to_tip",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=tip,
        origin=Origin(xyz=(0.0, 0.0, -0.072)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.10, lower=0.0, upper=0.006),
        mimic=Mimic("barrel_to_plunger", multiplier=1.5, offset=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    plunger = object_model.get_part("plunger")
    tip = object_model.get_part("tip")
    button_joint = object_model.get_articulation("barrel_to_plunger")

    ctx.allow_overlap(
        barrel,
        plunger,
        elem_a="button_bushing",
        elem_b="button_stem",
        reason="The plunger stem is intentionally captured in the simplified top guide bushing.",
    )
    ctx.allow_overlap(
        barrel,
        tip,
        elem_a="nose_socket",
        elem_b="tip_tube",
        reason="The retractable refill tip is intentionally captured by the tight nose socket guide.",
    )

    ctx.expect_within(
        plunger,
        barrel,
        axes="xy",
        inner_elem="button_stem",
        outer_elem="button_bushing",
        margin=0.0002,
        name="button stem is centered in the guide bushing",
    )
    ctx.expect_overlap(
        plunger,
        barrel,
        axes="z",
        elem_a="button_stem",
        elem_b="button_bushing",
        min_overlap=0.010,
        name="plunger stem remains retained in the top guide",
    )
    ctx.expect_within(
        tip,
        barrel,
        axes="xy",
        inner_elem="tip_tube",
        outer_elem="nose_socket",
        margin=0.0002,
        name="tip slides on the pen axis inside the nose socket",
    )
    ctx.expect_overlap(
        tip,
        barrel,
        axes="z",
        elem_a="tip_tube",
        elem_b="nose_socket",
        min_overlap=0.008,
        name="retracted tip remains guided by the nose socket",
    )

    rest_button = ctx.part_world_position(plunger)
    rest_tip = ctx.part_world_position(tip)
    with ctx.pose({button_joint: 0.004}):
        pressed_button = ctx.part_world_position(plunger)
        extended_tip = ctx.part_world_position(tip)
        ctx.expect_overlap(
            tip,
            barrel,
            axes="z",
            elem_a="tip_tube",
            elem_b="nose_socket",
            min_overlap=0.003,
            name="extended tip remains retained in the nose socket",
        )

    ctx.check(
        "push button moves into barrel top",
        rest_button is not None
        and pressed_button is not None
        and pressed_button[2] < rest_button[2] - 0.003,
        details=f"rest={rest_button}, pressed={pressed_button}",
    )
    ctx.check(
        "ballpoint tip extends from nose",
        rest_tip is not None
        and extended_tip is not None
        and extended_tip[2] < rest_tip[2] - 0.005,
        details=f"rest={rest_tip}, extended={extended_tip}",
    )

    return ctx.report()


object_model = build_object_model()
