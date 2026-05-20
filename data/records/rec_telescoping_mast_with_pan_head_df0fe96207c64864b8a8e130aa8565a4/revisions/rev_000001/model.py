from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _tube_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    segments: int = 64,
):
    """Thin-walled open-centered tube mesh with annular end lips."""
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(outer_radius, 0.0), (outer_radius, height)],
            [(inner_radius, 0.0), (inner_radius, height)],
            segments=segments,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rooftop_inspection_mast")

    galvanized = model.material("galvanized", rgba=(0.62, 0.65, 0.66, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    brushed = model.material("brushed_aluminum", rgba=(0.76, 0.78, 0.78, 1.0))
    black = model.material("black_polymer", rgba=(0.04, 0.04, 0.045, 1.0))
    warning = model.material("safety_yellow", rgba=(0.92, 0.70, 0.10, 1.0))

    base = model.part("base_can")
    base.visual(
        Cylinder(radius=0.205, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_steel,
        name="roof_flange",
    )
    base.visual(
        Cylinder(radius=0.168, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=dark_steel,
        name="can_floor",
    )
    base.visual(
        _tube_mesh(
            "base_can_shell",
            outer_radius=0.160,
            inner_radius=0.118,
            height=0.215,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=galvanized,
        name="can_shell",
    )
    base.visual(
        _tube_mesh(
            "base_can_lip",
            outer_radius=0.176,
            inner_radius=0.110,
            height=0.026,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.254)),
        material=dark_steel,
        name="top_lip",
    )
    base.visual(
        _tube_mesh(
            "lower_socket",
            outer_radius=0.064,
            inner_radius=0.037,
            height=0.120,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.210)),
        material=dark_steel,
        name="lower_socket",
    )
    base.visual(
        _tube_mesh(
            "outer_sleeve",
            outer_radius=0.045,
            inner_radius=0.034,
            height=0.560,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        material=galvanized,
        name="outer_sleeve",
    )
    base.visual(
        _tube_mesh(
            "outer_top_clamp",
            outer_radius=0.058,
            inner_radius=0.034,
            height=0.055,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.815)),
        material=dark_steel,
        name="outer_top_clamp",
    )

    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        c = math.cos(angle)
        s = math.sin(angle)
        base.visual(
            Box((0.110, 0.048, 0.014)),
            origin=Origin(
                xyz=(0.188 * c, 0.188 * s, 0.014),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_steel,
            name=f"mount_lug_{index}",
        )
        base.visual(
            Cylinder(radius=0.014, length=0.009),
            origin=Origin(xyz=(0.218 * c, 0.218 * s, 0.0255)),
            material=black,
            name=f"bolt_head_{index}",
        )

    base.visual(
        Box((0.032, 0.150, 0.135)),
        origin=Origin(xyz=(0.0, 0.133, 0.220)),
        material=warning,
        name="service_label",
    )
    base.visual(
        Cylinder(radius=0.006, length=0.048),
        origin=Origin(xyz=(0.063, 0.0, 0.842), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="outer_clamp_screw",
    )
    base.visual(
        Sphere(radius=0.016),
        origin=Origin(xyz=(0.091, 0.0, 0.842)),
        material=black,
        name="outer_clamp_knob",
    )

    middle = model.part("middle_tube")
    middle.visual(
        _tube_mesh(
            "middle_tube_shell",
            outer_radius=0.029,
            inner_radius=0.022,
            height=0.820,
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.420)),
        material=brushed,
        name="middle_tube",
    )
    middle.visual(
        Cylinder(radius=0.050, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=dark_steel,
        name="middle_stop_collar",
    )
    middle.visual(
        _tube_mesh(
            "middle_top_clamp",
            outer_radius=0.040,
            inner_radius=0.022,
            height=0.050,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
        material=dark_steel,
        name="middle_top_clamp",
    )
    middle.visual(
        Cylinder(radius=0.005, length=0.042),
        origin=Origin(xyz=(0.047, 0.0, 0.386), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="middle_clamp_screw",
    )
    middle.visual(
        Sphere(radius=0.013),
        origin=Origin(xyz=(0.073, 0.0, 0.386)),
        material=black,
        name="middle_clamp_knob",
    )

    upper = model.part("upper_tube")
    upper.visual(
        _tube_mesh(
            "upper_tube_shell",
            outer_radius=0.018,
            inner_radius=0.012,
            height=0.760,
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.360)),
        material=galvanized,
        name="upper_tube",
    )
    upper.visual(
        Cylinder(radius=0.034, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=dark_steel,
        name="upper_stop_collar",
    )
    upper.visual(
        Cylinder(radius=0.035, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.393)),
        material=dark_steel,
        name="thrust_washer",
    )
    upper.visual(
        Cylinder(radius=0.022, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.378)),
        material=brushed,
        name="pan_spindle",
    )

    pan_collar = model.part("pan_collar")
    pan_collar.visual(
        _tube_mesh(
            "pan_collar_ring",
            outer_radius=0.044,
            inner_radius=0.024,
            height=0.070,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=dark_steel,
        name="collar_ring",
    )
    pan_collar.visual(
        Box((0.120, 0.070, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.076)),
        material=dark_steel,
        name="bracket_base",
    )
    for index, x in enumerate((-0.043, 0.043)):
        pan_collar.visual(
            Box((0.018, 0.066, 0.082)),
            origin=Origin(xyz=(x, 0.0, 0.123)),
            material=dark_steel,
            name=f"bracket_ear_{index}",
        )
        pan_collar.visual(
            Cylinder(radius=0.017, length=0.020),
            origin=Origin(xyz=(x, 0.0, 0.132), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black,
            name=f"pivot_bushing_{index}",
        )
    pan_collar.visual(
        Cylinder(radius=0.007, length=0.104),
        origin=Origin(xyz=(0.0, 0.0, 0.132), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed,
        name="bracket_pin",
    )

    model.articulation(
        "base_to_middle",
        ArticulationType.PRISMATIC,
        parent=base,
        child=middle,
        origin=Origin(xyz=(0.0, 0.0, 0.860)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.18, lower=0.0, upper=0.320),
    )
    model.articulation(
        "middle_to_upper",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.400)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.16, lower=0.0, upper=0.300),
    )
    model.articulation(
        "upper_to_pan_collar",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=pan_collar,
        origin=Origin(xyz=(0.0, 0.0, 0.400)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.9,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_can")
    middle = object_model.get_part("middle_tube")
    upper = object_model.get_part("upper_tube")
    pan_collar = object_model.get_part("pan_collar")
    base_slide = object_model.get_articulation("base_to_middle")
    upper_slide = object_model.get_articulation("middle_to_upper")
    pan = object_model.get_articulation("upper_to_pan_collar")

    ctx.check(
        "mast has two prismatic stages and a revolute pan collar",
        base_slide.articulation_type == ArticulationType.PRISMATIC
        and upper_slide.articulation_type == ArticulationType.PRISMATIC
        and pan.articulation_type == ArticulationType.REVOLUTE,
        details=(
            f"types={base_slide.articulation_type},"
            f"{upper_slide.articulation_type},{pan.articulation_type}"
        ),
    )
    ctx.check(
        "all moving axes are vertical",
        base_slide.axis == (0.0, 0.0, 1.0)
        and upper_slide.axis == (0.0, 0.0, 1.0)
        and pan.axis == (0.0, 0.0, 1.0),
        details=f"axes={base_slide.axis},{upper_slide.axis},{pan.axis}",
    )

    ctx.expect_within(
        middle,
        base,
        axes="xy",
        inner_elem="middle_tube",
        outer_elem="outer_sleeve",
        margin=0.002,
        name="middle stage is centered in outer sleeve",
    )
    ctx.expect_overlap(
        middle,
        base,
        axes="z",
        elem_a="middle_tube",
        elem_b="outer_sleeve",
        min_overlap=0.100,
        name="middle stage retained in base sleeve",
    )
    ctx.expect_within(
        upper,
        middle,
        axes="xy",
        inner_elem="upper_tube",
        outer_elem="middle_tube",
        margin=0.002,
        name="upper stage is centered in middle tube",
    )
    ctx.expect_overlap(
        upper,
        middle,
        axes="z",
        elem_a="upper_tube",
        elem_b="middle_tube",
        min_overlap=0.100,
        name="upper stage retained in middle tube",
    )
    ctx.expect_gap(
        pan_collar,
        upper,
        axis="z",
        positive_elem="collar_ring",
        negative_elem="thrust_washer",
        max_gap=0.001,
        max_penetration=0.0,
        name="pan collar rests on the thrust washer",
    )

    rest_middle = ctx.part_world_position(middle)
    rest_upper = ctx.part_world_position(upper)
    with ctx.pose({base_slide: 0.320, upper_slide: 0.300, pan: math.pi / 2.0}):
        ctx.expect_overlap(
            middle,
            base,
            axes="z",
            elem_a="middle_tube",
            elem_b="outer_sleeve",
            min_overlap=0.050,
            name="extended middle still retained",
        )
        ctx.expect_overlap(
            upper,
            middle,
            axes="z",
            elem_a="upper_tube",
            elem_b="middle_tube",
            min_overlap=0.050,
            name="extended upper still retained",
        )
        extended_middle = ctx.part_world_position(middle)
        extended_upper = ctx.part_world_position(upper)

    ctx.check(
        "prismatic joints extend upward",
        rest_middle is not None
        and rest_upper is not None
        and extended_middle is not None
        and extended_upper is not None
        and extended_middle[2] > rest_middle[2] + 0.25
        and extended_upper[2] > rest_upper[2] + 0.55,
        details=(
            f"middle rest={rest_middle}, middle ext={extended_middle}, "
            f"upper rest={rest_upper}, upper ext={extended_upper}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
