from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


LOWER_TO_MIDDLE_TRAVEL = 0.36
MIDDLE_TO_UPPER_TRAVEL = 0.28
YAW_QUARTER_TURN = math.pi / 2.0


def _hollow_cylinder_geometry(
    *,
    outer_radius: float,
    inner_radius: float,
    z_min: float,
    z_max: float,
    segments: int = 64,
) -> MeshGeometry:
    """Closed annular tube with an open center bore."""
    geom = MeshGeometry()
    outer_bottom: list[int] = []
    outer_top: list[int] = []
    inner_bottom: list[int] = []
    inner_top: list[int] = []

    for i in range(segments):
        angle = 2.0 * math.pi * i / segments
        c = math.cos(angle)
        s = math.sin(angle)
        outer_bottom.append(geom.add_vertex(outer_radius * c, outer_radius * s, z_min))
        outer_top.append(geom.add_vertex(outer_radius * c, outer_radius * s, z_max))
        inner_bottom.append(geom.add_vertex(inner_radius * c, inner_radius * s, z_min))
        inner_top.append(geom.add_vertex(inner_radius * c, inner_radius * s, z_max))

    for i in range(segments):
        j = (i + 1) % segments

        # Outer wall.
        geom.add_face(outer_bottom[i], outer_bottom[j], outer_top[j])
        geom.add_face(outer_bottom[i], outer_top[j], outer_top[i])

        # Inner bore wall.
        geom.add_face(inner_bottom[i], inner_top[j], inner_bottom[j])
        geom.add_face(inner_bottom[i], inner_top[i], inner_top[j])

        # Top annular face.
        geom.add_face(outer_top[i], outer_top[j], inner_top[j])
        geom.add_face(outer_top[i], inner_top[j], inner_top[i])

        # Bottom annular face.
        geom.add_face(outer_bottom[i], inner_bottom[j], outer_bottom[j])
        geom.add_face(outer_bottom[i], inner_bottom[i], inner_bottom[j])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_mast_pan_cartridge")

    dark_steel = model.material("dark_steel", rgba=(0.14, 0.15, 0.16, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.28, 0.30, 0.32, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.70, 0.72, 0.74, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.86, 0.87, 0.84, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.035, 0.035, 0.038, 1.0))
    amber_mark = model.material("amber_mark", rgba=(0.95, 0.58, 0.16, 1.0))

    lower_sleeve = model.part("lower_sleeve")
    lower_sleeve.visual(
        Box((0.34, 0.34, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark_steel,
        name="floor_plate",
    )
    lower_sleeve.visual(
        mesh_from_geometry(
            _hollow_cylinder_geometry(
                outer_radius=0.055,
                inner_radius=0.045,
                z_min=0.040,
                z_max=0.800,
            ),
            "lower_sleeve_tube",
        ),
        material=gunmetal,
        name="lower_tube",
    )
    lower_sleeve.visual(
        mesh_from_geometry(
            _hollow_cylinder_geometry(
                outer_radius=0.075,
                inner_radius=0.045,
                z_min=0.700,
                z_max=0.815,
            ),
            "lower_sleeve_collar",
        ),
        material=dark_steel,
        name="top_collar",
    )
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        c = math.cos(angle)
        s = math.sin(angle)
        lower_sleeve.visual(
            Box((0.115, 0.018, 0.150)),
            origin=Origin(
                xyz=(0.105 * c, 0.105 * s, 0.115),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_steel,
            name=f"base_rib_{index}",
        )
        lower_sleeve.visual(
            Cylinder(radius=0.013, length=0.012),
            origin=Origin(xyz=(0.118 * c, 0.118 * s, 0.046)),
            material=gunmetal,
            name=f"bolt_head_{index}",
        )
    lower_sleeve.visual(
        Cylinder(radius=0.008, length=0.080),
        origin=Origin(xyz=(0.092, 0.0, 0.760), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_aluminum,
        name="clamp_screw",
    )
    lower_sleeve.visual(
        Sphere(radius=0.021),
        origin=Origin(xyz=(0.140, 0.0, 0.760)),
        material=black_rubber,
        name="clamp_knob",
    )
    lower_sleeve.visual(
        Box((0.010, 0.018, 0.260)),
        origin=Origin(xyz=(0.056, 0.0, 0.420)),
        material=amber_mark,
        name="height_scale",
    )

    middle_mast = model.part("middle_mast")
    middle_mast.visual(
        mesh_from_geometry(
            _hollow_cylinder_geometry(
                outer_radius=0.038,
                inner_radius=0.028,
                z_min=-0.660,
                z_max=0.420,
            ),
            "middle_mast_tube",
        ),
        material=brushed_aluminum,
        name="middle_tube",
    )
    middle_mast.visual(
        mesh_from_geometry(
            _hollow_cylinder_geometry(
                outer_radius=0.0455,
                inner_radius=0.028,
                z_min=-0.660,
                z_max=-0.610,
            ),
            "middle_mast_guide_band",
        ),
        material=gunmetal,
        name="guide_band",
    )
    middle_mast.visual(
        mesh_from_geometry(
            _hollow_cylinder_geometry(
                outer_radius=0.048,
                inner_radius=0.028,
                z_min=0.420,
                z_max=0.500,
            ),
            "middle_mast_collar",
        ),
        material=gunmetal,
        name="top_collar",
    )
    middle_mast.visual(
        Box((0.004, 0.012, 0.500)),
        origin=Origin(xyz=(0.040, 0.0, 0.080)),
        material=amber_mark,
        name="index_strip",
    )
    middle_mast.visual(
        Cylinder(radius=0.0065, length=0.060),
        origin=Origin(xyz=(0.060, 0.0, 0.462), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_aluminum,
        name="clamp_screw",
    )
    middle_mast.visual(
        Sphere(radius=0.016),
        origin=Origin(xyz=(0.096, 0.0, 0.462)),
        material=black_rubber,
        name="clamp_knob",
    )

    upper_mast = model.part("upper_mast")
    upper_mast.visual(
        Cylinder(radius=0.022, length=0.960),
        origin=Origin(xyz=(0.0, 0.0, -0.140)),
        material=satin_aluminum,
        name="upper_tube",
    )
    upper_mast.visual(
        mesh_from_geometry(
            _hollow_cylinder_geometry(
                outer_radius=0.0285,
                inner_radius=0.020,
                z_min=-0.600,
                z_max=-0.540,
            ),
            "upper_mast_bushing",
        ),
        material=black_rubber,
        name="lower_bushing",
    )
    upper_mast.visual(
        Box((0.003, 0.008, 0.560)),
        origin=Origin(xyz=(0.023, 0.0, -0.055)),
        material=amber_mark,
        name="index_strip",
    )
    upper_mast.visual(
        Cylinder(radius=0.033, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.365)),
        material=gunmetal,
        name="yaw_stator",
    )

    pan_cartridge = model.part("pan_cartridge")
    pan_cartridge.visual(
        Cylinder(radius=0.045, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark_steel,
        name="rotor_disc",
    )
    pan_cartridge.visual(
        Cylinder(radius=0.075, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=gunmetal,
        name="cartridge_drum",
    )
    pan_cartridge.visual(
        Cylinder(radius=0.064, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.132)),
        material=black_rubber,
        name="top_seal",
    )
    pan_cartridge.visual(
        Box((0.085, 0.052, 0.060)),
        origin=Origin(xyz=(0.090, 0.0, 0.088)),
        material=dark_steel,
        name="front_boss",
    )
    pan_cartridge.visual(
        Cylinder(radius=0.012, length=0.040),
        origin=Origin(xyz=(-0.094, 0.0, 0.087), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_rubber,
        name="cable_gland",
    )

    model.articulation(
        "lower_to_middle",
        ArticulationType.PRISMATIC,
        parent=lower_sleeve,
        child=middle_mast,
        origin=Origin(xyz=(0.0, 0.0, 0.800)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=160.0,
            velocity=0.18,
            lower=0.0,
            upper=LOWER_TO_MIDDLE_TRAVEL,
        ),
    )
    model.articulation(
        "middle_to_upper",
        ArticulationType.PRISMATIC,
        parent=middle_mast,
        child=upper_mast,
        origin=Origin(xyz=(0.0, 0.0, 0.500)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=110.0,
            velocity=0.16,
            lower=0.0,
            upper=MIDDLE_TO_UPPER_TRAVEL,
        ),
    )
    model.articulation(
        "upper_to_cartridge",
        ArticulationType.REVOLUTE,
        parent=upper_mast,
        child=pan_cartridge,
        origin=Origin(xyz=(0.0, 0.0, 0.390)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_sleeve = object_model.get_part("lower_sleeve")
    middle_mast = object_model.get_part("middle_mast")
    upper_mast = object_model.get_part("upper_mast")
    pan_cartridge = object_model.get_part("pan_cartridge")
    lower_to_middle = object_model.get_articulation("lower_to_middle")
    middle_to_upper = object_model.get_articulation("middle_to_upper")
    upper_to_cartridge = object_model.get_articulation("upper_to_cartridge")

    ctx.allow_overlap(
        lower_sleeve,
        middle_mast,
        elem_a="lower_tube",
        elem_b="guide_band",
        reason=(
            "The lower nylon guide band is intentionally modeled with a slight "
            "preload into the sleeve bore so the telescoping member is visibly supported."
        ),
    )
    ctx.allow_overlap(
        middle_mast,
        upper_mast,
        elem_a="middle_tube",
        elem_b="lower_bushing",
        reason=(
            "The upper mast bushing intentionally bears lightly against the middle sleeve "
            "bore to represent the sliding guide contact."
        ),
    )

    ctx.expect_within(
        middle_mast,
        lower_sleeve,
        axes="xy",
        inner_elem="middle_tube",
        outer_elem="lower_tube",
        margin=0.0,
        name="middle mast centered in lower sleeve",
    )
    ctx.expect_overlap(
        middle_mast,
        lower_sleeve,
        axes="z",
        elem_a="middle_tube",
        elem_b="lower_tube",
        min_overlap=0.28,
        name="collapsed middle mast remains inserted",
    )
    ctx.expect_overlap(
        middle_mast,
        lower_sleeve,
        axes="z",
        elem_a="guide_band",
        elem_b="lower_tube",
        min_overlap=0.040,
        name="lower guide band bears in sleeve",
    )
    ctx.expect_within(
        upper_mast,
        middle_mast,
        axes="xy",
        inner_elem="upper_tube",
        outer_elem="middle_tube",
        margin=0.0,
        name="upper mast centered in middle sleeve",
    )
    ctx.expect_overlap(
        upper_mast,
        middle_mast,
        axes="z",
        elem_a="upper_tube",
        elem_b="middle_tube",
        min_overlap=0.34,
        name="collapsed upper mast remains inserted",
    )
    ctx.expect_overlap(
        upper_mast,
        middle_mast,
        axes="z",
        elem_a="lower_bushing",
        elem_b="middle_tube",
        min_overlap=0.050,
        name="upper bushing bears in middle sleeve",
    )
    ctx.expect_gap(
        pan_cartridge,
        upper_mast,
        axis="z",
        positive_elem="rotor_disc",
        negative_elem="yaw_stator",
        max_gap=0.001,
        max_penetration=0.0,
        name="rotary cartridge seats on yaw stator",
    )

    middle_rest = ctx.part_world_position(middle_mast)
    with ctx.pose({lower_to_middle: LOWER_TO_MIDDLE_TRAVEL}):
        ctx.expect_overlap(
            middle_mast,
            lower_sleeve,
            axes="z",
            elem_a="middle_tube",
            elem_b="lower_tube",
            min_overlap=0.24,
            name="extended middle mast retains lower insertion",
        )
        middle_extended = ctx.part_world_position(middle_mast)
    ctx.check(
        "middle mast extends vertically",
        middle_rest is not None
        and middle_extended is not None
        and middle_extended[2] > middle_rest[2] + 0.30,
        details=f"rest={middle_rest}, extended={middle_extended}",
    )

    upper_rest = ctx.part_world_position(upper_mast)
    with ctx.pose({middle_to_upper: MIDDLE_TO_UPPER_TRAVEL}):
        ctx.expect_overlap(
            upper_mast,
            middle_mast,
            axes="z",
            elem_a="upper_tube",
            elem_b="middle_tube",
            min_overlap=0.22,
            name="extended upper mast retains middle insertion",
        )
        upper_extended = ctx.part_world_position(upper_mast)
    ctx.check(
        "upper mast extends vertically",
        upper_rest is not None
        and upper_extended is not None
        and upper_extended[2] > upper_rest[2] + 0.22,
        details=f"rest={upper_rest}, extended={upper_extended}",
    )

    boss_rest_aabb = ctx.part_element_world_aabb(pan_cartridge, elem="front_boss")
    with ctx.pose({upper_to_cartridge: YAW_QUARTER_TURN}):
        boss_yawed_aabb = ctx.part_element_world_aabb(pan_cartridge, elem="front_boss")

    def _center(aabb, axis_index: int) -> float | None:
        if aabb is None:
            return None
        return 0.5 * (aabb[0][axis_index] + aabb[1][axis_index])

    ctx.check(
        "cartridge yaws about mast centerline",
        _center(boss_rest_aabb, 0) is not None
        and _center(boss_yawed_aabb, 1) is not None
        and _center(boss_rest_aabb, 0) > 0.08
        and _center(boss_yawed_aabb, 1) > 0.08,
        details=f"rest={boss_rest_aabb}, yawed={boss_yawed_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
