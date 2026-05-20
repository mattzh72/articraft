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
)


def _add_square_tube(part, *, prefix: str, outer: float, wall: float, z_min: float, z_max: float, material: str) -> None:
    """Add a connected, visibly hollow square telescoping tube."""
    height = z_max - z_min
    z = (z_min + z_max) * 0.5
    half = outer * 0.5
    w2 = wall * 0.5

    # Four wall plates overlap at the corners so the authored part is one
    # continuous fabricated tube rather than four floating strips.
    part.visual(
        Box((outer, wall, height)),
        origin=Origin(xyz=(0.0, half - w2, z)),
        material=material,
        name=f"{prefix}_front_wall",
    )
    part.visual(
        Box((outer, wall, height)),
        origin=Origin(xyz=(0.0, -half + w2, z)),
        material=material,
        name=f"{prefix}_rear_wall",
    )
    part.visual(
        Box((wall, outer, height)),
        origin=Origin(xyz=(half - w2, 0.0, z)),
        material=material,
        name=f"{prefix}_side_wall_0",
    )
    part.visual(
        Box((wall, outer, height)),
        origin=Origin(xyz=(-half + w2, 0.0, z)),
        material=material,
        name=f"{prefix}_side_wall_1",
    )


def _add_external_collar(part, *, prefix: str, outer: float, collar: float, z_center: float, height: float, material: str) -> None:
    """Add an outside clamp collar that leaves the stage bore clear."""
    half = outer * 0.5
    c2 = collar * 0.5
    span = outer + 2.0 * collar
    part.visual(
        Box((span, collar, height)),
        origin=Origin(xyz=(0.0, half + c2, z_center)),
        material=material,
        name=f"{prefix}_front_clamp",
    )
    part.visual(
        Box((span, collar, height)),
        origin=Origin(xyz=(0.0, -half - c2, z_center)),
        material=material,
        name=f"{prefix}_rear_clamp",
    )
    part.visual(
        Box((collar, span, height)),
        origin=Origin(xyz=(half + c2, 0.0, z_center)),
        material=material,
        name=f"{prefix}_side_clamp_0",
    )
    part.visual(
        Box((collar, span, height)),
        origin=Origin(xyz=(-half - c2, 0.0, z_center)),
        material=material,
        name=f"{prefix}_side_clamp_1",
    )


def _add_glide_pads(
    part,
    *,
    prefix: str,
    tube_outer: float,
    pad_thickness: float,
    z_center: float,
    height: float,
    material: str,
) -> None:
    """Low-friction shoes that touch the receiving sleeve and make the slide supported."""
    half = tube_outer * 0.5
    p2 = pad_thickness * 0.5
    short = tube_outer * 0.38
    part.visual(
        Box((pad_thickness, short, height)),
        origin=Origin(xyz=(half + p2, 0.0, z_center)),
        material=material,
        name=f"{prefix}_side_pad_0",
    )
    part.visual(
        Box((pad_thickness, short, height)),
        origin=Origin(xyz=(-half - p2, 0.0, z_center)),
        material=material,
        name=f"{prefix}_side_pad_1",
    )
    part.visual(
        Box((short, pad_thickness, height)),
        origin=Origin(xyz=(0.0, half + p2, z_center)),
        material=material,
        name=f"{prefix}_front_pad",
    )
    part.visual(
        Box((short, pad_thickness, height)),
        origin=Origin(xyz=(0.0, -half - p2, z_center)),
        material=material,
        name=f"{prefix}_rear_pad",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_backed_telescoping_mast")

    model.material("black_powder_coat", rgba=(0.025, 0.028, 0.030, 1.0))
    model.material("dark_steel", rgba=(0.11, 0.12, 0.13, 1.0))
    model.material("zinc_pin", rgba=(0.62, 0.64, 0.62, 1.0))
    model.material("blue_anodized", rgba=(0.05, 0.18, 0.34, 1.0))
    model.material("brushed_aluminum", rgba=(0.70, 0.72, 0.70, 1.0))
    model.material("orange_faceplate", rgba=(0.95, 0.43, 0.08, 1.0))
    model.material("rubber_foot", rgba=(0.015, 0.014, 0.013, 1.0))
    model.material("black_acetal", rgba=(0.02, 0.021, 0.022, 1.0))

    rear_support = model.part("rear_support")
    # Grounded base with four feet.
    rear_support.visual(
        Box((0.46, 0.34, 0.025)),
        origin=Origin(xyz=(0.0, -0.015, 0.0125)),
        material="black_powder_coat",
        name="base_plate",
    )
    for ix, x in enumerate((-0.18, 0.18)):
        for iy, y in enumerate((-0.145, 0.105)):
            rear_support.visual(
                Cylinder(radius=0.028, length=0.010),
                origin=Origin(xyz=(x, y, -0.005)),
                material="rubber_foot",
                name=f"rubber_foot_{ix}_{iy}",
            )

    # A rear fork wraps the mast root: two cheeks tied by a back plate.
    rear_support.visual(
        Box((0.20, 0.022, 0.43)),
        origin=Origin(xyz=(0.0, -0.066, 0.240)),
        material="black_powder_coat",
        name="rear_backplate",
    )
    rear_support.visual(
        Box((0.014, 0.145, 0.42)),
        origin=Origin(xyz=(0.068, -0.004, 0.235)),
        material="black_powder_coat",
        name="fork_cheek_0",
    )
    rear_support.visual(
        Box((0.014, 0.145, 0.42)),
        origin=Origin(xyz=(-0.068, -0.004, 0.235)),
        material="black_powder_coat",
        name="fork_cheek_1",
    )
    rear_support.visual(
        Box((0.012, 0.135, 0.018)),
        origin=Origin(xyz=(0.067, -0.005, 0.034)),
        material="dark_steel",
        name="saddle_rail_0",
    )
    rear_support.visual(
        Box((0.012, 0.135, 0.018)),
        origin=Origin(xyz=(-0.067, -0.005, 0.034)),
        material="dark_steel",
        name="saddle_rail_1",
    )
    rear_support.visual(
        Cylinder(radius=0.009, length=0.185),
        origin=Origin(xyz=(0.0, -0.068, 0.250), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="zinc_pin",
        name="capture_pin",
    )
    rear_support.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.098, -0.068, 0.250), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="zinc_pin",
        name="pin_head_0",
    )
    rear_support.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(-0.098, -0.068, 0.250), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="zinc_pin",
        name="pin_head_1",
    )

    outer_stage = model.part("outer_stage")
    outer_height = 0.780
    outer_stage.visual(
        Box((0.110, 0.012, outer_height)),
        origin=Origin(xyz=(0.0, 0.049, outer_height * 0.5)),
        material="dark_steel",
        name="outer_front_wall",
    )
    outer_stage.visual(
        Box((0.110, 0.012, outer_height)),
        origin=Origin(xyz=(0.0, -0.049, outer_height * 0.5)),
        material="dark_steel",
        name="outer_rear_wall",
    )
    outer_stage.visual(
        Box((0.012, 0.110, outer_height)),
        origin=Origin(xyz=(0.049, 0.0, outer_height * 0.5)),
        material="dark_steel",
        name="outer_side_wall_0",
    )
    outer_stage.visual(
        Box((0.012, 0.110, outer_height)),
        origin=Origin(xyz=(-0.049, 0.0, outer_height * 0.5)),
        material="dark_steel",
        name="outer_side_wall_1",
    )
    _add_external_collar(
        outer_stage,
        prefix="outer_top",
        outer=0.110,
        collar=0.017,
        z_center=outer_height - 0.027,
        height=0.054,
        material="black_powder_coat",
    )
    outer_stage.visual(
        Box((0.100, 0.006, 0.21)),
        origin=Origin(xyz=(0.0, -0.052, 0.145)),
        material="black_powder_coat",
        name="rear_wear_plate",
    )

    middle_stage = model.part("middle_stage")
    middle_bottom = -0.580
    middle_top = 0.480
    _add_square_tube(
        middle_stage,
        prefix="middle",
        outer=0.074,
        wall=0.008,
        z_min=middle_bottom,
        z_max=middle_top,
        material="blue_anodized",
    )
    _add_external_collar(
        middle_stage,
        prefix="middle_top",
        outer=0.074,
        collar=0.012,
        z_center=middle_top - 0.022,
        height=0.044,
        material="black_powder_coat",
    )
    _add_glide_pads(
        middle_stage,
        prefix="middle_lower",
        tube_outer=0.074,
        pad_thickness=0.006,
        z_center=-0.520,
        height=0.100,
        material="black_acetal",
    )

    inner_stage = model.part("inner_stage")
    inner_bottom = -0.450
    inner_top = 0.420
    _add_square_tube(
        inner_stage,
        prefix="inner",
        outer=0.050,
        wall=0.006,
        z_min=inner_bottom,
        z_max=inner_top,
        material="brushed_aluminum",
    )
    inner_stage.visual(
        Box((0.070, 0.010, 0.036)),
        origin=Origin(xyz=(0.0, 0.030, inner_top - 0.018)),
        material="dark_steel",
        name="inner_top_front_clamp",
    )
    inner_stage.visual(
        Box((0.070, 0.010, 0.036)),
        origin=Origin(xyz=(0.0, -0.030, inner_top - 0.018)),
        material="dark_steel",
        name="inner_top_rear_clamp",
    )
    inner_stage.visual(
        Box((0.010, 0.070, 0.036)),
        origin=Origin(xyz=(0.030, 0.0, inner_top - 0.018)),
        material="dark_steel",
        name="inner_top_side_clamp_0",
    )
    inner_stage.visual(
        Box((0.010, 0.070, 0.036)),
        origin=Origin(xyz=(-0.030, 0.0, inner_top - 0.018)),
        material="dark_steel",
        name="inner_top_side_clamp_1",
    )
    _add_glide_pads(
        inner_stage,
        prefix="inner_lower",
        tube_outer=0.050,
        pad_thickness=0.004,
        z_center=-0.390,
        height=0.080,
        material="black_acetal",
    )

    faceplate = model.part("faceplate")
    faceplate.visual(
        Cylinder(radius=0.092, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material="orange_faceplate",
        name="rotary_plate",
    )
    faceplate.visual(
        Cylinder(radius=0.030, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material="zinc_pin",
        name="center_hub",
    )
    faceplate.visual(
        Box((0.030, 0.070, 0.010)),
        origin=Origin(xyz=(0.0, 0.105, 0.023)),
        material="orange_faceplate",
        name="index_tab",
    )
    for i, (x, y) in enumerate(((0.052, 0.052), (-0.052, 0.052), (-0.052, -0.052), (0.052, -0.052))):
        faceplate.visual(
            Cylinder(radius=0.006, length=0.008),
            origin=Origin(xyz=(x, y, 0.022)),
            material="zinc_pin",
            name=f"bolt_boss_{i}",
        )

    model.articulation(
        "support_to_outer",
        ArticulationType.FIXED,
        parent=rear_support,
        child=outer_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )
    model.articulation(
        "outer_slide",
        ArticulationType.PRISMATIC,
        parent=outer_stage,
        child=middle_stage,
        origin=Origin(xyz=(0.0, 0.0, outer_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.18, lower=0.0, upper=0.400),
    )
    model.articulation(
        "middle_slide",
        ArticulationType.PRISMATIC,
        parent=middle_stage,
        child=inner_stage,
        origin=Origin(xyz=(0.0, 0.0, middle_top)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.16, lower=0.0, upper=0.320),
    )
    model.articulation(
        "top_rotor",
        ArticulationType.REVOLUTE,
        parent=inner_stage,
        child=faceplate,
        origin=Origin(xyz=(0.0, 0.0, inner_top)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rear_support = object_model.get_part("rear_support")
    outer_stage = object_model.get_part("outer_stage")
    middle_stage = object_model.get_part("middle_stage")
    inner_stage = object_model.get_part("inner_stage")
    faceplate = object_model.get_part("faceplate")

    outer_slide = object_model.get_articulation("outer_slide")
    middle_slide = object_model.get_articulation("middle_slide")
    top_rotor = object_model.get_articulation("top_rotor")

    # The rear fork must actually straddle and capture the mast root.
    ctx.expect_contact(
        outer_stage,
        rear_support,
        elem_a="outer_rear_wall",
        elem_b="rear_backplate",
        name="rear fork backplate contacts outer mast root",
    )
    ctx.expect_gap(
        rear_support,
        outer_stage,
        axis="x",
        positive_elem="fork_cheek_0",
        negative_elem="outer_side_wall_0",
        min_gap=0.002,
        max_gap=0.010,
        name="positive fork cheek clears mast root",
    )
    ctx.expect_gap(
        outer_stage,
        rear_support,
        axis="x",
        positive_elem="outer_side_wall_1",
        negative_elem="fork_cheek_1",
        min_gap=0.002,
        max_gap=0.010,
        name="negative fork cheek clears mast root",
    )

    # Nested stages are centered in plan and remain inserted at rest.
    ctx.expect_within(
        middle_stage,
        outer_stage,
        axes="xy",
        margin=0.0,
        name="middle stage fits inside outer sleeve footprint",
    )
    ctx.expect_overlap(
        middle_stage,
        outer_stage,
        axes="z",
        min_overlap=0.50,
        name="middle stage has deep retained insertion at rest",
    )
    ctx.expect_within(
        inner_stage,
        middle_stage,
        axes="xy",
        margin=0.0,
        name="inner stage fits inside middle sleeve footprint",
    )
    ctx.expect_overlap(
        inner_stage,
        middle_stage,
        axes="z",
        min_overlap=0.40,
        name="inner stage has retained insertion at rest",
    )

    rest_faceplate_pos = ctx.part_world_position(faceplate)
    with ctx.pose({outer_slide: 0.400, middle_slide: 0.320}):
        ctx.expect_within(
            middle_stage,
            outer_stage,
            axes="xy",
            margin=0.0,
            name="extended middle stage remains centered in outer sleeve",
        )
        ctx.expect_overlap(
            middle_stage,
            outer_stage,
            axes="z",
            min_overlap=0.16,
            name="extended middle stage remains retained in outer sleeve",
        )
        ctx.expect_within(
            inner_stage,
            middle_stage,
            axes="xy",
            margin=0.0,
            name="extended inner stage remains centered in middle sleeve",
        )
        ctx.expect_overlap(
            inner_stage,
            middle_stage,
            axes="z",
            min_overlap=0.11,
            name="extended inner stage remains retained in middle sleeve",
        )
        extended_faceplate_pos = ctx.part_world_position(faceplate)

    ctx.check(
        "serial prismatic joints extend faceplate upward",
        rest_faceplate_pos is not None
        and extended_faceplate_pos is not None
        and extended_faceplate_pos[2] > rest_faceplate_pos[2] + 0.65,
        details=f"rest={rest_faceplate_pos}, extended={extended_faceplate_pos}",
    )

    ctx.expect_contact(
        faceplate,
        inner_stage,
        elem_a="rotary_plate",
        elem_b="inner_top_front_clamp",
        name="rotary faceplate sits on top bearing collar",
    )
    rest_rotor_pos = ctx.part_world_position(faceplate)
    with ctx.pose({top_rotor: 1.20}):
        spun_rotor_pos = ctx.part_world_position(faceplate)
    ctx.check(
        "faceplate revolute joint spins without translating",
        rest_rotor_pos is not None
        and spun_rotor_pos is not None
        and abs(rest_rotor_pos[0] - spun_rotor_pos[0]) < 1e-6
        and abs(rest_rotor_pos[1] - spun_rotor_pos[1]) < 1e-6
        and abs(rest_rotor_pos[2] - spun_rotor_pos[2]) < 1e-6,
        details=f"rest={rest_rotor_pos}, spun={spun_rotor_pos}",
    )

    return ctx.report()


object_model = build_object_model()
