from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


WOOD = "warm_cherry_wood"
BRASS = "brushed_brass"
AGED_BRASS = "aged_brass"
BLACK = "black_enamel"
STEEL = "polished_steel"
SCALE = 0.55


def _s(value: float) -> float:
    return value * SCALE


def _scaled(values: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple(_s(v) for v in values)


def _tapered_housing_shape() -> cq.Workplane:
    """Pyramidal metronome case body, authored in final meter units."""
    return (
        cq.Workplane("XY")
        .workplane(offset=0.030)
        .rect(0.220, 0.145)
        .workplane(offset=0.330)
        .rect(0.082, 0.058)
        .loft(combine=True)
    )


def _pendulum_shape() -> cq.Workplane:
    """One metal pendulum part: bored pivot collar fused to the slender rod."""
    collar = (
        cq.Workplane("YZ")
        .circle(0.012)
        .circle(0.0076)
        .extrude(0.038)
        .translate((-0.019, 0.0, 0.0))
    )
    rod = cq.Workplane("XY").circle(0.0035).extrude(0.555).translate((0.0, 0.0, 0.009))
    pointer_tip = cq.Workplane("XY").circle(0.006).extrude(0.010).translate((0.0, 0.0, 0.560))
    return collar.union(rod).union(pointer_tip)


def _weight_shape(outer_radius: float, bore_radius: float, height: float) -> cq.Workplane:
    """Cylindrical sliding weight with a real through-bore for the pendulum rod."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(bore_radius)
        .extrude(height)
        .translate((0.0, 0.0, -height / 2.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pyramid_metronome_dual_weights")
    model.material(WOOD, rgba=(0.55, 0.28, 0.12, 1.0))
    model.material(BRASS, rgba=(0.95, 0.70, 0.28, 1.0))
    model.material(AGED_BRASS, rgba=(0.58, 0.42, 0.18, 1.0))
    model.material(BLACK, rgba=(0.005, 0.005, 0.006, 1.0))
    model.material(STEEL, rgba=(0.70, 0.72, 0.72, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box(_scaled((0.300, 0.205, 0.030))),
        origin=Origin(xyz=_scaled((0.0, 0.0, 0.015))),
        material=WOOD,
        name="rectangular_base",
    )
    housing.visual(
        mesh_from_cadquery(
            _tapered_housing_shape(),
            "tapered_wood_housing",
            tolerance=0.0008,
            unit_scale=SCALE,
        ),
        material=WOOD,
        name="tapered_case",
    )
    housing.visual(
        Box(_scaled((0.104, 0.078, 0.012))),
        origin=Origin(xyz=_scaled((0.0, 0.0, 0.366))),
        material=WOOD,
        name="top_cap",
    )

    # Sloped brass tempo scale on the front face, with individual black tick marks.
    scale_angle = -math.atan((0.145 - 0.058) / (2.0 * 0.330))
    scale_center_z = _s(0.214)
    scale_face_y = _s(-0.145 / 2.0 + ((0.145 - 0.058) / (2.0 * 0.330)) * (0.214 - 0.030))
    housing.visual(
        Box(_scaled((0.030, 0.003, 0.205))),
        origin=Origin(xyz=(_s(-0.038), scale_face_y - _s(0.0008), scale_center_z), rpy=(scale_angle, 0.0, 0.0)),
        material=BRASS,
        name="tempo_scale",
    )
    for i, z_unscaled in enumerate((0.130, 0.160, 0.190, 0.220, 0.250, 0.280)):
        z = _s(z_unscaled)
        face_y = _s(-0.145 / 2.0 + ((0.145 - 0.058) / (2.0 * 0.330)) * (z_unscaled - 0.030))
        tick_width = _s(0.023 if i % 2 == 0 else 0.016)
        housing.visual(
            Box((tick_width, _s(0.0022), _s(0.0032))),
            origin=Origin(xyz=(_s(-0.038), face_y - _s(0.0020), z), rpy=(scale_angle, 0.0, 0.0)),
            material=BLACK,
            name=f"tempo_tick_{i}",
        )

    # Housing-top saddle and shaft that carry the pendulum pivot.
    pivot_xyz = _scaled((0.0, -0.012, 0.385))
    for x, name in ((-0.032, "pivot_cheek_0"), (0.032, "pivot_cheek_1")):
        housing.visual(
            Box(_scaled((0.007, 0.022, 0.022))),
            origin=Origin(xyz=(_s(x), pivot_xyz[1], _s(0.383))),
            material=BRASS,
            name=name,
        )
    housing.visual(
        Cylinder(radius=_s(0.0078), length=_s(0.092)),
        origin=Origin(xyz=pivot_xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=STEEL,
        name="pivot_shaft",
    )

    # Rear winding-key bushing on the sloped rear wall.
    rear_z = _s(0.180)
    rear_face_y = _s(0.145 / 2.0 - ((0.145 - 0.058) / (2.0 * 0.330)) * (0.180 - 0.030))
    housing.visual(
        Cylinder(radius=_s(0.010), length=_s(0.006)),
        origin=Origin(xyz=(0.0, rear_face_y + _s(0.003), rear_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=BRASS,
        name="key_bushing",
    )

    pendulum = model.part("pendulum_rod")
    pendulum.visual(
        mesh_from_cadquery(_pendulum_shape(), "pendulum_rod_with_pivot", tolerance=0.0004, unit_scale=SCALE),
        material=STEEL,
        name="rod_with_pivot",
    )

    coarse_weight = model.part("coarse_weight")
    coarse_weight.visual(
        mesh_from_cadquery(
            _weight_shape(0.027, 0.0033, 0.047),
            "coarse_sliding_weight",
            tolerance=0.0004,
            unit_scale=SCALE,
        ),
        material=AGED_BRASS,
        name="coarse_body",
    )

    fine_weight = model.part("fine_weight")
    fine_weight.visual(
        mesh_from_cadquery(
            _weight_shape(0.018, 0.0033, 0.032),
            "fine_sliding_weight",
            tolerance=0.0004,
            unit_scale=SCALE,
        ),
        material=BRASS,
        name="fine_body",
    )

    winding_key = model.part("winding_key")
    winding_key.visual(
        Cylinder(radius=_s(0.005), length=_s(0.040)),
        origin=Origin(xyz=_scaled((0.0, 0.020, 0.0)), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=AGED_BRASS,
        name="key_shaft",
    )
    winding_key.visual(
        Cylinder(radius=_s(0.012), length=_s(0.012)),
        origin=Origin(xyz=_scaled((0.0, 0.046, 0.0)), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=AGED_BRASS,
        name="key_hub",
    )
    winding_key.visual(
        Box(_scaled((0.070, 0.010, 0.018))),
        origin=Origin(xyz=_scaled((0.0, 0.052, 0.0))),
        material=AGED_BRASS,
        name="key_bar",
    )
    for x, name in ((-0.032, "key_lobe_0"), (0.032, "key_lobe_1")):
        winding_key.visual(
            Cylinder(radius=_s(0.014), length=_s(0.010)),
            origin=Origin(xyz=_scaled((x, 0.052, 0.0)), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=AGED_BRASS,
            name=name,
        )

    model.articulation(
        "housing_to_pendulum",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=pendulum,
        origin=Origin(xyz=pivot_xyz),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=5.0, lower=-0.36, upper=0.36),
    )
    model.articulation(
        "pendulum_to_coarse_weight",
        ArticulationType.PRISMATIC,
        parent=pendulum,
        child=coarse_weight,
        origin=Origin(xyz=_scaled((0.0, 0.0, 0.170))),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.12, lower=_s(-0.060), upper=_s(0.080)),
    )
    model.articulation(
        "pendulum_to_fine_weight",
        ArticulationType.PRISMATIC,
        parent=pendulum,
        child=fine_weight,
        origin=Origin(xyz=_scaled((0.0, 0.0, 0.380))),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.10, lower=_s(-0.070), upper=_s(0.120)),
    )
    model.articulation(
        "housing_to_winding_key",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=winding_key,
        origin=Origin(xyz=(0.0, rear_face_y + _s(0.006), rear_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.15, velocity=10.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pendulum = object_model.get_part("pendulum_rod")
    coarse = object_model.get_part("coarse_weight")
    fine = object_model.get_part("fine_weight")
    key = object_model.get_part("winding_key")
    swing = object_model.get_articulation("housing_to_pendulum")
    coarse_slide = object_model.get_articulation("pendulum_to_coarse_weight")
    fine_slide = object_model.get_articulation("pendulum_to_fine_weight")
    key_spin = object_model.get_articulation("housing_to_winding_key")

    ctx.allow_overlap(
        "housing",
        "pendulum_rod",
        elem_a="pivot_shaft",
        elem_b="rod_with_pivot",
        reason="The visible pivot collar is a captured bearing around the housing-top shaft, modeled with a tiny interference fit.",
    )
    ctx.allow_overlap(
        "pendulum_rod",
        "coarse_weight",
        elem_a="rod_with_pivot",
        elem_b="coarse_body",
        reason="The coarse tempo weight is a friction collar sliding on the rod, represented by slight hidden bore compression.",
    )
    ctx.allow_overlap(
        "pendulum_rod",
        "fine_weight",
        elem_a="rod_with_pivot",
        elem_b="fine_body",
        reason="The fine tempo weight is a small friction collar sliding on the rod, represented by slight hidden bore compression.",
    )
    ctx.expect_overlap(
        "housing",
        "pendulum_rod",
        axes="x",
        elem_a="pivot_shaft",
        elem_b="rod_with_pivot",
        min_overlap=_s(0.030),
        name="pendulum collar is retained on shaft",
    )
    ctx.expect_overlap(
        "coarse_weight",
        "pendulum_rod",
        axes="z",
        elem_a="coarse_body",
        elem_b="rod_with_pivot",
        min_overlap=_s(0.040),
        name="coarse weight surrounds a retained length of rod",
    )
    ctx.expect_overlap(
        "fine_weight",
        "pendulum_rod",
        axes="z",
        elem_a="fine_body",
        elem_b="rod_with_pivot",
        min_overlap=_s(0.028),
        name="fine weight surrounds a retained length of rod",
    )

    ctx.check(
        "two independent weight sliders",
        coarse_slide.articulation_type == ArticulationType.PRISMATIC
        and fine_slide.articulation_type == ArticulationType.PRISMATIC
        and coarse_slide.child == "coarse_weight"
        and fine_slide.child == "fine_weight",
    )
    ctx.expect_gap(
        fine,
        coarse,
        axis="z",
        min_gap=_s(0.110),
        name="fine weight rides above coarse weight",
    )

    coarse_rest = ctx.part_world_position(coarse)
    fine_rest = ctx.part_world_position(fine)
    with ctx.pose({coarse_slide: coarse_slide.motion_limits.upper}):
        coarse_high = ctx.part_world_position(coarse)
        fine_unchanged = ctx.part_world_position(fine)
    with ctx.pose({fine_slide: fine_slide.motion_limits.upper}):
        fine_high = ctx.part_world_position(fine)
        coarse_unchanged = ctx.part_world_position(coarse)
    ctx.check(
        "coarse weight slides along rod independently",
        coarse_rest is not None
        and coarse_high is not None
        and fine_rest is not None
        and fine_unchanged is not None
        and coarse_high[2] > coarse_rest[2] + _s(0.070)
        and abs(fine_unchanged[2] - fine_rest[2]) < _s(0.002),
        details=f"coarse_rest={coarse_rest}, coarse_high={coarse_high}, fine_rest={fine_rest}, fine_unchanged={fine_unchanged}",
    )
    ctx.check(
        "fine weight slides along rod independently",
        fine_rest is not None
        and fine_high is not None
        and coarse_rest is not None
        and coarse_unchanged is not None
        and fine_high[2] > fine_rest[2] + _s(0.100)
        and abs(coarse_unchanged[2] - coarse_rest[2]) < _s(0.002),
        details=f"fine_rest={fine_rest}, fine_high={fine_high}, coarse_rest={coarse_rest}, coarse_unchanged={coarse_unchanged}",
    )

    rest_aabb = ctx.part_world_aabb(pendulum)
    with ctx.pose({swing: 0.30}):
        swung_aabb = ctx.part_world_aabb(pendulum)
    ctx.check(
        "pendulum swings about top shaft",
        rest_aabb is not None and swung_aabb is not None and swung_aabb[0][1] < rest_aabb[0][1] - _s(0.060),
        details=f"rest_aabb={rest_aabb}, swung_aabb={swung_aabb}",
    )

    key_rest_aabb = ctx.part_world_aabb(key)
    with ctx.pose({key_spin: math.pi / 2.0}):
        key_rotated_aabb = ctx.part_world_aabb(key)
    ctx.check(
        "rear winding key rotates continuously",
        key_rest_aabb is not None
        and key_rotated_aabb is not None
        and (key_rotated_aabb[1][2] - key_rotated_aabb[0][2])
        > (key_rest_aabb[1][2] - key_rest_aabb[0][2]) + _s(0.030),
        details=f"key_rest_aabb={key_rest_aabb}, key_rotated_aabb={key_rotated_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
