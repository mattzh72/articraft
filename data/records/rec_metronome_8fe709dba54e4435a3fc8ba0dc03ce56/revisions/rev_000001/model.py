from __future__ import annotations

from math import pi

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
    ExtrudeGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_tabletop_metronome")

    wood = model.material("satin_walnut", rgba=(0.42, 0.22, 0.09, 1.0))
    dark = model.material("matte_black", rgba=(0.015, 0.014, 0.013, 1.0))
    rubber = model.material("soft_rubber", rgba=(0.025, 0.023, 0.022, 1.0))
    brass = model.material("brushed_brass", rgba=(0.92, 0.68, 0.30, 1.0))
    steel = model.material("blued_steel", rgba=(0.08, 0.09, 0.10, 1.0))

    housing = model.part("housing")
    case_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.22, 0.14, 0.016), 0.056, center=True),
        "rounded_case",
    )
    housing.visual(
        case_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=wood,
        name="case",
    )
    housing.visual(
        Box((0.078, 0.040, 0.004)),
        origin=Origin(xyz=(0.058, 0.0, 0.057)),
        material=brass,
        name="top_escutcheon",
    )

    for i, (x, y) in enumerate(
        ((-0.086, -0.052), (-0.086, 0.052), (0.086, -0.052), (0.086, 0.052))
    ):
        housing.visual(
            Box((0.027, 0.027, 0.064)),
            origin=Origin(xyz=(x, y, -0.030)),
            material=rubber,
            name=f"foot_{i}",
        )

    # Underside fork and pivot shaft leave a clear swinging gap.
    for i, y in enumerate((-0.026, 0.026)):
        housing.visual(
            Box((0.030, 0.008, 0.034)),
            origin=Origin(xyz=(0.0, y, -0.016)),
            material=brass,
            name=f"pivot_cheek_{i}",
        )
    housing.visual(
        Cylinder(radius=0.0042, length=0.066),
        origin=Origin(xyz=(0.0, 0.0, -0.018), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_shaft",
    )

    pendulum = model.part("pendulum")
    pivot_eye_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(0.010, -0.015), (0.010, 0.015)],
            [(0.0041, -0.015), (0.0041, 0.015)],
            segments=40,
            start_cap="flat",
            end_cap="flat",
            lip_samples=4,
        ),
        "pendulum_pivot_eye",
    )
    pendulum.visual(
        pivot_eye_mesh,
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="pivot_eye",
    )
    pendulum.visual(
        Cylinder(radius=0.0024, length=0.172),
        origin=Origin(xyz=(0.0, 0.0, -0.094)),
        material=steel,
        name="rod",
    )
    pendulum.visual(
        Sphere(radius=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.183)),
        material=brass,
        name="rod_tip",
    )

    weight = model.part("tempo_weight")
    weight_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(0.018, -0.016), (0.018, 0.016)],
            [(0.0062, -0.016), (0.0062, 0.016)],
            segments=48,
            start_cap="flat",
            end_cap="flat",
            lip_samples=4,
        ),
        "tempo_weight_sleeve",
    )
    weight.visual(weight_mesh, material=brass, name="sleeve")
    weight.visual(
        Cylinder(radius=0.0038, length=0.026),
        origin=Origin(xyz=(0.015, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="thumb_screw",
    )

    wind_key = model.part("winding_key")
    wind_key.visual(
        Cylinder(radius=0.0055, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=steel,
        name="stem",
    )
    wind_key.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=brass,
        name="hub",
    )
    wind_key.visual(
        Box((0.065, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=brass,
        name="wing_bar",
    )
    for i, x in enumerate((-0.034, 0.034)):
        wind_key.visual(
            Cylinder(radius=0.008, length=0.010),
            origin=Origin(xyz=(x, 0.0, 0.031)),
            material=brass,
            name=f"wing_end_{i}",
        )

    model.articulation(
        "housing_to_pendulum",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=pendulum,
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=6.0, lower=-0.48, upper=0.48),
    )
    model.articulation(
        "pendulum_to_weight",
        ArticulationType.PRISMATIC,
        parent=pendulum,
        child=weight,
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=0.085),
    )
    model.articulation(
        "housing_to_winding_key",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=wind_key,
        origin=Origin(xyz=(0.058, 0.0, 0.059)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    pendulum = object_model.get_part("pendulum")
    weight = object_model.get_part("tempo_weight")
    wind_key = object_model.get_part("winding_key")
    swing = object_model.get_articulation("housing_to_pendulum")
    slide = object_model.get_articulation("pendulum_to_weight")
    wind = object_model.get_articulation("housing_to_winding_key")

    ctx.allow_overlap(
        housing,
        pendulum,
        elem_a="pivot_shaft",
        elem_b="pivot_eye",
        reason="The steel pivot shaft is intentionally captured in the pendulum eye with a tiny bearing interference.",
    )
    ctx.allow_overlap(
        weight,
        pendulum,
        elem_a="thumb_screw",
        elem_b="rod",
        reason="The set screw intentionally clamps the sliding tempo weight against the pendulum rod.",
    )
    ctx.expect_overlap(
        housing,
        pendulum,
        axes="y",
        min_overlap=0.025,
        elem_a="pivot_shaft",
        elem_b="pivot_eye",
        name="pivot shaft passes through the pendulum eye",
    )
    ctx.expect_gap(
        weight,
        pendulum,
        axis="x",
        max_penetration=0.002,
        positive_elem="thumb_screw",
        negative_elem="rod",
        name="tempo weight set screw lightly clamps the rod",
    )
    ctx.expect_gap(
        wind_key,
        housing,
        axis="z",
        max_gap=0.001,
        max_penetration=0.000001,
        positive_elem="stem",
        negative_elem="top_escutcheon",
        name="winding key stem sits on top escutcheon",
    )
    ctx.expect_origin_distance(
        weight,
        pendulum,
        axes="xy",
        max_dist=0.001,
        name="tempo weight remains concentric with pendulum rod",
    )
    ctx.expect_overlap(
        weight,
        pendulum,
        axes="z",
        min_overlap=0.030,
        elem_a="sleeve",
        elem_b="rod",
        name="tempo weight surrounds the rod at the high setting",
    )

    rest_weight = ctx.part_world_position(weight)
    with ctx.pose({slide: 0.085}):
        ctx.expect_origin_distance(
            weight,
            pendulum,
            axes="xy",
            max_dist=0.001,
            name="tempo weight stays on the rod at the low setting",
        )
        ctx.expect_overlap(
            weight,
            pendulum,
            axes="z",
            min_overlap=0.030,
            elem_a="sleeve",
            elem_b="rod",
            name="tempo weight remains captured at full slide",
        )
        lowered_weight = ctx.part_world_position(weight)

    ctx.check(
        "tempo weight slides downward along the hanging rod",
        rest_weight is not None
        and lowered_weight is not None
        and lowered_weight[2] < rest_weight[2] - 0.075,
        details=f"rest={rest_weight}, lowered={lowered_weight}",
    )

    with ctx.pose({swing: 0.36}):
        tilted_pendulum = ctx.part_world_aabb(pendulum)
    with ctx.pose({swing: -0.36}):
        opposite_pendulum = ctx.part_world_aabb(pendulum)
    ctx.check(
        "pendulum swings side to side between corner feet",
        tilted_pendulum is not None
        and opposite_pendulum is not None
        and tilted_pendulum[0][0] < -0.045
        and opposite_pendulum[1][0] > 0.045,
        details=f"positive={tilted_pendulum}, negative={opposite_pendulum}",
    )

    with ctx.pose({wind: pi / 2.0}):
        turned_key = ctx.part_world_position(wind_key)
    ctx.check(
        "winding key turns about its fixed top shaft",
        turned_key is not None and abs(turned_key[0] - 0.058) < 0.001 and abs(turned_key[1]) < 0.001,
        details=f"turned_position={turned_key}",
    )

    return ctx.report()


object_model = build_object_model()
