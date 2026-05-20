from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def _hollow_tube_z(outer_radius: float, inner_radius: float, length: float):
    """CadQuery annular tube centered on local Z."""
    outer = cq.Workplane("XY").circle(outer_radius).extrude(length)
    cutter = cq.Workplane("XY").circle(inner_radius).extrude(length + 0.010).translate(
        (0.0, 0.0, -0.005)
    )
    return outer.cut(cutter).translate((0.0, 0.0, -0.5 * length))


def _hollow_tube_y(outer_radius: float, inner_radius: float, length: float):
    """CadQuery annular tube centered on local Y."""
    return _hollow_tube_z(outer_radius, inner_radius, length).rotate(
        (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0
    )


def _stem_body():
    """One connected forged stem with real through holes for steerer and bar."""
    steerer_collar = _hollow_tube_z(0.033, 0.0175, 0.060)
    bar_clamp = _hollow_tube_y(0.030, 0.0140, 0.064).translate((0.130, 0.0, 0.0))
    extension = cq.Workplane("XY").box(0.092, 0.034, 0.022).translate((0.068, 0.0, 0.0))
    top_web = cq.Workplane("XY").box(0.070, 0.024, 0.014).translate((0.070, 0.0, 0.018))
    return steerer_collar.union(extension).union(top_web).union(bar_clamp)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="xc_suspension_fork")

    matte_carbon = model.material("matte_carbon", rgba=(0.05, 0.055, 0.06, 1.0))
    anodized_black = model.material("anodized_black", rgba=(0.01, 0.012, 0.014, 1.0))
    satin_graphite = model.material("satin_graphite", rgba=(0.16, 0.17, 0.18, 1.0))
    polished_alloy = model.material("polished_alloy", rgba=(0.72, 0.74, 0.72, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    laser_mark = model.material("laser_mark", rgba=(0.60, 0.74, 0.78, 1.0))

    upper_fork = model.part("upper_fork")
    upper_fork.visual(
        Box((0.105, 0.345, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=satin_graphite,
        name="crown_bridge",
    )
    upper_fork.visual(
        Box((0.012, 0.260, 0.003)),
        origin=Origin(xyz=(0.054, 0.0, 0.012)),
        material=laser_mark,
        name="crown_front_stripe",
    )
    upper_fork.visual(
        Cylinder(radius=0.018, length=0.370),
        origin=Origin(xyz=(0.0, 0.0, 0.187)),
        material=matte_carbon,
        name="round_steerer",
    )

    sleeve_mesh = mesh_from_cadquery(_hollow_tube_z(0.034, 0.0235, 0.300), "upper_leg_sleeve")
    seal_mesh = mesh_from_cadquery(_hollow_tube_z(0.037, 0.0215, 0.018), "black_dust_seal")
    for index, y in enumerate((-0.130, 0.130)):
        upper_fork.visual(
            sleeve_mesh,
            origin=Origin(xyz=(0.0, y, -0.170)),
            material=satin_graphite,
            name=f"upper_sleeve_{index}",
        )
        upper_fork.visual(
            seal_mesh,
            origin=Origin(xyz=(0.0, y, -0.311)),
            material=dark_rubber,
            name=f"dust_seal_{index}",
        )

    # Two separate sliding lower stanchions: the suffixes avoid inventing a
    # left/right convention for this front-facing, nearly symmetric fork.
    for index, y in enumerate((-0.130, 0.130)):
        side = -1.0 if y < 0.0 else 1.0
        lower = model.part(f"lower_stanchion_{index}")
        lower.visual(
            Cylinder(radius=0.020, length=0.440),
            origin=Origin(xyz=(0.0, 0.0, -0.140)),
            material=polished_alloy,
            name="slider_tube",
        )
        lower.visual(
            Cylinder(radius=0.024, length=0.028),
            origin=Origin(xyz=(0.0, 0.0, 0.064)),
            material=dark_rubber,
            name="upper_bushing_band",
        )
        lower.visual(
            Box((0.040, 0.030, 0.058)),
            origin=Origin(xyz=(0.0, side * 0.022, -0.385)),
            material=anodized_black,
            name="dropout_tab",
        )
        lower.visual(
            Cylinder(radius=0.016, length=0.045),
            origin=Origin(xyz=(0.0, side * 0.040, -0.393), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=anodized_black,
            name="axle_boss",
        )
        model.articulation(
            f"sleeve_to_lower_{index}",
            ArticulationType.PRISMATIC,
            parent=upper_fork,
            child=lower,
            origin=Origin(xyz=(0.0, y, -0.320)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=85.0, velocity=0.35, lower=0.0, upper=0.100),
        )

    stem = model.part("stem")
    stem.visual(
        mesh_from_cadquery(_stem_body(), "short_xc_stem"),
        origin=Origin(),
        material=anodized_black,
        name="stem_body",
    )
    stem.visual(
        Box((0.010, 0.050, 0.006)),
        origin=Origin(xyz=(-0.031, 0.0, 0.020)),
        material=polished_alloy,
        name="steerer_clamp_slot",
    )
    stem.visual(
        Cylinder(radius=0.004, length=0.050),
        origin=Origin(xyz=(-0.034, 0.0, 0.010), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=polished_alloy,
        name="steerer_clamp_bolt",
    )
    stem.visual(
        Box((0.010, 0.052, 0.006)),
        origin=Origin(xyz=(0.156, 0.0, 0.0)),
        material=polished_alloy,
        name="bar_clamp_faceplate",
    )
    model.articulation(
        "steerer_to_stem",
        ArticulationType.REVOLUTE,
        parent=upper_fork,
        child=stem,
        origin=Origin(xyz=(0.0, 0.0, 0.372)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-0.65, upper=0.65),
    )

    handlebar = model.part("handlebar")
    handlebar.visual(
        Cylinder(radius=0.0105, length=0.760),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=matte_carbon,
        name="flat_bar",
    )
    handlebar.visual(
        Box((0.006, 0.560, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=laser_mark,
        name="top_index_ridge",
    )
    handlebar.visual(
        Cylinder(radius=0.016, length=0.130),
        origin=Origin(xyz=(0.0, -0.315, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_rubber,
        name="grip_0",
    )
    handlebar.visual(
        Cylinder(radius=0.016, length=0.130),
        origin=Origin(xyz=(0.0, 0.315, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_rubber,
        name="grip_1",
    )
    handlebar.visual(
        Cylinder(radius=0.0145, length=0.070),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=polished_alloy,
        name="center_clamp_zone",
    )
    model.articulation(
        "stem_to_handlebar",
        ArticulationType.REVOLUTE,
        parent=stem,
        child=handlebar,
        origin=Origin(xyz=(0.130, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-0.55, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    upper = object_model.get_part("upper_fork")
    stem = object_model.get_part("stem")
    handlebar = object_model.get_part("handlebar")
    stem_joint = object_model.get_articulation("steerer_to_stem")
    bar_joint = object_model.get_articulation("stem_to_handlebar")

    ctx.allow_overlap(
        upper,
        stem,
        elem_a="round_steerer",
        elem_b="stem_body",
        reason="The split stem collar is represented as a lightly compressed clamp around the steerer.",
    )
    ctx.expect_overlap(
        upper,
        stem,
        axes="z",
        elem_a="round_steerer",
        elem_b="stem_body",
        min_overlap=0.025,
        name="stem collar captures the round steerer",
    )
    ctx.expect_origin_distance(
        upper,
        stem,
        axes="xy",
        max_dist=0.001,
        name="stem rotates on steerer centerline",
    )

    ctx.allow_overlap(
        stem,
        handlebar,
        elem_a="stem_body",
        elem_b="center_clamp_zone",
        reason="The handlebar center sleeve is clamped inside the stem face clamp with slight compression.",
    )
    ctx.expect_within(
        handlebar,
        stem,
        axes="xz",
        inner_elem="center_clamp_zone",
        outer_elem="stem_body",
        margin=0.0,
        name="handlebar clamp zone is captured by the stem body",
    )
    ctx.expect_overlap(
        handlebar,
        stem,
        axes="y",
        elem_a="center_clamp_zone",
        elem_b="stem_body",
        min_overlap=0.060,
        name="stem clamp has real width over the handlebar",
    )

    lower_joints = [
        object_model.get_articulation("sleeve_to_lower_0"),
        object_model.get_articulation("sleeve_to_lower_1"),
    ]
    for index, joint in enumerate(lower_joints):
        lower = object_model.get_part(f"lower_stanchion_{index}")
        sleeve_name = f"upper_sleeve_{index}"
        ctx.allow_overlap(
            upper,
            lower,
            elem_a=sleeve_name,
            elem_b="upper_bushing_band",
            reason="The sliding bushing band is intentionally seated in the upper sleeve proxy.",
        )
        ctx.expect_within(
            lower,
            upper,
            axes="xy",
            inner_elem="upper_bushing_band",
            outer_elem=sleeve_name,
            margin=0.0,
            name=f"lower stanchion {index} stays centered in its upper sleeve",
        )
        ctx.expect_overlap(
            lower,
            upper,
            axes="z",
            elem_a="upper_bushing_band",
            elem_b=sleeve_name,
            min_overlap=0.025,
            name=f"lower stanchion {index} has retained sleeve insertion",
        )
        rest_position = ctx.part_world_position(lower)
        with ctx.pose({joint: 0.100}):
            ctx.expect_overlap(
                lower,
                upper,
                axes="z",
                elem_a="upper_bushing_band",
                elem_b=sleeve_name,
                min_overlap=0.025,
                name=f"compressed stanchion {index} remains inside sleeve",
            )
            compressed_position = ctx.part_world_position(lower)
        ctx.check(
            f"lower stanchion {index} slides upward",
            rest_position is not None
            and compressed_position is not None
            and compressed_position[2] > rest_position[2] + 0.095,
            details=f"rest={rest_position}, compressed={compressed_position}",
        )

    handlebar_rest = ctx.part_world_position(handlebar)
    with ctx.pose({stem_joint: 0.45}):
        handlebar_turned = ctx.part_world_position(handlebar)
    ctx.check(
        "stem revolute joint steers the handlebar",
        handlebar_rest is not None
        and handlebar_turned is not None
        and handlebar_turned[1] > handlebar_rest[1] + 0.045,
        details=f"rest={handlebar_rest}, turned={handlebar_turned}",
    )

    ridge_rest = ctx.part_element_world_aabb(handlebar, elem="top_index_ridge")
    with ctx.pose({bar_joint: 0.45}):
        ridge_rolled = ctx.part_element_world_aabb(handlebar, elem="top_index_ridge")
    ctx.check(
        "handlebar rolls in the stem clamp",
        ridge_rest is not None
        and ridge_rolled is not None
        and ridge_rolled[1][0] > ridge_rest[1][0] + 0.004,
        details=f"rest={ridge_rest}, rolled={ridge_rolled}",
    )

    return ctx.report()


object_model = build_object_model()
