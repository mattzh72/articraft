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


def _visor_body_mesh():
    """Padded visor body with real recesses for the mirror flap and extender sleeve."""

    body = (
        cq.Workplane("XY")
        .box(0.520, 0.028, 0.180)
        .translate((0.265, 0.0, -0.100))
        .edges("|Y")
        .fillet(0.018)
        .edges("|X")
        .fillet(0.006)
    )

    # Open-ended side sleeve: the separate sliding extender occupies this clear slot.
    extender_slot = cq.Workplane("XY").box(0.300, 0.040, 0.070).translate((0.385, 0.0, -0.125))

    # Shallow pocket cut into the cabin-facing surface for the real hinged mirror cover.
    mirror_pocket = cq.Workplane("XY").box(0.176, 0.007, 0.092).translate((0.145, -0.0115, -0.115))

    return body.cut(extender_slot).cut(mirror_pocket)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="padded_windshield_visor")

    vinyl = model.material("warm_greige_padded_vinyl", rgba=(0.68, 0.62, 0.52, 1.0))
    seam = model.material("slightly_dark_seam", rgba=(0.43, 0.38, 0.31, 1.0))
    dark = model.material("dark_molded_plastic", rgba=(0.03, 0.032, 0.03, 1.0))
    bracket_mat = model.material("satin_black_bracket", rgba=(0.01, 0.012, 0.012, 1.0))
    mirror_mat = model.material("muted_mirror_glass", rgba=(0.72, 0.80, 0.86, 0.72))

    roof_bracket = model.part("roof_bracket")
    roof_bracket.visual(
        Box((0.095, 0.060, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=bracket_mat,
        name="roof_mount_plate",
    )
    roof_bracket.visual(
        Cylinder(radius=0.020, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=bracket_mat,
        name="pivot_socket",
    )
    roof_bracket.visual(
        Cylinder(radius=0.026, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=bracket_mat,
        name="lower_washer",
    )

    retaining_clip = model.part("retaining_clip")
    retaining_clip.visual(
        Box((0.062, 0.038, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark,
        name="clip_roof_base",
    )
    retaining_clip.visual(
        Box((0.020, 0.018, 0.046)),
        origin=Origin(xyz=(0.0, 0.0, -0.028)),
        material=dark,
        name="clip_stem",
    )
    retaining_clip.visual(
        Box((0.040, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.049)),
        material=dark,
        name="upper_jaw",
    )
    retaining_clip.visual(
        Box((0.040, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.068)),
        material=dark,
        name="lower_jaw",
    )
    retaining_clip.visual(
        Box((0.010, 0.010, 0.025)),
        origin=Origin(xyz=(-0.024, 0.0, -0.0575)),
        material=dark,
        name="jaw_backbone",
    )

    swivel = model.part("swivel")
    swivel.visual(
        Cylinder(radius=0.017, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=bracket_mat,
        name="swivel_disk",
    )
    swivel.visual(
        Box((0.016, 0.018, 0.038)),
        origin=Origin(xyz=(-0.014, 0.0, -0.016)),
        material=bracket_mat,
        name="drop_link",
    )
    swivel.visual(
        Cylinder(radius=0.006, length=0.118),
        origin=Origin(xyz=(0.050, 0.0, -0.035), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bracket_mat,
        name="hinge_pin",
    )
    swivel.visual(
        Box((0.018, 0.006, 0.036)),
        origin=Origin(xyz=(0.020, -0.020, -0.035)),
        material=bracket_mat,
        name="front_yoke_cheek",
    )
    swivel.visual(
        Box((0.018, 0.006, 0.036)),
        origin=Origin(xyz=(0.020, 0.020, -0.035)),
        material=bracket_mat,
        name="rear_yoke_cheek",
    )
    swivel.visual(
        Box((0.052, 0.046, 0.010)),
        origin=Origin(xyz=(0.010, 0.0, -0.012)),
        material=bracket_mat,
        name="yoke_top_bridge",
    )

    visor = model.part("visor")
    visor.visual(
        mesh_from_cadquery(_visor_body_mesh(), "visor_padded_body", tolerance=0.0008),
        material=vinyl,
        name="padded_body",
    )
    visor.visual(
        Cylinder(radius=0.011, length=0.105),
        origin=Origin(xyz=(0.052, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=vinyl,
        name="hinge_sleeve",
    )
    visor.visual(
        Box((0.120, 0.006, 0.006)),
        origin=Origin(xyz=(0.052, 0.0, -0.010)),
        material=vinyl,
        name="sleeve_neck",
    )
    visor.visual(
        Box((0.152, 0.001, 0.064)),
        origin=Origin(xyz=(0.145, -0.0085, -0.116)),
        material=mirror_mat,
        name="mirror_glass",
    )
    visor.visual(
        Cylinder(radius=0.002, length=0.184),
        origin=Origin(xyz=(0.145, -0.0120, -0.076), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="mirror_hinge_pin",
    )
    visor.visual(
        Cylinder(radius=0.005, length=0.034),
        origin=Origin(xyz=(0.502, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="clip_pin",
    )
    visor.visual(
        Box((0.012, 0.008, 0.008)),
        origin=Origin(xyz=(0.502, 0.010, -0.007)),
        material=vinyl,
        name="clip_pin_boss",
    )
    visor.visual(
        Box((0.480, 0.003, 0.004)),
        origin=Origin(xyz=(0.270, -0.0150, -0.014)),
        material=seam,
        name="top_stitched_edge",
    )
    visor.visual(
        Box((0.500, 0.003, 0.004)),
        origin=Origin(xyz=(0.265, -0.0150, -0.186)),
        material=seam,
        name="lower_stitched_edge",
    )

    mirror_cover = model.part("mirror_cover")
    mirror_cover.visual(
        Box((0.160, 0.003, 0.078)),
        origin=Origin(xyz=(0.0, 0.0, -0.039)),
        material=vinyl,
        name="inset_flap",
    )
    mirror_cover.visual(
        Cylinder(radius=0.003, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="flap_hinge_barrel",
    )
    mirror_cover.visual(
        Box((0.040, 0.002, 0.006)),
        origin=Origin(xyz=(0.0, -0.002, -0.075)),
        material=seam,
        name="finger_lip",
    )

    extender = model.part("extender")
    extender.visual(
        Box((0.276, 0.018, 0.062)),
        origin=Origin(xyz=(0.385, 0.0, -0.125)),
        material=vinyl,
        name="sliding_panel",
    )
    extender.visual(
        Box((0.012, 0.021, 0.070)),
        origin=Origin(xyz=(0.528, 0.0, -0.125)),
        material=seam,
        name="pull_edge",
    )
    extender.visual(
        Box((0.230, 0.002, 0.004)),
        origin=Origin(xyz=(0.380, -0.010, -0.092)),
        material=seam,
        name="extender_top_seam",
    )

    model.articulation(
        "bracket_to_clip",
        ArticulationType.FIXED,
        parent=roof_bracket,
        child=retaining_clip,
        origin=Origin(xyz=(0.502, 0.0, 0.025)),
    )
    model.articulation(
        "bracket_to_swivel",
        ArticulationType.REVOLUTE,
        parent=roof_bracket,
        child=swivel,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.0, lower=0.0, upper=1.75),
    )
    model.articulation(
        "swivel_to_visor",
        ArticulationType.REVOLUTE,
        parent=swivel,
        child=visor,
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.3, lower=0.0, upper=1.90),
    )
    model.articulation(
        "visor_to_mirror_cover",
        ArticulationType.REVOLUTE,
        parent=visor,
        child=mirror_cover,
        origin=Origin(xyz=(0.145, -0.0120, -0.076)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=0.0, upper=1.65),
    )
    model.articulation(
        "visor_to_extender",
        ArticulationType.PRISMATIC,
        parent=visor,
        child=extender,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.20, lower=0.0, upper=0.160),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    visor = object_model.get_part("visor")
    extender = object_model.get_part("extender")
    cover = object_model.get_part("mirror_cover")
    clip = object_model.get_part("retaining_clip")
    swivel = object_model.get_part("swivel")
    main_hinge = object_model.get_articulation("swivel_to_visor")
    cover_hinge = object_model.get_articulation("visor_to_mirror_cover")
    slide = object_model.get_articulation("visor_to_extender")

    ctx.allow_overlap(
        swivel,
        visor,
        elem_a="hinge_pin",
        elem_b="hinge_sleeve",
        reason="The metal hinge pin is intentionally captured inside the visor hinge sleeve.",
    )
    ctx.allow_overlap(
        visor,
        cover,
        elem_a="mirror_hinge_pin",
        elem_b="flap_hinge_barrel",
        reason="The tiny vanity-cover hinge barrel intentionally wraps the panel-side hinge pin.",
    )
    ctx.expect_overlap(
        swivel,
        visor,
        axes="x",
        elem_a="hinge_pin",
        elem_b="hinge_sleeve",
        min_overlap=0.080,
        name="main hinge pin retained by sleeve",
    )
    ctx.expect_overlap(
        extender,
        visor,
        axes="x",
        elem_a="sliding_panel",
        elem_b="padded_body",
        min_overlap=0.150,
        name="collapsed extender remains nested in visor sleeve",
    )
    ctx.expect_within(
        extender,
        visor,
        axes="z",
        inner_elem="sliding_panel",
        outer_elem="padded_body",
        margin=0.005,
        name="extender panel fits within sleeve height",
    )
    ctx.expect_overlap(
        cover,
        visor,
        axes="xz",
        elem_a="inset_flap",
        elem_b="mirror_glass",
        min_overlap=0.055,
        name="mirror cover is a real flap over the mirror glass",
    )
    ctx.expect_gap(
        visor,
        cover,
        axis="y",
        positive_elem="mirror_glass",
        negative_elem="inset_flap",
        min_gap=0.0005,
        max_gap=0.005,
        name="mirror cover sits in front of separate mirror glass",
    )
    ctx.expect_gap(
        clip,
        visor,
        axis="z",
        positive_elem="upper_jaw",
        negative_elem="clip_pin",
        min_gap=0.001,
        max_gap=0.006,
        name="retaining clip upper jaw clears the pin",
    )
    ctx.expect_gap(
        visor,
        clip,
        axis="z",
        positive_elem="clip_pin",
        negative_elem="lower_jaw",
        min_gap=0.0005,
        max_gap=0.004,
        name="retaining clip lower jaw clears the pin",
    )

    def _center_axis(aabb, index: int) -> float | None:
        if aabb is None:
            return None
        return 0.5 * (aabb[0][index] + aabb[1][index])

    rest_visor_box = ctx.part_element_world_aabb(visor, elem="padded_body")
    with ctx.pose({main_hinge: 1.2}):
        tilted_visor_box = ctx.part_element_world_aabb(visor, elem="padded_body")
    rest_visor_y = _center_axis(rest_visor_box, 1)
    tilted_visor_y = _center_axis(tilted_visor_box, 1)
    ctx.check(
        "visor rotates on main roof hinge",
        rest_visor_y is not None and tilted_visor_y is not None and tilted_visor_y > rest_visor_y + 0.050,
        details=f"rest_y={rest_visor_y}, tilted_y={tilted_visor_y}",
    )

    rest_cover_box = ctx.part_element_world_aabb(cover, elem="inset_flap")
    with ctx.pose({cover_hinge: 1.0}):
        opened_cover_box = ctx.part_element_world_aabb(cover, elem="inset_flap")
    rest_cover_y = _center_axis(rest_cover_box, 1)
    opened_cover_y = _center_axis(opened_cover_box, 1)
    ctx.check(
        "mirror cover flips outward from visor face",
        rest_cover_y is not None and opened_cover_y is not None and opened_cover_y < rest_cover_y - 0.020,
        details=f"rest_y={rest_cover_y}, opened_y={opened_cover_y}",
    )

    rest_extender = ctx.part_world_position(extender)
    with ctx.pose({slide: 0.160}):
        extended_extender = ctx.part_world_position(extender)
        ctx.expect_overlap(
            extender,
            visor,
            axes="x",
            elem_a="sliding_panel",
            elem_b="padded_body",
            min_overlap=0.100,
            name="extended visor extender retains insertion",
        )
    ctx.check(
        "extender slides out laterally",
        rest_extender is not None and extended_extender is not None and extended_extender[0] > rest_extender[0] + 0.150,
        details=f"rest={rest_extender}, extended={extended_extender}",
    )

    return ctx.report()


object_model = build_object_model()
