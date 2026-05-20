from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="xz_positioning_stage", assets=ASSETS)

    base_gray = model.material("base_gray", rgba=(0.24, 0.25, 0.28, 1.0))
    rail_silver = model.material("rail_silver", rgba=(0.73, 0.75, 0.78, 1.0))
    carriage_blue = model.material("carriage_blue", rgba=(0.24, 0.35, 0.55, 1.0))
    slide_blue = model.material("slide_blue", rgba=(0.28, 0.41, 0.63, 1.0))
    pad_black = model.material("pad_black", rgba=(0.12, 0.12, 0.13, 1.0))

    base = model.part("base")
    base_shape = (
        cq.Workplane("XY")
        .box(0.34, 0.18, 0.02)
        .edges("|Z")
        .fillet(0.008)
        .faces("<Z")
        .workplane()
        .rect(0.24, 0.10)
        .cutBlind(0.006)
    )
    base.visual(
        mesh_from_cadquery(base_shape, "base.obj", assets=ASSETS),
        name="base_plate",
        material=base_gray,
    )
    base.inertial = Inertial.from_geometry(Box((0.34, 0.18, 0.02)), mass=4.2)

    x_rail = model.part("x_rail")
    x_rail_shape = (
        cq.Workplane("XY")
        .box(0.26, 0.03, 0.018)
        .union(cq.Workplane("XY").box(0.04, 0.05, 0.008).translate((-0.10, 0.0, -0.005)))
        .union(cq.Workplane("XY").box(0.04, 0.05, 0.008).translate((0.10, 0.0, -0.005)))
    )
    x_rail.visual(
        mesh_from_cadquery(x_rail_shape, "x_rail.obj", assets=ASSETS),
        name="x_rail_body",
        material=rail_silver,
    )
    x_rail.inertial = Inertial.from_geometry(Box((0.26, 0.05, 0.018)), mass=1.1)

    x_carriage = model.part("x_carriage")
    x_carriage_shape = (
        cq.Workplane("XY")
        .box(0.08, 0.07, 0.024, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.004)
        .union(
            cq.Workplane("XY")
            .box(0.05, 0.045, 0.014, centered=(True, True, False))
            .translate((0.0, 0.0, 0.024))
        )
    )
    x_carriage.visual(
        mesh_from_cadquery(x_carriage_shape, "x_carriage.obj", assets=ASSETS),
        name="x_carriage_body",
        material=carriage_blue,
    )
    x_carriage.inertial = Inertial.from_geometry(
        Box((0.08, 0.07, 0.038)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
    )

    z_upright = model.part("z_upright")
    z_upright_shape = (
        cq.Workplane("XY")
        .box(0.07, 0.04, 0.012, centered=(True, True, False))
        .union(
            cq.Workplane("XY")
            .box(0.024, 0.012, 0.25, centered=(True, True, False))
            .translate((0.0, 0.0, 0.012))
        )
        .union(
            cq.Workplane("XY")
            .box(0.05, 0.008, 0.21, centered=(True, True, False))
            .translate((0.0, 0.016, 0.03))
        )
    )
    z_upright.visual(
        mesh_from_cadquery(z_upright_shape, "z_upright.obj", assets=ASSETS),
        name="z_upright_body",
        material=rail_silver,
    )
    z_upright.inertial = Inertial.from_geometry(
        Box((0.07, 0.04, 0.262)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, 0.131)),
    )

    z_slide = model.part("z_slide")
    z_slide_shape = (
        cq.Workplane("XY")
        .box(0.072, 0.022, 0.05, centered=(True, True, False))
        .union(
            cq.Workplane("XY")
            .box(0.05, 0.022, 0.024, centered=(True, True, False))
            .translate((0.0, 0.0, 0.05))
        )
    )
    z_slide.visual(
        mesh_from_cadquery(z_slide_shape, "z_slide.obj", assets=ASSETS),
        name="z_slide_body",
        material=slide_blue,
    )
    z_slide.inertial = Inertial.from_geometry(
        Box((0.072, 0.022, 0.074)),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
    )

    top_pad = model.part("top_pad")
    top_pad_shape = (
        cq.Workplane("XY")
        .box(0.09, 0.03, 0.012, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.006)
    )
    top_pad.visual(
        mesh_from_cadquery(top_pad_shape, "top_pad.obj", assets=ASSETS),
        name="top_pad_plate",
        material=pad_black,
    )
    top_pad.inertial = Inertial.from_geometry(
        Box((0.09, 0.03, 0.012)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
    )

    model.articulation(
        "base_to_x_rail",
        ArticulationType.FIXED,
        parent=base,
        child=x_rail,
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
    )
    model.articulation(
        "x_rail_to_x_carriage",
        ArticulationType.PRISMATIC,
        parent=x_rail,
        child=x_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.25,
            lower=-0.10,
            upper=0.10,
        ),
    )
    model.articulation(
        "x_carriage_to_z_upright",
        ArticulationType.FIXED,
        parent=x_carriage,
        child=z_upright,
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
    )
    model.articulation(
        "z_upright_to_z_slide",
        ArticulationType.PRISMATIC,
        parent=z_upright,
        child=z_slide,
        origin=Origin(xyz=(0.0, 0.031, 0.04)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=70.0,
            velocity=0.20,
            lower=0.0,
            upper=0.15,
        ),
    )
    model.articulation(
        "z_slide_to_top_pad",
        ArticulationType.FIXED,
        parent=z_slide,
        child=top_pad,
        origin=Origin(xyz=(0.0, 0.004, 0.074)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    x_rail = object_model.get_part("x_rail")
    x_carriage = object_model.get_part("x_carriage")
    z_upright = object_model.get_part("z_upright")
    z_slide = object_model.get_part("z_slide")
    top_pad = object_model.get_part("top_pad")

    x_axis = object_model.get_articulation("x_rail_to_x_carriage")
    z_axis = object_model.get_articulation("z_upright_to_z_slide")

    base_plate = base.get_visual("base_plate")
    x_rail_body = x_rail.get_visual("x_rail_body")
    x_carriage_body = x_carriage.get_visual("x_carriage_body")
    z_upright_body = z_upright.get_visual("z_upright_body")
    z_slide_body = z_slide.get_visual("z_slide_body")
    top_pad_plate = top_pad.get_visual("top_pad_plate")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    for part_name in ("base", "x_rail", "x_carriage", "z_upright", "z_slide", "top_pad"):
        ctx.check(f"part_present_{part_name}", object_model.get_part(part_name) is not None, "")

    ctx.expect_contact(x_rail, base, elem_a=x_rail_body, elem_b=base_plate, contact_tol=0.0015)
    ctx.expect_gap(
        x_rail,
        base,
        axis="z",
        positive_elem=x_rail_body,
        negative_elem=base_plate,
        min_gap=0.0,
        max_gap=0.0015,
    )
    ctx.expect_overlap(x_rail, base, axes="xy", elem_a=x_rail_body, elem_b=base_plate, min_overlap=0.03)

    ctx.expect_contact(x_carriage, x_rail, elem_a=x_carriage_body, elem_b=x_rail_body, contact_tol=0.0015)
    ctx.expect_gap(
        x_carriage,
        x_rail,
        axis="z",
        positive_elem=x_carriage_body,
        negative_elem=x_rail_body,
        min_gap=0.0,
        max_gap=0.0015,
    )
    ctx.expect_origin_distance(x_carriage, x_rail, axes="y", max_dist=0.001)
    ctx.expect_overlap(x_carriage, x_rail, axes="xy", elem_a=x_carriage_body, elem_b=x_rail_body, min_overlap=0.03)

    ctx.expect_contact(z_upright, x_carriage, elem_a=z_upright_body, elem_b=x_carriage_body, contact_tol=0.0015)
    ctx.expect_gap(
        z_upright,
        x_carriage,
        axis="z",
        positive_elem=z_upright_body,
        negative_elem=x_carriage_body,
        min_gap=0.0,
        max_gap=0.0015,
    )

    ctx.expect_contact(z_slide, z_upright, elem_a=z_slide_body, elem_b=z_upright_body, contact_tol=0.0015)
    ctx.expect_gap(
        z_slide,
        z_upright,
        axis="y",
        positive_elem=z_slide_body,
        negative_elem=z_upright_body,
        min_gap=0.0,
        max_gap=0.0015,
    )
    ctx.expect_origin_distance(z_slide, z_upright, axes="x", max_dist=0.001)
    ctx.expect_overlap(z_slide, z_upright, axes="z", elem_a=z_slide_body, elem_b=z_upright_body, min_overlap=0.05)

    ctx.expect_contact(top_pad, z_slide, elem_a=top_pad_plate, elem_b=z_slide_body, contact_tol=0.0015)
    ctx.expect_gap(
        top_pad,
        z_slide,
        axis="z",
        positive_elem=top_pad_plate,
        negative_elem=z_slide_body,
        min_gap=0.0,
        max_gap=0.0015,
    )
    ctx.expect_overlap(top_pad, z_slide, axes="xy", elem_a=top_pad_plate, elem_b=z_slide_body, min_overlap=0.02)

    with ctx.pose({x_axis: -0.10}):
        ctx.expect_origin_distance(x_carriage, x_rail, axes="x", min_dist=0.099, max_dist=0.101)
        ctx.expect_overlap(x_carriage, x_rail, axes="x", elem_a=x_carriage_body, elem_b=x_rail_body, min_overlap=0.06)
        ctx.expect_origin_distance(z_slide, z_upright, axes="x", max_dist=0.001)

    with ctx.pose({x_axis: 0.10}):
        ctx.expect_origin_distance(x_carriage, x_rail, axes="x", min_dist=0.099, max_dist=0.101)
        ctx.expect_overlap(x_carriage, x_rail, axes="x", elem_a=x_carriage_body, elem_b=x_rail_body, min_overlap=0.06)
        ctx.expect_origin_distance(z_slide, z_upright, axes="x", max_dist=0.001)

    with ctx.pose({z_axis: 0.15}):
        ctx.expect_origin_gap(top_pad, z_upright, axis="z", min_gap=0.26, max_gap=0.27)
        ctx.expect_contact(top_pad, z_slide, elem_a=top_pad_plate, elem_b=z_slide_body, contact_tol=0.0015)
        ctx.expect_origin_distance(z_slide, z_upright, axes="x", max_dist=0.001)

    with ctx.pose({x_axis: 0.10, z_axis: 0.15}):
        ctx.expect_origin_distance(x_carriage, x_rail, axes="x", min_dist=0.099, max_dist=0.101)
        ctx.expect_origin_gap(top_pad, z_upright, axis="z", min_gap=0.26, max_gap=0.27)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
