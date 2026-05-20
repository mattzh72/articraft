from __future__ import annotations

from math import pi

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_stage_telescoping_slide")

    zinc = model.material("brushed_zinc", rgba=(0.62, 0.65, 0.66, 1.0))
    inner_zinc = model.material("polished_slide_steel", rgba=(0.78, 0.80, 0.79, 1.0))
    shadow = model.material("dark_recesses", rgba=(0.02, 0.025, 0.025, 1.0))
    tray_finish = model.material("blackened_tray", rgba=(0.05, 0.055, 0.06, 1.0))
    bolt_finish = model.material("dark_fasteners", rgba=(0.01, 0.01, 0.012, 1.0))

    # Overall dimensions are sized like a compact heavy-duty drawer runner.
    outer_len = 0.60
    outer_width = 0.066
    outer_height = 0.075
    wall = 0.006
    lip_depth = 0.018

    outer = model.part("outer_channel")
    # Fixed formed steel C-channel: rear mounting web, upper/lower flanges,
    # and return lips that capture the moving channel without intersecting it.
    outer.visual(
        Box((outer_len, wall, outer_height)),
        origin=Origin(xyz=(outer_len / 2.0, -outer_width / 2.0 + wall / 2.0, outer_height / 2.0)),
        material=zinc,
        name="mounting_web",
    )
    outer.visual(
        Box((outer_len, outer_width, wall)),
        origin=Origin(xyz=(outer_len / 2.0, 0.0, wall / 2.0)),
        material=zinc,
        name="lower_flange",
    )
    outer.visual(
        Box((outer_len, outer_width, wall)),
        origin=Origin(xyz=(outer_len / 2.0, 0.0, outer_height - wall / 2.0)),
        material=zinc,
        name="upper_flange",
    )
    outer.visual(
        Box((outer_len, wall, lip_depth)),
        origin=Origin(xyz=(outer_len / 2.0, outer_width / 2.0 - wall / 2.0, wall + lip_depth / 2.0)),
        material=zinc,
        name="lower_return_lip",
    )
    outer.visual(
        Box((outer_len, wall, lip_depth)),
        origin=Origin(
            xyz=(outer_len / 2.0, outer_width / 2.0 - wall / 2.0, outer_height - wall - lip_depth / 2.0)
        ),
        material=zinc,
        name="upper_return_lip",
    )
    # Dark flush mounting-slot inlays on the fixed web make the runner read as
    # a screwed-on drawer slide without creating separate unsupported islands.
    for idx, x in enumerate((0.10, 0.30, 0.50)):
        outer.visual(
            Box((0.055, 0.0012, 0.012)),
            origin=Origin(xyz=(x, -outer_width / 2.0 - 0.0002, outer_height / 2.0)),
            material=shadow,
            name=f"web_slot_{idx}",
        )

    inner_len = 0.44
    inner_width = 0.040
    inner_height = 0.030
    inner_wall = 0.004
    inner_center_z = outer_height / 2.0

    inner = model.part("inner_channel")
    # The child frame is at the rear edge of the inner channel, so hidden length
    # remains inside the outer channel at full extension.
    inner.visual(
        Box((inner_len, inner_wall, inner_height)),
        origin=Origin(xyz=(inner_len / 2.0, inner_width / 2.0 - inner_wall / 2.0, inner_center_z)),
        material=inner_zinc,
        name="inner_web",
    )
    inner.visual(
        Box((inner_len, inner_width, inner_wall)),
        origin=Origin(xyz=(inner_len / 2.0, 0.0, inner_center_z - inner_height / 2.0 + inner_wall / 2.0)),
        material=inner_zinc,
        name="lower_slide_flange",
    )
    inner.visual(
        Box((inner_len, inner_width, inner_wall)),
        origin=Origin(xyz=(inner_len / 2.0, 0.0, inner_center_z + inner_height / 2.0 - inner_wall / 2.0)),
        material=inner_zinc,
        name="upper_slide_flange",
    )
    inner.visual(
        Box((inner_len, 0.003, inner_height)),
        origin=Origin(xyz=(inner_len / 2.0, -inner_width / 2.0 + 0.0015, inner_center_z)),
        material=shadow,
        name="bearing_shadow",
    )
    inner.visual(
        Box((inner_len, 0.007, inner_wall)),
        origin=Origin(xyz=(inner_len / 2.0, 0.0235, inner_center_z - inner_height / 2.0 + inner_wall / 2.0)),
        material=shadow,
        name="lower_glide",
    )
    inner.visual(
        Box((inner_len, 0.007, inner_wall)),
        origin=Origin(xyz=(inner_len / 2.0, 0.0235, inner_center_z + inner_height / 2.0 - inner_wall / 2.0)),
        material=shadow,
        name="upper_glide",
    )

    tray = model.part("mounting_tray")
    # A small moving-end tray bolts to the nose of the inner slide and presents
    # a shallow platform with retaining lips for the carried load.
    tray.visual(
        Box((0.008, 0.044, 0.048)),
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
        material=tray_finish,
        name="back_plate",
    )
    tray.visual(
        Box((0.100, 0.120, 0.006)),
        origin=Origin(xyz=(0.055, 0.0, -0.027)),
        material=tray_finish,
        name="tray_floor",
    )
    tray.visual(
        Box((0.092, 0.006, 0.016)),
        origin=Origin(xyz=(0.057, 0.057, -0.016)),
        material=tray_finish,
        name="side_lip_0",
    )
    tray.visual(
        Box((0.092, 0.006, 0.016)),
        origin=Origin(xyz=(0.057, -0.057, -0.016)),
        material=tray_finish,
        name="side_lip_1",
    )
    tray.visual(
        Box((0.006, 0.120, 0.016)),
        origin=Origin(xyz=(0.104, 0.0, -0.016)),
        material=tray_finish,
        name="front_lip",
    )
    for idx, y in enumerate((-0.014, 0.014)):
        tray.visual(
            Cylinder(radius=0.005, length=0.003),
            origin=Origin(xyz=(0.0095, y, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=bolt_finish,
            name=f"bolt_head_{idx}",
        )
    for idx, x in enumerate((0.040, 0.075)):
        tray.visual(
            Box((0.032, 0.012, 0.0012)),
            origin=Origin(xyz=(x, 0.0, -0.0239)),
            material=shadow,
            name=f"tray_slot_{idx}",
        )

    slide = model.articulation(
        "channel_slide",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=inner,
        # At q=0 the tray is just beyond the fixed channel nose; positive travel
        # extends the moving end and tray along +X while retaining hidden length.
        origin=Origin(xyz=(outer_len - inner_len, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.35, lower=0.0, upper=0.24),
    )
    slide.meta["description"] = "Usable drawer-runner travel while preserving retained insertion."

    model.articulation(
        "tray_mount",
        ArticulationType.FIXED,
        parent=inner,
        child=tray,
        origin=Origin(xyz=(inner_len, 0.0, inner_center_z)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_channel")
    inner = object_model.get_part("inner_channel")
    tray = object_model.get_part("mounting_tray")
    slide = object_model.get_articulation("channel_slide")

    ctx.expect_within(
        inner,
        outer,
        axes="yz",
        margin=0.002,
        name="inner channel is captured inside outer cross-section",
    )
    ctx.expect_overlap(
        inner,
        outer,
        axes="x",
        min_overlap=0.40,
        name="collapsed slide has substantial retained insertion",
    )
    ctx.expect_contact(
        inner,
        outer,
        elem_a="lower_glide",
        elem_b="lower_return_lip",
        contact_tol=1e-6,
        name="lower glide bears on fixed return lip",
    )
    ctx.expect_contact(
        inner,
        outer,
        elem_a="upper_glide",
        elem_b="upper_return_lip",
        contact_tol=1e-6,
        name="upper glide bears on fixed return lip",
    )
    ctx.expect_contact(
        tray,
        inner,
        elem_a="back_plate",
        elem_b="inner_web",
        contact_tol=1e-6,
        name="moving tray is bolted to the inner-channel nose",
    )

    rest_inner_pos = ctx.part_world_position(inner)
    rest_tray_pos = ctx.part_world_position(tray)
    with ctx.pose({slide: 0.24}):
        ctx.expect_within(
            inner,
            outer,
            axes="yz",
            margin=0.002,
            name="extended inner channel remains centered in the outer channel",
        )
        ctx.expect_overlap(
            inner,
            outer,
            axes="x",
            min_overlap=0.18,
            name="extended slide keeps a hidden length engaged",
        )
        extended_inner_pos = ctx.part_world_position(inner)
        extended_tray_pos = ctx.part_world_position(tray)

    ctx.check(
        "prismatic joint extends along slide axis",
        rest_inner_pos is not None
        and extended_inner_pos is not None
        and extended_inner_pos[0] > rest_inner_pos[0] + 0.20,
        details=f"rest={rest_inner_pos}, extended={extended_inner_pos}",
    )
    ctx.check(
        "mounting tray travels with inner stage",
        rest_tray_pos is not None
        and extended_tray_pos is not None
        and extended_tray_pos[0] > rest_tray_pos[0] + 0.20,
        details=f"rest={rest_tray_pos}, extended={extended_tray_pos}",
    )

    return ctx.report()


object_model = build_object_model()
