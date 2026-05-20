from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _rounded_base() -> cq.Workplane:
    return cq.Workplane("XY").box(0.56, 0.42, 0.075).edges("|Z").fillet(0.045).translate((0.0, 0.0, 0.0375))


def _bowl_shell() -> cq.Workplane:
    """A real hollow shallow bowl: transparent wall, annular floor, open top."""
    wall = cq.Workplane("XY").circle(0.222).circle(0.208).extrude(0.172)
    floor = cq.Workplane("XY").circle(0.222).circle(0.034).extrude(0.022)
    return wall.union(floor).translate((0.0, 0.0, 0.008))


def _lid_and_feed_chute() -> cq.Workplane:
    feed_y = 0.073
    inner_x, inner_y = 0.136, 0.078
    outer_x, outer_y = 0.168, 0.108

    lid = cq.Workplane("XY").circle(0.218).extrude(0.016).translate((0.0, 0.0, 0.172))
    feed_cut = cq.Workplane("XY").box(inner_x, inner_y, 0.070).translate((0.0, feed_y, 0.180))
    shaft_cut = cq.Workplane("XY").circle(0.037).extrude(0.070).translate((0.0, 0.0, 0.150))
    lid = lid.cut(feed_cut).cut(shaft_cut)

    chute = (
        cq.Workplane("XY")
        .rect(outer_x, outer_y)
        .rect(inner_x, inner_y)
        .extrude(0.168)
        .translate((0.0, feed_y, 0.186))
    )
    return lid.union(chute)


def _lock_ring() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(0.112)
        .circle(0.034)
        .extrude(0.020)
        .translate((0.0, 0.0, -0.006))
    )


def _large_pusher() -> cq.Workplane:
    body = cq.Workplane("XY").rect(0.124, 0.070).rect(0.058, 0.034).extrude(-0.174)
    thumb_cap = cq.Workplane("XY").rect(0.154, 0.100).rect(0.058, 0.034).extrude(0.024)
    return body.union(thumb_cap)


def _small_pusher() -> cq.Workplane:
    stem = cq.Workplane("XY").rect(0.047, 0.026).extrude(-0.212)
    cap = cq.Workplane("XY").box(0.074, 0.052, 0.020).translate((0.0, 0.0, 0.010))
    return stem.union(cap)


def _slicing_disc() -> cq.Workplane:
    disc = cq.Workplane("XY").circle(0.154).circle(0.019).extrude(0.006).translate((0.0, 0.0, -0.003))
    hub = cq.Workplane("XY").circle(0.042).circle(0.019).extrude(0.024).translate((0.0, 0.0, -0.012))
    blade_a = (
        cq.Workplane("XY")
        .box(0.116, 0.020, 0.005)
        .translate((0.050, 0.030, 0.004))
        .rotate((0, 0, 0), (0, 0, 1), -22)
    )
    blade_b = (
        cq.Workplane("XY")
        .box(0.096, 0.018, 0.005)
        .translate((-0.050, -0.032, 0.004))
        .rotate((0, 0, 0), (0, 0, 1), 158)
    )
    return disc.union(hub).union(blade_a).union(blade_b)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="prep_food_processor")

    warm_white = model.material("warm_white", rgba=(0.93, 0.90, 0.84, 1.0))
    satin_grey = model.material("satin_grey", rgba=(0.42, 0.43, 0.42, 1.0))
    dark_panel = model.material("dark_panel", rgba=(0.055, 0.060, 0.065, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.016, 1.0))
    blue_clear = model.material("blue_clear", rgba=(0.58, 0.82, 0.95, 0.38))
    smoke_clear = model.material("smoke_clear", rgba=(0.55, 0.60, 0.62, 0.48))
    milky_clear = model.material("milky_clear", rgba=(0.88, 0.92, 0.93, 0.70))
    steel = model.material("brushed_steel", rgba=(0.72, 0.72, 0.69, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_rounded_base(), "base_body", tolerance=0.0015),
        material=warm_white,
        name="base_body",
    )
    base.visual(
        Box((0.300, 0.030, 0.076)),
        origin=Origin(xyz=(0.0, -0.222, 0.053)),
        material=dark_panel,
        name="front_panel",
    )
    base.visual(
        Cylinder(radius=0.128, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.088)),
        material=satin_grey,
        name="base_coupling",
    )
    base.visual(
        Cylinder(radius=0.016, length=0.145),
        origin=Origin(xyz=(0.0, 0.0, 0.157)),
        material=steel,
        name="center_shaft",
    )
    base.visual(
        Cylinder(radius=0.019, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.240)),
        material=steel,
        name="shaft_tip",
    )
    for idx, (x, y) in enumerate(((-0.205, -0.142), (-0.205, 0.142), (0.205, -0.142), (0.205, 0.142))):
            base.visual(
                Cylinder(radius=0.035, length=0.012),
                origin=Origin(xyz=(x, y, -0.006)),
                material=black_rubber,
                name=f"foot_{idx}",
            )

    bowl = model.part("bowl")
    bowl.visual(
        mesh_from_cadquery(_bowl_shell(), "bowl_shell", tolerance=0.0012),
        material=blue_clear,
        name="bowl_shell",
    )
    bowl.visual(
        mesh_from_cadquery(cq.Workplane("XY").circle(0.218).circle(0.208).extrude(0.014).translate((0.0, 0.0, 0.172)), "lid_rim", tolerance=0.001),
        material=smoke_clear,
        name="lid_rim",
    )
    # Four lid bridges connect the raised feed tube to the bowl rim without
    # covering the actual feed opening.
    for name, size, xyz in (
        ("lid_bridge_pos_x", (0.126, 0.108, 0.014), (0.147, 0.073, 0.179)),
        ("lid_bridge_neg_x", (0.126, 0.108, 0.014), (-0.147, 0.073, 0.179)),
        ("lid_bridge_pos_y", (0.168, 0.078, 0.014), (0.0, 0.166, 0.179)),
        ("lid_bridge_neg_y", (0.168, 0.070, 0.014), (0.0, -0.001, 0.179)),
    ):
        bowl.visual(Box(size), origin=Origin(xyz=xyz), material=smoke_clear, name=name)
    bowl.visual(
        Box((0.016, 0.108, 0.170)),
        origin=Origin(xyz=(0.076, 0.073, 0.269)),
        material=smoke_clear,
        name="chute_side_pos_x",
    )
    bowl.visual(
        Box((0.016, 0.108, 0.170)),
        origin=Origin(xyz=(-0.076, 0.073, 0.269)),
        material=smoke_clear,
        name="chute_side_neg_x",
    )
    bowl.visual(
        Box((0.136, 0.015, 0.170)),
        origin=Origin(xyz=(0.0, 0.1195, 0.269)),
        material=smoke_clear,
        name="chute_end_pos_y",
    )
    bowl.visual(
        Box((0.136, 0.015, 0.170)),
        origin=Origin(xyz=(0.0, 0.0265, 0.269)),
        material=smoke_clear,
        name="chute_end_neg_y",
    )
    for name, size, xyz in (
        ("chute_top_pos_x", (0.022, 0.128, 0.010), (0.095, 0.073, 0.354)),
        ("chute_top_neg_x", (0.022, 0.128, 0.010), (-0.095, 0.073, 0.354)),
        ("chute_top_pos_y", (0.188, 0.022, 0.010), (0.0, 0.138, 0.354)),
        ("chute_top_neg_y", (0.188, 0.022, 0.010), (0.0, 0.008, 0.354)),
    ):
        bowl.visual(Box(size), origin=Origin(xyz=xyz), material=smoke_clear, name=name)
    bowl.visual(
        mesh_from_cadquery(_lock_ring(), "lock_ring", tolerance=0.001),
        material=satin_grey,
        name="lock_ring",
    )

    disc = model.part("slicing_disc")
    disc.visual(
        mesh_from_cadquery(_slicing_disc(), "slicing_disc", tolerance=0.0008),
        material=steel,
        name="disc_blade",
    )

    large_pusher = model.part("large_pusher")
    large_pusher.visual(
        mesh_from_cadquery(_large_pusher(), "large_pusher", tolerance=0.001),
        material=milky_clear,
        name="large_body",
    )
    large_pusher.visual(
        Box((0.007, 0.010, 0.150)),
        origin=Origin(xyz=(0.0655, 0.0, -0.091)),
        material=milky_clear,
        name="guide_pos_x",
    )
    large_pusher.visual(
        Box((0.007, 0.010, 0.150)),
        origin=Origin(xyz=(-0.0655, 0.0, -0.091)),
        material=milky_clear,
        name="guide_neg_x",
    )

    small_pusher = model.part("small_pusher")
    small_pusher.visual(
        mesh_from_cadquery(_small_pusher(), "small_pusher", tolerance=0.001),
        material=milky_clear,
        name="small_body",
    )

    selector_knob = model.part("selector_knob")
    knob_geom = KnobGeometry(
        0.062,
        0.032,
        body_style="skirted",
        top_diameter=0.049,
        skirt=KnobSkirt(0.074, 0.006, flare=0.06, chamfer=0.001),
        grip=KnobGrip(style="fluted", count=20, depth=0.0012),
        indicator=KnobIndicator(style="line", mode="raised", depth=0.0008, angle_deg=90.0),
        center=False,
    )
    selector_knob.visual(
        mesh_from_geometry(knob_geom, "selector_knob"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_grey,
        name="knob_cap",
    )

    for idx, x in enumerate((-0.092, -0.038)):
        rocker = model.part(f"rocker_{idx}")
        rocker.visual(
            Box((0.042, 0.014, 0.054)),
            origin=Origin(xyz=(0.0, -0.007, 0.0)),
            material=black_rubber,
            name="rocker_cap",
        )
        rocker.visual(
            Cylinder(radius=0.0045, length=0.048),
            origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_grey,
            name="pivot_bar",
        )
        model.articulation(
            f"panel_to_rocker_{idx}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=rocker,
            origin=Origin(xyz=(x, -0.237, 0.058)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.5, velocity=4.0, lower=-0.28, upper=0.28),
        )

    model.articulation(
        "base_to_bowl",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0),
    )
    model.articulation(
        "shaft_to_disc",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=disc,
        origin=Origin(xyz=(0.0, 0.0, 0.226)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=25.0),
    )
    model.articulation(
        "chute_to_large_pusher",
        ArticulationType.PRISMATIC,
        parent=bowl,
        child=large_pusher,
        origin=Origin(xyz=(0.0, 0.073, 0.360)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.35, lower=0.0, upper=0.075),
    )
    model.articulation(
        "large_to_small_pusher",
        ArticulationType.PRISMATIC,
        parent=large_pusher,
        child=small_pusher,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.30, lower=0.0, upper=0.060),
    )
    model.articulation(
        "panel_to_selector",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=selector_knob,
        origin=Origin(xyz=(0.078, -0.237, 0.058)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    disc = object_model.get_part("slicing_disc")
    large = object_model.get_part("large_pusher")
    small = object_model.get_part("small_pusher")
    bowl_joint = object_model.get_articulation("base_to_bowl")
    disc_joint = object_model.get_articulation("shaft_to_disc")
    large_slide = object_model.get_articulation("chute_to_large_pusher")
    small_slide = object_model.get_articulation("large_to_small_pusher")

    ctx.allow_overlap(
        base,
        bowl,
        elem_a="base_coupling",
        elem_b="lock_ring",
        reason="The bowl's bayonet lock ring intentionally nests slightly into the short base coupling.",
    )
    ctx.allow_overlap(
        base,
        disc,
        elem_a="shaft_tip",
        elem_b="disc_blade",
        reason="The slicing disc hub is intentionally captured on the central drive shaft tip.",
    )
    ctx.allow_overlap(
        bowl,
        large,
        elem_a="chute_side_pos_x",
        elem_b="guide_pos_x",
        reason="The pusher's molded guide rib is intentionally a very close sliding fit against the feed chute wall.",
    )
    ctx.allow_overlap(
        bowl,
        large,
        elem_a="chute_side_neg_x",
        elem_b="guide_neg_x",
        reason="The opposite pusher guide rib is intentionally a very close sliding fit against the feed chute wall.",
    )
    ctx.expect_overlap(
        bowl,
        base,
        axes="xy",
        elem_a="lock_ring",
        elem_b="base_coupling",
        min_overlap=0.070,
        name="bowl lock ring is captured radially by base coupling",
    )
    ctx.expect_gap(
        bowl,
        base,
        axis="z",
        positive_elem="lock_ring",
        negative_elem="base_coupling",
        max_penetration=0.018,
        max_gap=0.004,
        name="bowl lock ring seats into shallow base coupling",
    )
    ctx.expect_within(
        base,
        disc,
        axes="xy",
        inner_elem="shaft_tip",
        outer_elem="disc_blade",
        margin=0.002,
        name="drive shaft tip is centered in the slicing disc hub",
    )
    ctx.expect_overlap(
        base,
        disc,
        axes="z",
        elem_a="shaft_tip",
        elem_b="disc_blade",
        min_overlap=0.006,
        name="drive shaft tip remains engaged with the disc hub",
    )
    ctx.expect_overlap(
        bowl,
        large,
        axes="xz",
        elem_a="chute_side_pos_x",
        elem_b="guide_pos_x",
        min_overlap=0.0005,
        name="positive pusher guide rib is engaged in the chute",
    )
    ctx.expect_overlap(
        bowl,
        large,
        axes="xz",
        elem_a="chute_side_neg_x",
        elem_b="guide_neg_x",
        min_overlap=0.0005,
        name="negative pusher guide rib is engaged in the chute",
    )

    ctx.check(
        "primary continuous rotations present",
        bowl_joint.articulation_type == ArticulationType.CONTINUOUS
        and disc_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"bowl={bowl_joint.articulation_type}, disc={disc_joint.articulation_type}",
    )
    ctx.check(
        "nested pusher slides present",
        large_slide.articulation_type == ArticulationType.PRISMATIC
        and small_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"large={large_slide.articulation_type}, small={small_slide.articulation_type}",
    )
    ctx.expect_within(
        large,
        bowl,
        axes="xy",
        inner_elem="large_body",
        margin=0.003,
        name="large pusher is centered in the feed chute footprint",
    )
    ctx.expect_overlap(
        large,
        bowl,
        axes="z",
        elem_a="large_body",
        min_overlap=0.130,
        name="large pusher remains deeply inserted when seated",
    )
    ctx.expect_within(
        small,
        large,
        axes="xy",
        inner_elem="small_body",
        outer_elem="large_body",
        margin=0.002,
        name="small pusher nests inside the large pusher footprint",
    )
    ctx.expect_within(
        disc,
        bowl,
        axes="xy",
        inner_elem="disc_blade",
        outer_elem="bowl_shell",
        margin=0.010,
        name="slicing disc stays inside the working bowl diameter",
    )

    rest_large = ctx.part_world_position(large)
    rest_small = ctx.part_world_position(small)
    with ctx.pose({large_slide: 0.075, small_slide: 0.060}):
        ctx.expect_overlap(
            large,
            bowl,
            axes="z",
            elem_a="large_body",
            min_overlap=0.055,
            name="large pusher retains insertion at full lift",
        )
        ctx.expect_overlap(
            small,
            large,
            axes="z",
            elem_a="small_body",
            elem_b="large_body",
            min_overlap=0.120,
            name="small pusher remains nested at full lift",
        )
        lifted_large = ctx.part_world_position(large)
        lifted_small = ctx.part_world_position(small)

    ctx.check(
        "pushers lift along the feed opening",
        rest_large is not None
        and lifted_large is not None
        and lifted_large[2] > rest_large[2] + 0.060
        and rest_small is not None
        and lifted_small is not None
        and lifted_small[2] > rest_small[2] + 0.120,
        details=f"large {rest_large}->{lifted_large}, small {rest_small}->{lifted_small}",
    )

    return ctx.report()


object_model = build_object_model()
