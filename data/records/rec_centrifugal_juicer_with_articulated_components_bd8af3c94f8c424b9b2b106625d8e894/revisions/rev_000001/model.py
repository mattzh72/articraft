from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _mat(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]) -> Material:
    return model.material(name, rgba=rgba)


def _rounded_base_mesh():
    """Smooth appliance base with a real front drawer pocket cut through it."""
    sx, sy, sz = 0.46, 0.38, 0.18
    body = cq.Workplane("XY").box(sx, sy, sz).translate((0.0, 0.0, sz / 2.0))
    body = body.edges("|Z").fillet(0.045)

    # Full-depth lower pocket for the drip drawer.  It opens through the front
    # and preserves top/bottom skins so the drawer is not intersecting a solid.
    drawer_pocket = (
        cq.Workplane("XY")
        .box(0.305, 0.250, 0.082)
        .translate((0.0, -0.105, 0.066))
    )
    body = body.cut(drawer_pocket)

    # Shallow flat on the right-side wall for the two rocker switches.
    switch_flat = (
        cq.Workplane("XY")
        .box(0.060, 0.170, 0.092)
        .translate((sx / 2.0 + 0.010, 0.0, 0.104))
    )
    body = body.cut(switch_flat)
    return body


def _drawer_mesh():
    """Shallow removable drip tray with a taller front fascia."""
    tray = cq.Workplane("XY").box(0.252, 0.172, 0.046)
    basin_cut = cq.Workplane("XY").box(0.210, 0.118, 0.033).translate((0.0, 0.000, 0.018))
    tray = tray.cut(basin_cut)
    fascia = cq.Workplane("XY").box(0.286, 0.016, 0.076).translate((0.0, -0.094, 0.006))
    pull_recess = cq.Workplane("XY").box(0.112, 0.020, 0.022).translate((0.0, -0.102, 0.000))
    return tray.union(fascia).cut(pull_recess)


def _clear_chamber_mesh():
    outer = [
        (0.138, 0.176),
        (0.162, 0.190),
        (0.172, 0.235),
        (0.158, 0.286),
    ]
    inner = [
        (0.122, 0.181),
        (0.146, 0.196),
        (0.154, 0.236),
        (0.141, 0.281),
    ]
    return LatheGeometry.from_shell_profiles(outer, inner, segments=80, lip_samples=8)


def _lid_seat_gasket_mesh():
    outer = [(0.171, 0.282), (0.171, 0.287)]
    inner = [(0.151, 0.282), (0.151, 0.287)]
    return LatheGeometry.from_shell_profiles(outer, inner, segments=80, lip_samples=4)


def _lid_shell_mesh():
    # One continuous transparent shell: shallow rounded chamber cover that rises
    # into the vertical feed chute.  Coordinates are local to the lid hinge frame;
    # the feed axis is translated forward from the rear hinge line.
    outer = [
        (0.170, -0.019),
        (0.163, 0.006),
        (0.136, 0.058),
        (0.062, 0.102),
        (0.060, 0.300),
    ]
    inner = [
        (0.154, -0.012),
        (0.147, 0.012),
        (0.122, 0.050),
        (0.046, 0.090),
        (0.046, 0.296),
    ]
    shell = LatheGeometry.from_shell_profiles(outer, inner, segments=88, lip_samples=8)
    shell.translate(0.0, -0.185, 0.0)
    return shell


def _basket_mesh():
    outer = [
        (0.040, 0.000),
        (0.064, 0.030),
        (0.096, 0.090),
        (0.112, 0.126),
    ]
    inner = [
        (0.028, 0.006),
        (0.052, 0.032),
        (0.082, 0.085),
        (0.096, 0.118),
    ]
    return LatheGeometry.from_shell_profiles(outer, inner, segments=96, lip_samples=6)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="family_juicer")

    warm_white = _mat(model, "warm_white_plastic", (0.93, 0.91, 0.84, 1.0))
    shadow = _mat(model, "dark_shadow", (0.035, 0.035, 0.040, 1.0))
    soft_black = _mat(model, "soft_black", (0.012, 0.014, 0.016, 1.0))
    clear = _mat(model, "clear_smoke_polycarbonate", (0.68, 0.88, 0.96, 0.36))
    clear_edge = _mat(model, "blue_tinted_clear_edge", (0.46, 0.74, 0.90, 0.50))
    steel = _mat(model, "brushed_stainless", (0.72, 0.74, 0.72, 1.0))
    blade = _mat(model, "bright_cutting_steel", (0.90, 0.92, 0.90, 1.0))
    switch_red = _mat(model, "muted_red_rocker", (0.68, 0.08, 0.05, 1.0))
    switch_gray = _mat(model, "charcoal_rocker", (0.10, 0.11, 0.12, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_rounded_base_mesh(), "rounded_base_shell", tolerance=0.0012),
        material=warm_white,
        name="rounded_base_shell",
    )
    base.visual(
        mesh_from_geometry(_clear_chamber_mesh(), "clear_lower_chamber"),
        material=clear,
        name="clear_lower_chamber",
    )
    base.visual(
        mesh_from_geometry(_lid_seat_gasket_mesh(), "lid_seat_gasket"),
        material=soft_black,
        name="lid_seat_gasket",
    )
    base.visual(
        Cylinder(radius=0.037, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.192)),
        material=steel,
        name="drive_coupler",
    )
    base.visual(
        Cylinder(radius=0.015, length=0.070),
        origin=Origin(xyz=(0.0, -0.174, 0.226), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=clear_edge,
        name="juice_spout",
    )
    base.visual(
        Box((0.320, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, -0.197, 0.109)),
        material=shadow,
        name="drawer_shadow_lip",
    )
    base.visual(
        Box((0.032, 0.164, 0.092)),
        origin=Origin(xyz=(0.214, 0.001, 0.104)),
        material=shadow,
        name="switch_mount_panel",
    )
    for x in (-0.126, 0.126):
        base.visual(
            Box((0.077, 0.030, 0.120)),
            origin=Origin(xyz=(x, 0.185, 0.240)),
            material=warm_white,
            name=f"rear_hinge_upright_{0 if x < 0 else 1}",
        )
        base.visual(
            Cylinder(radius=0.012, length=0.077),
            origin=Origin(xyz=(x, 0.197, 0.305), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"rear_hinge_knuckle_{0 if x < 0 else 1}",
        )

    drawer = model.part("drip_drawer")
    drawer.visual(
        mesh_from_cadquery(_drawer_mesh(), "drip_drawer_tray", tolerance=0.001),
        material=shadow,
        name="drip_drawer_tray",
    )

    basket = model.part("cutter_basket")
    basket.visual(
        mesh_from_geometry(_basket_mesh(), "spinning_filter_basket"),
        material=steel,
        name="spinning_filter_basket",
    )
    basket.visual(
        Cylinder(radius=0.030, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=steel,
        name="basket_hub",
    )
    basket.visual(
        Box((0.122, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.025), rpy=(0.0, 0.0, 0.31)),
        material=blade,
        name="cutter_blade_0",
    )
    basket.visual(
        Box((0.122, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.026), rpy=(0.0, 0.0, 2.40)),
        material=blade,
        name="cutter_blade_1",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(_lid_shell_mesh(), "clear_lid_chute"),
        material=clear,
        name="clear_lid_chute",
    )
    lid.visual(
        Box((0.125, 0.030, 0.012)),
        origin=Origin(xyz=(0.0, -0.008, -0.004)),
        material=clear_edge,
        name="rear_hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=0.010, length=0.132),
        origin=Origin(xyz=(0.0, 0.0, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=clear_edge,
        name="rear_hinge_barrel",
    )
    pusher = model.part("feed_pusher")
    pusher.visual(
        Cylinder(radius=0.038, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
        material=warm_white,
        name="pusher_stem",
    )
    pusher.visual(
        Cylinder(radius=0.057, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=warm_white,
        name="chute_stop_flange",
    )
    pusher.visual(
        Cylinder(radius=0.052, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        material=warm_white,
        name="pusher_cap",
    )
    pusher.visual(
        Cylinder(radius=0.020, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.081)),
        material=clear_edge,
        name="thumb_dimple",
    )

    for index, (y, mat) in enumerate(((-0.040, switch_red), (0.042, switch_gray))):
        rocker = model.part(f"rocker_switch_{index}")
        rocker.visual(
            Box((0.014, 0.038, 0.050)),
            origin=Origin(xyz=(0.007, 0.0, 0.0)),
            material=mat,
            name="rocker_cap",
        )
        rocker.visual(
            Box((0.004, 0.018, 0.004)),
            origin=Origin(xyz=(0.015, 0.0, 0.015)),
            material=warm_white,
            name="indicator_mark",
        )
        model.articulation(
            f"base_to_rocker_{index}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=rocker,
            origin=Origin(xyz=(0.230, y, 0.108)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.4, velocity=5.0, lower=-0.30, upper=0.30),
        )

    model.articulation(
        "base_to_drip_drawer",
        ArticulationType.PRISMATIC,
        parent=base,
        child=drawer,
        origin=Origin(xyz=(0.0, -0.094, 0.056)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.35, lower=0.0, upper=0.140),
    )
    model.articulation(
        "base_to_cutter_basket",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=basket,
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=80.0),
    )
    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, 0.190, 0.305)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.25),
    )
    model.articulation(
        "lid_to_feed_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.0, -0.185, 0.300)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=14.0, velocity=0.28, lower=0.0, upper=0.120),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    drawer = object_model.get_part("drip_drawer")
    pusher = object_model.get_part("feed_pusher")
    lid = object_model.get_part("lid")
    basket = object_model.get_part("cutter_basket")
    drawer_joint = object_model.get_articulation("base_to_drip_drawer")
    pusher_joint = object_model.get_articulation("lid_to_feed_pusher")
    lid_joint = object_model.get_articulation("base_to_lid")
    basket_joint = object_model.get_articulation("base_to_cutter_basket")
    rocker_0 = object_model.get_articulation("base_to_rocker_0")
    rocker_1 = object_model.get_articulation("base_to_rocker_1")

    ctx.allow_overlap(
        base,
        drawer,
        elem_a="rounded_base_shell",
        elem_b="drip_drawer_tray",
        reason=(
            "The removable drip drawer is intentionally retained inside the lower front "
            "pocket of the molded base; the base mesh is the visible pocket shell while "
            "the drawer tray slides through it."
        ),
    )

    ctx.check(
        "primary mechanisms are articulated",
        drawer_joint.articulation_type == ArticulationType.PRISMATIC
        and pusher_joint.articulation_type == ArticulationType.PRISMATIC
        and lid_joint.articulation_type == ArticulationType.REVOLUTE
        and basket_joint.articulation_type == ArticulationType.CONTINUOUS
        and rocker_0.articulation_type == ArticulationType.REVOLUTE
        and rocker_1.articulation_type == ArticulationType.REVOLUTE,
    )
    ctx.expect_overlap(
        drawer,
        base,
        axes="y",
        min_overlap=0.050,
        name="closed drip drawer remains inserted in front pocket",
    )
    ctx.expect_within(
        drawer,
        base,
        axes="xz",
        margin=0.004,
        elem_a="drip_drawer_tray",
        elem_b="rounded_base_shell",
        name="drip drawer fits within the molded pocket height and width",
    )
    ctx.expect_within(
        pusher,
        lid,
        axes="xy",
        margin=0.006,
        inner_elem="pusher_stem",
        outer_elem="clear_lid_chute",
        name="feed pusher is centered in the feed chute footprint",
    )
    ctx.expect_within(
        basket,
        base,
        axes="xy",
        margin=0.010,
        inner_elem="spinning_filter_basket",
        outer_elem="clear_lower_chamber",
        name="cutter basket sits inside the clear chamber",
    )

    rest_drawer = ctx.part_world_position(drawer)
    with ctx.pose({drawer_joint: 0.120}):
        extended_drawer = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            base,
            axes="y",
            min_overlap=0.030,
            name="extended drawer still retains rear engagement",
        )
    ctx.check(
        "drip drawer slides out toward the front",
        rest_drawer is not None
        and extended_drawer is not None
        and extended_drawer[1] < rest_drawer[1] - 0.100,
        details=f"rest={rest_drawer}, extended={extended_drawer}",
    )

    rest_pusher = ctx.part_world_position(pusher)
    with ctx.pose({pusher_joint: 0.100}):
        depressed_pusher = ctx.part_world_position(pusher)
    ctx.check(
        "feed pusher translates downward along chute",
        rest_pusher is not None
        and depressed_pusher is not None
        and depressed_pusher[2] < rest_pusher[2] - 0.080,
        details=f"rest={rest_pusher}, depressed={depressed_pusher}",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_joint: 0.95}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward on rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.050,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
