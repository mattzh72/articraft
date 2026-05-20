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
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


PORCELAIN = Material("warm_white_porcelain", rgba=(0.92, 0.90, 0.82, 1.0))
CHROME = Material("soft_chrome", rgba=(0.78, 0.80, 0.82, 1.0))
BLACK = Material("black_bakelite", rgba=(0.015, 0.014, 0.012, 1.0))
DARK = Material("deep_shadow", rgba=(0.02, 0.025, 0.03, 1.0))
STEEL = Material("brushed_stainless", rgba=(0.62, 0.67, 0.68, 1.0))
BLUE_GRAY = Material("bleach_blue_gray", rgba=(0.50, 0.62, 0.70, 1.0))


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """Small CadQuery rounded rectangular slab, centered on the origin."""
    shape = cq.Workplane("XY").box(*size)
    if radius > 0.0:
        shape = shape.edges("|Z").fillet(radius)
    return shape


def _cabinet_shell() -> cq.Workplane:
    """One porcelain cabinet mesh with an open service cavity and circular top opening."""
    width = 0.72
    depth = 0.72
    body_h = 0.84
    wall = 0.045
    deck_t = 0.045
    opening_r = 0.305
    opening_y = -0.05

    outer = (
        cq.Workplane("XY")
        .box(width, depth, body_h)
        .edges("|Z")
        .fillet(0.030)
        .translate((0.0, 0.0, body_h / 2.0))
    )
    inner = cq.Workplane("XY").box(
        width - 2.0 * wall,
        depth - 2.0 * wall,
        body_h - 0.080,
    ).translate((0.0, 0.0, 0.080 + (body_h - 0.080) / 2.0))
    shell = outer.cut(inner)

    deck = cq.Workplane("XY").box(width, depth, deck_t).translate(
        (0.0, 0.0, body_h + deck_t / 2.0)
    )
    cutter = cq.Workplane("XY").circle(opening_r).extrude(deck_t + 0.060).translate(
        (0.0, opening_y, body_h - 0.020)
    )
    deck = deck.cut(cutter)

    backsplash = (
        cq.Workplane("XY")
        .box(width, 0.120, 0.400)
        .edges("|Z")
        .fillet(0.020)
        .translate((0.0, 0.330, body_h + deck_t + 0.200))
    )

    toe_kick = (
        cq.Workplane("XY")
        .box(width * 0.92, 0.060, 0.070)
        .edges("|Z")
        .fillet(0.012)
        .translate((0.0, -0.330, 0.055))
    )

    return shell.union(deck).union(backsplash).union(toe_kick)


def _tub_shell() -> LatheGeometry:
    # Revolved thin-wall basket: open at the top, closed bowl at the bottom.
    outer = [
        (0.105, -0.285),
        (0.190, -0.250),
        (0.245, -0.170),
        (0.255, 0.135),
        (0.246, 0.275),
    ]
    inner = [
        (0.060, -0.245),
        (0.150, -0.220),
        (0.205, -0.145),
        (0.220, 0.115),
        (0.218, 0.245),
    ]
    return LatheGeometry.from_shell_profiles(
        outer,
        inner,
        segments=96,
        start_cap="round",
        end_cap="round",
        lip_samples=8,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retro_top_load_washer")

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_cabinet_shell(), "porcelain_cabinet", tolerance=0.0015),
        material=PORCELAIN,
        name="porcelain_shell",
    )

    # Circular chrome trim and a dark throat make the top-load opening read as
    # an actual hollow access to the tub, not a capped top.
    cabinet.visual(
        mesh_from_geometry(TorusGeometry(0.300, 0.010, radial_segments=24, tubular_segments=96), "opening_trim"),
        origin=Origin(xyz=(0.0, -0.05, 0.887)),
        material=CHROME,
        name="opening_trim",
    )

    # A black Bakelite control insert, chrome bezels, and small legends make the
    # tall backsplash read as a 1950s appliance control panel.
    panel_y = 0.267
    panel_front_y = 0.263
    cabinet.visual(
        Box((0.630, 0.008, 0.205)),
        origin=Origin(xyz=(0.0, panel_y, 1.090)),
        material=BLACK,
        name="control_panel",
    )
    cabinet.visual(
        Cylinder(radius=0.035, length=0.2355),
        origin=Origin(xyz=(0.0, -0.05, 0.19775)),
        material=STEEL,
        name="drive_spindle",
    )
    for i, x in enumerate((-0.205, 0.205)):
        cabinet.visual(
            Cylinder(radius=0.052, length=0.004),
            origin=Origin(xyz=(x, panel_front_y + 0.004, 1.120), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=CHROME,
            name=f"knob_bezel_{i}",
        )
        cabinet.visual(
            Box((0.085, 0.004, 0.012)),
            origin=Origin(xyz=(x, panel_front_y - 0.001, 1.183)),
            material=CHROME,
            name=f"knob_label_{i}",
        )

    for i, x in enumerate((-0.060, 0.060)):
        # Four little frame strips around each switch leave the rocker cap free
        # to contact the panel face without intersecting a solid plate.
        z = 1.055
        cabinet.visual(
            Box((0.064, 0.005, 0.008)),
            origin=Origin(xyz=(x, panel_front_y - 0.001, z + 0.044)),
            material=CHROME,
            name=f"switch_frame_top_{i}",
        )
        cabinet.visual(
            Box((0.064, 0.005, 0.008)),
            origin=Origin(xyz=(x, panel_front_y - 0.001, z - 0.044)),
            material=CHROME,
            name=f"switch_frame_bottom_{i}",
        )
        cabinet.visual(
            Box((0.008, 0.005, 0.080)),
            origin=Origin(xyz=(x - 0.036, panel_front_y - 0.001, z)),
            material=CHROME,
            name=f"switch_frame_side_{i}_0",
        )
        cabinet.visual(
            Box((0.008, 0.005, 0.080)),
            origin=Origin(xyz=(x + 0.036, panel_front_y - 0.001, z)),
            material=CHROME,
            name=f"switch_frame_side_{i}_1",
        )

    # Lid hinge receiver brackets on the cabinet and a small separate hinge for
    # the bleach-fill hatch.
    for i, x in enumerate((-0.295, 0.295)):
        cabinet.visual(
            Box((0.035, 0.032, 0.030)),
            origin=Origin(xyz=(x, 0.260, 0.910)),
            material=CHROME,
            name=f"lid_hinge_bracket_{i}",
        )
    cabinet.visual(
        Box((0.115, 0.006, 0.006)),
        origin=Origin(xyz=(-0.325, 0.170, 0.888)),
        material=BLUE_GRAY,
        name="bleach_recess",
    )
    cabinet.visual(
        Box((0.105, 0.012, 0.006)),
        origin=Origin(xyz=(-0.325, 0.246, 0.888)),
        material=CHROME,
        name="bleach_hinge_leaf",
    )
    cabinet.visual(
        Cylinder(radius=0.006, length=0.100),
        origin=Origin(xyz=(-0.325, 0.246, 0.895), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=CHROME,
        name="bleach_hinge_pin",
    )

    wash_tub = model.part("wash_tub")
    wash_tub.visual(
        mesh_from_geometry(_tub_shell(), "wash_tub_shell"),
        material=STEEL,
        name="tub_shell",
    )
    wash_tub.visual(
        Cylinder(radius=0.043, length=0.365),
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
        material=STEEL,
        name="center_agitator",
    )
    wash_tub.visual(
        Cylinder(radius=0.078, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, -0.248)),
        material=STEEL,
        name="drive_hub",
    )
    for i, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        r = 0.046
        wash_tub.visual(
            Box((0.010, 0.115, 0.105)),
            origin=Origin(
                xyz=(r * math.cos(angle), r * math.sin(angle), -0.130),
                rpy=(0.0, 0.0, angle),
            ),
            material=STEEL,
            name=f"agitator_fin_{i}",
        )

    model.articulation(
        "cabinet_to_wash_tub",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=wash_tub,
        origin=Origin(xyz=(0.0, -0.05, 0.580)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=7.0),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_rounded_box((0.540, 0.560, 0.035), 0.030), "lid_panel"),
        origin=Origin(xyz=(0.0, -0.280, 0.0175)),
        material=PORCELAIN,
        name="lid_panel",
    )
    lid.visual(
        Cylinder(radius=0.010, length=0.500),
        origin=Origin(xyz=(0.0, 0.000, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=CHROME,
        name="lid_hinge_barrel",
    )
    lid.visual(
        Box((0.220, 0.024, 0.018)),
        origin=Origin(xyz=(0.0, -0.510, 0.058)),
        material=CHROME,
        name="front_handle",
    )
    for i, x in enumerate((-0.085, 0.085)):
        lid.visual(
            Box((0.030, 0.018, 0.030)),
            origin=Origin(xyz=(x, -0.495, 0.040)),
            material=CHROME,
            name=f"handle_post_{i}",
        )

    model.articulation(
        "cabinet_to_lid",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(0.0, 0.245, 0.8965)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.75),
    )

    knob_geometry = KnobGeometry(
        0.074,
        0.038,
        body_style="skirted",
        top_diameter=0.060,
        edge_radius=0.0012,
        skirt=KnobSkirt(0.086, 0.007, flare=0.06, chamfer=0.001),
        grip=KnobGrip(style="fluted", count=22, depth=0.0012),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
        center=False,
    )
    for i, x in enumerate((-0.205, 0.205)):
        knob = model.part(f"knob_{i}")
        knob.visual(
            mesh_from_geometry(knob_geometry, f"chrome_knob_{i}"),
            material=CHROME,
            name="knob_cap",
        )
        model.articulation(
            f"cabinet_to_knob_{i}",
            ArticulationType.CONTINUOUS,
            parent=cabinet,
            child=knob,
            origin=Origin(xyz=(x, panel_front_y, 1.120), rpy=(math.pi / 2.0, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.8, velocity=8.0),
        )

    for i, x in enumerate((-0.060, 0.060)):
        rocker = model.part(f"rocker_{i}")
        rocker.visual(
            Box((0.044, 0.018, 0.066)),
            origin=Origin(xyz=(0.0, -0.009, 0.0)),
            material=PORCELAIN,
            name="rocker_cap",
        )
        rocker.visual(
            Box((0.032, 0.006, 0.038)),
            origin=Origin(xyz=(0.0, -0.018, 0.0)),
            material=BLACK,
            name="rocker_center_inlay",
        )
        model.articulation(
            f"cabinet_to_rocker_{i}",
            ArticulationType.REVOLUTE,
            parent=cabinet,
            child=rocker,
            origin=Origin(xyz=(x, panel_front_y, 1.055)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.4, velocity=5.0, lower=-0.28, upper=0.28),
        )

    hatch = model.part("bleach_hatch")
    hatch.visual(
        mesh_from_cadquery(_rounded_box((0.090, 0.110, 0.014), 0.012), "bleach_hatch_panel"),
        origin=Origin(xyz=(0.0, -0.055, 0.007)),
        material=PORCELAIN,
        name="hatch_panel",
    )
    hatch.visual(
        Box((0.050, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, -0.098, 0.0165)),
        material=CHROME,
        name="hatch_pull",
    )
    model.articulation(
        "cabinet_to_bleach_hatch",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=hatch,
        origin=Origin(xyz=(-0.325, 0.235, 0.891)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    lid = object_model.get_part("lid")
    wash_tub = object_model.get_part("wash_tub")
    hatch = object_model.get_part("bleach_hatch")

    lid_joint = object_model.get_articulation("cabinet_to_lid")
    tub_joint = object_model.get_articulation("cabinet_to_wash_tub")
    hatch_joint = object_model.get_articulation("cabinet_to_bleach_hatch")

    ctx.expect_within(
        wash_tub,
        cabinet,
        axes="xy",
        inner_elem="tub_shell",
        outer_elem="opening_trim",
        margin=0.004,
        name="wash tub sits inside the circular top opening",
    )
    ctx.expect_gap(
        cabinet,
        wash_tub,
        axis="z",
        positive_elem="opening_trim",
        negative_elem="tub_shell",
        min_gap=0.008,
        max_gap=0.060,
        name="tub lip remains below the chrome opening trim",
    )
    ctx.expect_gap(
        lid,
        cabinet,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="opening_trim",
        max_penetration=0.002,
        max_gap=0.020,
        name="closed lid seats on the top trim",
    )
    ctx.expect_overlap(
        lid,
        cabinet,
        axes="xy",
        elem_a="lid_panel",
        elem_b="opening_trim",
        min_overlap=0.48,
        name="lid covers the wash opening footprint",
    )

    rest_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_joint: 1.25}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid rotates upward on rear hinge",
        rest_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > rest_lid_aabb[1][2] + 0.20,
        details=f"rest={rest_lid_aabb}, open={open_lid_aabb}",
    )

    rest_hatch_aabb = ctx.part_world_aabb(hatch)
    with ctx.pose({hatch_joint: 0.95}):
        open_hatch_aabb = ctx.part_world_aabb(hatch)
    ctx.check(
        "bleach hatch opens upward on its small hinge",
        rest_hatch_aabb is not None
        and open_hatch_aabb is not None
        and open_hatch_aabb[1][2] > rest_hatch_aabb[1][2] + 0.035,
        details=f"rest={rest_hatch_aabb}, open={open_hatch_aabb}",
    )

    ctx.check(
        "wash tub has a continuous vertical spin axis",
        tub_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 3) for v in tub_joint.axis) == (0.0, 0.0, 1.0),
    )
    for i in range(2):
        knob_joint = object_model.get_articulation(f"cabinet_to_knob_{i}")
        rocker_joint = object_model.get_articulation(f"cabinet_to_rocker_{i}")
        ctx.check(
            f"knob {i} rotates continuously",
            knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        )
        ctx.check(
            f"rocker {i} pivots independently on a short horizontal axis",
            rocker_joint.articulation_type == ArticulationType.REVOLUTE
            and tuple(round(v, 3) for v in rocker_joint.axis) == (1.0, 0.0, 0.0)
            and rocker_joint.motion_limits is not None
            and rocker_joint.motion_limits.lower < 0.0
            and rocker_joint.motion_limits.upper > 0.0,
        )

    return ctx.report()


object_model = build_object_model()
