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


CHAMBER_INNER_RADIUS = 0.060
CHAMBER_RIM_RADIUS = 0.077
LID_HINGE_Y = -0.064
LID_HINGE_Z = 0.294
CHUTE_X = 0.031
CHUTE_Y = 0.090
CHUTE_TOP_Z = 0.112
BUTTON_SURFACE_Z = 0.272
BUTTON_LAYOUT = ((-0.050, 0.094), (-0.022, 0.094))


def _base_body_shape() -> cq.Workplane:
    lower_body = (
        cq.Workplane("XY")
        .ellipse(0.112, 0.096)
        .workplane(offset=0.165)
        .ellipse(0.095, 0.081)
        .workplane(offset=0.055)
        .ellipse(0.090, 0.076)
        .loft(combine=True)
    )

    chamber_cavity = (
        cq.Workplane("XY", origin=(0.0, 0.0, 0.135))
        .circle(CHAMBER_INNER_RADIUS)
        .extrude(0.165)
    )
    shoulder_ring = (
        cq.Workplane("XY", origin=(0.0, 0.0, 0.220))
        .circle(0.094)
        .circle(CHAMBER_INNER_RADIUS)
        .extrude(0.014)
    )
    chamber_rim = (
        cq.Workplane("XY", origin=(0.0, 0.0, 0.234))
        .circle(CHAMBER_RIM_RADIUS)
        .circle(CHAMBER_INNER_RADIUS)
        .extrude(0.050)
    )

    return lower_body.cut(chamber_cavity).union(shoulder_ring).union(chamber_rim)


def _lid_shape() -> cq.Workplane:
    cover_center_y = 0.064
    cover_bottom_z = -0.010
    cover_outer = (
        cq.Workplane("XY", origin=(0.0, cover_center_y, cover_bottom_z))
        .circle(0.084)
        .workplane(offset=0.022)
        .circle(0.076)
        .loft(combine=True)
    )
    cover_inner = (
        cq.Workplane("XY", origin=(0.0, cover_center_y, cover_bottom_z + 0.004))
        .circle(0.078)
        .workplane(offset=0.016)
        .circle(0.070)
        .loft(combine=True)
    )

    chute_outer = (
        cq.Workplane("XY", origin=(CHUTE_X, CHUTE_Y, 0.004))
        .circle(0.036)
        .workplane(offset=0.108)
        .circle(0.040)
        .loft(combine=True)
    )
    chute_inner = (
        cq.Workplane("XY", origin=(CHUTE_X, CHUTE_Y, -0.008))
        .circle(0.0305)
        .workplane(offset=0.126)
        .circle(0.0345)
        .loft(combine=True)
    )
    chute_opening = (
        cq.Workplane("XY", origin=(CHUTE_X, CHUTE_Y, -0.020))
        .circle(0.0305)
        .extrude(0.060)
    )

    lid = cover_outer.cut(cover_inner).union(chute_outer).cut(chute_inner).cut(chute_opening)

    rear_brace = (
        cq.Workplane("XY", origin=(0.0, -0.012, -0.001))
        .box(0.100, 0.018, 0.010, centered=(True, True, False))
    )
    return lid.union(rear_brace)


def _basket_shape() -> cq.Workplane:
    basket_outer = (
        cq.Workplane("XY")
        .circle(0.050)
        .workplane(offset=0.052)
        .circle(0.055)
        .loft(combine=True)
    )
    rim = cq.Workplane("XY", origin=(0.0, 0.0, 0.052)).circle(0.058).circle(0.048).extrude(0.010)
    hub = cq.Workplane("XY", origin=(0.0, 0.0, -0.004)).circle(0.010).extrude(0.022)
    grater_cone = (
        cq.Workplane("XY", origin=(0.0, 0.0, 0.018))
        .circle(0.024)
        .workplane(offset=0.020)
        .circle(0.010)
        .loft(combine=True)
    )
    spokes = cq.Workplane("XY", origin=(0.0, 0.0, 0.020))
    for angle in (0.0, 45.0, 90.0, 135.0):
        spokes = spokes.union(
            cq.Workplane("XY", origin=(0.0, 0.0, 0.020))
            .transformed(rotate=(0.0, 0.0, angle))
            .box(0.038, 0.004, 0.008, centered=(False, True, True))
        )
    return basket_outer.union(rim).union(hub).union(grater_cone).union(spokes)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="family_juicer")

    body_silver = model.material("body_silver", rgba=(0.70, 0.72, 0.74, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.12, 0.13, 0.14, 1.0))
    clear_smoke = model.material("clear_smoke", rgba=(0.70, 0.80, 0.88, 0.35))
    stainless = model.material("stainless", rgba=(0.84, 0.86, 0.88, 1.0))
    button_light = model.material("button_light", rgba=(0.82, 0.84, 0.86, 1.0))
    button_dark = model.material("button_dark", rgba=(0.18, 0.19, 0.20, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_body_shape(), "juicer_base_body"),
        material=body_silver,
        name="body_shell",
    )
    base.visual(
        Box((0.048, 0.034, 0.016)),
        origin=Origin(xyz=(0.0, 0.099, 0.154), rpy=(-0.24, 0.0, 0.0)),
        material=stainless,
        name="spout",
    )
    base.visual(
        Box((0.072, 0.030, 0.014)),
        origin=Origin(xyz=(-0.036, 0.085, 0.265)),
        material=dark_trim,
        name="control_pad",
    )
    base.visual(
        Cylinder(radius=0.010, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.159)),
        material=dark_trim,
        name="drive_hub",
    )
    base.visual(
        Box((0.102, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, -0.093, 0.238)),
        material=dark_trim,
        name="hatch_lintel",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shape(), "juicer_lid"),
        material=clear_smoke,
        name="lid_shell",
    )

    pusher = model.part("pusher")
    pusher.visual(
        Cylinder(radius=0.029, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=dark_trim,
        name="pusher_body",
    )
    pusher.visual(
        Cylinder(radius=0.037, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=dark_trim,
        name="pusher_cap",
    )
    pusher.visual(
        Cylinder(radius=0.010, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=dark_trim,
        name="pusher_post",
    )
    pusher.visual(
        Cylinder(radius=0.006, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.036), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="pusher_handle",
    )

    basket = model.part("basket")
    basket.visual(
        mesh_from_cadquery(_basket_shape(), "juicer_basket"),
        material=stainless,
        name="basket_shell",
    )

    hatch = model.part("hatch")
    hatch.visual(
        Box((0.092, 0.012, 0.094)),
        origin=Origin(xyz=(0.0, -0.006, -0.047)),
        material=body_silver,
        name="hatch_panel",
    )
    hatch.visual(
        Box((0.030, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, -0.011, -0.084)),
        material=dark_trim,
        name="hatch_pull",
    )

    buttons: list[tuple[str, tuple[float, float, float], str]] = [
        ("button_0", (BUTTON_LAYOUT[0][0], BUTTON_LAYOUT[0][1], BUTTON_SURFACE_Z), "button_light"),
        ("button_1", (BUTTON_LAYOUT[1][0], BUTTON_LAYOUT[1][1], BUTTON_SURFACE_Z), "button_dark"),
    ]
    for part_name, button_origin, material_name in buttons:
        button = model.part(part_name)
        button.visual(
            Cylinder(radius=0.0045, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, 0.004)),
            material=dark_trim,
            name="button_stem",
        )
        button.visual(
            Cylinder(radius=0.009, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.0105)),
            material=material_name,
            name="button_cap",
        )
        model.articulation(
            f"base_to_{part_name}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=button,
            origin=Origin(xyz=button_origin),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=4.0, velocity=0.04, lower=0.0, upper=0.003),
        )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, LID_HINGE_Y, LID_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )
    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(CHUTE_X, CHUTE_Y, CHUTE_TOP_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.12, lower=0.0, upper=0.040),
    )
    model.articulation(
        "base_to_basket",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=basket,
        origin=Origin(xyz=(0.0, 0.0, 0.188)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=20.0),
    )
    model.articulation(
        "base_to_hatch",
        ArticulationType.REVOLUTE,
        parent=base,
        child=hatch,
        origin=Origin(xyz=(0.0, -0.098, 0.248)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=1.4,
            lower=0.0,
            upper=math.radians(85.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    basket = object_model.get_part("basket")
    hatch = object_model.get_part("hatch")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")

    lid_hinge = object_model.get_articulation("base_to_lid")
    pusher_slide = object_model.get_articulation("lid_to_pusher")
    basket_spin = object_model.get_articulation("base_to_basket")
    hatch_hinge = object_model.get_articulation("base_to_hatch")
    button_0_joint = object_model.get_articulation("base_to_button_0")
    button_1_joint = object_model.get_articulation("base_to_button_1")

    ctx.expect_overlap(
        lid,
        base,
        axes="xy",
        min_overlap=0.12,
        name="lid closes over the juicing chamber",
    )
    ctx.expect_gap(
        lid,
        basket,
        axis="z",
        min_gap=0.020,
        max_gap=0.060,
        name="closed lid clears the basket",
    )

    rest_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: math.radians(62.0)}):
        open_lid_aabb = ctx.part_world_aabb(lid)
        open_pusher_pos = ctx.part_world_position(pusher)

    rest_pusher_pos = ctx.part_world_position(pusher)
    with ctx.pose({pusher_slide: 0.030}):
        pressed_pusher_pos = ctx.part_world_position(pusher)

    rest_hatch_aabb = ctx.part_world_aabb(hatch)
    with ctx.pose({hatch_hinge: math.radians(72.0)}):
        open_hatch_aabb = ctx.part_world_aabb(hatch)

    rest_button_0_pos = ctx.part_world_position(button_0)
    with ctx.pose({button_0_joint: 0.0025}):
        pressed_button_0_pos = ctx.part_world_position(button_0)

    rest_button_1_pos = ctx.part_world_position(button_1)
    with ctx.pose({button_1_joint: 0.0025}):
        pressed_button_1_pos = ctx.part_world_position(button_1)

    rest_basket_pos = ctx.part_world_position(basket)
    with ctx.pose({basket_spin: math.pi / 2.0}):
        spun_basket_pos = ctx.part_world_position(basket)

    ctx.check(
        "lid opens upward",
        rest_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > rest_lid_aabb[1][2] + 0.045,
        details=f"rest={rest_lid_aabb}, open={open_lid_aabb}",
    )
    ctx.check(
        "pusher follows the opening lid",
        rest_pusher_pos is not None
        and open_pusher_pos is not None
        and open_pusher_pos[2] > rest_pusher_pos[2] + 0.015
        and open_pusher_pos[1] < rest_pusher_pos[1] - 0.100,
        details=f"rest={rest_pusher_pos}, open={open_pusher_pos}",
    )
    ctx.check(
        "pusher descends into the chute",
        rest_pusher_pos is not None
        and pressed_pusher_pos is not None
        and pressed_pusher_pos[2] < rest_pusher_pos[2] - 0.020,
        details=f"rest={rest_pusher_pos}, pressed={pressed_pusher_pos}",
    )
    ctx.check(
        "rear hatch swings back",
        rest_hatch_aabb is not None
        and open_hatch_aabb is not None
        and open_hatch_aabb[0][1] < rest_hatch_aabb[0][1] - 0.035,
        details=f"rest={rest_hatch_aabb}, open={open_hatch_aabb}",
    )
    ctx.check(
        "button_0 depresses",
        rest_button_0_pos is not None
        and pressed_button_0_pos is not None
        and pressed_button_0_pos[2] < rest_button_0_pos[2] - 0.0015,
        details=f"rest={rest_button_0_pos}, pressed={pressed_button_0_pos}",
    )
    ctx.check(
        "button_1 depresses",
        rest_button_1_pos is not None
        and pressed_button_1_pos is not None
        and pressed_button_1_pos[2] < rest_button_1_pos[2] - 0.0015,
        details=f"rest={rest_button_1_pos}, pressed={pressed_button_1_pos}",
    )
    ctx.check(
        "basket spins in place about the vertical axis",
        rest_basket_pos is not None
        and spun_basket_pos is not None
        and abs(rest_basket_pos[0] - spun_basket_pos[0]) < 1e-6
        and abs(rest_basket_pos[1] - spun_basket_pos[1]) < 1e-6
        and abs(rest_basket_pos[2] - spun_basket_pos[2]) < 1e-6,
        details=f"rest={rest_basket_pos}, spun={spun_basket_pos}",
    )

    return ctx.report()


object_model = build_object_model()
