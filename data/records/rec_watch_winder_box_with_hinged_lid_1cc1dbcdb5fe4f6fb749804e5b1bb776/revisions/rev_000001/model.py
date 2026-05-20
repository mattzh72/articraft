from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """CadQuery box with softened vertical edges, authored around its center."""
    return cq.Workplane("XY").box(*size).edges("|Z").fillet(radius)


def _base_shell(depth: float, width: float, height: float, wall: float, floor: float) -> cq.Workplane:
    """Open-top tray: real hollow interior with a continuous bottom and walls."""
    outer = _rounded_box((depth, width, height), 0.018).translate((0.0, 0.0, height / 2.0))
    inner = (
        cq.Workplane("XY")
        .box(depth - 2.0 * wall, width - 2.0 * wall, height + 0.004)
        .translate((0.0, 0.0, floor + (height + 0.004) / 2.0))
    )
    return outer.cut(inner)


def _lid_shell(
    depth: float,
    width: float,
    height: float,
    wall: float,
    top: float,
    hinge_offset_x: float,
    bottom_z: float,
) -> cq.Workplane:
    """Shallow hollow cap in the lid joint frame; the underside is visibly open."""
    outer = _rounded_box((depth, width, height), 0.018).translate(
        (hinge_offset_x + depth / 2.0, 0.0, bottom_z + height / 2.0)
    )
    inner = (
        cq.Workplane("XY")
        .box(depth - 2.0 * wall, width - 2.0 * wall, height - top + 0.004)
        .translate(
            (
                hinge_offset_x + depth / 2.0,
                0.0,
                bottom_z - 0.002 + (height - top + 0.004) / 2.0,
            )
        )
    )
    return outer.cut(inner)


def _bearing_support(x_pos: float, axis_z: float, foot_z: float) -> cq.Workplane:
    """A connected pedestal, web, and hollow bearing collar for the cradle axle."""
    outer_r = 0.018
    inner_r = 0.010
    thickness_x = 0.014
    collar = (
        cq.Workplane("YZ")
        .circle(outer_r)
        .circle(inner_r)
        .extrude(thickness_x, both=True)
        .translate((x_pos, 0.0, axis_z))
    )
    web_h = axis_z - foot_z
    web = (
        cq.Workplane("XY")
        .box(thickness_x, 0.014, web_h)
        .translate((x_pos, 0.0, foot_z + web_h / 2.0))
    )
    foot = (
        cq.Workplane("XY")
        .box(thickness_x + 0.010, 0.056, 0.005)
        .translate((x_pos, 0.0, foot_z + 0.0025))
    )
    return collar.union(web).union(foot)


def _rotor_bezel(radius: float, inner_radius: float, length: float) -> cq.Workplane:
    """Rotating cup-like rim, axis along local X."""
    return cq.Workplane("YZ").circle(radius).circle(inner_radius).extrude(length, both=True)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_watch_winder_box")

    # Overall proportions: compact premium single-watch presentation winder.
    depth = 0.250
    width = 0.340
    base_h = 0.095
    wall = 0.012
    floor = 0.014
    lid_h = 0.055
    seam_gap = 0.002

    painted_metal = model.material("satin_graphite_painted_metal", rgba=(0.06, 0.065, 0.07, 1.0))
    warm_trim = model.material("brushed_champagne_metal", rgba=(0.78, 0.66, 0.48, 1.0))
    dark_polymer = model.material("matte_black_polymer", rgba=(0.015, 0.016, 0.018, 1.0))
    elastomer = model.material("soft_charcoal_elastomer", rgba=(0.025, 0.026, 0.028, 1.0))
    velvet = model.material("deep_navy_velvet", rgba=(0.01, 0.018, 0.055, 1.0))
    smoke = model.material("smoked_acrylic_inlay", rgba=(0.05, 0.06, 0.07, 0.45))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shell(depth, width, base_h, wall, floor), "base_shell", tolerance=0.0008),
        material=painted_metal,
        name="base_shell",
    )
    base.visual(
        Box((depth - 2.0 * wall - 0.010, width - 2.0 * wall - 0.010, 0.006)),
        origin=Origin(xyz=(0.006, 0.0, floor + 0.003)),
        material=velvet,
        name="floor_liner",
    )
    base.visual(
        Box((0.006, width - 0.040, 0.003)),
        origin=Origin(xyz=(depth / 2.0 - wall - 0.002, 0.0, base_h - 0.0005)),
        material=dark_polymer,
        name="front_gasket",
    )
    for y in (-width / 2.0 + 0.048, width / 2.0 - 0.048):
        base.visual(
            Box((0.035, 0.020, 0.006)),
            origin=Origin(xyz=(0.070, y, -0.003)),
            material=elastomer,
            name=f"foot_{0 if y < 0 else 1}",
        )
        base.visual(
            Box((0.035, 0.020, 0.006)),
            origin=Origin(xyz=(-0.070, y, -0.003)),
            material=elastomer,
            name=f"foot_{2 if y < 0 else 3}",
        )

    # Exposed lid hinge: alternating knuckles around the actual joint line.
    hinge_x = -depth / 2.0 - 0.006
    hinge_z = base_h + 0.012
    hinge_radius = 0.007
    hinge_len = 0.260
    base.visual(
        Box((0.010, hinge_len, 0.010)),
        origin=Origin(xyz=(hinge_x + 0.001, 0.0, base_h)),
        material=warm_trim,
        name="base_hinge_leaf",
    )
    for i, y in enumerate((-0.104, 0.0, 0.104)):
        base.visual(
            Cylinder(radius=hinge_radius, length=0.048),
            origin=Origin(xyz=(hinge_x, y, hinge_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=warm_trim,
            name=f"base_hinge_knuckle_{i}",
        )

    # Cradle bearing supports are fixed to the tray floor and have true clearance holes.
    spin_axis_x = 0.020
    spin_axis_z = floor + 0.052
    for name, xpos in (("rear_bearing", spin_axis_x - 0.058), ("front_bearing", spin_axis_x + 0.058)):
        base.visual(
            mesh_from_cadquery(_bearing_support(xpos, spin_axis_z, floor), name, tolerance=0.0006),
            material=warm_trim,
            name=name,
        )
    base.visual(
        Box((0.024, 0.080, 0.018)),
        origin=Origin(xyz=(spin_axis_x - 0.072, 0.0, floor + 0.009)),
        material=dark_polymer,
        name="motor_housing",
    )

    lid = model.part("lid")
    lid_offset_x = 0.006
    lid_bottom_z = -0.012 + seam_gap
    lid.visual(
        mesh_from_cadquery(
            _lid_shell(depth, width, lid_h, wall, 0.016, lid_offset_x, lid_bottom_z),
            "lid_shell",
            tolerance=0.0008,
        ),
        material=painted_metal,
        name="lid_shell",
    )
    lid.visual(
        Box((depth - 0.070, width - 0.085, 0.004)),
        origin=Origin(xyz=(lid_offset_x + depth / 2.0 + 0.010, 0.0, lid_bottom_z + lid_h - 0.001)),
        material=smoke,
        name="top_inlay",
    )
    lid.visual(
        Box((depth - 0.040, width - 0.052, 0.004)),
        origin=Origin(xyz=(lid_offset_x + depth / 2.0, 0.0, lid_bottom_z + lid_h - 0.014)),
        material=velvet,
        name="inner_liner",
    )
    lid.visual(
        Box((0.024, hinge_len, 0.004)),
        origin=Origin(xyz=(0.010, 0.0, 0.006)),
        material=warm_trim,
        name="lid_hinge_leaf",
    )
    for i, y in enumerate((-0.052, 0.052)):
        lid.visual(
            Cylinder(radius=hinge_radius, length=0.048),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=warm_trim,
            name=f"lid_hinge_knuckle_{i}",
        )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.5, lower=0.0, upper=1.25),
        motion_properties=MotionProperties(damping=0.08, friction=0.02),
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.0075, length=0.140),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_trim,
        name="axle",
    )
    cradle.visual(
        mesh_from_cadquery(_rotor_bezel(0.050, 0.036, 0.030), "cradle_bezel", tolerance=0.0006),
        material=dark_polymer,
        name="cradle_bezel",
    )
    cradle.visual(
        Cylinder(radius=0.035, length=0.034),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=elastomer,
        name="watch_cushion",
    )
    cradle.visual(
        Box((0.004, 0.036, 0.006)),
        origin=Origin(xyz=(0.018, 0.0, 0.036)),
        material=warm_trim,
        name="index_mark",
    )

    model.articulation(
        "cradle_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=cradle,
        origin=Origin(xyz=(spin_axis_x, 0.0, spin_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=2.0),
        motion_properties=MotionProperties(damping=0.02, friction=0.01),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    cradle = object_model.get_part("cradle")
    lid_hinge = object_model.get_articulation("lid_hinge")
    cradle_spin = object_model.get_articulation("cradle_spin")

    for bearing in ("rear_bearing", "front_bearing"):
        ctx.allow_overlap(
            base,
            cradle,
            elem_a=bearing,
            elem_b="axle",
            reason="The rotating axle is intentionally captured in a fixed bearing collar with a close sleeve fit.",
        )

    ctx.expect_gap(
        lid,
        base,
        axis="z",
        min_gap=0.001,
        max_gap=0.004,
        positive_elem="lid_shell",
        negative_elem="base_shell",
        name="closed lid keeps a tight visible seam above the base",
    )
    ctx.expect_overlap(
        lid,
        base,
        axes="xy",
        min_overlap=0.20,
        elem_a="lid_shell",
        elem_b="base_shell",
        name="lid footprint aligns with presentation box body",
    )

    # The cradle axle visibly passes through two fixed bearing collars.
    for bearing in ("rear_bearing", "front_bearing"):
        ctx.expect_within(
            cradle,
            base,
            axes="yz",
            margin=0.004,
            inner_elem="axle",
            outer_elem=bearing,
            name=f"cradle axle is centered in {bearing}",
        )
        ctx.expect_overlap(
            cradle,
            base,
            axes="x",
            min_overlap=0.008,
            elem_a="axle",
            elem_b=bearing,
            name=f"cradle axle reaches {bearing}",
        )

    closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({lid_hinge: 1.10}):
        open_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    ctx.check(
        "lid hinge opens upward from the rear barrel",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.10,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    rest_mark = ctx.part_element_world_aabb(cradle, elem="index_mark")
    with ctx.pose({cradle_spin: math.pi / 2.0}):
        spun_mark = ctx.part_element_world_aabb(cradle, elem="index_mark")
    ctx.check(
        "cradle spin moves the off-axis index mark around the axle",
        rest_mark is not None
        and spun_mark is not None
        and abs(
            ((rest_mark[0][2] + rest_mark[1][2]) / 2.0)
            - ((spun_mark[0][2] + spun_mark[1][2]) / 2.0)
        )
        > 0.025,
        details=f"rest={rest_mark}, spun={spun_mark}",
    )

    return ctx.report()


object_model = build_object_model()
