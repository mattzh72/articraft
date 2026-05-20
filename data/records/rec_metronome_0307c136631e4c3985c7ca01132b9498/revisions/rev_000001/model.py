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
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


HOUSING_HEIGHT = 0.34
PIVOT_Z = 0.325
PIVOT_Y = -0.005
ROD_LENGTH = 0.285
WEIGHT_TRAVEL = 0.13


def _housing_shell() -> cq.Workplane:
    """Broad squat frustum shell with a rounded tempo-window and top slot."""
    base_w, base_d = 0.40, 0.24
    top_w, top_d = 0.14, 0.09
    wall = 0.018

    outer = (
        cq.Workplane("XY")
        .rect(base_w, base_d)
        .workplane(offset=HOUSING_HEIGHT)
        .rect(top_w, top_d)
        .loft(combine=True)
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=0.028)
        .rect(base_w - 2.0 * wall, base_d - 2.0 * wall)
        .workplane(offset=HOUSING_HEIGHT + 0.020)
        .rect(top_w - 2.0 * wall, top_d - 2.0 * wall)
        .loft(combine=True)
    )
    shell = outer.cut(inner)

    # The front tempo-scale aperture is a large rounded vertical slot cut only
    # through the sloped front wall, leaving the back and side walls intact.
    window_box = (
        cq.Workplane("XY")
        .box(0.118, 0.180, 0.200, centered=(True, True, True))
        .translate((0.0, -0.105, 0.165))
    )
    window_top = (
        cq.Workplane("XZ")
        .center(0.0, 0.265)
        .circle(0.059)
        .extrude(0.180, both=True)
        .translate((0.0, -0.105, 0.0))
    )
    window_bottom = (
        cq.Workplane("XZ")
        .center(0.0, 0.065)
        .circle(0.059)
        .extrude(0.180, both=True)
        .translate((0.0, -0.105, 0.0))
    )
    shell = shell.cut(window_box).cut(window_top).cut(window_bottom)

    # Narrow open slot at the top where the pendulum emerges and swings.
    top_slot = (
        cq.Workplane("XY")
        .box(0.086, 0.046, 0.110, centered=(True, True, True))
        .translate((0.0, PIVOT_Y, HOUSING_HEIGHT + 0.005))
    )
    return shell.cut(top_slot)


def _sliding_weight() -> cq.Workplane:
    """A heavy cylindrical collar with a real clearance bore for the rod."""
    outer_r = 0.038
    # A very close clearance bore makes the collar read as actually riding on
    # the pendulum rod instead of floating around it.
    bore_r = 0.00435
    length = 0.055
    outer = cq.Workplane("XY").circle(outer_r).extrude(length).translate((0.0, 0.0, -length / 2.0))
    bore = cq.Workplane("XY").circle(bore_r).extrude(length + 0.010).translate((0.0, 0.0, -(length + 0.010) / 2.0))
    weight = outer.cut(bore)
    try:
        weight = weight.edges("|Z").fillet(0.003)
    except Exception:
        pass
    return weight


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wide_studio_metronome")

    walnut = Material("satin_walnut", rgba=(0.38, 0.19, 0.08, 1.0))
    dark_walnut = Material("dark_walnut_edges", rgba=(0.17, 0.08, 0.035, 1.0))
    black = Material("matte_black", rgba=(0.005, 0.005, 0.004, 1.0))
    ivory = Material("aged_ivory_scale", rgba=(0.90, 0.82, 0.58, 1.0))
    ink = Material("black_print", rgba=(0.02, 0.018, 0.012, 1.0))
    brass = Material("brushed_brass", rgba=(0.95, 0.68, 0.25, 1.0))
    steel = Material("polished_steel", rgba=(0.72, 0.74, 0.72, 1.0))

    for material in (walnut, dark_walnut, black, ivory, ink, brass, steel):
        model.materials.append(material)

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_housing_shell(), "squat_pyramid_housing", tolerance=0.0012),
        material=walnut,
        name="pyramid_shell",
    )
    housing.visual(
        Box((0.44, 0.27, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_walnut,
        name="wide_plinth",
    )
    housing.visual(
        Box((0.160, 0.006, 0.245)),
        origin=Origin(xyz=(0.0, -0.032, 0.170)),
        material=ivory,
        name="tempo_scale_plate",
    )
    housing.visual(
        Box((0.175, 0.065, 0.035)),
        origin=Origin(xyz=(0.0, -0.030, 0.042)),
        material=dark_walnut,
        name="scale_lower_mount",
    )
    # Printed tempo ticks, intentionally proud by a fraction of a millimeter so
    # they read as markings but remain connected to the plate visual.
    tick_zs = [0.070, 0.090, 0.112, 0.135, 0.158, 0.182, 0.207, 0.232, 0.257, 0.278]
    for i, z in enumerate(tick_zs):
        long_tick = i in (0, 3, 6, 9)
        tick_w = 0.070 if long_tick else 0.046
        housing.visual(
            Box((tick_w, 0.0022, 0.004)),
            origin=Origin(xyz=(0.0, -0.0350, z)),
            material=ink,
            name=f"tempo_tick_{i}",
        )
        if long_tick:
            # Small block clusters suggest printed tempo numerals without relying
            # on font assets.
            housing.visual(
                Box((0.010, 0.0022, 0.014)),
                origin=Origin(xyz=(-0.055, -0.0350, z + 0.010)),
                material=ink,
                name=f"tempo_number_{i}",
            )

    # Pivot cheeks and a brass bushing on the side provide visible physical
    # support for the moving mechanisms.
    for x, name in ((-0.051, "pivot_cheek_0"), (0.051, "pivot_cheek_1")):
        housing.visual(
            Box((0.020, 0.038, 0.035)),
            origin=Origin(xyz=(x, PIVOT_Y, PIVOT_Z)),
            material=dark_walnut,
            name=name,
        )
    housing.visual(
        Box((0.150, 0.024, 0.020)),
        origin=Origin(xyz=(0.0, 0.019, PIVOT_Z + 0.012)),
        material=dark_walnut,
        name="pivot_bridge",
    )
    side_bushing_x = 0.142
    housing.visual(
        Box((0.038, 0.075, 0.075)),
        origin=Origin(xyz=(side_bushing_x - 0.010, 0.020, 0.172)),
        material=dark_walnut,
        name="side_bushing_mount",
    )
    housing.visual(
        Cylinder(radius=0.032, length=0.014),
        origin=Origin(xyz=(side_bushing_x, 0.020, 0.172), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="side_bushing",
    )

    pendulum = model.part("pendulum")
    pendulum.visual(
        Cylinder(radius=0.0042, length=ROD_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, ROD_LENGTH / 2.0)),
        material=steel,
        name="rod_shaft",
    )
    pendulum.visual(
        Sphere(radius=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=brass,
        name="pivot_boss",
    )
    pendulum.visual(
        Cylinder(radius=0.004, length=0.082),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="pivot_pin",
    )
    pendulum.visual(
        Cylinder(radius=0.008, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, ROD_LENGTH + 0.009)),
        material=brass,
        name="top_cap",
    )

    weight = model.part("sliding_weight")
    weight.visual(
        mesh_from_cadquery(_sliding_weight(), "sliding_cylindrical_weight", tolerance=0.0008),
        material=brass,
        name="weight_shell",
    )
    weight.visual(
        Box((0.030, 0.002, 0.040)),
        origin=Origin(xyz=(0.0, -0.0385, 0.0)),
        material=ink,
        name="weight_index_line",
    )
    weight.visual(
        Box((0.0020, 0.0060, 0.026)),
        origin=Origin(xyz=(0.0047, 0.0, 0.0)),
        material=ink,
        name="weight_friction_pad",
    )

    key = model.part("winding_key")
    key.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="key_collar",
    )
    for y, suffix in ((0.040, "0"), (-0.040, "1")):
        key.visual(
            Box((0.012, 0.052, 0.026)),
            origin=Origin(xyz=(0.020, y, 0.0)),
            material=brass,
            name=f"key_wing_{suffix}",
        )
        key.visual(
            Cylinder(radius=0.017, length=0.012),
            origin=Origin(xyz=(0.020, y + (0.028 if y > 0 else -0.028), 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brass,
            name=f"key_lobe_{suffix}",
        )

    pendulum_joint = model.articulation(
        "pendulum_pivot",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=pendulum,
        origin=Origin(xyz=(0.0, PIVOT_Y, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=2.5, lower=-0.45, upper=0.45),
    )
    model.articulation(
        "weight_slide",
        ArticulationType.PRISMATIC,
        parent=pendulum,
        child=weight,
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.20, lower=0.0, upper=WEIGHT_TRAVEL),
    )
    model.articulation(
        "key_axle",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=key,
        origin=Origin(xyz=(side_bushing_x + 0.007, 0.020, 0.172)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.7, velocity=5.0),
    )

    pendulum_joint.meta["qc_samples"] = [-0.38, 0.0, 0.38]
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    pendulum = object_model.get_part("pendulum")
    weight = object_model.get_part("sliding_weight")
    key = object_model.get_part("winding_key")
    pendulum_pivot = object_model.get_articulation("pendulum_pivot")
    weight_slide = object_model.get_articulation("weight_slide")
    key_axle = object_model.get_articulation("key_axle")

    ctx.allow_overlap(
        pendulum,
        weight,
        elem_a="rod_shaft",
        elem_b="weight_friction_pad",
        reason="A small internal friction shoe intentionally presses on the pendulum rod so the sliding tempo weight is retained.",
    )

    ctx.check(
        "primary metronome mechanisms are articulated",
        pendulum_pivot.articulation_type == ArticulationType.REVOLUTE
        and weight_slide.articulation_type == ArticulationType.PRISMATIC
        and key_axle.articulation_type == ArticulationType.CONTINUOUS,
        details=f"{pendulum_pivot.articulation_type}, {weight_slide.articulation_type}, {key_axle.articulation_type}",
    )
    ctx.expect_within(
        pendulum,
        weight,
        axes="xy",
        inner_elem="rod_shaft",
        outer_elem="weight_shell",
        margin=0.002,
        name="sliding weight is centered around the rod",
    )
    ctx.expect_gap(
        weight,
        pendulum,
        axis="x",
        positive_elem="weight_friction_pad",
        negative_elem="rod_shaft",
        max_penetration=0.001,
        name="weight friction pad lightly bears on the rod",
    )

    rod_aabb = ctx.part_element_world_aabb(pendulum, elem="rod_shaft")
    housing_aabb = ctx.part_world_aabb(housing)
    ctx.check(
        "pendulum rod protrudes above the squat housing",
        rod_aabb is not None
        and housing_aabb is not None
        and rod_aabb[1][2] > housing_aabb[1][2] + 0.20,
        details=f"rod_aabb={rod_aabb}, housing_aabb={housing_aabb}",
    )

    rest_weight_pos = ctx.part_world_position(weight)
    with ctx.pose({pendulum_pivot: 0.36}):
        swung_weight_pos = ctx.part_world_position(weight)
    ctx.check(
        "pendulum pivot swings the weight sideways",
        rest_weight_pos is not None
        and swung_weight_pos is not None
        and swung_weight_pos[0] > rest_weight_pos[0] + 0.030,
        details=f"rest={rest_weight_pos}, swung={swung_weight_pos}",
    )

    with ctx.pose({weight_slide: WEIGHT_TRAVEL}):
        raised_weight_pos = ctx.part_world_position(weight)
        ctx.expect_within(
            pendulum,
            weight,
            axes="xy",
            inner_elem="rod_shaft",
            outer_elem="weight_shell",
            margin=0.002,
            name="raised weight remains coaxial with the rod",
        )
    ctx.check(
        "weight slide raises the tempo weight along the rod",
        rest_weight_pos is not None
        and raised_weight_pos is not None
        and raised_weight_pos[2] > rest_weight_pos[2] + WEIGHT_TRAVEL - 0.005,
        details=f"rest={rest_weight_pos}, raised={raised_weight_pos}",
    )

    ctx.expect_contact(
        key,
        housing,
        elem_a="key_collar",
        elem_b="side_bushing",
        contact_tol=0.001,
        name="winding key collar sits flush on the side bushing",
    )

    return ctx.report()


object_model = build_object_model()
