from __future__ import annotations

from math import cos, pi, sin

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


OPTICAL_AXIS = (1.0, 0.0, 0.0)


def _annular_tube(outer_radius: float, inner_radius: float, length: float):
    """CadQuery hollow cylinder centered on local Z; rotated to X at attach time."""
    outer = cq.Workplane("XY").circle(outer_radius).extrude(length / 2.0, both=True)
    bore = cq.Workplane("XY").circle(inner_radius).extrude(length, both=True)
    return outer.cut(bore)


def _add_x_tube(
    part,
    *,
    name: str,
    outer_radius: float,
    inner_radius: float,
    length: float,
    x: float,
    material: Material,
) -> None:
    part.visual(
        mesh_from_cadquery(
            _annular_tube(outer_radius, inner_radius, length),
            name,
            tolerance=0.00045,
            angular_tolerance=0.06,
        ),
        origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _radial_box(
    part,
    *,
    name: str,
    x: float,
    radius: float,
    angle: float,
    size: tuple[float, float, float],
    material: Material,
) -> None:
    """Add a small box whose local Z thickness is radial around the X axis."""
    radial_y = -radius * sin(angle)
    radial_z = radius * cos(angle)
    part.visual(
        Box(size),
        origin=Origin(xyz=(x, radial_y, radial_z), rpy=(angle, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_retracting_kit_zoom")

    matte_black = Material("matte_black", rgba=(0.004, 0.004, 0.004, 1.0))
    satin_black = Material("satin_black", rgba=(0.025, 0.027, 0.030, 1.0))
    rubber = Material("ribbed_rubber", rgba=(0.008, 0.008, 0.008, 1.0))
    anodized = Material("dark_anodized", rgba=(0.045, 0.048, 0.052, 1.0))
    metal = Material("brushed_mount_metal", rgba=(0.62, 0.60, 0.56, 1.0))
    white = Material("engraved_white", rgba=(0.92, 0.92, 0.86, 1.0))
    red = Material("mount_red", rgba=(0.85, 0.03, 0.025, 1.0))
    glass = Material("coated_glass", rgba=(0.17, 0.45, 0.78, 0.45))

    # The fixed outer barrel is a realistic hollow sleeve with a raised rear
    # camera mount, a front shoulder, and a smaller fixed tube running under the
    # free-rotating zoom ring.  Dimensions are approximately those of a compact
    # 18-55 mm kit zoom: about 72 mm diameter and 65 mm collapsed length.
    outer = model.part("outer_barrel")
    _add_x_tube(
        outer,
        name="main_sleeve",
        outer_radius=0.0290,
        inner_radius=0.0255,
        length=0.112,
        x=0.003,
        material=satin_black,
    )
    _add_x_tube(
        outer,
        name="rear_barrel",
        outer_radius=0.0340,
        inner_radius=0.0245,
        length=0.028,
        x=-0.042,
        material=anodized,
    )
    _add_x_tube(
        outer,
        name="rear_mount",
        outer_radius=0.0320,
        inner_radius=0.0200,
        length=0.010,
        x=-0.061,
        material=metal,
    )
    _add_x_tube(
        outer,
        name="front_shoulder",
        outer_radius=0.0340,
        inner_radius=0.0260,
        length=0.020,
        x=0.048,
        material=anodized,
    )
    _add_x_tube(
        outer,
        name="rear_zoom_race",
        outer_radius=0.0315,
        inner_radius=0.0290,
        length=0.004,
        x=-0.018,
        material=anodized,
    )
    _add_x_tube(
        outer,
        name="front_zoom_race",
        outer_radius=0.0315,
        inner_radius=0.0290,
        length=0.004,
        x=0.024,
        material=anodized,
    )
    for i, angle in enumerate((0.0, 2.0 * pi / 3.0, 4.0 * pi / 3.0)):
        _radial_box(
            outer,
            name=f"bayonet_tab_{i}",
            x=-0.066,
            radius=0.0330,
            angle=angle,
            size=(0.006, 0.016, 0.004),
            material=metal,
        )
    _radial_box(
        outer,
        name="fixed_index_line",
        x=-0.026,
        radius=0.0341,
        angle=0.0,
        size=(0.010, 0.0022, 0.0010),
        material=white,
    )
    _radial_box(
        outer,
        name="mount_red_dot",
        x=-0.052,
        radius=0.0342,
        angle=0.42,
        size=(0.0032, 0.0032, 0.0010),
        material=red,
    )

    # A separate rubber zoom ring rides around the thinner fixed sleeve.  The
    # many longitudinal ribs make its rotary control function legible.
    zoom_ring = model.part("zoom_ring")
    _add_x_tube(
        zoom_ring,
        name="rubber_sleeve",
        outer_radius=0.0365,
        inner_radius=0.0315,
        length=0.046,
        x=0.0,
        material=rubber,
    )
    for i in range(36):
        angle = 2.0 * pi * i / 36.0
        _radial_box(
            zoom_ring,
            name=f"grip_rib_{i:02d}",
            x=0.0,
            radius=0.0375,
            angle=angle,
            size=(0.043, 0.0018, 0.0022),
            material=rubber,
        )
    for i, x in enumerate((-0.014, 0.003, 0.019)):
        _radial_box(
            zoom_ring,
            name=f"zoom_mark_{i}",
            x=x,
            radius=0.0388,
            angle=0.0,
            size=(0.0022, 0.0080, 0.0009),
            material=white,
        )

    # This hidden sliding core is the prismatic carriage of the retracting
    # barrel.  It remains inside the fixed sleeve while preserving a real hollow
    # optical bore and giving the rotating inner barrel a supported spindle.
    inner_carriage = model.part("inner_carriage")
    _add_x_tube(
        inner_carriage,
        name="sliding_core",
        outer_radius=0.0135,
        inner_radius=0.0100,
        length=0.070,
        x=0.018,
        material=matte_black,
    )
    _add_x_tube(
        inner_carriage,
        name="guide_bushing",
        outer_radius=0.0255,
        inner_radius=0.0135,
        length=0.010,
        x=-0.010,
        material=matte_black,
    )
    _add_x_tube(
        inner_carriage,
        name="front_bearing",
        outer_radius=0.0160,
        inner_radius=0.0135,
        length=0.012,
        x=0.020,
        material=matte_black,
    )
    _radial_box(
        inner_carriage,
        name="cam_follower_pad",
        x=0.002,
        radius=0.0140,
        angle=pi,
        size=(0.010, 0.004, 0.0012),
        material=metal,
    )

    # The visible inner zoom barrel surrounds the carriage.  It is hollow,
    # carries the front optical glass, and has a white index mark so its rotation
    # is visible when the zoom mechanism is driven.
    inner_barrel = model.part("inner_barrel")
    _add_x_tube(
        inner_barrel,
        name="front_barrel",
        outer_radius=0.0235,
        inner_radius=0.0160,
        length=0.056,
        x=0.028,
        material=satin_black,
    )
    _add_x_tube(
        inner_barrel,
        name="filter_ring",
        outer_radius=0.0248,
        inner_radius=0.0170,
        length=0.012,
        x=0.057,
        material=matte_black,
    )
    part_glass = Cylinder(radius=0.0172, length=0.002)
    inner_barrel.visual(
        part_glass,
        origin=Origin(xyz=(0.064, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=glass,
        name="front_glass",
    )
    _add_x_tube(
        inner_barrel,
        name="retaining_ring",
        outer_radius=0.0198,
        inner_radius=0.0162,
        length=0.003,
        x=0.061,
        material=matte_black,
    )
    _radial_box(
        inner_barrel,
        name="barrel_index_mark",
        x=0.035,
        radius=0.0238,
        angle=0.0,
        size=(0.018, 0.0032, 0.0009),
        material=white,
    )

    zoom_joint = model.articulation(
        "outer_to_zoom_ring",
        ArticulationType.REVOLUTE,
        parent=outer,
        child=zoom_ring,
        origin=Origin(xyz=(0.003, 0.0, 0.0)),
        axis=OPTICAL_AXIS,
        motion_limits=MotionLimits(effort=1.0, velocity=3.0, lower=0.0, upper=1.25),
    )
    model.articulation(
        "outer_to_inner_carriage",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=inner_carriage,
        origin=Origin(),
        axis=OPTICAL_AXIS,
        motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=0.034),
    )
    model.articulation(
        "carriage_to_inner_barrel",
        ArticulationType.REVOLUTE,
        parent=inner_carriage,
        child=inner_barrel,
        origin=Origin(),
        axis=OPTICAL_AXIS,
        motion_limits=MotionLimits(effort=1.0, velocity=3.0, lower=0.0, upper=1.25),
        mimic=Mimic(joint=zoom_joint.name, multiplier=1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_barrel")
    zoom_ring = object_model.get_part("zoom_ring")
    inner_carriage = object_model.get_part("inner_carriage")
    inner_barrel = object_model.get_part("inner_barrel")
    zoom_joint = object_model.get_articulation("outer_to_zoom_ring")
    slide_joint = object_model.get_articulation("outer_to_inner_carriage")

    ctx.allow_overlap(
        inner_carriage,
        outer,
        elem_a="guide_bushing",
        elem_b="main_sleeve",
        reason=(
            "The carriage guide bushing is intentionally seated in the outer barrel "
            "sleeve as the retained sliding support for the retracting optics."
        ),
    )
    ctx.expect_contact(
        inner_carriage,
        outer,
        elem_a="guide_bushing",
        elem_b="main_sleeve",
        name="guide bushing bears against the outer sleeve",
    )
    ctx.allow_overlap(
        inner_barrel,
        inner_carriage,
        elem_a="front_barrel",
        elem_b="front_bearing",
        reason=(
            "The rotating front barrel is captured on the carriage bearing; the "
            "small local mesh intersection represents the nested rotary bearing fit."
        ),
    )
    ctx.expect_contact(
        inner_barrel,
        inner_carriage,
        elem_a="front_barrel",
        elem_b="front_bearing",
        name="front bearing supports the rotating inner barrel",
    )

    ctx.expect_overlap(
        zoom_ring,
        outer,
        axes="x",
        elem_a="rubber_sleeve",
        elem_b="main_sleeve",
        min_overlap=0.040,
        name="zoom ring wraps the fixed outer sleeve",
    )
    ctx.expect_within(
        inner_barrel,
        outer,
        axes="yz",
        inner_elem="front_barrel",
        outer_elem="front_shoulder",
        margin=0.002,
        name="collapsed inner barrel is radially inside front throat",
    )
    ctx.expect_overlap(
        inner_barrel,
        outer,
        axes="x",
        elem_a="front_barrel",
        elem_b="main_sleeve",
        min_overlap=0.050,
        name="collapsed barrel remains mostly nested",
    )
    ctx.expect_within(
        inner_carriage,
        inner_barrel,
        axes="yz",
        inner_elem="sliding_core",
        outer_elem="front_barrel",
        margin=0.002,
        name="sliding core stays inside the rotating inner barrel",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_pos = ctx.part_world_position(inner_barrel)
    rest_mark = _aabb_center(ctx.part_element_world_aabb(inner_barrel, elem="barrel_index_mark"))
    with ctx.pose({zoom_joint: 1.25, slide_joint: 0.034}):
        ctx.expect_overlap(
            inner_barrel,
            outer,
            axes="x",
            elem_a="front_barrel",
            elem_b="main_sleeve",
            min_overlap=0.020,
            name="extended barrel keeps retained insertion",
        )
        extended_pos = ctx.part_world_position(inner_barrel)
        extended_mark = _aabb_center(
            ctx.part_element_world_aabb(inner_barrel, elem="barrel_index_mark")
        )

    ctx.check(
        "zoom drive telescopes the inner barrel outward",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.030,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )
    ctx.check(
        "inner barrel index mark rotates about the optical axis",
        rest_mark is not None
        and extended_mark is not None
        and abs(extended_mark[1] - rest_mark[1]) > 0.015,
        details=f"rest_mark={rest_mark}, extended_mark={extended_mark}",
    )

    return ctx.report()


object_model = build_object_model()
