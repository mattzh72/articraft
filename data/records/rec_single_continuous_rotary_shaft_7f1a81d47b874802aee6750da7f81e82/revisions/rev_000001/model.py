from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_spindle")

    body_color = model.material("body_paint", color=(0.28, 0.31, 0.34))
    cartridge_color = model.material("cartridge_paint", color=(0.62, 0.64, 0.66))
    steel_color = model.material("shaft_steel", color=(0.74, 0.76, 0.78))

    base = (
        cq.Workplane("XY")
        .box(0.34, 0.24, 0.06, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.012)
    )
    upper_body = (
        cq.Workplane("XZ")
        .moveTo(-0.14, 0.06)
        .lineTo(0.14, 0.06)
        .lineTo(0.14, 0.11)
        .lineTo(-0.02, 0.11)
        .lineTo(-0.08, 0.09)
        .close()
        .extrude(0.16)
        .translate((0.0, 0.08, 0.0))
    )
    body_shape = base.union(upper_body)

    cartridge_shell = cq.Workplane("YZ").circle(0.045).extrude(0.10).translate((-0.05, 0.0, 0.0))
    front_flange = cq.Workplane("YZ").circle(0.055).extrude(0.014).translate((0.04, 0.0, 0.0))
    rear_ring = cq.Workplane("YZ").circle(0.050).extrude(0.012).translate((-0.05, 0.0, 0.0))
    mounting_foot = (
        cq.Workplane("XY")
        .rect(0.075, 0.10)
        .extrude(0.05)
        .translate((-0.005, 0.0, -0.055))
    )
    bore = cq.Workplane("YZ").circle(0.015).extrude(0.13).translate((-0.065, 0.0, 0.0))
    cartridge_shape = cartridge_shell.union(front_flange).union(rear_ring).union(mounting_foot).cut(bore)

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(body_shape, "body_casting"),
        material=body_color,
        name="body_casting",
    )

    cartridge = model.part("cartridge")
    cartridge.visual(
        mesh_from_cadquery(cartridge_shape, "spindle_cartridge"),
        material=cartridge_color,
        name="housing",
    )

    shaft = model.part("shaft")
    shaft.visual(
        Cylinder(radius=0.012, length=0.104),
        origin=Origin(xyz=(0.002, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_color,
        name="journal",
    )
    shaft.visual(
        Cylinder(radius=0.026, length=0.004),
        origin=Origin(xyz=(-0.052, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_color,
        name="rear_collar",
    )
    shaft.visual(
        Cylinder(radius=0.026, length=0.004),
        origin=Origin(xyz=(0.056, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_color,
        name="front_shoulder",
    )
    shaft.visual(
        Cylinder(radius=0.018, length=0.038),
        origin=Origin(xyz=(0.077, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_color,
        name="nose_shaft",
    )
    shaft.visual(
        Cylinder(radius=0.036, length=0.010),
        origin=Origin(xyz=(0.101, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_color,
        name="nose_plate",
    )
    shaft.visual(
        Cylinder(radius=0.005, length=0.010),
        origin=Origin(xyz=(0.111, 0.024, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_color,
        name="drive_pin",
    )

    model.articulation(
        "body_to_cartridge",
        ArticulationType.FIXED,
        parent=body,
        child=cartridge,
        origin=Origin(xyz=(0.10, 0.0, 0.165)),
    )
    model.articulation(
        "cartridge_to_shaft",
        ArticulationType.CONTINUOUS,
        parent=cartridge,
        child=shaft,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cartridge = object_model.get_part("cartridge")
    shaft = object_model.get_part("shaft")
    spindle = object_model.get_articulation("cartridge_to_shaft")

    ctx.check(
        "spindle joint is continuous about x",
        spindle.articulation_type == ArticulationType.CONTINUOUS
        and spindle.axis == (1.0, 0.0, 0.0)
        and spindle.motion_limits is not None
        and spindle.motion_limits.lower is None
        and spindle.motion_limits.upper is None,
        details=(
            f"type={spindle.articulation_type}, axis={spindle.axis}, "
            f"limits={spindle.motion_limits}"
        ),
    )
    ctx.expect_contact(
        cartridge,
        body,
        elem_a="housing",
        elem_b="body_casting",
        name="cartridge mounts directly on the body",
    )
    ctx.expect_within(
        shaft,
        cartridge,
        axes="yz",
        inner_elem="journal",
        outer_elem="housing",
        margin=0.0,
        name="shaft journal stays centered within the cartridge",
    )
    ctx.expect_contact(
        shaft,
        cartridge,
        elem_a="front_shoulder",
        elem_b="housing",
        name="shaft shoulder is supported by the cartridge face",
    )
    ctx.expect_overlap(
        shaft,
        cartridge,
        axes="x",
        elem_a="journal",
        elem_b="housing",
        min_overlap=0.09,
        name="shaft remains captured through the cartridge length",
    )
    ctx.expect_gap(
        shaft,
        cartridge,
        axis="x",
        positive_elem="nose_plate",
        negative_elem="housing",
        min_gap=0.038,
        name="nose plate stands proud of the cartridge face",
    )

    def aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_origin = ctx.part_world_position(shaft)
    rest_pin = aabb_center(ctx.part_element_world_aabb(shaft, elem="drive_pin"))
    with ctx.pose({spindle: math.pi / 2.0}):
        turned_origin = ctx.part_world_position(shaft)
        turned_pin = aabb_center(ctx.part_element_world_aabb(shaft, elem="drive_pin"))
    rest_offset = (
        None
        if rest_origin is None or rest_pin is None
        else tuple(rest_pin[i] - rest_origin[i] for i in range(3))
    )
    turned_offset = (
        None
        if turned_origin is None or turned_pin is None
        else tuple(turned_pin[i] - turned_origin[i] for i in range(3))
    )
    ctx.check(
        "drive pin follows rotation about the spindle axis",
        rest_offset is not None
        and turned_offset is not None
        and rest_offset[1] > 0.018
        and abs(rest_offset[2]) < 0.006
        and abs(turned_offset[1]) < 0.006
        and turned_offset[2] > 0.018,
        details=(
            f"rest_origin={rest_origin}, rest_pin={rest_pin}, rest_offset={rest_offset}, "
            f"turned_origin={turned_origin}, turned_pin={turned_pin}, "
            f"turned_offset={turned_offset}"
        ),
    )

    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
