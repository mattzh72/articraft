from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_OUTER_RADIUS = 0.100
BASE_INNER_RADIUS = 0.060
BASE_HEIGHT = 0.020
HUB_RADIUS = 0.030
WEB_WIDTH = 0.016
CORE_OUTER_RADIUS = 0.028
CORE_INNER_RADIUS = 0.017
CORE_HEIGHT = 0.022
PLATE_X = 0.120
PLATE_Y = 0.100
PLATE_THICKNESS = 0.012


def _build_ring_base_shape() -> cq.Workplane:
    ring = (
        cq.Workplane("XY")
        .circle(BASE_OUTER_RADIUS)
        .circle(BASE_INNER_RADIUS)
        .extrude(BASE_HEIGHT)
    )

    hub = cq.Workplane("XY").circle(HUB_RADIUS).extrude(BASE_HEIGHT)

    web_x = cq.Workplane("XY").box(
        BASE_INNER_RADIUS * 2.0,
        WEB_WIDTH,
        BASE_HEIGHT,
        centered=(True, True, False),
    )
    web_y = cq.Workplane("XY").box(
        WEB_WIDTH,
        BASE_INNER_RADIUS * 2.0,
        BASE_HEIGHT,
        centered=(True, True, False),
    )

    base = ring.union(hub).union(web_x).union(web_y)
    return base.edges("|Z").fillet(0.002)


def _build_upper_core_shape() -> cq.Workplane:
    core = (
        cq.Workplane("XY")
        .circle(CORE_OUTER_RADIUS)
        .circle(CORE_INNER_RADIUS)
        .extrude(CORE_HEIGHT)
    )
    return core.edges(">Z").chamfer(0.001)


def _build_upper_plate_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(
            PLATE_X,
            PLATE_Y,
            PLATE_THICKNESS,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, CORE_HEIGHT))
        .edges("|Z")
        .fillet(0.008)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ring_base_yaw_stage")

    model.material("base_charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("plate_silver", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("core_steel", rgba=(0.54, 0.57, 0.61, 1.0))

    ring_base = model.part("ring_base")
    ring_base.visual(
        mesh_from_cadquery(_build_ring_base_shape(), "ring_base_body"),
        material="base_charcoal",
        name="base_body",
    )
    ring_base.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_OUTER_RADIUS, length=BASE_HEIGHT),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
    )

    upper_plate = model.part("upper_plate")
    upper_plate.visual(
        mesh_from_cadquery(_build_upper_core_shape(), "upper_plate_core"),
        material="core_steel",
        name="upper_core",
    )
    upper_plate.visual(
        mesh_from_cadquery(_build_upper_plate_shape(), "upper_plate_panel"),
        material="plate_silver",
        name="upper_panel",
    )
    upper_plate.inertial = Inertial.from_geometry(
        Box((PLATE_X, PLATE_Y, CORE_HEIGHT + PLATE_THICKNESS)),
        mass=0.95,
        origin=Origin(xyz=(0.0, 0.0, (CORE_HEIGHT + PLATE_THICKNESS) / 2.0)),
    )

    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=ring_base,
        child=upper_plate,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=28.0,
            velocity=2.2,
            lower=-2.8,
            upper=2.8,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    ring_base = object_model.get_part("ring_base")
    upper_plate = object_model.get_part("upper_plate")
    base_yaw = object_model.get_articulation("base_yaw")
    upper_panel = upper_plate.get_visual("upper_panel")

    limits = base_yaw.motion_limits
    ctx.check(
        "yaw joint uses the vertical core axis",
        base_yaw.axis == (0.0, 0.0, 1.0)
        and limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < -1.0
        and limits.upper > 1.0,
        details=f"axis={base_yaw.axis}, limits={limits}",
    )

    with ctx.pose({base_yaw: 0.0}):
        ctx.expect_contact(
            upper_plate,
            ring_base,
            contact_tol=0.001,
            name="upper stage seats on the ring base hub",
        )
        ctx.expect_gap(
            upper_plate,
            ring_base,
            axis="z",
            positive_elem=upper_panel,
            min_gap=0.019,
            max_gap=0.025,
            name="rotary core remains visible beneath the upper plate",
        )
        ctx.expect_overlap(
            upper_plate,
            ring_base,
            axes="xy",
            min_overlap=0.090,
            name="upper plate stays centered over the ring base",
        )

    with ctx.pose({base_yaw: 0.0}):
        rest_aabb = ctx.part_element_world_aabb(upper_plate, elem=upper_panel)
        rest_pos = ctx.part_world_position(upper_plate)
    with ctx.pose({base_yaw: pi / 2.0}):
        turned_aabb = ctx.part_element_world_aabb(upper_plate, elem=upper_panel)
        turned_pos = ctx.part_world_position(upper_plate)

    rest_x = rest_aabb[1][0] - rest_aabb[0][0] if rest_aabb is not None else None
    rest_y = rest_aabb[1][1] - rest_aabb[0][1] if rest_aabb is not None else None
    turned_x = turned_aabb[1][0] - turned_aabb[0][0] if turned_aabb is not None else None
    turned_y = turned_aabb[1][1] - turned_aabb[0][1] if turned_aabb is not None else None

    ctx.check(
        "upper plate yaws in place",
        rest_pos is not None
        and turned_pos is not None
        and abs(rest_pos[0] - turned_pos[0]) < 1e-6
        and abs(rest_pos[1] - turned_pos[1]) < 1e-6
        and abs(rest_pos[2] - turned_pos[2]) < 1e-6
        and rest_x is not None
        and rest_y is not None
        and turned_x is not None
        and turned_y is not None
        and rest_x > rest_y
        and turned_y > turned_x,
        details=(
            f"rest_pos={rest_pos}, turned_pos={turned_pos}, "
            f"rest_xy=({rest_x}, {rest_y}), turned_xy=({turned_x}, {turned_y})"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
