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


GUIDEWAY_LENGTH = 0.54
GUIDEWAY_WIDTH = 0.12
GUIDEWAY_BASE_THICKNESS = 0.014
GUIDEWAY_RAIL_LENGTH = 0.46
GUIDEWAY_RAIL_WIDTH = 0.018
GUIDEWAY_RAIL_HEIGHT = 0.014
GUIDEWAY_RAIL_OFFSET = 0.032
GUIDEWAY_CONTACT_Z = GUIDEWAY_BASE_THICKNESS + GUIDEWAY_RAIL_HEIGHT

TRUCK_LENGTH = 0.15
TRUCK_WIDTH = 0.105
TRUCK_PAD_WIDTH = 0.021
TRUCK_PAD_THICKNESS = 0.006
TRUCK_BODY_HEIGHT = 0.044
TRUCK_TOP_CAP_HEIGHT = 0.012
TRUCK_HEIGHT = TRUCK_PAD_THICKNESS + TRUCK_BODY_HEIGHT + TRUCK_TOP_CAP_HEIGHT
TRUCK_FRONT_BLOCK_LENGTH = 0.062
TRUCK_FRONT_BLOCK_DEPTH = 0.020
TRUCK_FRONT_BLOCK_HEIGHT = 0.040
TRUCK_FRONT_BLOCK_OVERLAP = 0.014
TRUCK_FRONT_FACE_Y = (
    (TRUCK_WIDTH / 2.0)
    - TRUCK_FRONT_BLOCK_OVERLAP
    + TRUCK_FRONT_BLOCK_DEPTH
)

SLIDE_LOWER = -0.16
SLIDE_UPPER = 0.16

SPINDLE_FLANGE_RADIUS = 0.030
SPINDLE_FLANGE_LENGTH = 0.018
SPINDLE_BODY_RADIUS = 0.024
SPINDLE_BODY_LENGTH = 0.056
SPINDLE_NOSE_RADIUS = 0.014
SPINDLE_NOSE_LENGTH = 0.022
SPINDLE_TOOL_FACE_RADIUS = 0.020
SPINDLE_TOOL_FACE_THICKNESS = 0.006
SPINDLE_TOOL_FACE_EMBED = 0.001
SPINDLE_INDEX_TAB_SIZE = (0.018, 0.016, 0.010)
SPINDLE_INDEX_TAB_Y = 0.042
SPINDLE_INDEX_TAB_Z = SPINDLE_BODY_RADIUS + (SPINDLE_INDEX_TAB_SIZE[2] / 2.0) - 0.003
SPINDLE_AXIS_Z = 0.034


def _make_guideway_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(
        GUIDEWAY_LENGTH,
        GUIDEWAY_WIDTH,
        GUIDEWAY_BASE_THICKNESS,
    ).translate((0.0, 0.0, GUIDEWAY_BASE_THICKNESS / 2.0))

    for rail_y in (-GUIDEWAY_RAIL_OFFSET, GUIDEWAY_RAIL_OFFSET):
        rail = cq.Workplane("XY").box(
            GUIDEWAY_RAIL_LENGTH,
            GUIDEWAY_RAIL_WIDTH,
            GUIDEWAY_RAIL_HEIGHT,
        ).translate(
            (
                0.0,
                rail_y,
                GUIDEWAY_BASE_THICKNESS + (GUIDEWAY_RAIL_HEIGHT / 2.0),
            )
        )
        body = body.union(rail)

    center_land = cq.Workplane("XY").box(
        GUIDEWAY_RAIL_LENGTH,
        0.026,
        0.006,
    ).translate((0.0, 0.0, GUIDEWAY_BASE_THICKNESS + 0.003))
    body = body.union(center_land)

    hole_points = [(-0.18, 0.0), (-0.06, 0.0), (0.06, 0.0), (0.18, 0.0)]
    mounting_holes = (
        cq.Workplane("XY")
        .pushPoints(hole_points)
        .circle(0.007)
        .extrude(GUIDEWAY_BASE_THICKNESS + 0.004)
        .translate((0.0, 0.0, -0.002))
    )
    return body.cut(mounting_holes)


def _make_truck_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(
        TRUCK_LENGTH,
        TRUCK_WIDTH,
        TRUCK_BODY_HEIGHT,
    ).translate((0.0, 0.0, TRUCK_PAD_THICKNESS + (TRUCK_BODY_HEIGHT / 2.0)))

    top_cap = cq.Workplane("XY").box(
        0.122,
        0.084,
        TRUCK_TOP_CAP_HEIGHT,
    ).translate(
        (
            0.0,
            0.0,
            TRUCK_PAD_THICKNESS + TRUCK_BODY_HEIGHT + (TRUCK_TOP_CAP_HEIGHT / 2.0),
        )
    )
    body = body.union(top_cap)

    for pad_y in (-GUIDEWAY_RAIL_OFFSET, GUIDEWAY_RAIL_OFFSET):
        pad = cq.Workplane("XY").box(
            TRUCK_LENGTH - 0.01,
            TRUCK_PAD_WIDTH,
            TRUCK_PAD_THICKNESS,
        ).translate((0.0, pad_y, TRUCK_PAD_THICKNESS / 2.0))
        body = body.union(pad)

    front_block_center_y = (
        (TRUCK_WIDTH / 2.0)
        - TRUCK_FRONT_BLOCK_OVERLAP
        + (TRUCK_FRONT_BLOCK_DEPTH / 2.0)
    )
    front_block = cq.Workplane("XY").box(
        TRUCK_FRONT_BLOCK_LENGTH,
        TRUCK_FRONT_BLOCK_DEPTH,
        TRUCK_FRONT_BLOCK_HEIGHT,
    ).translate(
        (
            0.0,
            front_block_center_y,
            TRUCK_PAD_THICKNESS + 0.010 + (TRUCK_FRONT_BLOCK_HEIGHT / 2.0),
        )
    )
    return body.union(front_block)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="guideway_tool_slide")

    model.material("guideway_gray", rgba=(0.58, 0.60, 0.63, 1.0))
    model.material("truck_orange", rgba=(0.86, 0.46, 0.12, 1.0))
    model.material("spindle_metal", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("tool_face_dark", rgba=(0.22, 0.24, 0.27, 1.0))

    guideway = model.part("guideway")
    guideway.visual(
        mesh_from_cadquery(_make_guideway_shape(), "guideway_body"),
        material="guideway_gray",
        name="guideway_body",
    )
    guideway.inertial = Inertial.from_geometry(
        Box((GUIDEWAY_LENGTH, GUIDEWAY_WIDTH, GUIDEWAY_CONTACT_Z)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, GUIDEWAY_CONTACT_Z / 2.0)),
    )

    truck = model.part("truck")
    truck.visual(
        mesh_from_cadquery(_make_truck_shape(), "truck_body"),
        material="truck_orange",
        name="truck_body",
    )
    truck.inertial = Inertial.from_geometry(
        Box((TRUCK_LENGTH, TRUCK_WIDTH, TRUCK_HEIGHT)),
        mass=2.1,
        origin=Origin(xyz=(0.0, 0.0, TRUCK_HEIGHT / 2.0)),
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=SPINDLE_FLANGE_RADIUS, length=SPINDLE_FLANGE_LENGTH),
        origin=Origin(
            xyz=(0.0, SPINDLE_FLANGE_LENGTH / 2.0, 0.0),
            rpy=(-pi / 2.0, 0.0, 0.0),
        ),
        material="spindle_metal",
        name="rear_flange",
    )
    spindle.visual(
        Cylinder(radius=SPINDLE_BODY_RADIUS, length=SPINDLE_BODY_LENGTH),
        origin=Origin(
            xyz=(0.0, SPINDLE_FLANGE_LENGTH + (SPINDLE_BODY_LENGTH / 2.0), 0.0),
            rpy=(-pi / 2.0, 0.0, 0.0),
        ),
        material="spindle_metal",
        name="cartridge_body",
    )
    spindle.visual(
        Cylinder(radius=SPINDLE_NOSE_RADIUS, length=SPINDLE_NOSE_LENGTH),
        origin=Origin(
            xyz=(
                0.0,
                SPINDLE_FLANGE_LENGTH + SPINDLE_BODY_LENGTH + (SPINDLE_NOSE_LENGTH / 2.0),
                0.0,
            ),
            rpy=(-pi / 2.0, 0.0, 0.0),
        ),
        material="spindle_metal",
        name="nose_body",
    )
    spindle.visual(
        Cylinder(radius=SPINDLE_TOOL_FACE_RADIUS, length=SPINDLE_TOOL_FACE_THICKNESS),
        origin=Origin(
            xyz=(
                0.0,
                SPINDLE_FLANGE_LENGTH
                + SPINDLE_BODY_LENGTH
                + SPINDLE_NOSE_LENGTH
                + (SPINDLE_TOOL_FACE_THICKNESS / 2.0)
                - SPINDLE_TOOL_FACE_EMBED,
                0.0,
            ),
            rpy=(-pi / 2.0, 0.0, 0.0),
        ),
        material="tool_face_dark",
        name="tool_face",
    )
    spindle.visual(
        Box(SPINDLE_INDEX_TAB_SIZE),
        origin=Origin(
            xyz=(0.0, SPINDLE_INDEX_TAB_Y, SPINDLE_INDEX_TAB_Z),
        ),
        material="tool_face_dark",
        name="index_tab",
    )
    spindle.inertial = Inertial.from_geometry(
        Cylinder(radius=SPINDLE_FLANGE_RADIUS, length=0.11),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.055, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "guideway_to_truck",
        ArticulationType.PRISMATIC,
        parent=guideway,
        child=truck,
        origin=Origin(xyz=(0.0, 0.0, GUIDEWAY_CONTACT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=SLIDE_LOWER,
            upper=SLIDE_UPPER,
            effort=220.0,
            velocity=0.30,
        ),
    )
    model.articulation(
        "truck_to_spindle",
        ArticulationType.REVOLUTE,
        parent=truck,
        child=spindle,
        origin=Origin(xyz=(0.0, TRUCK_FRONT_FACE_Y, SPINDLE_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-1.4,
            upper=1.4,
            effort=20.0,
            velocity=6.0,
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

    guideway = object_model.get_part("guideway")
    truck = object_model.get_part("truck")
    spindle = object_model.get_part("spindle")
    slide = object_model.get_articulation("guideway_to_truck")
    rotary = object_model.get_articulation("truck_to_spindle")

    ctx.expect_contact(
        truck,
        guideway,
        name="truck sits on the guideway rails at rest",
    )
    ctx.expect_within(
        truck,
        guideway,
        axes="y",
        margin=0.0,
        name="truck stays laterally within guideway width",
    )
    ctx.expect_contact(
        spindle,
        truck,
        name="spindle cartridge seats against the truck mount",
    )

    rest_truck_pos = ctx.part_world_position(truck)
    with ctx.pose({slide: SLIDE_UPPER}):
        ctx.expect_contact(
            truck,
            guideway,
            name="truck remains supported at maximum slide extension",
        )
        extended_truck_pos = ctx.part_world_position(truck)

    ctx.check(
        "truck translates along +x on the guideway",
        rest_truck_pos is not None
        and extended_truck_pos is not None
        and extended_truck_pos[0] > rest_truck_pos[0] + 0.12,
        details=f"rest={rest_truck_pos}, extended={extended_truck_pos}",
    )

    def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
        if aabb is None:
            return None
        low, high = aabb
        return tuple((low[i] + high[i]) / 2.0 for i in range(3))

    with ctx.pose({slide: 0.04, rotary: 0.0}):
        tab_rest = _aabb_center(ctx.part_element_world_aabb(spindle, elem="index_tab"))
    with ctx.pose({slide: 0.04, rotary: 1.2}):
        ctx.expect_contact(
            spindle,
            truck,
            name="spindle stays mounted while rotating",
        )
        tab_rotated = _aabb_center(ctx.part_element_world_aabb(spindle, elem="index_tab"))

    ctx.check(
        "spindle rotates about its local tool axis",
        tab_rest is not None
        and tab_rotated is not None
        and tab_rotated[0] > tab_rest[0] + 0.015
        and abs(tab_rotated[1] - tab_rest[1]) < 0.003,
        details=f"rest={tab_rest}, rotated={tab_rotated}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
