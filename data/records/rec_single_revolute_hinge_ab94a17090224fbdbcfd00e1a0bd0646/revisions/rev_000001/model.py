from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BLOCK_DEPTH = 0.050
BLOCK_WIDTH = 0.050
BLOCK_HEIGHT = 0.075
HINGE_AXIS_HEIGHT = BLOCK_HEIGHT / 2.0

OUTER_KNUCKLE_RADIUS = 0.0085
OUTER_KNUCKLE_LENGTH = 0.018
LEAF_BARREL_LENGTH = 0.021
AXIAL_GAP = 0.0
PIN_RADIUS = 0.0034
PIN_LENGTH = 2.0 * OUTER_KNUCKLE_LENGTH + LEAF_BARREL_LENGTH + 2.0 * AXIAL_GAP
PIN_BORE_RADIUS = PIN_RADIUS + 0.00025
PIN_HEAD_RADIUS = 0.0050
PIN_HEAD_THICKNESS = 0.0016

LEAF_BARREL_OUTER_RADIUS = 0.0072
LEAF_BARREL_INNER_RADIUS = PIN_BORE_RADIUS
LEAF_HEIGHT = PIN_LENGTH
LEAF_PLATE_LENGTH = 0.095
LEAF_PLATE_THICKNESS = 0.004
LEAF_PLATE_START_X = OUTER_KNUCKLE_RADIUS + 0.0015
BRIDGE_START_X = LEAF_BARREL_OUTER_RADIUS - 0.0018
BRIDGE_END_X = LEAF_PLATE_START_X + 0.006


def _cylinder(radius: float, length: float, *, center: tuple[float, float, float]) -> cq.Workplane:
    cx, cy, cz = center
    return cq.Workplane("XY").circle(radius).extrude(length).translate((cx, cy, cz - length / 2.0))


def _knuckle_center_zs() -> tuple[float, float]:
    upper_center_z = (
        HINGE_AXIS_HEIGHT
        + LEAF_BARREL_LENGTH / 2.0
        + AXIAL_GAP
        + OUTER_KNUCKLE_LENGTH / 2.0
    )
    lower_center_z = (
        HINGE_AXIS_HEIGHT
        - LEAF_BARREL_LENGTH / 2.0
        - AXIAL_GAP
        - OUTER_KNUCKLE_LENGTH / 2.0
    )
    return upper_center_z, lower_center_z


def make_support_body() -> cq.Workplane:
    body = cq.Workplane("XY").box(BLOCK_DEPTH, BLOCK_WIDTH, BLOCK_HEIGHT).translate(
        (-BLOCK_DEPTH / 2.0, 0.0, BLOCK_HEIGHT / 2.0)
    )
    barrel_relief = cq.Workplane("XY").box(0.020, 0.020, LEAF_BARREL_LENGTH + 0.004).translate(
        (-0.010, 0.0, HINGE_AXIS_HEIGHT)
    )
    return body.cut(barrel_relief)


def make_support_knuckles() -> cq.Workplane:
    upper_center_z, lower_center_z = _knuckle_center_zs()
    upper_outer = _cylinder(
        OUTER_KNUCKLE_RADIUS,
        OUTER_KNUCKLE_LENGTH,
        center=(0.0, 0.0, upper_center_z),
    )
    lower_outer = _cylinder(
        OUTER_KNUCKLE_RADIUS,
        OUTER_KNUCKLE_LENGTH,
        center=(0.0, 0.0, lower_center_z),
    )
    upper_bore = _cylinder(
        PIN_BORE_RADIUS,
        OUTER_KNUCKLE_LENGTH + 0.002,
        center=(0.0, 0.0, upper_center_z),
    )
    lower_bore = _cylinder(
        PIN_BORE_RADIUS,
        OUTER_KNUCKLE_LENGTH + 0.002,
        center=(0.0, 0.0, lower_center_z),
    )
    upper = upper_outer.cut(upper_bore)
    lower = lower_outer.cut(lower_bore)
    return cq.Workplane(obj=cq.Compound.makeCompound([upper.val(), lower.val()]))


def make_support_pin() -> cq.Workplane:
    upper_center_z, lower_center_z = _knuckle_center_zs()
    top_head = _cylinder(
        PIN_HEAD_RADIUS,
        PIN_HEAD_THICKNESS,
        center=(0.0, 0.0, upper_center_z + OUTER_KNUCKLE_LENGTH / 2.0 + PIN_HEAD_THICKNESS / 2.0),
    )
    bottom_head = _cylinder(
        PIN_HEAD_RADIUS,
        PIN_HEAD_THICKNESS,
        center=(0.0, 0.0, lower_center_z - OUTER_KNUCKLE_LENGTH / 2.0 - PIN_HEAD_THICKNESS / 2.0),
    )
    return cq.Workplane(obj=cq.Compound.makeCompound([top_head.val(), bottom_head.val()]))


def make_leaf_plate() -> cq.Workplane:
    return cq.Workplane("XY").box(
        LEAF_PLATE_LENGTH,
        LEAF_PLATE_THICKNESS,
        LEAF_HEIGHT,
    ).translate((LEAF_PLATE_START_X + LEAF_PLATE_LENGTH / 2.0, 0.0, 0.0))


def make_leaf_bridge() -> cq.Workplane:
    bridge_length = BRIDGE_END_X - BRIDGE_START_X
    return (
        cq.Workplane("XY")
        .box(bridge_length, LEAF_PLATE_THICKNESS, LEAF_BARREL_LENGTH)
        .translate((BRIDGE_START_X + bridge_length / 2.0, 0.0, 0.0))
    )


def make_leaf_barrel() -> cq.Workplane:
    outer = _cylinder(LEAF_BARREL_OUTER_RADIUS, LEAF_BARREL_LENGTH, center=(0.0, 0.0, 0.0))
    inner = _cylinder(
        LEAF_BARREL_INNER_RADIUS,
        LEAF_BARREL_LENGTH + 0.004,
        center=(0.0, 0.0, 0.0),
    )
    return outer.cut(inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="block_leaf_hinge")

    support_color = model.material("support_paint", rgba=(0.24, 0.26, 0.30, 1.0))
    leaf_color = model.material("leaf_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    pin_color = model.material("pin_steel", rgba=(0.82, 0.83, 0.86, 1.0))

    support = model.part("support")
    support.visual(
        mesh_from_cadquery(make_support_body(), "support_body"),
        material=support_color,
        name="support_body",
    )
    support.visual(
        mesh_from_cadquery(make_support_knuckles(), "support_knuckles"),
        material=support_color,
        name="support_knuckles",
    )
    support.visual(
        mesh_from_cadquery(make_support_pin(), "support_pin"),
        material=pin_color,
        name="support_pin",
    )

    leaf = model.part("leaf")
    leaf.visual(
        mesh_from_cadquery(make_leaf_plate(), "leaf_plate"),
        material=leaf_color,
        name="leaf_plate",
    )
    leaf.visual(
        mesh_from_cadquery(make_leaf_bridge(), "leaf_bridge"),
        material=leaf_color,
        name="leaf_bridge",
    )
    leaf.visual(
        mesh_from_cadquery(make_leaf_barrel(), "leaf_barrel"),
        material=leaf_color,
        name="leaf_barrel",
    )

    model.articulation(
        "support_to_leaf",
        ArticulationType.REVOLUTE,
        parent=support,
        child=leaf,
        origin=Origin(xyz=(0.0, 0.0, HINGE_AXIS_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=1.75,
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

    support = object_model.get_part("support")
    leaf = object_model.get_part("leaf")
    hinge = object_model.get_articulation("support_to_leaf")

    support.get_visual("support_body")
    support.get_visual("support_knuckles")
    support.get_visual("support_pin")
    leaf.get_visual("leaf_plate")
    leaf.get_visual("leaf_bridge")
    leaf.get_visual("leaf_barrel")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            leaf,
            support,
            axis="x",
            min_gap=0.008,
            max_gap=0.012,
            positive_elem="leaf_plate",
            negative_elem="support_body",
            name="leaf plate sits outboard of the grounded block",
        )
        ctx.expect_overlap(
            leaf,
            support,
            axes="xy",
            min_overlap=0.006,
            elem_a="leaf_barrel",
            elem_b="support_knuckles",
            name="leaf barrel stays centered within the grounded hinge half",
        )
        ctx.expect_contact(
            leaf,
            support,
            elem_a="leaf_barrel",
            elem_b="support_knuckles",
            name="moving barrel bears against the grounded knuckle faces at rest",
        )

    rest_aabb = ctx.part_element_world_aabb(leaf, elem="leaf_plate")
    with ctx.pose({hinge: 1.2}):
        open_aabb = ctx.part_element_world_aabb(leaf, elem="leaf_plate")

    def aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((lo + hi) / 2.0 for lo, hi in zip(mins, maxs))

    rest_center = aabb_center(rest_aabb)
    open_center = aabb_center(open_aabb)
    ctx.check(
        "positive hinge rotation swings the leaf open toward +Y",
        rest_center is not None
        and open_center is not None
        and open_center[1] > rest_center[1] + 0.04,
        details=f"rest_center={rest_center}, open_center={open_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
