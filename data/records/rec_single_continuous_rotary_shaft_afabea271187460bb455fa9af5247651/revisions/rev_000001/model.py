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


RAIL_LENGTH = 0.46
RAIL_WIDTH = 0.08
RAIL_BASE_HEIGHT = 0.012
RAIL_LAND_LENGTH = 0.36
RAIL_LAND_WIDTH = 0.05
RAIL_LAND_HEIGHT = 0.008
RAIL_TOP_Z = RAIL_BASE_HEIGHT + RAIL_LAND_HEIGHT

BLOCK_CENTER_OFFSET_X = 0.11
BLOCK_LENGTH = 0.042
BLOCK_BASE_WIDTH = 0.062
BLOCK_BASE_HEIGHT = 0.014
BLOCK_BODY_WIDTH = 0.048
BLOCK_AXIS_HEIGHT = 0.044
BLOCK_CAP_RADIUS = 0.022
BLOCK_BORE_RADIUS = 0.0105
BLOCK_BOLT_RADIUS = 0.0045
BLOCK_BOLT_OFFSET_X = 0.013

SHAFT_AXIS_Z = RAIL_TOP_Z + BLOCK_AXIS_HEIGHT
SHAFT_JOURNAL_RADIUS = 0.010
SHAFT_LEFT_STUB_RADIUS = 0.008
SHAFT_COLLAR_RADIUS = 0.014
SHAFT_HUB_RADIUS = 0.018
SHAFT_FLANGE_RADIUS = 0.016

MOUNT_SLOT_LENGTH = 0.032
MOUNT_SLOT_WIDTH = 0.014


def x_cylinder(radius: float, x0: float, x1: float) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(x1 - x0).translate((x0, 0.0, 0.0))


def make_rail_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(
        RAIL_LENGTH,
        RAIL_WIDTH,
        RAIL_BASE_HEIGHT,
        centered=(True, True, False),
    )
    land = (
        cq.Workplane("XY")
        .box(
            RAIL_LAND_LENGTH,
            RAIL_LAND_WIDTH,
            RAIL_LAND_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, RAIL_BASE_HEIGHT))
    )
    rail = base.union(land)
    return (
        rail.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(-0.17, 0.0), (0.17, 0.0)])
        .slot2D(MOUNT_SLOT_LENGTH, MOUNT_SLOT_WIDTH, angle=0.0)
        .cutThruAll()
    )


def make_bearing_block_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(
        BLOCK_LENGTH,
        BLOCK_BASE_WIDTH,
        BLOCK_BASE_HEIGHT,
        centered=(True, True, False),
    )
    pedestal = (
        cq.Workplane("XY")
        .box(
            BLOCK_LENGTH * 0.82,
            BLOCK_BODY_WIDTH,
            BLOCK_AXIS_HEIGHT - BLOCK_BASE_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, BLOCK_BASE_HEIGHT))
    )
    cap = (
        cq.Workplane("YZ")
        .circle(BLOCK_CAP_RADIUS)
        .extrude(BLOCK_LENGTH)
        .translate((-BLOCK_LENGTH / 2.0, 0.0, BLOCK_AXIS_HEIGHT))
    )
    block = base.union(pedestal).union(cap)

    mount_holes = (
        cq.Workplane("XY")
        .pushPoints([(-BLOCK_BOLT_OFFSET_X, 0.0), (BLOCK_BOLT_OFFSET_X, 0.0)])
        .circle(BLOCK_BOLT_RADIUS)
        .extrude(BLOCK_BASE_HEIGHT + 0.002)
    )
    bore = (
        cq.Workplane("YZ")
        .circle(BLOCK_BORE_RADIUS)
        .extrude(BLOCK_LENGTH + 0.004)
        .translate((-(BLOCK_LENGTH + 0.004) / 2.0, 0.0, BLOCK_AXIS_HEIGHT))
    )
    return block.cut(mount_holes).cut(bore)


def make_shaft_shape() -> cq.Workplane:
    shaft = x_cylinder(SHAFT_JOURNAL_RADIUS, -0.131, 0.131)
    shaft = shaft.union(x_cylinder(SHAFT_LEFT_STUB_RADIUS, -0.185, -0.149))
    shaft = shaft.union(x_cylinder(SHAFT_COLLAR_RADIUS, -0.149, -0.131))
    shaft = shaft.union(x_cylinder(SHAFT_HUB_RADIUS, -0.025, 0.025))
    shaft = shaft.union(x_cylinder(SHAFT_FLANGE_RADIUS, 0.131, 0.139))

    drive_flat = (
        cq.Workplane("XY")
        .box(0.036, 0.020, 0.010, centered=(True, True, False))
        .translate((-0.167, 0.0, 0.004))
    )
    return shaft.cut(drive_flat)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stepped_rotary_shaft_assembly")

    rail_material = model.material("rail_gray", rgba=(0.34, 0.36, 0.39, 1.0))
    block_material = model.material("bearing_black", rgba=(0.18, 0.19, 0.21, 1.0))
    shaft_material = model.material("machined_steel", rgba=(0.74, 0.76, 0.78, 1.0))

    rail = model.part("rail")
    rail.visual(
        mesh_from_cadquery(make_rail_shape(), "rail"),
        material=rail_material,
        name="rail_body",
    )

    left_block = model.part("left_bearing_block")
    left_block.visual(
        mesh_from_cadquery(make_bearing_block_shape(), "left_bearing_block"),
        material=block_material,
        name="left_block_body",
    )

    right_block = model.part("right_bearing_block")
    right_block.visual(
        mesh_from_cadquery(make_bearing_block_shape(), "right_bearing_block"),
        material=block_material,
        name="right_block_body",
    )

    shaft = model.part("shaft")
    shaft.visual(
        mesh_from_cadquery(make_shaft_shape(), "shaft"),
        material=shaft_material,
        name="shaft_body",
    )

    model.articulation(
        "rail_to_left_bearing_block",
        ArticulationType.FIXED,
        parent=rail,
        child=left_block,
        origin=Origin(xyz=(-BLOCK_CENTER_OFFSET_X, 0.0, RAIL_TOP_Z)),
    )
    model.articulation(
        "rail_to_right_bearing_block",
        ArticulationType.FIXED,
        parent=rail,
        child=right_block,
        origin=Origin(xyz=(BLOCK_CENTER_OFFSET_X, 0.0, RAIL_TOP_Z)),
    )
    model.articulation(
        "rail_to_shaft",
        ArticulationType.CONTINUOUS,
        parent=rail,
        child=shaft,
        origin=Origin(xyz=(0.0, 0.0, SHAFT_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rail = object_model.get_part("rail")
    left_block = object_model.get_part("left_bearing_block")
    right_block = object_model.get_part("right_bearing_block")
    shaft = object_model.get_part("shaft")
    shaft_spin = object_model.get_articulation("rail_to_shaft")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(left_block, rail, name="left_block_seated_on_rail")
    ctx.expect_contact(right_block, rail, name="right_block_seated_on_rail")
    ctx.expect_contact(shaft, left_block, name="shaft_retained_at_left_block")
    ctx.expect_contact(shaft, right_block, name="shaft_retained_at_right_block")
    ctx.expect_overlap(
        shaft,
        left_block,
        axes="x",
        min_overlap=0.04,
        name="shaft_runs_through_left_block",
    )
    ctx.expect_overlap(
        shaft,
        right_block,
        axes="x",
        min_overlap=0.04,
        name="shaft_runs_through_right_block",
    )
    ctx.expect_gap(
        shaft,
        rail,
        axis="z",
        min_gap=0.024,
        name="shaft_clears_rail",
    )

    limits = shaft_spin.motion_limits
    joint_ok = (
        shaft_spin.articulation_type == ArticulationType.CONTINUOUS
        and shaft_spin.axis == (1.0, 0.0, 0.0)
        and limits is not None
        and limits.lower is None
        and limits.upper is None
        and shaft_spin.origin.xyz == (0.0, 0.0, SHAFT_AXIS_Z)
    )
    ctx.check(
        "shaft_is_one_continuous_revolute_axis",
        joint_ok,
        details="shaft must be a continuous joint centered on the supported X-axis centerline",
    )

    with ctx.pose({shaft_spin: 1.2}):
        ctx.expect_contact(shaft, left_block, name="shaft_stays_supported_at_left_block_when_spun")
        ctx.expect_contact(shaft, right_block, name="shaft_stays_supported_at_right_block_when_spun")
        ctx.expect_gap(
            shaft,
            rail,
            axis="z",
            min_gap=0.024,
            name="shaft_clears_rail_when_spun",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
