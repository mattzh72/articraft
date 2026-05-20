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


BASE_LENGTH = 0.58
BASE_WIDTH = 0.42
PLINTH_HEIGHT = 0.055
HOUSING_LENGTH = 0.46
HOUSING_WIDTH = 0.34
HOUSING_HEIGHT = 0.08
SUPPORT_RING_OD = 0.33
SUPPORT_RING_ID = 0.214
SUPPORT_RING_HEIGHT = 0.02
BODY_TOP_Z = PLINTH_HEIGHT + HOUSING_HEIGHT + SUPPORT_RING_HEIGHT
WELL_DEPTH = 0.05

ROTARY_DISC_DIAMETER = SUPPORT_RING_OD
ROTARY_DISC_HEIGHT = 0.015
ROTARY_DRUM_DIAMETER = 0.208
ROTARY_DRUM_BOTTOM = 0.0
ROTARY_DRUM_HEIGHT = 0.057
ROTARY_TURRET_DIAMETER = 0.272
ROTARY_TURRET_HEIGHT = 0.03
BRIDGE_WIDTH = 0.23
BRIDGE_DEPTH = 0.19
BRIDGE_BASE_Z = ROTARY_DISC_HEIGHT + ROTARY_TURRET_HEIGHT
BRIDGE_HEIGHT = 0.038
CHEEK_CENTER_X = 0.115
CHEEK_THICKNESS = 0.03
CHEEK_DEPTH = 0.126
CHEEK_BASE_Z = 0.07
CHEEK_HEIGHT = 0.095
TRUNNION_AXIS_Z = 0.145
TRUNNION_SHAFT_RADIUS = 0.018
TRUNNION_BORE_RADIUS = 0.024
OUTER_BOSS_RADIUS = 0.04
OUTER_BOSS_LENGTH = 0.014

TABLE_WIDTH_X = 0.178
TABLE_DEPTH_Y = 0.156
TABLE_THICKNESS = 0.024
TABLE_PLATE_CENTER_Z = 0.058
TABLE_RIB_WIDTH = 0.05
TABLE_RIB_DEPTH = 0.09
TABLE_RIB_HEIGHT = 0.046
TRUNNION_SHAFT_LENGTH = 0.176
TRUNNION_COLLAR_RADIUS = 0.028
TRUNNION_COLLAR_LENGTH = 0.024
TRUNNION_COLLAR_CENTER_X = 0.088


def _rounded_box(length: float, width: float, height: float, radius: float) -> cq.Workplane:
    solid = cq.Workplane("XY").box(length, width, height, centered=(True, True, False))
    if radius > 0.0:
        solid = solid.edges("|Z").fillet(radius)
    return solid


def _build_body_shape() -> cq.Workplane:
    plinth = _rounded_box(BASE_LENGTH, BASE_WIDTH, PLINTH_HEIGHT, 0.02)
    housing = _rounded_box(HOUSING_LENGTH, HOUSING_WIDTH, HOUSING_HEIGHT, 0.016).translate(
        (0.0, 0.0, PLINTH_HEIGHT)
    )
    support_ring = (
        cq.Workplane("XY")
        .circle(SUPPORT_RING_OD / 2.0)
        .circle(SUPPORT_RING_ID / 2.0)
        .extrude(SUPPORT_RING_HEIGHT)
        .translate((0.0, 0.0, PLINTH_HEIGHT + HOUSING_HEIGHT))
    )

    body = plinth.union(housing).union(support_ring)

    front_pocket = (
        cq.Workplane("XY")
        .box(0.24, 0.075, 0.048, centered=(True, True, False))
        .translate((0.0, (HOUSING_WIDTH / 2.0) + 0.017, 0.03))
    )
    rear_pocket = (
        cq.Workplane("XY")
        .box(0.18, 0.07, 0.04, centered=(True, True, False))
        .translate((0.0, -(HOUSING_WIDTH / 2.0) - 0.018, 0.034))
    )
    left_side_relief = (
        cq.Workplane("XY")
        .box(0.08, 0.19, 0.05, centered=(True, True, False))
        .translate(((HOUSING_LENGTH / 2.0) + 0.012, 0.0, 0.028))
    )
    right_side_relief = (
        cq.Workplane("XY")
        .box(0.08, 0.19, 0.05, centered=(True, True, False))
        .translate((-(HOUSING_LENGTH / 2.0) - 0.012, 0.0, 0.028))
    )
    center_well = (
        cq.Workplane("XY")
        .circle(SUPPORT_RING_ID / 2.0)
        .extrude(WELL_DEPTH)
        .translate((0.0, 0.0, BODY_TOP_Z - WELL_DEPTH))
    )

    return body.cut(front_pocket).cut(rear_pocket).cut(left_side_relief).cut(right_side_relief).cut(center_well)


def _build_rotary_carrier_shape() -> cq.Workplane:
    disc = cq.Workplane("XY").circle(ROTARY_DISC_DIAMETER / 2.0).extrude(ROTARY_DISC_HEIGHT)
    drum = (
        cq.Workplane("XY")
        .circle(ROTARY_DRUM_DIAMETER / 2.0)
        .extrude(ROTARY_DRUM_HEIGHT)
        .translate((0.0, 0.0, ROTARY_DRUM_BOTTOM))
    )
    turret = (
        cq.Workplane("XY")
        .circle(ROTARY_TURRET_DIAMETER / 2.0)
        .extrude(ROTARY_TURRET_HEIGHT)
        .translate((0.0, 0.0, ROTARY_DISC_HEIGHT))
    )
    left_cheek = (
        cq.Workplane("XY")
        .box(CHEEK_THICKNESS, CHEEK_DEPTH, CHEEK_HEIGHT, centered=(True, True, False))
        .translate((CHEEK_CENTER_X, 0.0, CHEEK_BASE_Z))
    )
    right_cheek = (
        cq.Workplane("XY")
        .box(CHEEK_THICKNESS, CHEEK_DEPTH, CHEEK_HEIGHT, centered=(True, True, False))
        .translate((-CHEEK_CENTER_X, 0.0, CHEEK_BASE_Z))
    )
    left_rear_brace = (
        cq.Workplane("XY")
        .box(0.05, 0.03, 0.06, centered=(True, True, False))
        .translate((CHEEK_CENTER_X - 0.012, -0.078, 0.057))
    )
    right_rear_brace = (
        cq.Workplane("XY")
        .box(0.05, 0.03, 0.06, centered=(True, True, False))
        .translate((-(CHEEK_CENTER_X - 0.012), -0.078, 0.057))
    )
    left_boss = (
        cq.Workplane("YZ")
        .circle(OUTER_BOSS_RADIUS)
        .extrude(OUTER_BOSS_LENGTH / 2.0, both=True)
        .translate((CHEEK_CENTER_X + (CHEEK_THICKNESS / 2.0) + (OUTER_BOSS_LENGTH / 2.0), 0.0, TRUNNION_AXIS_Z))
    )
    right_boss = (
        cq.Workplane("YZ")
        .circle(OUTER_BOSS_RADIUS)
        .extrude(OUTER_BOSS_LENGTH / 2.0, both=True)
        .translate((-(CHEEK_CENTER_X + (CHEEK_THICKNESS / 2.0) + (OUTER_BOSS_LENGTH / 2.0)), 0.0, TRUNNION_AXIS_Z))
    )

    carrier = (
        disc.union(drum)
        .union(turret)
        .union(left_cheek)
        .union(right_cheek)
        .union(left_rear_brace)
        .union(right_rear_brace)
        .union(left_boss)
        .union(right_boss)
    )

    trunnion_bore = (
        cq.Workplane("YZ")
        .circle(TRUNNION_BORE_RADIUS)
        .extrude(0.4, both=True)
        .translate((0.0, 0.0, TRUNNION_AXIS_Z))
    )

    return carrier.cut(trunnion_bore)


def _build_drive_pod_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.07, 0.05, 0.055, centered=(True, True, False))
        .translate((0.055, -0.122, ROTARY_DISC_HEIGHT))
    )


def _build_work_face_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(TABLE_WIDTH_X, TABLE_DEPTH_Y, TABLE_THICKNESS)
        .translate((0.0, 0.0, TABLE_PLATE_CENTER_Z))
    )
    shaft = cq.Workplane("YZ").circle(TRUNNION_SHAFT_RADIUS).extrude(TRUNNION_SHAFT_LENGTH / 2.0, both=True)
    center_hub = (
        cq.Workplane("XY")
        .box(TABLE_RIB_WIDTH, TABLE_RIB_DEPTH, TABLE_RIB_HEIGHT)
        .translate((0.0, 0.0, 0.02))
    )
    rear_gusset = (
        cq.Workplane("XY")
        .box(0.088, 0.03, 0.04)
        .translate((0.0, -0.02, 0.028))
    )
    left_collar = (
        cq.Workplane("YZ")
        .circle(TRUNNION_COLLAR_RADIUS)
        .extrude(TRUNNION_COLLAR_LENGTH / 2.0, both=True)
        .translate((TRUNNION_COLLAR_CENTER_X, 0.0, 0.0))
    )
    right_collar = (
        cq.Workplane("YZ")
        .circle(TRUNNION_COLLAR_RADIUS)
        .extrude(TRUNNION_COLLAR_LENGTH / 2.0, both=True)
        .translate((-TRUNNION_COLLAR_CENTER_X, 0.0, 0.0))
    )

    table = plate.union(shaft).union(center_hub).union(rear_gusset).union(left_collar).union(right_collar)

    for slot_x in (-0.05, 0.0, 0.05):
        cutter = (
            cq.Workplane("XY")
            .box(0.012, 0.134, 0.006)
            .translate((slot_x, 0.0, TABLE_PLATE_CENTER_Z + (TABLE_THICKNESS / 2.0) - 0.003))
        )
        table = table.cut(cutter)

    return table


def _build_front_reference_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.12, 0.01, 0.008)
        .translate((0.0, (TABLE_DEPTH_Y / 2.0) - 0.01, TABLE_PLATE_CENTER_Z + (TABLE_THICKNESS / 2.0) + 0.004))
    )


def _center_from_aabb(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) / 2.0 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_trunnion_table")

    model.material("body_paint", rgba=(0.18, 0.20, 0.22, 1.0))
    model.material("carrier_gray", rgba=(0.55, 0.57, 0.60, 1.0))
    model.material("machined_steel", rgba=(0.79, 0.81, 0.83, 1.0))
    model.material("pod_dark", rgba=(0.14, 0.15, 0.17, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "body_shell"),
        material="body_paint",
        name="body_shell",
    )
    body.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, BODY_TOP_Z)),
        mass=82.0,
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP_Z / 2.0)),
    )

    rotary = model.part("rotary_carrier")
    rotary.visual(
        Cylinder(radius=ROTARY_DISC_DIAMETER / 2.0, length=ROTARY_DISC_HEIGHT),
        material="carrier_gray",
        origin=Origin(xyz=(0.0, 0.0, ROTARY_DISC_HEIGHT / 2.0)),
        name="carrier_disc",
    )
    rotary.visual(
        Cylinder(radius=ROTARY_DRUM_DIAMETER / 2.0, length=ROTARY_DRUM_HEIGHT),
        material="carrier_gray",
        origin=Origin(xyz=(0.0, 0.0, ROTARY_DRUM_HEIGHT / 2.0)),
        name="carrier_drum",
    )
    rotary.visual(
        Cylinder(radius=ROTARY_TURRET_DIAMETER / 2.0, length=ROTARY_TURRET_HEIGHT),
        material="carrier_gray",
        origin=Origin(xyz=(0.0, 0.0, ROTARY_DISC_HEIGHT + (ROTARY_TURRET_HEIGHT / 2.0))),
        name="carrier_turret",
    )
    rotary.visual(
        Box((0.176, 0.026, 0.024)),
        material="carrier_gray",
        origin=Origin(xyz=(0.0, -0.072, 0.057)),
        name="carrier_spine",
    )
    rotary.visual(
        Box((0.05, 0.04, 0.07)),
        material="carrier_gray",
        origin=Origin(xyz=(CHEEK_CENTER_X - 0.012, -0.055, 0.08)),
        name="left_cheek_brace",
    )
    rotary.visual(
        Box((0.05, 0.04, 0.07)),
        material="carrier_gray",
        origin=Origin(xyz=(-(CHEEK_CENTER_X - 0.012), -0.055, 0.08)),
        name="right_cheek_brace",
    )
    rotary.visual(
        Box((CHEEK_THICKNESS, CHEEK_DEPTH, CHEEK_HEIGHT)),
        material="carrier_gray",
        origin=Origin(xyz=(CHEEK_CENTER_X, 0.0, CHEEK_BASE_Z + (CHEEK_HEIGHT / 2.0))),
        name="left_cheek",
    )
    rotary.visual(
        Box((CHEEK_THICKNESS, CHEEK_DEPTH, CHEEK_HEIGHT)),
        material="carrier_gray",
        origin=Origin(xyz=(-CHEEK_CENTER_X, 0.0, CHEEK_BASE_Z + (CHEEK_HEIGHT / 2.0))),
        name="right_cheek",
    )
    rotary.visual(
        Cylinder(radius=OUTER_BOSS_RADIUS, length=OUTER_BOSS_LENGTH),
        material="carrier_gray",
        origin=Origin(
            xyz=(CHEEK_CENTER_X + (CHEEK_THICKNESS / 2.0) + (OUTER_BOSS_LENGTH / 2.0), 0.0, TRUNNION_AXIS_Z),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        name="left_outer_boss",
    )
    rotary.visual(
        Cylinder(radius=OUTER_BOSS_RADIUS, length=OUTER_BOSS_LENGTH),
        material="carrier_gray",
        origin=Origin(
            xyz=(-(CHEEK_CENTER_X + (CHEEK_THICKNESS / 2.0) + (OUTER_BOSS_LENGTH / 2.0)), 0.0, TRUNNION_AXIS_Z),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        name="right_outer_boss",
    )
    rotary.visual(
        Box((0.07, 0.05, 0.055)),
        material="pod_dark",
        origin=Origin(xyz=(0.055, -0.122, 0.0425)),
        name="index_pod",
    )
    rotary.visual(
        Box((0.001, 0.001, 0.001)),
        material="carrier_gray",
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="carrier_shell",
    )
    rotary.inertial = Inertial.from_geometry(
        Box((ROTARY_DISC_DIAMETER, BRIDGE_DEPTH, 0.22)),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
    )

    work_face = model.part("work_face")
    work_face.visual(
        Box((0.156, 0.054, 0.04)),
        material="machined_steel",
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="trunnion_beam",
    )
    work_face.visual(
        Cylinder(radius=TRUNNION_SHAFT_RADIUS, length=0.022),
        material="machined_steel",
        origin=Origin(xyz=(0.089, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        name="left_trunnion_stub",
    )
    work_face.visual(
        Cylinder(radius=TRUNNION_SHAFT_RADIUS, length=0.022),
        material="machined_steel",
        origin=Origin(xyz=(-0.089, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        name="right_trunnion_stub",
    )
    work_face.visual(
        Box((0.066, 0.094, 0.046)),
        material="machined_steel",
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        name="tilt_pedestal",
    )
    work_face.visual(
        Box((0.088, 0.03, 0.04)),
        material="machined_steel",
        origin=Origin(xyz=(0.0, -0.02, 0.028)),
        name="rear_gusset",
    )
    work_face.visual(
        Box((TABLE_WIDTH_X, TABLE_DEPTH_Y, TABLE_THICKNESS)),
        material="machined_steel",
        origin=Origin(xyz=(0.0, 0.0, TABLE_PLATE_CENTER_Z)),
        name="work_face_shell",
    )
    work_face.visual(
        Box((0.12, 0.01, 0.008)),
        material="carrier_gray",
        origin=Origin(
            xyz=(0.0, (TABLE_DEPTH_Y / 2.0) - 0.01, TABLE_PLATE_CENTER_Z + (TABLE_THICKNESS / 2.0) + 0.004)
        ),
        name="front_reference",
    )
    work_face.inertial = Inertial.from_geometry(
        Box((0.21, 0.17, 0.12)),
        mass=17.0,
        origin=Origin(xyz=(0.0, 0.0, -0.01)),
    )

    model.articulation(
        "body_to_rotary",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rotary,
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-3.14159, upper=3.14159, effort=120.0, velocity=1.6),
    )
    model.articulation(
        "rotary_to_work_face",
        ArticulationType.REVOLUTE,
        parent=rotary,
        child=work_face,
        origin=Origin(xyz=(0.0, 0.0, TRUNNION_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-1.15, upper=1.15, effort=80.0, velocity=1.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    rotary = object_model.get_part("rotary_carrier")
    work_face = object_model.get_part("work_face")
    base_rotation = object_model.get_articulation("body_to_rotary")
    table_tilt = object_model.get_articulation("rotary_to_work_face")
    index_pod = rotary.get_visual("index_pod")
    front_reference = work_face.get_visual("front_reference")

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

    with ctx.pose({base_rotation: 0.0, table_tilt: 0.0}):
        ctx.expect_contact(rotary, body, contact_tol=1e-6, name="rotary carrier is seated on the grounded body")
        ctx.expect_contact(work_face, rotary, contact_tol=1e-6, name="work face is supported by the trunnion carrier")
        ctx.expect_overlap(rotary, body, axes="xy", min_overlap=0.28, name="rotary carrier sits over the body support footprint")
        ctx.expect_gap(work_face, body, axis="z", min_gap=0.10, name="upper work face clears the grounded body")

    with ctx.pose({base_rotation: 0.0, table_tilt: 0.0}):
        pod_rest = _center_from_aabb(ctx.part_element_world_aabb(rotary, elem=index_pod))
        lip_rest = _center_from_aabb(ctx.part_element_world_aabb(work_face, elem=front_reference))
    with ctx.pose({base_rotation: 1.5708, table_tilt: 0.0}):
        pod_quarter_turn = _center_from_aabb(ctx.part_element_world_aabb(rotary, elem=index_pod))
    with ctx.pose({base_rotation: 0.0, table_tilt: 0.75}):
        lip_tilted = _center_from_aabb(ctx.part_element_world_aabb(work_face, elem=front_reference))

    rotary_motion_ok = (
        pod_rest is not None
        and pod_quarter_turn is not None
        and pod_rest[1] < -0.09
        and pod_quarter_turn[0] > 0.09
        and abs(pod_quarter_turn[1]) < 0.07
    )
    ctx.check(
        "lower rotary base turns about the vertical axis",
        rotary_motion_ok,
        details=f"rest={pod_rest}, quarter_turn={pod_quarter_turn}",
    )

    tilt_motion_ok = (
        lip_rest is not None
        and lip_tilted is not None
        and lip_tilted[2] > (lip_rest[2] + 0.02)
        and lip_tilted[1] < (lip_rest[1] - 0.015)
    )
    ctx.check(
        "upper work face positively tilts about the trunnion axis",
        tilt_motion_ok,
        details=f"rest={lip_rest}, tilted={lip_tilted}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
