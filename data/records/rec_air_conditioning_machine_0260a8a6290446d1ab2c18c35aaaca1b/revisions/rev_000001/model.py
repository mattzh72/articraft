from __future__ import annotations

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


BODY_WIDTH = 0.94
BODY_HEIGHT = 0.295
SIDE_WALL = 0.014

OUTER_PROFILE = [
    (0.000, 0.000),
    (0.000, BODY_HEIGHT),
    (0.090, BODY_HEIGHT),
    (0.145, 0.291),
    (0.188, 0.277),
    (0.220, 0.243),
    (0.236, 0.197),
    (0.233, 0.150),
    (0.218, 0.109),
    (0.193, 0.079),
    (0.156, 0.049),
    (0.106, 0.022),
    (0.050, 0.010),
]

INNER_PROFILE = [
    (0.006, 0.015),
    (0.006, BODY_HEIGHT - 0.018),
    (0.074, BODY_HEIGHT - 0.018),
    (0.126, 0.272),
    (0.166, 0.258),
    (0.196, 0.227),
    (0.209, 0.188),
    (0.206, 0.146),
    (0.194, 0.112),
    (0.172, 0.087),
    (0.134, 0.061),
    (0.088, 0.036),
    (0.046, 0.020),
]

OUTLET_WIDTH = 0.842
OUTLET_HEIGHT = 0.038
OUTLET_DEPTH = 0.116
OUTLET_Y_CENTER = 0.147
OUTLET_Z_CENTER = 0.072
OUTLET_FRONT_Y = OUTLET_Y_CENTER + OUTLET_DEPTH / 2.0
OUTLET_BOTTOM_Z = OUTLET_Z_CENTER - OUTLET_HEIGHT / 2.0

FLAP_WIDTH = OUTLET_WIDTH - 0.004
FLAP_CHORD = 0.062
FLAP_THICKNESS = 0.0032
FLAP_ROD_RADIUS = 0.0028
FLAP_REST_OPEN_RAD = 0.30
FLAP_JOURNAL_LENGTH = 0.037
FLAP_JOURNAL_SIZE = 0.0065
FLAP_JOURNAL_CENTER_X = FLAP_WIDTH / 2.0 + FLAP_JOURNAL_LENGTH / 2.0
FLAP_MOUNT_LENGTH = 0.010
FLAP_MOUNT_DEPTH = 0.060

VANE_HEIGHT = 0.032
VANE_SHAFT_LENGTH = 0.036
VANE_THICKNESS = 0.0055
VANE_CHORD = 0.014
VANE_SHAFT_RADIUS = 0.0024
VANE_Y = 0.154
VANE_Z = OUTLET_Z_CENTER
VANE_X_OFFSET = 0.145


def _extruded_profile(points: list[tuple[float, float]], width: float):
    return cq.Workplane("YZ").polyline(points).close().extrude(width, both=True)


def _build_body_shape():
    outer = _extruded_profile(OUTER_PROFILE, BODY_WIDTH / 2.0)
    inner = _extruded_profile(INNER_PROFILE, (BODY_WIDTH - 2.0 * SIDE_WALL) / 2.0)

    shell = outer.cut(inner)

    outlet_cut = (
        cq.Workplane("XY")
        .box(OUTLET_WIDTH, OUTLET_DEPTH, OUTLET_HEIGHT)
        .translate((0.0, OUTLET_Y_CENTER, OUTLET_Z_CENTER))
    )
    top_seam = (
        cq.Workplane("XY")
        .box(BODY_WIDTH * 0.78, 0.004, 0.018)
        .translate((0.0, 0.184, 0.243))
    )

    return shell.cut(outlet_cut).cut(top_seam)


def _build_flap_shape():
    hinge_rod = cq.Workplane("YZ").circle(FLAP_ROD_RADIUS).extrude(FLAP_WIDTH / 2.0, both=True)
    end_journal = (
        cq.Workplane("XY")
        .box(FLAP_JOURNAL_LENGTH, FLAP_JOURNAL_SIZE, FLAP_JOURNAL_SIZE)
        .translate((FLAP_JOURNAL_CENTER_X, 0.0, 0.0))
    )
    mirrored_end_journal = end_journal.mirror("YZ")
    flap_panel = (
        cq.Workplane("XY")
        .box(FLAP_WIDTH, FLAP_CHORD, FLAP_THICKNESS)
        .translate((0.0, FLAP_CHORD / 2.0, -0.0015))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -FLAP_REST_OPEN_RAD * 180.0 / 3.141592653589793)
    )
    return hinge_rod.union(end_journal).union(mirrored_end_journal).union(flap_panel)


def _build_vane_shape():
    shaft = cq.Workplane("XY").circle(VANE_SHAFT_RADIUS).extrude(VANE_SHAFT_LENGTH / 2.0, both=True)
    blade = (
        cq.Workplane("XY")
        .box(VANE_THICKNESS, VANE_CHORD, VANE_HEIGHT)
        .translate((0.0, VANE_CHORD / 2.0, 0.0))
    )
    return shaft.union(blade)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ductless_mini_split")

    model.material("housing_white", rgba=(0.95, 0.95, 0.93, 1.0))
    model.material("flap_white", rgba=(0.92, 0.92, 0.90, 1.0))
    model.material("vane_gray", rgba=(0.63, 0.66, 0.69, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "body_shell"),
        material="housing_white",
        name="shell",
    )
    flap_mount_y = OUTLET_FRONT_Y - 0.024
    flap_mount_z = OUTLET_BOTTOM_Z + 0.010
    for side, suffix in ((-1.0, "0"), (1.0, "1")):
        body.visual(
            Box((FLAP_MOUNT_LENGTH, FLAP_MOUNT_DEPTH, FLAP_JOURNAL_SIZE)),
            origin=Origin(xyz=(side * (FLAP_JOURNAL_CENTER_X + 0.0235), flap_mount_y, flap_mount_z)),
            material="housing_white",
            name=f"flap_mount_{suffix}",
        )

    for vane_name, vane_x in (("vane_0", -VANE_X_OFFSET), ("vane_1", VANE_X_OFFSET)):
        for z_sign, suffix in ((-1.0, "lower"), (1.0, "upper")):
            mount_length = 0.006 if suffix == "upper" else 0.004
            mount_offset = VANE_SHAFT_LENGTH / 2.0 + mount_length / 2.0
            body.visual(
                Cylinder(radius=0.0032, length=mount_length),
                origin=Origin(
                    xyz=(
                        vane_x,
                        VANE_Y,
                        VANE_Z + z_sign * mount_offset,
                    )
                ),
                material="housing_white",
                name=f"{vane_name}_{suffix}_mount",
            )
        body.visual(
            Box((0.012, 0.064, 0.008)),
            origin=Origin(xyz=(vane_x, 0.182, 0.096)),
            material="housing_white",
            name=f"{vane_name}_roof_bridge",
        )

    flap = model.part("flap")
    flap.visual(
        mesh_from_cadquery(_build_flap_shape(), "outlet_flap"),
        material="flap_white",
        name="blade",
    )

    for vane_name in ("vane_0", "vane_1"):
        vane = model.part(vane_name)
        vane.visual(
            mesh_from_cadquery(_build_vane_shape(), vane_name),
            material="vane_gray",
            name="blade",
        )

    model.articulation(
        "body_to_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flap,
        origin=Origin(xyz=(0.0, flap_mount_y, flap_mount_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.28, upper=0.78, effort=10.0, velocity=1.6),
    )

    model.articulation(
        "body_to_vane_0",
        ArticulationType.REVOLUTE,
        parent=body,
        child="vane_0",
        origin=Origin(xyz=(-VANE_X_OFFSET, VANE_Y, VANE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-0.65, upper=0.65, effort=3.0, velocity=2.5),
    )

    model.articulation(
        "body_to_vane_1",
        ArticulationType.REVOLUTE,
        parent=body,
        child="vane_1",
        origin=Origin(xyz=(VANE_X_OFFSET, VANE_Y, VANE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-0.65, upper=0.65, effort=3.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    flap = object_model.get_part("flap")
    vane_0 = object_model.get_part("vane_0")
    vane_1 = object_model.get_part("vane_1")

    flap_joint = object_model.get_articulation("body_to_flap")
    vane_0_joint = object_model.get_articulation("body_to_vane_0")
    vane_1_joint = object_model.get_articulation("body_to_vane_1")

    ctx.expect_origin_gap(
        flap,
        vane_0,
        axis="y",
        min_gap=0.02,
        max_gap=0.06,
        name="vertical vanes sit behind the outlet flap",
    )
    ctx.expect_origin_distance(
        vane_0,
        vane_1,
        axes="x",
        min_dist=0.24,
        max_dist=0.32,
        name="two vanes are visibly separated across the outlet",
    )

    rest_flap_aabb = ctx.part_world_aabb(flap)
    with ctx.pose({flap_joint: flap_joint.motion_limits.upper}):
        open_flap_aabb = ctx.part_world_aabb(flap)
    ctx.check(
        "flap swings downward when opened",
        rest_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[0][2] < rest_flap_aabb[0][2] - 0.03,
        details=f"rest={rest_flap_aabb}, open={open_flap_aabb}",
    )

    rest_vane_0_aabb = ctx.part_world_aabb(vane_0)
    with ctx.pose({vane_0_joint: vane_0_joint.motion_limits.upper}):
        turned_vane_0_aabb = ctx.part_world_aabb(vane_0)
    ctx.check(
        "vane_0 pivots independently toward one side",
        rest_vane_0_aabb is not None
        and turned_vane_0_aabb is not None
        and turned_vane_0_aabb[0][0] < rest_vane_0_aabb[0][0] - 0.006,
        details=f"rest={rest_vane_0_aabb}, turned={turned_vane_0_aabb}",
    )

    rest_vane_1_aabb = ctx.part_world_aabb(vane_1)
    with ctx.pose({vane_1_joint: vane_1_joint.motion_limits.lower}):
        turned_vane_1_aabb = ctx.part_world_aabb(vane_1)
    ctx.check(
        "vane_1 pivots independently toward the opposite side",
        rest_vane_1_aabb is not None
        and turned_vane_1_aabb is not None
        and turned_vane_1_aabb[1][0] > rest_vane_1_aabb[1][0] + 0.006,
        details=f"rest={rest_vane_1_aabb}, turned={turned_vane_1_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
