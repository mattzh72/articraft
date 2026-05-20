from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

BACKPLATE_W = 0.132
BACKPLATE_H = 0.132
BACKPLATE_T = 0.003
HOUSING_W = 0.116
HOUSING_H = 0.116
HOUSING_T = 0.022
HOUSING_CORNER_R = 0.015
HOUSING_EDGE_R = 0.0035
SCREEN_Y = 0.024
SCREEN_W = 0.045
SCREEN_H = 0.018
SCREEN_DEPTH = 0.0018
DIAL_RADIUS = 0.039
DIAL_SEAT_RADIUS = 0.0415
DIAL_SEAT_DEPTH = 0.0042
DIAL_REAR_INSERT = 0.0032
DIAL_TOTAL_T = 0.0115
DIAL_CAP_RADIUS = 0.029
DIAL_CAP_T = 0.0025
JOINT_Z = BACKPLATE_T + HOUSING_T - 0.0006
DIAL_LIMIT = 2.4


def _make_body_shape() -> cq.Workplane:
    backplate = cq.Workplane("XY").box(
        BACKPLATE_W,
        BACKPLATE_H,
        BACKPLATE_T,
        centered=(True, True, False),
    )
    backplate = backplate.edges("|Z").fillet(0.006)

    housing = cq.Workplane("XY").box(
        HOUSING_W,
        HOUSING_H,
        HOUSING_T,
        centered=(True, True, False),
    )
    housing = housing.edges("|Z").fillet(HOUSING_CORNER_R)
    housing = housing.edges("#Z").fillet(HOUSING_EDGE_R)
    housing = housing.translate((0.0, 0.0, BACKPLATE_T))

    front_z = BACKPLATE_T + HOUSING_T
    dial_pocket = (
        cq.Workplane("XY")
        .circle(DIAL_SEAT_RADIUS)
        .extrude(DIAL_SEAT_DEPTH)
        .translate((0.0, 0.0, front_z - DIAL_SEAT_DEPTH))
    )
    screen_pocket = (
        cq.Workplane("XY")
        .center(0.0, SCREEN_Y)
        .slot2D(SCREEN_W, SCREEN_H)
        .extrude(SCREEN_DEPTH)
        .translate((0.0, 0.0, front_z - SCREEN_DEPTH))
    )

    return backplate.union(housing).cut(dial_pocket).cut(screen_pocket)


def _make_screen_shape() -> cq.Workplane:
    front_z = BACKPLATE_T + HOUSING_T
    return (
        cq.Workplane("XY")
        .center(0.0, SCREEN_Y)
        .slot2D(SCREEN_W * 0.92, SCREEN_H * 0.84)
        .extrude(SCREEN_DEPTH)
        .translate((0.0, 0.0, front_z - SCREEN_DEPTH))
    )


def _make_dial_shape() -> cq.Workplane:
    dial = cq.Workplane("XY").circle(DIAL_RADIUS).extrude(DIAL_TOTAL_T)
    dial = dial.translate((0.0, 0.0, -DIAL_REAR_INSERT))
    dial = dial.faces(">Z").edges().fillet(0.0024)
    dial = dial.faces("<Z").edges().fillet(0.0012)

    groove = (
        cq.Workplane("XY")
        .rect(0.0032, 0.0085)
        .extrude(0.0088)
        .translate((DIAL_RADIUS - 0.0013, 0.0, -DIAL_REAR_INSERT + 0.0010))
    )
    for angle in range(0, 360, 15):
        dial = dial.cut(groove.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle))

    return dial


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_thermostat", assets=ASSETS)

    model.material("shell_white", rgba=(0.95, 0.96, 0.94, 1.0))
    model.material("glass_smoke", rgba=(0.12, 0.14, 0.16, 0.92))
    model.material("dial_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("dial_cap", rgba=(0.90, 0.91, 0.89, 1.0))
    model.material("accent_blue", rgba=(0.27, 0.63, 0.90, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_body_shape(), "thermostat_body.obj", assets=ASSETS),
        material="shell_white",
    )
    body.visual(
        mesh_from_cadquery(_make_screen_shape(), "thermostat_screen.obj", assets=ASSETS),
        material="glass_smoke",
    )
    body.inertial = Inertial.from_geometry(
        Box((BACKPLATE_W, BACKPLATE_H, BACKPLATE_T + HOUSING_T)),
        mass=0.36,
        origin=Origin(xyz=(0.0, 0.0, (BACKPLATE_T + HOUSING_T) * 0.5)),
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_cadquery(_make_dial_shape(), "thermostat_dial.obj", assets=ASSETS),
        material="dial_aluminum",
    )
    dial.visual(
        Cylinder(radius=DIAL_CAP_RADIUS, length=DIAL_CAP_T),
        origin=Origin(xyz=(0.0, 0.0, 0.00585)),
        material="dial_cap",
    )
    dial.visual(
        Box((0.0026, 0.016, 0.0012)),
        origin=Origin(xyz=(0.0, DIAL_RADIUS * 0.60, 0.00715)),
        material="accent_blue",
    )
    dial.inertial = Inertial.from_geometry(
        Cylinder(radius=DIAL_RADIUS, length=DIAL_TOTAL_T),
        mass=0.09,
        origin=Origin(xyz=(0.0, 0.0, 0.5 * DIAL_TOTAL_T - DIAL_REAR_INSERT)),
    )

    model.articulation(
        "body_to_dial",
        ArticulationType.REVOLUTE,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, 0.0, JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-DIAL_LIMIT,
            upper=DIAL_LIMIT,
            effort=1.0,
            velocity=5.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=128,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_origin_distance("dial", "body", axes="xy", max_dist=0.001)
    ctx.expect_aabb_overlap("dial", "body", axes="xy", min_overlap=0.076)
    ctx.expect_aabb_gap("dial", "body", axis="z", max_gap=0.001, max_penetration=0.005)

    with ctx.pose(body_to_dial=-DIAL_LIMIT):
        ctx.expect_aabb_overlap("dial", "body", axes="xy", min_overlap=0.074)
        ctx.expect_aabb_gap(
            "dial",
            "body",
            axis="z",
            max_gap=0.001,
            max_penetration=0.005,
        )

    with ctx.pose(body_to_dial=0.85):
        ctx.expect_aabb_overlap("dial", "body", axes="xy", min_overlap=0.074)
        ctx.expect_aabb_gap(
            "dial",
            "body",
            axis="z",
            max_gap=0.001,
            max_penetration=0.005,
        )

    with ctx.pose(body_to_dial=DIAL_LIMIT):
        ctx.expect_aabb_overlap("dial", "body", axes="xy", min_overlap=0.074)
        ctx.expect_aabb_gap(
            "dial",
            "body",
            axis="z",
            max_gap=0.001,
            max_penetration=0.005,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
