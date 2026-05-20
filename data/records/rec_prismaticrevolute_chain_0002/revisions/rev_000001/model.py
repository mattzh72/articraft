from __future__ import annotations

from pathlib import Path

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.mesh_dir
MESH_DIR.mkdir(parents=True, exist_ok=True)
# >>> USER_CODE_START
# In sdk_hybrid, author visual meshes with cadquery + mesh_from_cadquery.
from math import cos, sin

RAIL_LENGTH = 0.42
RAIL_DEPTH = 0.04
RAIL_HEIGHT = 0.05
RAIL_CLEARANCE = 0.003

CARRIAGE_WIDTH = 0.09
CARRIAGE_BODY_DEPTH = 0.024
CARRIAGE_BODY_HEIGHT = 0.032
CARRIAGE_UPRIGHT_WIDTH = 0.05
CARRIAGE_UPRIGHT_DEPTH = 0.012
CARRIAGE_UPRIGHT_HEIGHT = 0.072
CARRIAGE_UPRIGHT_CENTER_Y = 0.03
CARRIAGE_UPRIGHT_CENTER_Z = 0.016
CARRIAGE_ARM_WIDTH = 0.032
CARRIAGE_ARM_DEPTH = 0.016
CARRIAGE_ARM_HEIGHT = 0.012
CARRIAGE_ARM_CENTER_Y = 0.034
CARRIAGE_ARM_CENTER_Z = 0.046

DISPLAY_WIDTH = 0.27
DISPLAY_HEIGHT = 0.18
DISPLAY_THICKNESS = 0.012

SLIDE_LIMIT = 0.14
HINGE_Y = 0.045
HINGE_Z = 0.048
TILT_LIMIT = 0.55

RAIL_MASS = 1.6
CARRIAGE_MASS = 0.7
DISPLAY_MASS = 1.8


def _require_cadquery():
    import cadquery as cq

    return cq


def _build_rail_shape(cq):
    rail = (
        cq.Workplane("YZ")
        .rect(RAIL_DEPTH, RAIL_HEIGHT)
        .extrude(RAIL_LENGTH / 2.0, both=True)
        .edges("|X")
        .fillet(0.0035)
    )
    rail = (
        rail.faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .slot2D(RAIL_LENGTH * 0.78, RAIL_HEIGHT * 0.34, 0)
        .cutBlind(-0.01)
    )
    rail = (
        rail.faces("<Y")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(-RAIL_LENGTH * 0.28, 0.0), (RAIL_LENGTH * 0.28, 0.0)])
        .slot2D(0.05, 0.008, 0)
        .cutBlind(-0.006)
    )
    return rail


def _build_carriage_shape(cq):
    body = (
        cq.Workplane("XY")
        .box(CARRIAGE_WIDTH, CARRIAGE_BODY_DEPTH, CARRIAGE_BODY_HEIGHT)
        .translate((0.0, CARRIAGE_BODY_DEPTH / 2.0, 0.0))
    )
    body = (
        body.faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .slot2D(CARRIAGE_WIDTH * 0.58, 0.01, 0)
        .cutBlind(-0.012)
    )
    upright = (
        cq.Workplane("XY")
        .box(
            CARRIAGE_UPRIGHT_WIDTH,
            CARRIAGE_UPRIGHT_DEPTH,
            CARRIAGE_UPRIGHT_HEIGHT,
        )
        .translate(
            (
                0.0,
                CARRIAGE_UPRIGHT_CENTER_Y,
                CARRIAGE_UPRIGHT_CENTER_Z,
            )
        )
    )
    arm = (
        cq.Workplane("XY")
        .box(CARRIAGE_ARM_WIDTH, CARRIAGE_ARM_DEPTH, CARRIAGE_ARM_HEIGHT)
        .translate((0.0, CARRIAGE_ARM_CENTER_Y, CARRIAGE_ARM_CENTER_Z))
    )
    hinge_boss = (
        cq.Workplane("YZ")
        .circle(0.008)
        .extrude(CARRIAGE_UPRIGHT_WIDTH / 2.0, both=True)
        .translate((0.0, HINGE_Y - 0.006, HINGE_Z))
    )
    return body.union(upright).union(arm).union(hinge_boss)


def _build_display_shape(cq):
    plate = (
        cq.Workplane("XY")
        .box(DISPLAY_WIDTH, DISPLAY_THICKNESS, DISPLAY_HEIGHT)
        .translate((0.0, DISPLAY_THICKNESS / 2.0, -DISPLAY_HEIGHT / 2.0))
        .edges("|Y")
        .fillet(0.004)
    )
    central_pad = (
        cq.Workplane("XY")
        .box(DISPLAY_WIDTH * 0.44, DISPLAY_THICKNESS * 1.7, DISPLAY_HEIGHT * 0.34)
        .translate((0.0, DISPLAY_THICKNESS * 0.85, -DISPLAY_HEIGHT * 0.42))
    )
    top_reinforcement = (
        cq.Workplane("XY")
        .box(DISPLAY_WIDTH * 0.7, DISPLAY_THICKNESS * 1.3, 0.018)
        .translate((0.0, DISPLAY_THICKNESS * 0.65, -0.012))
    )
    lower_lip = (
        cq.Workplane("XY")
        .box(DISPLAY_WIDTH * 0.36, DISPLAY_THICKNESS * 1.5, 0.024)
        .translate((0.0, DISPLAY_THICKNESS * 0.75, -DISPLAY_HEIGHT + 0.02))
    )
    return plate.union(central_pad).union(top_reinforcement).union(lower_lip)


def _obj_topology_stats(path: Path):
    vertices = []
    faces = []
    for raw_line in path.read_text().splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("v "):
            _, xs, ys, zs = line.split(maxsplit=3)
            vertices.append((float(xs), float(ys), float(zs)))
            continue
        if not line.startswith("f "):
            continue
        items = []
        for token in line.split()[1:]:
            index = int(token.split("/")[0])
            if index < 0:
                index = len(vertices) + index
            else:
                index -= 1
            items.append(index)
        faces.append(items)

    def triangle_area2(triangle):
        ax, ay, az = vertices[triangle[0]]
        bx, by, bz = vertices[triangle[1]]
        cx, cy, cz = vertices[triangle[2]]
        abx, aby, abz = bx - ax, by - ay, bz - az
        acx, acy, acz = cx - ax, cy - ay, cz - az
        cxp = aby * acz - abz * acy
        cyp = abz * acx - abx * acz
        czp = abx * acy - aby * acx
        return cxp * cxp + cyp * cyp + czp * czp

    edge_counts = {}
    degenerate_triangles = 0
    for face in faces:
        if len(face) < 3:
            continue
        for index, start in enumerate(face):
            end = face[(index + 1) % len(face)]
            key = tuple(sorted((start, end)))
            edge_counts[key] = edge_counts.get(key, 0) + 1
        for index in range(1, len(face) - 1):
            triangle = (face[0], face[index], face[index + 1])
            if len({triangle[0], triangle[1], triangle[2]}) < 3:
                degenerate_triangles += 1
                continue
            if triangle_area2(triangle) <= 1e-14:
                degenerate_triangles += 1

    boundary_edges = sum(1 for count in edge_counts.values() if count == 1)
    nonmanifold_edges = sum(1 for count in edge_counts.values() if count > 2)
    return {
        "vertex_count": len(vertices),
        "face_count": len(faces),
        "boundary_edges": boundary_edges,
        "nonmanifold_edges": nonmanifold_edges,
        "degenerate_triangles": degenerate_triangles,
    }


def _display_center_world(slide_x: float, tilt_angle: float):
    local_y = DISPLAY_THICKNESS / 2.0
    local_z = -DISPLAY_HEIGHT / 2.0
    world_y = (
        RAIL_DEPTH / 2.0
        + RAIL_CLEARANCE
        + HINGE_Y
        + local_y * cos(tilt_angle)
        - local_z * sin(tilt_angle)
    )
    world_z = HINGE_Z + local_y * sin(tilt_angle) + local_z * cos(tilt_angle)
    return (
        slide_x,
        world_y,
        world_z,
    )


def build_object_model() -> ArticulatedObject:
    cq = _require_cadquery()

    model = ArticulatedObject(name="sliding_monitor_mount", assets=ASSETS)
    model.material("anodized_aluminum", rgba=(0.73, 0.75, 0.79, 1.0))
    model.material("powder_coat", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("display_black", rgba=(0.09, 0.09, 0.1, 1.0))

    rail = model.part("rail")
    rail.visual(
        mesh_from_cadquery(_build_rail_shape(cq), MESH_DIR / "rail.obj"),
        material="anodized_aluminum",
    )

    rail.inertial = Inertial.from_geometry(
        Box((RAIL_LENGTH, RAIL_DEPTH, RAIL_HEIGHT)),
        mass=RAIL_MASS,
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_build_carriage_shape(cq), MESH_DIR / "carriage.obj"),
        material="powder_coat",
    )



    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_WIDTH, 0.048, 0.082)),
        mass=CARRIAGE_MASS,
        origin=Origin(xyz=(0.0, 0.024, 0.015)),
    )

    display_plate = model.part("display_plate")
    display_plate.visual(
        mesh_from_cadquery(_build_display_shape(cq), MESH_DIR / "display_plate.obj"),
        material="display_black",
    )

    display_plate.inertial = Inertial.from_geometry(
        Box((DISPLAY_WIDTH, DISPLAY_THICKNESS, DISPLAY_HEIGHT)),
        mass=DISPLAY_MASS,
        origin=Origin(xyz=(0.0, DISPLAY_THICKNESS / 2.0, -DISPLAY_HEIGHT / 2.0)),
    )

    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent="rail",
        child="carriage",
        origin=Origin(xyz=(0.0, RAIL_DEPTH / 2.0 + RAIL_CLEARANCE, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-SLIDE_LIMIT,
            upper=SLIDE_LIMIT,
            effort=120.0,
            velocity=0.25,
        ),
    )
    model.articulation(
        "carriage_to_display",
        ArticulationType.REVOLUTE,
        parent="carriage",
        child="display_plate",
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=TILT_LIMIT,
            effort=30.0,
            velocity=1.2,
        ),
    )

    model.meta["design"] = {
        "rail_length": RAIL_LENGTH,
        "rail_depth": RAIL_DEPTH,
        "rail_height": RAIL_HEIGHT,
        "slide_limit": SLIDE_LIMIT,
        "tilt_limit": TILT_LIMIT,
        "clearance": RAIL_CLEARANCE,
        "hinge_y": HINGE_Y,
        "hinge_z": HINGE_Z,
        "display_width": DISPLAY_WIDTH,
        "display_height": DISPLAY_HEIGHT,
        "display_thickness": DISPLAY_THICKNESS,
        "carriage_width": CARRIAGE_WIDTH,
        "mesh_names": ("rail.obj", "carriage.obj", "display_plate.obj"),
    }

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
    )
    ctx.expect_origin_distance("carriage", "rail", axes="xy", max_dist=0.04)
    ctx.expect_origin_distance("display_plate", "carriage", axes="xy", max_dist=0.08)
    ctx.expect_origin_distance("display_plate", "rail", axes="xy", max_dist=0.08)
    ctx.expect_joint_motion_axis(
        "rail_to_carriage",
        "carriage",
        world_axis="x",
        direction="positive",
        min_delta=0.08,
    )
    ctx.expect_joint_motion_axis(
        "carriage_to_display",
        "display_plate",
        world_axis="y",
        direction="positive",
        min_delta=0.02,
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
