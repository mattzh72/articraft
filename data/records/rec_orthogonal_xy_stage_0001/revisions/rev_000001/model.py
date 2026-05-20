from __future__ import annotations

from pathlib import Path

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
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
MESH_DIR = ASSETS.ensure_mesh_dir()


# >>> USER_CODE_START
BASE_L = 0.280
BASE_W = 0.180
BASE_T = 0.022

X_RAIL_L = 0.240
X_RAIL_W = 0.018
X_RAIL_H = 0.012
X_RAIL_Y = 0.055

X_SHOE_L = 0.070
X_SHOE_W = 0.026
X_SHOE_H_VIS = 0.010
X_SHOE_H_COL = 0.009
X_BRIDGE_L = 0.125
X_BRIDGE_W = 0.145
X_BRIDGE_T = 0.012
X_PAD_L = 0.102
X_PAD_W = 0.092
X_PAD_T = 0.010

Y_RAIL_L = 0.130
Y_RAIL_W = 0.014
Y_RAIL_H = 0.008
Y_RAIL_X = 0.032

Y_SHOE_L = 0.024
Y_SHOE_W = 0.055
Y_SHOE_H_VIS = 0.008
Y_SHOE_H_COL = 0.007
Y_TOP_L = 0.140
Y_TOP_W = 0.112
Y_TOP_T = 0.012
Y_PAD_L = 0.090
Y_PAD_W = 0.068
Y_PAD_T = 0.008

X_JOINT_Z = (BASE_T / 2.0) + X_RAIL_H + (X_SHOE_H_VIS / 2.0)
Y_RAIL_CENTER_Z = (X_BRIDGE_T / 2.0) + X_PAD_T + (Y_RAIL_H / 2.0)
Y_JOINT_Z = Y_RAIL_CENTER_Z + (Y_RAIL_H / 2.0) + (Y_SHOE_H_VIS / 2.0)

OBJ_FILENAMES = (
    "stage_base_body.obj",
    "stage_x_rails.obj",
    "stage_x_carriage.obj",
    "stage_y_rails.obj",
    "stage_top_stage.obj",
)


def _base_body_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(BASE_L, BASE_W, BASE_T)
        .edges("|Z")
        .fillet(0.004)
        .faces(">Z")
        .workplane()
        .rect(0.220, 0.120, forConstruction=True)
        .vertices()
        .circle(0.004)
        .cutBlind(-0.006)
    )
    return body


def _x_rails_shape() -> cq.Workplane:
    rail = cq.Workplane("XY").box(X_RAIL_L, X_RAIL_W, X_RAIL_H).edges("|X").fillet(0.0015)
    rail_z = (BASE_T / 2.0) + (X_RAIL_H / 2.0)
    left = rail.translate((0.0, X_RAIL_Y, rail_z))
    right = rail.translate((0.0, -X_RAIL_Y, rail_z))
    return left.union(right)


def _x_carriage_shape() -> cq.Workplane:
    shoe = cq.Workplane("XY").box(X_SHOE_L, X_SHOE_W, X_SHOE_H_VIS).edges("|X").fillet(0.0012)
    left_shoe = shoe.translate((0.0, X_RAIL_Y, 0.0))
    right_shoe = shoe.translate((0.0, -X_RAIL_Y, 0.0))

    bridge = (
        cq.Workplane("XY")
        .box(X_BRIDGE_L, X_BRIDGE_W, X_BRIDGE_T)
        .edges("|Z")
        .fillet(0.003)
        .faces(">Z")
        .workplane()
        .rect(0.090, 0.100)
        .cutBlind(-0.0035)
        .translate((0.0, 0.0, (X_SHOE_H_VIS / 2.0) + (X_BRIDGE_T / 2.0)))
    )

    pad = (
        cq.Workplane("XY")
        .box(X_PAD_L, X_PAD_W, X_PAD_T)
        .edges("|Z")
        .fillet(0.002)
        .translate(
            (
                0.0,
                0.0,
                (X_SHOE_H_VIS / 2.0) + X_BRIDGE_T + (X_PAD_T / 2.0),
            )
        )
    )

    return left_shoe.union(right_shoe).union(bridge).union(pad)


def _y_rails_shape() -> cq.Workplane:
    rail = cq.Workplane("XY").box(Y_RAIL_W, Y_RAIL_L, Y_RAIL_H).edges("|Y").fillet(0.0012)
    left = rail.translate((Y_RAIL_X, 0.0, Y_RAIL_CENTER_Z))
    right = rail.translate((-Y_RAIL_X, 0.0, Y_RAIL_CENTER_Z))
    return left.union(right)


def _y_stage_shape() -> cq.Workplane:
    shoe = cq.Workplane("XY").box(Y_SHOE_L, Y_SHOE_W, Y_SHOE_H_VIS).edges("|Y").fillet(0.001)
    left_shoe = shoe.translate((Y_RAIL_X, 0.0, 0.0))
    right_shoe = shoe.translate((-Y_RAIL_X, 0.0, 0.0))

    top = (
        cq.Workplane("XY")
        .box(Y_TOP_L, Y_TOP_W, Y_TOP_T)
        .edges("|Z")
        .fillet(0.003)
        .faces(">Z")
        .workplane()
        .pushPoints([(-0.038, 0.0), (0.038, 0.0)])
        .slot2D(0.024, 0.008, 90)
        .cutThruAll()
        .translate((0.0, 0.0, (Y_SHOE_H_VIS / 2.0) + (Y_TOP_T / 2.0)))
    )

    pad = (
        cq.Workplane("XY")
        .box(Y_PAD_L, Y_PAD_W, Y_PAD_T)
        .edges("|Z")
        .fillet(0.002)
        .translate((0.0, 0.0, (Y_SHOE_H_VIS / 2.0) + Y_TOP_T + (Y_PAD_T / 2.0)))
    )

    return left_shoe.union(right_shoe).union(top).union(pad)


def _add_mesh_visual(part, shape: cq.Workplane, filename: str, material: str) -> None:
    part.visual(mesh_from_cadquery(shape, filename, assets=ASSETS), material=material)


def _obj_path(filename: str) -> Path:
    return Path(MESH_DIR) / filename


def _parse_face_indices(tokens: list[str], vertex_count: int) -> list[int]:
    indices: list[int] = []
    for token in tokens:
        raw = token.split("/")[0]
        if not raw:
            continue
        idx = int(raw)
        if idx < 0:
            idx = vertex_count + idx + 1
        indices.append(idx - 1)
    return indices


def _triangles_from_obj(
    path: Path,
) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]]]:
    vertices: list[tuple[float, float, float]] = []
    triangles: list[tuple[int, int, int]] = []
    with path.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            if line.startswith("v "):
                _, xs, ys, zs, *_ = line.split()
                vertices.append((float(xs), float(ys), float(zs)))
                continue
            if line.startswith("f "):
                face = _parse_face_indices(line.split()[1:], len(vertices))
                for i in range(1, len(face) - 1):
                    triangles.append((face[0], face[i], face[i + 1]))
    return vertices, triangles


def _triangle_area2(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    c: tuple[float, float, float],
) -> float:
    ab = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
    ac = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
    cross = (
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    )
    return (cross[0] * cross[0]) + (cross[1] * cross[1]) + (cross[2] * cross[2])


def _assert_obj_closed_and_nondegenerate(path: Path) -> None:
    vertices, triangles = _triangles_from_obj(path)
    assert vertices, f"{path.name}: OBJ has no vertices"
    assert triangles, f"{path.name}: OBJ has no faces"

    edge_counts: dict[tuple[int, int], int] = {}
    kept_triangles = 0
    for tri in triangles:
        if len(set(tri)) != 3:
            continue
        a, b, c = (vertices[tri[0]], vertices[tri[1]], vertices[tri[2]])
        if _triangle_area2(a, b, c) <= 1e-14:
            continue
        kept_triangles += 1
        for start, end in ((tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])):
            key = (start, end) if start < end else (end, start)
            edge_counts[key] = edge_counts.get(key, 0) + 1

    assert kept_triangles > 0, f"{path.name}: OBJ has no nondegenerate triangles"
    bad_edges = [edge for edge, count in edge_counts.items() if count != 2]
    assert not bad_edges, f"{path.name}: non-closed mesh edges detected"


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_xy_stage", assets=ASSETS)

    model.material("anodized_black", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("ground_steel", rgba=(0.72, 0.74, 0.78, 1.0))
    model.material("machined_gray", rgba=(0.60, 0.62, 0.66, 1.0))
    model.material("plate_blue", rgba=(0.41, 0.47, 0.60, 1.0))

    base = model.part("base")
    _add_mesh_visual(base, _base_body_shape(), OBJ_FILENAMES[0], "anodized_black")
    _add_mesh_visual(base, _x_rails_shape(), OBJ_FILENAMES[1], "ground_steel")



    base.inertial = Inertial.from_geometry(Box((BASE_L, BASE_W, BASE_T)), mass=4.2)

    x_stage = model.part("x_stage")
    _add_mesh_visual(x_stage, _x_carriage_shape(), OBJ_FILENAMES[2], "machined_gray")
    _add_mesh_visual(x_stage, _y_rails_shape(), OBJ_FILENAMES[3], "ground_steel")






    x_stage.inertial = Inertial.from_geometry(
        Box((X_BRIDGE_L, X_BRIDGE_W, 0.032)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
    )

    y_stage = model.part("y_stage")
    _add_mesh_visual(y_stage, _y_stage_shape(), OBJ_FILENAMES[4], "plate_blue")




    y_stage.inertial = Inertial.from_geometry(
        Box((Y_TOP_L, Y_TOP_W, 0.026)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
    )

    model.articulation(
        "base_to_x",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_stage,
        origin=Origin(xyz=(0.0, 0.0, X_JOINT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.060,
            upper=0.060,
            effort=120.0,
            velocity=0.25,
        ),
    )
    model.articulation(
        "x_to_y",
        ArticulationType.PRISMATIC,
        parent=x_stage,
        child=y_stage,
        origin=Origin(xyz=(0.0, 0.0, Y_JOINT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.035,
            upper=0.035,
            effort=90.0,
            velocity=0.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=128,
        overlap_tol=0.0005,
        overlap_volume_tol=0.0,
    )
    ctx.expect_joint_motion_axis(
        "base_to_x",
        "x_stage",
        world_axis="x",
        direction="positive",
        min_delta=0.01,
    )
    ctx.expect_joint_motion_axis(
        "x_to_y",
        "y_stage",
        world_axis="y",
        direction="positive",
        min_delta=0.01,
    )
    ctx.expect_origin_distance("x_stage", "base", axes="xy", max_dist=0.02)
    ctx.expect_origin_distance("y_stage", "x_stage", axes="xy", max_dist=0.02)
    ctx.expect_aabb_overlap("x_stage", "base", axes="xy", min_overlap=0.08)
    ctx.expect_aabb_overlap("y_stage", "x_stage", axes="xy", min_overlap=0.07)
    ctx.expect_aabb_overlap("y_stage", "base", axes="xy", min_overlap=0.07)
    ctx.expect_aabb_gap("x_stage", "base", axis="z", max_gap=0.02, max_penetration=0.002)
    ctx.expect_aabb_gap("y_stage", "x_stage", axis="z", max_gap=0.02, max_penetration=0.006)
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
