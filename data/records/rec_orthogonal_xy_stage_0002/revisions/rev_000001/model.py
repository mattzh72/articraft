from __future__ import annotations

from contextlib import nullcontext
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
BASE_L = 0.28
BASE_W = 0.18
BASE_T = 0.012
X_RAIL_LEN = 0.22
X_RAIL_SPACING = 0.11
X_RAIL_W = 0.014
X_RAIL_H = 0.008
X_TRAVEL = 0.06

X_CARRIAGE_L = 0.14
X_CARRIAGE_W = 0.13
X_CARRIAGE_T = 0.012
X_BLOCK_L = 0.028
X_BLOCK_W = 0.024
X_BLOCK_H = 0.008
X_BLOCK_SPACING_X = 0.074

Y_RAIL_LEN = 0.16
Y_RAIL_SPACING = 0.08
Y_RAIL_W = 0.014
Y_RAIL_H = 0.008
Y_TRAVEL = 0.045

Y_STAGE_L = 0.12
Y_STAGE_W = 0.10
Y_STAGE_T = 0.01
Y_BLOCK_L = 0.024
Y_BLOCK_W = 0.028
Y_BLOCK_H = 0.008
Y_BLOCK_SPACING_Y = 0.078
TOOL_MOUNT_W = 0.038
TOOL_MOUNT_H = 0.008

X_JOINT_Z = BASE_T / 2.0 + X_RAIL_H
Y_JOINT_Z = X_BLOCK_H + X_CARRIAGE_T + Y_RAIL_H

TOPOLOGY_QC_MESHES = (
    "base_plate.obj",
    "x_carriage_plate.obj",
    "y_stage_plate.obj",
)


def _merge_shapes(*shapes: cq.Workplane) -> cq.Workplane:
    merged = shapes[0]
    for shape in shapes[1:]:
        merged = merged.union(shape)
    return merged


def _add_visual_mesh(part, shape: cq.Workplane, filename: str, material: str) -> None:
    part.visual(mesh_from_cadquery(shape, filename, assets=ASSETS), material=material)


def _plate_with_counterbores(
    length: float,
    width: float,
    thickness: float,
    *,
    corner_radius: float,
    hole_spacing_x: float,
    hole_spacing_y: float,
    hole_diameter: float,
    cbore_diameter: float,
    cbore_depth: float,
) -> cq.Workplane:
    plate = cq.Workplane("XY").box(length, width, thickness)
    plate = plate.edges("|Z").fillet(corner_radius)
    return (
        plate.faces(">Z")
        .workplane()
        .rect(hole_spacing_x, hole_spacing_y, forConstruction=True)
        .vertices()
        .cboreHole(hole_diameter, cbore_diameter, cbore_depth, depth=None)
    )


def _rail_bar(length_x: float, width_y: float, height_z: float) -> cq.Workplane:
    return cq.Workplane("XY").box(length_x, width_y, height_z).edges("|Z").fillet(0.0012)


def _bearing_block(length_x: float, width_y: float, height_z: float) -> cq.Workplane:
    block = cq.Workplane("XY").box(length_x, width_y, height_z).edges("|Z").fillet(0.001)
    return (
        block.faces(">Z")
        .workplane()
        .rect(length_x * 0.45, width_y * 0.45, forConstruction=True)
        .vertices()
        .cboreHole(0.0035, 0.0065, 0.0018, depth=None)
    )


def _pose_context(ctx: TestContext, **pose_values):
    pose_method = getattr(ctx, "pose", None)
    if pose_method is None:
        return nullcontext()
    for args, kwargs in (
        ((), pose_values),
        ((pose_values,), {}),
        ((), {"poses": pose_values}),
    ):
        try:
            return pose_method(*args, **kwargs)
        except TypeError:
            continue
    return pose_method(pose_values)


def _load_obj_mesh(obj_path: Path):
    vertices: list[tuple[float, float, float]] = []
    triangles: list[tuple[int, int, int]] = []
    with obj_path.open("r", encoding="utf-8") as obj_file:
        for raw_line in obj_file:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            if line.startswith("v "):
                _, x_str, y_str, z_str = line.split()[:4]
                vertices.append((float(x_str), float(y_str), float(z_str)))
            elif line.startswith("f "):
                indices = [int(token.split("/")[0]) - 1 for token in line.split()[1:]]
                if len(indices) < 3:
                    continue
                for idx in range(1, len(indices) - 1):
                    triangles.append((indices[0], indices[idx], indices[idx + 1]))
    return vertices, triangles


def _assert_closed_obj_mesh(obj_path: Path) -> None:
    vertices, triangles = _load_obj_mesh(obj_path)
    assert triangles, f"{obj_path.name} did not contain any faces"
    weld_tol = 1e-7
    welded_keys: dict[tuple[int, int, int], int] = {}
    welded_vertex_ids: list[int] = []
    representative_vertices: list[tuple[float, float, float]] = []
    for vertex in vertices:
        key = tuple(int(round(component / weld_tol)) for component in vertex)
        welded_id = welded_keys.get(key)
        if welded_id is None:
            welded_id = len(representative_vertices)
            welded_keys[key] = welded_id
            representative_vertices.append(vertex)
        welded_vertex_ids.append(welded_id)
    edge_counts: dict[tuple[int, int], int] = {}
    for tri in triangles:
        a, b, c = tri
        assert len({a, b, c}) == 3, f"{obj_path.name} contains a degenerate triangle index set"
        wa = welded_vertex_ids[a]
        wb = welded_vertex_ids[b]
        wc = welded_vertex_ids[c]
        assert len({wa, wb, wc}) == 3, f"{obj_path.name} contains a welded degenerate triangle"
        ax, ay, az = representative_vertices[wa]
        bx, by, bz = representative_vertices[wb]
        cx, cy, cz = representative_vertices[wc]
        ab = (bx - ax, by - ay, bz - az)
        ac = (cx - ax, cy - ay, cz - az)
        cross = (
            ab[1] * ac[2] - ab[2] * ac[1],
            ab[2] * ac[0] - ab[0] * ac[2],
            ab[0] * ac[1] - ab[1] * ac[0],
        )
        area_twice_sq = cross[0] ** 2 + cross[1] ** 2 + cross[2] ** 2
        assert area_twice_sq > 1e-24, f"{obj_path.name} contains a welded degenerate triangle"
        for edge in ((wa, wb), (wb, wc), (wc, wa)):
            undirected = tuple(sorted(edge))
            edge_counts[undirected] = edge_counts.get(undirected, 0) + 1
    bad_edges = [edge for edge, count in edge_counts.items() if count != 2]
    assert not bad_edges, f"{obj_path.name} has open or non-manifold boundary edges"


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_cartesian_stage", assets=ASSETS)

    model.material("machined_aluminum", rgba=(0.78, 0.80, 0.83, 1.0))
    model.material("anodized_blue", rgba=(0.19, 0.32, 0.62, 1.0))
    model.material("rail_steel", rgba=(0.56, 0.58, 0.61, 1.0))
    model.material("bearing_black", rgba=(0.12, 0.13, 0.15, 1.0))

    base = model.part("base")
    base_plate = _plate_with_counterbores(
        BASE_L,
        BASE_W,
        BASE_T,
        corner_radius=0.004,
        hole_spacing_x=BASE_L - 0.06,
        hole_spacing_y=BASE_W - 0.06,
        hole_diameter=0.006,
        cbore_diameter=0.011,
        cbore_depth=0.003,
    )
    x_rail_left = _rail_bar(X_RAIL_LEN, X_RAIL_W, X_RAIL_H).translate(
        (0.0, -X_RAIL_SPACING / 2.0, BASE_T / 2.0 + X_RAIL_H / 2.0)
    )
    x_rail_right = _rail_bar(X_RAIL_LEN, X_RAIL_W, X_RAIL_H).translate(
        (0.0, X_RAIL_SPACING / 2.0, BASE_T / 2.0 + X_RAIL_H / 2.0)
    )
    _add_visual_mesh(base, base_plate, "base_plate.obj", "machined_aluminum")
    _add_visual_mesh(
        base,
        _merge_shapes(x_rail_left, x_rail_right),
        "x_rails.obj",
        "rail_steel",
    )



    base.inertial = Inertial.from_geometry(
        Box((BASE_L, BASE_W, BASE_T + X_RAIL_H)),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.0, X_RAIL_H / 2.0)),
    )

    x_stage = model.part("x_stage")
    x_carriage_plate = _plate_with_counterbores(
        X_CARRIAGE_L,
        X_CARRIAGE_W,
        X_CARRIAGE_T,
        corner_radius=0.0035,
        hole_spacing_x=X_CARRIAGE_L - 0.05,
        hole_spacing_y=X_CARRIAGE_W - 0.045,
        hole_diameter=0.005,
        cbore_diameter=0.009,
        cbore_depth=0.0025,
    ).translate((0.0, 0.0, X_BLOCK_H + X_CARRIAGE_T / 2.0))
    x_blocks = _merge_shapes(
        _bearing_block(X_BLOCK_L, X_BLOCK_W, X_BLOCK_H).translate(
            (-X_BLOCK_SPACING_X / 2.0, -X_RAIL_SPACING / 2.0, X_BLOCK_H / 2.0)
        ),
        _bearing_block(X_BLOCK_L, X_BLOCK_W, X_BLOCK_H).translate(
            (X_BLOCK_SPACING_X / 2.0, -X_RAIL_SPACING / 2.0, X_BLOCK_H / 2.0)
        ),
        _bearing_block(X_BLOCK_L, X_BLOCK_W, X_BLOCK_H).translate(
            (-X_BLOCK_SPACING_X / 2.0, X_RAIL_SPACING / 2.0, X_BLOCK_H / 2.0)
        ),
        _bearing_block(X_BLOCK_L, X_BLOCK_W, X_BLOCK_H).translate(
            (X_BLOCK_SPACING_X / 2.0, X_RAIL_SPACING / 2.0, X_BLOCK_H / 2.0)
        ),
    )
    y_rails = _merge_shapes(
        _rail_bar(Y_RAIL_W, Y_RAIL_LEN, Y_RAIL_H).translate(
            (-Y_RAIL_SPACING / 2.0, 0.0, X_BLOCK_H + X_CARRIAGE_T + Y_RAIL_H / 2.0)
        ),
        _rail_bar(Y_RAIL_W, Y_RAIL_LEN, Y_RAIL_H).translate(
            (Y_RAIL_SPACING / 2.0, 0.0, X_BLOCK_H + X_CARRIAGE_T + Y_RAIL_H / 2.0)
        ),
    )
    _add_visual_mesh(x_stage, x_carriage_plate, "x_carriage_plate.obj", "anodized_blue")
    _add_visual_mesh(x_stage, x_blocks, "x_bearing_blocks.obj", "bearing_black")
    _add_visual_mesh(x_stage, y_rails, "y_rails.obj", "rail_steel")
    for x_offset in (-X_BLOCK_SPACING_X / 2.0, X_BLOCK_SPACING_X / 2.0):
        for y_offset in (-X_RAIL_SPACING / 2.0, X_RAIL_SPACING / 2.0):
            pass



    x_stage.inertial = Inertial.from_geometry(
        Box((X_CARRIAGE_L, X_CARRIAGE_W, Y_JOINT_Z)),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.0, Y_JOINT_Z / 2.0)),
    )

    y_stage = model.part("y_stage")
    y_stage_plate = _plate_with_counterbores(
        Y_STAGE_L,
        Y_STAGE_W,
        Y_STAGE_T,
        corner_radius=0.003,
        hole_spacing_x=Y_STAGE_L - 0.045,
        hole_spacing_y=Y_STAGE_W - 0.04,
        hole_diameter=0.0045,
        cbore_diameter=0.008,
        cbore_depth=0.002,
    ).translate((0.0, 0.0, Y_BLOCK_H + Y_STAGE_T / 2.0))
    y_blocks = _merge_shapes(
        _bearing_block(Y_BLOCK_L, Y_BLOCK_W, Y_BLOCK_H).translate(
            (-Y_RAIL_SPACING / 2.0, -Y_BLOCK_SPACING_Y / 2.0, Y_BLOCK_H / 2.0)
        ),
        _bearing_block(Y_BLOCK_L, Y_BLOCK_W, Y_BLOCK_H).translate(
            (Y_RAIL_SPACING / 2.0, -Y_BLOCK_SPACING_Y / 2.0, Y_BLOCK_H / 2.0)
        ),
        _bearing_block(Y_BLOCK_L, Y_BLOCK_W, Y_BLOCK_H).translate(
            (-Y_RAIL_SPACING / 2.0, Y_BLOCK_SPACING_Y / 2.0, Y_BLOCK_H / 2.0)
        ),
        _bearing_block(Y_BLOCK_L, Y_BLOCK_W, Y_BLOCK_H).translate(
            (Y_RAIL_SPACING / 2.0, Y_BLOCK_SPACING_Y / 2.0, Y_BLOCK_H / 2.0)
        ),
    )
    tool_mount = (
        cq.Workplane("XY")
        .box(TOOL_MOUNT_W, TOOL_MOUNT_W, TOOL_MOUNT_H)
        .edges("|Z")
        .fillet(0.0012)
        .faces(">Z")
        .workplane()
        .hole(0.012)
        .translate((0.0, 0.0, Y_BLOCK_H + Y_STAGE_T + TOOL_MOUNT_H / 2.0))
    )
    _add_visual_mesh(y_stage, y_stage_plate, "y_stage_plate.obj", "machined_aluminum")
    _add_visual_mesh(y_stage, y_blocks, "y_bearing_blocks.obj", "bearing_black")
    _add_visual_mesh(y_stage, tool_mount, "tool_mount.obj", "rail_steel")
    for x_offset in (-Y_RAIL_SPACING / 2.0, Y_RAIL_SPACING / 2.0):
        for y_offset in (-Y_BLOCK_SPACING_Y / 2.0, Y_BLOCK_SPACING_Y / 2.0):
            pass


    y_stage.inertial = Inertial.from_geometry(
        Box((Y_STAGE_L, Y_STAGE_W, Y_BLOCK_H + Y_STAGE_T + TOOL_MOUNT_H)),
        mass=0.85,
        origin=Origin(xyz=(0.0, 0.0, (Y_BLOCK_H + Y_STAGE_T + TOOL_MOUNT_H) / 2.0)),
    )

    model.articulation(
        "x_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_stage,
        origin=Origin(xyz=(0.0, 0.0, X_JOINT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-X_TRAVEL,
            upper=X_TRAVEL,
            effort=80.0,
            velocity=0.25,
        ),
    )
    model.articulation(
        "y_slide",
        ArticulationType.PRISMATIC,
        parent=x_stage,
        child=y_stage,
        origin=Origin(xyz=(0.0, 0.0, Y_JOINT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-Y_TRAVEL,
            upper=Y_TRAVEL,
            effort=60.0,
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

    mesh_dir = Path(MESH_DIR)
    for mesh_name in TOPOLOGY_QC_MESHES:
        _assert_closed_obj_mesh(mesh_dir / mesh_name)

    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=128,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
    )

    ctx.expect_origin_distance("x_stage", "base", axes="xy", max_dist=0.001)
    ctx.expect_origin_distance("y_stage", "x_stage", axes="xy", max_dist=0.001)
    ctx.expect_aabb_gap("x_stage", "base", axis="z", max_gap=0.002, max_penetration=0.0)
    ctx.expect_aabb_gap("y_stage", "x_stage", axis="z", max_gap=0.002, max_penetration=0.0)
    ctx.expect_aabb_overlap("x_stage", "base", axes="xy", min_overlap=0.10)
    ctx.expect_aabb_overlap("y_stage", "x_stage", axes="xy", min_overlap=0.06)
    ctx.expect_origin_gap("y_stage", "base", axis="z", min_gap=0.02)
    ctx.expect_joint_motion_axis(
        "x_slide",
        "x_stage",
        world_axis="x",
        direction="positive",
        min_delta=0.01,
    )
    ctx.expect_joint_motion_axis(
        "y_slide",
        "y_stage",
        world_axis="y",
        direction="positive",
        min_delta=0.01,
    )

    with _pose_context(ctx, x_slide=-X_TRAVEL, y_slide=0.0):
        ctx.expect_aabb_gap("x_stage", "base", axis="z", max_gap=0.002, max_penetration=0.0)
        ctx.expect_aabb_overlap("x_stage", "base", axes="xy", min_overlap=0.10)
        ctx.expect_aabb_gap("y_stage", "x_stage", axis="z", max_gap=0.002, max_penetration=0.0)

    with _pose_context(ctx, x_slide=X_TRAVEL, y_slide=0.0):
        ctx.expect_aabb_gap("x_stage", "base", axis="z", max_gap=0.002, max_penetration=0.0)
        ctx.expect_aabb_overlap("x_stage", "base", axes="xy", min_overlap=0.10)
        ctx.expect_aabb_gap("y_stage", "x_stage", axis="z", max_gap=0.002, max_penetration=0.0)

    with _pose_context(ctx, x_slide=0.0, y_slide=-Y_TRAVEL):
        ctx.expect_aabb_gap("y_stage", "x_stage", axis="z", max_gap=0.002, max_penetration=0.0)
        ctx.expect_aabb_overlap("y_stage", "x_stage", axes="xy", min_overlap=0.05)

    with _pose_context(ctx, x_slide=0.0, y_slide=Y_TRAVEL):
        ctx.expect_aabb_gap("y_stage", "x_stage", axis="z", max_gap=0.002, max_penetration=0.0)
        ctx.expect_aabb_overlap("y_stage", "x_stage", axes="xy", min_overlap=0.05)

    with _pose_context(ctx, x_slide=X_TRAVEL, y_slide=Y_TRAVEL):
        ctx.expect_aabb_gap("x_stage", "base", axis="z", max_gap=0.002, max_penetration=0.0)
        ctx.expect_aabb_gap("y_stage", "x_stage", axis="z", max_gap=0.002, max_penetration=0.0)
        ctx.expect_aabb_overlap("x_stage", "base", axes="xy", min_overlap=0.10)
        ctx.expect_aabb_overlap("y_stage", "x_stage", axes="xy", min_overlap=0.05)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
