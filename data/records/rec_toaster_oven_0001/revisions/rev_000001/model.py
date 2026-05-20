from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    BoxGeometry,
    Cylinder,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)
HERE = Path(getattr(ASSETS, "asset_root", Path(__file__).resolve().parent))
MESH_DIR = ASSETS.mesh_dir
MESH_DIR.mkdir(parents=True, exist_ok=True)

BODY_WIDTH = 0.44
BODY_DEPTH = 0.335
BODY_HEIGHT = 0.275
BODY_FRONT_Y = BODY_DEPTH / 2.0
BODY_BASE_CLEARANCE = 0.008

DOOR_CENTER_X = -0.064
DOOR_OPEN_ANGLE = 1.05

CONTROL_X = 0.149
CONTROL_PANEL_FRONT_Y = BODY_FRONT_Y + 0.004


def _material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    try:
        return Material(name=name, color=rgba)
    except TypeError:
        return Material(name=name, rgba=rgba)


def _mesh_path(name: str) -> Path:
    try:
        return Path(ASSETS.mesh_path(name))
    except AttributeError:
        return MESH_DIR / name


def _box_geom(size: tuple[float, float, float], center: tuple[float, float, float]) -> BoxGeometry:
    geom = BoxGeometry(size)
    geom.translate(*center)
    return geom


def _merge_geometries(*geometries):
    merged = geometries[0].copy()
    for geometry in geometries[1:]:
        merged.merge(geometry)
    return merged


def _make_body_mesh():
    side_t = 0.014
    top_t = 0.012
    bottom_t = 0.016
    back_t = 0.012
    front_t = 0.008
    partition_t = 0.012

    left_wall = _box_geom(
        (side_t, BODY_DEPTH, BODY_HEIGHT - BODY_BASE_CLEARANCE),
        (
            -BODY_WIDTH / 2.0 + side_t / 2.0,
            0.0,
            BODY_BASE_CLEARANCE + (BODY_HEIGHT - BODY_BASE_CLEARANCE) / 2.0,
        ),
    )
    right_wall = _box_geom(
        (side_t, BODY_DEPTH, BODY_HEIGHT - BODY_BASE_CLEARANCE),
        (
            BODY_WIDTH / 2.0 - side_t / 2.0,
            0.0,
            BODY_BASE_CLEARANCE + (BODY_HEIGHT - BODY_BASE_CLEARANCE) / 2.0,
        ),
    )
    top_panel = _box_geom(
        (BODY_WIDTH - 2.0 * side_t, BODY_DEPTH, top_t),
        (0.0, 0.0, BODY_HEIGHT - top_t / 2.0),
    )
    bottom_panel = _box_geom(
        (BODY_WIDTH - 2.0 * side_t, BODY_DEPTH, bottom_t),
        (0.0, 0.0, BODY_BASE_CLEARANCE + bottom_t / 2.0),
    )
    back_panel = _box_geom(
        (BODY_WIDTH - 2.0 * side_t, back_t, BODY_HEIGHT - BODY_BASE_CLEARANCE),
        (
            0.0,
            -BODY_DEPTH / 2.0 + back_t / 2.0,
            BODY_BASE_CLEARANCE + (BODY_HEIGHT - BODY_BASE_CLEARANCE) / 2.0,
        ),
    )
    partition = _box_geom(
        (partition_t, BODY_DEPTH, BODY_HEIGHT - BODY_BASE_CLEARANCE),
        (0.086, 0.0, BODY_BASE_CLEARANCE + (BODY_HEIGHT - BODY_BASE_CLEARANCE) / 2.0),
    )
    door_header = _box_geom(
        (0.312, front_t, 0.022),
        (DOOR_CENTER_X, BODY_FRONT_Y - front_t / 2.0, 0.213),
    )
    door_footer = _box_geom(
        (0.312, front_t, 0.018),
        (DOOR_CENTER_X, BODY_FRONT_Y - front_t / 2.0, 0.037),
    )
    control_slab = _box_geom(
        (0.114, front_t, 0.212),
        (CONTROL_X, BODY_FRONT_Y - front_t / 2.0, 0.129),
    )

    return _merge_geometries(
        left_wall,
        right_wall,
        top_panel,
        bottom_panel,
        back_panel,
        partition,
        door_header,
        door_footer,
        control_slab,
    )


def _make_liner_mesh():
    width = 0.262
    depth = 0.272
    height = 0.172
    wall_t = 0.004

    back = _box_geom((width, wall_t, height), (0.0, wall_t / 2.0, 0.0))
    left = _box_geom((wall_t, depth, height), (-width / 2.0 + wall_t / 2.0, depth / 2.0, 0.0))
    right = _box_geom((wall_t, depth, height), (width / 2.0 - wall_t / 2.0, depth / 2.0, 0.0))
    floor = _box_geom((width, depth, wall_t), (0.0, depth / 2.0, -height / 2.0 + wall_t / 2.0))
    roof = _box_geom((width, depth, wall_t), (0.0, depth / 2.0, height / 2.0 - wall_t / 2.0))
    side_guide_left = _box_geom((0.010, 0.205, 0.006), (-width / 2.0 + 0.005, 0.130, -0.035))
    side_guide_right = _box_geom((0.010, 0.205, 0.006), (width / 2.0 - 0.005, 0.130, -0.035))
    rack_plate = _box_geom((width + 0.004, 0.215, wall_t), (0.0, 0.130, -0.030))
    rack_lip = _box_geom((width + 0.004, wall_t, 0.016), (0.0, 0.236, -0.026))

    return _merge_geometries(
        back,
        left,
        right,
        floor,
        roof,
        side_guide_left,
        side_guide_right,
        rack_plate,
        rack_lip,
    )


def _add_knob_geometry(
    part, radius: float, depth: float, body_material: Material, cap_material: Material
) -> None:
    part.visual(
        Cylinder(radius=radius + 0.002, length=0.004),
        origin=Origin(xyz=(0.0, 0.002, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_material,
    )
    part.visual(
        Cylinder(radius=radius, length=depth),
        origin=Origin(xyz=(0.0, 0.002 + depth / 2.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_material,
    )
    part.visual(
        Cylinder(radius=radius * 0.72, length=0.010),
        origin=Origin(xyz=(0.0, depth + 0.007, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cap_material,
    )
    part.visual(
        Box((0.004, 0.006, radius * 0.85)),
        origin=Origin(xyz=(0.0, depth + 0.012, radius * 0.43)),
        material=cap_material,
    )
    part.inertial = Inertial.from_geometry(
        Cylinder(radius=radius + 0.002, length=depth + 0.015),
        mass=0.06,
        origin=Origin(xyz=(0.0, (depth + 0.015) / 2.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toaster_oven", assets=ASSETS)

    stainless_steel = _material("stainless_steel", (0.73, 0.75, 0.78, 1.0))
    brushed_aluminum = _material("brushed_aluminum", (0.84, 0.85, 0.87, 1.0))
    black_plastic = _material("black_plastic", (0.09, 0.09, 0.10, 1.0))
    smoked_glass = _material("smoked_glass", (0.14, 0.16, 0.18, 0.35))
    black_glass = _material("black_glass", (0.05, 0.05, 0.06, 1.0))
    dark_enamel = _material("dark_enamel", (0.23, 0.23, 0.24, 1.0))
    rubber = _material("rubber", (0.06, 0.06, 0.07, 1.0))
    red_lens = _material("red_lens", (0.74, 0.08, 0.05, 0.85))

    if hasattr(model, "materials"):
        model.materials.extend(
            [
                stainless_steel,
                brushed_aluminum,
                black_plastic,
                smoked_glass,
                black_glass,
                dark_enamel,
                rubber,
                red_lens,
            ]
        )

    body = model.part("body")
    body_mesh = mesh_from_geometry(_make_body_mesh(), _mesh_path("toaster_oven_body.obj"))
    body.visual(body_mesh, material=stainless_steel)
    body.visual(
        Box((0.102, 0.004, 0.192)),
        origin=Origin(xyz=(CONTROL_X, BODY_FRONT_Y + 0.002, 0.129)),
        material=black_glass,
    )
    body.visual(
        Box((0.118, 0.003, 0.014)),
        origin=Origin(xyz=(DOOR_CENTER_X, BODY_FRONT_Y + 0.0015, 0.236)),
        material=black_glass,
    )
    body.visual(
        Cylinder(radius=0.005, length=0.004),
        origin=Origin(xyz=(CONTROL_X, BODY_FRONT_Y + 0.004, 0.048), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=red_lens,
    )
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            body.visual(
                Box((0.032, 0.054, BODY_BASE_CLEARANCE)),
                origin=Origin(xyz=(x_sign * 0.150, y_sign * 0.110, BODY_BASE_CLEARANCE / 2.0)),
                material=rubber,
            )
    body.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)),
        mass=7.4,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT / 2.0)),
    )

    liner = model.part("liner")
    liner_mesh = mesh_from_geometry(_make_liner_mesh(), _mesh_path("toaster_oven_liner.obj"))
    liner.visual(liner_mesh, material=dark_enamel)
    liner.inertial = Inertial.from_geometry(
        Box((0.262, 0.272, 0.172)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.136, 0.0)),
    )

    door = model.part("door")
    door.visual(
        Box((0.306, 0.020, 0.026)),
        origin=Origin(xyz=(0.0, 0.010, 0.013)),
        material=stainless_steel,
    )
    door.visual(
        Box((0.306, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, 0.010, 0.165)),
        material=stainless_steel,
    )
    door.visual(
        Box((0.020, 0.020, 0.130)),
        origin=Origin(xyz=(-0.143, 0.010, 0.091)),
        material=stainless_steel,
    )
    door.visual(
        Box((0.020, 0.020, 0.130)),
        origin=Origin(xyz=(0.143, 0.010, 0.091)),
        material=stainless_steel,
    )
    door.visual(
        Box((0.246, 0.006, 0.112)),
        origin=Origin(xyz=(0.0, 0.003, 0.100)),
        material=smoked_glass,
    )
    door.visual(
        Box((0.018, 0.030, 0.016)),
        origin=Origin(xyz=(-0.096, 0.025, 0.145)),
        material=black_plastic,
    )
    door.visual(
        Box((0.018, 0.030, 0.016)),
        origin=Origin(xyz=(0.096, 0.025, 0.145)),
        material=black_plastic,
    )
    door.visual(
        Box((0.196, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, 0.045, 0.145)),
        material=black_plastic,
    )
    door.inertial = Inertial.from_geometry(
        Box((0.306, 0.055, 0.174)),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.0275, 0.087)),
    )

    function_knob = model.part("function_knob")
    _add_knob_geometry(
        function_knob,
        radius=0.018,
        depth=0.020,
        body_material=black_plastic,
        cap_material=brushed_aluminum,
    )

    temperature_knob = model.part("temperature_knob")
    _add_knob_geometry(
        temperature_knob,
        radius=0.018,
        depth=0.020,
        body_material=black_plastic,
        cap_material=brushed_aluminum,
    )

    timer_knob = model.part("timer_knob")
    _add_knob_geometry(
        timer_knob,
        radius=0.021,
        depth=0.022,
        body_material=black_plastic,
        cap_material=brushed_aluminum,
    )

    model.articulation(
        "body_to_liner",
        ArticulationType.FIXED,
        parent="body",
        child="liner",
        origin=Origin(xyz=(DOOR_CENTER_X, -0.149, 0.125)),
    )
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent="body",
        child="door",
        origin=Origin(xyz=(DOOR_CENTER_X, BODY_FRONT_Y + 0.0025, 0.046)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.15),
    )
    model.articulation(
        "function_selector",
        ArticulationType.REVOLUTE,
        parent="body",
        child="function_knob",
        origin=Origin(xyz=(CONTROL_X, CONTROL_PANEL_FRONT_Y, 0.205)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=3.0, lower=-2.2, upper=2.2),
    )
    model.articulation(
        "temperature_selector",
        ArticulationType.REVOLUTE,
        parent="body",
        child="temperature_knob",
        origin=Origin(xyz=(CONTROL_X, CONTROL_PANEL_FRONT_Y, 0.148)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=3.0, lower=-2.4, upper=2.4),
    )
    model.articulation(
        "timer_selector",
        ArticulationType.REVOLUTE,
        parent="body",
        child="timer_knob",
        origin=Origin(xyz=(CONTROL_X, CONTROL_PANEL_FRONT_Y, 0.088)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=3.0, lower=-2.6, upper=2.6),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")

    ctx.allow_overlap(
        "body",
        "door",
        reason="The drop-down door closes almost flush against the front frame, so generated collision hulls can conservatively touch at the hinge seam.",
    )
    ctx.allow_overlap(
        "body",
        "function_knob",
        reason="Flush-mounted selector knob skirts sit against the control panel face.",
    )
    ctx.allow_overlap(
        "body",
        "temperature_knob",
        reason="Flush-mounted selector knob skirts sit against the control panel face.",
    )
    ctx.allow_overlap(
        "body",
        "timer_knob",
        reason="Flush-mounted selector knob skirts sit against the control panel face.",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=192, overlap_tol=0.004, overlap_volume_tol=0.0)

    ctx.expect_aabb_overlap("body", "liner", axes="xy", min_overlap=0.20)
    ctx.expect_aabb_overlap("door", "body", axes="xy", min_overlap=0.003)
    ctx.expect_joint_motion_axis(
        "door_hinge", "door", world_axis="z", direction="negative", min_delta=0.055
    )
    ctx.expect_origin_distance("door", "body", axes="xy", max_dist=0.21)
    ctx.expect_origin_distance("function_knob", "body", axes="xy", max_dist=0.235)
    ctx.expect_origin_distance("temperature_knob", "body", axes="xy", max_dist=0.235)
    ctx.expect_origin_distance("timer_knob", "body", axes="xy", max_dist=0.24)

    with ctx.pose(door_hinge=DOOR_OPEN_ANGLE):
        ctx.expect_origin_distance("door", "body", axes="xy", max_dist=0.21)
        ctx.expect_aabb_overlap("body", "liner", axes="xy", min_overlap=0.20)

    with ctx.pose(function_selector=1.8, temperature_selector=-1.6, timer_selector=2.3):
        ctx.expect_origin_distance("function_knob", "body", axes="xy", max_dist=0.235)
        ctx.expect_origin_distance("temperature_knob", "body", axes="xy", max_dist=0.235)
        ctx.expect_origin_distance("timer_knob", "body", axes="xy", max_dist=0.24)

    with ctx.pose(
        door_hinge=DOOR_OPEN_ANGLE,
        function_selector=1.8,
        temperature_selector=-1.6,
        timer_selector=2.3,
    ):
        ctx.expect_origin_distance("door", "body", axes="xy", max_dist=0.21)
        ctx.expect_aabb_overlap("body", "liner", axes="xy", min_overlap=0.20)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
