from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.

# >>> USER_CODE_START
import math
from pathlib import Path

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    DomeGeometry,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.mesh_dir

WIDTH = 0.598
DEPTH = 0.635
HEIGHT = 0.848
SIDE_T = 0.018
TOP_T = 0.020
FRONT_PANEL_T = 0.018
DOOR_CENTER_Z = 0.425


def _circle_profile(radius: float, *, segments: int = 64, center: tuple[float, float] = (0.0, 0.0)) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos((2.0 * math.pi * i) / segments),
            cy + radius * math.sin((2.0 * math.pi * i) / segments),
        )
        for i in range(segments)
    ]


def _save_mesh(name: str, geometry) -> object:
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    return mesh_from_geometry(geometry, MESH_DIR / name)


def _build_meshes() -> dict[str, object]:
    lower_panel = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.564, 0.676, radius=0.020, corner_segments=10),
        [_circle_profile(0.176, segments=72, center=(0.0, 0.039))],
        height=FRONT_PANEL_T,
        cap=True,
        center=True,
        closed=True,
    )
    lower_panel.rotate_x(math.pi / 2.0)

    fascia = ExtrudeGeometry(
        rounded_rect_profile(0.558, 0.098, radius=0.012, corner_segments=8),
        height=0.006,
        cap=True,
        center=True,
        closed=True,
    )
    fascia.rotate_x(math.pi / 2.0)

    door_ring = TorusGeometry(radius=0.165, tube=0.035, radial_segments=28, tubular_segments=72)
    door_ring.rotate_x(math.pi / 2.0)

    gasket_ring = TorusGeometry(radius=0.150, tube=0.028, radial_segments=24, tubular_segments=64)
    gasket_ring.rotate_x(math.pi / 2.0)

    drum_lip = TorusGeometry(radius=0.145, tube=0.012, radial_segments=20, tubular_segments=64)
    drum_lip.rotate_x(math.pi / 2.0)

    door_glass = DomeGeometry(radius=(0.130, 0.130, 0.043), radial_segments=40, height_segments=18, closed=True)
    door_glass.rotate_x(-math.pi / 2.0)

    door_handle = tube_from_spline_points(
        [
            (0.326, 0.036, -0.056),
            (0.356, 0.055, -0.038),
            (0.364, 0.063, 0.000),
            (0.356, 0.055, 0.038),
            (0.326, 0.036, 0.056),
        ],
        radius=0.008,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )

    knob_grip = TorusGeometry(radius=0.033, tube=0.004, radial_segments=16, tubular_segments=40)
    knob_grip.rotate_x(math.pi / 2.0)

    return {
        "lower_panel": _save_mesh("washer_lower_panel.obj", lower_panel),
        "fascia": _save_mesh("washer_fascia.obj", fascia),
        "door_ring": _save_mesh("washer_door_ring.obj", door_ring),
        "gasket_ring": _save_mesh("washer_gasket_ring.obj", gasket_ring),
        "drum_lip": _save_mesh("washer_drum_lip.obj", drum_lip),
        "door_glass": _save_mesh("washer_door_glass.obj", door_glass),
        "door_handle": _save_mesh("washer_door_handle.obj", door_handle),
        "knob_grip": _save_mesh("washer_knob_grip.obj", knob_grip),
    }


def build_object_model() -> ArticulatedObject:
    meshes = _build_meshes()
    model = ArticulatedObject(name="front_load_washer", assets=ASSETS)

    white_enamel = model.material("white_enamel", rgba=(0.95, 0.96, 0.97, 1.0))
    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    matte_black = model.material("matte_black", rgba=(0.08, 0.08, 0.09, 1.0))
    rubber = model.material("rubber", rgba=(0.12, 0.12, 0.13, 1.0))
    stainless = model.material("stainless", rgba=(0.70, 0.72, 0.74, 1.0))
    silver_plastic = model.material("silver_plastic", rgba=(0.78, 0.80, 0.82, 1.0))
    tinted_glass = model.material("tinted_glass", rgba=(0.58, 0.68, 0.76, 0.28))
    screen_black = model.material("screen_black", rgba=(0.05, 0.06, 0.07, 0.94))
    accent = model.material("accent", rgba=(0.96, 0.53, 0.20, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((WIDTH - 0.036, DEPTH - 0.040, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=graphite,
    )
    cabinet.visual(
        Box((SIDE_T, DEPTH, 0.798)),
        origin=Origin(xyz=(-WIDTH / 2.0 + SIDE_T / 2.0, 0.0, 0.429)),
        material=white_enamel,
    )
    cabinet.visual(
        Box((SIDE_T, DEPTH, 0.798)),
        origin=Origin(xyz=(WIDTH / 2.0 - SIDE_T / 2.0, 0.0, 0.429)),
        material=white_enamel,
    )
    cabinet.visual(
        Box((WIDTH - 0.036, SIDE_T, 0.798)),
        origin=Origin(xyz=(0.0, -DEPTH / 2.0 + SIDE_T / 2.0, 0.429)),
        material=white_enamel,
    )
    cabinet.visual(
        Box((WIDTH, DEPTH, TOP_T)),
        origin=Origin(xyz=(0.0, 0.0, HEIGHT - TOP_T / 2.0)),
        material=white_enamel,
    )
    cabinet.visual(
        meshes["lower_panel"],
        origin=Origin(xyz=(0.0, 0.309, 0.386)),
        material=white_enamel,
    )
    cabinet.visual(
        Box((0.562, 0.052, 0.112)),
        origin=Origin(xyz=(0.0, 0.291, 0.775)),
        material=white_enamel,
    )
    cabinet.visual(
        meshes["fascia"],
        origin=Origin(xyz=(0.0, 0.319, 0.782)),
        material=graphite,
    )
    cabinet.visual(
        Box((0.175, 0.014, 0.078)),
        origin=Origin(xyz=(-0.170, 0.283, 0.750)),
        material=screen_black,
    )
    cabinet.visual(
        Box((0.164, 0.008, 0.050)),
        origin=Origin(xyz=(0.198, 0.321, 0.787)),
        material=screen_black,
    )
    for button_x in (-0.018, 0.020, 0.058, 0.096, 0.134):
        cabinet.visual(
            Box((0.024, 0.006, 0.012)),
            origin=Origin(xyz=(button_x, 0.321, 0.736)),
            material=graphite,
        )
    cabinet.visual(
        Box((0.034, 0.006, 0.014)),
        origin=Origin(xyz=(0.250, 0.321, 0.735)),
        material=graphite,
    )
    cabinet.visual(
        Box((0.012, 0.006, 0.012)),
        origin=Origin(xyz=(0.278, 0.321, 0.735)),
        material=accent,
    )
    cabinet.visual(
        meshes["gasket_ring"],
        origin=Origin(xyz=(0.0, 0.285, DOOR_CENTER_Z)),
        material=rubber,
    )
    cabinet.visual(
        Cylinder(radius=0.120, length=0.140),
        origin=Origin(xyz=(0.0, 0.200, DOOR_CENTER_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
    )
    cabinet.visual(
        meshes["drum_lip"],
        origin=Origin(xyz=(0.0, 0.220, DOOR_CENTER_Z)),
        material=stainless,
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((WIDTH, DEPTH, HEIGHT)),
        mass=58.0,
        origin=Origin(xyz=(0.0, 0.0, HEIGHT / 2.0)),
    )

    door = model.part("door")
    door.visual(
        Box((0.028, 0.020, 0.140)),
        origin=Origin(xyz=(0.014, 0.010, 0.0)),
        material=graphite,
    )
    door.visual(
        Box((0.105, 0.022, 0.060)),
        origin=Origin(xyz=(0.0525, 0.010, 0.0)),
        material=graphite,
    )
    door.visual(
        Cylinder(radius=0.010, length=0.080),
        origin=Origin(xyz=(0.0, 0.014, 0.0)),
        material=graphite,
    )
    door.visual(
        meshes["door_ring"],
        origin=Origin(xyz=(0.198, 0.038, 0.0)),
        material=graphite,
    )
    door.visual(
        meshes["door_glass"],
        origin=Origin(xyz=(0.198, 0.018, 0.0)),
        material=tinted_glass,
    )
    door.visual(
        meshes["door_handle"],
        origin=Origin(),
        material=silver_plastic,
    )
    door.inertial = Inertial.from_geometry(
        Box((0.420, 0.080, 0.420)),
        mass=4.5,
        origin=Origin(xyz=(0.198, 0.038, 0.0)),
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        Cylinder(radius=0.040, length=0.024),
        origin=Origin(xyz=(0.0, 0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=silver_plastic,
    )
    selector_knob.visual(
        Cylinder(radius=0.030, length=0.012),
        origin=Origin(xyz=(0.0, 0.030, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
    )
    selector_knob.visual(
        meshes["knob_grip"],
        origin=Origin(xyz=(0.0, 0.022, 0.0)),
        material=silver_plastic,
    )
    selector_knob.visual(
        Box((0.006, 0.006, 0.014)),
        origin=Origin(xyz=(0.0, 0.035, 0.030)),
        material=accent,
    )
    selector_knob.inertial = Inertial.from_geometry(
        Box((0.090, 0.050, 0.090)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.020, 0.0)),
    )

    detergent_drawer = model.part("detergent_drawer")
    detergent_drawer.visual(
        Box((0.170, 0.110, 0.078)),
        origin=Origin(xyz=(0.0, -0.055, 0.0)),
        material=white_enamel,
    )
    detergent_drawer.visual(
        Box((0.176, 0.028, 0.086)),
        origin=Origin(xyz=(0.0, 0.014, 0.0)),
        material=white_enamel,
    )
    detergent_drawer.visual(
        Box((0.110, 0.010, 0.022)),
        origin=Origin(xyz=(0.0, 0.024, 0.0)),
        material=screen_black,
    )
    detergent_drawer.inertial = Inertial.from_geometry(
        Box((0.176, 0.138, 0.086)),
        mass=0.9,
        origin=Origin(xyz=(0.0, -0.041, 0.0)),
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent="cabinet",
        child="door",
        origin=Origin(xyz=(-0.198, 0.309, DOOR_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.2, lower=0.0, upper=1.65),
    )
    model.articulation(
        "selector_turn",
        ArticulationType.CONTINUOUS,
        parent="cabinet",
        child="selector_knob",
        origin=Origin(xyz=(0.118, 0.317, 0.756)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=8.0),
    )
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent="cabinet",
        child="detergent_drawer",
        origin=Origin(xyz=(-0.170, 0.290, 0.750)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.20, lower=0.0, upper=0.115),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.015)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_overlap("door", "cabinet", axes="xz", min_overlap=0.18)
    ctx.expect_aabb_gap("door", "cabinet", axis="y", max_gap=0.008, max_penetration=0.020)
    ctx.expect_aabb_contact("door", "cabinet")
    ctx.expect_aabb_overlap("selector_knob", "cabinet", axes="xz", min_overlap=0.03)
    ctx.expect_aabb_gap("selector_knob", "cabinet", axis="y", max_gap=0.008, max_penetration=0.020)
    ctx.expect_aabb_overlap("detergent_drawer", "cabinet", axes="xz", min_overlap=0.06)
    ctx.expect_aabb_contact("detergent_drawer", "cabinet")
    ctx.expect_aabb_gap("selector_knob", "door", axis="z", max_gap=0.45, max_penetration=0.0)
    ctx.expect_aabb_gap("detergent_drawer", "door", axis="z", max_gap=0.45, max_penetration=0.0)
    ctx.expect_origin_distance("selector_knob", "detergent_drawer", axes="z", max_dist=0.020)
    ctx.expect_joint_motion_axis("door_hinge", "door", world_axis="x", direction="negative", min_delta=0.10)
    ctx.expect_joint_motion_axis("drawer_slide", "detergent_drawer", world_axis="y", direction="positive", min_delta=0.05)

    with ctx.pose(door_hinge=1.45):
        ctx.expect_aabb_overlap("door", "cabinet", axes="z", min_overlap=0.35)
        ctx.expect_aabb_gap("door", "cabinet", axis="y", max_gap=0.40, max_penetration=0.030)

    with ctx.pose(drawer_slide=0.10):
        ctx.expect_aabb_overlap("detergent_drawer", "cabinet", axes="xz", min_overlap=0.06)
        ctx.expect_aabb_contact("detergent_drawer", "cabinet")

    with ctx.pose(selector_turn=math.pi / 2.0):
        ctx.expect_aabb_overlap("selector_knob", "cabinet", axes="xz", min_overlap=0.03)
        ctx.expect_aabb_gap("selector_knob", "cabinet", axis="y", max_gap=0.008, max_penetration=0.020)

    with ctx.pose(selector_turn=math.pi):
        ctx.expect_aabb_overlap("selector_knob", "cabinet", axes="xz", min_overlap=0.03)
        ctx.expect_aabb_gap("selector_knob", "cabinet", axis="y", max_gap=0.008, max_penetration=0.020)

    return ctx.report()
# >>> USER_CODE_END

object_model = build_object_model()
