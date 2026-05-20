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
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
FRAME_OUTER_W = 1.22
FRAME_OUTER_H = 1.82
FRAME_INNER_W = 0.96
FRAME_INNER_H = 1.56
FRAME_DEPTH = 0.045

FLANGE_OUTER_W = 1.32
FLANGE_OUTER_H = 1.92
FLANGE_INNER_W = 1.04
FLANGE_INNER_H = 1.64
FLANGE_DEPTH = 0.006

GASKET_OUTER_W = 1.02
GASKET_OUTER_H = 1.62
GASKET_INNER_W = 0.92
GASKET_INNER_H = 1.52
GASKET_DEPTH = 0.004

SHUTTER_W = 1.04
SHUTTER_H = 1.64
SHEET_DEPTH = 0.016
SHUTTER_BACK_Z = 0.026
SHUTTER_CENTER_Z = SHUTTER_BACK_Z + SHEET_DEPTH / 2.0
HINGE_WORLD_X = -0.52
HINGE_WORLD_Z = 0.023

JOINT_NAME = "frame_to_shutter"


def _make_material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    try:
        return Material(name=name, rgba=rgba)
    except TypeError:
        return Material(name=name, color=rgba)


def _ring_mesh(
    mesh_name: str,
    outer_w: float,
    outer_h: float,
    inner_w: float,
    inner_h: float,
    depth: float,
    *,
    outer_radius: float,
    inner_radius: float,
):
    geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(outer_w, outer_h, outer_radius, corner_segments=8),
        [rounded_rect_profile(inner_w, inner_h, inner_radius, corner_segments=6)],
        height=depth,
        cap=True,
        center=True,
        closed=True,
    )
    return mesh_from_geometry(geom, ASSETS.mesh_path(mesh_name))


def _aabb_center(aabb) -> tuple[float, float, float]:
    center = getattr(aabb, "center", None)
    if center is not None:
        if isinstance(center, tuple):
            return center
        if hasattr(center, "x"):
            return (center.x, center.y, center.z)
    if isinstance(aabb, tuple):
        if len(aabb) == 2 and all(isinstance(v, tuple) and len(v) == 3 for v in aabb):
            (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
            return (
                0.5 * (min_x + max_x),
                0.5 * (min_y + max_y),
                0.5 * (min_z + max_z),
            )
        if len(aabb) == 6:
            min_x, min_y, min_z, max_x, max_y, max_z = aabb
            return (
                0.5 * (min_x + max_x),
                0.5 * (min_y + max_y),
                0.5 * (min_z + max_z),
            )
    if hasattr(aabb, "min_x") and hasattr(aabb, "max_x"):
        min_x, max_x = aabb.min_x, aabb.max_x
        min_y, max_y = aabb.min_y, aabb.max_y
        min_z, max_z = aabb.min_z, aabb.max_z
    else:
        min_x, max_x = aabb.xmin, aabb.xmax
        min_y, max_y = aabb.ymin, aabb.ymax
        min_z, max_z = aabb.zmin, aabb.zmax
    return (
        0.5 * (min_x + max_x),
        0.5 * (min_y + max_y),
        0.5 * (min_z + max_z),
    )


def _aabb_bounds(aabb) -> tuple[float, float, float, float, float, float]:
    if isinstance(aabb, tuple):
        if len(aabb) == 2 and all(isinstance(v, tuple) and len(v) == 3 for v in aabb):
            (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
            return (min_x, min_y, min_z, max_x, max_y, max_z)
        if len(aabb) == 6:
            return aabb
    if hasattr(aabb, "min_x") and hasattr(aabb, "max_x"):
        return (
            aabb.min_x,
            aabb.min_y,
            aabb.min_z,
            aabb.max_x,
            aabb.max_y,
            aabb.max_z,
        )
    return (aabb.xmin, aabb.ymin, aabb.zmin, aabb.xmax, aabb.ymax, aabb.zmax)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="storm_shutter_assembly", assets=ASSETS)

    powder_coat = _make_material("powder_coat_charcoal", (0.27, 0.29, 0.31, 1.0))
    galvanized = _make_material("galvanized_steel", (0.71, 0.73, 0.75, 1.0))
    stainless = _make_material("stainless_hardware", (0.58, 0.60, 0.62, 1.0))
    gasket = _make_material("epdm_gasket", (0.08, 0.08, 0.09, 1.0))

    if hasattr(model, "materials"):
        model.materials.extend([powder_coat, galvanized, stainless, gasket])

    frame_ring = _ring_mesh(
        "storm_shutter_frame_ring.obj",
        FRAME_OUTER_W,
        FRAME_OUTER_H,
        FRAME_INNER_W,
        FRAME_INNER_H,
        FRAME_DEPTH,
        outer_radius=0.025,
        inner_radius=0.012,
    )
    flange_ring = _ring_mesh(
        "storm_shutter_flange.obj",
        FLANGE_OUTER_W,
        FLANGE_OUTER_H,
        FLANGE_INNER_W,
        FLANGE_INNER_H,
        FLANGE_DEPTH,
        outer_radius=0.020,
        inner_radius=0.010,
    )
    seal_ring = _ring_mesh(
        "storm_shutter_gasket.obj",
        GASKET_OUTER_W,
        GASKET_OUTER_H,
        GASKET_INNER_W,
        GASKET_INNER_H,
        GASKET_DEPTH,
        outer_radius=0.010,
        inner_radius=0.006,
    )
    shutter_trim = _ring_mesh(
        "storm_shutter_trim.obj",
        0.98,
        1.58,
        0.78,
        1.38,
        0.010,
        outer_radius=0.012,
        inner_radius=0.008,
    )

    frame = model.part("frame")
    frame.visual(frame_ring, material=powder_coat)
    frame.visual(
        flange_ring,
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=galvanized,
    )
    frame.visual(
        seal_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=gasket,
    )
    frame.visual(
        Box((0.08, 1.70, 0.016)),
        origin=Origin(xyz=(-0.55, 0.0, 0.010)),
        material=galvanized,
        name="hinge_backer",
    )
    frame.visual(
        Box((0.06, 1.50, 0.012)),
        origin=Origin(xyz=(0.56, 0.0, 0.014)),
        material=galvanized,
        name="strike_channel",
    )
    frame.visual(
        Box((1.28, 0.06, 0.026)),
        origin=Origin(xyz=(0.0, 0.905, 0.010)),
        material=galvanized,
        name="head_flashing",
    )
    frame.visual(
        Box((1.26, 0.07, 0.022)),
        origin=Origin(xyz=(0.0, -0.95, 0.008)),
        material=galvanized,
        name="sill_flashing",
    )

    for y_pos in (-0.58, 0.58):
        frame.visual(
            Cylinder(radius=0.008, length=0.26),
            origin=Origin(
                xyz=(HINGE_WORLD_X, y_pos, 0.014),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=stainless,
            name=f"frame_hinge_barrel_{'top' if y_pos > 0 else 'bottom'}",
        )

    for y_pos in (-0.60, 0.0, 0.60):
        frame.visual(
            Box((0.020, 0.10, 0.020)),
            origin=Origin(xyz=(0.57, y_pos, 0.028)),
            material=stainless,
            name=f"keeper_{str(y_pos).replace('.', '_').replace('-', 'n')}",
        )

    frame.inertial = Inertial.from_geometry(
        Box((1.30, 1.95, 0.08)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    shutter = model.part("shutter")
    shutter.visual(
        Box((SHUTTER_W, SHUTTER_H, SHEET_DEPTH)),
        origin=Origin(xyz=(SHUTTER_W / 2.0, 0.0, SHUTTER_CENTER_Z)),
        material=powder_coat,
        name="panel_sheet",
    )
    shutter.visual(
        shutter_trim,
        origin=Origin(xyz=(SHUTTER_W / 2.0, 0.0, 0.046)),
        material=galvanized,
        name="panel_trim",
    )
    shutter.visual(
        Box((0.09, SHUTTER_H, 0.024)),
        origin=Origin(xyz=(0.045, 0.0, 0.041)),
        material=galvanized,
        name="hinge_stile",
    )
    shutter.visual(
        Box((0.09, SHUTTER_H, 0.024)),
        origin=Origin(xyz=(SHUTTER_W - 0.045, 0.0, 0.041)),
        material=galvanized,
        name="lock_stile",
    )
    shutter.visual(
        Box((0.86, 0.09, 0.024)),
        origin=Origin(xyz=(SHUTTER_W / 2.0, 0.76, 0.041)),
        material=galvanized,
        name="top_rail",
    )
    shutter.visual(
        Box((0.86, 0.09, 0.024)),
        origin=Origin(xyz=(SHUTTER_W / 2.0, -0.76, 0.041)),
        material=galvanized,
        name="bottom_rail",
    )
    shutter.visual(
        Box((0.07, 1.34, 0.018)),
        origin=Origin(xyz=(SHUTTER_W / 2.0, 0.0, 0.045)),
        material=galvanized,
        name="center_stiffener",
    )
    shutter.visual(
        Box((0.82, 0.06, 0.018)),
        origin=Origin(xyz=(SHUTTER_W / 2.0, 0.33, 0.045)),
        material=galvanized,
        name="mid_rail_upper",
    )
    shutter.visual(
        Box((0.82, 0.06, 0.018)),
        origin=Origin(xyz=(SHUTTER_W / 2.0, -0.33, 0.045)),
        material=galvanized,
        name="mid_rail_lower",
    )
    shutter.visual(
        Box((0.82, 0.05, 0.012)),
        origin=Origin(
            xyz=(SHUTTER_W / 2.0, 0.30, 0.046),
            rpy=(0.0, 0.0, -0.72),
        ),
        material=galvanized,
        name="wind_brace_upper",
    )
    shutter.visual(
        Box((0.82, 0.05, 0.012)),
        origin=Origin(
            xyz=(SHUTTER_W / 2.0, -0.30, 0.046),
            rpy=(0.0, 0.0, 0.72),
        ),
        material=galvanized,
        name="wind_brace_lower",
    )
    shutter.visual(
        Box((0.07, 1.58, 0.012)),
        origin=Origin(xyz=(0.035, 0.0, 0.006)),
        material=stainless,
        name="hinge_leaf",
    )
    shutter.visual(
        Box((0.032, 1.56, 0.024)),
        origin=Origin(xyz=(0.016, 0.0, 0.018)),
        material=galvanized,
        name="hinge_reinforcement_bridge",
    )
    shutter.visual(
        Cylinder(radius=0.007, length=0.28),
        origin=Origin(
            xyz=(0.0, 0.0, 0.008),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=stainless,
        name="center_hinge_knuckle",
    )
    shutter.visual(
        Box((0.028, 1.42, 0.012)),
        origin=Origin(xyz=(0.97, 0.0, 0.048)),
        material=stainless,
        name="lock_bar",
    )
    for y_pos in (-0.64, 0.0, 0.64):
        shutter.visual(
            Box((0.05, 0.08, 0.018)),
            origin=Origin(xyz=(0.995, y_pos, 0.049)),
            material=stainless,
            name=f"latch_dog_{str(y_pos).replace('.', '_').replace('-', 'n')}",
        )
    shutter.visual(
        Box((0.020, 0.020, 0.016)),
        origin=Origin(xyz=(0.92, 0.0, 0.050)),
        material=stainless,
        name="handle_spindle",
    )
    shutter.visual(
        Cylinder(radius=0.030, length=0.010),
        origin=Origin(xyz=(0.92, 0.0, 0.058)),
        material=stainless,
        name="handwheel",
    )
    shutter.visual(
        Box((0.10, 0.018, 0.016)),
        origin=Origin(xyz=(0.88, 0.0, 0.071)),
        material=stainless,
        name="wheel_grip",
    )

    shutter.inertial = Inertial.from_geometry(
        Box((SHUTTER_W, SHUTTER_H, 0.08)),
        mass=24.0,
        origin=Origin(xyz=(SHUTTER_W / 2.0, 0.0, SHUTTER_CENTER_Z)),
    )

    model.articulation(
        JOINT_NAME,
        ArticulationType.REVOLUTE,
        parent="frame",
        child="shutter",
        origin=Origin(xyz=(HINGE_WORLD_X, 0.0, HINGE_WORLD_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.6,
            lower=0.0,
            upper=1.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "frame",
        "shutter",
        reason="hinge leaves, knuckles, and compression seal intentionally interleave around the shared pivot",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
    )

    ctx.expect_aabb_overlap("shutter", "frame", axes="xy", min_overlap=0.90)
    ctx.expect_joint_motion_axis(
        JOINT_NAME,
        "shutter",
        world_axis="z",
        direction="positive",
        min_delta=0.10,
    )

    frame_aabb = ctx.part_world_aabb("frame", use="collision")
    shutter_aabb = ctx.part_world_aabb("shutter", use="collision")
    closed_center = _aabb_center(shutter_aabb)
    frame_bounds = _aabb_bounds(frame_aabb)
    shutter_bounds = _aabb_bounds(shutter_aabb)

    if shutter_bounds[0] < frame_bounds[0] - 0.02:
        raise AssertionError(
            "Closed shutter should not extend far beyond the hinge-side frame edge."
        )
    if shutter_bounds[3] > frame_bounds[3] + 0.02:
        raise AssertionError("Closed shutter should remain captured within the frame width.")
    if shutter_bounds[1] < frame_bounds[1] - 0.02 or shutter_bounds[4] > frame_bounds[4] + 0.02:
        raise AssertionError(
            "Closed shutter should stay vertically within the protective frame opening."
        )
    if shutter_bounds[5] <= frame_bounds[5] + 0.015:
        raise AssertionError(
            "Closed shutter panel should sit proud of the outer frame for weather sealing."
        )

    with ctx.pose(**{JOINT_NAME: 0.75}):
        mid_center = _aabb_center(ctx.part_world_aabb("shutter", use="collision"))
        if mid_center[2] <= closed_center[2] + 0.08:
            raise AssertionError("Shutter should swing outward by mid-travel.")
        if mid_center[0] >= closed_center[0] - 0.10:
            raise AssertionError("Shutter should pivot back toward the hinge side.")
        ctx.expect_aabb_overlap("shutter", "frame", axes="xy", min_overlap=0.18)

    with ctx.pose(**{JOINT_NAME: 1.45}):
        open_center = _aabb_center(ctx.part_world_aabb("shutter", use="collision"))
        if open_center[2] <= closed_center[2] + 0.25:
            raise AssertionError("Fully opened shutter should project clearly away from the frame.")
        if open_center[0] >= closed_center[0] - 0.22:
            raise AssertionError("Opened shutter should remain parked around the hinge side.")
        if abs(open_center[1] - closed_center[1]) > 0.02:
            raise AssertionError("Opening motion should stay vertically aligned on its hinge axis.")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
