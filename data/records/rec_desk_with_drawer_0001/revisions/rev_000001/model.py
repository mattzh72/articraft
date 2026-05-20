from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

DESK_W = 1.30
DESK_D = 0.62
DESK_H = 0.76
TOP_T = 0.038
SUPPORT_H = DESK_H - TOP_T

PANEL_T = 0.036
PANEL_D = 0.56
PANEL_X = 0.595

MODESTY_T = 0.018
MODESTY_H = 0.32
MODESTY_Y = PANEL_D / 2.0 - MODESTY_T / 2.0
MODESTY_Z = 0.34

INNER_SPAN_W = 2.0 * PANEL_X - PANEL_T

DRAWER_OPENING_W = 0.57
APRON_T = 0.018
APRON_H = 0.11
APRON_Z = SUPPORT_H - APRON_H / 2.0
APRON_Y = -PANEL_D / 2.0 + APRON_T / 2.0
APRON_SEG_W = (INNER_SPAN_W - DRAWER_OPENING_W) / 2.0
APRON_SEG_X = DRAWER_OPENING_W / 2.0 + APRON_SEG_W / 2.0

TOP_RAIL_H = 0.04
TOP_RAIL_Z = SUPPORT_H - TOP_RAIL_H / 2.0

REAR_RAIL_D = 0.03
REAR_RAIL_H = 0.06
REAR_RAIL_Y = 0.20
REAR_RAIL_Z = SUPPORT_H - REAR_RAIL_H / 2.0

HOUSING_TOP_T = 0.016
HOUSING_TOP_D = 0.44
HOUSING_TOP_W = DRAWER_OPENING_W
HOUSING_TOP_Y = -PANEL_D / 2.0 + HOUSING_TOP_D / 2.0
HOUSING_TOP_Z = SUPPORT_H - HOUSING_TOP_T / 2.0

GUIDE_T = 0.018
GUIDE_H = 0.18
GUIDE_D = 0.42

DRAWER_FACE_W = 0.55
DRAWER_FACE_H = 0.15
DRAWER_FACE_T = 0.02

DRAWER_BOX_W = 0.49
DRAWER_BOX_D = 0.40
DRAWER_BOX_H = 0.10
DRAWER_SIDE_T = 0.012
DRAWER_BOTTOM_T = 0.008
DRAWER_BACK_T = 0.012

DRAWER_FACE_CENTER_Y = -PANEL_D / 2.0 + DRAWER_FACE_T / 2.0
DRAWER_BOX_CENTER_Y = DRAWER_FACE_CENTER_Y + DRAWER_FACE_T / 2.0 + DRAWER_BOX_D / 2.0
DRAWER_BOX_CENTER_Z = 0.595

DRAWER_CENTER_OFFSET_X = DRAWER_BOX_W / 2.0 - DRAWER_SIDE_T / 2.0
GUIDE_CENTER_X = DRAWER_BOX_W / 2.0 + GUIDE_T / 2.0 + 0.003
GUIDE_CENTER_Y = -PANEL_D / 2.0 + GUIDE_D / 2.0
GUIDE_CENTER_Z = SUPPORT_H - GUIDE_H / 2.0

DRAWER_TRAVEL = 0.32

BACK_STOP_W = DRAWER_OPENING_W
BACK_STOP_D = 0.03
BACK_STOP_H = 0.06
BACK_STOP_Y = GUIDE_CENTER_Y + GUIDE_D / 2.0 - BACK_STOP_D / 2.0
BACK_STOP_Z = 0.585

FOOT_R = 0.011
FOOT_H = 0.010
FOOT_Y = 0.22


def _material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    try:
        return Material(name=name, color=rgba)
    except TypeError:
        return Material(name=name, rgba=rgba)


def _rounded_panel_mesh(
    width: float, height: float, thickness: float, radius: float, filename: str
):
    geom = ExtrudeGeometry.centered(
        rounded_rect_profile(width, height, radius, corner_segments=10),
        thickness,
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(geom, ASSETS.mesh_path(filename))


def _handle_mesh(filename: str):
    geom = tube_from_spline_points(
        [
            (-0.078, 0.0, 0.0),
            (-0.056, -0.012, 0.0),
            (0.0, -0.022, 0.0),
            (0.056, -0.012, 0.0),
            (0.078, 0.0, 0.0),
        ],
        radius=0.0045,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    return mesh_from_geometry(geom, ASSETS.mesh_path(filename))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="study_desk_with_drawer", assets=ASSETS)

    oak = _material("oak_veneer", (0.60, 0.45, 0.29, 1.0))
    warm_white = _material("warm_white_lacquer", (0.93, 0.93, 0.91, 1.0))
    birch = _material("birch_plywood", (0.82, 0.74, 0.58, 1.0))
    graphite = _material("graphite_metal", (0.22, 0.23, 0.25, 1.0))
    black_plastic = _material("black_plastic", (0.10, 0.10, 0.11, 1.0))
    model.materials.extend([oak, warm_white, birch, graphite, black_plastic])

    tabletop_mesh = _rounded_panel_mesh(DESK_W, DESK_D, TOP_T, 0.03, "tabletop.obj")
    drawer_face_mesh = _rounded_panel_mesh(
        DRAWER_FACE_W,
        DRAWER_FACE_H,
        DRAWER_FACE_T,
        0.01,
        "drawer_face.obj",
    )
    handle_mesh = _handle_mesh("drawer_handle.obj")

    desk_base = model.part("desk_base")
    desk_base.visual(
        Box((PANEL_T, PANEL_D, SUPPORT_H)),
        origin=Origin(xyz=(-PANEL_X, 0.0, SUPPORT_H / 2.0)),
        material=warm_white,
        name="left_panel",
    )
    desk_base.visual(
        Box((PANEL_T, PANEL_D, SUPPORT_H)),
        origin=Origin(xyz=(PANEL_X, 0.0, SUPPORT_H / 2.0)),
        material=warm_white,
        name="right_panel",
    )
    desk_base.visual(
        Box((INNER_SPAN_W, MODESTY_T, MODESTY_H)),
        origin=Origin(xyz=(0.0, MODESTY_Y, MODESTY_Z)),
        material=warm_white,
        name="modesty_panel",
    )
    desk_base.visual(
        Box((APRON_SEG_W, APRON_T, APRON_H)),
        origin=Origin(xyz=(-APRON_SEG_X, APRON_Y, APRON_Z)),
        material=warm_white,
        name="left_apron",
    )
    desk_base.visual(
        Box((APRON_SEG_W, APRON_T, APRON_H)),
        origin=Origin(xyz=(APRON_SEG_X, APRON_Y, APRON_Z)),
        material=warm_white,
        name="right_apron",
    )
    desk_base.visual(
        Box((DRAWER_OPENING_W, APRON_T, TOP_RAIL_H)),
        origin=Origin(xyz=(0.0, APRON_Y, TOP_RAIL_Z)),
        material=warm_white,
        name="drawer_opening_top_rail",
    )
    desk_base.visual(
        Box((INNER_SPAN_W, REAR_RAIL_D, REAR_RAIL_H)),
        origin=Origin(xyz=(0.0, REAR_RAIL_Y, REAR_RAIL_Z)),
        material=warm_white,
        name="rear_under_top_rail",
    )
    desk_base.visual(
        Box((HOUSING_TOP_W, HOUSING_TOP_D, HOUSING_TOP_T)),
        origin=Origin(xyz=(0.0, HOUSING_TOP_Y, HOUSING_TOP_Z)),
        material=birch,
        name="drawer_housing_top",
    )
    desk_base.visual(
        Box((GUIDE_T, GUIDE_D, GUIDE_H)),
        origin=Origin(xyz=(-GUIDE_CENTER_X, GUIDE_CENTER_Y, GUIDE_CENTER_Z)),
        material=birch,
        name="left_guide_panel",
    )
    desk_base.visual(
        Box((GUIDE_T, GUIDE_D, GUIDE_H)),
        origin=Origin(xyz=(GUIDE_CENTER_X, GUIDE_CENTER_Y, GUIDE_CENTER_Z)),
        material=birch,
        name="right_guide_panel",
    )
    desk_base.visual(
        Box((BACK_STOP_W, BACK_STOP_D, BACK_STOP_H)),
        origin=Origin(xyz=(0.0, BACK_STOP_Y, BACK_STOP_Z)),
        material=birch,
        name="back_stop",
    )
    for sx in (-PANEL_X, PANEL_X):
        for sy in (-FOOT_Y, FOOT_Y):
            desk_base.visual(
                Cylinder(radius=FOOT_R, length=FOOT_H),
                origin=Origin(xyz=(sx, sy, FOOT_H / 2.0)),
                material=black_plastic,
            )
    desk_base.inertial = Inertial.from_geometry(
        Box((DESK_W * 0.96, PANEL_D, SUPPORT_H)),
        mass=33.0,
        origin=Origin(xyz=(0.0, 0.0, SUPPORT_H / 2.0)),
    )

    tabletop = model.part("tabletop")
    tabletop.visual(
        tabletop_mesh,
        origin=Origin(xyz=(0.0, 0.06, TOP_T / 2.0)),
        material=oak,
        name="top",
    )
    tabletop.visual(
        Cylinder(radius=0.028, length=TOP_T * 0.94),
        origin=Origin(xyz=(0.34, 0.24, TOP_T / 2.0 + 0.0008)),
        material=black_plastic,
        name="cable_grommet",
    )
    tabletop.inertial = Inertial.from_geometry(
        Box((DESK_W, DESK_D, TOP_T)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.06, TOP_T / 2.0)),
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((DRAWER_SIDE_T, DRAWER_BOX_D, DRAWER_BOX_H)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=birch,
        name="left_side",
    )
    drawer.visual(
        Box((DRAWER_SIDE_T, DRAWER_BOX_D, DRAWER_BOX_H)),
        origin=Origin(xyz=(2.0 * DRAWER_CENTER_OFFSET_X, 0.0, 0.0)),
        material=birch,
        name="right_side",
    )
    drawer.visual(
        Box((DRAWER_BOX_W - 2.0 * DRAWER_SIDE_T, DRAWER_BOX_D, DRAWER_BOTTOM_T)),
        origin=Origin(
            xyz=(DRAWER_CENTER_OFFSET_X, 0.0, -DRAWER_BOX_H / 2.0 + DRAWER_BOTTOM_T / 2.0)
        ),
        material=birch,
        name="bottom",
    )
    drawer.visual(
        Box((DRAWER_BOX_W - 2.0 * DRAWER_SIDE_T, DRAWER_BACK_T, DRAWER_BOX_H - 0.01)),
        origin=Origin(
            xyz=(
                DRAWER_CENTER_OFFSET_X,
                DRAWER_BOX_D / 2.0 - DRAWER_BACK_T / 2.0,
                -0.005,
            )
        ),
        material=birch,
        name="back",
    )
    drawer.visual(
        drawer_face_mesh,
        origin=Origin(
            xyz=(DRAWER_CENTER_OFFSET_X, DRAWER_FACE_CENTER_Y - DRAWER_BOX_CENTER_Y, 0.005),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=warm_white,
        name="front_face",
    )
    drawer.visual(
        handle_mesh,
        origin=Origin(xyz=(DRAWER_CENTER_OFFSET_X, -0.218, 0.005)),
        material=graphite,
        name="pull_handle",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((DRAWER_FACE_W, DRAWER_BOX_D, DRAWER_FACE_H)),
        mass=4.8,
        origin=Origin(xyz=(DRAWER_CENTER_OFFSET_X, -0.02, 0.0)),
    )

    model.articulation(
        "base_to_tabletop",
        ArticulationType.FIXED,
        parent="desk_base",
        child="tabletop",
        origin=Origin(xyz=(0.0, HOUSING_TOP_Y, SUPPORT_H)),
    )
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent="desk_base",
        child="drawer",
        origin=Origin(
            xyz=(
                -DRAWER_BOX_W / 2.0 + DRAWER_SIDE_T / 2.0,
                DRAWER_BOX_CENTER_Y,
                DRAWER_BOX_CENTER_Z,
            )
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.6,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=128,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_overlap("tabletop", "desk_base", axes="xy", min_overlap=0.20)
    ctx.expect_aabb_gap("tabletop", "desk_base", axis="z", max_gap=0.0015, max_penetration=0.001)
    ctx.expect_joint_motion_axis(
        "drawer_slide",
        "drawer",
        world_axis="y",
        direction="negative",
        min_delta=0.05,
    )

    with ctx.pose(drawer_slide=0.0):
        ctx.expect_aabb_within("drawer", "tabletop", axes="xy")
        ctx.expect_aabb_overlap("drawer", "desk_base", axes="xy", min_overlap=0.25)
        ctx.expect_aabb_overlap("drawer", "tabletop", axes="xy", min_overlap=0.25)

    with ctx.pose(drawer_slide=DRAWER_TRAVEL * 0.5):
        ctx.expect_aabb_overlap("drawer", "desk_base", axes="xy", min_overlap=0.18)
        ctx.expect_aabb_overlap("drawer", "tabletop", axes="xy", min_overlap=0.18)

    with ctx.pose(drawer_slide=DRAWER_TRAVEL):
        ctx.expect_aabb_overlap("drawer", "desk_base", axes="xy", min_overlap=0.08)
        ctx.expect_aabb_overlap("drawer", "tabletop", axes="xy", min_overlap=0.10)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
