from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    LoftGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    VentGrilleGeometry,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

STAINLESS = Material("stainless_steel", (0.80, 0.81, 0.83, 1.0))
BRUSHED_ALUMINUM = Material("brushed_aluminum", (0.69, 0.70, 0.72, 1.0))
DARK_TRIM = Material("dark_trim", (0.19, 0.20, 0.22, 1.0))
BLACK_PLASTIC = Material("black_plastic", (0.08, 0.08, 0.09, 1.0))
LENS_WHITE = Material("lens_white", (0.95, 0.97, 1.00, 0.45))

FILTER_WIDTH = 0.244
FILTER_DEPTH = 0.194
FILTER_THICKNESS = 0.010
FILTER_CENTER_X = 0.156
FILTER_HINGE_Y = -0.084
FILTER_HINGE_Z = 0.0205


def _grille_panel_geometry(
    panel_size,
    thickness,
    *,
    frame,
    slat_pitch,
    slat_width,
    slat_angle_deg,
    corner_radius,
    center=True,
):
    geom = VentGrilleGeometry(
        panel_size,
        frame=frame,
        face_thickness=thickness,
        duct_depth=max(0.0015, thickness * 0.75),
        duct_wall=max(0.001, min(frame * 0.45, thickness * 0.75)),
        slat_pitch=slat_pitch,
        slat_width=slat_width,
        slat_angle_deg=slat_angle_deg,
        corner_radius=corner_radius,
    )
    if not center:
        geom.translate(0.0, 0.0, thickness * 0.5)
    return geom


def _canopy_profile(
    width: float, y_back: float, y_front: float, z: float, chamfer: float
) -> list[tuple[float, float, float]]:
    half_w = width * 0.5
    front_chamfer = min(chamfer, half_w * 0.45, (y_front - y_back) * 0.45)
    return [
        (-half_w, y_back, z),
        (half_w, y_back, z),
        (half_w, y_front - front_chamfer, z),
        (half_w - front_chamfer, y_front, z),
        (-half_w + front_chamfer, y_front, z),
        (-half_w, y_front - front_chamfer, z),
    ]


def _offset_profile(
    profile: list[tuple[float, float]], dx: float, dy: float
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="kitchen_range_hood", assets=ASSETS)

    canopy_geom = LoftGeometry(
        [
            _canopy_profile(0.40, -0.145, 0.095, 0.215, 0.020),
            _canopy_profile(0.56, -0.160, 0.145, 0.130, 0.050),
            _canopy_profile(0.66, -0.175, 0.205, 0.040, 0.085),
        ],
        cap=True,
        closed=True,
    )
    canopy_mesh = mesh_from_geometry(canopy_geom, ASSETS.mesh_path("range_hood_canopy.obj"))

    underside_outer = rounded_rect_profile(0.63, 0.31, radius=0.028, corner_segments=8)
    filter_hole = rounded_rect_profile(
        FILTER_WIDTH + 0.028,
        FILTER_DEPTH + 0.028,
        radius=0.014,
        corner_segments=6,
    )
    underside_frame_geom = ExtrudeWithHolesGeometry(
        underside_outer,
        [
            _offset_profile(filter_hole, -FILTER_CENTER_X, 0.020),
            _offset_profile(filter_hole, FILTER_CENTER_X, 0.020),
        ],
        height=0.008,
        cap=True,
        center=True,
        closed=True,
    )
    underside_frame_mesh = mesh_from_geometry(
        underside_frame_geom,
        ASSETS.mesh_path("range_hood_underside_frame.obj"),
    )

    chimney_vent_geom = _grille_panel_geometry(
        panel_size=(0.235, 0.105),
        thickness=0.004,
        frame=0.007,
        slat_pitch=0.018,
        slat_width=0.008,
        slat_angle_deg=28.0,
        corner_radius=0.004,
        center=True,
    )
    chimney_vent_mesh = mesh_from_geometry(
        chimney_vent_geom,
        ASSETS.mesh_path("range_hood_chimney_vent.obj"),
    )

    filter_geom = _grille_panel_geometry(
        panel_size=(FILTER_WIDTH, FILTER_DEPTH),
        thickness=FILTER_THICKNESS,
        frame=0.012,
        slat_pitch=0.026,
        slat_width=0.010,
        slat_angle_deg=31.0,
        corner_radius=0.006,
        center=True,
    )
    filter_mesh = mesh_from_geometry(filter_geom, ASSETS.mesh_path("range_hood_filter.obj"))

    body = model.part("hood_body")
    body.visual(canopy_mesh, material=STAINLESS)
    body.visual(
        Box((0.40, 0.24, 0.016)),
        origin=Origin(xyz=(0.0, -0.028, 0.208)),
        material=STAINLESS,
    )
    body.visual(
        Box((0.34, 0.22, 0.36)),
        origin=Origin(xyz=(0.0, -0.028, 0.410)),
        material=STAINLESS,
    )
    body.visual(
        Box((0.30, 0.18, 0.032)),
        origin=Origin(xyz=(0.0, -0.028, 0.223)),
        material=STAINLESS,
    )
    body.visual(
        underside_frame_mesh,
        origin=Origin(xyz=(0.0, 0.020, 0.029)),
        material=BRUSHED_ALUMINUM,
    )
    body.visual(
        Box((0.36, 0.22, 0.024)),
        origin=Origin(xyz=(0.0, 0.010, 0.040)),
        material=DARK_TRIM,
    )
    body.visual(
        Box((0.60, 0.30, 0.024)),
        origin=Origin(xyz=(0.0, 0.020, 0.044)),
        material=DARK_TRIM,
    )
    body.visual(
        Box((0.46, 0.060, 0.012)),
        origin=Origin(xyz=(0.0, 0.140, 0.052)),
        material=STAINLESS,
    )
    body.visual(
        Box((0.46, 0.036, 0.014)),
        origin=Origin(xyz=(0.0, 0.182, 0.068)),
        material=STAINLESS,
    )
    body.visual(
        Box((0.28, 0.024, 0.016)),
        origin=Origin(xyz=(0.090, 0.204, 0.066)),
        material=DARK_TRIM,
    )
    body.visual(
        Box((0.25, 0.014, 0.004)),
        origin=Origin(xyz=(0.0, 0.198, 0.068)),
        material=DARK_TRIM,
    )
    body.visual(
        chimney_vent_mesh,
        origin=Origin(xyz=(0.0, 0.082, 0.447), rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=DARK_TRIM,
    )
    body.visual(
        Box((0.246, 0.012, 0.112)),
        origin=Origin(xyz=(0.0, 0.076, 0.447)),
        material=DARK_TRIM,
    )
    body.visual(
        Cylinder(radius=0.019, length=0.008),
        origin=Origin(xyz=(-0.275, 0.118, 0.024)),
        material=LENS_WHITE,
    )
    body.visual(
        Box((0.052, 0.052, 0.016)),
        origin=Origin(xyz=(-0.275, 0.118, 0.032)),
        material=BRUSHED_ALUMINUM,
    )
    body.visual(
        Cylinder(radius=0.019, length=0.008),
        origin=Origin(xyz=(0.275, 0.118, 0.024)),
        material=LENS_WHITE,
    )
    body.visual(
        Box((0.052, 0.052, 0.016)),
        origin=Origin(xyz=(0.275, 0.118, 0.032)),
        material=BRUSHED_ALUMINUM,
    )
    body.inertial = Inertial.from_geometry(
        Box((0.66, 0.38, 0.56)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.015, 0.290)),
    )

    for part_name in ("left_filter", "right_filter"):
        filter_part = model.part(part_name)
        filter_part.visual(
            filter_mesh,
            origin=Origin(xyz=(0.0, FILTER_DEPTH * 0.5, -FILTER_THICKNESS * 0.5)),
            material=BRUSHED_ALUMINUM,
        )
        filter_part.visual(
            Box((0.084, 0.010, 0.006)),
            origin=Origin(xyz=(0.0, FILTER_DEPTH - 0.007, -0.011)),
            material=DARK_TRIM,
        )
        filter_part.inertial = Inertial.from_geometry(
            Box((FILTER_WIDTH, FILTER_DEPTH, 0.014)),
            mass=0.9,
            origin=Origin(xyz=(0.0, FILTER_DEPTH * 0.5, -0.007)),
        )

    fan_switch = model.part("fan_switch")
    fan_switch.visual(
        Cylinder(radius=0.0025, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=BLACK_PLASTIC,
    )
    fan_switch.visual(
        Box((0.014, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -0.002, -0.005)),
        material=BLACK_PLASTIC,
    )
    fan_switch.inertial = Inertial.from_geometry(
        Box((0.014, 0.010, 0.010)),
        mass=0.05,
        origin=Origin(xyz=(0.0, -0.002, -0.005)),
    )

    model.articulation(
        "left_filter_hinge",
        ArticulationType.REVOLUTE,
        parent="hood_body",
        child="left_filter",
        origin=Origin(xyz=(-FILTER_CENTER_X, FILTER_HINGE_Y, FILTER_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=-1.18,
            upper=0.0,
        ),
    )
    model.articulation(
        "right_filter_hinge",
        ArticulationType.REVOLUTE,
        parent="hood_body",
        child="right_filter",
        origin=Origin(xyz=(FILTER_CENTER_X, FILTER_HINGE_Y, FILTER_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=-1.18,
            upper=0.0,
        ),
    )
    model.articulation(
        "fan_switch_hinge",
        ArticulationType.REVOLUTE,
        parent="hood_body",
        child="fan_switch",
        origin=Origin(xyz=(0.208, 0.197, 0.071)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=6.0,
            lower=-0.35,
            upper=0.35,
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
        "hood_body",
        "left_filter",
        reason="Seated baffle filter sits in a cutout; generated collision hulls conservatively bridge louvers and aperture edges.",
    )
    ctx.allow_overlap(
        "hood_body",
        "right_filter",
        reason="Seated baffle filter sits in a cutout; generated collision hulls conservatively bridge louvers and aperture edges.",
    )
    ctx.allow_overlap(
        "hood_body",
        "fan_switch",
        reason="The rocker switch pivots within a shallow front control recess, so generated collision hulls conservatively intersect the fascia slot.",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
    )

    ctx.expect_aabb_overlap("left_filter", "hood_body", axes="xy", min_overlap=0.09)
    ctx.expect_aabb_overlap("right_filter", "hood_body", axes="xy", min_overlap=0.09)
    ctx.expect_aabb_gap("hood_body", "left_filter", axis="z", max_gap=0.008, max_penetration=0.014)
    ctx.expect_aabb_gap("hood_body", "right_filter", axis="z", max_gap=0.008, max_penetration=0.014)
    ctx.expect_origin_distance("left_filter", "right_filter", axes="xy", max_dist=0.34)
    ctx.expect_aabb_overlap("fan_switch", "hood_body", axes="xy", min_overlap=0.005)

    ctx.expect_joint_motion_axis(
        "left_filter_hinge",
        "left_filter",
        world_axis="z",
        direction="positive",
        min_delta=0.060,
    )
    ctx.expect_joint_motion_axis(
        "right_filter_hinge",
        "right_filter",
        world_axis="z",
        direction="positive",
        min_delta=0.060,
    )
    ctx.expect_joint_motion_axis(
        "fan_switch_hinge",
        "fan_switch",
        world_axis="y",
        direction="positive",
        min_delta=0.002,
    )

    with ctx.pose(left_filter_hinge=-1.10):
        ctx.expect_aabb_overlap("left_filter", "hood_body", axes="xy", min_overlap=0.03)
        ctx.expect_aabb_gap("hood_body", "left_filter", axis="z", max_gap=0.24, max_penetration=0.014)

    with ctx.pose(right_filter_hinge=-1.10):
        ctx.expect_aabb_overlap("right_filter", "hood_body", axes="xy", min_overlap=0.03)
        ctx.expect_aabb_gap("hood_body", "right_filter", axis="z", max_gap=0.24, max_penetration=0.014)

    with ctx.pose(left_filter_hinge=-1.10, right_filter_hinge=-1.10):
        ctx.expect_origin_distance("left_filter", "right_filter", axes="xy", max_dist=0.36)

    with ctx.pose(fan_switch_hinge=0.30):
        ctx.expect_aabb_overlap("fan_switch", "hood_body", axes="xy", min_overlap=0.004)

    with ctx.pose(fan_switch_hinge=-0.30):
        ctx.expect_aabb_overlap("fan_switch", "hood_body", axes="xy", min_overlap=0.004)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
