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
PI = math.pi


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


def _mesh_path(filename: str) -> Path:
    if hasattr(ASSETS, "mesh_path"):
        return Path(ASSETS.mesh_path(filename))
    mesh_dir = ASSETS.mesh_dir
    mesh_dir.mkdir(parents=True, exist_ok=True)
    return mesh_dir / filename


def _material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    try:
        return Material(name=name, color=rgba)
    except TypeError:
        return Material(name=name, rgba=rgba)


def _profile_xyz(profile: list[tuple[float, float]], z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in profile]


def _aabb_bounds(aabb) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    if all(hasattr(aabb, name) for name in ("min_x", "min_y", "min_z", "max_x", "max_y", "max_z")):
        return (
            (aabb.min_x, aabb.min_y, aabb.min_z),
            (aabb.max_x, aabb.max_y, aabb.max_z),
        )
    if hasattr(aabb, "min") and hasattr(aabb, "max"):
        return tuple(aabb.min), tuple(aabb.max)
    if hasattr(aabb, "mins") and hasattr(aabb, "maxs"):
        return tuple(aabb.mins), tuple(aabb.maxs)
    if isinstance(aabb, (tuple, list)):
        if len(aabb) == 2 and all(isinstance(v, (tuple, list)) and len(v) == 3 for v in aabb):
            return tuple(aabb[0]), tuple(aabb[1])
        if len(aabb) == 6:
            return tuple(aabb[:3]), tuple(aabb[3:])
    raise AssertionError(f"Unsupported AABB representation: {type(aabb)!r}")


def _aabb_center(aabb) -> tuple[float, float, float]:
    mins, maxs = _aabb_bounds(aabb)
    return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roof_vent_tower", assets=ASSETS)

    galvanized = _material("galvanized_steel", (0.74, 0.76, 0.78, 1.0))
    membrane = _material("roof_membrane", (0.22, 0.24, 0.27, 1.0))
    rubber = _material("black_rubber", (0.08, 0.08, 0.08, 1.0))
    shadowed_steel = _material("shadowed_powdercoat", (0.47, 0.50, 0.53, 1.0))
    fastener = _material("stainless_fastener", (0.84, 0.85, 0.86, 1.0))

    tower_body = model.part("tower_body")

    flashing_lower = rounded_rect_profile(0.78, 0.58, radius=0.050, corner_segments=8)
    flashing_upper = rounded_rect_profile(0.52, 0.36, radius=0.030, corner_segments=8)
    flashing_mesh = mesh_from_geometry(
        LoftGeometry(
            [
                _profile_xyz(flashing_lower, 0.000),
                _profile_xyz(flashing_upper, 0.032),
            ],
            cap=True,
            closed=True,
        ),
        _mesh_path("roof_vent_flashing.obj"),
    )
    tower_body.visual(flashing_mesh, material=membrane, name="roof_flashing")

    tower_body.visual(
        Box((0.46, 0.30, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.092)),
        material=galvanized,
        name="weatherproof_curb",
    )
    tower_body.visual(
        Box((0.36, 0.24, 0.56)),
        origin=Origin(xyz=(0.0, 0.0, 0.432)),
        material=galvanized,
        name="tower_shell",
    )

    for x, side in ((-0.171, "left"), (0.171, "right")):
        tower_body.visual(
            Box((0.018, 0.018, 0.56)),
            origin=Origin(xyz=(x, 0.0, 0.432)),
            material=shadowed_steel,
            name=f"corner_batten_{side}",
        )

    tower_body.visual(
        Box((0.40, 0.28, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.726)),
        material=galvanized,
        name="coping_cap",
    )

    for idx, (x, y) in enumerate(((-0.17, -0.11), (-0.17, 0.11), (0.17, -0.11), (0.17, 0.11))):
        tower_body.visual(
            Cylinder(radius=0.006, length=0.008),
            origin=Origin(xyz=(x, y, 0.744)),
            material=fastener,
            name=f"cap_fastener_{idx}",
        )

    tower_body.visual(
        Box((0.25, 0.08, 0.19)),
        origin=Origin(xyz=(0.0, 0.160, 0.520)),
        material=galvanized,
        name="outlet_throat",
    )

    tower_body.visual(
        Box((0.34, 0.20, 0.016)),
        origin=Origin(xyz=(0.0, 0.145, 0.666), rpy=(-0.34, 0.0, 0.0)),
        material=galvanized,
        name="rain_hood",
    )
    for x, side in ((-0.163, "left"), (0.163, "right")):
        tower_body.visual(
            Box((0.014, 0.145, 0.145)),
            origin=Origin(xyz=(x, 0.146, 0.602), rpy=(-0.34, 0.0, 0.0)),
            material=galvanized,
            name=f"hood_cheek_{side}",
        )
    tower_body.visual(
        Box((0.30, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, 0.232, 0.627)),
        material=shadowed_steel,
        name="drip_lip",
    )

    for x, tag in ((-0.110, "left"), (0.110, "right")):
        tower_body.visual(
            Box((0.024, 0.090, 0.080)),
            origin=Origin(xyz=(x, 0.134, 0.577), rpy=(-0.28, 0.0, 0.0)),
            material=shadowed_steel,
            name=f"hood_gusset_{tag}",
        )

    rear_louver_mesh = mesh_from_geometry(
        _grille_panel_geometry(
            panel_size=(0.22, 0.26),
            thickness=0.012,
            frame=0.010,
            slat_pitch=0.024,
            slat_width=0.010,
            slat_angle_deg=34.0,
            corner_radius=0.006,
            center=True,
        ),
        _mesh_path("roof_vent_rear_louver.obj"),
    )
    tower_body.visual(
        rear_louver_mesh,
        origin=Origin(xyz=(0.0, -0.123, 0.440), rpy=(PI / 2.0, 0.0, 0.0)),
        material=shadowed_steel,
        name="rear_louver_panel",
    )

    tower_body.visual(
        Box((0.012, 0.008, 0.190)),
        origin=Origin(xyz=(-0.119, 0.204, 0.520)),
        material=rubber,
        name="gasket_left",
    )
    tower_body.visual(
        Box((0.012, 0.008, 0.190)),
        origin=Origin(xyz=(0.119, 0.204, 0.520)),
        material=rubber,
        name="gasket_right",
    )
    tower_body.visual(
        Box((0.250, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, 0.204, 0.609)),
        material=rubber,
        name="gasket_top",
    )
    tower_body.visual(
        Box((0.250, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, 0.204, 0.431)),
        material=rubber,
        name="gasket_bottom",
    )
    tower_body.visual(
        Box((0.224, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, 0.214, 0.606)),
        material=shadowed_steel,
        name="hinge_mount_strip",
    )
    for x, side in ((-0.094, "left"), (0.094, "right")):
        tower_body.visual(
            Box((0.018, 0.022, 0.044)),
            origin=Origin(xyz=(x, 0.212, 0.585)),
            material=shadowed_steel,
            name=f"hinge_bracket_{side}",
        )

    tower_body.inertial = Inertial.from_geometry(
        Box((0.78, 0.58, 0.75)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, 0.375)),
    )

    outlet_flap = model.part("outlet_flap")
    outlet_flap.visual(
        Box((0.230, 0.014, 0.172)),
        origin=Origin(xyz=(0.0, 0.014, -0.086)),
        material=galvanized,
        name="flap_leaf",
    )
    outlet_flap.visual(
        Cylinder(radius=0.008, length=0.202),
        origin=Origin(xyz=(0.0, 0.014, -0.004), rpy=(0.0, PI / 2.0, 0.0)),
        material=fastener,
        name="hinge_barrel",
    )
    for x, tag in ((-0.066, "left"), (0.066, "right")):
        outlet_flap.visual(
            Box((0.020, 0.006, 0.120)),
            origin=Origin(xyz=(x, 0.020, -0.098)),
            material=shadowed_steel,
            name=f"panel_stiffener_{tag}",
        )
    outlet_flap.visual(
        Box((0.194, 0.010, 0.022)),
        origin=Origin(xyz=(0.0, 0.019, -0.161)),
        material=shadowed_steel,
        name="bottom_hem",
    )
    for x, side in ((-0.107, "left"), (0.107, "right")):
        outlet_flap.visual(
            Box((0.008, 0.010, 0.172)),
            origin=Origin(xyz=(x, 0.018, -0.086)),
            material=shadowed_steel,
            name=f"side_hem_{side}",
        )

    outlet_flap.inertial = Inertial.from_geometry(
        Box((0.230, 0.018, 0.172)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.014, -0.086)),
    )

    model.articulation(
        "outlet_flap_hinge",
        ArticulationType.REVOLUTE,
        parent="tower_body",
        child="outlet_flap",
        origin=Origin(xyz=(0.0, 0.214, 0.603)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.5,
            lower=0.0,
            upper=0.95,
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
        "outlet_flap",
        "tower_body",
        reason=(
            "The weather-sealed top hinge intentionally nests the flap knuckle against "
            "the outlet frame; generated collision hulls conservatively report small "
            "contacts along the shared hinge line through the swing range."
        ),
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=160, overlap_tol=0.004, overlap_volume_tol=0.0)

    ctx.expect_joint_motion_axis(
        "outlet_flap_hinge",
        "outlet_flap",
        world_axis="y",
        direction="positive",
        min_delta=0.05,
    )

    body_aabb = ctx.part_world_aabb("tower_body")
    body_mins, body_maxs = _aabb_bounds(body_aabb)
    outlet_face_y = 0.160 + 0.08 * 0.5
    gasket_outer_y = 0.204 + 0.008 * 0.5
    hood_front_y = 0.232 + 0.014 * 0.5

    with ctx.pose(outlet_flap_hinge=0.0):
        flap_closed = ctx.part_world_aabb("outlet_flap")
        closed_mins, closed_maxs = _aabb_bounds(flap_closed)
        closed_center = _aabb_center(flap_closed)

        assert abs(closed_center[0]) < 0.01, "Closed flap should stay centered on the outlet."
        assert outlet_face_y + 0.005 < closed_mins[1] < gasket_outer_y + 0.02, (
            "Closed flap should hang just ahead of the vent face and weather gasket."
        )
        assert body_mins[2] + 0.40 < closed_mins[2] < body_mins[2] + 0.48, (
            "Outlet flap should live in the upper vent throat, not near the curb."
        )
        assert closed_maxs[2] < body_maxs[2] - 0.06, (
            "Closed flap should remain tucked under the rain hood."
        )

    with ctx.pose(outlet_flap_hinge=0.48):
        flap_mid = ctx.part_world_aabb("outlet_flap")
        mid_center = _aabb_center(flap_mid)

        assert mid_center[1] > closed_center[1] + 0.025, (
            "Partially opened flap should already project outward for airflow."
        )
        assert mid_center[2] > closed_center[2] + 0.015, "Top-hinged flap should lift as it opens."

    with ctx.pose(outlet_flap_hinge=0.90):
        flap_open = ctx.part_world_aabb("outlet_flap")
        open_mins, open_maxs = _aabb_bounds(flap_open)
        open_center = _aabb_center(flap_open)

        assert open_center[1] > closed_center[1] + 0.06, (
            "Open flap should swing clearly forward from the tower body."
        )
        assert open_center[2] > closed_center[2] + 0.03, (
            "Open flap should rise rather than sag downward."
        )
        assert open_mins[2] > body_mins[2] + 0.50, (
            "Fully opened flap should stay high in the hooded outlet zone."
        )
        assert open_maxs[1] > hood_front_y + 0.05, (
            "Fully opened flap should establish a clear exhaust path beyond the hood line."
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
