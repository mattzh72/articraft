from __future__ import annotations

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
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
    repair_loft,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)

BODY_WIDTH = 0.074
BODY_DEPTH = 0.048
BODY_SHOULDER_Z = 0.104
CAP_ORIGIN_Z = 0.101
CAP_HEIGHT = 0.028


def _rounded_rect_section(
    width: float,
    depth: float,
    radius: float,
    z: float,
    *,
    corner_segments: int = 10,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z)
        for x, y in rounded_rect_profile(width, depth, radius, corner_segments=corner_segments)
    ]

def _save_mesh(geometry, filename: str):
    return mesh_from_geometry(geometry, ASSETS.mesh_dir / filename)


def _ring_geometry(
    outer_radius: float,
    inner_radius: float,
    height: float,
    center_z: float,
    *,
    radial_segments: int = 56,
):
    outer = CylinderGeometry(radius=outer_radius, height=height, radial_segments=radial_segments)
    outer.translate(0.0, 0.0, center_z)
    inner = CylinderGeometry(
        radius=inner_radius,
        height=height + 0.002,
        radial_segments=radial_segments,
    )
    inner.translate(0.0, 0.0, center_z)
    return boolean_difference(outer, inner)


def _thread_path(
    radius: float,
    z_start: float,
    z_end: float,
    *,
    turns: float,
    phase: float,
    samples: int,
) -> list[tuple[float, float, float]]:
    pts: list[tuple[float, float, float]] = []
    for i in range(samples + 1):
        t = i / samples
        ang = phase + (2.0 * math.pi * turns * t)
        pts.append(
            (
                radius * math.cos(ang),
                radius * math.sin(ang),
                z_start + (z_end - z_start) * t,
            )
        )
    return pts


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_screwcap_bottle", assets=ASSETS)

    body_frosted = model.material("body_frosted", rgba=(0.21, 0.24, 0.25, 0.72))
    body_satin = model.material("body_satin", rgba=(0.14, 0.15, 0.16, 1.0))
    cap_matte = model.material("cap_matte", rgba=(0.08, 0.08, 0.09, 1.0))
    cap_satin = model.material("cap_satin", rgba=(0.24, 0.25, 0.27, 1.0))
    accent_metal = model.material("accent_metal", rgba=(0.62, 0.63, 0.65, 1.0))

    body_sections = [
        _rounded_rect_section(0.066, 0.042, 0.008, 0.000),
        _rounded_rect_section(0.070, 0.045, 0.010, 0.010),
        _rounded_rect_section(0.074, 0.048, 0.011, 0.050),
        _rounded_rect_section(0.073, 0.047, 0.011, 0.082),
        _rounded_rect_section(0.055, 0.038, 0.010, 0.095),
        _rounded_rect_section(0.039, 0.034, 0.009, BODY_SHOULDER_Z),
    ]
    body_shell_mesh = _save_mesh(repair_loft(section_loft(body_sections)), "bottle_body_shell.obj")

    neck_shell_mesh = _save_mesh(
        _ring_geometry(outer_radius=0.0162, inner_radius=0.0118, height=0.020, center_z=0.114),
        "bottle_neck_shell.obj",
    )
    cap_shell_mesh = _save_mesh(
        _ring_geometry(outer_radius=0.0230, inner_radius=0.0190, height=CAP_HEIGHT, center_z=CAP_HEIGHT / 2.0),
        "cap_shell.obj",
    )
    cap_thread_a_mesh = _save_mesh(
        _ring_geometry(outer_radius=0.0192, inner_radius=0.0171, height=0.0016, center_z=0.0105),
        "cap_thread_a.obj",
    )
    cap_thread_b_mesh = _save_mesh(
        _ring_geometry(outer_radius=0.0192, inner_radius=0.0171, height=0.0016, center_z=0.0163),
        "cap_thread_b.obj",
    )

    body = model.part("body")
    body.visual(body_shell_mesh, material=body_frosted, name="body_shell")
    body.visual(neck_shell_mesh, material=body_satin, name="neck_finish")
    body.visual(
        Cylinder(radius=0.0194, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.1035)),
        material=body_satin,
        name="finish_bead",
    )
    body.visual(
        Cylinder(radius=0.0168, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.1225)),
        material=body_satin,
        name="neck_lip",
    )
    body.visual(
        Cylinder(radius=0.0285, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=body_satin,
        name="base_ring",
    )

    body_thread_a = _save_mesh(
        tube_from_spline_points(
            _thread_path(0.0167, 0.1070, 0.1180, turns=1.18, phase=0.0, samples=26),
            radius=0.00082,
            samples_per_segment=5,
            radial_segments=10,
            cap_ends=True,
        ),
        "body_thread_a.obj",
    )
    body_thread_b = _save_mesh(
        tube_from_spline_points(
            _thread_path(0.0167, 0.1082, 0.1170, turns=0.92, phase=math.pi, samples=22),
            radius=0.00082,
            samples_per_segment=5,
            radial_segments=10,
            cap_ends=True,
        ),
        "body_thread_b.obj",
    )
    body.visual(body_thread_a, material=body_satin, name="thread_start_a")
    body.visual(body_thread_b, material=body_satin, name="thread_start_b")
    body.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, 0.124)),
        mass=0.19,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
    )

    cap = model.part("cap")
    cap.visual(cap_shell_mesh, material=cap_matte, name="cap_skirt")
    cap.visual(
        Cylinder(radius=0.0193, length=0.0022),
        origin=Origin(xyz=(0.0, 0.0, 0.0269)),
        material=cap_satin,
        name="cap_top",
    )
    cap.visual(
        Cylinder(radius=0.0116, length=0.007),
        origin=Origin(xyz=(0.0, 0.0, 0.0185)),
        material=cap_satin,
        name="seal_plug",
    )
    cap.visual(cap_thread_a_mesh, material=cap_satin, name="cap_inner_thread_a")
    cap.visual(cap_thread_b_mesh, material=cap_satin, name="cap_inner_thread_b")
    cap.visual(
        Cylinder(radius=0.0218, length=0.0018),
        origin=Origin(xyz=(0.0, 0.0, 0.0055)),
        material=accent_metal,
        name="cap_rim",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0230, length=CAP_HEIGHT),
        mass=0.035,
        origin=Origin(xyz=(0.0, 0.0, CAP_HEIGHT / 2.0)),
    )

    model.articulation(
        "cap_spin",
        ArticulationType.CONTINUOUS,
        parent="body",
        child="cap",
        origin=Origin(xyz=(0.0, 0.0, CAP_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.allow_overlap(
        "cap",
        "body",
        reason="Nested thread crests and the sealing plug intentionally seat inside the neck finish.",
    )
    ctx.check_articulation_overlaps(max_pose_samples=128)
    ctx.warn_if_coplanar_surfaces(ignore_adjacent=True, ignore_fixed=True)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_aabb_contact("cap", "body")
    ctx.expect_origin_distance("cap", "body", axes="xy", max_dist=0.0015)
    ctx.expect_aabb_overlap("cap", "body", axes="xy", min_overlap=0.038)
    ctx.expect_aabb_gap(
        "cap",
        "body",
        axis="z",
        min_gap=-0.012,
        max_gap=-0.004,
        positive_elem="seal_plug",
        negative_elem="neck_lip",
        name="seal_plug_engagement_depth",
    )
    ctx.expect_aabb_gap(
        "cap",
        "body",
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="cap_rim",
        negative_elem="finish_bead",
        name="clean_cap_to_finish_break",
    )
    ctx.expect_aabb_gap(
        "cap",
        "body",
        axis="z",
        min_gap=0.018,
        max_gap=0.032,
        positive_elem="cap_top",
        negative_elem="body_shell",
        name="cap_reads_proud_above_body",
    )

    for angle in (0.0, 1.7, 3.4):
        with ctx.pose({"cap_spin": angle}):
            ctx.expect_origin_distance("cap", "body", axes="xy", max_dist=0.0015)
            ctx.expect_aabb_contact("cap", "body")
            ctx.expect_aabb_gap(
                "cap",
                "body",
                axis="z",
                min_gap=-0.012,
                max_gap=-0.004,
                positive_elem="seal_plug",
                negative_elem="neck_lip",
                name=f"seal_depth_pose_{angle:.1f}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
