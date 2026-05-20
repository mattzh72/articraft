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
    CylinderGeometry,
    Inertial,
    LoftGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _make_material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    try:
        return Material(name=name, rgba=rgba)
    except TypeError:
        try:
            return Material(name=name, color=rgba)
        except TypeError:
            material = Material(name=name)
            if hasattr(material, "rgba"):
                material.rgba = rgba
            elif hasattr(material, "color"):
                material.color = rgba
            return material


def _rounded_loop(
    width: float,
    depth: float,
    radius: float,
    z: float,
    center_xy: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float, float]]:
    cx, cy = center_xy
    return [
        (cx + x, cy + y, z)
        for x, y in rounded_rect_profile(width, depth, radius, corner_segments=14)
    ]


def _build_body_shell_mesh():
    body_geom = LoftGeometry(
        [
            _rounded_loop(0.36, 0.26, 0.045, 0.035),
            _rounded_loop(0.40, 0.30, 0.055, 0.12),
            _rounded_loop(0.40, 0.30, 0.055, 0.74),
            _rounded_loop(0.37, 0.27, 0.045, 0.90),
        ],
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(body_geom, ASSETS.mesh_path("turnstile_body_shell.obj"))


def _build_control_head_mesh():
    head_geom = LoftGeometry(
        [
            _rounded_loop(0.14, 0.12, 0.018, 0.001, center_xy=(0.0, 0.0)),
            _rounded_loop(0.13, 0.11, 0.018, 0.040, center_xy=(0.0, 0.006)),
            _rounded_loop(0.10, 0.09, 0.014, 0.075, center_xy=(0.0, 0.014)),
        ],
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(head_geom, ASSETS.mesh_path("turnstile_control_head.obj"))


def _build_rotor_core_mesh():
    rotor_geom = CylinderGeometry(radius=0.05, height=0.09, radial_segments=32)
    rotor_geom.rotate_x(math.pi / 2.0).translate(0.0, 0.049, 0.0)

    front_face = CylinderGeometry(radius=0.058, height=0.012, radial_segments=32)
    front_face.rotate_x(math.pi / 2.0).translate(0.0, 0.096, 0.0)
    rotor_geom.merge(front_face)

    rear_shoulder = CylinderGeometry(radius=0.064, height=0.01, radial_segments=32)
    rear_shoulder.rotate_x(math.pi / 2.0).translate(0.0, 0.012, 0.0)
    rotor_geom.merge(rear_shoulder)

    arm_radius = 0.013
    arm_plane_y = 0.096
    arm_tip_radius = 0.35
    for angle in (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0):
        direction_x = math.cos(angle)
        direction_z = math.sin(angle)
        arm_geom = tube_from_spline_points(
            [
                (0.055 * direction_x, arm_plane_y, 0.055 * direction_z),
                (0.20 * direction_x, arm_plane_y, 0.20 * direction_z),
                (arm_tip_radius * direction_x, arm_plane_y, arm_tip_radius * direction_z),
            ],
            radius=arm_radius,
            samples_per_segment=14,
            radial_segments=20,
            cap_ends=True,
        )
        rotor_geom.merge(arm_geom)

    return mesh_from_geometry(rotor_geom, ASSETS.mesh_path("turnstile_rotor_core.obj"))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="turnstile_gate", assets=ASSETS)

    materials = {
        "powder_coat": _make_material("powder_coat", (0.16, 0.18, 0.20, 1.0)),
        "brushed_steel": _make_material("brushed_steel", (0.77, 0.79, 0.82, 1.0)),
        "matte_black": _make_material("matte_black", (0.08, 0.08, 0.09, 1.0)),
        "glass": _make_material("glass", (0.15, 0.24, 0.28, 0.45)),
        "indicator_green": _make_material("indicator_green", (0.18, 0.82, 0.40, 0.8)),
        "indicator_red": _make_material("indicator_red", (0.84, 0.20, 0.20, 0.8)),
        "rubber": _make_material("rubber", (0.05, 0.05, 0.05, 1.0)),
    }
    try:
        model.materials.extend(materials.values())
    except Exception:
        pass

    body = model.part("body")
    body.visual(
        Box((0.46, 0.36, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=materials["powder_coat"],
        name="floor_plinth",
    )
    body.visual(
        _build_body_shell_mesh(),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=materials["brushed_steel"],
        name="cabinet_shell",
    )
    body.visual(
        Box((0.34, 0.20, 0.014)),
        origin=Origin(xyz=(0.0, 0.03, 0.895)),
        material=materials["matte_black"],
        name="top_deck",
    )
    body.visual(
        Box((0.22, 0.006, 0.30)),
        origin=Origin(xyz=(0.0, -0.147, 0.48)),
        material=materials["matte_black"],
        name="service_panel",
    )
    body.visual(
        Cylinder(radius=0.062, length=0.018),
        origin=Origin(xyz=(0.0, 0.145, 0.93), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=materials["brushed_steel"],
        name="bearing_collar",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.46, 0.36, 1.02)),
        mass=82.0,
        origin=Origin(xyz=(0.0, 0.0, 0.51)),
    )

    control_head = model.part("control_head")
    control_head.visual(
        _build_control_head_mesh(),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=materials["matte_black"],
        name="reader_pod",
    )
    control_head.visual(
        Box((0.088, 0.006, 0.075)),
        origin=Origin(xyz=(0.0, 0.047, 0.040), rpy=(-0.34, 0.0, 0.0)),
        material=materials["glass"],
        name="display_glass",
    )
    control_head.visual(
        Box((0.030, 0.004, 0.018)),
        origin=Origin(xyz=(-0.034, 0.053, 0.060), rpy=(-0.34, 0.0, 0.0)),
        material=materials["indicator_green"],
        name="status_arrow",
    )
    control_head.visual(
        Box((0.026, 0.004, 0.018)),
        origin=Origin(xyz=(0.033, 0.053, 0.060), rpy=(-0.34, 0.0, 0.0)),
        material=materials["indicator_red"],
        name="status_stop",
    )
    control_head.inertial = Inertial.from_geometry(
        Box((0.14, 0.12, 0.08)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.008, 0.04)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        _build_rotor_core_mesh(),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=materials["brushed_steel"],
        name="tripod_rotor",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0), start=1):
        rotor.visual(
            Sphere(radius=0.018),
            origin=Origin(xyz=(0.35 * math.cos(angle), 0.096, 0.35 * math.sin(angle))),
            material=materials["rubber"],
            name=f"arm_tip_{index}",
        )
    rotor.inertial = Inertial.from_geometry(
        Box((0.72, 0.11, 0.72)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.096, 0.0)),
    )

    model.articulation(
        "body_to_control_head",
        ArticulationType.FIXED,
        parent="body",
        child="control_head",
        origin=Origin(xyz=(0.0, 0.06, 0.90)),
    )
    model.articulation(
        "tripod_rotation",
        ArticulationType.CONTINUOUS,
        parent="body",
        child="rotor",
        origin=Origin(xyz=(0.0, 0.154, 0.93)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=3.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=192,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
    )

    ctx.expect_aabb_overlap("control_head", "body", axes="xy", min_overlap=0.10)
    ctx.expect_origin_distance("control_head", "body", axes="xy", max_dist=0.08)
    ctx.expect_origin_distance("control_head", "rotor", axes="xy", max_dist=0.16)

    for angle in (0.0, math.pi / 3.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0):
        with ctx.pose(tripod_rotation=angle):
            ctx.expect_aabb_overlap("rotor", "body", axes="xy", min_overlap=0.02)
            ctx.expect_origin_distance("rotor", "body", axes="xy", max_dist=0.24)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
