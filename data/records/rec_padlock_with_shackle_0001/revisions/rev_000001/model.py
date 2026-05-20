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
    Inertial,
    LoftGeometry,
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


def _loft_section(width: float, depth: float, radius: float, z: float):
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="security_padlock", assets=ASSETS)

    brass = Material(name="solid_brass", rgba=(0.73, 0.60, 0.23, 1.0))
    steel = Material(name="hardened_steel", rgba=(0.80, 0.82, 0.85, 1.0))
    dark = Material(name="graphite_black", rgba=(0.10, 0.10, 0.11, 1.0))
    model.materials.extend([brass, steel, dark])

    body_width = 0.046
    body_depth = 0.022
    body_height = 0.055
    shoulder_height = 0.048
    top_width = 0.038

    bore_half_spacing = 0.013
    bore_radius = 0.0044
    shackle_radius = 0.0034
    shackle_open_angle = 1.70

    body_geom = LoftGeometry(
        [
            _loft_section(body_width, body_depth, 0.0048, 0.0),
            _loft_section(body_width, body_depth, 0.0048, 0.030),
            _loft_section(0.041, body_depth, 0.0042, shoulder_height),
            _loft_section(top_width, body_depth, 0.0034, body_height),
        ],
        cap=True,
        closed=True,
    )
    body_mesh = mesh_from_geometry(body_geom, ASSETS.mesh_path("padlock_body.obj"))

    shackle_geom = tube_from_spline_points(
        [
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.016),
            (0.002, 0.0, 0.026),
            (0.009, 0.0, 0.035),
            (0.017, 0.0, 0.035),
            (0.024, 0.0, 0.026),
            (0.026, 0.0, 0.016),
            (0.026, 0.0, 0.0),
        ],
        radius=shackle_radius,
        samples_per_segment=18,
        radial_segments=24,
        cap_ends=True,
    )
    shackle_mesh = mesh_from_geometry(shackle_geom, ASSETS.mesh_path("padlock_shackle.obj"))

    body = model.part("body")
    body.visual(body_mesh, material=brass, name="body_shell")
    body.visual(
        Box((0.040, body_depth * 0.90, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=steel,
        name="bottom_plate",
    )
    body.visual(
        Box((0.030, 0.0012, 0.034)),
        origin=Origin(xyz=(0.0, body_depth / 2.0 - 0.0004, 0.029)),
        material=steel,
        name="front_face_plate",
    )
    body.visual(
        Cylinder(radius=0.0082, length=0.0024),
        origin=Origin(
            xyz=(0.0, body_depth / 2.0 - 0.0001, 0.0195),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="key_plug",
    )
    body.visual(
        Box((0.0020, 0.0026, 0.0105)),
        origin=Origin(xyz=(0.0, body_depth / 2.0 + 0.0001, 0.0198)),
        material=dark,
        name="key_slot",
    )
    body.visual(
        Box((0.0060, 0.0026, 0.0018)),
        origin=Origin(xyz=(0.0, body_depth / 2.0 + 0.0001, 0.0167)),
        material=dark,
        name="ward_notch",
    )
    for sign, name in ((-1.0, "left_bore"), (1.0, "right_bore")):
        body.visual(
            Cylinder(radius=bore_radius, length=0.012),
            origin=Origin(xyz=(sign * bore_half_spacing, 0.0, body_height - 0.006)),
            material=dark,
            name=name,
        )
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=0.34,
        origin=Origin(xyz=(0.0, 0.0, body_height / 2.0)),
    )

    shackle = model.part("shackle")
    shackle.visual(shackle_mesh, material=steel, name="shackle_rod")
    shackle.inertial = Inertial.from_geometry(
        Box((0.033, 0.008, 0.038)),
        mass=0.08,
        origin=Origin(xyz=(0.013, 0.0, 0.019)),
    )

    model.articulation(
        "shackle_hinge",
        ArticulationType.REVOLUTE,
        parent="body",
        child="shackle",
        origin=Origin(xyz=(-bore_half_spacing, 0.0, body_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=0.0,
            upper=shackle_open_angle,
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
        max_pose_samples=160,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_overlap("shackle", "body", axes="xy", min_overlap=0.005)
    ctx.expect_aabb_gap("shackle", "body", axis="z", max_gap=0.0015, max_penetration=0.004)
    ctx.expect_joint_motion_axis(
        "shackle_hinge",
        "shackle",
        world_axis="y",
        direction="positive",
        min_delta=0.01,
    )

    with ctx.pose(shackle_hinge=0.90):
        ctx.expect_aabb_overlap("shackle", "body", axes="xy", min_overlap=0.004)
        ctx.expect_aabb_gap("shackle", "body", axis="z", max_gap=0.002, max_penetration=0.004)

    with ctx.pose(shackle_hinge=1.70):
        ctx.expect_aabb_overlap("shackle", "body", axes="xy", min_overlap=0.003)
        ctx.expect_aabb_gap("shackle", "body", axis="z", max_gap=0.002, max_penetration=0.004)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
