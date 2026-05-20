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
    superellipse_profile,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _profile_at_z(
    profile: list[tuple[float, float]], z: float, dy: float = 0.0
) -> list[tuple[float, float, float]]:
    return [(x, y + dy, z) for x, y in profile]


def _build_bottle_body_mesh():
    base_profile = rounded_rect_profile(0.078, 0.050, radius=0.011, corner_segments=10)
    body_profile = rounded_rect_profile(0.082, 0.052, radius=0.012, corner_segments=10)
    shoulder_mid = rounded_rect_profile(0.074, 0.048, radius=0.011, corner_segments=10)
    shoulder_top = rounded_rect_profile(0.050, 0.036, radius=0.008, corner_segments=10)
    bottle_geom = LoftGeometry(
        [
            _profile_at_z(base_profile, 0.000),
            _profile_at_z(body_profile, 0.010),
            _profile_at_z(body_profile, 0.145),
            _profile_at_z(shoulder_mid, 0.180),
            _profile_at_z(shoulder_top, 0.205),
        ],
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(bottle_geom, ASSETS.mesh_path("pump_bottle_body.obj"))


def _build_collar_ring_mesh():
    collar_geom = ExtrudeWithHolesGeometry(
        superellipse_profile(0.046, 0.046, exponent=2.0, segments=40),
        [superellipse_profile(0.020, 0.020, exponent=2.0, segments=40)],
        height=0.018,
        center=True,
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(collar_geom, ASSETS.mesh_path("pump_bottle_collar.obj"))


def _build_pump_pad_mesh():
    lower = rounded_rect_profile(0.026, 0.042, radius=0.005, corner_segments=8)
    middle = rounded_rect_profile(0.029, 0.056, radius=0.006, corner_segments=8)
    upper = rounded_rect_profile(0.031, 0.061, radius=0.0065, corner_segments=8)
    pad_geom = LoftGeometry(
        [
            _profile_at_z(lower, 0.018, dy=-0.006),
            _profile_at_z(middle, 0.028, dy=-0.004),
            _profile_at_z(upper, 0.035, dy=-0.002),
        ],
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(pad_geom, ASSETS.mesh_path("pump_bottle_head.obj"))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pump_bottle", assets=ASSETS)

    amber_plastic = Material(name="amber_plastic", rgba=(0.80, 0.67, 0.34, 0.42))
    cream_label = Material(name="cream_label", rgba=(0.95, 0.93, 0.89, 1.0))
    sage_ink = Material(name="sage_ink", rgba=(0.42, 0.50, 0.45, 1.0))
    matte_black = Material(name="matte_black", rgba=(0.13, 0.13, 0.14, 1.0))
    brushed_steel = Material(name="brushed_steel", rgba=(0.77, 0.78, 0.80, 1.0))
    natural_tube = Material(name="natural_tube", rgba=(0.92, 0.92, 0.89, 0.72))
    model.materials.extend(
        [amber_plastic, cream_label, sage_ink, matte_black, brushed_steel, natural_tube]
    )

    body_mesh = _build_bottle_body_mesh()
    collar_ring_mesh = _build_collar_ring_mesh()
    pump_pad_mesh = _build_pump_pad_mesh()

    body = model.part("body")
    body.visual(body_mesh, material=amber_plastic, name="bottle_shell")
    body.visual(
        Cylinder(radius=0.014, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.213)),
        material=amber_plastic,
        name="neck",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.222)),
        material=brushed_steel,
        name="crimp_band",
    )
    body.visual(
        collar_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.236)),
        material=brushed_steel,
        name="collar_ring",
    )
    body.visual(
        Cylinder(radius=0.0016, length=0.190),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=natural_tube,
        name="dip_tube",
    )
    body.visual(
        Box((0.056, 0.0012, 0.090)),
        origin=Origin(xyz=(0.0, 0.0254, 0.093)),
        material=cream_label,
        name="front_label",
    )
    body.visual(
        Box((0.042, 0.0014, 0.016)),
        origin=Origin(xyz=(0.0, 0.0255, 0.142)),
        material=sage_ink,
        name="front_label_band",
    )
    body.visual(
        Box((0.040, 0.0010, 0.060)),
        origin=Origin(xyz=(0.0, -0.0253, 0.092)),
        material=cream_label,
        name="back_label",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.082, 0.052, 0.245)),
        mass=0.58,
        origin=Origin(xyz=(0.0, 0.0, 0.1225)),
    )

    pump_head = model.part("pump_head")
    pump_head.visual(
        Cylinder(radius=0.0065, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=matte_black,
        name="stem",
    )
    pump_head.visual(pump_pad_mesh, material=matte_black, name="actuator_pad")
    pump_head.visual(
        Box((0.012, 0.016, 0.006)),
        origin=Origin(xyz=(0.0, 0.020, 0.021)),
        material=matte_black,
        name="nozzle_support",
    )
    pump_head.visual(
        Cylinder(radius=0.0055, length=0.022),
        origin=Origin(xyz=(0.0, 0.037, 0.028), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="nozzle_body",
    )
    pump_head.visual(
        Cylinder(radius=0.0018, length=0.009),
        origin=Origin(xyz=(0.0, 0.048, 0.0215)),
        material=matte_black,
        name="outlet_tip",
    )
    pump_head.visual(
        Cylinder(radius=0.005, length=0.010),
        origin=Origin(xyz=(0.0, -0.028, 0.026), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="rear_cap",
    )
    pump_head.inertial = Inertial.from_geometry(
        Box((0.032, 0.082, 0.040)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.008, 0.025)),
    )

    model.articulation(
        "pump_stroke",
        ArticulationType.PRISMATIC,
        parent="body",
        child="pump_head",
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.15,
            lower=-0.012,
            upper=0.0,
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
        "body",
        "pump_head",
        reason="the pump stem telescopes inside the hollow collar and mesh-derived collision hulls may conservatively close the bore",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=160, overlap_tol=0.003, overlap_volume_tol=0.0)
    ctx.expect_aabb_overlap("pump_head", "body", axes="xy", min_overlap=0.018)
    ctx.expect_origin_distance("pump_head", "body", axes="xy", max_dist=0.020)
    ctx.expect_aabb_gap("pump_head", "body", axis="z", max_gap=0.015, max_penetration=0.0)
    ctx.expect_joint_motion_axis(
        "pump_stroke",
        "pump_head",
        world_axis="z",
        direction="positive",
        min_delta=0.008,
    )
    with ctx.pose(pump_stroke=-0.006):
        ctx.expect_aabb_overlap("pump_head", "body", axes="xy", min_overlap=0.018)
        ctx.expect_origin_distance("pump_head", "body", axes="xy", max_dist=0.021)
    with ctx.pose(pump_stroke=-0.012):
        ctx.expect_aabb_overlap("pump_head", "body", axes="xy", min_overlap=0.016)
        ctx.expect_origin_distance("pump_head", "body", axes="xy", max_dist=0.022)
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
