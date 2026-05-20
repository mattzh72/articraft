from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _fuselage_shell() -> cq.Workplane:
    """Tapered mapping-drone fuselage, authored along the X flight axis."""
    sections = [
        (-0.72, 0.040, 0.050),
        (-0.52, 0.130, 0.115),
        (-0.12, 0.185, 0.155),
        (0.38, 0.145, 0.130),
        (0.72, 0.030, 0.038),
    ]
    x0, ry0, rz0 = sections[0]
    wp = cq.Workplane("YZ").workplane(offset=x0).ellipse(ry0, rz0)
    prev_x = x0
    for x, ry, rz in sections[1:]:
        wp = wp.workplane(offset=x - prev_x).ellipse(ry, rz)
        prev_x = x
    return wp.loft(combine=True)


def _wing_panel(span_sign: float) -> cq.Workplane:
    """A tapered, mildly swept airfoil panel whose root face starts at y=0."""

    def profile(chord: float, sweep: float, thickness: float) -> list[tuple[float, float]]:
        return [
            (0.48 * chord + sweep, 0.000),
            (0.26 * chord + sweep, 0.55 * thickness),
            (-0.18 * chord + sweep, 0.42 * thickness),
            (-0.52 * chord + sweep, -0.010),
            (-0.16 * chord + sweep, -0.45 * thickness),
            (0.36 * chord + sweep, -0.18 * thickness),
        ]

    span = 1.22 * span_sign
    mid = 0.58 * span_sign
    return (
        cq.Workplane("XZ")
        .polyline(profile(0.72, 0.00, 0.060))
        .close()
        .workplane(offset=mid)
        .polyline(profile(0.56, -0.055, 0.045))
        .close()
        .workplane(offset=span - mid)
        .polyline(profile(0.34, -0.130, 0.028))
        .close()
        .loft(combine=True)
    )


def _vertical_fin() -> cq.Workplane:
    profile = [
        (-0.22, 0.000),
        (0.22, 0.000),
        (0.13, 0.075),
        (-0.02, 0.470),
        (-0.23, 0.080),
    ]
    return cq.Workplane("XZ").polyline(profile).close().extrude(0.055, both=True)


def _camera_pod_body() -> cq.Workplane:
    # A rounded rectangular mapping camera pod, with the hinge barrel added as
    # a primitive visual so its axis remains exact for the revolute joint.
    return cq.Workplane("XY").box(0.205, 0.150, 0.118).edges("|Z").fillet(0.025).edges(">Z").fillet(0.012)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fixed_wing_mapping_survey_drone")

    model.material("matte_white", rgba=(0.88, 0.90, 0.86, 1.0))
    model.material("warm_gray", rgba=(0.52, 0.55, 0.55, 1.0))
    model.material("carbon_black", rgba=(0.02, 0.025, 0.025, 1.0))
    model.material("dark_glass", rgba=(0.015, 0.020, 0.030, 1.0))
    model.material("blue_lens", rgba=(0.08, 0.16, 0.26, 0.85))

    fuselage = model.part("fuselage")
    fuselage.visual(
        mesh_from_cadquery(_fuselage_shell(), "tapered_fuselage", tolerance=0.002),
        material="matte_white",
        name="fuselage_shell",
    )
    # Wing carry-through fairing gives the fixed wings a credible structural
    # root and a flat mating face instead of making them pierce a curved body.
    fuselage.visual(
        Box((0.58, 0.372, 0.056)),
        origin=Origin(xyz=(-0.02, 0.0, 0.060)),
        material="matte_white",
        name="wing_saddle",
    )
    fuselage.visual(
        Box((0.380, 0.076, 0.028)),
        origin=Origin(xyz=(-0.480, 0.0, 0.144)),
        material="matte_white",
        name="tail_fin_saddle",
    )
    fuselage.visual(
        Cylinder(radius=0.075, length=0.180),
        origin=Origin(xyz=(-0.690, 0.0, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="warm_gray",
        name="motor_cowling",
    )
    fuselage.visual(
        Cylinder(radius=0.018, length=0.280),
        origin=Origin(xyz=(0.430, 0.0, -0.018), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="carbon_black",
        name="nose_sensor_strip",
    )
    fuselage.visual(
        Box((0.180, 0.210, 0.040)),
        origin=Origin(xyz=(0.160, 0.0, -0.155)),
        material="warm_gray",
        name="camera_mount_plate",
    )
    for y in (-0.105, 0.105):
        fuselage.visual(
            Box((0.050, 0.040, 0.100)),
            origin=Origin(xyz=(0.160, y, -0.205)),
            material="warm_gray",
            name=f"camera_hinge_ear_{0 if y < 0 else 1}",
        )

    left_wing = model.part("left_wing")
    left_wing.visual(
        mesh_from_cadquery(_wing_panel(-1.0), "left_wing_panel", tolerance=0.002),
        material="matte_white",
        name="wing_panel",
    )
    left_wing.visual(
        Cylinder(radius=0.018, length=1.12),
        origin=Origin(xyz=(-0.045, 0.56, 0.016), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="carbon_black",
        name="carbon_spar",
    )

    right_wing = model.part("right_wing")
    right_wing.visual(
        mesh_from_cadquery(_wing_panel(1.0), "right_wing_panel", tolerance=0.002),
        material="matte_white",
        name="wing_panel",
    )
    right_wing.visual(
        Cylinder(radius=0.018, length=1.12),
        origin=Origin(xyz=(-0.045, -0.56, 0.016), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="carbon_black",
        name="carbon_spar",
    )

    vertical_fin = model.part("vertical_fin")
    vertical_fin.visual(
        mesh_from_cadquery(_vertical_fin(), "vertical_tail_fin", tolerance=0.0015),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="matte_white",
        name="fin_panel",
    )
    vertical_fin.visual(
        Box((0.360, 0.070, 0.025)),
        origin=Origin(xyz=(-0.010, 0.0, 0.012)),
        material="warm_gray",
        name="fin_root_fairing",
    )

    propeller = model.part("propeller")
    propeller.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                0.190,
                0.036,
                3,
                thickness=0.030,
                blade_pitch_deg=34.0,
                blade_sweep_deg=24.0,
                blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=14.0, camber=0.12),
                hub=FanRotorHub(style="spinner", rear_collar_radius=0.030, rear_collar_height=0.020),
            ),
            "pusher_propeller_rotor",
        ),
        material="carbon_black",
        name="prop_blades",
    )
    propeller.visual(
        Cylinder(radius=0.012, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material="warm_gray",
        name="prop_shaft",
    )

    camera_pod = model.part("camera_pod")
    camera_pod.visual(
        Cylinder(radius=0.021, length=0.170),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="warm_gray",
        name="hinge_barrel",
    )
    camera_pod.visual(
        mesh_from_cadquery(_camera_pod_body(), "camera_pod_body", tolerance=0.0015),
        origin=Origin(xyz=(-0.100, 0.0, -0.071)),
        material="warm_gray",
        name="pod_shell",
    )
    camera_pod.visual(
        Cylinder(radius=0.045, length=0.012),
        origin=Origin(xyz=(-0.100, 0.0, -0.136)),
        material="dark_glass",
        name="camera_window",
    )
    camera_pod.visual(
        Sphere(radius=0.026),
        origin=Origin(xyz=(-0.100, 0.0, -0.143)),
        material="blue_lens",
        name="recessed_lens",
    )

    model.articulation(
        "fuselage_to_left_wing",
        ArticulationType.FIXED,
        parent=fuselage,
        child=left_wing,
        origin=Origin(xyz=(-0.020, 0.186, 0.060)),
    )
    model.articulation(
        "fuselage_to_right_wing",
        ArticulationType.FIXED,
        parent=fuselage,
        child=right_wing,
        origin=Origin(xyz=(-0.020, -0.186, 0.060)),
    )
    model.articulation(
        "fuselage_to_vertical_fin",
        ArticulationType.FIXED,
        parent=fuselage,
        child=vertical_fin,
        origin=Origin(xyz=(-0.470, 0.0, 0.158)),
    )
    model.articulation(
        "fuselage_to_propeller",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=propeller,
        origin=Origin(xyz=(-0.860, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=120.0),
    )
    model.articulation(
        "fuselage_to_camera_pod",
        ArticulationType.REVOLUTE,
        parent=fuselage,
        child=camera_pod,
        origin=Origin(xyz=(0.160, 0.0, -0.205)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.5, lower=-0.70, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fuselage = object_model.get_part("fuselage")
    left_wing = object_model.get_part("left_wing")
    right_wing = object_model.get_part("right_wing")
    camera_pod = object_model.get_part("camera_pod")
    left_joint = object_model.get_articulation("fuselage_to_left_wing")
    right_joint = object_model.get_articulation("fuselage_to_right_wing")
    prop_joint = object_model.get_articulation("fuselage_to_propeller")
    pod_joint = object_model.get_articulation("fuselage_to_camera_pod")

    ctx.check(
        "wing sections are fixed to fuselage",
        left_joint.articulation_type == ArticulationType.FIXED
        and right_joint.articulation_type == ArticulationType.FIXED,
        details=f"left={left_joint.articulation_type}, right={right_joint.articulation_type}",
    )
    ctx.check(
        "rear propeller is continuous",
        prop_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={prop_joint.articulation_type}",
    )
    ctx.check(
        "camera pod has a limited tilt hinge",
        pod_joint.articulation_type == ArticulationType.REVOLUTE
        and pod_joint.motion_limits is not None
        and pod_joint.motion_limits.lower < 0.0
        and pod_joint.motion_limits.upper > 0.0,
        details=f"type={pod_joint.articulation_type}, limits={pod_joint.motion_limits}",
    )

    ctx.expect_gap(
        left_wing,
        fuselage,
        axis="y",
        max_gap=0.003,
        max_penetration=0.0005,
        name="left wing root seats on saddle",
    )
    ctx.expect_gap(
        fuselage,
        right_wing,
        axis="y",
        max_gap=0.003,
        max_penetration=0.0005,
        name="right wing root seats on saddle",
    )
    ctx.expect_overlap(
        left_wing,
        fuselage,
        axes="xz",
        min_overlap=0.040,
        name="left wing root overlaps carry-through projection",
    )
    ctx.expect_overlap(
        right_wing,
        fuselage,
        axes="xz",
        min_overlap=0.040,
        name="right wing root overlaps carry-through projection",
    )

    rest_aabb = ctx.part_world_aabb(camera_pod)
    with ctx.pose({pod_joint: -0.60}):
        down_aabb = ctx.part_world_aabb(camera_pod)
    ctx.check(
        "camera pod tilts downward from front hinge",
        rest_aabb is not None and down_aabb is not None and down_aabb[0][2] < rest_aabb[0][2] - 0.015,
        details=f"rest={rest_aabb}, tilted={down_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
