from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def _rounded_box(size: tuple[float, float, float], center: tuple[float, float, float], radius: float = 0.0):
    """CadQuery rounded box in local model meters."""
    shape = cq.Workplane("XY").box(size[0], size[1], size[2])
    if radius > 0.0:
        try:
            shape = shape.edges("|Z").fillet(radius)
        except Exception:
            # A failed cosmetic fillet should not remove the load-bearing member.
            shape = cq.Workplane("XY").box(size[0], size[1], size[2])
    return shape.translate(center)


def _y_cylinder(length: float, radius: float, center: tuple[float, float, float]):
    """Cylinder with its axis on local +Y."""
    return cq.Workplane("XZ").cylinder(length, radius).translate(center)


def _union_all(*shapes):
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def _bolt_circle(
    part,
    *,
    name_prefix: str,
    material,
    y: float,
    radius: float,
    count: int,
    bolt_radius: float,
    bolt_length: float,
    center_z: float = 0.0,
) -> None:
    """Add stainless cap screws on a Y-facing bearing cover."""
    for i in range(count):
        angle = 2.0 * math.pi * i / count
        x = radius * math.cos(angle)
        z = center_z + radius * math.sin(angle)
        part.visual(
            Cylinder(radius=bolt_radius, length=bolt_length),
            origin=Origin(xyz=(x, y, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=material,
            name=f"{name_prefix}_{i}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_robotic_leg")

    armor = model.material("anodized_armor", rgba=(0.20, 0.23, 0.24, 1.0))
    dark_armor = model.material("dark_bay_cover", rgba=(0.055, 0.060, 0.062, 1.0))
    rubber = model.material("black_epdm_seal", rgba=(0.015, 0.014, 0.012, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.72, 0.72, 0.68, 1.0))
    warning = model.material("amber_service_label", rgba=(0.95, 0.58, 0.10, 1.0))

    # Root hip carriage: a roofed weatherproof yoke that anchors the serial leg.
    hip_mount = model.part("hip_mount")
    hip_gap = 0.130
    hip_cheek_thickness = 0.026
    hip_cheek_y = hip_gap / 2.0 + hip_cheek_thickness / 2.0
    hip_mount.visual(
        Box((0.36, 0.24, 0.046)),
        origin=Origin(xyz=(0.0, 0.0, 0.172)),
        material=armor,
        name="hip_base_plate",
    )
    hip_mount.visual(
        Box((0.23, 0.20, 0.065)),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=armor,
        name="hip_top_bridge",
    )
    hip_mount.visual(
        Box((0.185, hip_cheek_thickness, 0.255)),
        origin=Origin(xyz=(0.0, hip_cheek_y, 0.0)),
        material=armor,
        name="hip_cheek_outer",
    )
    hip_mount.visual(
        Box((0.185, hip_cheek_thickness, 0.255)),
        origin=Origin(xyz=(0.0, -hip_cheek_y, 0.0)),
        material=armor,
        name="hip_cheek_inner",
    )
    hip_mount.visual(
        Box((0.070, hip_gap, 0.070)),
        origin=Origin(xyz=(-0.075, 0.0, 0.124)),
        material=armor,
        name="hip_rear_bridge",
    )
    hip_mount.visual(
        Box((0.42, 0.30, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.2075)),
        material=dark_armor,
        name="hip_drip_hood",
    )
    for side, y in (("outer", hip_cheek_y + hip_cheek_thickness / 2.0 + 0.006), ("inner", -hip_cheek_y - hip_cheek_thickness / 2.0 - 0.006)):
        hip_mount.visual(
            Cylinder(radius=0.095, length=0.012),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"hip_seal_{side}",
        )
        _bolt_circle(
            hip_mount,
            name_prefix=f"hip_bolt_{side}",
            material=stainless,
            y=y + (0.006 if y > 0 else -0.006),
            radius=0.070,
            count=6,
            bolt_radius=0.007,
            bolt_length=0.006,
        )

    # Thigh: upper trunnion, load-bearing sealed shell, front actuator bay, and knee yoke.
    thigh = model.part("thigh")
    knee_gap = 0.134
    knee_cheek_thickness = 0.024
    knee_cheek_y = knee_gap / 2.0 + knee_cheek_thickness / 2.0
    thigh_shell = _union_all(
        _rounded_box((0.145, 0.108, 0.135), (0.0, 0.0, -0.084), 0.012),
        _rounded_box((0.165, 0.122, 0.440), (0.000, 0.0, -0.335), 0.020),
        _rounded_box((0.050, 0.094, 0.360), (-0.082, 0.0, -0.345), 0.010),
    )
    thigh.visual(
        mesh_from_cadquery(thigh_shell, "sealed_thigh_shell"),
        material=armor,
        name="thigh_shell",
    )
    thigh.visual(
        Cylinder(radius=0.073, length=hip_gap),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="hip_trunnion",
    )
    thigh.visual(
        Cylinder(radius=0.081, length=hip_gap),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="hip_hub",
    )
    thigh.visual(
        Box((0.050, 0.128, 0.300)),
        origin=Origin(xyz=(0.103, 0.0, -0.342)),
        material=dark_armor,
        name="thigh_actuator_bay",
    )
    thigh.visual(
        Box((0.030, 0.150, 0.024)),
        origin=Origin(xyz=(0.118, 0.0, -0.181)),
        material=dark_armor,
        name="thigh_drip_lip",
    )
    thigh.visual(
        Box((0.010, 0.135, 0.320)),
        origin=Origin(xyz=(0.130, 0.0, -0.342)),
        material=rubber,
        name="thigh_bay_gasket",
    )
    thigh.visual(
        Box((0.052, 0.090, 0.014)),
        origin=Origin(xyz=(0.136, 0.0, -0.486)),
        material=warning,
        name="thigh_service_label",
    )
    thigh.visual(
        Box((0.160, knee_cheek_thickness, 0.225)),
        origin=Origin(xyz=(0.0, knee_cheek_y, -0.650)),
        material=armor,
        name="knee_cheek_outer",
    )
    thigh.visual(
        Box((0.160, knee_cheek_thickness, 0.225)),
        origin=Origin(xyz=(0.0, -knee_cheek_y, -0.650)),
        material=armor,
        name="knee_cheek_inner",
    )
    thigh.visual(
        Box((0.075, knee_gap, 0.080)),
        origin=Origin(xyz=(-0.075, 0.0, -0.500)),
        material=armor,
        name="knee_rear_bridge",
    )
    for side, y in (("outer", knee_cheek_y + knee_cheek_thickness / 2.0 + 0.006), ("inner", -knee_cheek_y - knee_cheek_thickness / 2.0 - 0.006)):
        thigh.visual(
            Cylinder(radius=0.078, length=0.012),
            origin=Origin(xyz=(0.0, y, -0.650), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"knee_seal_{side}",
        )
        _bolt_circle(
            thigh,
            name_prefix=f"knee_bolt_{side}",
            material=stainless,
            y=y + (0.006 if y > 0 else -0.006),
            radius=0.058,
            count=6,
            bolt_radius=0.006,
            bolt_length=0.006,
            center_z=-0.650,
        )

    # Shank: captured knee hub, narrower sealed shell, serviceable actuator pod, and ankle fork.
    shank = model.part("shank")
    ankle_gap = 0.112
    ankle_cheek_thickness = 0.022
    ankle_cheek_y = ankle_gap / 2.0 + ankle_cheek_thickness / 2.0
    shank_shell = _union_all(
        _rounded_box((0.128, 0.104, 0.112), (0.0, 0.0, -0.072), 0.012),
        _rounded_box((0.140, 0.110, 0.375), (0.0, 0.0, -0.288), 0.018),
        _rounded_box((0.044, 0.086, 0.300), (-0.070, 0.0, -0.300), 0.009),
    )
    shank.visual(
        mesh_from_cadquery(shank_shell, "sealed_shank_shell"),
        material=armor,
        name="shank_shell",
    )
    shank.visual(
        Cylinder(radius=0.064, length=knee_gap),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="knee_trunnion",
    )
    shank.visual(
        Cylinder(radius=0.071, length=knee_gap),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="knee_hub",
    )
    shank.visual(
        Box((0.045, 0.116, 0.240)),
        origin=Origin(xyz=(0.091, 0.0, -0.290)),
        material=dark_armor,
        name="shank_actuator_bay",
    )
    shank.visual(
        Box((0.028, 0.136, 0.024)),
        origin=Origin(xyz=(0.106, 0.0, -0.164)),
        material=dark_armor,
        name="shank_drip_lip",
    )
    shank.visual(
        Box((0.010, 0.124, 0.260)),
        origin=Origin(xyz=(0.116, 0.0, -0.290)),
        material=rubber,
        name="shank_bay_gasket",
    )
    shank.visual(
        Box((0.132, ankle_cheek_thickness, 0.178)),
        origin=Origin(xyz=(0.0, ankle_cheek_y, -0.550)),
        material=armor,
        name="ankle_cheek_outer",
    )
    shank.visual(
        Box((0.132, ankle_cheek_thickness, 0.178)),
        origin=Origin(xyz=(0.0, -ankle_cheek_y, -0.550)),
        material=armor,
        name="ankle_cheek_inner",
    )
    shank.visual(
        Box((0.062, ankle_gap, 0.065)),
        origin=Origin(xyz=(-0.064, 0.0, -0.435)),
        material=armor,
        name="ankle_rear_bridge",
    )
    for side, y in (("outer", ankle_cheek_y + ankle_cheek_thickness / 2.0 + 0.005), ("inner", -ankle_cheek_y - ankle_cheek_thickness / 2.0 - 0.005)):
        shank.visual(
            Cylinder(radius=0.062, length=0.010),
            origin=Origin(xyz=(0.0, y, -0.550), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"ankle_seal_{side}",
        )
        _bolt_circle(
            shank,
            name_prefix=f"ankle_bolt_{side}",
            material=stainless,
            y=y + (0.005 if y > 0 else -0.005),
            radius=0.046,
            count=4,
            bolt_radius=0.0055,
            bolt_length=0.005,
            center_z=-0.550,
        )

    # Foot: sealed ankle trunnion and a grippy weatherproof sole with drip edge.
    foot = model.part("foot")
    foot.visual(
        Box((0.115, 0.092, 0.090)),
        origin=Origin(xyz=(0.020, 0.0, -0.050)),
        material=armor,
        name="ankle_block",
    )
    foot.visual(
        Box((0.390, 0.190, 0.048)),
        origin=Origin(xyz=(0.130, 0.0, -0.133)),
        material=armor,
        name="foot_frame",
    )
    foot.visual(
        Box((0.105, 0.150, 0.060)),
        origin=Origin(xyz=(-0.060, 0.0, -0.120)),
        material=armor,
        name="heel_block",
    )
    foot.visual(
        Cylinder(radius=0.054, length=ankle_gap),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="ankle_trunnion",
    )
    foot.visual(
        Cylinder(radius=0.060, length=ankle_gap),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="ankle_hub",
    )
    foot.visual(
        Box((0.430, 0.210, 0.030)),
        origin=Origin(xyz=(0.145, 0.0, -0.172)),
        material=rubber,
        name="rubber_sole",
    )
    for i, x in enumerate((-0.030, 0.060, 0.150, 0.240, 0.330)):
        foot.visual(
            Box((0.052, 0.180, 0.018)),
            origin=Origin(xyz=(x, 0.0, -0.196)),
            material=rubber,
            name=f"sole_tread_{i}",
        )
    foot.visual(
        Box((0.140, 0.098, 0.020)),
        origin=Origin(xyz=(-0.020, 0.0, -0.070)),
        material=dark_armor,
        name="ankle_drip_skirt",
    )

    model.articulation(
        "hip_pitch",
        ArticulationType.REVOLUTE,
        parent=hip_mount,
        child=thigh,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=420.0, velocity=2.1, lower=-0.65, upper=0.95),
        motion_properties=MotionProperties(damping=18.0, friction=1.2),
    )
    model.articulation(
        "knee_pitch",
        ArticulationType.REVOLUTE,
        parent=thigh,
        child=shank,
        origin=Origin(xyz=(0.0, 0.0, -0.650)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=360.0, velocity=2.4, lower=0.0, upper=1.75),
        motion_properties=MotionProperties(damping=14.0, friction=1.0),
    )
    model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent=shank,
        child=foot,
        origin=Origin(xyz=(0.0, 0.0, -0.550)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=210.0, velocity=2.8, lower=-0.60, upper=0.75),
        motion_properties=MotionProperties(damping=9.0, friction=0.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hip_mount = object_model.get_part("hip_mount")
    thigh = object_model.get_part("thigh")
    shank = object_model.get_part("shank")
    foot = object_model.get_part("foot")
    hip = object_model.get_articulation("hip_pitch")
    knee = object_model.get_articulation("knee_pitch")
    ankle = object_model.get_articulation("ankle_pitch")

    ctx.expect_contact(
        thigh,
        hip_mount,
        elem_a="hip_hub",
        elem_b="hip_cheek_outer",
        contact_tol=0.004,
        name="hip trunnion bears against sealed yoke",
    )
    ctx.expect_contact(
        shank,
        thigh,
        elem_a="knee_hub",
        elem_b="knee_cheek_outer",
        contact_tol=0.004,
        name="knee hub is captured by thigh yoke",
    )
    ctx.expect_contact(
        foot,
        shank,
        elem_a="ankle_hub",
        elem_b="ankle_cheek_outer",
        contact_tol=0.004,
        name="ankle hub is captured by shank fork",
    )
    ctx.expect_overlap(
        thigh,
        hip_mount,
        axes="xz",
        elem_a="hip_hub",
        elem_b="hip_cheek_outer",
        min_overlap=0.070,
        name="hip bearing has broad load path",
    )
    ctx.expect_overlap(
        shank,
        thigh,
        axes="xz",
        elem_a="knee_hub",
        elem_b="knee_cheek_outer",
        min_overlap=0.055,
        name="knee bearing has broad load path",
    )
    ctx.expect_overlap(
        foot,
        shank,
        axes="xz",
        elem_a="ankle_hub",
        elem_b="ankle_cheek_outer",
        min_overlap=0.045,
        name="ankle bearing has broad load path",
    )

    rest_foot = ctx.part_world_position(foot)
    with ctx.pose({knee: 1.0}):
        flexed_foot = ctx.part_world_position(foot)
    ctx.check(
        "knee pitch moves distal chain rearward",
        rest_foot is not None
        and flexed_foot is not None
        and flexed_foot[0] < rest_foot[0] - 0.25
        and flexed_foot[2] > rest_foot[2] + 0.15,
        details=f"rest={rest_foot}, flexed={flexed_foot}",
    )

    rest_knee = ctx.part_world_position(shank)
    with ctx.pose({hip: 0.55}):
        swung_knee = ctx.part_world_position(shank)
    ctx.check(
        "hip pitch swings whole leg as first serial joint",
        rest_knee is not None
        and swung_knee is not None
        and swung_knee[0] < rest_knee[0] - 0.25,
        details=f"rest={rest_knee}, swung={swung_knee}",
    )

    rest_aabb = ctx.part_world_aabb(foot)
    with ctx.pose({ankle: 0.55}):
        ankle_aabb = ctx.part_world_aabb(foot)
    ctx.check(
        "ankle pitch rotates the sealed foot",
        rest_aabb is not None
        and ankle_aabb is not None
        and ankle_aabb[0][0] < rest_aabb[0][0] - 0.03
        and ankle_aabb[1][2] > rest_aabb[1][2] + 0.015,
        details=f"rest={rest_aabb}, ankle={ankle_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
