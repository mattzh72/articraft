from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_naval_remote_station")

    naval_gray = model.material("naval_gray", rgba=(0.36, 0.40, 0.42, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.08, 0.09, 0.10, 1.0))
    black = model.material("matte_black", rgba=(0.01, 0.012, 0.014, 1.0))
    glass = model.material("smoked_glass", rgba=(0.02, 0.08, 0.11, 0.72))
    safety_red = model.material("safety_red", rgba=(0.65, 0.04, 0.03, 1.0))

    base = model.part("deck_base")
    base.visual(
        Cylinder(radius=0.52, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=dark_gray,
        name="deck_plate",
    )
    base.visual(
        Cylinder(radius=0.38, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0775)),
        material=naval_gray,
        name="fixed_bearing_ring",
    )
    for i in range(12):
        angle = i * math.tau / 12.0
        base.visual(
            Cylinder(radius=0.016, length=0.010),
            origin=Origin(
                xyz=(0.445 * math.cos(angle), 0.445 * math.sin(angle), 0.060),
            ),
            material=black,
            name=f"deck_bolt_{i}",
        )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.335, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=naval_gray,
        name="rotary_drum",
    )
    pedestal.visual(
        Cylinder(radius=0.225, length=0.280),
        origin=Origin(xyz=(0.0, 0.0, 0.290)),
        material=naval_gray,
        name="support_column",
    )

    body_profile = [
        (-0.405, 0.420),
        (0.335, 0.420),
        (0.425, 0.540),
        (0.260, 0.790),
        (-0.270, 0.840),
        (-0.430, 0.680),
    ]
    body_shape = (
        cq.Workplane("XZ")
        .polyline(body_profile)
        .close()
        .extrude(0.280, both=True)
    )
    pedestal.visual(
        mesh_from_cadquery(body_shape, "faceted_turret_body", tolerance=0.002),
        material=naval_gray,
        name="turret_body",
    )

    # Front cheeks visually support the pitching gun trunnion.
    pedestal.visual(
        Box((0.150, 0.080, 0.175)),
        origin=Origin(xyz=(0.390, 0.315, 0.620)),
        material=naval_gray,
        name="front_yoke_upper",
    )
    pedestal.visual(
        Box((0.150, 0.080, 0.175)),
        origin=Origin(xyz=(0.390, -0.315, 0.620)),
        material=naval_gray,
        name="front_yoke_lower",
    )
    pedestal.visual(
        Box((0.120, 0.530, 0.055)),
        origin=Origin(xyz=(0.355, 0.0, 0.535)),
        material=dark_gray,
        name="trunnion_shadow",
    )

    # A smoked sensor window sits under the articulated side cover.
    pedestal.visual(
        Box((0.185, 0.012, 0.155)),
        origin=Origin(xyz=(0.175, 0.285, 0.640)),
        material=glass,
        name="side_sensor_window",
    )
    pedestal.visual(
        Box((0.050, 0.022, 0.285)),
        origin=Origin(xyz=(0.300, 0.289, 0.640)),
        material=naval_gray,
        name="cover_hinge_leaf",
    )
    pedestal.visual(
        Box((0.175, 0.020, 0.030)),
        origin=Origin(xyz=(-0.125, 0.289, 0.835)),
        material=dark_gray,
        name="top_service_lip",
    )
    pedestal.visual(
        Box((0.210, 0.020, 0.035)),
        origin=Origin(xyz=(-0.160, -0.289, 0.650)),
        material=dark_gray,
        name="side_service_lip",
    )

    gun_mount = model.part("gun_mount")
    gun_mount.visual(
        Cylinder(radius=0.052, length=0.600),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="trunnion_hub",
    )
    gun_mount.visual(
        Box((0.170, 0.180, 0.115)),
        origin=Origin(xyz=(0.070, 0.0, -0.005)),
        material=dark_gray,
        name="receiver_block",
    )
    gun_mount.visual(
        Cylinder(radius=0.032, length=0.760),
        origin=Origin(xyz=(0.485, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="barrel",
    )
    gun_mount.visual(
        Cylinder(radius=0.046, length=0.070),
        origin=Origin(xyz=(0.895, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="muzzle_brake",
    )
    gun_mount.visual(
        Box((0.020, 0.120, 0.085)),
        origin=Origin(xyz=(0.895, 0.0, 0.0)),
        material=black,
        name="muzzle_baffle",
    )

    sensor_cover = model.part("sensor_cover")
    sensor_cover.visual(
        Box((0.245, 0.028, 0.245)),
        origin=Origin(xyz=(-0.1225, 0.016, 0.000)),
        material=naval_gray,
        name="armored_cover",
    )
    sensor_cover.visual(
        Cylinder(radius=0.019, length=0.285),
        origin=Origin(xyz=(0.0, 0.016, 0.0)),
        material=dark_gray,
        name="hinge_barrel",
    )
    sensor_cover.visual(
        Box((0.020, 0.020, 0.150)),
        origin=Origin(xyz=(-0.224, 0.033, 0.0)),
        material=safety_red,
        name="release_handle",
    )

    model.articulation(
        "base_to_pedestal",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=pedestal,
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=250.0, velocity=1.0),
    )
    model.articulation(
        "pedestal_to_gun_mount",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=gun_mount,
        origin=Origin(xyz=(0.460, 0.0, 0.620)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.4, lower=-0.25, upper=0.70),
    )
    model.articulation(
        "pedestal_to_sensor_cover",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=sensor_cover,
        origin=Origin(xyz=(0.300, 0.305, 0.640)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.1, lower=0.0, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("deck_base")
    pedestal = object_model.get_part("pedestal")
    gun = object_model.get_part("gun_mount")
    cover = object_model.get_part("sensor_cover")
    yaw = object_model.get_articulation("base_to_pedestal")
    pitch = object_model.get_articulation("pedestal_to_gun_mount")
    cover_hinge = object_model.get_articulation("pedestal_to_sensor_cover")

    ctx.allow_overlap(
        gun,
        pedestal,
        elem_a="trunnion_hub",
        elem_b="front_yoke_upper",
        reason="The gun trunnion is intentionally captured inside the upper yoke cheek bearing.",
    )
    ctx.allow_overlap(
        gun,
        pedestal,
        elem_a="trunnion_hub",
        elem_b="front_yoke_lower",
        reason="The gun trunnion is intentionally captured inside the lower yoke cheek bearing.",
    )

    ctx.check(
        "pedestal yaw uses vertical continuous axis",
        yaw.articulation_type == ArticulationType.CONTINUOUS and tuple(yaw.axis) == (0.0, 0.0, 1.0),
        details=f"type={yaw.articulation_type}, axis={yaw.axis}",
    )
    ctx.check(
        "gun pitch axis is horizontal",
        pitch.motion_limits is not None
        and pitch.motion_limits.lower < 0.0
        and pitch.motion_limits.upper > 0.5
        and tuple(pitch.axis) == (0.0, -1.0, 0.0),
        details=f"axis={pitch.axis}, limits={pitch.motion_limits}",
    )
    ctx.check(
        "cover hinge opens on vertical side axis",
        cover_hinge.motion_limits is not None
        and cover_hinge.motion_limits.lower == 0.0
        and cover_hinge.motion_limits.upper > 1.0
        and tuple(cover_hinge.axis) == (0.0, 0.0, -1.0),
        details=f"axis={cover_hinge.axis}, limits={cover_hinge.motion_limits}",
    )

    ctx.expect_gap(
        pedestal,
        base,
        axis="z",
        positive_elem="rotary_drum",
        negative_elem="fixed_bearing_ring",
        max_gap=0.001,
        max_penetration=0.00001,
        name="rotary drum seats on fixed bearing ring",
    )
    ctx.expect_overlap(
        gun,
        pedestal,
        axes="yz",
        elem_a="trunnion_hub",
        elem_b="front_yoke_upper",
        min_overlap=0.010,
        name="trunnion remains captured in upper yoke",
    )
    ctx.expect_overlap(
        gun,
        pedestal,
        axes="yz",
        elem_a="trunnion_hub",
        elem_b="front_yoke_lower",
        min_overlap=0.010,
        name="trunnion remains captured in lower yoke",
    )
    ctx.expect_overlap(
        cover,
        pedestal,
        axes="xz",
        elem_a="armored_cover",
        elem_b="side_sensor_window",
        min_overlap=0.120,
        name="closed cover protects side sensor window",
    )

    rest_muzzle = ctx.part_element_world_aabb(gun, elem="muzzle_brake")
    rest_handle = ctx.part_element_world_aabb(cover, elem="release_handle")
    with ctx.pose({pitch: pitch.motion_limits.upper, cover_hinge: cover_hinge.motion_limits.upper}):
        raised_muzzle = ctx.part_element_world_aabb(gun, elem="muzzle_brake")
        opened_handle = ctx.part_element_world_aabb(cover, elem="release_handle")

    def aabb_center_y(box):
        return None if box is None else 0.5 * (box[0][1] + box[1][1])

    def aabb_center_z(box):
        return None if box is None else 0.5 * (box[0][2] + box[1][2])

    ctx.check(
        "gun mount pitches upward at upper limit",
        rest_muzzle is not None
        and raised_muzzle is not None
        and aabb_center_z(raised_muzzle) > aabb_center_z(rest_muzzle) + 0.20,
        details=f"rest={rest_muzzle}, raised={raised_muzzle}",
    )
    ctx.check(
        "sensor cover swings outward from side",
        rest_handle is not None
        and opened_handle is not None
        and aabb_center_y(opened_handle) > aabb_center_y(rest_handle) + 0.10,
        details=f"rest={rest_handle}, opened={opened_handle}",
    )

    return ctx.report()


object_model = build_object_model()
