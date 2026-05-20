from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="remote_weapon_station")

    dark_paint = model.material("dark_paint", rgba=(0.20, 0.21, 0.23, 1.0))
    steel = model.material("steel", rgba=(0.32, 0.33, 0.35, 1.0))
    black = model.material("black", rgba=(0.07, 0.07, 0.08, 1.0))
    glass = model.material("glass", rgba=(0.16, 0.24, 0.30, 1.0))
    shield = model.material("shield", rgba=(0.27, 0.29, 0.31, 1.0))

    mount_base = model.part("mount_base")
    mount_base.visual(
        Cylinder(radius=0.18, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=steel,
        name="base_disc",
    )
    mount_base.visual(
        Box((0.14, 0.14, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=dark_paint,
        name="adapter_block",
    )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.11, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=dark_paint,
        name="rotating_drum",
    )
    pedestal.visual(
        Box((0.06, 0.022, 0.12)),
        origin=Origin(xyz=(0.055, 0.10, 0.30)),
        material=dark_paint,
        name="left_ear",
    )
    pedestal.visual(
        Box((0.06, 0.022, 0.12)),
        origin=Origin(xyz=(0.055, -0.10, 0.30)),
        material=dark_paint,
        name="right_ear",
    )
    pedestal.visual(
        Box((0.045, 0.16, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        material=dark_paint,
        name="rear_brace",
    )

    tilt_frame = model.part("tilt_frame")
    tilt_frame.visual(
        Cylinder(radius=0.018, length=0.178),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="trunnion_shaft",
    )
    tilt_frame.visual(
        Box((0.22, 0.03, 0.10)),
        origin=Origin(xyz=(0.07, 0.075, -0.01)),
        material=dark_paint,
        name="left_arm",
    )
    tilt_frame.visual(
        Box((0.22, 0.03, 0.10)),
        origin=Origin(xyz=(0.07, -0.075, -0.01)),
        material=dark_paint,
        name="right_arm",
    )
    tilt_frame.visual(
        Box((0.12, 0.13, 0.025)),
        origin=Origin(xyz=(0.16, 0.0, -0.0725)),
        material=dark_paint,
        name="lower_cross_member",
    )

    gun_body = model.part("gun_body")
    gun_body.visual(
        Box((0.05, 0.08, 0.10)),
        origin=Origin(xyz=(-0.035, 0.0, 0.0)),
        material=black,
        name="rear_drive",
    )
    gun_body.visual(
        Box((0.24, 0.10, 0.12)),
        origin=Origin(xyz=(0.11, 0.0, 0.0)),
        material=black,
        name="receiver_body",
    )
    gun_body.visual(
        Cylinder(radius=0.03, length=0.22),
        origin=Origin(xyz=(0.30, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=black,
        name="barrel_jacket",
    )
    gun_body.visual(
        Cylinder(radius=0.012, length=0.33),
        origin=Origin(xyz=(0.485, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="barrel",
    )
    gun_body.visual(
        Cylinder(radius=0.018, length=0.06),
        origin=Origin(xyz=(0.68, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="muzzle_brake",
    )

    optics_block = model.part("optics_block")
    optics_block.visual(
        Box((0.08, 0.035, 0.05)),
        origin=Origin(xyz=(0.04, 0.0, 0.025)),
        material=dark_paint,
        name="mount_foot",
    )
    optics_block.visual(
        Box((0.17, 0.11, 0.13)),
        origin=Origin(xyz=(0.085, 0.0, 0.115)),
        material=dark_paint,
        name="main_housing",
    )
    optics_block.visual(
        Box((0.055, 0.12, 0.10)),
        origin=Origin(xyz=(0.195, 0.0, 0.105)),
        material=dark_paint,
        name="sensor_hood",
    )
    optics_block.visual(
        Box((0.008, 0.072, 0.058)),
        origin=Origin(xyz=(0.219, 0.0, 0.105)),
        material=glass,
        name="sensor_window",
    )
    optics_block.visual(
        Box((0.02, 0.12, 0.02)),
        origin=Origin(xyz=(0.2075, 0.0, 0.155)),
        material=dark_paint,
        name="hood_brow",
    )

    shield_flap = model.part("shield_flap")
    shield_flap.visual(
        Box((0.018, 0.11, 0.012)),
        origin=Origin(xyz=(0.009, 0.0, -0.006)),
        material=shield,
        name="hinge_rib",
    )
    shield_flap.visual(
        Box((0.012, 0.11, 0.065)),
        origin=Origin(xyz=(0.006, 0.0, -0.0325)),
        material=shield,
        name="flap_panel",
    )

    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=mount_base,
        child=pedestal,
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=200.0,
            velocity=1.2,
            lower=-pi,
            upper=pi,
        ),
    )
    model.articulation(
        "pedestal_pitch",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=tilt_frame,
        origin=Origin(xyz=(0.055, 0.0, 0.31)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.8,
            lower=-0.45,
            upper=1.0,
        ),
    )
    model.articulation(
        "frame_to_gun",
        ArticulationType.FIXED,
        parent=tilt_frame,
        child=gun_body,
        origin=Origin(xyz=(0.10, 0.0, 0.0)),
    )
    model.articulation(
        "frame_to_optics",
        ArticulationType.FIXED,
        parent=tilt_frame,
        child=optics_block,
        origin=Origin(xyz=(0.12, 0.095, 0.04)),
    )
    model.articulation(
        "optics_to_shield",
        ArticulationType.REVOLUTE,
        parent=optics_block,
        child=shield_flap,
        origin=Origin(xyz=(0.2225, 0.0, 0.155)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=2.5,
            lower=0.0,
            upper=1.1,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    mount_base = object_model.get_part("mount_base")
    pedestal = object_model.get_part("pedestal")
    tilt_frame = object_model.get_part("tilt_frame")
    gun_body = object_model.get_part("gun_body")
    optics_block = object_model.get_part("optics_block")
    shield_flap = object_model.get_part("shield_flap")

    yaw = object_model.get_articulation("base_yaw")
    pitch = object_model.get_articulation("pedestal_pitch")
    shield_hinge = object_model.get_articulation("optics_to_shield")

    def aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    ctx.expect_contact(mount_base, pedestal, name="pedestal sits on the pan base")
    ctx.expect_contact(pedestal, tilt_frame, name="tilt frame is carried by pedestal trunnions")
    ctx.expect_contact(
        gun_body,
        tilt_frame,
        elem_a="receiver_body",
        elem_b="lower_cross_member",
        name="gun receiver is seated in the tilt frame",
    )
    ctx.expect_contact(
        optics_block,
        tilt_frame,
        elem_a="mount_foot",
        elem_b="left_arm",
        name="optics block is bolted onto the frame rail",
    )
    ctx.expect_contact(
        shield_flap,
        optics_block,
        elem_a="flap_panel",
        elem_b="sensor_hood",
        name="shield flap rests against the optics hood",
    )

    muzzle_rest = aabb_center(ctx.part_element_world_aabb(gun_body, elem="muzzle_brake"))
    with ctx.pose({pitch: 0.75}):
        muzzle_pitched = aabb_center(ctx.part_element_world_aabb(gun_body, elem="muzzle_brake"))
    ctx.check(
        "positive pitch raises the gun",
        muzzle_rest is not None
        and muzzle_pitched is not None
        and muzzle_pitched[2] > muzzle_rest[2] + 0.18,
        details=f"rest={muzzle_rest}, pitched={muzzle_pitched}",
    )

    with ctx.pose({yaw: 0.65}):
        muzzle_yawed = aabb_center(ctx.part_element_world_aabb(gun_body, elem="muzzle_brake"))
    ctx.check(
        "positive yaw slews the gun to the left",
        muzzle_rest is not None
        and muzzle_yawed is not None
        and muzzle_yawed[1] > muzzle_rest[1] + 0.12,
        details=f"rest={muzzle_rest}, yawed={muzzle_yawed}",
    )

    flap_rest = aabb_center(ctx.part_element_world_aabb(shield_flap, elem="flap_panel"))
    with ctx.pose({shield_hinge: 0.9}):
        flap_open = aabb_center(ctx.part_element_world_aabb(shield_flap, elem="flap_panel"))
    ctx.check(
        "shield flap folds forward when opened",
        flap_rest is not None
        and flap_open is not None
        and flap_open[0] > flap_rest[0] + 0.015,
        details=f"rest={flap_rest}, open={flap_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
