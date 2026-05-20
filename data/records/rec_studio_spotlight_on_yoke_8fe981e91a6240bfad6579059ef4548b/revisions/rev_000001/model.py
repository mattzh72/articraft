from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retro_studio_spotlight")

    painted_steel = model.material("painted_steel", rgba=(0.27, 0.29, 0.30, 1.0))
    yoke_gray = model.material("yoke_gray", rgba=(0.56, 0.58, 0.60, 1.0))
    hardware = model.material("hardware", rgba=(0.70, 0.72, 0.74, 1.0))
    knob_black = model.material("knob_black", rgba=(0.11, 0.11, 0.12, 1.0))
    base_black = model.material("base_black", rgba=(0.18, 0.18, 0.19, 1.0))

    base_plate_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.42, 0.30, 0.035), 0.03),
        "stand_base_plate",
    )

    can_shell_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.022, -0.135),
                (0.050, -0.128),
                (0.082, -0.110),
                (0.104, -0.070),
                (0.112, -0.010),
                (0.113, 0.120),
                (0.118, 0.210),
                (0.122, 0.275),
                (0.116, 0.290),
            ],
            [
                (0.010, -0.125),
                (0.042, -0.118),
                (0.074, -0.102),
                (0.095, -0.064),
                (0.102, -0.004),
                (0.103, 0.116),
                (0.107, 0.206),
                (0.109, 0.282),
            ],
            segments=56,
        ),
        "spotlight_can_shell",
    )
    front_rim_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.113, tube=0.008, radial_segments=18, tubular_segments=56),
        "spotlight_front_rim",
    )

    stand = model.part("stand_base")
    stand.visual(
        base_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=base_black,
        name="base_plate",
    )
    stand.visual(
        Cylinder(radius=0.034, length=0.33),
        origin=Origin(xyz=(0.0, 0.0, 0.195)),
        material=painted_steel,
        name="center_post",
    )
    stand.visual(
        Box((0.12, 0.10, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.380)),
        material=painted_steel,
        name="post_collar",
    )
    stand.visual(
        Box((0.18, 0.10, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.411)),
        material=hardware,
        name="top_adapter_plate",
    )
    stand.visual(
        Box((0.018, 0.08, 0.12)),
        origin=Origin(xyz=(0.045, 0.0, 0.326)),
        material=painted_steel,
        name="front_reinforcement",
    )
    stand.visual(
        Box((0.018, 0.08, 0.12)),
        origin=Origin(xyz=(-0.045, 0.0, 0.326)),
        material=painted_steel,
        name="rear_reinforcement",
    )
    stand.visual(
        Box((0.08, 0.018, 0.12)),
        origin=Origin(xyz=(0.0, 0.045, 0.326)),
        material=painted_steel,
        name="left_reinforcement",
    )
    stand.visual(
        Box((0.08, 0.018, 0.12)),
        origin=Origin(xyz=(0.0, -0.045, 0.326)),
        material=painted_steel,
        name="right_reinforcement",
    )
    for index, (bx, by) in enumerate(
        ((0.067, 0.034), (0.067, -0.034))
    ):
        stand.visual(
            Cylinder(radius=0.007, length=0.008),
            origin=Origin(xyz=(bx, by, 0.421)),
            material=hardware,
            name=f"adapter_bolt_{index}",
        )
    stand.inertial = Inertial.from_geometry(
        Box((0.42, 0.30, 0.43)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, 0.215)),
    )

    yoke = model.part("yoke")
    yoke.visual(
        Box((0.11, 0.06, 0.08)),
        origin=Origin(xyz=(-0.050, 0.0, 0.040)),
        material=yoke_gray,
        name="mount_block",
    )
    yoke.visual(
        Box((0.05, 0.32, 0.04)),
        origin=Origin(xyz=(-0.040, 0.0, 0.065)),
        material=yoke_gray,
        name="lower_bridge",
    )
    yoke.visual(
        Box((0.03, 0.03, 0.34)),
        origin=Origin(xyz=(0.0, 0.160, 0.215)),
        material=yoke_gray,
        name="left_arm",
    )
    yoke.visual(
        Box((0.03, 0.03, 0.34)),
        origin=Origin(xyz=(0.0, -0.160, 0.215)),
        material=yoke_gray,
        name="right_arm",
    )
    yoke.visual(
        Box((0.018, 0.012, 0.12)),
        origin=Origin(xyz=(0.014, 0.181, 0.235)),
        material=hardware,
        name="left_doubler",
    )
    yoke.visual(
        Box((0.018, 0.012, 0.12)),
        origin=Origin(xyz=(0.014, -0.181, 0.235)),
        material=hardware,
        name="right_doubler",
    )
    yoke.visual(
        Box((0.016, 0.060, 0.026)),
        origin=Origin(xyz=(-0.064, 0.045, 0.070)),
        material=yoke_gray,
        name="left_gusset",
    )
    yoke.visual(
        Box((0.016, 0.060, 0.026)),
        origin=Origin(xyz=(-0.064, -0.045, 0.070)),
        material=yoke_gray,
        name="right_gusset",
    )
    yoke.visual(
        Box((0.02, 0.32, 0.03)),
        origin=Origin(xyz=(-0.082, 0.0, 0.095)),
        material=painted_steel,
        name="rear_tie_plate",
    )
    yoke.visual(
        Cylinder(radius=0.032, length=0.008),
        origin=Origin(xyz=(0.0, 0.146, 0.235), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="left_trunnion_pad",
    )
    yoke.visual(
        Cylinder(radius=0.032, length=0.008),
        origin=Origin(xyz=(0.0, -0.146, 0.235), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="right_trunnion_pad",
    )
    yoke.visual(
        Cylinder(radius=0.013, length=0.016),
        origin=Origin(xyz=(0.0, 0.158, 0.235), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="left_lock_shaft",
    )
    yoke.visual(
        Cylinder(radius=0.036, length=0.024),
        origin=Origin(xyz=(0.0, 0.178, 0.235), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="left_lock_knob",
    )
    yoke.visual(
        Cylinder(radius=0.013, length=0.016),
        origin=Origin(xyz=(0.0, -0.158, 0.235), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="right_lock_shaft",
    )
    yoke.visual(
        Cylinder(radius=0.036, length=0.024),
        origin=Origin(xyz=(0.0, -0.178, 0.235), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="right_lock_knob",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.16, 0.42, 0.39)),
        mass=3.4,
        origin=Origin(xyz=(-0.03, 0.0, 0.195)),
    )

    lamp = model.part("lamp_can")
    lamp.visual(
        can_shell_mesh,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=painted_steel,
        name="can_shell",
    )
    lamp.visual(
        front_rim_mesh,
        origin=Origin(xyz=(0.280, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="front_rim",
    )
    lamp.visual(
        Cylinder(radius=0.046, length=0.024),
        origin=Origin(xyz=(-0.126, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=painted_steel,
        name="rear_cap",
    )
    lamp.visual(
        Box((0.14, 0.08, 0.008)),
        origin=Origin(xyz=(0.020, 0.0, 0.116)),
        material=hardware,
        name="top_service_hatch",
    )
    lamp.visual(
        Box((0.012, 0.050, 0.014)),
        origin=Origin(xyz=(-0.040, 0.0, 0.109)),
        material=hardware,
        name="top_hatch_left_rail",
    )
    lamp.visual(
        Box((0.012, 0.050, 0.014)),
        origin=Origin(xyz=(0.065, 0.0, 0.109)),
        material=hardware,
        name="top_hatch_right_rail",
    )
    lamp.visual(
        Box((0.09, 0.07, 0.028)),
        origin=Origin(xyz=(-0.092, 0.0, 0.086)),
        material=hardware,
        name="rear_service_hatch",
    )
    for index, (bx, by, bz) in enumerate(
        (
            (-0.040, 0.028, 0.123),
            (-0.040, -0.028, 0.123),
            (0.080, 0.028, 0.123),
            (0.080, -0.028, 0.123),
            (-0.120, 0.022, 0.100),
            (-0.120, -0.022, 0.100),
            (-0.064, 0.022, 0.100),
            (-0.064, -0.022, 0.100),
        )
    ):
        lamp.visual(
            Cylinder(radius=0.0055, length=0.008),
            origin=Origin(xyz=(bx, by, bz)),
            material=hardware,
            name=f"service_bolt_{index}",
        )
    lamp.visual(
        Cylinder(radius=0.050, length=0.010),
        origin=Origin(xyz=(0.0, 0.117, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="left_trunnion_plate",
    )
    lamp.visual(
        Cylinder(radius=0.028, length=0.033),
        origin=Origin(xyz=(0.0, 0.1285, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="left_trunnion_boss",
    )
    lamp.visual(
        Cylinder(radius=0.050, length=0.010),
        origin=Origin(xyz=(0.0, -0.117, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="right_trunnion_plate",
    )
    lamp.visual(
        Cylinder(radius=0.028, length=0.033),
        origin=Origin(xyz=(0.0, -0.1285, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="right_trunnion_boss",
    )
    lamp.inertial = Inertial.from_geometry(
        Box((0.42, 0.30, 0.30)),
        mass=4.6,
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
    )

    model.articulation(
        "stand_to_yoke",
        ArticulationType.FIXED,
        parent=stand,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.417)),
    )
    model.articulation(
        "yoke_tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=lamp,
        origin=Origin(xyz=(0.0, 0.0, 0.235)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=math.radians(-20.0),
            upper=math.radians(55.0),
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand_base")
    yoke = object_model.get_part("yoke")
    lamp = object_model.get_part("lamp_can")
    tilt = object_model.get_articulation("yoke_tilt")

    stand_plate = stand.get_visual("top_adapter_plate")
    yoke_mount = yoke.get_visual("mount_block")
    left_pad = yoke.get_visual("left_trunnion_pad")
    right_pad = yoke.get_visual("right_trunnion_pad")
    left_boss = lamp.get_visual("left_trunnion_boss")
    right_boss = lamp.get_visual("right_trunnion_boss")
    front_rim = lamp.get_visual("front_rim")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=16)

    ctx.expect_contact(
        stand,
        yoke,
        elem_a=stand_plate,
        elem_b=yoke_mount,
        name="yoke_mounts_to_adapter_plate",
    )
    ctx.expect_contact(
        lamp,
        yoke,
        elem_a=left_boss,
        elem_b=left_pad,
        name="left_trunnion_is_carried_by_yoke",
    )
    ctx.expect_contact(
        lamp,
        yoke,
        elem_a=right_boss,
        elem_b=right_pad,
        name="right_trunnion_is_carried_by_yoke",
    )
    ctx.expect_gap(
        lamp,
        yoke,
        axis="z",
        positive_elem="can_shell",
        negative_elem="lower_bridge",
        min_gap=0.010,
        name="can_clears_lower_bridge_at_rest",
    )

    def _aabb_center_z(part, elem) -> float:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        assert aabb is not None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    with ctx.pose({tilt: 0.0}):
        closed_front_z = _aabb_center_z(lamp, front_rim)
    with ctx.pose({tilt: math.radians(45.0)}):
        raised_front_z = _aabb_center_z(lamp, front_rim)
    ctx.check(
        "positive_tilt_raises_front_of_spotlight",
        raised_front_z > closed_front_z + 0.05,
        details=(
            f"front rim center z should rise when opening upward; "
            f"closed={closed_front_z:.4f}, raised={raised_front_z:.4f}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
