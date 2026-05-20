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


def _add_wheel_visuals(part, *, tire_mat, rim_mat, hub_mat) -> None:
    spin_rpy = (pi / 2.0, 0.0, 0.0)
    part.visual(
        Cylinder(radius=0.030, length=0.018),
        origin=Origin(rpy=spin_rpy),
        material=tire_mat,
        name="tire",
    )
    part.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(rpy=spin_rpy),
        material=rim_mat,
        name="rim",
    )
    part.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(rpy=spin_rpy),
        material=hub_mat,
        name="hub",
    )


def _add_door_visuals(part, *, side_sign: float, body_mat, glass_mat, stripe_mat) -> None:
    thickness = 0.006
    outer_y = side_sign * thickness * 0.5
    stripe_y = side_sign * 0.0036
    glass_y = side_sign * 0.0034
    part.visual(
        Box((0.082, 0.006, 0.040)),
        origin=Origin(xyz=(0.041, outer_y, 0.020)),
        material=body_mat,
        name="door_panel",
    )
    part.visual(
        Box((0.044, 0.0016, 0.016)),
        origin=Origin(xyz=(0.054, glass_y, 0.031)),
        material=glass_mat,
        name="door_window",
    )
    part.visual(
        Box((0.048, 0.0018, 0.012)),
        origin=Origin(xyz=(0.050, stripe_y, 0.014)),
        material=stripe_mat,
        name="police_stripe",
    )
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toy_police_sedan")

    black_paint = model.material("black_paint", rgba=(0.12, 0.13, 0.15, 1.0))
    white_paint = model.material("white_paint", rgba=(0.93, 0.94, 0.95, 1.0))
    bumper_grey = model.material("bumper_grey", rgba=(0.34, 0.35, 0.37, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    steel = model.material("steel", rgba=(0.71, 0.73, 0.76, 1.0))
    dark_hub = model.material("dark_hub", rgba=(0.22, 0.23, 0.25, 1.0))
    window_tint = model.material("window_tint", rgba=(0.36, 0.48, 0.56, 0.45))
    police_blue = model.material("police_blue", rgba=(0.10, 0.23, 0.66, 1.0))
    beacon_red = model.material("beacon_red", rgba=(0.82, 0.10, 0.12, 0.9))
    beacon_blue = model.material("beacon_blue", rgba=(0.10, 0.25, 0.86, 0.9))

    body = model.part("body")
    body.visual(
        Box((0.245, 0.078, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=black_paint,
        name="lower_body",
    )
    body.visual(
        Box((0.080, 0.092, 0.030)),
        origin=Origin(xyz=(-0.082, 0.0, 0.033)),
        material=black_paint,
        name="front_clip",
    )
    body.visual(
        Box((0.074, 0.092, 0.022)),
        origin=Origin(xyz=(0.086, 0.0, 0.031)),
        material=black_paint,
        name="rear_clip_base",
    )
    body.visual(
        Box((0.122, 0.074, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=white_paint,
        name="cabin_core",
    )
    body.visual(
        Box((0.018, 0.009, 0.034)),
        origin=Origin(xyz=(-0.041, 0.033, 0.062)),
        material=white_paint,
        name="left_a_pillar",
    )
    body.visual(
        Box((0.018, 0.009, 0.034)),
        origin=Origin(xyz=(-0.041, -0.033, 0.062)),
        material=white_paint,
        name="right_a_pillar",
    )
    body.visual(
        Box((0.012, 0.009, 0.034)),
        origin=Origin(xyz=(0.048, 0.033, 0.062)),
        material=white_paint,
        name="left_b_pillar",
    )
    body.visual(
        Box((0.012, 0.009, 0.034)),
        origin=Origin(xyz=(0.048, -0.033, 0.062)),
        material=white_paint,
        name="right_b_pillar",
    )
    body.visual(
        Box((0.020, 0.008, 0.038)),
        origin=Origin(xyz=(0.052, 0.029, 0.060)),
        material=white_paint,
        name="left_c_pillar",
    )
    body.visual(
        Box((0.020, 0.008, 0.038)),
        origin=Origin(xyz=(0.052, -0.029, 0.060)),
        material=white_paint,
        name="right_c_pillar",
    )
    body.visual(
        Box((0.090, 0.082, 0.012)),
        origin=Origin(xyz=(0.002, 0.0, 0.084)),
        material=white_paint,
        name="roof",
    )
    body.visual(
        Box((0.070, 0.084, 0.010)),
        origin=Origin(xyz=(-0.082, 0.0, 0.052)),
        material=black_paint,
        name="hood_top",
    )
    body.visual(
        Box((0.006, 0.072, 0.034)),
        origin=Origin(xyz=(-0.030, 0.0, 0.067), rpy=(0.0, -0.68, 0.0)),
        material=window_tint,
        name="windshield",
    )
    body.visual(
        Box((0.006, 0.068, 0.028)),
        origin=Origin(xyz=(0.052, 0.0, 0.066), rpy=(0.0, 0.72, 0.0)),
        material=window_tint,
        name="rear_window",
    )
    body.visual(
        Box((0.012, 0.082, 0.013)),
        origin=Origin(xyz=(0.058, 0.0, 0.0485)),
        material=black_paint,
        name="trunk_front_frame",
    )
    body.visual(
        Box((0.048, 0.008, 0.013)),
        origin=Origin(xyz=(0.088, 0.041, 0.0485)),
        material=black_paint,
        name="left_trunk_rail",
    )
    body.visual(
        Box((0.048, 0.008, 0.013)),
        origin=Origin(xyz=(0.088, -0.041, 0.0485)),
        material=black_paint,
        name="right_trunk_rail",
    )
    body.visual(
        Box((0.012, 0.082, 0.013)),
        origin=Origin(xyz=(0.118, 0.0, 0.0485)),
        material=black_paint,
        name="rear_sill",
    )
    body.visual(
        Box((0.018, 0.094, 0.010)),
        origin=Origin(xyz=(-0.121, 0.0, 0.022)),
        material=bumper_grey,
        name="front_bumper",
    )
    body.visual(
        Box((0.014, 0.094, 0.010)),
        origin=Origin(xyz=(0.123, 0.0, 0.022)),
        material=bumper_grey,
        name="rear_bumper",
    )
    body.visual(
        Box((0.020, 0.028, 0.004)),
        origin=Origin(xyz=(0.000, 0.0, 0.091)),
        material=steel,
        name="lightbar_base",
    )
    body.visual(
        Box((0.009, 0.012, 0.006)),
        origin=Origin(xyz=(0.000, 0.007, 0.095)),
        material=beacon_red,
        name="lightbar_red",
    )
    body.visual(
        Box((0.009, 0.012, 0.006)),
        origin=Origin(xyz=(0.000, -0.007, 0.095)),
        material=beacon_blue,
        name="lightbar_blue",
    )

    left_front_door = model.part("left_front_door")
    _add_door_visuals(
        left_front_door,
        side_sign=1.0,
        body_mat=white_paint,
        glass_mat=window_tint,
        stripe_mat=police_blue,
    )

    right_front_door = model.part("right_front_door")
    _add_door_visuals(
        right_front_door,
        side_sign=-1.0,
        body_mat=white_paint,
        glass_mat=window_tint,
        stripe_mat=police_blue,
    )

    trunk_lid = model.part("trunk_lid")
    trunk_lid.visual(
        Box((0.048, 0.072, 0.006)),
        origin=Origin(xyz=(0.024, 0.0, 0.003)),
        material=black_paint,
        name="trunk_panel",
    )
    trunk_lid.visual(
        Box((0.010, 0.064, 0.002)),
        origin=Origin(xyz=(0.018, 0.0, 0.0065)),
        material=police_blue,
        name="trunk_badge",
    )

    wheel_parts = {}
    for wheel_name in (
        "front_left_wheel",
        "front_right_wheel",
        "rear_left_wheel",
        "rear_right_wheel",
    ):
        wheel = model.part(wheel_name)
        _add_wheel_visuals(wheel, tire_mat=wheel_rubber, rim_mat=steel, hub_mat=dark_hub)
        wheel_parts[wheel_name] = wheel

    model.articulation(
        "left_front_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_front_door,
        origin=Origin(xyz=(-0.040, 0.0405, 0.038)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=1.15),
    )
    model.articulation(
        "right_front_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_front_door,
        origin=Origin(xyz=(-0.040, -0.0405, 0.038)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=1.15),
    )
    model.articulation(
        "trunk_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=trunk_lid,
        origin=Origin(xyz=(0.064, 0.0, 0.055)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.0, lower=0.0, upper=1.05),
    )

    wheel_origins = {
        "front_left_wheel_spin": (-0.076, 0.055, 0.030),
        "front_right_wheel_spin": (-0.076, -0.055, 0.030),
        "rear_left_wheel_spin": (0.078, 0.055, 0.030),
        "rear_right_wheel_spin": (0.078, -0.055, 0.030),
    }
    wheel_children = {
        "front_left_wheel_spin": wheel_parts["front_left_wheel"],
        "front_right_wheel_spin": wheel_parts["front_right_wheel"],
        "rear_left_wheel_spin": wheel_parts["rear_left_wheel"],
        "rear_right_wheel_spin": wheel_parts["rear_right_wheel"],
    }
    for joint_name, origin_xyz in wheel_origins.items():
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel_children[joint_name],
            origin=Origin(xyz=origin_xyz),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=20.0),
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

    body = object_model.get_part("body")
    left_front_door = object_model.get_part("left_front_door")
    right_front_door = object_model.get_part("right_front_door")
    trunk_lid = object_model.get_part("trunk_lid")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")

    left_hinge = object_model.get_articulation("left_front_door_hinge")
    right_hinge = object_model.get_articulation("right_front_door_hinge")
    trunk_hinge = object_model.get_articulation("trunk_hinge")

    wheel_joints = [
        object_model.get_articulation("front_left_wheel_spin"),
        object_model.get_articulation("front_right_wheel_spin"),
        object_model.get_articulation("rear_left_wheel_spin"),
        object_model.get_articulation("rear_right_wheel_spin"),
    ]

    expected_parts = [
        body,
        left_front_door,
        right_front_door,
        trunk_lid,
        front_left_wheel,
        front_right_wheel,
        rear_left_wheel,
        rear_right_wheel,
    ]
    ctx.check(
        "all requested parts exist",
        all(part is not None for part in expected_parts),
    )

    ctx.check(
        "front doors use vertical hinges",
        left_hinge.articulation_type == ArticulationType.REVOLUTE
        and right_hinge.articulation_type == ArticulationType.REVOLUTE
        and left_hinge.axis == (0.0, 0.0, 1.0)
        and right_hinge.axis == (0.0, 0.0, -1.0),
        details=f"left_axis={left_hinge.axis}, right_axis={right_hinge.axis}",
    )
    ctx.check(
        "trunk lid uses a rear horizontal hinge",
        trunk_hinge.articulation_type == ArticulationType.REVOLUTE and trunk_hinge.axis == (0.0, -1.0, 0.0),
        details=f"trunk_axis={trunk_hinge.axis}",
    )
    ctx.check(
        "wheels spin continuously",
        all(
            joint.articulation_type == ArticulationType.CONTINUOUS
            and joint.axis == (0.0, 1.0, 0.0)
            and joint.motion_limits is not None
            and joint.motion_limits.lower is None
            and joint.motion_limits.upper is None
            for joint in wheel_joints
        ),
        details=str([(joint.name, joint.articulation_type, joint.axis, joint.motion_limits) for joint in wheel_joints]),
    )

    ctx.expect_overlap(
        left_front_door,
        body,
        axes="xz",
        min_overlap=0.030,
        name="left door sits in the cabin side opening",
    )
    ctx.expect_overlap(
        right_front_door,
        body,
        axes="xz",
        min_overlap=0.030,
        name="right door sits in the cabin side opening",
    )
    ctx.expect_overlap(
        trunk_lid,
        body,
        axes="xy",
        min_overlap=0.040,
        name="trunk lid covers the rear deck opening",
    )

    closed_left = ctx.part_world_aabb(left_front_door)
    closed_right = ctx.part_world_aabb(right_front_door)
    closed_trunk = ctx.part_world_aabb(trunk_lid)
    with ctx.pose({left_hinge: 1.0, right_hinge: 1.0, trunk_hinge: 0.90}):
        open_left = ctx.part_world_aabb(left_front_door)
        open_right = ctx.part_world_aabb(right_front_door)
        open_trunk = ctx.part_world_aabb(trunk_lid)

    ctx.check(
        "left door swings outward",
        closed_left is not None and open_left is not None and open_left[1][1] > closed_left[1][1] + 0.020,
        details=f"closed_left={closed_left}, open_left={open_left}",
    )
    ctx.check(
        "right door swings outward",
        closed_right is not None and open_right is not None and open_right[0][1] < closed_right[0][1] - 0.020,
        details=f"closed_right={closed_right}, open_right={open_right}",
    )
    ctx.check(
        "trunk lid lifts upward",
        closed_trunk is not None and open_trunk is not None and open_trunk[1][2] > closed_trunk[1][2] + 0.020,
        details=f"closed_trunk={closed_trunk}, open_trunk={open_trunk}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
