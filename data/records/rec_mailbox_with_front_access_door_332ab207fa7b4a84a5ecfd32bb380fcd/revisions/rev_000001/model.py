from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_parcel_mailbox")

    body_color = model.material("body_color", rgba=(0.20, 0.23, 0.25, 1.0))
    pedestal_color = model.material("pedestal_color", rgba=(0.10, 0.10, 0.11, 1.0))
    hardware_color = model.material("hardware_color", rgba=(0.63, 0.66, 0.69, 1.0))

    pedestal_base_h = 0.02
    pedestal_h = 0.74
    collar_h = 0.04

    body_w = 0.46
    body_d = 0.38
    body_h = 0.56
    wall_t = 0.018
    body_z0 = pedestal_base_h + pedestal_h + collar_h
    body_z1 = body_z0 + body_h
    body_front_y = body_d / 2.0

    inner_w = body_w - 2.0 * wall_t
    upper_open_w = 0.284
    upper_cheek_w = (inner_w - upper_open_w) / 2.0

    floor_top_z = body_z0 + wall_t
    lower_open_top_z = 1.20
    rail_h = 0.04
    rail_top_z = lower_open_top_z + rail_h
    upper_hinge_z = body_z1 - wall_t

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((0.34, 0.28, pedestal_base_h)),
        origin=Origin(xyz=(0.0, 0.0, pedestal_base_h / 2.0)),
        material=pedestal_color,
        name="base_plate",
    )
    cabinet.visual(
        Box((0.16, 0.14, pedestal_h)),
        origin=Origin(xyz=(0.0, 0.0, pedestal_base_h + pedestal_h / 2.0)),
        material=pedestal_color,
        name="pedestal_post",
    )
    cabinet.visual(
        Box((0.20, 0.18, collar_h)),
        origin=Origin(xyz=(0.0, 0.0, pedestal_base_h + pedestal_h + collar_h / 2.0)),
        material=pedestal_color,
        name="mount_collar",
    )
    cabinet.visual(
        Box((body_w, body_d, wall_t)),
        origin=Origin(xyz=(0.0, 0.0, body_z0 + wall_t / 2.0)),
        material=body_color,
        name="floor_pan",
    )
    cabinet.visual(
        Box((body_w, body_d, wall_t)),
        origin=Origin(xyz=(0.0, 0.0, body_z1 - wall_t / 2.0)),
        material=body_color,
        name="roof_pan",
    )
    cabinet.visual(
        Box((body_w, wall_t, body_h)),
        origin=Origin(xyz=(0.0, -body_front_y + wall_t / 2.0, body_z0 + body_h / 2.0)),
        material=body_color,
        name="rear_wall",
    )
    cabinet.visual(
        Box((wall_t, body_d, body_h)),
        origin=Origin(
            xyz=(-body_w / 2.0 + wall_t / 2.0, 0.0, body_z0 + body_h / 2.0)
        ),
        material=body_color,
        name="left_wall",
    )
    cabinet.visual(
        Box((wall_t, body_d, body_h)),
        origin=Origin(
            xyz=(body_w / 2.0 - wall_t / 2.0, 0.0, body_z0 + body_h / 2.0)
        ),
        material=body_color,
        name="right_wall",
    )
    cabinet.visual(
        Box((inner_w, wall_t, rail_h)),
        origin=Origin(
            xyz=(
                0.0,
                body_front_y - wall_t / 2.0,
                lower_open_top_z + rail_h / 2.0,
            )
        ),
        material=body_color,
        name="mid_rail",
    )
    cabinet.visual(
        Box((upper_cheek_w, wall_t, upper_hinge_z - rail_top_z)),
        origin=Origin(
            xyz=(
                -(upper_open_w / 2.0 + upper_cheek_w / 2.0),
                body_front_y - wall_t / 2.0,
                rail_top_z + (upper_hinge_z - rail_top_z) / 2.0,
            )
        ),
        material=body_color,
        name="upper_left_face",
    )
    cabinet.visual(
        Box((upper_cheek_w, wall_t, upper_hinge_z - rail_top_z)),
        origin=Origin(
            xyz=(
                upper_open_w / 2.0 + upper_cheek_w / 2.0,
                body_front_y - wall_t / 2.0,
                rail_top_z + (upper_hinge_z - rail_top_z) / 2.0,
            )
        ),
        material=body_color,
        name="upper_right_face",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((body_w, body_d, body_z1)),
        mass=32.0,
        origin=Origin(xyz=(0.0, 0.0, body_z1 / 2.0)),
    )

    flap_w = upper_open_w - 0.006
    flap_h = upper_hinge_z - rail_top_z - 0.006
    flap_t = 0.014

    deposit_flap = model.part("deposit_flap")
    deposit_flap.visual(
        Box((flap_w, flap_t, flap_h)),
        origin=Origin(xyz=(0.0, flap_t / 2.0, -flap_h / 2.0)),
        material=body_color,
        name="flap_panel",
    )
    deposit_flap.visual(
        Box((flap_w * 0.76, 0.012, 0.015)),
        origin=Origin(xyz=(0.0, flap_t + 0.006, -flap_h + 0.0075)),
        material=hardware_color,
        name="flap_pull",
    )
    deposit_flap.inertial = Inertial.from_geometry(
        Box((flap_w, flap_t, flap_h)),
        mass=1.2,
        origin=Origin(xyz=(0.0, flap_t / 2.0, -flap_h / 2.0)),
    )

    door_w = inner_w - 0.006
    door_h = lower_open_top_z - floor_top_z - 0.006
    door_t = 0.016

    retrieval_door = model.part("retrieval_door")
    retrieval_door.visual(
        Box((door_w, door_t, door_h)),
        origin=Origin(xyz=(door_w / 2.0, door_t / 2.0, 0.0)),
        material=body_color,
        name="door_panel",
    )
    retrieval_door.visual(
        Box((0.026, 0.012, 0.12)),
        origin=Origin(xyz=(door_w - 0.046, door_t + 0.006, 0.0)),
        material=hardware_color,
        name="door_pull",
    )
    retrieval_door.inertial = Inertial.from_geometry(
        Box((door_w, door_t, door_h)),
        mass=4.5,
        origin=Origin(xyz=(door_w / 2.0, door_t / 2.0, 0.0)),
    )

    model.articulation(
        "cabinet_to_deposit_flap",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=deposit_flap,
        origin=Origin(xyz=(0.0, body_front_y, upper_hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=1.25,
        ),
    )
    model.articulation(
        "cabinet_to_retrieval_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=retrieval_door,
        origin=Origin(
            xyz=(
                -(inner_w / 2.0),
                body_front_y,
                floor_top_z + door_h / 2.0 + 0.003,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=0.0,
            upper=1.95,
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

    cabinet = object_model.get_part("cabinet")
    deposit_flap = object_model.get_part("deposit_flap")
    retrieval_door = object_model.get_part("retrieval_door")
    flap_joint = object_model.get_articulation("cabinet_to_deposit_flap")
    door_joint = object_model.get_articulation("cabinet_to_retrieval_door")

    flap_limits = flap_joint.motion_limits
    door_limits = door_joint.motion_limits
    ctx.check(
        "deposit flap uses a horizontal hinge axis",
        flap_joint.axis == (1.0, 0.0, 0.0)
        and flap_limits is not None
        and flap_limits.lower == 0.0
        and flap_limits.upper is not None
        and 1.0 <= flap_limits.upper <= 1.4,
        details=f"axis={flap_joint.axis}, limits={flap_limits}",
    )
    ctx.check(
        "retrieval door uses a vertical side hinge axis",
        door_joint.axis == (0.0, 0.0, 1.0)
        and door_limits is not None
        and door_limits.lower == 0.0
        and door_limits.upper is not None
        and 1.4 <= door_limits.upper <= 2.2,
        details=f"axis={door_joint.axis}, limits={door_limits}",
    )

    with ctx.pose({flap_joint: 0.0, door_joint: 0.0}):
        ctx.expect_gap(
            deposit_flap,
            cabinet,
            axis="y",
            max_gap=0.002,
            max_penetration=0.0,
            name="deposit flap closes against the cabinet front",
        )
        ctx.expect_overlap(
            deposit_flap,
            cabinet,
            axes="xz",
            min_overlap=0.08,
            name="deposit flap sits within the upper front face footprint",
        )
        ctx.expect_gap(
            retrieval_door,
            cabinet,
            axis="y",
            max_gap=0.002,
            max_penetration=0.0,
            name="retrieval door closes against the cabinet front",
        )
        ctx.expect_overlap(
            retrieval_door,
            cabinet,
            axes="xz",
            min_overlap=0.20,
            name="retrieval door covers the lower compartment opening",
        )

        flap_closed_aabb = ctx.part_world_aabb(deposit_flap)
        door_closed_aabb = ctx.part_world_aabb(retrieval_door)

    with ctx.pose({flap_joint: 1.05}):
        flap_open_aabb = ctx.part_world_aabb(deposit_flap)

    with ctx.pose({door_joint: 1.1}):
        door_open_aabb = ctx.part_world_aabb(retrieval_door)

    flap_opens_out = (
        flap_closed_aabb is not None
        and flap_open_aabb is not None
        and flap_open_aabb[1][1] > flap_closed_aabb[1][1] + 0.05
        and flap_open_aabb[0][2] > flap_closed_aabb[0][2] + 0.02
    )
    ctx.check(
        "deposit flap swings outward and upward",
        flap_opens_out,
        details=f"closed={flap_closed_aabb}, open={flap_open_aabb}",
    )

    door_opens_out = (
        door_closed_aabb is not None
        and door_open_aabb is not None
        and door_open_aabb[1][1] > door_closed_aabb[1][1] + 0.12
    )
    ctx.check(
        "retrieval door swings outward from the front face",
        door_opens_out,
        details=f"closed={door_closed_aabb}, open={door_open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
