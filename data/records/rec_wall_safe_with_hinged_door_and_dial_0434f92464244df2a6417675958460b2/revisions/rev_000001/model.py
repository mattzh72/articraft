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
    model = ArticulatedObject(name="wall_safe")

    cabinet_paint = model.material("cabinet_paint", rgba=(0.18, 0.20, 0.22, 1.0))
    door_paint = model.material("door_paint", rgba=(0.23, 0.24, 0.26, 1.0))
    inner_paint = model.material("inner_paint", rgba=(0.11, 0.11, 0.12, 1.0))
    trim_metal = model.material("trim_metal", rgba=(0.58, 0.59, 0.60, 1.0))
    brass = model.material("brass", rgba=(0.62, 0.54, 0.35, 1.0))

    body_w = 0.40
    body_h = 0.56
    body_d = 0.20
    wall_t = 0.016
    back_t = 0.006
    flange_w = 0.50
    flange_h = 0.66
    flange_t = 0.012
    opening_w = body_w - 2.0 * wall_t
    opening_h = body_h - 2.0 * wall_t

    door_w = 0.360
    door_h = 0.520
    door_t = 0.045
    hinge_axis_x = opening_w / 2.0 + 0.002
    door_back_y = body_d / 2.0 + 0.002
    hinge_offset = 0.006

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((body_w, back_t, body_h)),
        origin=Origin(xyz=(0.0, -body_d / 2.0 + back_t / 2.0, 0.0)),
        material=inner_paint,
        name="body_back",
    )
    cabinet.visual(
        Box((wall_t, body_d - back_t, body_h)),
        origin=Origin(xyz=(body_w / 2.0 - wall_t / 2.0, back_t / 2.0, 0.0)),
        material=cabinet_paint,
        name="body_right_wall",
    )
    cabinet.visual(
        Box((wall_t, body_d - back_t, body_h)),
        origin=Origin(xyz=(-body_w / 2.0 + wall_t / 2.0, back_t / 2.0, 0.0)),
        material=cabinet_paint,
        name="body_left_wall",
    )
    cabinet.visual(
        Box((opening_w, body_d - back_t, wall_t)),
        origin=Origin(xyz=(0.0, back_t / 2.0, body_h / 2.0 - wall_t / 2.0)),
        material=cabinet_paint,
        name="body_top_wall",
    )
    cabinet.visual(
        Box((opening_w, body_d - back_t, wall_t)),
        origin=Origin(xyz=(0.0, back_t / 2.0, -body_h / 2.0 + wall_t / 2.0)),
        material=cabinet_paint,
        name="body_bottom_wall",
    )
    cabinet.visual(
        Box(((flange_w - opening_w) / 2.0, flange_t, flange_h)),
        origin=Origin(
            xyz=((opening_w + flange_w) / 4.0, body_d / 2.0 - flange_t / 2.0, 0.0)
        ),
        material=door_paint,
        name="frame_right_strip",
    )
    cabinet.visual(
        Box(((flange_w - opening_w) / 2.0, flange_t, flange_h)),
        origin=Origin(
            xyz=(-(opening_w + flange_w) / 4.0, body_d / 2.0 - flange_t / 2.0, 0.0)
        ),
        material=door_paint,
        name="frame_left_strip",
    )
    cabinet.visual(
        Box((flange_w, flange_t, (flange_h - opening_h) / 2.0)),
        origin=Origin(
            xyz=(0.0, body_d / 2.0 - flange_t / 2.0, (opening_h + flange_h) / 4.0)
        ),
        material=door_paint,
        name="frame_top_strip",
    )
    cabinet.visual(
        Box((flange_w, flange_t, (flange_h - opening_h) / 2.0)),
        origin=Origin(
            xyz=(0.0, body_d / 2.0 - flange_t / 2.0, -(opening_h + flange_h) / 4.0)
        ),
        material=door_paint,
        name="frame_bottom_strip",
    )
    for idx, z_center in enumerate((0.20, 0.0, -0.20), start=1):
        cabinet.visual(
            Cylinder(radius=0.005, length=0.090),
            origin=Origin(
                xyz=(hinge_axis_x + 0.008, body_d / 2.0 - 0.0055, z_center),
            ),
            material=trim_metal,
            name=f"hinge_knuckle_{idx}",
        )

    door = model.part("door")
    door.visual(
        Box((door_w, door_t, door_h)),
        origin=Origin(xyz=(-(hinge_offset + door_w / 2.0), door_t / 2.0, 0.0)),
        material=door_paint,
        name="door_panel",
    )
    door.visual(
        Box((door_w - 0.060, 0.012, door_h - 0.080)),
        origin=Origin(
            xyz=(-(hinge_offset + door_w / 2.0), door_t - 0.006, 0.0),
        ),
        material=cabinet_paint,
        name="door_face_plate",
    )
    door.visual(
        Box((0.004, 0.010, 0.140)),
        origin=Origin(xyz=(-0.004, -0.005, 0.165)),
        material=trim_metal,
        name="door_hinge_leaf_upper",
    )
    door.visual(
        Box((0.004, 0.010, 0.140)),
        origin=Origin(xyz=(-0.004, -0.005, -0.165)),
        material=trim_metal,
        name="door_hinge_leaf_lower",
    )
    door.visual(
        Cylinder(radius=0.008, length=0.030),
        origin=Origin(
            xyz=(-0.140, door_t + 0.015, -0.110),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=trim_metal,
        name="handle_post_lower",
    )
    door.visual(
        Cylinder(radius=0.008, length=0.030),
        origin=Origin(
            xyz=(-0.140, door_t + 0.015, -0.030),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=trim_metal,
        name="handle_post_upper",
    )
    door.visual(
        Cylinder(radius=0.010, length=0.120),
        origin=Origin(
            xyz=(-0.140, door_t + 0.040, -0.070),
        ),
        material=trim_metal,
        name="handle_grip",
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.036, length=0.016),
        origin=Origin(xyz=(0.0, 0.008, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="dial_back",
    )
    dial.visual(
        Cylinder(radius=0.029, length=0.012),
        origin=Origin(xyz=(0.0, 0.020, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim_metal,
        name="dial_ring",
    )
    dial.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(0.0, 0.031, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim_metal,
        name="dial_cap",
    )

    deposit_flap = model.part("deposit_flap")
    deposit_flap.visual(
        Box((0.145, 0.010, 0.080)),
        origin=Origin(xyz=(0.0, 0.005, -0.040)),
        material=door_paint,
        name="flap_panel",
    )
    deposit_flap.visual(
        Box((0.100, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.014, -0.075)),
        material=trim_metal,
        name="flap_pull_lip",
    )

    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(hinge_axis_x, door_back_y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.6,
            lower=0.0,
            upper=1.9,
        ),
    )
    model.articulation(
        "door_to_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(-0.245, door_t, 0.165)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
    )
    model.articulation(
        "door_to_deposit_flap",
        ArticulationType.REVOLUTE,
        parent=door,
        child=deposit_flap,
        origin=Origin(xyz=(-0.155, door_t, 0.080)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=1.2,
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
    door = object_model.get_part("door")
    dial = object_model.get_part("dial")
    deposit_flap = object_model.get_part("deposit_flap")

    main_hinge = object_model.get_articulation("cabinet_to_door")
    dial_spin = object_model.get_articulation("door_to_dial")
    flap_hinge = object_model.get_articulation("door_to_deposit_flap")

    ctx.check(
        "main door uses a vertical right-side hinge",
        main_hinge.articulation_type == ArticulationType.REVOLUTE
        and main_hinge.axis == (0.0, 0.0, -1.0)
        and main_hinge.motion_limits is not None
        and main_hinge.motion_limits.lower == 0.0
        and main_hinge.motion_limits.upper is not None
        and main_hinge.motion_limits.upper >= 1.8,
        details=(
            f"type={main_hinge.articulation_type}, axis={main_hinge.axis}, "
            f"limits={main_hinge.motion_limits}"
        ),
    )
    ctx.check(
        "combination dial spins continuously about its central axis",
        dial_spin.articulation_type == ArticulationType.CONTINUOUS
        and dial_spin.axis == (0.0, 1.0, 0.0)
        and dial_spin.motion_limits is not None
        and dial_spin.motion_limits.lower is None
        and dial_spin.motion_limits.upper is None,
        details=(
            f"type={dial_spin.articulation_type}, axis={dial_spin.axis}, "
            f"limits={dial_spin.motion_limits}"
        ),
    )
    ctx.check(
        "deposit flap uses a top horizontal hinge",
        flap_hinge.articulation_type == ArticulationType.REVOLUTE
        and flap_hinge.axis == (1.0, 0.0, 0.0)
        and flap_hinge.motion_limits is not None
        and flap_hinge.motion_limits.lower == 0.0
        and flap_hinge.motion_limits.upper is not None
        and flap_hinge.motion_limits.upper >= 1.0,
        details=(
            f"type={flap_hinge.articulation_type}, axis={flap_hinge.axis}, "
            f"limits={flap_hinge.motion_limits}"
        ),
    )

    ctx.expect_contact(
        dial,
        door,
        elem_a="dial_back",
        elem_b="door_face_plate",
        name="dial mounts flush to the door face",
    )
    ctx.expect_contact(
        deposit_flap,
        door,
        elem_a="flap_panel",
        elem_b="door_face_plate",
        name="deposit flap seats against the door face",
    )
    ctx.expect_overlap(
        door,
        cabinet,
        axes="xz",
        elem_a="door_panel",
        min_overlap=0.30,
        name="main door covers the front opening footprint",
    )

    closed_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({main_hinge: 1.2}):
        ctx.expect_gap(
            door,
            cabinet,
            axis="y",
            positive_elem="door_panel",
            min_gap=0.004,
            name="opened door clears the cabinet face",
        )
        open_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "main door swings outward from the safe face",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][1] > closed_door_aabb[1][1] + 0.12,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    closed_flap_aabb = ctx.part_world_aabb(deposit_flap)
    with ctx.pose({flap_hinge: 0.9}):
        open_flap_aabb = ctx.part_world_aabb(deposit_flap)
    ctx.check(
        "deposit flap opens outward from its top edge",
        closed_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[1][1] > closed_flap_aabb[1][1] + 0.04,
        details=f"closed={closed_flap_aabb}, open={open_flap_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
