from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


CABINET_WIDTH = 0.245
CABINET_DEPTH = 0.305
CABINET_HEIGHT = 0.920
SIDE_WALL_THICKNESS = 0.012
TOP_THICKNESS = 0.016
BOTTOM_THICKNESS = 0.024
BACK_THICKNESS = 0.008
FRONT_STILE_WIDTH = 0.018
FRONT_FRAME_DEPTH = 0.020

TRAY_COUNT = 12
TRAY_PITCH = 0.070
TRAY_FLOOR_Z0 = BOTTOM_THICKNESS + 0.010
TRAY_JOINT_Y = 0.017
TRAY_BODY_WIDTH = 0.198
TRAY_FRONT_WIDTH = 0.203
TRAY_DEPTH = 0.276
TRAY_BOTTOM_THICKNESS = 0.0045
TRAY_SIDE_THICKNESS = 0.006
TRAY_BACK_THICKNESS = 0.006
TRAY_FRONT_THICKNESS = 0.008
TRAY_SIDE_HEIGHT = 0.026
TRAY_BACK_HEIGHT = 0.028
TRAY_FRONT_HEIGHT = 0.050
TRAY_SLIDE_TRAVEL = 0.160

CHANNEL_WIDTH = 0.021
CHANNEL_LENGTH = 0.255
CHANNEL_START_Y = 0.014
CHANNEL_EMBED = 0.002
CHANNEL_WOOD_THICKNESS = 0.006
CHANNEL_FELT_THICKNESS = 0.0015

PAD_THICKNESS = 0.010
PAD_SIDE_MARGIN = 0.014
PAD_FRONT_MARGIN = 0.028
PAD_BACK_MARGIN = 0.020


def _tray_name(index: int) -> str:
    return f"tray_{index:02d}"


def _joint_name(index: int) -> str:
    return f"cabinet_to_tray_{index:02d}"


def _slot_floor_z(index: int) -> float:
    return TRAY_FLOOR_Z0 + (index - 1) * TRAY_PITCH


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="watchmakers_watch_storage_cabinet")

    walnut = model.material("walnut", rgba=(0.29, 0.19, 0.12, 1.0))
    walnut_dark = model.material("walnut_dark", rgba=(0.18, 0.11, 0.07, 1.0))
    brass = model.material("aged_brass", rgba=(0.71, 0.58, 0.28, 1.0))
    felt = model.material("olive_felt", rgba=(0.22, 0.30, 0.17, 1.0))
    felt_dark = model.material("felt_shadow", rgba=(0.18, 0.24, 0.13, 1.0))

    cabinet = model.part("cabinet_body")

    cabinet.visual(
        Box((CABINET_WIDTH, CABINET_DEPTH, BOTTOM_THICKNESS)),
        origin=Origin(xyz=(0.0, CABINET_DEPTH * 0.5, BOTTOM_THICKNESS * 0.5)),
        material=walnut_dark,
        name="cabinet_base",
    )
    cabinet.visual(
        Box((CABINET_WIDTH, CABINET_DEPTH, TOP_THICKNESS)),
        origin=Origin(
            xyz=(0.0, CABINET_DEPTH * 0.5, CABINET_HEIGHT - TOP_THICKNESS * 0.5)
        ),
        material=walnut_dark,
        name="cabinet_top",
    )
    cabinet.visual(
        Box((SIDE_WALL_THICKNESS, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(
                -CABINET_WIDTH * 0.5 + SIDE_WALL_THICKNESS * 0.5,
                CABINET_DEPTH * 0.5,
                CABINET_HEIGHT * 0.5,
            )
        ),
        material=walnut,
        name="left_wall",
    )
    cabinet.visual(
        Box((SIDE_WALL_THICKNESS, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(
                CABINET_WIDTH * 0.5 - SIDE_WALL_THICKNESS * 0.5,
                CABINET_DEPTH * 0.5,
                CABINET_HEIGHT * 0.5,
            )
        ),
        material=walnut,
        name="right_wall",
    )
    cabinet.visual(
        Box((CABINET_WIDTH - 2.0 * SIDE_WALL_THICKNESS + 0.004, BACK_THICKNESS, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(0.0, BACK_THICKNESS * 0.5, CABINET_HEIGHT * 0.5)
        ),
        material=walnut_dark,
        name="back_panel",
    )
    cabinet.visual(
        Box((FRONT_STILE_WIDTH, FRONT_FRAME_DEPTH, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(
                -CABINET_WIDTH * 0.5 + FRONT_STILE_WIDTH * 0.5,
                CABINET_DEPTH - FRONT_FRAME_DEPTH * 0.5,
                CABINET_HEIGHT * 0.5,
            )
        ),
        material=walnut_dark,
        name="left_stile",
    )
    cabinet.visual(
        Box((FRONT_STILE_WIDTH, FRONT_FRAME_DEPTH, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(
                CABINET_WIDTH * 0.5 - FRONT_STILE_WIDTH * 0.5,
                CABINET_DEPTH - FRONT_FRAME_DEPTH * 0.5,
                CABINET_HEIGHT * 0.5,
            )
        ),
        material=walnut_dark,
        name="right_stile",
    )
    cabinet.visual(
        Box((CABINET_WIDTH - 2.0 * FRONT_STILE_WIDTH, FRONT_FRAME_DEPTH, 0.028)),
        origin=Origin(
            xyz=(
                0.0,
                CABINET_DEPTH - FRONT_FRAME_DEPTH * 0.5,
                CABINET_HEIGHT - TOP_THICKNESS - 0.004,
            )
        ),
        material=walnut_dark,
        name="top_lip",
    )

    cavity_inner_face_x = CABINET_WIDTH * 0.5 - SIDE_WALL_THICKNESS
    channel_center_x = cavity_inner_face_x - CHANNEL_WIDTH * 0.5 + CHANNEL_EMBED
    channel_center_y = CHANNEL_START_Y + CHANNEL_LENGTH * 0.5

    for index in range(1, TRAY_COUNT + 1):
        floor_z = _slot_floor_z(index)
        wood_center_z = (
            floor_z - CHANNEL_FELT_THICKNESS - CHANNEL_WOOD_THICKNESS * 0.5
        )
        felt_center_z = floor_z - CHANNEL_FELT_THICKNESS * 0.5
        for side_name, sign in (("left", -1.0), ("right", 1.0)):
            cabinet.visual(
                Box((CHANNEL_WIDTH, CHANNEL_LENGTH, CHANNEL_WOOD_THICKNESS)),
                origin=Origin(
                    xyz=(sign * channel_center_x, channel_center_y, wood_center_z)
                ),
                material=walnut_dark,
                name=f"channel_{side_name}_{index:02d}_wood",
            )
            cabinet.visual(
                Box((CHANNEL_WIDTH - 0.002, CHANNEL_LENGTH, CHANNEL_FELT_THICKNESS)),
                origin=Origin(
                    xyz=(sign * channel_center_x, channel_center_y, felt_center_z)
                ),
                material=felt_dark,
                name=f"channel_{side_name}_{index:02d}_felt",
            )

    cabinet.inertial = Inertial.from_geometry(
        Box((CABINET_WIDTH, CABINET_DEPTH, CABINET_HEIGHT)),
        mass=18.0,
        origin=Origin(
            xyz=(0.0, CABINET_DEPTH * 0.5, CABINET_HEIGHT * 0.5)
        ),
    )

    pad_width = TRAY_BODY_WIDTH - 2.0 * (TRAY_SIDE_THICKNESS + PAD_SIDE_MARGIN)
    pad_depth = TRAY_DEPTH - PAD_FRONT_MARGIN - PAD_BACK_MARGIN
    pad_center_y = PAD_BACK_MARGIN + pad_depth * 0.5

    for index in range(1, TRAY_COUNT + 1):
        tray = model.part(_tray_name(index))

        tray.visual(
            Box((TRAY_BODY_WIDTH, TRAY_DEPTH, TRAY_BOTTOM_THICKNESS)),
            origin=Origin(
                xyz=(0.0, TRAY_DEPTH * 0.5, TRAY_BOTTOM_THICKNESS * 0.5)
            ),
            material=walnut,
            name="tray_floor",
        )
        tray.visual(
            Box((TRAY_SIDE_THICKNESS, TRAY_DEPTH, TRAY_SIDE_HEIGHT)),
            origin=Origin(
                xyz=(
                    -TRAY_BODY_WIDTH * 0.5 + TRAY_SIDE_THICKNESS * 0.5,
                    TRAY_DEPTH * 0.5,
                    TRAY_SIDE_HEIGHT * 0.5,
                )
            ),
            material=walnut,
            name="left_side",
        )
        tray.visual(
            Box((TRAY_SIDE_THICKNESS, TRAY_DEPTH, TRAY_SIDE_HEIGHT)),
            origin=Origin(
                xyz=(
                    TRAY_BODY_WIDTH * 0.5 - TRAY_SIDE_THICKNESS * 0.5,
                    TRAY_DEPTH * 0.5,
                    TRAY_SIDE_HEIGHT * 0.5,
                )
            ),
            material=walnut,
            name="right_side",
        )
        tray.visual(
            Box((TRAY_BODY_WIDTH - 2.0 * TRAY_SIDE_THICKNESS, TRAY_BACK_THICKNESS, TRAY_BACK_HEIGHT)),
            origin=Origin(
                xyz=(
                    0.0,
                    TRAY_BACK_THICKNESS * 0.5,
                    TRAY_BACK_HEIGHT * 0.5,
                )
            ),
            material=walnut,
            name="back_wall",
        )
        tray.visual(
            Box((TRAY_FRONT_WIDTH, TRAY_FRONT_THICKNESS, TRAY_FRONT_HEIGHT)),
            origin=Origin(
                xyz=(
                    0.0,
                    TRAY_DEPTH - TRAY_FRONT_THICKNESS * 0.5,
                    TRAY_FRONT_HEIGHT * 0.5,
                )
            ),
            material=walnut_dark,
            name="front_face",
        )
        tray.visual(
            Box((pad_width, pad_depth, PAD_THICKNESS)),
            origin=Origin(
                xyz=(
                    0.0,
                    pad_center_y,
                    TRAY_BOTTOM_THICKNESS + PAD_THICKNESS * 0.5 - 0.0003,
                )
            ),
            material=felt,
            name="felt_pad",
        )
        tray.visual(
            Cylinder(radius=0.0025, length=0.008),
            origin=Origin(
                xyz=(0.0, TRAY_DEPTH + 0.0005, 0.021),
                rpy=(1.5707963267948966, 0.0, 0.0),
            ),
            material=brass,
            name="pull_stem",
        )
        tray.visual(
            Sphere(radius=0.0065),
            origin=Origin(xyz=(0.0, TRAY_DEPTH + 0.0035, 0.021)),
            material=brass,
            name="pull_knob",
        )

        tray.inertial = Inertial.from_geometry(
            Box((TRAY_FRONT_WIDTH, TRAY_DEPTH + 0.014, TRAY_FRONT_HEIGHT)),
            mass=0.42,
            origin=Origin(
                xyz=(0.0, (TRAY_DEPTH + 0.014) * 0.5, TRAY_FRONT_HEIGHT * 0.5)
            ),
        )

        model.articulation(
            _joint_name(index),
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=tray,
            origin=Origin(xyz=(0.0, TRAY_JOINT_Y, _slot_floor_z(index))),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=20.0,
                velocity=0.18,
                lower=0.0,
                upper=TRAY_SLIDE_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    part_names = {part.name for part in object_model.parts}
    joint_names = {joint.name for joint in object_model.articulations}

    expected_trays = {_tray_name(index) for index in range(1, TRAY_COUNT + 1)}
    expected_joints = {_joint_name(index) for index in range(1, TRAY_COUNT + 1)}

    ctx.check(
        "all twelve tray parts are present",
        expected_trays.issubset(part_names),
        details=f"present={sorted(part_names)}",
    )
    ctx.check(
        "all twelve tray slides are present",
        expected_joints.issubset(joint_names),
        details=f"present={sorted(joint_names)}",
    )

    slide_ok = True
    slide_details: list[str] = []
    for index in range(1, TRAY_COUNT + 1):
        slide = object_model.get_articulation(_joint_name(index))
        limits = slide.motion_limits
        ok = (
            slide.articulation_type == ArticulationType.PRISMATIC
            and slide.axis == (0.0, 1.0, 0.0)
            and limits is not None
            and limits.lower == 0.0
            and limits.upper == TRAY_SLIDE_TRAVEL
        )
        slide_ok = slide_ok and ok
        if not ok:
            slide_details.append(
                f"{slide.name}: type={slide.articulation_type}, axis={slide.axis}, limits={limits}"
            )
    ctx.check(
        "tray slides are prismatic forward runners",
        slide_ok,
        details="; ".join(slide_details),
    )

    cabinet = object_model.get_part("cabinet_body")
    bottom_tray = object_model.get_part(_tray_name(1))
    middle_tray = object_model.get_part(_tray_name(6))
    top_tray = object_model.get_part(_tray_name(12))
    middle_slide = object_model.get_articulation(_joint_name(6))

    with ctx.pose({middle_slide: 0.0}):
        ctx.expect_contact(
            middle_tray,
            cabinet,
            elem_b="channel_left_06_felt",
            contact_tol=0.0008,
            name="middle tray rests on the left felt-lined guide",
        )
        ctx.expect_contact(
            middle_tray,
            cabinet,
            elem_b="channel_right_06_felt",
            contact_tol=0.0008,
            name="middle tray rests on the right felt-lined guide",
        )
        ctx.expect_within(
            bottom_tray,
            cabinet,
            axes="xz",
            margin=0.015,
            name="bottom tray stays centered within the cabinet column",
        )
        ctx.expect_within(
            top_tray,
            cabinet,
            axes="xz",
            margin=0.015,
            name="top tray stays centered within the cabinet column",
        )

    closed_pos = ctx.part_world_position(middle_tray)
    with ctx.pose({middle_slide: TRAY_SLIDE_TRAVEL}):
        ctx.expect_within(
            middle_tray,
            cabinet,
            axes="xz",
            margin=0.015,
            name="middle tray stays laterally aligned when extended",
        )
        ctx.expect_overlap(
            middle_tray,
            cabinet,
            axes="y",
            min_overlap=0.120,
            name="middle tray retains insertion at full extension",
        )
        open_pos = ctx.part_world_position(middle_tray)

    ctx.check(
        "middle tray opens forward",
        closed_pos is not None
        and open_pos is not None
        and open_pos[1] > closed_pos[1] + 0.12,
        details=f"closed={closed_pos}, open={open_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
