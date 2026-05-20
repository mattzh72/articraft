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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


LEFT_BLOCK_X = -0.120
RIGHT_BLOCK_X = 0.140
SHAFT_AXIS_Z = 0.150

FRAME_LEFT_X = -0.280
FRAME_RIGHT_X = 0.215
FRAME_RAIL_Y = 0.086
FRAME_RAIL_WIDTH = 0.044
FRAME_RAIL_HEIGHT = 0.038
FRAME_PAD_SIZE = (0.118, 0.214, 0.018)
FRAME_PAD_TOP_Z = 0.088
FRAME_FOOT_XS = (-0.225, 0.160)
FRAME_FOOT_Y = 0.117

PILLOW_BASE_SIZE = (0.098, 0.108, 0.016)
PILLOW_AXIS_TO_BASE = 0.062
PILLOW_CHEEK_SIZE = (0.080, 0.018, 0.070)
PILLOW_CHEEK_Y = 0.036
PILLOW_CAP_SIZE = (0.092, 0.094, 0.018)
PILLOW_CAP_Z = 0.032
PILLOW_WEB_SIZE = (0.018, 0.072, 0.022)
PILLOW_WEB_XS = (-0.028, 0.028)
PILLOW_PAD_LENGTH = 0.082
PILLOW_PAD_WIDTH = 0.018
PILLOW_PAD_THICKNESS = 0.006

SHAFT_MAIN_RADIUS = 0.020
SHAFT_JOURNAL_RADIUS = 0.021
LEFT_HUB_RADIUS = 0.031
RIGHT_COLLAR_RADIUS = 0.029
FLANGE_HUB_RADIUS = 0.039
FLANGE_RADIUS = 0.105

SHAFT_X_MIN = -0.080
SHAFT_X_MAX = 0.410
LEFT_JOURNAL_X0 = -0.041
LEFT_JOURNAL_X1 = 0.041
RIGHT_JOURNAL_X0 = 0.219
RIGHT_JOURNAL_X1 = 0.301
LEFT_HUB_X0 = -0.089
LEFT_HUB_X1 = -0.053
RIGHT_COLLAR_X0 = 0.313
RIGHT_COLLAR_X1 = 0.325
FLANGE_HUB_X0 = 0.325
FLANGE_HUB_X1 = 0.410
FLANGE_DISK_X0 = 0.349
FLANGE_DISK_X1 = 0.373


def _x_cylinder_origin(x0: float, x1: float) -> Origin:
    return Origin(
        xyz=(((x0 + x1) / 2.0), 0.0, 0.0),
        rpy=(0.0, math.pi / 2.0, 0.0),
    )


def _add_frame_visuals(frame, *, material: str) -> None:
    rail_length = FRAME_RIGHT_X - FRAME_LEFT_X
    rail_center_x = (FRAME_LEFT_X + FRAME_RIGHT_X) / 2.0

    for name, y_pos in (("left_rail", FRAME_RAIL_Y), ("right_rail", -FRAME_RAIL_Y)):
        frame.visual(
            Box((rail_length, FRAME_RAIL_WIDTH, FRAME_RAIL_HEIGHT)),
            origin=Origin(xyz=(rail_center_x, y_pos, FRAME_RAIL_HEIGHT / 2.0)),
            material=material,
            name=name,
        )

    for name, x_pos in (
        ("crossmember_left", -0.225),
        ("crossmember_mid_left", -0.070),
        ("crossmember_mid_right", 0.060),
        ("crossmember_right", 0.165),
    ):
        frame.visual(
            Box((0.040, 0.182, 0.030)),
            origin=Origin(xyz=(x_pos, 0.0, 0.015)),
            material=material,
            name=name,
        )

    pad_center_z = FRAME_PAD_TOP_Z - (FRAME_PAD_SIZE[2] / 2.0)
    for name, x_pos in (("left_pad", LEFT_BLOCK_X), ("right_pad", RIGHT_BLOCK_X)):
        frame.visual(
            Box(FRAME_PAD_SIZE),
            origin=Origin(xyz=(x_pos, 0.0, pad_center_z)),
            material=material,
            name=name,
        )
        for idx, offset in enumerate((-0.030, 0.030), start=1):
            frame.visual(
                Box((0.014, 0.164, FRAME_PAD_TOP_Z - FRAME_PAD_SIZE[2] - FRAME_RAIL_HEIGHT)),
                origin=Origin(
                    xyz=(
                        x_pos + offset,
                        0.0,
                        FRAME_RAIL_HEIGHT + ((FRAME_PAD_TOP_Z - FRAME_PAD_SIZE[2] - FRAME_RAIL_HEIGHT) / 2.0),
                    )
                ),
                material=material,
                name=f"{name}_riser_{idx}",
            )

    for foot_index, x_pos in enumerate(FRAME_FOOT_XS, start=1):
        for side_name, y_sign in (("front", 1.0), ("rear", -1.0)):
            frame.visual(
                Box((0.090, 0.056, 0.012)),
                origin=Origin(xyz=(x_pos, y_sign * FRAME_FOOT_Y, 0.006)),
                material=material,
                name=f"foot_{foot_index}_{side_name}",
            )
            frame.visual(
                Box((0.055, 0.012, 0.026)),
                origin=Origin(xyz=(x_pos, y_sign * 0.1015, 0.025)),
                material=material,
                name=f"foot_web_{foot_index}_{side_name}",
            )


def _add_pillow_block_visuals(block, *, material: str) -> None:
    block.visual(
        Box(PILLOW_BASE_SIZE),
        origin=Origin(xyz=(0.0, 0.0, -PILLOW_AXIS_TO_BASE + (PILLOW_BASE_SIZE[2] / 2.0))),
        material=material,
        name="base",
    )

    for idx, y_pos in enumerate((-PILLOW_CHEEK_Y, PILLOW_CHEEK_Y), start=1):
        block.visual(
            Box(PILLOW_CHEEK_SIZE),
            origin=Origin(xyz=(0.0, y_pos, -0.011)),
            material=material,
            name=f"cheek_{idx}",
        )

    for idx, x_pos in enumerate(PILLOW_WEB_XS, start=1):
        block.visual(
            Box(PILLOW_WEB_SIZE),
            origin=Origin(xyz=(x_pos, 0.0, -0.036)),
            material=material,
            name=f"web_{idx}",
        )

    for idx, (x_pos, y_pos) in enumerate(
        ((-0.032, -0.033), (-0.032, 0.033), (0.032, -0.033), (0.032, 0.033)),
        start=1,
    ):
        block.visual(
            Box((0.030, 0.022, 0.028)),
            origin=Origin(xyz=(x_pos, y_pos, -0.040)),
            material=material,
            name=f"foot_bracket_{idx}",
        )

    block.visual(
        Box(PILLOW_CAP_SIZE),
        origin=Origin(xyz=(0.0, 0.0, PILLOW_CAP_Z)),
        material=material,
        name="cap",
    )

    for idx, (x_pos, y_pos) in enumerate(
        ((-0.028, -0.026), (-0.028, 0.026), (0.028, -0.026), (0.028, 0.026)),
        start=1,
    ):
        block.visual(
            Cylinder(radius=0.006, length=0.010),
            origin=Origin(xyz=(x_pos, y_pos, 0.031)),
            material=material,
            name=f"cap_bolt_{idx}",
        )

    block.visual(
        Box((PILLOW_PAD_LENGTH, PILLOW_PAD_WIDTH, PILLOW_PAD_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                -(SHAFT_JOURNAL_RADIUS + (PILLOW_PAD_THICKNESS / 2.0)),
            )
        ),
        material=material,
        name="lower_bearing_pad",
    )
    block.visual(
        Box((PILLOW_PAD_LENGTH, PILLOW_PAD_WIDTH, PILLOW_PAD_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                SHAFT_JOURNAL_RADIUS + (PILLOW_PAD_THICKNESS / 2.0),
            )
        ),
        material=material,
        name="upper_bearing_pad",
    )


def _add_shaft_visuals(shaft, *, steel: str, face_material: str) -> None:
    shaft.visual(
        Cylinder(radius=SHAFT_MAIN_RADIUS, length=SHAFT_X_MAX - SHAFT_X_MIN),
        origin=_x_cylinder_origin(SHAFT_X_MIN, SHAFT_X_MAX),
        material=steel,
        name="main_shaft",
    )
    shaft.visual(
        Cylinder(radius=SHAFT_JOURNAL_RADIUS, length=LEFT_JOURNAL_X1 - LEFT_JOURNAL_X0),
        origin=_x_cylinder_origin(LEFT_JOURNAL_X0, LEFT_JOURNAL_X1),
        material=steel,
        name="left_journal",
    )
    shaft.visual(
        Cylinder(radius=SHAFT_JOURNAL_RADIUS, length=RIGHT_JOURNAL_X1 - RIGHT_JOURNAL_X0),
        origin=_x_cylinder_origin(RIGHT_JOURNAL_X0, RIGHT_JOURNAL_X1),
        material=steel,
        name="right_journal",
    )
    shaft.visual(
        Cylinder(radius=LEFT_HUB_RADIUS, length=LEFT_HUB_X1 - LEFT_HUB_X0),
        origin=_x_cylinder_origin(LEFT_HUB_X0, LEFT_HUB_X1),
        material=steel,
        name="left_hub",
    )
    shaft.visual(
        Box((LEFT_HUB_X1 - LEFT_HUB_X0, 0.014, 0.010)),
        origin=Origin(
            xyz=(
                (LEFT_HUB_X0 + LEFT_HUB_X1) / 2.0,
                0.0,
                LEFT_HUB_RADIUS + 0.001,
            )
        ),
        material=steel,
        name="left_key",
    )
    shaft.visual(
        Cylinder(radius=RIGHT_COLLAR_RADIUS, length=RIGHT_COLLAR_X1 - RIGHT_COLLAR_X0),
        origin=_x_cylinder_origin(RIGHT_COLLAR_X0, RIGHT_COLLAR_X1),
        material=steel,
        name="right_collar",
    )
    shaft.visual(
        Cylinder(radius=FLANGE_HUB_RADIUS, length=FLANGE_HUB_X1 - FLANGE_HUB_X0),
        origin=_x_cylinder_origin(FLANGE_HUB_X0, FLANGE_HUB_X1),
        material=steel,
        name="flange_hub",
    )
    shaft.visual(
        Box((FLANGE_HUB_X1 - FLANGE_HUB_X0, 0.016, 0.012)),
        origin=Origin(
            xyz=(
                (FLANGE_HUB_X0 + FLANGE_HUB_X1) / 2.0,
                0.0,
                FLANGE_HUB_RADIUS + 0.001,
            )
        ),
        material=steel,
        name="flange_key",
    )
    shaft.visual(
        Cylinder(radius=FLANGE_RADIUS, length=FLANGE_DISK_X1 - FLANGE_DISK_X0),
        origin=_x_cylinder_origin(FLANGE_DISK_X0, FLANGE_DISK_X1),
        material=face_material,
        name="flange_disk",
    )
    shaft.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(
            xyz=((FLANGE_DISK_X0 + FLANGE_DISK_X1) / 2.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="flange_center_boss",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="horizontal_spin_fixture")

    model.material("frame_graphite", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("cast_housing", rgba=(0.56, 0.58, 0.60, 1.0))
    model.material("shaft_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("machined_face", rgba=(0.84, 0.85, 0.87, 1.0))

    frame = model.part("frame")
    _add_frame_visuals(frame, material="frame_graphite")

    left_pillow_block = model.part("left_pillow_block")
    _add_pillow_block_visuals(left_pillow_block, material="cast_housing")

    right_pillow_block = model.part("right_pillow_block")
    _add_pillow_block_visuals(right_pillow_block, material="cast_housing")

    shaft = model.part("shaft")
    _add_shaft_visuals(shaft, steel="shaft_steel", face_material="machined_face")

    model.articulation(
        "frame_to_left_pillow_block",
        ArticulationType.FIXED,
        parent=frame,
        child=left_pillow_block,
        origin=Origin(xyz=(LEFT_BLOCK_X, 0.0, SHAFT_AXIS_Z)),
    )
    model.articulation(
        "frame_to_right_pillow_block",
        ArticulationType.FIXED,
        parent=frame,
        child=right_pillow_block,
        origin=Origin(xyz=(RIGHT_BLOCK_X, 0.0, SHAFT_AXIS_Z)),
    )
    model.articulation(
        "frame_to_shaft",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=shaft,
        origin=Origin(xyz=(LEFT_BLOCK_X, 0.0, SHAFT_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    left_pillow_block = object_model.get_part("left_pillow_block")
    right_pillow_block = object_model.get_part("right_pillow_block")
    shaft = object_model.get_part("shaft")
    shaft_spin = object_model.get_articulation("frame_to_shaft")

    left_pad = frame.get_visual("left_pad")
    right_pad = frame.get_visual("right_pad")
    right_rail = frame.get_visual("right_rail")
    left_block_base = left_pillow_block.get_visual("base")
    right_block_base = right_pillow_block.get_visual("base")
    left_lower_pad = left_pillow_block.get_visual("lower_bearing_pad")
    left_upper_pad = left_pillow_block.get_visual("upper_bearing_pad")
    right_lower_pad = right_pillow_block.get_visual("lower_bearing_pad")
    right_upper_pad = right_pillow_block.get_visual("upper_bearing_pad")
    left_journal = shaft.get_visual("left_journal")
    right_journal = shaft.get_visual("right_journal")
    main_shaft = shaft.get_visual("main_shaft")
    flange_disk = shaft.get_visual("flange_disk")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "expected part set present",
        {part.name for part in object_model.parts}
        == {"frame", "left_pillow_block", "right_pillow_block", "shaft"},
        details=str([part.name for part in object_model.parts]),
    )
    ctx.check(
        "shaft articulation is continuous about x",
        (
            shaft_spin.articulation_type == ArticulationType.CONTINUOUS
            and tuple(shaft_spin.axis) == (1.0, 0.0, 0.0)
            and shaft_spin.motion_limits is not None
            and shaft_spin.motion_limits.lower is None
            and shaft_spin.motion_limits.upper is None
        ),
        details=(
            f"type={shaft_spin.articulation_type}, axis={shaft_spin.axis}, "
            f"limits={shaft_spin.motion_limits}"
        ),
    )
    ctx.expect_origin_gap(
        right_pillow_block,
        left_pillow_block,
        axis="x",
        min_gap=0.24,
        max_gap=0.28,
        name="pillow block span is practical",
    )
    ctx.expect_contact(
        left_pillow_block,
        frame,
        elem_a=left_block_base,
        elem_b=left_pad,
        name="left pillow block mounted on frame pad",
    )
    ctx.expect_contact(
        right_pillow_block,
        frame,
        elem_a=right_block_base,
        elem_b=right_pad,
        name="right pillow block mounted on frame pad",
    )
    ctx.expect_contact(
        shaft,
        left_pillow_block,
        elem_a=left_journal,
        elem_b=left_lower_pad,
        name="shaft grounded by left lower pad",
    )
    ctx.expect_contact(
        shaft,
        left_pillow_block,
        elem_a=left_journal,
        elem_b=left_upper_pad,
        name="shaft captured by left cap pad",
    )
    ctx.expect_contact(
        shaft,
        right_pillow_block,
        elem_a=right_journal,
        elem_b=right_lower_pad,
        name="shaft grounded by right lower pad",
    )
    ctx.expect_contact(
        shaft,
        right_pillow_block,
        elem_a=right_journal,
        elem_b=right_upper_pad,
        name="shaft captured by right cap pad",
    )
    ctx.expect_gap(
        shaft,
        frame,
        axis="z",
        min_gap=0.040,
        positive_elem=main_shaft,
        negative_elem=left_pad,
        name="shaft clears support pads vertically",
    )
    ctx.expect_gap(
        shaft,
        right_pillow_block,
        axis="x",
        min_gap=0.020,
        max_gap=0.040,
        positive_elem=flange_disk,
        negative_elem=right_block_base,
        name="flange clears outboard support",
    )
    ctx.expect_gap(
        shaft,
        frame,
        axis="x",
        min_gap=0.012,
        positive_elem=flange_disk,
        negative_elem=right_rail,
        name="flange projects beyond frame face",
    )

    with ctx.pose({shaft_spin: math.pi / 2.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="rotated shaft pose stays clear")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
