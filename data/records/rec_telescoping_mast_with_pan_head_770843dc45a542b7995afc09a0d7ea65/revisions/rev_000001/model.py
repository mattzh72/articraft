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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BASE_SPAN = 0.72
BASE_ARM_WIDTH = 0.10
BASE_ARM_THICK = 0.025
BASE_BLOCK_SIZE = 0.20
BASE_BLOCK_THICK = 0.030
SOCKET_SIZE = 0.110
SOCKET_HEIGHT = 0.085
MAST_SOCKET_FUSE = 0.001

OUTER_SIZE = 0.074
OUTER_WALL = 0.004
OUTER_HEIGHT = 0.95

MIDDLE_SIZE = 0.060
MIDDLE_WALL = 0.003
MIDDLE_LENGTH = 0.95
MIDDLE_REST_INSERT = 0.44
MIDDLE_TRAVEL = 0.26

TOP_SIZE = 0.048
TOP_WALL = 0.003
TOP_LENGTH = 0.80
TOP_REST_INSERT = 0.34
TOP_TRAVEL = 0.22
TOP_CAP_SIZE = 0.056
TOP_CAP_THICK = 0.010

GUIDE_PROTRUSION = 0.003
GUIDE_EMBED = 0.0005
MIDDLE_GUIDE_WIDTH = 0.018
MIDDLE_GUIDE_LENGTH = 0.120
TOP_GUIDE_WIDTH = 0.016
TOP_GUIDE_LENGTH = 0.090

HEAD_BASE_RADIUS = 0.050
HEAD_BASE_HEIGHT = 0.018
HEAD_NECK_RADIUS = 0.021
HEAD_NECK_HEIGHT = 0.028
HEAD_FACE_SIZE = 0.100
HEAD_FACE_THICK = 0.012


def _centered_box_origin(
    size: tuple[float, float, float],
    *,
    xyz: tuple[float, float, float],
) -> Origin:
    return Origin(xyz=(xyz[0], xyz[1], xyz[2] + 0.5 * size[2]))


def _add_visuals(part, visuals, *, material: str) -> None:
    for geometry, origin, name in visuals:
        part.visual(geometry, origin=origin, material=material, name=name)


def _square_tube_visuals(
    prefix: str,
    *,
    outer_size: float,
    wall: float,
    length: float,
    z0: float,
) -> tuple[tuple[Box, Origin, str], ...]:
    wall_center = 0.5 * (outer_size - wall)
    return (
        (
            Box((wall, outer_size, length)),
            _centered_box_origin((wall, outer_size, length), xyz=(wall_center, 0.0, z0)),
            f"{prefix}_wall_pos_x",
        ),
        (
            Box((wall, outer_size, length)),
            _centered_box_origin((wall, outer_size, length), xyz=(-wall_center, 0.0, z0)),
            f"{prefix}_wall_neg_x",
        ),
        (
            Box((outer_size, wall, length)),
            _centered_box_origin((outer_size, wall, length), xyz=(0.0, wall_center, z0)),
            f"{prefix}_wall_pos_y",
        ),
        (
            Box((outer_size, wall, length)),
            _centered_box_origin((outer_size, wall, length), xyz=(0.0, -wall_center, z0)),
            f"{prefix}_wall_neg_y",
        ),
    )


def _guide_visuals(
    prefix: str,
    *,
    tube_size: float,
    guide_width: float,
    guide_length: float,
    z0: float,
) -> tuple[tuple[Box, Origin, str], ...]:
    guide_thickness = GUIDE_PROTRUSION + GUIDE_EMBED
    guide_center = 0.5 * tube_size + 0.5 * guide_thickness - GUIDE_EMBED
    return (
        (
            Box((guide_thickness, guide_width, guide_length)),
            _centered_box_origin(
                (guide_thickness, guide_width, guide_length),
                xyz=(guide_center, 0.0, z0),
            ),
            f"{prefix}_guide_pos_x",
        ),
        (
            Box((guide_thickness, guide_width, guide_length)),
            _centered_box_origin(
                (guide_thickness, guide_width, guide_length),
                xyz=(-guide_center, 0.0, z0),
            ),
            f"{prefix}_guide_neg_x",
        ),
        (
            Box((guide_width, guide_thickness, guide_length)),
            _centered_box_origin(
                (guide_width, guide_thickness, guide_length),
                xyz=(0.0, guide_center, z0),
            ),
            f"{prefix}_guide_pos_y",
        ),
        (
            Box((guide_width, guide_thickness, guide_length)),
            _centered_box_origin(
                (guide_width, guide_thickness, guide_length),
                xyz=(0.0, -guide_center, z0),
            ),
            f"{prefix}_guide_neg_y",
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="survey_mast_rotary_top_plate")

    model.material("powder_coat_dark", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("anodized_aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("brushed_alloy", rgba=(0.84, 0.85, 0.87, 1.0))
    model.material("machined_head", rgba=(0.63, 0.66, 0.70, 1.0))

    outer_mast = model.part("outer_mast")
    socket_wall = 0.5 * (SOCKET_SIZE - (OUTER_SIZE - 2.0 * MAST_SOCKET_FUSE))
    outer_mast_visuals = [
        (
            Box((0.140, 0.080, 0.010)),
            _centered_box_origin((0.140, 0.080, 0.010), xyz=(BASE_SPAN * 0.5 - 0.070, 0.0, 0.0)),
            "foot_pad_pos_x",
        ),
        (
            Box((0.140, 0.080, 0.010)),
            _centered_box_origin((0.140, 0.080, 0.010), xyz=(-(BASE_SPAN * 0.5 - 0.070), 0.0, 0.0)),
            "foot_pad_neg_x",
        ),
        (
            Box((0.080, 0.140, 0.010)),
            _centered_box_origin((0.080, 0.140, 0.010), xyz=(0.0, BASE_SPAN * 0.5 - 0.070, 0.0)),
            "foot_pad_pos_y",
        ),
        (
            Box((0.080, 0.140, 0.010)),
            _centered_box_origin((0.080, 0.140, 0.010), xyz=(0.0, -(BASE_SPAN * 0.5 - 0.070), 0.0)),
            "foot_pad_neg_y",
        ),
        (
            Box((BASE_SPAN, BASE_ARM_WIDTH, BASE_ARM_THICK)),
            _centered_box_origin((BASE_SPAN, BASE_ARM_WIDTH, BASE_ARM_THICK), xyz=(0.0, 0.0, 0.0)),
            "base_cross_x",
        ),
        (
            Box((BASE_ARM_WIDTH, BASE_SPAN, BASE_ARM_THICK)),
            _centered_box_origin((BASE_ARM_WIDTH, BASE_SPAN, BASE_ARM_THICK), xyz=(0.0, 0.0, 0.0)),
            "base_cross_y",
        ),
        (
            Box((BASE_BLOCK_SIZE, BASE_BLOCK_SIZE, BASE_BLOCK_THICK)),
            _centered_box_origin(
                (BASE_BLOCK_SIZE, BASE_BLOCK_SIZE, BASE_BLOCK_THICK),
                xyz=(0.0, 0.0, BASE_ARM_THICK),
            ),
            "base_block",
        ),
        (
            Box((0.020, SOCKET_SIZE, 0.060)),
            _centered_box_origin((0.020, SOCKET_SIZE, 0.060), xyz=(0.055, 0.0, BASE_ARM_THICK + BASE_BLOCK_THICK)),
            "socket_rib_pos_x",
        ),
        (
            Box((0.020, SOCKET_SIZE, 0.060)),
            _centered_box_origin((0.020, SOCKET_SIZE, 0.060), xyz=(-0.055, 0.0, BASE_ARM_THICK + BASE_BLOCK_THICK)),
            "socket_rib_neg_x",
        ),
        (
            Box((SOCKET_SIZE, 0.020, 0.060)),
            _centered_box_origin((SOCKET_SIZE, 0.020, 0.060), xyz=(0.0, 0.055, BASE_ARM_THICK + BASE_BLOCK_THICK)),
            "socket_rib_pos_y",
        ),
        (
            Box((SOCKET_SIZE, 0.020, 0.060)),
            _centered_box_origin((SOCKET_SIZE, 0.020, 0.060), xyz=(0.0, -0.055, BASE_ARM_THICK + BASE_BLOCK_THICK)),
            "socket_rib_neg_y",
        ),
    ]
    outer_mast_visuals.extend(
        _square_tube_visuals(
            "socket",
            outer_size=SOCKET_SIZE,
            wall=socket_wall,
            length=SOCKET_HEIGHT,
            z0=BASE_ARM_THICK + BASE_BLOCK_THICK,
        )
    )
    outer_mast_visuals.extend(
        _square_tube_visuals(
            "outer_tube",
            outer_size=OUTER_SIZE,
            wall=OUTER_WALL,
            length=OUTER_HEIGHT,
            z0=BASE_ARM_THICK + BASE_BLOCK_THICK + SOCKET_HEIGHT - MAST_SOCKET_FUSE,
        )
    )
    _add_visuals(outer_mast, outer_mast_visuals, material="powder_coat_dark")
    outer_mast.inertial = Inertial.from_geometry(
        Box((BASE_BLOCK_SIZE, BASE_BLOCK_SIZE, OUTER_HEIGHT + SOCKET_HEIGHT + BASE_BLOCK_THICK)),
        mass=12.0,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                0.5 * (OUTER_HEIGHT + SOCKET_HEIGHT + BASE_BLOCK_THICK) + BASE_ARM_THICK,
            )
        ),
    )

    middle_tube = model.part("middle_tube")
    _add_visuals(
        middle_tube,
        [
            *_square_tube_visuals(
                "middle_tube",
                outer_size=MIDDLE_SIZE,
                wall=MIDDLE_WALL,
                length=MIDDLE_LENGTH,
                z0=-MIDDLE_REST_INSERT,
            ),
            *_guide_visuals(
                "middle_tube",
                tube_size=MIDDLE_SIZE,
                guide_width=MIDDLE_GUIDE_WIDTH,
                guide_length=MIDDLE_GUIDE_LENGTH,
                z0=-MIDDLE_REST_INSERT + 0.040,
            ),
        ],
        material="anodized_aluminum",
    )
    middle_tube.inertial = Inertial.from_geometry(
        Box((MIDDLE_SIZE, MIDDLE_SIZE, MIDDLE_LENGTH)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.5 * MIDDLE_LENGTH - MIDDLE_REST_INSERT)),
    )

    top_tube = model.part("top_tube")
    _add_visuals(
        top_tube,
        [
            *_square_tube_visuals(
                "top_tube",
                outer_size=TOP_SIZE,
                wall=TOP_WALL,
                length=TOP_LENGTH,
                z0=-TOP_REST_INSERT,
            ),
            *_guide_visuals(
                "top_tube",
                tube_size=TOP_SIZE,
                guide_width=TOP_GUIDE_WIDTH,
                guide_length=TOP_GUIDE_LENGTH,
                z0=-TOP_REST_INSERT + 0.025,
            ),
            (
                Box((TOP_CAP_SIZE, TOP_CAP_SIZE, TOP_CAP_THICK)),
                _centered_box_origin(
                    (TOP_CAP_SIZE, TOP_CAP_SIZE, TOP_CAP_THICK),
                    xyz=(0.0, 0.0, TOP_LENGTH - TOP_REST_INSERT),
                ),
                "top_tube_cap",
            ),
        ],
        material="brushed_alloy",
    )
    top_tube.inertial = Inertial.from_geometry(
        Box((TOP_CAP_SIZE, TOP_CAP_SIZE, TOP_LENGTH + TOP_CAP_THICK)),
        mass=1.8,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                0.5 * (TOP_LENGTH + TOP_CAP_THICK) - TOP_REST_INSERT,
            )
        ),
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=HEAD_BASE_RADIUS, length=HEAD_BASE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * HEAD_BASE_HEIGHT)),
        material="machined_head",
        name="pan_head_base",
    )
    pan_head.visual(
        Cylinder(radius=HEAD_NECK_RADIUS, length=HEAD_NECK_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, HEAD_BASE_HEIGHT + 0.5 * HEAD_NECK_HEIGHT)),
        material="machined_head",
        name="pan_head_neck",
    )
    pan_head.visual(
        Box((HEAD_FACE_SIZE, HEAD_FACE_SIZE, HEAD_FACE_THICK)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                HEAD_BASE_HEIGHT + HEAD_NECK_HEIGHT + 0.5 * HEAD_FACE_THICK,
            )
        ),
        material="machined_head",
        name="pan_head_face",
    )
    pan_head.inertial = Inertial.from_geometry(
        Box((HEAD_FACE_SIZE, HEAD_FACE_SIZE, HEAD_BASE_HEIGHT + HEAD_NECK_HEIGHT + HEAD_FACE_THICK)),
        mass=0.9,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                0.5 * (HEAD_BASE_HEIGHT + HEAD_NECK_HEIGHT + HEAD_FACE_THICK),
            )
        ),
    )

    outer_top_z = BASE_ARM_THICK + BASE_BLOCK_THICK + SOCKET_HEIGHT - MAST_SOCKET_FUSE + OUTER_HEIGHT
    middle_top_z = MIDDLE_LENGTH - MIDDLE_REST_INSERT
    top_head_mount_z = TOP_LENGTH - TOP_REST_INSERT + TOP_CAP_THICK

    model.articulation(
        "outer_to_middle_lift",
        ArticulationType.PRISMATIC,
        parent=outer_mast,
        child=middle_tube,
        origin=Origin(xyz=(0.0, 0.0, outer_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=MIDDLE_TRAVEL,
            effort=250.0,
            velocity=0.25,
        ),
    )
    model.articulation(
        "middle_to_top_lift",
        ArticulationType.PRISMATIC,
        parent=middle_tube,
        child=top_tube,
        origin=Origin(xyz=(0.0, 0.0, middle_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=TOP_TRAVEL,
            effort=180.0,
            velocity=0.25,
        ),
    )
    model.articulation(
        "top_tube_to_pan_head",
        ArticulationType.REVOLUTE,
        parent=top_tube,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, top_head_mount_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-pi,
            upper=pi,
            effort=20.0,
            velocity=2.0,
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

    outer_mast = object_model.get_part("outer_mast")
    middle_tube = object_model.get_part("middle_tube")
    top_tube = object_model.get_part("top_tube")
    pan_head = object_model.get_part("pan_head")

    outer_to_middle = object_model.get_articulation("outer_to_middle_lift")
    middle_to_top = object_model.get_articulation("middle_to_top_lift")
    pan_joint = object_model.get_articulation("top_tube_to_pan_head")

    ctx.expect_within(
        middle_tube,
        outer_mast,
        axes="xy",
        margin=0.0,
        name="middle tube stays centered within outer mast footprint",
    )
    ctx.expect_overlap(
        middle_tube,
        outer_mast,
        axes="z",
        min_overlap=MIDDLE_REST_INSERT - 0.01,
        name="middle tube starts with substantial insertion into outer mast",
    )
    with ctx.pose({outer_to_middle: MIDDLE_TRAVEL}):
        ctx.expect_within(
            middle_tube,
            outer_mast,
            axes="xy",
            margin=0.0,
            name="middle tube remains centered at full extension",
        )
        ctx.expect_overlap(
            middle_tube,
            outer_mast,
            axes="z",
            min_overlap=MIDDLE_REST_INSERT - MIDDLE_TRAVEL - 0.01,
            name="middle tube retains insertion at full extension",
        )

    ctx.expect_within(
        top_tube,
        middle_tube,
        axes="xy",
        margin=0.0,
        name="top tube stays centered within middle tube footprint",
    )
    ctx.expect_overlap(
        top_tube,
        middle_tube,
        axes="z",
        min_overlap=TOP_REST_INSERT - 0.01,
        name="top tube starts with substantial insertion into middle tube",
    )
    with ctx.pose(
        {
            outer_to_middle: MIDDLE_TRAVEL,
            middle_to_top: TOP_TRAVEL,
        }
    ):
        ctx.expect_within(
            top_tube,
            middle_tube,
            axes="xy",
            margin=0.0,
            name="top tube remains centered at full extension",
        )
        ctx.expect_overlap(
            top_tube,
            middle_tube,
            axes="z",
            min_overlap=TOP_REST_INSERT - TOP_TRAVEL - 0.01,
            name="top tube retains insertion at full extension",
        )

    middle_rest_pos = ctx.part_world_position(middle_tube)
    with ctx.pose({outer_to_middle: MIDDLE_TRAVEL}):
        middle_extended_pos = ctx.part_world_position(middle_tube)
    ctx.check(
        "middle stage extends upward",
        middle_rest_pos is not None
        and middle_extended_pos is not None
        and middle_extended_pos[2] > middle_rest_pos[2] + 0.10,
        details=f"rest={middle_rest_pos}, extended={middle_extended_pos}",
    )

    top_rest_pos = ctx.part_world_position(top_tube)
    with ctx.pose({middle_to_top: TOP_TRAVEL}):
        top_extended_pos = ctx.part_world_position(top_tube)
    ctx.check(
        "top stage extends upward",
        top_rest_pos is not None
        and top_extended_pos is not None
        and top_extended_pos[2] > top_rest_pos[2] + 0.08,
        details=f"rest={top_rest_pos}, extended={top_extended_pos}",
    )

    ctx.expect_contact(
        pan_head,
        top_tube,
        name="pan head seats on the top tube cap",
    )

    pan_rest_aabb = ctx.part_world_aabb(pan_head)
    pan_rest_pos = ctx.part_world_position(pan_head)
    with ctx.pose({pan_joint: pi / 4.0}):
        pan_rotated_aabb = ctx.part_world_aabb(pan_head)
        pan_rotated_pos = ctx.part_world_position(pan_head)

    pan_rest_span_x = None if pan_rest_aabb is None else pan_rest_aabb[1][0] - pan_rest_aabb[0][0]
    pan_rotated_span_x = (
        None if pan_rotated_aabb is None else pan_rotated_aabb[1][0] - pan_rotated_aabb[0][0]
    )
    ctx.check(
        "pan head rotates in place about vertical mast axis",
        pan_rest_pos is not None
        and pan_rotated_pos is not None
        and abs(pan_rest_pos[0] - pan_rotated_pos[0]) < 1e-6
        and abs(pan_rest_pos[1] - pan_rotated_pos[1]) < 1e-6
        and pan_rest_span_x is not None
        and pan_rotated_span_x is not None
        and pan_rotated_span_x > pan_rest_span_x + 0.01,
        details=(
            f"rest_pos={pan_rest_pos}, rotated_pos={pan_rotated_pos}, "
            f"rest_span_x={pan_rest_span_x}, rotated_span_x={pan_rotated_span_x}"
        ),
    )

    ctx.check(
        "pan joint is vertical",
        tuple(round(v, 6) for v in pan_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={pan_joint.axis}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
