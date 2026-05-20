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


OUTER_LENGTH = 0.54
OUTER_WIDTH = 0.12
OUTER_HEIGHT = 0.075
OUTER_WALL = 0.006
OUTER_MOUNT_LENGTH = 0.08

MIDDLE_LENGTH = 0.42
MIDDLE_WIDTH = 0.104
MIDDLE_HEIGHT = 0.056
MIDDLE_WALL = 0.005

INNER_LENGTH = 0.31
INNER_WIDTH = 0.09
INNER_HEIGHT = 0.041
INNER_WALL = 0.004

TIP_LENGTH = 0.12
TIP_WIDTH = 0.078
TIP_HEIGHT = 0.03

OUTER_TO_MIDDLE_TRAVEL = 0.24
MIDDLE_TO_INNER_TRAVEL = 0.18
INNER_TO_TIP_TRAVEL = 0.14

OUTER_TO_MIDDLE_Z = OUTER_WALL + 0.003
MIDDLE_TO_INNER_Z = MIDDLE_WALL + 0.003
INNER_TO_TIP_Z = INNER_WALL + 0.0015

def _add_u_channel_visuals(
    part,
    *,
    material: str,
    floor_name: str,
    left_name: str,
    right_name: str,
    length: float,
    width: float,
    height: float,
    wall: float,
    rear_mount: float = 0.0,
    mount_width_scale: float = 1.35,
) -> None:
    fuse_depth = wall * 0.2
    side_height = height - wall + fuse_depth

    part.visual(
        Box((length, width, wall)),
        origin=Origin(xyz=(length / 2.0, 0.0, wall / 2.0)),
        material=material,
        name=floor_name,
    )
    part.visual(
        Box((length, wall, side_height)),
        origin=Origin(
            xyz=(
                length / 2.0,
                width / 2.0 - wall / 2.0,
                wall + side_height / 2.0 - fuse_depth,
            )
        ),
        material=material,
        name=left_name,
    )
    part.visual(
        Box((length, wall, side_height)),
        origin=Origin(
            xyz=(
                length / 2.0,
                -width / 2.0 + wall / 2.0,
                wall + side_height / 2.0 - fuse_depth,
            )
        ),
        material=material,
        name=right_name,
    )

    if rear_mount > 0.0:
        plate_thickness = wall * 1.8
        part.visual(
            Box((rear_mount, width * mount_width_scale, plate_thickness)),
            origin=Origin(
                xyz=(-rear_mount / 2.0, 0.0, plate_thickness * 0.15),
            ),
            material=material,
            name="outer_mount_plate",
        )
        part.visual(
            Box((wall * 2.5, width * 1.05, height * 0.32)),
            origin=Origin(
                xyz=(-wall * 1.25, 0.0, height * 0.16),
            ),
            material=material,
            name="outer_rear_block",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_carriage_slide_chain")

    model.material("outer_steel", rgba=(0.24, 0.27, 0.30, 1.0))
    model.material("middle_steel", rgba=(0.46, 0.49, 0.52, 1.0))
    model.material("inner_steel", rgba=(0.69, 0.71, 0.74, 1.0))
    model.material("tip_alloy", rgba=(0.82, 0.84, 0.86, 1.0))

    outer = model.part("outer_guide")
    _add_u_channel_visuals(
        outer,
        material="outer_steel",
        floor_name="outer_floor",
        left_name="outer_left_wall",
        right_name="outer_right_wall",
        length=OUTER_LENGTH,
        width=OUTER_WIDTH,
        height=OUTER_HEIGHT,
        wall=OUTER_WALL,
        rear_mount=OUTER_MOUNT_LENGTH,
        mount_width_scale=1.42,
    )
    outer.inertial = Inertial.from_geometry(
        Box((OUTER_LENGTH + OUTER_MOUNT_LENGTH, OUTER_WIDTH * 1.42, OUTER_HEIGHT + 0.01)),
        mass=4.2,
        origin=Origin(
            xyz=(
                (OUTER_LENGTH - OUTER_MOUNT_LENGTH) / 2.0,
                0.0,
                OUTER_HEIGHT / 2.0 - 0.002,
            )
        ),
    )

    middle = model.part("middle_stage")
    _add_u_channel_visuals(
        middle,
        material="middle_steel",
        floor_name="middle_floor",
        left_name="middle_left_wall",
        right_name="middle_right_wall",
        length=MIDDLE_LENGTH,
        width=MIDDLE_WIDTH,
        height=MIDDLE_HEIGHT,
        wall=MIDDLE_WALL,
    )
    middle.inertial = Inertial.from_geometry(
        Box((MIDDLE_LENGTH, MIDDLE_WIDTH, MIDDLE_HEIGHT)),
        mass=2.4,
        origin=Origin(xyz=(MIDDLE_LENGTH / 2.0, 0.0, MIDDLE_HEIGHT / 2.0)),
    )

    inner = model.part("inner_stage")
    _add_u_channel_visuals(
        inner,
        material="inner_steel",
        floor_name="inner_floor",
        left_name="inner_left_wall",
        right_name="inner_right_wall",
        length=INNER_LENGTH,
        width=INNER_WIDTH,
        height=INNER_HEIGHT,
        wall=INNER_WALL,
    )
    inner.inertial = Inertial.from_geometry(
        Box((INNER_LENGTH, INNER_WIDTH, INNER_HEIGHT)),
        mass=1.5,
        origin=Origin(xyz=(INNER_LENGTH / 2.0, 0.0, INNER_HEIGHT / 2.0)),
    )

    tip = model.part("tip_stage")
    tip.visual(
        Box((TIP_LENGTH, TIP_WIDTH, TIP_HEIGHT)),
        material="tip_alloy",
        name="tip_body",
        origin=Origin(xyz=(TIP_LENGTH / 2.0, 0.0, TIP_HEIGHT / 2.0)),
    )
    tip.inertial = Inertial.from_geometry(
        Box((TIP_LENGTH, TIP_WIDTH, TIP_HEIGHT)),
        mass=0.7,
        origin=Origin(xyz=(TIP_LENGTH / 2.0, 0.0, TIP_HEIGHT / 2.0)),
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(0.0, 0.0, OUTER_TO_MIDDLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=OUTER_TO_MIDDLE_TRAVEL,
            effort=180.0,
            velocity=0.45,
        ),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_TO_INNER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=MIDDLE_TO_INNER_TRAVEL,
            effort=140.0,
            velocity=0.45,
        ),
    )
    model.articulation(
        "inner_to_tip",
        ArticulationType.PRISMATIC,
        parent=inner,
        child=tip,
        origin=Origin(xyz=(0.0, 0.0, INNER_TO_TIP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=INNER_TO_TIP_TRAVEL,
            effort=90.0,
            velocity=0.5,
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

    outer = object_model.get_part("outer_guide")
    middle = object_model.get_part("middle_stage")
    inner = object_model.get_part("inner_stage")
    tip = object_model.get_part("tip_stage")

    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")
    inner_to_tip = object_model.get_articulation("inner_to_tip")

    for joint, upper in (
        (outer_to_middle, OUTER_TO_MIDDLE_TRAVEL),
        (middle_to_inner, MIDDLE_TO_INNER_TRAVEL),
        (inner_to_tip, INNER_TO_TIP_TRAVEL),
    ):
        ctx.check(
            f"{joint.name} slides along +X",
            tuple(joint.axis) == (1.0, 0.0, 0.0),
            details=f"axis={joint.axis}",
        )
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name} travel limit matches design",
            limits is not None and limits.lower == 0.0 and limits.upper == upper,
            details=f"limits={limits}",
        )

    def _check_nested_stage(
        parent,
        child,
        joint,
        *,
        travel: float,
        rest_overlap: float,
        extended_overlap: float,
    ) -> None:
        ctx.expect_within(
            child,
            parent,
            axes="yz",
            name=f"{child.name} stays centered in {parent.name} at rest",
        )
        ctx.expect_overlap(
            child,
            parent,
            axes="x",
            min_overlap=rest_overlap,
            name=f"{child.name} remains inserted in {parent.name} at rest",
        )
        rest_pos = ctx.part_world_position(child)
        with ctx.pose({joint: travel}):
            ctx.expect_within(
                child,
                parent,
                axes="yz",
                name=f"{child.name} stays centered in {parent.name} at full extension",
            )
            ctx.expect_overlap(
                child,
                parent,
                axes="x",
                min_overlap=extended_overlap,
                name=f"{child.name} retains insertion in {parent.name} at full extension",
            )
            extended_pos = ctx.part_world_position(child)

        ctx.check(
            f"{child.name} extends in the positive X direction",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[0] > rest_pos[0] + 0.75 * travel,
            details=f"rest={rest_pos}, extended={extended_pos}, travel={travel}",
        )

    _check_nested_stage(
        outer,
        middle,
        outer_to_middle,
        travel=OUTER_TO_MIDDLE_TRAVEL,
        rest_overlap=MIDDLE_LENGTH,
        extended_overlap=OUTER_LENGTH - OUTER_TO_MIDDLE_TRAVEL,
    )
    _check_nested_stage(
        middle,
        inner,
        middle_to_inner,
        travel=MIDDLE_TO_INNER_TRAVEL,
        rest_overlap=INNER_LENGTH,
        extended_overlap=MIDDLE_LENGTH - MIDDLE_TO_INNER_TRAVEL,
    )
    _check_nested_stage(
        inner,
        tip,
        inner_to_tip,
        travel=INNER_TO_TIP_TRAVEL,
        rest_overlap=TIP_LENGTH,
        extended_overlap=TIP_LENGTH,
    )

    with ctx.pose(
        {
            outer_to_middle: OUTER_TO_MIDDLE_TRAVEL,
            middle_to_inner: MIDDLE_TO_INNER_TRAVEL,
            inner_to_tip: INNER_TO_TIP_TRAVEL,
        }
    ):
        middle_pos = ctx.part_world_position(middle)
        inner_pos = ctx.part_world_position(inner)
        tip_pos = ctx.part_world_position(tip)

    ctx.check(
        "serial stages telescope outward in order",
        middle_pos is not None
        and inner_pos is not None
        and tip_pos is not None
        and 0.0 < middle_pos[0] < inner_pos[0] < tip_pos[0],
        details=f"middle={middle_pos}, inner={inner_pos}, tip={tip_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
