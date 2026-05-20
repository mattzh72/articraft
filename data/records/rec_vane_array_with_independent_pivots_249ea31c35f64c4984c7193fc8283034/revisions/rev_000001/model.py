from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FRAME_OUTER_WIDTH = 1.28
FRAME_OUTER_HEIGHT = 0.96
FRAME_DEPTH = 0.085
FRAME_SIDE_WIDTH = 0.06
FRAME_TOP_RAIL = 0.06
FRAME_BOTTOM_RAIL = 0.09

OPENING_WIDTH = FRAME_OUTER_WIDTH - 2.0 * FRAME_SIDE_WIDTH
OPENING_HEIGHT = FRAME_OUTER_HEIGHT - FRAME_TOP_RAIL - FRAME_BOTTOM_RAIL

SLAT_COUNT = 5
SLAT_DEPTH = 0.068
SLAT_THICKNESS = 0.012
SLAT_COLLAR_LENGTH = 0.008
SLAT_COLLAR_RADIUS = 0.012
SLAT_CORE_LENGTH = OPENING_WIDTH - 2.0 * SLAT_COLLAR_LENGTH

SLAT_LIMIT = 1.0


def _slat_centers() -> list[float]:
    step = 0.15
    first = FRAME_BOTTOM_RAIL + (OPENING_HEIGHT - step * (SLAT_COUNT - 1)) / 2.0
    return [first + i * step for i in range(SLAT_COUNT)]


def _slat_shape():
    core = (
        cq.Workplane("YZ")
        .ellipse(SLAT_DEPTH / 2.0, SLAT_THICKNESS / 2.0)
        .extrude(SLAT_CORE_LENGTH / 2.0, both=True)
        .val()
    )
    left_collar = cq.Solid.makeCylinder(
        SLAT_COLLAR_RADIUS,
        SLAT_COLLAR_LENGTH,
        cq.Vector(-OPENING_WIDTH / 2.0, 0.0, 0.0),
        cq.Vector(1.0, 0.0, 0.0),
    )
    right_collar = cq.Solid.makeCylinder(
        SLAT_COLLAR_RADIUS,
        SLAT_COLLAR_LENGTH,
        cq.Vector(OPENING_WIDTH / 2.0, 0.0, 0.0),
        cq.Vector(-1.0, 0.0, 0.0),
    )
    return core.fuse(left_collar).fuse(right_collar)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="framed_louver_bank")

    model.material("frame_coat", rgba=(0.21, 0.24, 0.27, 1.0))
    model.material("slat_finish", rgba=(0.75, 0.78, 0.80, 1.0))

    slat_centers = _slat_centers()

    frame = model.part("frame")
    frame.visual(
        Box((FRAME_SIDE_WIDTH, FRAME_DEPTH, FRAME_OUTER_HEIGHT)),
        origin=Origin(
            xyz=(-FRAME_OUTER_WIDTH / 2.0 + FRAME_SIDE_WIDTH / 2.0, 0.0, FRAME_OUTER_HEIGHT / 2.0)
        ),
        material="frame_coat",
        name="left_jamb",
    )
    frame.visual(
        Box((FRAME_SIDE_WIDTH, FRAME_DEPTH, FRAME_OUTER_HEIGHT)),
        origin=Origin(
            xyz=(FRAME_OUTER_WIDTH / 2.0 - FRAME_SIDE_WIDTH / 2.0, 0.0, FRAME_OUTER_HEIGHT / 2.0)
        ),
        material="frame_coat",
        name="right_jamb",
    )
    frame.visual(
        Box((FRAME_OUTER_WIDTH, FRAME_DEPTH, FRAME_TOP_RAIL)),
        origin=Origin(xyz=(0.0, 0.0, FRAME_OUTER_HEIGHT - FRAME_TOP_RAIL / 2.0)),
        material="frame_coat",
        name="top_rail",
    )
    frame.visual(
        Box((FRAME_OUTER_WIDTH, FRAME_DEPTH, FRAME_BOTTOM_RAIL)),
        origin=Origin(xyz=(0.0, 0.0, FRAME_BOTTOM_RAIL / 2.0)),
        material="frame_coat",
        name="bottom_rail",
    )
    frame.inertial = Inertial.from_geometry(
        Box((FRAME_OUTER_WIDTH, FRAME_DEPTH, FRAME_OUTER_HEIGHT)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, FRAME_OUTER_HEIGHT / 2.0)),
    )

    for index, z_pos in enumerate(slat_centers, start=1):
        slat = model.part(f"slat_{index}")
        slat.visual(
            mesh_from_cadquery(_slat_shape(), f"slat_{index}_shell"),
            material="slat_finish",
            name="slat_shell",
        )
        slat.inertial = Inertial.from_geometry(
            Box((OPENING_WIDTH, SLAT_DEPTH, SLAT_THICKNESS)),
            mass=1.15,
        )

        model.articulation(
            f"frame_to_slat_{index}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=slat,
            origin=Origin(xyz=(0.0, 0.0, z_pos)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                lower=-SLAT_LIMIT,
                upper=SLAT_LIMIT,
                effort=3.0,
                velocity=1.5,
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

    frame = object_model.get_part("frame")
    slats = [object_model.get_part(f"slat_{i}") for i in range(1, SLAT_COUNT + 1)]
    joints = [object_model.get_articulation(f"frame_to_slat_{i}") for i in range(1, SLAT_COUNT + 1)]

    ctx.check("frame present", frame is not None)
    for index, (slat, joint) in enumerate(zip(slats, joints), start=1):
        limits = joint.motion_limits
        ctx.check(f"slat_{index} present", slat is not None)
        ctx.check(
            f"slat_{index} rotates on its long axis",
            joint.axis == (1.0, 0.0, 0.0)
            and limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower < 0.0 < limits.upper,
            details=f"axis={joint.axis}, limits={limits}",
        )
        ctx.expect_contact(
            slat,
            frame,
            contact_tol=0.0005,
            name=f"slat_{index} stays supported by the frame pivots",
        )

    for lower, upper in zip(slats[:-1], slats[1:]):
        ctx.expect_gap(
            upper,
            lower,
            axis="z",
            min_gap=0.05,
            positive_elem="slat_shell",
            negative_elem="slat_shell",
            name=f"{upper.name} clears {lower.name} at rest",
        )

    staggered_pose = {
        joints[i]: 0.85 if i % 2 == 0 else -0.85
        for i in range(SLAT_COUNT)
    }
    with ctx.pose(staggered_pose):
        for lower, upper in zip(slats[:-1], slats[1:]):
            ctx.expect_gap(
                upper,
                lower,
                axis="z",
                min_gap=0.02,
                positive_elem="slat_shell",
                negative_elem="slat_shell",
                name=f"{upper.name} clears {lower.name} in a staggered pose",
            )

    mid_index = SLAT_COUNT // 2
    rest_aabb = ctx.part_element_world_aabb(slats[mid_index], elem="slat_shell")
    rest_origin = ctx.part_world_position(slats[mid_index])
    with ctx.pose({joints[mid_index]: 0.8}):
        tilted_aabb = ctx.part_element_world_aabb(slats[mid_index], elem="slat_shell")
        tilted_origin = ctx.part_world_position(slats[mid_index])

    rest_dims = None
    tilted_dims = None
    if rest_aabb is not None:
        rest_dims = tuple(rest_aabb[1][axis] - rest_aabb[0][axis] for axis in range(3))
    if tilted_aabb is not None:
        tilted_dims = tuple(tilted_aabb[1][axis] - tilted_aabb[0][axis] for axis in range(3))

    ctx.check(
        "middle slat tilts about its long axis without drifting off the pivot line",
        rest_dims is not None
        and tilted_dims is not None
        and rest_origin is not None
        and tilted_origin is not None
        and tilted_dims[2] > rest_dims[2] + 0.03
        and abs(tilted_dims[0] - rest_dims[0]) < 0.01
        and abs(tilted_origin[2] - rest_origin[2]) < 1e-6,
        details=(
            f"rest_dims={rest_dims}, tilted_dims={tilted_dims}, "
            f"rest_origin={rest_origin}, tilted_origin={tilted_origin}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
