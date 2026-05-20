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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="angle_vise")

    cast_iron = model.material("cast_iron", rgba=(0.26, 0.28, 0.30, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.14, 0.14, 0.15, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.320, 0.180, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=cast_iron,
        name="base_plate",
    )
    base.visual(
        Box((0.110, 0.180, 0.032)),
        origin=Origin(xyz=(-0.105, 0.0, 0.034)),
        material=cast_iron,
        name="rear_pedestal",
    )
    base.visual(
        Box((0.120, 0.180, 0.022)),
        origin=Origin(xyz=(0.085, 0.0, 0.029)),
        material=cast_iron,
        name="front_rib",
    )
    base.visual(
        Box((0.032, 0.028, 0.058)),
        origin=Origin(xyz=(-0.105, 0.064, 0.079)),
        material=cast_iron,
        name="left_yoke",
    )
    base.visual(
        Box((0.032, 0.028, 0.058)),
        origin=Origin(xyz=(-0.105, -0.064, 0.079)),
        material=cast_iron,
        name="right_yoke",
    )
    base.visual(
        Box((0.060, 0.092, 0.014)),
        origin=Origin(xyz=(0.015, 0.0, 0.047)),
        material=machined_steel,
        name="angle_table_support",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.320, 0.180, 0.108)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
    )

    tilt_frame = model.part("tilt_frame")
    tilt_frame.visual(
        Cylinder(radius=0.013, length=0.100),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_oxide,
        name="frame_trunnion",
    )
    tilt_frame.visual(
        Box((0.050, 0.060, 0.026)),
        origin=Origin(xyz=(0.015, 0.0, 0.013)),
        material=cast_iron,
        name="trunnion_web",
    )
    tilt_frame.visual(
        Box((0.255, 0.094, 0.018)),
        origin=Origin(xyz=(0.1275, 0.0, 0.022)),
        material=machined_steel,
        name="slide_bed",
    )
    tilt_frame.visual(
        Box((0.200, 0.018, 0.010)),
        origin=Origin(xyz=(0.132, 0.036, 0.036)),
        material=machined_steel,
        name="left_way",
    )
    tilt_frame.visual(
        Box((0.200, 0.018, 0.010)),
        origin=Origin(xyz=(0.132, -0.036, 0.036)),
        material=machined_steel,
        name="right_way",
    )
    tilt_frame.visual(
        Box((0.030, 0.094, 0.026)),
        origin=Origin(xyz=(0.019, 0.0, 0.044)),
        material=cast_iron,
        name="fixed_jaw_pedestal",
    )
    tilt_frame.visual(
        Box((0.024, 0.094, 0.060)),
        origin=Origin(xyz=(0.018, 0.0, 0.087)),
        material=cast_iron,
        name="fixed_jaw_block",
    )
    tilt_frame.visual(
        Box((0.008, 0.094, 0.050)),
        origin=Origin(xyz=(0.034, 0.0, 0.082)),
        material=machined_steel,
        name="fixed_jaw_face",
    )
    tilt_frame.visual(
        Box((0.016, 0.094, 0.030)),
        origin=Origin(xyz=(0.247, 0.0, 0.046)),
        material=cast_iron,
        name="front_stop",
    )
    tilt_frame.inertial = Inertial.from_geometry(
        Box((0.255, 0.100, 0.092)),
        mass=4.5,
        origin=Origin(xyz=(0.128, 0.0, 0.046)),
    )

    moving_jaw = model.part("moving_jaw")
    moving_jaw.visual(
        Box((0.130, 0.046, 0.012)),
        origin=Origin(xyz=(-0.028, 0.0, 0.006)),
        material=machined_steel,
        name="jaw_carriage",
    )
    moving_jaw.visual(
        Box((0.028, 0.094, 0.058)),
        origin=Origin(xyz=(0.014, 0.0, 0.041)),
        material=cast_iron,
        name="jaw_block",
    )
    moving_jaw.visual(
        Box((0.040, 0.060, 0.018)),
        origin=Origin(xyz=(0.018, 0.0, 0.079)),
        material=cast_iron,
        name="jaw_cap",
    )
    moving_jaw.visual(
        Box((0.008, 0.086, 0.046)),
        origin=Origin(xyz=(0.004, 0.0, 0.049)),
        material=machined_steel,
        name="moving_jaw_face",
    )
    moving_jaw.inertial = Inertial.from_geometry(
        Box((0.130, 0.094, 0.097)),
        mass=2.0,
        origin=Origin(xyz=(-0.010, 0.0, 0.048)),
    )

    model.articulation(
        "base_to_tilt_frame",
        ArticulationType.REVOLUTE,
        parent=base,
        child=tilt_frame,
        origin=Origin(xyz=(-0.105, 0.0, 0.078)),
        # The table extends along local +X from the rear trunnion.
        # -Y makes positive q tilt the front edge upward.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(55.0),
        ),
    )
    model.articulation(
        "tilt_frame_to_moving_jaw",
        ArticulationType.PRISMATIC,
        parent=tilt_frame,
        child=moving_jaw,
        origin=Origin(xyz=(0.132, 0.0, 0.031)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.08,
            lower=0.0,
            upper=0.070,
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

    base = object_model.get_part("base")
    tilt_frame = object_model.get_part("tilt_frame")
    moving_jaw = object_model.get_part("moving_jaw")
    tilt_joint = object_model.get_articulation("base_to_tilt_frame")
    jaw_slide = object_model.get_articulation("tilt_frame_to_moving_jaw")

    ctx.expect_contact(
        base,
        tilt_frame,
        elem_a="left_yoke",
        elem_b="frame_trunnion",
        name="left yoke supports the rear trunnion",
    )
    ctx.expect_contact(
        base,
        tilt_frame,
        elem_a="right_yoke",
        elem_b="frame_trunnion",
        name="right yoke supports the rear trunnion",
    )
    ctx.expect_within(
        moving_jaw,
        tilt_frame,
        axes="y",
        inner_elem="jaw_carriage",
        outer_elem="slide_bed",
        margin=0.0,
        name="moving jaw carriage stays centered on the bed width",
    )
    with ctx.pose({jaw_slide: jaw_slide.motion_limits.upper}):
        ctx.expect_overlap(
            moving_jaw,
            tilt_frame,
            axes="x",
            elem_a="jaw_carriage",
            elem_b="slide_bed",
            min_overlap=0.120,
            name="moving jaw retains insertion on the bed at full opening",
        )

    closed_stop_aabb = ctx.part_element_world_aabb(tilt_frame, elem="front_stop")
    with ctx.pose({tilt_joint: tilt_joint.motion_limits.upper}):
        open_stop_aabb = ctx.part_element_world_aabb(tilt_frame, elem="front_stop")

    def _center_z(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float | None:
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    ctx.check(
        "tilt joint raises the front of the vise",
        closed_stop_aabb is not None
        and open_stop_aabb is not None
        and _center_z(open_stop_aabb) is not None
        and _center_z(closed_stop_aabb) is not None
        and _center_z(open_stop_aabb) > _center_z(closed_stop_aabb) + 0.08,
        details=f"closed_stop={closed_stop_aabb}, open_stop={open_stop_aabb}",
    )

    with ctx.pose({tilt_joint: math.radians(30.0), jaw_slide: 0.0}):
        closed_pos = ctx.part_world_position(moving_jaw)
    with ctx.pose({tilt_joint: math.radians(30.0), jaw_slide: jaw_slide.motion_limits.upper}):
        open_pos = ctx.part_world_position(moving_jaw)

    ctx.check(
        "moving jaw slides forward along the tilted bed",
        closed_pos is not None
        and open_pos is not None
        and open_pos[0] > closed_pos[0] + 0.04
        and open_pos[2] > closed_pos[2] + 0.02,
        details=f"closed_pos={closed_pos}, open_pos={open_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
