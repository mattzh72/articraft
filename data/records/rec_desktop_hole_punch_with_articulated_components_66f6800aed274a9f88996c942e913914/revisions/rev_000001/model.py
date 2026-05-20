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
    model = ArticulatedObject(name="adjustable_multi_hole_punch")

    housing = model.material("housing", rgba=(0.18, 0.19, 0.21, 1.0))
    lever = model.material("lever", rgba=(0.30, 0.31, 0.34, 1.0))
    metal = model.material("metal", rgba=(0.74, 0.77, 0.80, 1.0))
    knob = model.material("knob", rgba=(0.11, 0.12, 0.13, 1.0))
    accent = model.material("accent", rgba=(0.85, 0.24, 0.12, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.320, 0.118, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=housing,
        name="base_plate",
    )
    body.visual(
        Box((0.320, 0.014, 0.016)),
        origin=Origin(xyz=(0.0, -0.052, 0.008)),
        material=housing,
        name="front_lip",
    )
    body.visual(
        Box((0.012, 0.104, 0.020)),
        origin=Origin(xyz=(-0.154, 0.0, 0.010)),
        material=housing,
        name="left_skirt",
    )
    body.visual(
        Box((0.012, 0.104, 0.020)),
        origin=Origin(xyz=(0.154, 0.0, 0.010)),
        material=housing,
        name="right_skirt",
    )
    body.visual(
        Box((0.272, 0.050, 0.030)),
        origin=Origin(xyz=(0.0, 0.018, 0.021)),
        material=housing,
        name="rear_housing",
    )
    body.visual(
        Box((0.246, 0.030, 0.012)),
        origin=Origin(xyz=(0.0, -0.004, 0.036)),
        material=metal,
        name="die_block",
    )
    for index, x in enumerate((-0.096, 0.0, 0.096)):
        body.visual(
            Cylinder(radius=0.009, length=0.010),
            origin=Origin(xyz=(x, -0.008, 0.047)),
            material=metal,
            name=f"die_ring_{index}",
        )
    body.visual(
        Box((0.286, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, -0.044, 0.019)),
        material=metal,
        name="front_fence",
    )
    body.visual(
        Box((0.018, 0.010, 0.022)),
        origin=Origin(xyz=(-0.142, -0.043, 0.011)),
        material=metal,
        name="fence_bracket_0",
    )
    body.visual(
        Box((0.018, 0.010, 0.022)),
        origin=Origin(xyz=(0.142, -0.043, 0.011)),
        material=metal,
        name="fence_bracket_1",
    )
    body.visual(
        Box((0.020, 0.016, 0.040)),
        origin=Origin(xyz=(-0.126, 0.050, 0.043)),
        material=metal,
        name="hinge_bracket_0",
    )
    body.visual(
        Box((0.020, 0.016, 0.040)),
        origin=Origin(xyz=(0.126, 0.050, 0.043)),
        material=metal,
        name="hinge_bracket_1",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.320, 0.118, 0.070)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.008, length=0.232),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="hinge_barrel",
    )
    handle.visual(
        Box((0.272, 0.034, 0.014)),
        origin=Origin(xyz=(0.0, -0.028, -0.004)),
        material=lever,
        name="rear_web",
    )
    handle.visual(
        Box((0.286, 0.158, 0.014)),
        origin=Origin(xyz=(0.0, -0.102, -0.006)),
        material=lever,
        name="lever_plate",
    )
    handle.visual(
        Box((0.210, 0.050, 0.010)),
        origin=Origin(xyz=(0.0, -0.080, 0.006)),
        material=lever,
        name="top_rib",
    )
    handle.visual(
        Cylinder(radius=0.011, length=0.288),
        origin=Origin(xyz=(0.0, -0.176, -0.001), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent,
        name="front_grip",
    )
    for index, x in enumerate((-0.096, 0.0, 0.096)):
        handle.visual(
            Cylinder(radius=0.0075, length=0.018),
            origin=Origin(xyz=(x, -0.012, -0.009)),
            material=metal,
            name=f"punch_pin_{index}",
        )
    handle.inertial = Inertial.from_geometry(
        Box((0.290, 0.190, 0.040)),
        mass=0.9,
        origin=Origin(xyz=(0.0, -0.095, -0.004)),
    )

    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, 0.050, 0.060)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(70.0),
        ),
    )

    stop = model.part("stop")
    stop.visual(
        Box((0.028, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, 0.000, -0.001)),
        material=housing,
        name="carriage_body",
    )
    stop.visual(
        Box((0.024, 0.004, 0.024)),
        origin=Origin(xyz=(0.0, -0.001, 0.010)),
        material=housing,
        name="rear_clamp",
    )
    stop.visual(
        Box((0.028, 0.016, 0.004)),
        origin=Origin(xyz=(0.0, 0.006, 0.016)),
        material=housing,
        name="top_hook",
    )
    stop.visual(
        Box((0.006, 0.024, 0.028)),
        origin=Origin(xyz=(-0.004, -0.012, 0.004)),
        material=metal,
        name="paper_stop",
    )
    stop.visual(
        Box((0.012, 0.016, 0.024)),
        origin=Origin(xyz=(0.014, -0.002, 0.008)),
        material=housing,
        name="dial_pedestal",
    )
    stop.inertial = Inertial.from_geometry(
        Box((0.042, 0.040, 0.040)),
        mass=0.12,
        origin=Origin(xyz=(0.003, 0.010, 0.011)),
    )

    model.articulation(
        "body_to_stop",
        ArticulationType.PRISMATIC,
        parent=body,
        child=stop,
        origin=Origin(xyz=(0.0, -0.065, 0.019)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.18,
            lower=-0.110,
            upper=0.110,
        ),
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.004, length=0.008),
        origin=Origin(xyz=(0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="shaft",
    )
    dial.visual(
        Cylinder(radius=0.014, length=0.012),
        origin=Origin(xyz=(0.014, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob,
        name="knob_body",
    )
    dial.visual(
        Box((0.004, 0.016, 0.004)),
        origin=Origin(xyz=(0.018, 0.0, 0.011)),
        material=accent,
        name="indicator",
    )
    dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.014, length=0.022),
        mass=0.03,
        origin=Origin(xyz=(0.011, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "stop_to_dial",
        ArticulationType.REVOLUTE,
        parent=stop,
        child=dial,
        origin=Origin(xyz=(0.020, -0.002, 0.010)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=4.0,
            lower=-1.2,
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
    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    stop = object_model.get_part("stop")
    dial = object_model.get_part("dial")

    handle_hinge = object_model.get_articulation("body_to_handle")
    stop_slide = object_model.get_articulation("body_to_stop")
    dial_joint = object_model.get_articulation("stop_to_dial")

    def elem_center(part_obj, elem_name: str):
        aabb = ctx.part_element_world_aabb(part_obj, elem=elem_name)
        if aabb is None:
            return None
        (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
        return (
            0.5 * (min_x + max_x),
            0.5 * (min_y + max_y),
            0.5 * (min_z + max_z),
        )

    ctx.check(
        "handle hinge uses rear transverse axis",
        tuple(handle_hinge.axis) == (-1.0, 0.0, 0.0),
        details=f"axis={handle_hinge.axis}",
    )
    ctx.check(
        "stop slides laterally along front fence",
        tuple(stop_slide.axis) == (1.0, 0.0, 0.0),
        details=f"axis={stop_slide.axis}",
    )
    ctx.check(
        "dial rotates about side shaft axis",
        tuple(dial_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={dial_joint.axis}",
    )

    ctx.expect_gap(
        handle,
        body,
        axis="z",
        positive_elem="front_grip",
        negative_elem="die_block",
        min_gap=0.005,
        max_gap=0.030,
        name="closed handle rests just above die block",
    )
    ctx.expect_within(
        stop,
        body,
        axes="x",
        inner_elem="carriage_body",
        outer_elem="front_fence",
        margin=0.010,
        name="stop carriage stays within fence span at rest",
    )

    closed_grip = elem_center(handle, "front_grip")
    with ctx.pose({handle_hinge: handle_hinge.motion_limits.upper}):
        open_grip = elem_center(handle, "front_grip")
    ctx.check(
        "handle opens upward from the rear hinge",
        closed_grip is not None
        and open_grip is not None
        and open_grip[2] > closed_grip[2] + 0.080
        and open_grip[1] > closed_grip[1] + 0.040,
        details=f"closed={closed_grip}, open={open_grip}",
    )

    rest_stop = elem_center(stop, "paper_stop")
    with ctx.pose({stop_slide: stop_slide.motion_limits.upper}):
        max_stop = elem_center(stop, "paper_stop")
        ctx.expect_within(
            stop,
            body,
            axes="x",
            inner_elem="carriage_body",
            outer_elem="front_fence",
            margin=0.010,
            name="stop carriage stays within fence span at max travel",
        )
    ctx.check(
        "paper stop shifts along the fence",
        rest_stop is not None
        and max_stop is not None
        and max_stop[0] > rest_stop[0] + 0.090,
        details=f"rest={rest_stop}, max={max_stop}",
    )

    rest_indicator = elem_center(dial, "indicator")
    with ctx.pose({dial_joint: dial_joint.motion_limits.upper}):
        turned_indicator = elem_center(dial, "indicator")
    ctx.check(
        "selector dial visibly rotates",
        rest_indicator is not None
        and turned_indicator is not None
        and abs(turned_indicator[2] - rest_indicator[2]) > 0.006
        and abs(turned_indicator[1] - rest_indicator[1]) > 0.006,
        details=f"rest={rest_indicator}, turned={turned_indicator}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
