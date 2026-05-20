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
    model = ArticulatedObject(name="compact_pickup_tailgate")

    body_paint = model.material("body_paint", rgba=(0.16, 0.24, 0.34, 1.0))
    bed_liner = model.material("bed_liner", rgba=(0.17, 0.17, 0.18, 1.0))
    hardware = model.material("hardware", rgba=(0.10, 0.10, 0.11, 1.0))
    metal = model.material("metal", rgba=(0.65, 0.66, 0.68, 1.0))

    bed_frame = model.part("bed_frame")
    bed_frame.visual(
        Box((0.610, 1.280, 0.028)),
        origin=Origin(xyz=(0.322, 0.0, 0.086)),
        material=bed_liner,
        name="bed_floor",
    )
    bed_frame.visual(
        Box((0.060, 0.160, 0.028)),
        origin=Origin(xyz=(-0.012, 0.180, 0.086)),
        material=bed_liner,
        name="left_sill_pad",
    )
    bed_frame.visual(
        Box((0.060, 0.160, 0.028)),
        origin=Origin(xyz=(-0.012, -0.180, 0.086)),
        material=bed_liner,
        name="right_sill_pad",
    )
    bed_frame.visual(
        Box((0.610, 0.060, 0.390)),
        origin=Origin(xyz=(0.322, 0.670, 0.295)),
        material=body_paint,
        name="left_bedside",
    )
    bed_frame.visual(
        Box((0.610, 0.060, 0.390)),
        origin=Origin(xyz=(0.322, -0.670, 0.295)),
        material=body_paint,
        name="right_bedside",
    )
    bed_frame.visual(
        Box((0.610, 0.088, 0.036)),
        origin=Origin(xyz=(0.322, 0.656, 0.492)),
        material=bed_liner,
        name="left_rail_cap",
    )
    bed_frame.visual(
        Box((0.610, 0.088, 0.036)),
        origin=Origin(xyz=(0.322, -0.656, 0.492)),
        material=bed_liner,
        name="right_rail_cap",
    )
    bed_frame.visual(
        Box((0.082, 0.040, 0.070)),
        origin=Origin(xyz=(-0.010, 0.655, 0.115)),
        material=body_paint,
        name="left_hinge_support",
    )
    bed_frame.visual(
        Box((0.082, 0.040, 0.070)),
        origin=Origin(xyz=(-0.010, -0.655, 0.115)),
        material=body_paint,
        name="right_hinge_support",
    )
    bed_frame.visual(
        Box((0.028, 1.400, 0.420)),
        origin=Origin(xyz=(0.641, 0.0, 0.310)),
        material=body_paint,
        name="front_bulkhead",
    )
    bed_frame.inertial = Inertial.from_geometry(
        Box((0.669, 1.400, 0.456)),
        mass=65.0,
        origin=Origin(xyz=(0.305, 0.0, 0.228)),
    )

    tailgate = model.part("tailgate")
    tailgate.visual(
        Box((0.018, 1.240, 0.332)),
        origin=Origin(xyz=(0.015, 0.0, 0.206)),
        material=body_paint,
        name="outer_skin",
    )
    tailgate.visual(
        Box((0.045, 1.240, 0.040)),
        origin=Origin(xyz=(0.023, 0.0, 0.020)),
        material=body_paint,
        name="bottom_rail",
    )
    tailgate.visual(
        Box((0.034, 1.240, 0.030)),
        origin=Origin(xyz=(0.020, 0.0, 0.347)),
        material=body_paint,
        name="top_cap",
    )
    tailgate.visual(
        Box((0.045, 0.026, 0.338)),
        origin=Origin(xyz=(0.023, 0.607, 0.209)),
        material=body_paint,
        name="left_side_frame",
    )
    tailgate.visual(
        Box((0.045, 0.026, 0.338)),
        origin=Origin(xyz=(0.023, -0.607, 0.209)),
        material=body_paint,
        name="right_side_frame",
    )
    tailgate.visual(
        Cylinder(radius=0.013, length=0.160),
        origin=Origin(xyz=(0.010, 0.360, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="left_hinge_barrel",
    )
    tailgate.visual(
        Cylinder(radius=0.013, length=0.120),
        origin=Origin(xyz=(0.010, 0.000, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="center_hinge_barrel",
    )
    tailgate.visual(
        Cylinder(radius=0.013, length=0.160),
        origin=Origin(xyz=(0.010, -0.360, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="right_hinge_barrel",
    )
    tailgate.visual(
        Box((0.006, 1.130, 0.232)),
        origin=Origin(xyz=(0.041, 0.0, 0.192)),
        material=bed_liner,
        name="inner_panel",
    )
    tailgate.visual(
        Box((0.010, 0.022, 0.290)),
        origin=Origin(xyz=(0.037, 0.594, 0.190)),
        material=bed_liner,
        name="left_inner_flange",
    )
    tailgate.visual(
        Box((0.010, 0.022, 0.290)),
        origin=Origin(xyz=(0.037, -0.594, 0.190)),
        material=bed_liner,
        name="right_inner_flange",
    )
    tailgate.visual(
        Box((0.010, 1.160, 0.022)),
        origin=Origin(xyz=(0.037, 0.0, 0.323)),
        material=bed_liner,
        name="top_inner_flange",
    )
    tailgate.visual(
        Box((0.010, 1.160, 0.024)),
        origin=Origin(xyz=(0.037, 0.0, 0.050)),
        material=bed_liner,
        name="bottom_inner_flange",
    )
    tailgate.visual(
        Box((0.012, 0.940, 0.056)),
        origin=Origin(xyz=(0.035, 0.0, 0.262)),
        material=bed_liner,
        name="upper_inner_rib",
    )
    tailgate.visual(
        Box((0.012, 0.940, 0.060)),
        origin=Origin(xyz=(0.035, 0.0, 0.122)),
        material=bed_liner,
        name="lower_inner_rib",
    )
    tailgate.visual(
        Box((0.010, 0.660, 0.104)),
        origin=Origin(xyz=(0.034, 0.0, 0.192)),
        material=bed_liner,
        name="center_inner_rib",
    )
    tailgate.visual(
        Box((0.006, 0.056, 0.032)),
        origin=Origin(xyz=(0.037, 0.435, 0.304)),
        material=hardware,
        name="left_latch_bezel",
    )
    tailgate.visual(
        Box((0.006, 0.056, 0.032)),
        origin=Origin(xyz=(0.037, -0.435, 0.304)),
        material=hardware,
        name="right_latch_bezel",
    )
    tailgate.visual(
        Box((0.006, 0.045, 0.018)),
        origin=Origin(xyz=(0.035, 0.340, 0.349)),
        material=hardware,
        name="left_work_stop_pad",
    )
    tailgate.visual(
        Box((0.006, 0.045, 0.018)),
        origin=Origin(xyz=(0.035, -0.340, 0.349)),
        material=hardware,
        name="right_work_stop_pad",
    )
    tailgate.inertial = Inertial.from_geometry(
        Box((0.050, 1.240, 0.370)),
        mass=12.0,
        origin=Origin(xyz=(0.022, 0.0, 0.185)),
    )

    left_latch_knob = model.part("left_latch_knob")
    left_latch_knob.visual(
        Cylinder(radius=0.005, length=0.012),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="shaft",
    )
    left_latch_knob.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="hub",
    )
    left_latch_knob.visual(
        Box((0.008, 0.040, 0.014)),
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
        material=hardware,
        name="paddle",
    )
    left_latch_knob.visual(
        Box((0.006, 0.012, 0.010)),
        origin=Origin(xyz=(0.019, 0.0, 0.012)),
        material=hardware,
        name="pointer",
    )
    left_latch_knob.inertial = Inertial.from_geometry(
        Box((0.025, 0.040, 0.024)),
        mass=0.08,
        origin=Origin(xyz=(0.0125, 0.0, 0.0)),
    )

    right_latch_knob = model.part("right_latch_knob")
    right_latch_knob.visual(
        Cylinder(radius=0.005, length=0.012),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="shaft",
    )
    right_latch_knob.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="hub",
    )
    right_latch_knob.visual(
        Box((0.008, 0.040, 0.014)),
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
        material=hardware,
        name="paddle",
    )
    right_latch_knob.visual(
        Box((0.006, 0.012, 0.010)),
        origin=Origin(xyz=(0.019, 0.0, 0.012)),
        material=hardware,
        name="pointer",
    )
    right_latch_knob.inertial = Inertial.from_geometry(
        Box((0.025, 0.040, 0.024)),
        mass=0.08,
        origin=Origin(xyz=(0.0125, 0.0, 0.0)),
    )

    work_stop = model.part("work_stop")
    work_stop.visual(
        Cylinder(radius=0.007, length=0.040),
        origin=Origin(xyz=(0.0, 0.340, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="left_sleeve",
    )
    work_stop.visual(
        Cylinder(radius=0.007, length=0.040),
        origin=Origin(xyz=(0.0, -0.340, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="right_sleeve",
    )
    work_stop.visual(
        Box((0.010, 0.028, 0.084)),
        origin=Origin(xyz=(0.006, 0.340, -0.042)),
        material=metal,
        name="left_arm",
    )
    work_stop.visual(
        Box((0.010, 0.028, 0.084)),
        origin=Origin(xyz=(0.006, -0.340, -0.042)),
        material=metal,
        name="right_arm",
    )
    work_stop.visual(
        Box((0.012, 1.000, 0.018)),
        origin=Origin(xyz=(0.010, 0.0, -0.078)),
        material=metal,
        name="stop_bar",
    )
    work_stop.inertial = Inertial.from_geometry(
        Box((0.020, 1.000, 0.090)),
        mass=0.9,
        origin=Origin(xyz=(0.010, 0.0, -0.040)),
    )

    model.articulation(
        "bed_to_tailgate",
        ArticulationType.REVOLUTE,
        parent=bed_frame,
        child=tailgate,
        origin=Origin(xyz=(-0.036, 0.0, 0.100)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=1.3,
            lower=0.0,
            upper=math.radians(86.0),
        ),
    )
    model.articulation(
        "tailgate_to_left_latch_knob",
        ArticulationType.REVOLUTE,
        parent=tailgate,
        child=left_latch_knob,
        origin=Origin(xyz=(0.046, 0.435, 0.304)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
            lower=-1.2,
            upper=1.2,
        ),
    )
    model.articulation(
        "tailgate_to_right_latch_knob",
        ArticulationType.REVOLUTE,
        parent=tailgate,
        child=right_latch_knob,
        origin=Origin(xyz=(0.046, -0.435, 0.304)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
            lower=-1.2,
            upper=1.2,
        ),
    )
    model.articulation(
        "tailgate_to_work_stop",
        ArticulationType.REVOLUTE,
        parent=tailgate,
        child=work_stop,
        origin=Origin(xyz=(0.045, 0.0, 0.349)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(78.0),
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

    bed_frame = object_model.get_part("bed_frame")
    tailgate = object_model.get_part("tailgate")
    left_latch_knob = object_model.get_part("left_latch_knob")
    right_latch_knob = object_model.get_part("right_latch_knob")
    work_stop = object_model.get_part("work_stop")

    tailgate_hinge = object_model.get_articulation("bed_to_tailgate")
    left_knob_joint = object_model.get_articulation("tailgate_to_left_latch_knob")
    right_knob_joint = object_model.get_articulation("tailgate_to_right_latch_knob")
    work_stop_joint = object_model.get_articulation("tailgate_to_work_stop")

    def _axis_span(aabb, axis_index: int) -> float | None:
        if aabb is None:
            return None
        return aabb[1][axis_index] - aabb[0][axis_index]

    def _max_aabb_delta(aabb_a, aabb_b) -> float | None:
        if aabb_a is None or aabb_b is None:
            return None
        return max(
            abs(aabb_a[corner][axis] - aabb_b[corner][axis])
            for corner in (0, 1)
            for axis in range(3)
        )

    ctx.expect_contact(
        left_latch_knob,
        tailgate,
        elem_a="shaft",
        elem_b="left_latch_bezel",
        name="left latch knob seats on its bezel",
    )
    ctx.expect_contact(
        right_latch_knob,
        tailgate,
        elem_a="shaft",
        elem_b="right_latch_bezel",
        name="right latch knob seats on its bezel",
    )
    ctx.expect_contact(
        work_stop,
        tailgate,
        elem_a="left_sleeve",
        elem_b="left_work_stop_pad",
        name="work-stop left sleeve seats on hinge pad",
    )
    ctx.expect_contact(
        work_stop,
        tailgate,
        elem_a="right_sleeve",
        elem_b="right_work_stop_pad",
        name="work-stop right sleeve seats on hinge pad",
    )

    closed_gate_aabb = ctx.part_world_aabb(tailgate)
    with ctx.pose({tailgate_hinge: math.radians(82.0)}):
        open_gate_aabb = ctx.part_world_aabb(tailgate)

    gate_opens_down = (
        closed_gate_aabb is not None
        and open_gate_aabb is not None
        and open_gate_aabb[0][0] < closed_gate_aabb[0][0] - 0.22
        and open_gate_aabb[1][2] < closed_gate_aabb[1][2] - 0.18
    )
    ctx.check(
        "tailgate rotates down from the lower hinge line",
        gate_opens_down,
        details=f"closed={closed_gate_aabb}, open={open_gate_aabb}",
    )

    left_paddle_closed = ctx.part_element_world_aabb(left_latch_knob, elem="paddle")
    right_paddle_closed = ctx.part_element_world_aabb(right_latch_knob, elem="paddle")
    with ctx.pose({left_knob_joint: math.pi / 2.0}):
        left_paddle_turned = ctx.part_element_world_aabb(left_latch_knob, elem="paddle")
        right_paddle_static_during_left = ctx.part_element_world_aabb(right_latch_knob, elem="paddle")

    left_closed_y = _axis_span(left_paddle_closed, 1)
    left_closed_z = _axis_span(left_paddle_closed, 2)
    left_turned_y = _axis_span(left_paddle_turned, 1)
    left_turned_z = _axis_span(left_paddle_turned, 2)
    right_static_delta = _max_aabb_delta(right_paddle_closed, right_paddle_static_during_left)
    left_knob_rotates = (
        left_closed_y is not None
        and left_closed_z is not None
        and left_turned_y is not None
        and left_turned_z is not None
        and right_static_delta is not None
        and left_closed_y > left_closed_z + 0.015
        and left_turned_z > left_turned_y + 0.015
        and right_static_delta < 1e-6
    )
    ctx.check(
        "left latch knob rotates on its own shaft",
        left_knob_rotates,
        details=(
            f"closed={left_paddle_closed}, turned={left_paddle_turned}, "
            f"right_static_delta={right_static_delta}"
        ),
    )

    with ctx.pose({right_knob_joint: math.pi / 2.0}):
        right_paddle_turned = ctx.part_element_world_aabb(right_latch_knob, elem="paddle")
        left_paddle_static_during_right = ctx.part_element_world_aabb(left_latch_knob, elem="paddle")

    right_closed_y = _axis_span(right_paddle_closed, 1)
    right_closed_z = _axis_span(right_paddle_closed, 2)
    right_turned_y = _axis_span(right_paddle_turned, 1)
    right_turned_z = _axis_span(right_paddle_turned, 2)
    left_static_delta = _max_aabb_delta(left_paddle_closed, left_paddle_static_during_right)
    right_knob_rotates = (
        right_closed_y is not None
        and right_closed_z is not None
        and right_turned_y is not None
        and right_turned_z is not None
        and left_static_delta is not None
        and right_closed_y > right_closed_z + 0.015
        and right_turned_z > right_turned_y + 0.015
        and left_static_delta < 1e-6
    )
    ctx.check(
        "right latch knob rotates on its own shaft",
        right_knob_rotates,
        details=(
            f"closed={right_paddle_closed}, turned={right_paddle_turned}, "
            f"left_static_delta={left_static_delta}"
        ),
    )

    with ctx.pose({tailgate_hinge: math.radians(82.0), work_stop_joint: 0.0}):
        stowed_stop_aabb = ctx.part_world_aabb(work_stop)
    with ctx.pose(
        {
            tailgate_hinge: math.radians(82.0),
            work_stop_joint: math.radians(78.0),
        }
    ):
        raised_stop_aabb = ctx.part_world_aabb(work_stop)

    work_stop_raises = (
        stowed_stop_aabb is not None
        and raised_stop_aabb is not None
        and open_gate_aabb is not None
        and raised_stop_aabb[1][2] > stowed_stop_aabb[1][2] + 0.055
        and raised_stop_aabb[1][2] > open_gate_aabb[1][2] + 0.065
    )
    ctx.check(
        "work-stop bar folds up from the top edge",
        work_stop_raises,
        details=f"stowed={stowed_stop_aabb}, raised={raised_stop_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
