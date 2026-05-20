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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="long_reach_hole_punch")

    body = model.material("body", rgba=(0.24, 0.26, 0.29, 1.0))
    body_dark = model.material("body_dark", rgba=(0.16, 0.17, 0.19, 1.0))
    handle_grip = model.material("handle_grip", rgba=(0.08, 0.08, 0.09, 1.0))
    rail = model.material("rail", rgba=(0.75, 0.76, 0.78, 1.0))

    frame = model.part("frame")

    base_plate = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.380, 0.130, 0.015), 0.008),
        "base_plate",
    )
    frame.visual(
        base_plate,
        origin=Origin(xyz=(0.000, 0.000, 0.004)),
        material=body,
        name="base_plate",
    )
    frame.visual(
        Box((0.112, 0.108, 0.046)),
        origin=Origin(xyz=(-0.134, 0.000, 0.023)),
        material=body,
        name="rear_body",
    )
    frame.visual(
        Box((0.074, 0.074, 0.020)),
        origin=Origin(xyz=(-0.108, 0.000, 0.052)),
        material=body_dark,
        name="punch_head",
    )
    for sign in (-1.0, 1.0):
        frame.visual(
            Box((0.262, 0.012, 0.020)),
            origin=Origin(xyz=(0.018, sign * 0.040, 0.018)),
            material=body_dark,
            name=f"side_rail_{int((sign + 1.0) * 0.5)}",
        )
        frame.visual(
            Box((0.020, 0.014, 0.058)),
            origin=Origin(xyz=(-0.158, sign * 0.031, 0.071)),
            material=body,
            name=f"hinge_cheek_{int((sign + 1.0) * 0.5)}",
        )
        frame.visual(
            Cylinder(radius=0.008, length=0.014),
            origin=Origin(
                xyz=(-0.158, sign * 0.031, 0.093),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=body_dark,
            name=f"hinge_boss_{int((sign + 1.0) * 0.5)}",
        )
    frame.visual(
        Box((0.020, 0.112, 0.022)),
        origin=Origin(xyz=(0.151, 0.000, 0.019)),
        material=rail,
        name="front_fence",
    )
    frame.visual(
        Box((0.030, 0.118, 0.006)),
        origin=Origin(xyz=(0.138, 0.000, 0.033)),
        material=rail,
        name="fence_cap",
    )
    frame.visual(
        Box((0.052, 0.056, 0.010)),
        origin=Origin(xyz=(-0.134, 0.000, 0.082)),
        material=body_dark,
        name="hinge_bridge",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.380, 0.130, 0.102)),
        mass=1.8,
        origin=Origin(xyz=(0.000, 0.000, 0.030)),
    )

    handle = model.part("handle")
    for sign in (-1.0, 1.0):
        handle.visual(
            Cylinder(radius=0.0065, length=0.016),
            origin=Origin(
                xyz=(0.000, sign * 0.012, 0.000),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=body_dark,
            name=f"hinge_barrel_{int((sign + 1.0) * 0.5)}",
        )
        handle.visual(
            Box((0.026, 0.016, 0.024)),
            origin=Origin(xyz=(0.016, sign * 0.012, 0.011)),
            material=body,
            name=f"hinge_riser_{int((sign + 1.0) * 0.5)}",
        )
    handle.visual(
        Box((0.034, 0.044, 0.018)),
        origin=Origin(xyz=(0.028, 0.000, 0.024)),
        material=body,
        name="rear_tie",
    )
    handle.visual(
        Box((0.096, 0.050, 0.018)),
        origin=Origin(
            xyz=(0.074, 0.000, 0.036),
            rpy=(0.000, 0.120, 0.000),
        ),
        material=body,
        name="lever_neck",
    )
    handle.visual(
        Box((0.238, 0.058, 0.014)),
        origin=Origin(
            xyz=(0.135, 0.000, 0.042),
            rpy=(0.000, 0.210, 0.000),
        ),
        material=body,
        name="lever_body",
    )
    handle.visual(
        Box((0.052, 0.072, 0.022)),
        origin=Origin(xyz=(0.232, 0.000, 0.071)),
        material=handle_grip,
        name="grip_block",
    )
    handle.visual(
        Box((0.086, 0.066, 0.024)),
        origin=Origin(xyz=(0.188, 0.000, 0.055)),
        material=handle_grip,
        name="grip_neck",
    )
    handle.visual(
        Cylinder(radius=0.016, length=0.094),
        origin=Origin(
            xyz=(0.242, 0.000, 0.078),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=handle_grip,
        name="grip_roll",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.260, 0.090, 0.110)),
        mass=0.75,
        origin=Origin(xyz=(0.130, 0.000, 0.044)),
    )

    model.articulation(
        "frame_to_handle",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=handle,
        origin=Origin(xyz=(-0.158, 0.000, 0.093)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=0.0,
            upper=1.18,
        ),
    )

    guide = model.part("guide")
    guide.visual(
        Box((0.050, 0.024, 0.008)),
        origin=Origin(xyz=(0.004, 0.000, 0.004)),
        material=rail,
        name="shoe",
    )
    guide.visual(
        Box((0.008, 0.024, 0.038)),
        origin=Origin(xyz=(0.028, 0.000, -0.011)),
        material=rail,
        name="stop_plate",
    )
    guide.visual(
        Box((0.010, 0.012, 0.014)),
        origin=Origin(xyz=(0.032, 0.012, 0.006)),
        material=body_dark,
        name="knob_mount",
    )
    guide.inertial = Inertial.from_geometry(
        Box((0.048, 0.028, 0.044)),
        mass=0.18,
        origin=Origin(xyz=(0.012, 0.000, 0.002)),
    )

    model.articulation(
        "frame_to_guide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=guide,
        origin=Origin(xyz=(0.137, 0.000, 0.036)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.20,
            lower=-0.042,
            upper=0.042,
        ),
    )

    stop_knob = model.part("stop_knob")
    stop_knob.visual(
        Cylinder(radius=0.004, length=0.006),
        origin=Origin(
            xyz=(0.000, 0.003, 0.000),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=body_dark,
        name="shaft",
    )
    stop_knob.visual(
        Cylinder(radius=0.013, length=0.008),
        origin=Origin(
            xyz=(0.000, 0.010, 0.000),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=handle_grip,
        name="wheel",
    )
    stop_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.013, length=0.014),
        mass=0.06,
        origin=Origin(
            xyz=(0.000, 0.008, 0.000),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
    )

    model.articulation(
        "guide_to_stop_knob",
        ArticulationType.CONTINUOUS,
        parent=guide,
        child=stop_knob,
        origin=Origin(xyz=(0.032, 0.018, 0.006)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=6.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    handle = object_model.get_part("handle")
    guide = object_model.get_part("guide")
    stop_knob = object_model.get_part("stop_knob")

    handle_hinge = object_model.get_articulation("frame_to_handle")
    guide_slide = object_model.get_articulation("frame_to_guide")
    knob_spin = object_model.get_articulation("guide_to_stop_knob")

    ctx.expect_contact(
        guide,
        frame,
        elem_a="shoe",
        elem_b="fence_cap",
        name="guide shoe rests on the fence cap",
    )
    ctx.expect_contact(
        guide,
        frame,
        elem_a="stop_plate",
        elem_b="front_fence",
        name="guide stop plate meets the front fence",
    )

    closed_grip = ctx.part_element_world_aabb(handle, elem="grip_roll")
    with ctx.pose({handle_hinge: 1.0}):
        open_grip = ctx.part_element_world_aabb(handle, elem="grip_roll")
    ctx.check(
        "handle lifts high when opened",
        closed_grip is not None
        and open_grip is not None
        and open_grip[0][2] > closed_grip[0][2] + 0.12
        and open_grip[1][0] < closed_grip[1][0] - 0.12,
        details=f"closed={closed_grip}, open={open_grip}",
    )

    rest_guide = ctx.part_world_position(guide)
    rest_knob = ctx.part_world_position(stop_knob)
    with ctx.pose({guide_slide: guide_slide.motion_limits.upper}):
        high_guide = ctx.part_world_position(guide)
        high_knob = ctx.part_world_position(stop_knob)
        ctx.expect_contact(
            guide,
            frame,
            elem_a="shoe",
            elem_b="fence_cap",
            name="guide shoe stays supported at positive travel",
        )
    with ctx.pose({guide_slide: guide_slide.motion_limits.lower}):
        low_guide = ctx.part_world_position(guide)
        low_knob = ctx.part_world_position(stop_knob)
        ctx.expect_contact(
            guide,
            frame,
            elem_a="shoe",
            elem_b="fence_cap",
            name="guide shoe stays supported at negative travel",
        )
    ctx.check(
        "guide slides transversely across the fence",
        rest_guide is not None
        and high_guide is not None
        and low_guide is not None
        and high_guide[1] > rest_guide[1] + 0.035
        and low_guide[1] < rest_guide[1] - 0.035,
        details=f"rest={rest_guide}, high={high_guide}, low={low_guide}",
    )
    ctx.check(
        "stop knob travels with the guide carriage",
        rest_knob is not None
        and high_knob is not None
        and low_knob is not None
        and high_knob[1] > rest_knob[1] + 0.035
        and low_knob[1] < rest_knob[1] - 0.035,
        details=f"rest={rest_knob}, high={high_knob}, low={low_knob}",
    )
    ctx.check(
        "stop knob uses a side-facing spin axis",
        knob_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 3) for v in knob_spin.axis) == (0.0, 1.0, 0.0),
        details=(
            f"type={knob_spin.articulation_type}, axis={knob_spin.axis}, "
            f"handle_axis={handle_hinge.axis}, guide_axis={guide_slide.axis}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
