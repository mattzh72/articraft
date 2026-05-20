from __future__ import annotations

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
    model = ArticulatedObject(name="compact_service_gantry")

    frame_mat = model.material("dark_powder_coated_steel", rgba=(0.09, 0.10, 0.11, 1.0))
    rail_mat = model.material("polished_linear_rail", rgba=(0.72, 0.74, 0.76, 1.0))
    bridge_mat = model.material("painted_blue_bridge", rgba=(0.05, 0.20, 0.42, 1.0))
    slide_mat = model.material("yellow_cross_slide_casting", rgba=(0.95, 0.66, 0.08, 1.0))
    tool_mat = model.material("matte_tool_face", rgba=(0.02, 0.025, 0.03, 1.0))
    stop_mat = model.material("black_rubber_stops", rgba=(0.015, 0.015, 0.014, 1.0))

    lower_frame = model.part("lower_frame")
    lower_frame.visual(
        Box((0.92, 0.58, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=frame_mat,
        name="base_plate",
    )
    lower_frame.visual(
        Box((0.84, 0.045, 0.035)),
        origin=Origin(xyz=(0.0, -0.235, 0.0565)),
        material=frame_mat,
        name="rail_pedestal_0",
    )
    lower_frame.visual(
        Cylinder(radius=0.018, length=0.82),
        origin=Origin(xyz=(0.0, -0.235, 0.091), rpy=(0.0, pi / 2.0, 0.0)),
        material=rail_mat,
        name="travel_rail_0",
    )
    lower_frame.visual(
        Box((0.036, 0.080, 0.070)),
        origin=Origin(xyz=(-0.43, -0.235, 0.075)),
        material=stop_mat,
        name="rear_stop_0",
    )
    lower_frame.visual(
        Box((0.036, 0.080, 0.070)),
        origin=Origin(xyz=(0.43, -0.235, 0.075)),
        material=stop_mat,
        name="front_stop_0",
    )
    lower_frame.visual(
        Box((0.84, 0.045, 0.035)),
        origin=Origin(xyz=(0.0, 0.235, 0.0565)),
        material=frame_mat,
        name="rail_pedestal_1",
    )
    lower_frame.visual(
        Cylinder(radius=0.018, length=0.82),
        origin=Origin(xyz=(0.0, 0.235, 0.091), rpy=(0.0, pi / 2.0, 0.0)),
        material=rail_mat,
        name="travel_rail_1",
    )
    lower_frame.visual(
        Box((0.036, 0.080, 0.070)),
        origin=Origin(xyz=(-0.43, 0.235, 0.075)),
        material=stop_mat,
        name="rear_stop_1",
    )
    lower_frame.visual(
        Box((0.036, 0.080, 0.070)),
        origin=Origin(xyz=(0.43, 0.235, 0.075)),
        material=stop_mat,
        name="front_stop_1",
    )

    bridge = model.part("bridge")
    bridge.visual(
        Box((0.100, 0.600, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.325)),
        material=bridge_mat,
        name="bridge_beam",
    )
    bridge.visual(
        Box((0.140, 0.085, 0.045)),
        origin=Origin(xyz=(0.0, -0.235, 0.1315)),
        material=bridge_mat,
        name="rail_carriage_0",
    )
    bridge.visual(
        Box((0.080, 0.075, 0.165)),
        origin=Origin(xyz=(0.0, -0.235, 0.2325)),
        material=bridge_mat,
        name="upright_cheek_0",
    )
    bridge.visual(
        Box((0.050, 0.100, 0.026)),
        origin=Origin(xyz=(0.0, -0.235, 0.174)),
        material=rail_mat,
        name="linear_bearing_cap_0",
    )
    bridge.visual(
        Box((0.140, 0.085, 0.045)),
        origin=Origin(xyz=(0.0, 0.235, 0.1315)),
        material=bridge_mat,
        name="rail_carriage_1",
    )
    bridge.visual(
        Box((0.080, 0.075, 0.165)),
        origin=Origin(xyz=(0.0, 0.235, 0.2325)),
        material=bridge_mat,
        name="upright_cheek_1",
    )
    bridge.visual(
        Box((0.050, 0.100, 0.026)),
        origin=Origin(xyz=(0.0, 0.235, 0.174)),
        material=rail_mat,
        name="linear_bearing_cap_1",
    )

    cross_slide = model.part("cross_slide")
    cross_slide.visual(
        Box((0.035, 0.180, 0.150)),
        origin=Origin(xyz=(-0.0175, 0.0, 0.0)),
        material=slide_mat,
        name="wide_saddle_plate",
    )
    cross_slide.visual(
        Box((0.048, 0.205, 0.030)),
        origin=Origin(xyz=(-0.024, 0.0, 0.060)),
        material=rail_mat,
        name="upper_slide_shoe",
    )
    cross_slide.visual(
        Box((0.048, 0.205, 0.030)),
        origin=Origin(xyz=(-0.024, 0.0, -0.060)),
        material=rail_mat,
        name="lower_slide_shoe",
    )
    cross_slide.visual(
        Box((0.060, 0.125, 0.095)),
        origin=Origin(xyz=(-0.060, 0.0, 0.0)),
        material=slide_mat,
        name="tool_support_boss",
    )

    tool_face = model.part("tool_face")
    tool_face.visual(
        Box((0.022, 0.070, 0.070)),
        origin=Origin(xyz=(-0.011, 0.0, 0.0)),
        material=tool_mat,
        name="small_work_face",
    )

    model.articulation(
        "frame_to_bridge",
        ArticulationType.PRISMATIC,
        parent=lower_frame,
        child=bridge,
        origin=Origin(xyz=(-0.22, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.44),
    )
    model.articulation(
        "bridge_to_cross_slide",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=cross_slide,
        origin=Origin(xyz=(-0.050, -0.16, 0.325)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.25, lower=0.0, upper=0.32),
    )
    model.articulation(
        "cross_slide_to_tool_face",
        ArticulationType.FIXED,
        parent=cross_slide,
        child=tool_face,
        origin=Origin(xyz=(-0.090, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower_frame = object_model.get_part("lower_frame")
    bridge = object_model.get_part("bridge")
    cross_slide = object_model.get_part("cross_slide")
    tool_face = object_model.get_part("tool_face")
    bridge_joint = object_model.get_articulation("frame_to_bridge")
    slide_joint = object_model.get_articulation("bridge_to_cross_slide")

    ctx.check(
        "bridge and cross-slide are orthogonal prismatic joints",
        bridge_joint.articulation_type == ArticulationType.PRISMATIC
        and slide_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(bridge_joint.axis) == (1.0, 0.0, 0.0)
        and tuple(slide_joint.axis) == (0.0, 1.0, 0.0),
        details=f"bridge axis={bridge_joint.axis}, slide axis={slide_joint.axis}",
    )

    ctx.expect_gap(
        bridge,
        lower_frame,
        axis="z",
        positive_elem="rail_carriage_0",
        negative_elem="travel_rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="bridge carriage sits on first fixed rail",
    )
    ctx.expect_gap(
        bridge,
        lower_frame,
        axis="z",
        positive_elem="rail_carriage_1",
        negative_elem="travel_rail_1",
        max_gap=0.001,
        max_penetration=0.0,
        name="bridge carriage sits on second fixed rail",
    )
    ctx.expect_contact(
        cross_slide,
        bridge,
        elem_a="wide_saddle_plate",
        elem_b="bridge_beam",
        contact_tol=0.001,
        name="cross-slide saddle bears on bridge face",
    )
    ctx.expect_contact(
        tool_face,
        cross_slide,
        elem_a="small_work_face",
        elem_b="tool_support_boss",
        contact_tol=0.001,
        name="tool face is mounted to support boss",
    )

    rest_bridge_pos = ctx.part_world_position(bridge)
    rest_slide_pos = ctx.part_world_position(cross_slide)
    with ctx.pose({bridge_joint: 0.44, slide_joint: 0.32}):
        moved_bridge_pos = ctx.part_world_position(bridge)
        moved_slide_pos = ctx.part_world_position(cross_slide)
        ctx.expect_overlap(
            bridge,
            lower_frame,
            axes="x",
            elem_a="rail_carriage_0",
            elem_b="travel_rail_0",
            min_overlap=0.08,
            name="bridge remains captured on the rail at full travel",
        )
        ctx.expect_within(
            cross_slide,
            bridge,
            axes="y",
            inner_elem="wide_saddle_plate",
            outer_elem="bridge_beam",
            margin=0.005,
            name="cross-slide remains within bridge span at full travel",
        )

    ctx.check(
        "bridge translates along the frame X axis",
        rest_bridge_pos is not None
        and moved_bridge_pos is not None
        and moved_bridge_pos[0] > rest_bridge_pos[0] + 0.40,
        details=f"rest={rest_bridge_pos}, moved={moved_bridge_pos}",
    )
    ctx.check(
        "cross-slide translates along the bridge Y axis",
        rest_slide_pos is not None
        and moved_slide_pos is not None
        and moved_slide_pos[1] > rest_slide_pos[1] + 0.28,
        details=f"rest={rest_slide_pos}, moved={moved_slide_pos}",
    )

    saddle_aabb = ctx.part_element_world_aabb(cross_slide, elem="wide_saddle_plate")
    face_aabb = ctx.part_element_world_aabb(tool_face, elem="small_work_face")
    if saddle_aabb is not None and face_aabb is not None:
        saddle_size = (
            saddle_aabb[1][0] - saddle_aabb[0][0],
            saddle_aabb[1][1] - saddle_aabb[0][1],
            saddle_aabb[1][2] - saddle_aabb[0][2],
        )
        face_size = (
            face_aabb[1][0] - face_aabb[0][0],
            face_aabb[1][1] - face_aabb[0][1],
            face_aabb[1][2] - face_aabb[0][2],
        )
        hardware_larger = saddle_size[1] > face_size[1] * 2.0 and saddle_size[2] > face_size[2] * 2.0
    else:
        hardware_larger = False
        saddle_size = face_size = None
    ctx.check(
        "cross-slide hardware is visibly larger than the tool face",
        hardware_larger,
        details=f"saddle_size={saddle_size}, face_size={face_size}",
    )

    return ctx.report()


object_model = build_object_model()
