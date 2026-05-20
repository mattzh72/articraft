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
    model = ArticulatedObject(name="heavy_duty_paper_punch")

    body_metal = model.material("body_metal", rgba=(0.27, 0.29, 0.31, 1.0))
    trim_black = model.material("trim_black", rgba=(0.10, 0.11, 0.12, 1.0))
    steel = model.material("steel", rgba=(0.74, 0.77, 0.80, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.18, 0.18, 0.18, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.340, 0.120, 0.012)),
        origin=Origin(xyz=(0.0, 0.064, 0.006)),
        material=body_metal,
        name="rear_deck",
    )
    body.visual(
        Box((0.340, 0.022, 0.012)),
        origin=Origin(xyz=(0.0, -0.109, 0.006)),
        material=body_metal,
        name="front_strip",
    )
    body.visual(
        Box((0.090, 0.102, 0.012)),
        origin=Origin(xyz=(-0.125, -0.047, 0.006)),
        material=body_metal,
        name="left_deck",
    )
    body.visual(
        Box((0.090, 0.102, 0.012)),
        origin=Origin(xyz=(0.125, -0.047, 0.006)),
        material=body_metal,
        name="right_deck",
    )
    body.visual(
        Box((0.300, 0.004, 0.040)),
        origin=Origin(xyz=(0.0, -0.117, 0.020)),
        material=trim_black,
        name="guide_back",
    )
    body.visual(
        Box((0.300, 0.006, 0.006)),
        origin=Origin(xyz=(0.0, -0.103, 0.007)),
        material=trim_black,
        name="guide_lower",
    )
    body.visual(
        Box((0.300, 0.006, 0.006)),
        origin=Origin(xyz=(0.0, -0.103, 0.033)),
        material=trim_black,
        name="guide_upper",
    )
    body.visual(
        Box((0.300, 0.008, 0.006)),
        origin=Origin(xyz=(0.0, -0.100, 0.020)),
        material=trim_black,
        name="guide_lip",
    )
    body.visual(
        Box((0.024, 0.150, 0.070)),
        origin=Origin(xyz=(-0.138, 0.010, 0.047)),
        material=body_metal,
        name="left_cheek",
    )
    body.visual(
        Box((0.024, 0.150, 0.070)),
        origin=Origin(xyz=(0.138, 0.010, 0.047)),
        material=body_metal,
        name="right_cheek",
    )
    body.visual(
        Box((0.252, 0.120, 0.020)),
        origin=Origin(xyz=(0.0, 0.020, 0.084)),
        material=body_metal,
        name="top_beam",
    )
    body.visual(
        Box((0.300, 0.035, 0.032)),
        origin=Origin(xyz=(0.0, -0.055, 0.066)),
        material=body_metal,
        name="front_bridge",
    )
    body.visual(
        Box((0.120, 0.032, 0.024)),
        origin=Origin(xyz=(0.0, -0.070, 0.052)),
        material=trim_black,
        name="punch_head",
    )
    body.visual(
        Box((0.160, 0.038, 0.006)),
        origin=Origin(xyz=(0.0, -0.053, 0.015)),
        material=steel,
        name="die_plate",
    )
    body.visual(
        Box((0.026, 0.020, 0.018)),
        origin=Origin(xyz=(-0.138, 0.084, 0.067)),
        material=body_metal,
        name="left_hinge_pedestal",
    )
    body.visual(
        Box((0.026, 0.020, 0.018)),
        origin=Origin(xyz=(0.138, 0.084, 0.067)),
        material=body_metal,
        name="right_hinge_pedestal",
    )
    body.visual(
        Box((0.008, 0.020, 0.038)),
        origin=Origin(xyz=(-0.146, -0.109, 0.020)),
        material=trim_black,
        name="left_guide_end",
    )
    body.visual(
        Box((0.008, 0.020, 0.038)),
        origin=Origin(xyz=(0.146, -0.109, 0.020)),
        material=trim_black,
        name="right_guide_end",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.340, 0.250, 0.100)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.008, length=0.290),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )
    handle.visual(
        Box((0.016, 0.120, 0.018)),
        origin=Origin(xyz=(-0.135, -0.055, 0.045), rpy=(-0.68, 0.0, 0.0)),
        material=steel,
        name="left_arm",
    )
    handle.visual(
        Box((0.016, 0.120, 0.018)),
        origin=Origin(xyz=(0.135, -0.055, 0.045), rpy=(-0.68, 0.0, 0.0)),
        material=steel,
        name="right_arm",
    )
    handle.visual(
        Box((0.018, 0.018, 0.016)),
        origin=Origin(xyz=(-0.135, -0.003, 0.006)),
        material=steel,
        name="left_knuckle",
    )
    handle.visual(
        Box((0.018, 0.018, 0.016)),
        origin=Origin(xyz=(0.135, -0.003, 0.006)),
        material=steel,
        name="right_knuckle",
    )
    handle.visual(
        Box((0.278, 0.028, 0.016)),
        origin=Origin(xyz=(0.0, -0.102, 0.085)),
        material=steel,
        name="handle_beam",
    )
    handle.visual(
        Cylinder(radius=0.014, length=0.270),
        origin=Origin(
            xyz=(0.0, -0.113, 0.091),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=grip_rubber,
        name="grip_pad",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.290, 0.130, 0.110)),
        mass=1.2,
        origin=Origin(xyz=(0.0, -0.060, 0.050)),
    )

    gauge_block = model.part("gauge_block")
    gauge_block.visual(
        Box((0.028, 0.007, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=steel,
        name="gauge_body",
    )
    gauge_block.visual(
        Box((0.016, 0.004, 0.018)),
        origin=Origin(xyz=(0.0, 0.006, 0.032)),
        material=body_metal,
        name="gauge_tab",
    )
    gauge_block.visual(
        Box((0.005, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, -0.002, 0.020)),
        material=trim_black,
        name="gauge_marker",
    )
    gauge_block.inertial = Inertial.from_geometry(
        Box((0.028, 0.014, 0.036)),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.001, 0.022)),
    )

    waste_door = model.part("waste_door")
    waste_door.visual(
        Box((0.154, 0.082, 0.003)),
        origin=Origin(xyz=(0.0, -0.041, -0.0015)),
        material=trim_black,
        name="door_panel",
    )
    waste_door.visual(
        Box((0.100, 0.008, 0.006)),
        origin=Origin(xyz=(0.0, -0.072, -0.0045)),
        material=body_metal,
        name="door_pull",
    )
    waste_door.visual(
        Cylinder(radius=0.004, length=0.154),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="door_hinge",
    )
    waste_door.inertial = Inertial.from_geometry(
        Box((0.154, 0.082, 0.008)),
        mass=0.18,
        origin=Origin(xyz=(0.0, -0.041, -0.003)),
    )

    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, 0.089, 0.086)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=1.2,
            lower=0.0,
            upper=0.48,
        ),
    )
    model.articulation(
        "body_to_gauge_block",
        ArticulationType.PRISMATIC,
        parent=body,
        child=gauge_block,
        origin=Origin(xyz=(0.0, -0.110, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=0.18,
            lower=-0.115,
            upper=0.115,
        ),
    )
    model.articulation(
        "body_to_waste_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=waste_door,
        origin=Origin(xyz=(0.0, 0.001, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.5,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    gauge_block = object_model.get_part("gauge_block")
    waste_door = object_model.get_part("waste_door")

    handle_joint = object_model.get_articulation("body_to_handle")
    gauge_joint = object_model.get_articulation("body_to_gauge_block")
    door_joint = object_model.get_articulation("body_to_waste_door")

    ctx.expect_gap(
        gauge_block,
        body,
        axis="y",
        positive_elem="gauge_body",
        negative_elem="guide_back",
        min_gap=0.0010,
        max_gap=0.0045,
        name="gauge block clears the rear guide wall",
    )
    ctx.expect_gap(
        body,
        gauge_block,
        axis="y",
        positive_elem="guide_lip",
        negative_elem="gauge_body",
        min_gap=0.0005,
        max_gap=0.0035,
        name="gauge block stays behind the front lip",
    )
    ctx.expect_gap(
        gauge_block,
        body,
        axis="z",
        positive_elem="gauge_body",
        negative_elem="guide_lower",
        min_gap=0.0005,
        max_gap=0.0060,
        name="gauge block rides above the lower rail",
    )
    ctx.expect_gap(
        body,
        gauge_block,
        axis="z",
        positive_elem="guide_upper",
        negative_elem="gauge_body",
        min_gap=0.0005,
        max_gap=0.0060,
        name="gauge block stays below the upper rail",
    )

    rest_gauge_pos = ctx.part_world_position(gauge_block)
    with ctx.pose({gauge_joint: 0.095}):
        shifted_gauge_pos = ctx.part_world_position(gauge_block)
    ctx.check(
        "gauge block slides along the front guide",
        rest_gauge_pos is not None
        and shifted_gauge_pos is not None
        and shifted_gauge_pos[0] > rest_gauge_pos[0] + 0.08,
        details=f"rest={rest_gauge_pos}, shifted={shifted_gauge_pos}",
    )

    rest_grip_aabb = ctx.part_element_world_aabb(handle, elem="grip_pad")
    with ctx.pose({handle_joint: 0.45}):
        pressed_grip_aabb = ctx.part_element_world_aabb(handle, elem="grip_pad")
        ctx.expect_gap(
            handle,
            body,
            axis="z",
            positive_elem="grip_pad",
            negative_elem="top_beam",
            min_gap=0.010,
            name="pressed handle remains above the punch housing",
        )
    ctx.check(
        "lever handle rotates downward from the raised position",
        rest_grip_aabb is not None
        and pressed_grip_aabb is not None
        and pressed_grip_aabb[1][2] < rest_grip_aabb[1][2] - 0.040,
        details=f"rest={rest_grip_aabb}, pressed={pressed_grip_aabb}",
    )

    closed_door_aabb = ctx.part_element_world_aabb(waste_door, elem="door_panel")
    with ctx.pose({door_joint: 1.10}):
        open_door_aabb = ctx.part_element_world_aabb(waste_door, elem="door_panel")
    ctx.check(
        "waste door swings downward below the punch body",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][2] < closed_door_aabb[0][2] - 0.050,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
