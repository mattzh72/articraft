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
    model = ArticulatedObject(name="compact_router")

    body_black = model.material("body_black", rgba=(0.10, 0.11, 0.12, 1.0))
    charcoal = model.material("charcoal", rgba=(0.15, 0.16, 0.17, 1.0))
    switch_gray = model.material("switch_gray", rgba=(0.58, 0.60, 0.62, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    body_width = 0.186
    body_depth = 0.120
    base_thickness = 0.005
    wall_thickness = 0.004
    shell_height = 0.018
    total_height = 0.025
    hinge_y = 0.018
    hinge_z = 0.015
    hinge_x = body_width * 0.5

    slot_width = 0.026
    slot_height = 0.004
    slot_travel = 0.012

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, total_height)),
        mass=0.62,
        origin=Origin(xyz=(0.0, 0.0, total_height * 0.5)),
    )
    body.visual(
        Box((body_width - 0.004, body_depth - 0.004, base_thickness)),
        origin=Origin(xyz=(0.0, 0.0, base_thickness * 0.5)),
        material=charcoal,
        name="elem_base_plate",
    )

    roof_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(body_width, body_depth, 0.015, corner_segments=8),
            0.004,
            center=True,
        ),
        "router_roof",
    )
    body.visual(
        roof_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=body_black,
        name="elem_top_shell",
    )

    wall_z = base_thickness + shell_height * 0.5
    wall_length = body_depth - 2.0 * wall_thickness
    front_rear_span = body_width - 2.0 * wall_thickness
    body.visual(
        Box((wall_thickness, wall_length, shell_height)),
        origin=Origin(xyz=(-body_width * 0.5 + wall_thickness * 0.5, 0.0, wall_z)),
        material=body_black,
        name="elem_left_wall",
    )
    body.visual(
        Box((wall_thickness, wall_length, shell_height)),
        origin=Origin(xyz=(body_width * 0.5 - wall_thickness * 0.5, 0.0, wall_z)),
        material=body_black,
        name="elem_right_wall",
    )
    body.visual(
        Box((front_rear_span, wall_thickness, shell_height)),
        origin=Origin(xyz=(0.0, -body_depth * 0.5 + wall_thickness * 0.5, wall_z)),
        material=body_black,
        name="elem_front_wall",
    )

    rear_y = body_depth * 0.5 - wall_thickness * 0.5
    body.visual(
        Box((front_rear_span, wall_thickness, 0.008)),
        origin=Origin(xyz=(0.0, rear_y, 0.009)),
        material=body_black,
        name="elem_rear_lower_bar",
    )
    body.visual(
        Box((front_rear_span, wall_thickness, 0.006)),
        origin=Origin(xyz=(0.0, rear_y, 0.020)),
        material=body_black,
        name="elem_rear_upper_bar",
    )
    rear_jamb_width = (front_rear_span - slot_width) * 0.5
    jamb_center_x = slot_width * 0.5 + rear_jamb_width * 0.5
    body.visual(
        Box((rear_jamb_width, wall_thickness, slot_height)),
        origin=Origin(xyz=(-jamb_center_x, rear_y, 0.015)),
        material=body_black,
        name="elem_rear_left_jamb",
    )
    body.visual(
        Box((rear_jamb_width, wall_thickness, slot_height)),
        origin=Origin(xyz=(jamb_center_x, rear_y, 0.015)),
        material=body_black,
        name="elem_rear_right_jamb",
    )

    body.visual(
        Cylinder(radius=0.0045, length=0.014),
        origin=Origin(
            xyz=(-hinge_x, hinge_y, hinge_z),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=charcoal,
        name="elem_left_hinge_boss",
    )
    body.visual(
        Cylinder(radius=0.0045, length=0.014),
        origin=Origin(
            xyz=(hinge_x, hinge_y, hinge_z),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=charcoal,
        name="elem_right_hinge_boss",
    )

    for foot_index, foot_x in enumerate((-0.060, -0.020, 0.020, 0.060), start=1):
        body.visual(
            Cylinder(radius=0.006, length=0.0015),
            origin=Origin(
                xyz=(foot_x, 0.037 if foot_index <= 2 else -0.037, 0.00075),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=rubber,
            name=f"elem_foot_{foot_index}",
        )

    def add_antenna(
        part_name: str,
        side_sign: float,
        *,
        barrel_name: str,
        stem_name: str,
        paddle_name: str,
    ) -> None:
        antenna = model.part(part_name)
        antenna.inertial = Inertial.from_geometry(
            Box((0.018, 0.010, 0.132)),
            mass=0.06,
            origin=Origin(xyz=(0.010 * side_sign, 0.0, 0.066)),
        )
        antenna.visual(
            Cylinder(radius=0.0038, length=0.010),
            origin=Origin(
                xyz=(0.0038 * side_sign, 0.0, 0.0),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=charcoal,
            name=barrel_name,
        )
        antenna.visual(
            Box((0.006, 0.006, 0.018)),
            origin=Origin(xyz=(0.007 * side_sign, 0.0, 0.009)),
            material=charcoal,
            name=stem_name,
        )
        antenna.visual(
            Box((0.012, 0.007, 0.118)),
            origin=Origin(xyz=(0.010 * side_sign, 0.0, 0.068)),
            material=charcoal,
            name=paddle_name,
        )

    add_antenna(
        "left_antenna",
        -1.0,
        barrel_name="elem_left_barrel",
        stem_name="elem_left_stem",
        paddle_name="elem_left_paddle",
    )
    add_antenna(
        "right_antenna",
        1.0,
        barrel_name="elem_right_barrel",
        stem_name="elem_right_stem",
        paddle_name="elem_right_paddle",
    )

    wireless_switch = model.part("wireless_switch")
    wireless_switch.inertial = Inertial.from_geometry(
        Box((0.010, 0.010, 0.006)),
        mass=0.015,
        origin=Origin(xyz=(0.0, -0.003, 0.003)),
    )
    wireless_switch.visual(
        Box((0.008, 0.003, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=switch_gray,
        name="elem_switch_tab",
    )
    wireless_switch.visual(
        Box((0.004, 0.004, 0.004)),
        origin=Origin(xyz=(0.0, -0.0025, 0.002)),
        material=switch_gray,
        name="elem_switch_neck",
    )
    wireless_switch.visual(
        Box((0.006, 0.008, 0.003)),
        origin=Origin(xyz=(0.0, -0.007, 0.0015)),
        material=charcoal,
        name="elem_switch_guide",
    )

    model.articulation(
        "body_to_left_antenna",
        ArticulationType.REVOLUTE,
        parent=body,
        child="left_antenna",
        origin=Origin(xyz=(-hinge_x, hinge_y, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.2,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )
    model.articulation(
        "body_to_right_antenna",
        ArticulationType.REVOLUTE,
        parent=body,
        child="right_antenna",
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.2,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )
    model.articulation(
        "body_to_wireless_switch",
        ArticulationType.PRISMATIC,
        parent=body,
        child=wireless_switch,
        origin=Origin(xyz=(-slot_travel * 0.5, rear_y, 0.013)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=0.05,
            lower=0.0,
            upper=slot_travel,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    slot_travel = 0.012
    body = object_model.get_part("body")
    left_antenna = object_model.get_part("left_antenna")
    right_antenna = object_model.get_part("right_antenna")
    wireless_switch = object_model.get_part("wireless_switch")
    left_hinge = object_model.get_articulation("body_to_left_antenna")
    right_hinge = object_model.get_articulation("body_to_right_antenna")
    switch_slide = object_model.get_articulation("body_to_wireless_switch")

    ctx.expect_contact(
        body,
        left_antenna,
        elem_a="elem_left_hinge_boss",
        elem_b="elem_left_barrel",
        contact_tol=0.001,
        name="left antenna stays mounted on left hinge boss",
    )
    ctx.expect_contact(
        body,
        right_antenna,
        elem_a="elem_right_hinge_boss",
        elem_b="elem_right_barrel",
        contact_tol=0.001,
        name="right antenna stays mounted on right hinge boss",
    )
    ctx.expect_gap(
        body,
        wireless_switch,
        axis="z",
        positive_elem="elem_rear_upper_bar",
        negative_elem="elem_switch_tab",
        min_gap=0.0,
        max_gap=0.0015,
        name="switch tab stays just below the upper slot guide",
    )
    ctx.expect_gap(
        wireless_switch,
        body,
        axis="z",
        positive_elem="elem_switch_tab",
        negative_elem="elem_rear_lower_bar",
        min_gap=0.0,
        max_gap=0.0015,
        name="switch tab stays just above the lower slot guide",
    )

    left_closed = ctx.part_element_world_aabb(left_antenna, elem="elem_left_paddle")
    right_closed = ctx.part_element_world_aabb(right_antenna, elem="elem_right_paddle")
    switch_rest = ctx.part_world_position(wireless_switch)
    with ctx.pose({left_hinge: math.radians(60.0), right_hinge: math.radians(60.0), switch_slide: slot_travel}):
        left_open = ctx.part_element_world_aabb(left_antenna, elem="elem_left_paddle")
        right_open = ctx.part_element_world_aabb(right_antenna, elem="elem_right_paddle")
        switch_extended = ctx.part_world_position(wireless_switch)
        ctx.expect_gap(
            body,
            wireless_switch,
            axis="z",
            positive_elem="elem_rear_upper_bar",
            negative_elem="elem_switch_tab",
            min_gap=0.0,
            max_gap=0.0015,
            name="switch tab remains guided at full travel",
        )
        ctx.expect_gap(
            wireless_switch,
            body,
            axis="z",
            positive_elem="elem_switch_tab",
            negative_elem="elem_rear_lower_bar",
            min_gap=0.0,
            max_gap=0.0015,
            name="switch tab remains supported by the lower slot guide at full travel",
        )

    ctx.check(
        "left antenna swings farther outward when opened",
        left_closed is not None
        and left_open is not None
        and left_open[0][0] < left_closed[0][0] - 0.025,
        details=f"closed={left_closed}, open={left_open}",
    )
    ctx.check(
        "right antenna swings farther outward when opened",
        right_closed is not None
        and right_open is not None
        and right_open[1][0] > right_closed[1][0] + 0.025,
        details=f"closed={right_closed}, open={right_open}",
    )
    ctx.check(
        "wireless switch slides rightward along the rear slot",
        switch_rest is not None
        and switch_extended is not None
        and switch_extended[0] > switch_rest[0] + 0.010,
        details=f"rest={switch_rest}, extended={switch_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
