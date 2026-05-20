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
    model = ArticulatedObject(name="compact_pedal_bin")

    shell_finish = model.material("shell_finish", rgba=(0.78, 0.80, 0.82, 1.0))
    pedal_finish = model.material("pedal_finish", rgba=(0.13, 0.13, 0.14, 1.0))
    hinge_finish = model.material("hinge_finish", rgba=(0.36, 0.37, 0.39, 1.0))

    width = 0.24
    depth = 0.20
    height = 0.36
    wall = 0.004
    floor_thickness = 0.006
    eps = 0.001

    body = model.part("body")

    front_y = -depth / 2.0 + wall / 2.0
    rear_y = depth / 2.0 - wall / 2.0
    side_x = width / 2.0 - wall / 2.0

    opening_center_y = -0.005
    opening_width_y = 0.110
    opening_center_z = 0.122
    opening_height_z = 0.135
    opening_y0 = opening_center_y - opening_width_y / 2.0
    opening_y1 = opening_center_y + opening_width_y / 2.0
    opening_z0 = opening_center_z - opening_height_z / 2.0
    opening_z1 = opening_center_z + opening_height_z / 2.0

    right_wall_y_min = -depth / 2.0 + wall - eps
    right_wall_y_max = depth / 2.0 - wall + eps

    body.visual(
        Box((width, wall, height)),
        origin=Origin(xyz=(0.0, front_y, height / 2.0)),
        material=shell_finish,
        name="front_wall",
    )
    body.visual(
        Box((width, wall, height)),
        origin=Origin(xyz=(0.0, rear_y, height / 2.0)),
        material=shell_finish,
        name="rear_wall",
    )
    body.visual(
        Box((wall, depth - 2.0 * wall + 2.0 * eps, height)),
        origin=Origin(xyz=(-side_x, 0.0, height / 2.0)),
        material=shell_finish,
        name="left_wall",
    )

    front_strip_len = (opening_y0 - right_wall_y_min) + eps
    rear_strip_len = right_wall_y_max - (opening_y1 - eps)
    mid_strip_len = opening_width_y + 2.0 * eps
    bottom_strip_height = opening_z0 + eps
    top_strip_height = height - (opening_z1 - eps)

    body.visual(
        Box((wall, front_strip_len, height)),
        origin=Origin(
            xyz=(side_x, right_wall_y_min + front_strip_len / 2.0, height / 2.0)
        ),
        material=shell_finish,
        name="right_wall_front_strip",
    )
    body.visual(
        Box((wall, rear_strip_len, height)),
        origin=Origin(
            xyz=(side_x, right_wall_y_max - rear_strip_len / 2.0, height / 2.0)
        ),
        material=shell_finish,
        name="right_wall_rear_strip",
    )
    body.visual(
        Box((wall, mid_strip_len, bottom_strip_height)),
        origin=Origin(
            xyz=(side_x, opening_center_y, bottom_strip_height / 2.0)
        ),
        material=shell_finish,
        name="right_wall_bottom_strip",
    )
    body.visual(
        Box((wall, mid_strip_len, top_strip_height)),
        origin=Origin(
            xyz=(
                side_x,
                opening_center_y,
                height - top_strip_height / 2.0,
            )
        ),
        material=shell_finish,
        name="right_wall_top_strip",
    )
    body.visual(
        Box(
            (
                width - 2.0 * wall + 2.0 * eps,
                depth - 2.0 * wall + 2.0 * eps,
                floor_thickness,
            )
        ),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                floor_thickness / 2.0,
            )
        ),
        material=shell_finish,
        name="floor_pan",
    )

    lid_axis_y = depth / 2.0 + 0.006
    lid_axis_z = height + 0.006
    lid_barrel_radius = 0.005
    lid_body_barrel_len = 0.038
    for x_pos, name in ((-0.075, "lid_body_hinge_left"), (0.075, "lid_body_hinge_right")):
        body.visual(
            Box((0.018, 0.010, 0.018)),
            origin=Origin(xyz=(x_pos, depth / 2.0 + 0.002, lid_axis_z - 0.014)),
            material=hinge_finish,
            name=f"{name}_bridge",
        )
        body.visual(
            Cylinder(radius=lid_barrel_radius, length=lid_body_barrel_len),
            origin=Origin(
                xyz=(x_pos, lid_axis_y, lid_axis_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=hinge_finish,
            name=name,
        )

    hatch_axis_x = width / 2.0 + 0.001
    hatch_barrel_radius = 0.004
    hatch_body_barrel_len = 0.025
    hatch_axis_y = opening_center_y + 0.047
    for z_pos, name in (
        (opening_center_z - 0.052, "hatch_body_hinge_lower"),
        (opening_center_z + 0.052, "hatch_body_hinge_upper"),
    ):
        body.visual(
            Cylinder(radius=hatch_barrel_radius, length=hatch_body_barrel_len),
            origin=Origin(
                xyz=(hatch_axis_x, hatch_axis_y, z_pos),
            ),
            material=hinge_finish,
            name=name,
        )
    body.visual(
        Box((0.008, 0.012, 0.015)),
        origin=Origin(xyz=(width / 2.0 - 0.002, hatch_axis_y, 0.050)),
        material=hinge_finish,
        name="hatch_hinge_lower_bridge",
    )
    body.visual(
        Box((0.008, 0.012, 0.013)),
        origin=Origin(xyz=(width / 2.0 - 0.002, hatch_axis_y, 0.193)),
        material=hinge_finish,
        name="hatch_hinge_upper_bridge",
    )

    pedal_axis_y = -depth / 2.0 - 0.011
    pedal_axis_z = 0.052
    body.visual(
        Box((0.020, 0.018, 0.022)),
        origin=Origin(xyz=(-0.068, -0.102, pedal_axis_z)),
        material=hinge_finish,
        name="pedal_bracket_left",
    )
    body.visual(
        Box((0.020, 0.018, 0.022)),
        origin=Origin(xyz=(0.068, -0.102, pedal_axis_z)),
        material=hinge_finish,
        name="pedal_bracket_right",
    )
    for x_pos, name in ((-0.068, "pedal_body_pivot_left"), (0.068, "pedal_body_pivot_right")):
        body.visual(
            Cylinder(radius=0.006, length=0.024),
            origin=Origin(
                xyz=(x_pos, pedal_axis_y, pedal_axis_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=hinge_finish,
            name=name,
        )

    body.inertial = Inertial.from_geometry(
        Box((width, depth, height)),
        mass=5.2,
        origin=Origin(xyz=(0.0, 0.0, height / 2.0)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.244, 0.190, 0.010)),
        origin=Origin(xyz=(0.0, -0.118, 0.003)),
        material=shell_finish,
        name="lid_panel",
    )
    lid.visual(
        Box((0.232, 0.004, 0.022)),
        origin=Origin(xyz=(0.0, -0.215, -0.010)),
        material=shell_finish,
        name="lid_front_hem",
    )
    lid.visual(
        Box((0.180, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, -0.018, -0.004)),
        material=hinge_finish,
        name="lid_hinge_leaf",
    )
    lid.visual(
        Box((0.112, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, -0.009, -0.002)),
        material=hinge_finish,
        name="lid_hinge_web",
    )
    lid.visual(
        Cylinder(radius=lid_barrel_radius, length=0.112),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_finish,
        name="lid_center_barrel",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.244, 0.210, 0.034)),
        mass=0.7,
        origin=Origin(xyz=(0.0, -0.100, -0.004)),
    )

    hatch = model.part("side_hatch")
    hatch.visual(
        Box((0.0035, 0.094, 0.123)),
        origin=Origin(xyz=(-0.00175, -0.047, 0.0)),
        material=shell_finish,
        name="hatch_panel",
    )
    hatch.visual(
        Cylinder(radius=hatch_barrel_radius, length=0.077),
        origin=Origin(),
        material=hinge_finish,
        name="hatch_center_barrel",
    )
    hatch.inertial = Inertial.from_geometry(
        Box((0.012, 0.102, 0.130)),
        mass=0.22,
        origin=Origin(xyz=(-0.002, -0.047, 0.0)),
    )

    pedal = model.part("pedal")
    pedal.visual(
        Cylinder(radius=0.006, length=0.112),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_finish,
        name="pedal_pivot_barrel",
    )
    pedal.visual(
        Box((0.018, 0.070, 0.022)),
        origin=Origin(xyz=(-0.045, -0.040, -0.010)),
        material=pedal_finish,
        name="pedal_left_arm",
    )
    pedal.visual(
        Box((0.018, 0.070, 0.022)),
        origin=Origin(xyz=(0.045, -0.040, -0.010)),
        material=pedal_finish,
        name="pedal_right_arm",
    )
    pedal.visual(
        Box((0.170, 0.055, 0.012)),
        origin=Origin(xyz=(0.0, -0.060, -0.025)),
        material=pedal_finish,
        name="pedal_tread",
    )
    pedal.visual(
        Box((0.110, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, -0.030, -0.010)),
        material=pedal_finish,
        name="pedal_bridge",
    )
    pedal.inertial = Inertial.from_geometry(
        Box((0.170, 0.090, 0.040)),
        mass=0.35,
        origin=Origin(xyz=(0.0, -0.050, -0.015)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, lid_axis_y, lid_axis_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(82.0),
        ),
    )
    model.articulation(
        "body_to_side_hatch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hatch,
        origin=Origin(xyz=(hatch_axis_x, hatch_axis_y, opening_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )
    model.articulation(
        "body_to_pedal",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pedal,
        origin=Origin(xyz=(0.0, pedal_axis_y, pedal_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=3.0,
            lower=0.0,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hatch = object_model.get_part("side_hatch")
    pedal = object_model.get_part("pedal")

    lid_joint = object_model.get_articulation("body_to_lid")
    hatch_joint = object_model.get_articulation("body_to_side_hatch")
    pedal_joint = object_model.get_articulation("body_to_pedal")

    lid_limits = lid_joint.motion_limits
    hatch_limits = hatch_joint.motion_limits
    pedal_limits = pedal_joint.motion_limits

    ctx.check(
        "primary articulations use intended axes",
        lid_joint.axis == (-1.0, 0.0, 0.0)
        and hatch_joint.axis == (0.0, 0.0, 1.0)
        and pedal_joint.axis == (1.0, 0.0, 0.0),
        details=(
            f"lid={lid_joint.axis}, hatch={hatch_joint.axis}, pedal={pedal_joint.axis}"
        ),
    )
    ctx.check(
        "motion limits describe a pedal bin mechanism",
        lid_limits is not None
        and hatch_limits is not None
        and pedal_limits is not None
        and lid_limits.lower == 0.0
        and hatch_limits.lower == 0.0
        and pedal_limits.lower == 0.0
        and lid_limits.upper is not None
        and lid_limits.upper > 1.2
        and hatch_limits.upper is not None
        and hatch_limits.upper > 1.0
        and pedal_limits.upper is not None
        and pedal_limits.upper >= 0.5,
        details=(
            f"lid={lid_limits}, hatch={hatch_limits}, pedal={pedal_limits}"
        ),
    )

    with ctx.pose({lid_joint: 0.0, hatch_joint: 0.0, pedal_joint: 0.0}):
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_panel",
            min_overlap=0.18,
            name="lid panel covers the top opening footprint",
        )
        ctx.expect_overlap(
            hatch,
            body,
            axes="yz",
            elem_a="hatch_panel",
            min_overlap=0.09,
            name="service hatch sits over the side opening footprint",
        )
        closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
        closed_hatch_aabb = ctx.part_element_world_aabb(hatch, elem="hatch_panel")
        closed_pedal_aabb = ctx.part_element_world_aabb(pedal, elem="pedal_tread")

    with ctx.pose({lid_joint: lid_limits.upper}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")

    with ctx.pose({hatch_joint: hatch_limits.upper}):
        open_hatch_aabb = ctx.part_element_world_aabb(hatch, elem="hatch_panel")

    with ctx.pose({pedal_joint: pedal_limits.upper}):
        depressed_pedal_aabb = ctx.part_element_world_aabb(pedal, elem="pedal_tread")

    ctx.check(
        "lid rises when opened",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.08,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )
    ctx.check(
        "side hatch swings outward",
        closed_hatch_aabb is not None
        and open_hatch_aabb is not None
        and open_hatch_aabb[1][0] > closed_hatch_aabb[1][0] + 0.045,
        details=f"closed={closed_hatch_aabb}, open={open_hatch_aabb}",
    )
    ctx.check(
        "pedal drops when pressed",
        closed_pedal_aabb is not None
        and depressed_pedal_aabb is not None
        and depressed_pedal_aabb[0][2] < closed_pedal_aabb[0][2] - 0.01,
        details=f"closed={closed_pedal_aabb}, pressed={depressed_pedal_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
