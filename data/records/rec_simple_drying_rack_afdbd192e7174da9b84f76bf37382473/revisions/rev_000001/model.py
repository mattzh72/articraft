from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan, pi

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
    model = ArticulatedObject(name="fold_out_drying_rack")

    painted_steel = model.material("painted_steel", rgba=(0.84, 0.86, 0.88, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.17, 0.18, 0.19, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.09, 0.10, 1.0))

    frame_radius = 0.015
    rail_radius = 0.009
    central_half_width = 0.38
    central_height = 0.88
    support_half_width = 0.33
    support_back = 0.48
    support_drop = 0.84
    wing_width = 0.24
    wing_height = 0.46
    wing_deploy = 0.92
    leg_length = (support_back**2 + support_drop**2) ** 0.5
    leg_tilt = -atan(support_back / support_drop)

    central_frame = model.part("central_frame")
    central_frame.inertial = Inertial.from_geometry(
        Box((0.94, 0.30, 0.95)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, 0.47)),
    )
    central_frame.visual(
        Cylinder(radius=frame_radius, length=central_height),
        origin=Origin(xyz=(-central_half_width, 0.0, central_height * 0.5)),
        material=painted_steel,
        name="left_post",
    )
    central_frame.visual(
        Cylinder(radius=frame_radius, length=central_height),
        origin=Origin(xyz=(central_half_width, 0.0, central_height * 0.5)),
        material=painted_steel,
        name="right_post",
    )
    central_frame.visual(
        Cylinder(radius=frame_radius, length=central_half_width * 2.0),
        origin=Origin(xyz=(0.0, 0.0, central_height), rpy=(0.0, 1.57079632679, 0.0)),
        material=painted_steel,
        name="top_rail",
    )
    central_frame.visual(
        Cylinder(radius=frame_radius, length=central_half_width * 2.0),
        origin=Origin(xyz=(0.0, 0.0, 0.10), rpy=(0.0, 1.57079632679, 0.0)),
        material=painted_steel,
        name="bottom_rail",
    )
    for index, z in enumerate((0.21, 0.33, 0.45, 0.57, 0.69, 0.81)):
        central_frame.visual(
            Cylinder(radius=rail_radius, length=central_half_width * 2.0 - 0.01),
            origin=Origin(xyz=(0.0, 0.0, z), rpy=(0.0, 1.57079632679, 0.0)),
            material=painted_steel,
            name=f"central_rail_{index}",
        )
    central_frame.visual(
        Box((0.28, 0.09, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, central_height + 0.02)),
        material=dark_plastic,
        name="top_bracket",
    )
    central_frame.visual(
        Box((0.06, 0.045, 0.08)),
        origin=Origin(xyz=(-0.29, -0.0225, central_height - 0.04)),
        material=dark_plastic,
        name="left_support_hinge_cheek",
    )
    central_frame.visual(
        Box((0.06, 0.045, 0.08)),
        origin=Origin(xyz=(0.29, -0.0225, central_height - 0.04)),
        material=dark_plastic,
        name="right_support_hinge_cheek",
    )
    central_frame.visual(
        Box((0.03, 0.07, 0.24)),
        origin=Origin(xyz=(-central_half_width, 0.0, 0.77)),
        material=dark_plastic,
        name="left_wing_mount",
    )
    central_frame.visual(
        Box((0.03, 0.07, 0.24)),
        origin=Origin(xyz=(central_half_width, 0.0, 0.77)),
        material=dark_plastic,
        name="right_wing_mount",
    )
    central_frame.visual(
        Box((0.10, 0.28, 0.035)),
        origin=Origin(xyz=(-0.33, 0.02, 0.0175)),
        material=dark_plastic,
        name="left_foot",
    )
    central_frame.visual(
        Box((0.10, 0.28, 0.035)),
        origin=Origin(xyz=(0.33, 0.02, 0.0175)),
        material=dark_plastic,
        name="right_foot",
    )
    central_frame.visual(
        Box((0.11, 0.28, 0.008)),
        origin=Origin(xyz=(-0.33, 0.02, 0.004)),
        material=rubber,
        name="left_pad",
    )
    central_frame.visual(
        Box((0.11, 0.28, 0.008)),
        origin=Origin(xyz=(0.33, 0.02, 0.004)),
        material=rubber,
        name="right_pad",
    )

    lower_support = model.part("lower_support")
    lower_support.inertial = Inertial.from_geometry(
        Box((0.80, 0.58, 0.88)),
        mass=3.8,
        origin=Origin(xyz=(0.0, -0.24, -0.42)),
    )
    lower_support.visual(
        Cylinder(radius=frame_radius, length=support_half_width * 2.0),
        origin=Origin(rpy=(0.0, 1.57079632679, 0.0)),
        material=painted_steel,
        name="support_top_bar",
    )
    lower_support.visual(
        Cylinder(radius=frame_radius, length=leg_length),
        origin=Origin(xyz=(-support_half_width, -support_back * 0.5, -support_drop * 0.5), rpy=(leg_tilt, 0.0, 0.0)),
        material=painted_steel,
        name="left_support_leg",
    )
    lower_support.visual(
        Cylinder(radius=frame_radius, length=leg_length),
        origin=Origin(xyz=(support_half_width, -support_back * 0.5, -support_drop * 0.5), rpy=(leg_tilt, 0.0, 0.0)),
        material=painted_steel,
        name="right_support_leg",
    )
    for index, fraction in enumerate((0.28, 0.52, 0.76)):
        lower_support.visual(
            Cylinder(radius=rail_radius, length=support_half_width * 2.0),
            origin=Origin(
                xyz=(0.0, -support_back * fraction, -support_drop * fraction),
                rpy=(0.0, 1.57079632679, 0.0),
            ),
            material=painted_steel,
            name=f"support_rail_{index}",
        )
    lower_support.visual(
        Box((0.17, 0.12, 0.045)),
        origin=Origin(xyz=(-support_half_width, -support_back, -support_drop + 0.0225)),
        material=dark_plastic,
        name="left_rear_foot",
    )
    lower_support.visual(
        Box((0.17, 0.12, 0.045)),
        origin=Origin(xyz=(support_half_width, -support_back, -support_drop + 0.0225)),
        material=dark_plastic,
        name="right_rear_foot",
    )
    lower_support.visual(
        Box((0.18, 0.13, 0.008)),
        origin=Origin(xyz=(-support_half_width, -support_back, -support_drop + 0.004)),
        material=rubber,
        name="left_rear_pad",
    )
    lower_support.visual(
        Box((0.18, 0.13, 0.008)),
        origin=Origin(xyz=(support_half_width, -support_back, -support_drop + 0.004)),
        material=rubber,
        name="right_rear_pad",
    )

    left_wing = model.part("left_wing")
    left_wing.inertial = Inertial.from_geometry(
        Box((wing_width + 0.03, 0.08, wing_height + 0.02)),
        mass=1.4,
        origin=Origin(xyz=(-0.5 * (wing_width + 0.03), 0.0, -0.5 * wing_height)),
    )
    left_wing.visual(
        Box((0.03, 0.07, wing_height)),
        origin=Origin(xyz=(-0.015, 0.0, -0.5 * wing_height)),
        material=dark_plastic,
        name="left_inner_stile",
    )
    left_wing.visual(
        Cylinder(radius=frame_radius, length=wing_height),
        origin=Origin(xyz=(-wing_width, 0.0, -0.5 * wing_height)),
        material=painted_steel,
        name="left_outer_stile",
    )
    left_wing.visual(
        Cylinder(radius=frame_radius, length=wing_width - 0.03),
        origin=Origin(
            xyz=(-0.5 * (wing_width + 0.03), 0.0, 0.0),
            rpy=(0.0, pi * 0.5, 0.0),
        ),
        material=painted_steel,
        name="left_top_rail",
    )
    left_wing.visual(
        Cylinder(radius=frame_radius, length=wing_width - 0.03),
        origin=Origin(
            xyz=(-0.5 * (wing_width + 0.03), 0.0, -wing_height),
            rpy=(0.0, pi * 0.5, 0.0),
        ),
        material=painted_steel,
        name="left_bottom_rail",
    )
    for index, z in enumerate((-0.11, -0.22, -0.33)):
        left_wing.visual(
            Cylinder(radius=rail_radius, length=wing_width - 0.03),
            origin=Origin(
                xyz=(-0.5 * (wing_width + 0.03), 0.0, z),
                rpy=(0.0, pi * 0.5, 0.0),
            ),
            material=painted_steel,
            name=f"left_wing_rail_{index}",
        )

    right_wing = model.part("right_wing")
    right_wing.inertial = Inertial.from_geometry(
        Box((wing_width + 0.03, 0.08, wing_height + 0.02)),
        mass=1.4,
        origin=Origin(xyz=(0.5 * (wing_width + 0.03), 0.0, -0.5 * wing_height)),
    )
    right_wing.visual(
        Box((0.03, 0.07, wing_height)),
        origin=Origin(xyz=(0.015, 0.0, -0.5 * wing_height)),
        material=dark_plastic,
        name="right_inner_stile",
    )
    right_wing.visual(
        Cylinder(radius=frame_radius, length=wing_height),
        origin=Origin(xyz=(wing_width, 0.0, -0.5 * wing_height)),
        material=painted_steel,
        name="right_outer_stile",
    )
    right_wing.visual(
        Cylinder(radius=frame_radius, length=wing_width - 0.03),
        origin=Origin(
            xyz=(0.5 * (wing_width + 0.03), 0.0, 0.0),
            rpy=(0.0, pi * 0.5, 0.0),
        ),
        material=painted_steel,
        name="right_top_rail",
    )
    right_wing.visual(
        Cylinder(radius=frame_radius, length=wing_width - 0.03),
        origin=Origin(
            xyz=(0.5 * (wing_width + 0.03), 0.0, -wing_height),
            rpy=(0.0, pi * 0.5, 0.0),
        ),
        material=painted_steel,
        name="right_bottom_rail",
    )
    for index, z in enumerate((-0.11, -0.22, -0.33)):
        right_wing.visual(
            Cylinder(radius=rail_radius, length=wing_width - 0.03),
            origin=Origin(
                xyz=(0.5 * (wing_width + 0.03), 0.0, z),
                rpy=(0.0, pi * 0.5, 0.0),
            ),
            material=painted_steel,
            name=f"right_wing_rail_{index}",
        )

    model.articulation(
        "support_hinge",
        ArticulationType.REVOLUTE,
        parent=central_frame,
        child=lower_support,
        origin=Origin(xyz=(0.0, -0.06, central_height - 0.04)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.5, lower=0.0, upper=1.10),
    )
    model.articulation(
        "left_wing_hinge",
        ArticulationType.REVOLUTE,
        parent=central_frame,
        child=left_wing,
        origin=Origin(
            xyz=(-(central_half_width + frame_radius), 0.0, central_height - 0.02),
            rpy=(0.0, wing_deploy, 0.0),
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.6, lower=-1.10, upper=0.25),
    )
    model.articulation(
        "right_wing_hinge",
        ArticulationType.REVOLUTE,
        parent=central_frame,
        child=right_wing,
        origin=Origin(
            xyz=((central_half_width + frame_radius), 0.0, central_height - 0.02),
            rpy=(0.0, -wing_deploy, 0.0),
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.6, lower=-0.25, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    central_frame = object_model.get_part("central_frame")
    lower_support = object_model.get_part("lower_support")
    left_wing = object_model.get_part("left_wing")
    right_wing = object_model.get_part("right_wing")

    support_hinge = object_model.get_articulation("support_hinge")
    left_wing_hinge = object_model.get_articulation("left_wing_hinge")
    right_wing_hinge = object_model.get_articulation("right_wing_hinge")

    ctx.expect_contact(
        lower_support,
        central_frame,
        elem_a="support_top_bar",
        elem_b="left_support_hinge_cheek",
        name="lower support top bar bears on left hinge cheek",
    )
    ctx.expect_contact(
        lower_support,
        central_frame,
        elem_a="support_top_bar",
        elem_b="right_support_hinge_cheek",
        name="lower support top bar bears on right hinge cheek",
    )
    ctx.expect_contact(
        left_wing,
        central_frame,
        elem_a="left_inner_stile",
        elem_b="left_wing_mount",
        name="left wing hinge strap seats on left mount",
    )
    ctx.expect_contact(
        right_wing,
        central_frame,
        elem_a="right_inner_stile",
        elem_b="right_wing_mount",
        name="right wing hinge strap seats on right mount",
    )

    front_pad_aabb = ctx.part_element_world_aabb(central_frame, elem="left_pad")
    rear_pad_aabb = ctx.part_element_world_aabb(lower_support, elem="left_rear_pad")
    rear_pad_bottom = None if rear_pad_aabb is None else rear_pad_aabb[0][2]
    front_pad_bottom = None if front_pad_aabb is None else front_pad_aabb[0][2]
    ctx.check(
        "front and rear feet share the same floor plane at rest",
        front_pad_bottom is not None
        and rear_pad_bottom is not None
        and abs(front_pad_bottom - rear_pad_bottom) <= 0.001,
        details=f"front_pad_bottom={front_pad_bottom}, rear_pad_bottom={rear_pad_bottom}",
    )

    left_outer_rest = ctx.part_element_world_aabb(left_wing, elem="left_outer_stile")
    right_outer_rest = ctx.part_element_world_aabb(right_wing, elem="right_outer_stile")
    left_rest_top = None if left_outer_rest is None else left_outer_rest[1][2]
    right_rest_top = None if right_outer_rest is None else right_outer_rest[1][2]

    with ctx.pose({support_hinge: 1.0, left_wing_hinge: -1.0, right_wing_hinge: 1.0}):
        folded_rear_pad_aabb = ctx.part_element_world_aabb(lower_support, elem="left_rear_pad")
        folded_left_outer = ctx.part_element_world_aabb(left_wing, elem="left_outer_stile")
        folded_right_outer = ctx.part_element_world_aabb(right_wing, elem="right_outer_stile")

        folded_rear_pad_bottom = None if folded_rear_pad_aabb is None else folded_rear_pad_aabb[0][2]
        folded_left_top = None if folded_left_outer is None else folded_left_outer[1][2]
        folded_right_top = None if folded_right_outer is None else folded_right_outer[1][2]

        ctx.check(
            "lower support folds up clear of the floor",
            folded_rear_pad_bottom is not None and folded_rear_pad_bottom >= 0.70,
            details=f"folded_rear_pad_bottom={folded_rear_pad_bottom}",
        )
        ctx.check(
            "left wing folds down from the raised drying position",
            left_rest_top is not None
            and folded_left_top is not None
            and folded_left_top <= left_rest_top - 0.18,
            details=f"left_rest_top={left_rest_top}, folded_left_top={folded_left_top}",
        )
        ctx.check(
            "right wing folds down from the raised drying position",
            right_rest_top is not None
            and folded_right_top is not None
            and folded_right_top <= right_rest_top - 0.18,
            details=f"right_rest_top={right_rest_top}, folded_right_top={folded_right_top}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
