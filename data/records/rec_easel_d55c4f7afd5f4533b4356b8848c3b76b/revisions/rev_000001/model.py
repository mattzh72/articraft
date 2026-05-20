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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tabletop_mini_easel")

    wood = model.material("wood", rgba=(0.74, 0.58, 0.37, 1.0))
    wood_dark = model.material("wood_dark", rgba=(0.60, 0.45, 0.28, 1.0))
    steel = model.material("steel", rgba=(0.76, 0.78, 0.80, 1.0))
    clip_black = model.material("clip_black", rgba=(0.12, 0.12, 0.12, 1.0))
    rubber = model.material("rubber", rgba=(0.16, 0.14, 0.12, 1.0))

    apex_height = 0.338
    front_foot_x = 0.104
    front_leg_drop = apex_height
    front_leg_length = math.hypot(front_foot_x, front_leg_drop)
    leg_angle = math.atan2(front_foot_x, front_leg_drop)
    leg_side_offset = 0.007

    shelf_center = (0.0, 0.004, 0.072)
    shelf_width = 0.128
    shelf_depth = 0.016
    shelf_thickness = 0.010

    rail_center = (0.0, 0.023, 0.192)
    rail_width = 0.018
    rail_thickness = 0.008
    rail_height = 0.230
    slot_width = 0.0052
    slot_height = 0.140
    rail_web_width = (rail_width - slot_width) / 2.0
    rail_top_block = 0.045
    rail_bottom_block = rail_height - slot_height - rail_top_block
    slot_top_from_center = (rail_height / 2.0) - rail_top_block

    left_leg = model.part("left_front_leg")
    left_leg.visual(
        Box((0.018, 0.010, front_leg_length)),
        origin=Origin(
            xyz=(-front_foot_x / 2.0, -leg_side_offset, apex_height / 2.0),
            rpy=(0.0, leg_angle, 0.0),
        ),
        material=wood,
        name="left_leg_bar",
    )
    left_leg.visual(
        Box((0.012, 0.012, 0.030)),
        origin=Origin(xyz=(-0.070, shelf_center[1], shelf_center[2])),
        material=wood_dark,
        name="left_shelf_cleat",
    )
    left_leg.visual(
        Box((0.018, 0.006, 0.024)),
        origin=Origin(xyz=(-0.004, -0.006, apex_height - 0.012)),
        material=steel,
        name="left_hinge_leaf",
    )
    left_leg.visual(
        Cylinder(radius=0.0045, length=0.008),
        origin=Origin(
            xyz=(-0.0015, -0.010, apex_height - 0.004),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="left_hinge_barrel",
    )
    left_leg.inertial = Inertial.from_geometry(
        Box((0.140, 0.030, 0.360)),
        mass=0.20,
        origin=Origin(xyz=(-0.040, 0.0, 0.170)),
    )

    right_leg = model.part("right_front_leg")
    right_leg.visual(
        Box((0.018, 0.010, front_leg_length)),
        origin=Origin(
            xyz=(front_foot_x / 2.0, leg_side_offset, -apex_height / 2.0),
            rpy=(0.0, -leg_angle, 0.0),
        ),
        material=wood,
        name="right_leg_bar",
    )
    right_leg.visual(
        Box((0.012, 0.012, 0.030)),
        origin=Origin(xyz=(0.070, shelf_center[1], shelf_center[2] - apex_height)),
        material=wood_dark,
        name="right_shelf_cleat",
    )
    right_leg.visual(
        Box((0.018, 0.006, 0.024)),
        origin=Origin(xyz=(0.004, 0.006, -0.012)),
        material=steel,
        name="right_hinge_leaf",
    )
    right_leg.visual(
        Cylinder(radius=0.0045, length=0.012),
        origin=Origin(
            xyz=(0.0015, 0.0, -0.004),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="right_hinge_barrel",
    )
    right_leg.inertial = Inertial.from_geometry(
        Box((0.140, 0.030, 0.360)),
        mass=0.18,
        origin=Origin(xyz=(0.052, 0.0, -0.170)),
    )

    model.articulation(
        "apex_hinge",
        ArticulationType.REVOLUTE,
        parent=left_leg,
        child=right_leg,
        origin=Origin(xyz=(0.0, 0.0, apex_height)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=1.0,
            lower=-0.22,
            upper=0.24,
        ),
    )

    shelf = model.part("shelf")
    shelf.visual(
        Box((shelf_width, shelf_depth, shelf_thickness)),
        material=wood,
        name="shelf_board",
    )
    shelf.visual(
        Box((shelf_width, 0.007, 0.016)),
        origin=Origin(xyz=(0.0, 0.0115, 0.013)),
        material=wood_dark,
        name="shelf_lip",
    )
    for x_center, name in ((-0.012, "rear_hinge_barrel_left"), (0.012, "rear_hinge_barrel_right")):
        shelf.visual(
            Cylinder(radius=0.004, length=0.010),
            origin=Origin(
                xyz=(x_center, -0.009, -0.001),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=steel,
            name=name,
        )
    shelf.inertial = Inertial.from_geometry(
        Box((0.170, 0.025, 0.040)),
        mass=0.10,
    )
    model.articulation(
        "left_leg_to_shelf",
        ArticulationType.FIXED,
        parent=left_leg,
        child=shelf,
        origin=Origin(xyz=shelf_center),
    )

    front_rail = model.part("front_rail")
    front_rail.visual(
        Box((rail_web_width, rail_thickness, rail_height)),
        origin=Origin(xyz=(-(slot_width / 2.0 + rail_web_width / 2.0), 0.0, 0.0)),
        material=wood,
        name="rail_left_web",
    )
    front_rail.visual(
        Box((rail_web_width, rail_thickness, rail_height)),
        origin=Origin(xyz=((slot_width / 2.0 + rail_web_width / 2.0), 0.0, 0.0)),
        material=wood,
        name="rail_right_web",
    )
    front_rail.visual(
        Box((rail_width, rail_thickness, rail_top_block)),
        origin=Origin(xyz=(0.0, 0.0, (rail_height - rail_top_block) / 2.0)),
        material=wood_dark,
        name="rail_top_block",
    )
    front_rail.visual(
        Box((rail_width, rail_thickness, rail_bottom_block)),
        origin=Origin(xyz=(0.0, 0.0, -(rail_height - rail_bottom_block) / 2.0)),
        material=wood_dark,
        name="rail_bottom_block",
    )
    front_rail.visual(
        Sphere(radius=0.006),
        origin=Origin(xyz=(0.0, 0.004, rail_height / 2.0 + 0.006)),
        material=wood_dark,
        name="rail_knob",
    )
    front_rail.inertial = Inertial.from_geometry(
        Box((0.030, 0.015, 0.250)),
        mass=0.08,
    )
    model.articulation(
        "left_leg_to_front_rail",
        ArticulationType.FIXED,
        parent=left_leg,
        child=front_rail,
        origin=Origin(xyz=rail_center),
    )

    rear_strut = model.part("rear_prop_strut")
    rear_strut_length = math.hypot(0.170, 0.071)
    rear_strut_angle = -math.atan2(0.170, 0.071)
    rear_strut.visual(
        Cylinder(radius=0.004, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="rear_strut_hinge_barrel",
    )
    rear_strut.visual(
        Box((0.020, 0.010, 0.016)),
        origin=Origin(xyz=(0.0, -0.006, -0.004)),
        material=wood_dark,
        name="rear_strut_head",
    )
    rear_strut.visual(
        Box((0.016, 0.009, rear_strut_length)),
        origin=Origin(
            xyz=(0.0, -0.085, -0.0355),
            rpy=(rear_strut_angle, 0.0, 0.0),
        ),
        material=wood,
        name="rear_strut_bar",
    )
    rear_strut.visual(
        Box((0.022, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, -0.170, -0.071)),
        material=rubber,
        name="rear_foot",
    )
    rear_strut.inertial = Inertial.from_geometry(
        Box((0.030, 0.020, 0.190)),
        mass=0.08,
        origin=Origin(xyz=(0.0, -0.085, -0.036)),
    )
    model.articulation(
        "shelf_to_rear_prop",
        ArticulationType.REVOLUTE,
        parent=shelf,
        child=rear_strut,
        origin=Origin(xyz=(0.0, -0.013, -0.001)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.0,
            lower=-0.30,
            upper=0.38,
        ),
    )

    clip_wire = wire_from_points(
        [
            (-0.012, 0.002, 0.008),
            (-0.012, -0.005, 0.016),
            (0.0, -0.007, 0.020),
            (0.012, -0.005, 0.016),
            (0.012, 0.002, 0.008),
        ],
        radius=0.0014,
        corner_mode="fillet",
        corner_radius=0.002,
        cap_ends=False,
    )
    clip_mesh = mesh_from_geometry(clip_wire, "spring_clip_wire")

    spring_clip = model.part("spring_clip")
    for x_center, name in ((-0.0115, "clip_left_cheek"), (0.0115, "clip_right_cheek")):
        spring_clip.visual(
            Box((0.005, 0.014, 0.024)),
            origin=Origin(xyz=(x_center, 0.0, 0.0)),
            material=clip_black,
            name=name,
        )
    spring_clip.visual(
        Box((0.029, 0.005, 0.024)),
        origin=Origin(xyz=(0.0, -0.0065, 0.0)),
        material=clip_black,
        name="clip_back_bridge",
    )
    spring_clip.visual(
        Cylinder(radius=0.0022, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="clip_slot_pin",
    )
    spring_clip.visual(
        Box((0.038, 0.004, 0.018)),
        origin=Origin(xyz=(0.0, 0.013, 0.0)),
        material=clip_black,
        name="clip_front_jaw",
    )
    spring_clip.visual(
        Box((0.038, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.009, 0.009)),
        material=clip_black,
        name="clip_top_lip",
    )
    spring_clip.visual(
        Box((0.020, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.009, -0.012)),
        material=clip_black,
        name="clip_lower_pad",
    )
    spring_clip.visual(
        clip_mesh,
        material=steel,
        name="clip_spring_wire",
    )
    spring_clip.inertial = Inertial.from_geometry(
        Box((0.050, 0.025, 0.040)),
        mass=0.04,
    )
    model.articulation(
        "front_rail_to_clip",
        ArticulationType.PRISMATIC,
        parent=front_rail,
        child=spring_clip,
        origin=Origin(xyz=(0.0, 0.0, slot_top_from_center)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.08,
            lower=0.0,
            upper=0.135,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    left_leg = object_model.get_part("left_front_leg")
    right_leg = object_model.get_part("right_front_leg")
    shelf = object_model.get_part("shelf")
    front_rail = object_model.get_part("front_rail")
    rear_strut = object_model.get_part("rear_prop_strut")
    spring_clip = object_model.get_part("spring_clip")

    right_leg_bar = right_leg.get_visual("right_leg_bar")
    rear_foot = rear_strut.get_visual("rear_foot")

    apex_hinge = object_model.get_articulation("apex_hinge")
    rear_hinge = object_model.get_articulation("shelf_to_rear_prop")
    clip_slide = object_model.get_articulation("front_rail_to_clip")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(left_leg, right_leg, name="front legs meet at apex hinge")
    ctx.expect_contact(shelf, left_leg, name="shelf is mounted to left leg")
    ctx.expect_contact(shelf, right_leg, name="shelf reaches right leg cleat")
    ctx.expect_contact(front_rail, shelf, name="front rail lands on shelf")
    ctx.expect_contact(rear_strut, shelf, name="rear strut stays pinned to shelf hinge")
    ctx.expect_contact(spring_clip, front_rail, name="spring clip rides on rail faces")

    ctx.expect_overlap(shelf, left_leg, axes="yz", min_overlap=0.01, name="shelf aligns with left support zone")
    ctx.expect_overlap(shelf, right_leg, axes="yz", min_overlap=0.01, name="shelf aligns with right support zone")
    ctx.expect_overlap(spring_clip, front_rail, axes="xz", min_overlap=0.012, name="clip stays centered on rail")

    ctx.check(
        "apex hinge axis is fore-aft",
        tuple(apex_hinge.axis) == (0.0, 1.0, 0.0),
        f"axis={apex_hinge.axis}",
    )
    ctx.check(
        "rear strut hinge axis is left-right",
        tuple(rear_hinge.axis) == (1.0, 0.0, 0.0),
        f"axis={rear_hinge.axis}",
    )
    ctx.check(
        "clip slot axis is vertical",
        tuple(clip_slide.axis) == (0.0, 0.0, -1.0),
        f"axis={clip_slide.axis}",
    )

    clip_rest = ctx.part_world_position(spring_clip)
    if clip_rest is None:
        ctx.fail("spring clip rest position available", "spring clip world position could not be measured")
    else:
        with ctx.pose({clip_slide: 0.120}):
            clip_low = ctx.part_world_position(spring_clip)
            if clip_low is None:
                ctx.fail("spring clip lowered position available", "lowered spring clip world position could not be measured")
            else:
                ctx.check(
                    "spring clip slides downward along slot",
                    clip_low[2] < clip_rest[2] - 0.10,
                    f"rest_z={clip_rest[2]:.4f}, low_z={clip_low[2]:.4f}",
                )
            ctx.expect_contact(spring_clip, front_rail, name="spring clip remains guided when lowered")

    right_rest = ctx.part_element_world_aabb(right_leg, elem=right_leg_bar)
    if right_rest is None:
        ctx.fail("right leg rest aabb available", "right leg bar AABB could not be measured")
    else:
        with ctx.pose({apex_hinge: 0.14}):
            right_closed = ctx.part_element_world_aabb(right_leg, elem=right_leg_bar)
            if right_closed is None:
                ctx.fail("right leg posed aabb available", "posed right leg bar AABB could not be measured")
            else:
                rest_center_x = (right_rest[0][0] + right_rest[1][0]) / 2.0
                posed_center_x = (right_closed[0][0] + right_closed[1][0]) / 2.0
                ctx.check(
                    "apex hinge narrows right leg stance",
                    posed_center_x < rest_center_x - 0.02,
                    f"rest_center_x={rest_center_x:.4f}, posed_center_x={posed_center_x:.4f}",
                )
            ctx.expect_contact(left_leg, right_leg, name="front hinge stays physically mated in posed stance")

    rear_rest = ctx.part_element_world_aabb(rear_strut, elem=rear_foot)
    if rear_rest is None:
        ctx.fail("rear foot rest aabb available", "rear foot AABB could not be measured")
    else:
        with ctx.pose({rear_hinge: 0.38}):
            rear_folded = ctx.part_element_world_aabb(rear_strut, elem=rear_foot)
            if rear_folded is None:
                ctx.fail("rear foot posed aabb available", "posed rear foot AABB could not be measured")
            else:
                ctx.check(
                    "rear prop folds toward front frame",
                    rear_folded[0][1] > rear_rest[0][1] + 0.03,
                    f"rest_min_y={rear_rest[0][1]:.4f}, posed_min_y={rear_folded[0][1]:.4f}",
                )
            ctx.expect_contact(rear_strut, shelf, name="rear prop hinge remains seated when folded")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
