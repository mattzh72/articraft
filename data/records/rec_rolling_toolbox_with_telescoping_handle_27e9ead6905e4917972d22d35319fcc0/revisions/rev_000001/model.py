from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="workshop_tote_on_wheels")

    model.material("red_polymer", rgba=(0.78, 0.08, 0.04, 1.0))
    model.material("dark_polymer", rgba=(0.03, 0.035, 0.04, 1.0))
    model.material("rubber_black", rgba=(0.01, 0.01, 0.012, 1.0))
    model.material("zinc_steel", rgba=(0.58, 0.60, 0.58, 1.0))
    model.material("shadow_grey", rgba=(0.14, 0.15, 0.16, 1.0))

    body = model.part("body")

    # Rectangular lower tote body: open-topped shell on a wheeled service-cart scale.
    body.visual(
        Box((0.70, 0.42, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material="red_polymer",
        name="lower_floor",
    )
    body.visual(
        Box((0.70, 0.035, 0.34)),
        origin=Origin(xyz=(0.0, 0.1925, 0.35)),
        material="red_polymer",
        name="lower_side_0",
    )
    body.visual(
        Box((0.70, 0.035, 0.34)),
        origin=Origin(xyz=(0.0, -0.1925, 0.35)),
        material="red_polymer",
        name="lower_side_1",
    )
    body.visual(
        Box((0.035, 0.42, 0.34)),
        origin=Origin(xyz=(0.3325, 0.0, 0.35)),
        material="red_polymer",
        name="front_wall",
    )
    body.visual(
        Box((0.035, 0.42, 0.34)),
        origin=Origin(xyz=(-0.3325, 0.0, 0.35)),
        material="red_polymer",
        name="rear_wall",
    )
    body.visual(
        Box((0.61, 0.012, 0.17)),
        origin=Origin(xyz=(0.0, 0.2135, 0.35)),
        material="dark_polymer",
        name="side_recess_0",
    )
    body.visual(
        Box((0.61, 0.012, 0.17)),
        origin=Origin(xyz=(0.0, -0.2135, 0.35)),
        material="dark_polymer",
        name="side_recess_1",
    )

    # Raised top organizer compartment under the hinged lid.
    body.visual(
        Box((0.68, 0.40, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.535)),
        material="red_polymer",
        name="top_floor",
    )
    body.visual(
        Box((0.68, 0.030, 0.13)),
        origin=Origin(xyz=(0.0, 0.185, 0.615)),
        material="red_polymer",
        name="top_side_0",
    )
    body.visual(
        Box((0.68, 0.030, 0.13)),
        origin=Origin(xyz=(0.0, -0.185, 0.615)),
        material="red_polymer",
        name="top_side_1",
    )
    body.visual(
        Box((0.035, 0.40, 0.13)),
        origin=Origin(xyz=(0.3225, 0.0, 0.615)),
        material="red_polymer",
        name="top_front_wall",
    )
    body.visual(
        Box((0.035, 0.40, 0.13)),
        origin=Origin(xyz=(-0.3225, 0.0, 0.615)),
        material="red_polymer",
        name="top_rear_wall",
    )
    body.visual(
        Box((0.64, 0.34, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.56)),
        material="shadow_grey",
        name="organizer_well",
    )
    body.visual(
        Box((0.64, 0.020, 0.025)),
        origin=Origin(xyz=(0.0, 0.195, 0.665)),
        material="dark_polymer",
        name="top_rim_0",
    )
    body.visual(
        Box((0.64, 0.020, 0.025)),
        origin=Origin(xyz=(0.0, -0.195, 0.665)),
        material="dark_polymer",
        name="top_rim_1",
    )
    body.visual(
        Box((0.020, 0.40, 0.025)),
        origin=Origin(xyz=(0.345, 0.0, 0.665)),
        material="dark_polymer",
        name="top_front_rim",
    )

    # Fixed rear transport axle, brackets, front skid feet, and telescoping-handle sockets.
    body.visual(
        Cylinder(radius=0.009, length=0.56),
        origin=Origin(xyz=(-0.285, 0.0, 0.105), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="zinc_steel",
        name="transport_axle",
    )
    for index, y in enumerate((0.18, -0.18)):
        body.visual(
            Box((0.050, 0.040, 0.125)),
            origin=Origin(xyz=(-0.285, y, 0.105)),
            material="dark_polymer",
            name=f"axle_bracket_{index}",
        )
    for index, y in enumerate((0.145, -0.145)):
        body.visual(
            Box((0.090, 0.070, 0.080)),
            origin=Origin(xyz=(0.275, y, 0.040)),
            material="rubber_black",
            name=f"front_foot_{index}",
        )
        body.visual(
            Box((0.050, 0.050, 0.060)),
            origin=Origin(xyz=(0.275, y, 0.100)),
            material="dark_polymer",
            name=f"foot_post_{index}",
        )
    for index, y in enumerate((0.15, -0.15)):
        body.visual(
            Box((0.075, 0.055, 0.31)),
            origin=Origin(xyz=(-0.47, y, 0.585)),
            material="zinc_steel",
            name=f"handle_sleeve_{index}",
        )
        side_y = y + (0.038 if y > 0.0 else -0.038)
        for level, z in enumerate((0.47, 0.63)):
            body.visual(
                Box((0.160, 0.025, 0.040)),
                origin=Origin(xyz=(-0.420, side_y, z)),
                material="zinc_steel",
                name=f"rail_bracket_{index}_{level}",
            )

    lid = model.part("lid")
    lid.visual(
        Box((0.73, 0.405, 0.035)),
        origin=Origin(xyz=(0.385, 0.0, 0.0175)),
        material="red_polymer",
        name="lid_panel",
    )
    lid.visual(
        Box((0.035, 0.36, 0.040)),
        origin=Origin(xyz=(0.765, 0.0, 0.004)),
        material="dark_polymer",
        name="front_latch_lip",
    )
    lid.visual(
        Box((0.48, 0.035, 0.020)),
        origin=Origin(xyz=(0.36, 0.115, 0.045)),
        material="dark_polymer",
        name="lid_rib_0",
    )
    lid.visual(
        Box((0.48, 0.035, 0.020)),
        origin=Origin(xyz=(0.36, -0.115, 0.045)),
        material="dark_polymer",
        name="lid_rib_1",
    )
    lid.visual(
        Cylinder(radius=0.014, length=0.42),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="zinc_steel",
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.070, 0.38, 0.012)),
        origin=Origin(xyz=(0.035, 0.0, 0.006)),
        material="zinc_steel",
        name="hinge_leaf",
    )

    lower_handle = model.part("lower_handle")
    for index, y in enumerate((0.15, -0.15)):
        lower_handle.visual(
            Box((0.050, 0.026, 0.62)),
            origin=Origin(xyz=(0.0, y, 0.030)),
            material="zinc_steel",
            name=f"lower_rail_{index}",
        )
    lower_handle.visual(
        Box((0.030, 0.37, 0.025)),
        origin=Origin(xyz=(-0.040, 0.0, 0.352)),
        material="zinc_steel",
        name="lower_crossbar",
    )

    upper_handle = model.part("upper_handle")
    for index, y in enumerate((0.15, -0.15)):
        upper_handle.visual(
            Box((0.017, 0.017, 0.44)),
            origin=Origin(xyz=(0.0, y, 0.020)),
            material="zinc_steel",
            name=f"upper_rail_{index}",
        )
    upper_handle.visual(
        Cylinder(radius=0.025, length=0.50),
        origin=Origin(xyz=(0.0, 0.0, 0.245), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="rubber_black",
        name="cross_grip",
    )
    upper_handle.visual(
        Box((0.035, 0.34, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        material="zinc_steel",
        name="grip_core",
    )

    rim_mesh = mesh_from_geometry(
        WheelGeometry(
            0.070,
            0.070,
            rim=WheelRim(inner_radius=0.046, flange_height=0.006, flange_thickness=0.003),
            hub=WheelHub(
                radius=0.028,
                width=0.056,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.034, hole_diameter=0.004),
            ),
            face=WheelFace(dish_depth=0.005, front_inset=0.003, rear_inset=0.003),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.003, window_radius=0.008),
            bore=WheelBore(style="round", diameter=0.018),
        ),
        "rear_wheel_rim",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.105,
            0.072,
            inner_radius=0.070,
            tread=TireTread(style="block", depth=0.006, count=20, land_ratio=0.55),
            sidewall=TireSidewall(style="square", bulge=0.025),
            shoulder=TireShoulder(width=0.008, radius=0.003),
        ),
        "rear_wheel_tire",
    )
    for index, y in enumerate((0.250, -0.250)):
        wheel = model.part(f"wheel_{index}")
        wheel.visual(
            tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material="rubber_black",
            name="tire",
        )
        wheel.visual(
            rim_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material="zinc_steel",
            name="rim",
        )
        model.articulation(
            f"body_to_wheel_{index}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(-0.285, y, 0.105)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=12.0),
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.390, 0.0, 0.680)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=1.35),
    )
    model.articulation(
        "body_to_lower_handle",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lower_handle,
        origin=Origin(xyz=(-0.470, 0.0, 0.690)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.25, lower=0.0, upper=0.10),
    )
    model.articulation(
        "lower_to_upper_handle",
        ArticulationType.PRISMATIC,
        parent=lower_handle,
        child=upper_handle,
        origin=Origin(xyz=(0.0, 0.0, 0.280)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.25, lower=0.0, upper=0.12),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    lower_handle = object_model.get_part("lower_handle")
    upper_handle = object_model.get_part("upper_handle")
    lid_hinge = object_model.get_articulation("body_to_lid")
    lower_slide = object_model.get_articulation("body_to_lower_handle")
    upper_slide = object_model.get_articulation("lower_to_upper_handle")

    for index in range(2):
        ctx.allow_overlap(
            body,
            lower_handle,
            elem_a=f"handle_sleeve_{index}",
            elem_b=f"lower_rail_{index}",
            reason="The lower telescoping rail is intentionally represented as retained inside the fixed rear sleeve proxy.",
        )
        ctx.allow_overlap(
            lower_handle,
            upper_handle,
            elem_a=f"lower_rail_{index}",
            elem_b=f"upper_rail_{index}",
            reason="The upper handle rail is intentionally represented as sliding inside the lower rail proxy.",
        )
        wheel = object_model.get_part(f"wheel_{index}")
        ctx.allow_overlap(
            body,
            wheel,
            elem_a="transport_axle",
            elem_b="rim",
            reason="The wheel hub is captured around the continuous transport axle at the bore.",
        )
        ctx.expect_overlap(
            body,
            wheel,
            axes="y",
            elem_a="transport_axle",
            elem_b="rim",
            min_overlap=0.050,
            name=f"wheel {index} rides on transport axle",
        )

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="top_front_wall",
        min_gap=0.0,
        max_gap=0.006,
        name="closed lid seats above top compartment",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_panel",
        elem_b="top_floor",
        min_overlap=0.34,
        name="lid covers organizer footprint",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({lid_hinge: 1.1}):
        opened_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    ctx.check(
        "lid rotates upward on rear hinge",
        closed_lid_aabb is not None
        and opened_lid_aabb is not None
        and opened_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.18,
        details=f"closed={closed_lid_aabb}, opened={opened_lid_aabb}",
    )

    for index in range(2):
        ctx.expect_within(
            lower_handle,
            body,
            axes="xy",
            inner_elem=f"lower_rail_{index}",
            outer_elem=f"handle_sleeve_{index}",
            margin=0.002,
            name=f"lower rail {index} centered in fixed sleeve",
        )
        ctx.expect_overlap(
            lower_handle,
            body,
            axes="z",
            elem_a=f"lower_rail_{index}",
            elem_b=f"handle_sleeve_{index}",
            min_overlap=0.25,
            name=f"lower rail {index} retained collapsed",
        )
        ctx.expect_within(
            upper_handle,
            lower_handle,
            axes="xy",
            inner_elem=f"upper_rail_{index}",
            outer_elem=f"lower_rail_{index}",
            margin=0.002,
            name=f"upper rail {index} centered in lower rail",
        )
        ctx.expect_overlap(
            upper_handle,
            lower_handle,
            axes="z",
            elem_a=f"upper_rail_{index}",
            elem_b=f"lower_rail_{index}",
            min_overlap=0.24,
            name=f"upper rail {index} retained collapsed",
        )

    rest_upper_pos = ctx.part_world_position(upper_handle)
    with ctx.pose({lower_slide: 0.10, upper_slide: 0.12}):
        extended_upper_pos = ctx.part_world_position(upper_handle)
        for index in range(2):
            ctx.expect_overlap(
                lower_handle,
                body,
                axes="z",
                elem_a=f"lower_rail_{index}",
                elem_b=f"handle_sleeve_{index}",
                min_overlap=0.18,
                name=f"lower rail {index} retained extended",
            )
            ctx.expect_overlap(
                upper_handle,
                lower_handle,
                axes="z",
                elem_a=f"upper_rail_{index}",
                elem_b=f"lower_rail_{index}",
                min_overlap=0.12,
                name=f"upper rail {index} retained extended",
            )
    ctx.check(
        "telescoping handle stages extend upward",
        rest_upper_pos is not None
        and extended_upper_pos is not None
        and extended_upper_pos[2] > rest_upper_pos[2] + 0.20,
        details=f"rest={rest_upper_pos}, extended={extended_upper_pos}",
    )

    wheel_joints = [
        object_model.get_articulation("body_to_wheel_0"),
        object_model.get_articulation("body_to_wheel_1"),
    ]
    ctx.check(
        "rear wheel pair uses continuous axle joints",
        all(j.articulation_type == ArticulationType.CONTINUOUS and tuple(j.axis) == (0.0, 1.0, 0.0) for j in wheel_joints),
        details=f"joints={[j.articulation_type for j in wheel_joints]}, axes={[j.axis for j in wheel_joints]}",
    )

    return ctx.report()


object_model = build_object_model()
