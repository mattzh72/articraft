from __future__ import annotations

import math

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
    model = ArticulatedObject(name="rural_parcel_box")

    body_mat = model.material("powder_coated_sage", rgba=(0.32, 0.43, 0.32, 1.0))
    door_mat = model.material("darker_green_doors", rgba=(0.20, 0.31, 0.22, 1.0))
    gasket_mat = model.material("black_rubber_gasket", rgba=(0.01, 0.012, 0.010, 1.0))
    steel_mat = model.material("galvanized_steel", rgba=(0.68, 0.69, 0.65, 1.0))
    shadow_mat = model.material("deep_shadow", rgba=(0.025, 0.028, 0.026, 1.0))
    warning_mat = model.material("delivery_label_yellow", rgba=(0.92, 0.76, 0.22, 1.0))

    width = 0.82
    depth = 0.60
    wall = 0.055
    front_y = -depth / 2.0
    body_bottom = 0.36
    body_height = 1.16
    body_top = body_bottom + body_height
    body_mid_z = body_bottom + body_height / 2.0

    lower_hinge_z = 0.47
    upper_hinge_z = 1.405
    hinge_y = front_y - 0.035

    body = model.part("body")

    # Thick insulated cabinet shell and front frame.
    body.visual(
        Box((wall, depth, body_height)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, 0.0, body_mid_z)),
        material=body_mat,
        name="left_insulated_wall",
    )
    body.visual(
        Box((wall, depth, body_height)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, 0.0, body_mid_z)),
        material=body_mat,
        name="right_insulated_wall",
    )
    body.visual(
        Box((width, wall, body_height)),
        origin=Origin(xyz=(0.0, depth / 2.0 - wall / 2.0, body_mid_z)),
        material=body_mat,
        name="rear_insulated_wall",
    )
    body.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, body_top - wall / 2.0)),
        material=body_mat,
        name="top_insulated_slab",
    )
    body.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, body_bottom + wall / 2.0)),
        material=body_mat,
        name="bottom_insulated_slab",
    )
    body.visual(
        Box((width, 0.040, 0.075)),
        origin=Origin(xyz=(0.0, front_y - 0.018, body_top - 0.0375)),
        material=body_mat,
        name="front_top_lintel",
    )
    body.visual(
        Box((width, 0.040, 0.075)),
        origin=Origin(xyz=(0.0, front_y - 0.018, body_bottom + 0.0375)),
        material=body_mat,
        name="front_bottom_sill",
    )
    body.visual(
        Box((width, 0.040, 0.080)),
        origin=Origin(xyz=(0.0, front_y - 0.018, 1.095)),
        material=body_mat,
        name="deposit_divider_rail",
    )
    body.visual(
        Box((0.070, 0.044, body_height)),
        origin=Origin(xyz=(-width / 2.0 + 0.035, front_y - 0.020, body_mid_z)),
        material=body_mat,
        name="front_side_jamb_0",
    )
    body.visual(
        Box((0.070, 0.044, body_height)),
        origin=Origin(xyz=(width / 2.0 - 0.035, front_y - 0.020, body_mid_z)),
        material=body_mat,
        name="front_side_jamb_1",
    )

    # Recessed dark openings and rubber-looking weather seals make the closed
    # faces read as doors set into a thick insulated cabinet.
    body.visual(
        Box((0.700, 0.010, 0.595)),
        origin=Origin(xyz=(0.0, front_y - 0.002, 0.785)),
        material=shadow_mat,
        name="lower_cavity_shadow",
    )
    body.visual(
        Box((0.700, 0.010, 0.265)),
        origin=Origin(xyz=(0.0, front_y - 0.002, 1.265)),
        material=shadow_mat,
        name="upper_cavity_shadow",
    )
    body.visual(
        Box((0.720, 0.012, 0.025)),
        origin=Origin(xyz=(0.0, front_y - 0.034, 1.112)),
        material=gasket_mat,
        name="upper_weather_gasket",
    )
    body.visual(
        Box((0.780, 0.012, 0.035)),
        origin=Origin(xyz=(0.0, front_y - 0.034, 0.415)),
        material=gasket_mat,
        name="lower_weather_gasket",
    )

    # Fixed leaves for the two horizontal hinges.
    body.visual(
        Box((0.690, 0.030, 0.026)),
        origin=Origin(xyz=(0.0, front_y - 0.006, lower_hinge_z)),
        material=steel_mat,
        name="lower_fixed_hinge_leaf",
    )
    body.visual(
        Box((0.690, 0.030, 0.026)),
        origin=Origin(xyz=(0.0, front_y - 0.006, upper_hinge_z)),
        material=steel_mat,
        name="upper_fixed_hinge_leaf",
    )

    # Short stand with connected rails and foot pads under the insulated box.
    body.visual(
        Box((0.72, 0.46, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.342)),
        material=steel_mat,
        name="stand_top_plate",
    )
    for i, x in enumerate((-0.30, 0.30)):
        for j, y in enumerate((-0.19, 0.19)):
            body.visual(
                Box((0.055, 0.055, 0.320)),
                origin=Origin(xyz=(x, y, 0.172)),
                material=steel_mat,
                name=f"stand_leg_{i}_{j}",
            )
            body.visual(
                Box((0.135, 0.105, 0.030)),
                origin=Origin(xyz=(x, y, 0.015)),
                material=steel_mat,
                name=f"stand_foot_{i}_{j}",
            )
    body.visual(
        Box((0.670, 0.045, 0.045)),
        origin=Origin(xyz=(0.0, -0.19, 0.055)),
        material=steel_mat,
        name="stand_front_rail",
    )
    body.visual(
        Box((0.670, 0.045, 0.045)),
        origin=Origin(xyz=(0.0, 0.19, 0.055)),
        material=steel_mat,
        name="stand_rear_rail",
    )
    body.visual(
        Box((0.045, 0.425, 0.045)),
        origin=Origin(xyz=(-0.30, 0.0, 0.055)),
        material=steel_mat,
        name="stand_side_rail_0",
    )
    body.visual(
        Box((0.045, 0.425, 0.045)),
        origin=Origin(xyz=(0.30, 0.0, 0.055)),
        material=steel_mat,
        name="stand_side_rail_1",
    )

    lower_door = model.part("retrieval_door")
    lower_door.visual(
        Box((0.680, 0.038, 0.600)),
        origin=Origin(xyz=(0.0, -0.045, 0.320)),
        material=door_mat,
        name="lower_door_panel",
    )
    lower_door.visual(
        Box((0.700, 0.010, 0.030)),
        origin=Origin(xyz=(0.0, -0.067, 0.610)),
        material=gasket_mat,
        name="lower_top_seal",
    )
    lower_door.visual(
        Box((0.030, 0.010, 0.590)),
        origin=Origin(xyz=(-0.355, -0.067, 0.315)),
        material=gasket_mat,
        name="lower_side_seal_0",
    )
    lower_door.visual(
        Box((0.030, 0.010, 0.590)),
        origin=Origin(xyz=(0.355, -0.067, 0.315)),
        material=gasket_mat,
        name="lower_side_seal_1",
    )
    lower_door.visual(
        Cylinder(radius=0.016, length=0.640),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_mat,
        name="lower_hinge_barrel",
    )
    lower_door.visual(
        Box((0.620, 0.030, 0.024)),
        origin=Origin(xyz=(0.0, -0.020, 0.020)),
        material=steel_mat,
        name="lower_moving_hinge_leaf",
    )
    lower_door.visual(
        Cylinder(radius=0.018, length=0.360),
        origin=Origin(xyz=(0.0, -0.095, 0.430), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_mat,
        name="lower_pull_bar",
    )
    for x in (-0.150, 0.150):
        lower_door.visual(
            Cylinder(radius=0.010, length=0.060),
            origin=Origin(xyz=(x, -0.070, 0.430), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=steel_mat,
            name=f"lower_handle_stem_{0 if x < 0 else 1}",
        )
    lower_door.visual(
        Box((0.230, 0.006, 0.055)),
        origin=Origin(xyz=(0.0, -0.067, 0.520)),
        material=warning_mat,
        name="parcel_label_plate",
    )

    upper_flap = model.part("deposit_flap")
    upper_flap.visual(
        Box((0.660, 0.036, 0.270)),
        origin=Origin(xyz=(0.0, -0.045, -0.145)),
        material=door_mat,
        name="upper_flap_panel",
    )
    upper_flap.visual(
        Cylinder(radius=0.014, length=0.620),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_mat,
        name="upper_hinge_barrel",
    )
    upper_flap.visual(
        Box((0.600, 0.028, 0.024)),
        origin=Origin(xyz=(0.0, -0.018, -0.012)),
        material=steel_mat,
        name="upper_moving_hinge_leaf",
    )
    upper_flap.visual(
        Box((0.390, 0.046, 0.035)),
        origin=Origin(xyz=(0.0, -0.078, -0.235)),
        material=steel_mat,
        name="deposit_pull_lip",
    )
    upper_flap.visual(
        Box((0.620, 0.010, 0.026)),
        origin=Origin(xyz=(0.0, -0.065, -0.280)),
        material=gasket_mat,
        name="upper_bottom_seal",
    )

    model.articulation(
        "body_to_retrieval_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lower_door,
        origin=Origin(xyz=(0.0, hinge_y, lower_hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.6, lower=0.0, upper=1.75),
    )
    model.articulation(
        "body_to_deposit_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=upper_flap,
        origin=Origin(xyz=(0.0, hinge_y, upper_hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=0.0, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lower = object_model.get_part("retrieval_door")
    upper = object_model.get_part("deposit_flap")
    lower_hinge = object_model.get_articulation("body_to_retrieval_door")
    upper_hinge = object_model.get_articulation("body_to_deposit_flap")

    ctx.allow_overlap(
        body,
        lower,
        elem_a="lower_fixed_hinge_leaf",
        elem_b="lower_hinge_barrel",
        reason="The simplified lower hinge barrel is captured by the fixed hinge leaf with a tiny local seated overlap.",
    )

    with ctx.pose({lower_hinge: 0.0, upper_hinge: 0.0}):
        ctx.expect_gap(
            body,
            lower,
            axis="y",
            positive_elem="deposit_divider_rail",
            negative_elem="lower_door_panel",
            max_gap=0.060,
            max_penetration=0.0,
            name="closed lower door sits proud of front frame",
        )
        ctx.expect_gap(
            body,
            upper,
            axis="y",
            positive_elem="front_top_lintel",
            negative_elem="upper_flap_panel",
            max_gap=0.060,
            max_penetration=0.0,
            name="closed deposit flap sits proud of front frame",
        )
        ctx.expect_overlap(
            lower,
            body,
            axes="x",
            elem_a="lower_hinge_barrel",
            elem_b="lower_fixed_hinge_leaf",
            min_overlap=0.55,
            name="lower hinge spans the retrieval opening",
        )
        ctx.expect_gap(
            body,
            lower,
            axis="y",
            positive_elem="lower_fixed_hinge_leaf",
            negative_elem="lower_hinge_barrel",
            max_penetration=0.003,
            name="lower hinge capture overlap stays local",
        )
        ctx.expect_overlap(
            upper,
            body,
            axes="x",
            elem_a="upper_hinge_barrel",
            elem_b="upper_fixed_hinge_leaf",
            min_overlap=0.55,
            name="upper hinge spans the deposit flap",
        )

    rest_lower_box = ctx.part_element_world_aabb(lower, elem="lower_door_panel")
    rest_upper_box = ctx.part_element_world_aabb(upper, elem="upper_flap_panel")
    with ctx.pose({lower_hinge: 1.25, upper_hinge: 0.95}):
        opened_lower_box = ctx.part_element_world_aabb(lower, elem="lower_door_panel")
        opened_upper_box = ctx.part_element_world_aabb(upper, elem="upper_flap_panel")
        ctx.expect_gap(
            body,
            lower,
            axis="y",
            positive_elem="deposit_divider_rail",
            negative_elem="lower_door_panel",
            min_gap=0.010,
            name="drop front rotates outward from cabinet",
        )
        ctx.expect_gap(
            body,
            upper,
            axis="y",
            positive_elem="front_top_lintel",
            negative_elem="upper_flap_panel",
            min_gap=0.005,
            name="deposit flap rotates outward from cabinet",
        )

    def center_z(aabb):
        if aabb is None:
            return None
        return (aabb[0][2] + aabb[1][2]) / 2.0

    def front_y(aabb):
        if aabb is None:
            return None
        return aabb[0][1]

    rest_lower_z = center_z(rest_lower_box)
    opened_lower_z = center_z(opened_lower_box)
    rest_upper_z = center_z(rest_upper_box)
    opened_upper_z = center_z(opened_upper_box)
    rest_lower_front = front_y(rest_lower_box)
    opened_lower_front = front_y(opened_lower_box)
    rest_upper_front = front_y(rest_upper_box)
    opened_upper_front = front_y(opened_upper_box)

    ctx.check(
        "retrieval door hinge is the low horizontal axis",
        rest_lower_z is not None
        and opened_lower_z is not None
        and rest_lower_front is not None
        and opened_lower_front is not None
        and opened_lower_z < rest_lower_z - 0.03
        and opened_lower_front < rest_lower_front - 0.05,
        details=f"rest={rest_lower_box}, opened={opened_lower_box}",
    )
    ctx.check(
        "deposit flap hinge is the upper horizontal axis",
        rest_upper_z is not None
        and opened_upper_z is not None
        and rest_upper_front is not None
        and opened_upper_front is not None
        and opened_upper_z > rest_upper_z + 0.02
        and opened_upper_front < rest_upper_front - 0.04,
        details=f"rest={rest_upper_box}, opened={opened_upper_box}",
    )

    return ctx.report()


object_model = build_object_model()
