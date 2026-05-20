from __future__ import annotations

from math import isclose

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_by_side_refrigerator")

    appliance_white = Material("warm_white_enamel", rgba=(0.93, 0.92, 0.88, 1.0))
    side_white = Material("slightly_darker_side", rgba=(0.82, 0.84, 0.84, 1.0))
    dark_gasket = Material("black_rubber_gasket", rgba=(0.015, 0.016, 0.018, 1.0))
    dark_cavity = Material("dark_cabinet_cavity", rgba=(0.08, 0.085, 0.09, 1.0))
    brushed_metal = Material("brushed_metal", rgba=(0.66, 0.68, 0.69, 1.0))
    smoked_plastic = Material("smoked_service_plastic", rgba=(0.10, 0.12, 0.14, 1.0))

    # Real-world refrigerator proportions, in meters.
    width = 0.92
    depth = 0.68
    height = 1.80
    wall = 0.040
    back_wall = 0.035
    front_y = -0.315
    rear_y = front_y + depth
    cabinet_y = (front_y + rear_y) / 2.0
    seam_x = -0.080
    center_gap = 0.008
    left_hinge_x = -width / 2.0
    right_hinge_x = width / 2.0
    hinge_y = front_y - 0.050
    hinge_z = height / 2.0

    door_height = 1.72
    door_thickness = 0.080
    left_door_width = seam_x - center_gap / 2.0 - left_hinge_x
    right_door_width = right_hinge_x - (seam_x + center_gap / 2.0)

    cabinet = model.part("cabinet")
    # Open-front insulated body: side walls, top, bottom, back wall, and the
    # off-center divider that makes one tall left compartment and a wider right
    # compartment.
    cabinet.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, cabinet_y, height / 2.0)),
        material=side_white,
        name="outer_side_0",
    )
    cabinet.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, cabinet_y, height / 2.0)),
        material=side_white,
        name="outer_side_1",
    )
    cabinet.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, cabinet_y, height - wall / 2.0)),
        material=side_white,
        name="top_cap",
    )
    cabinet.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, cabinet_y, wall / 2.0)),
        material=side_white,
        name="bottom_cap",
    )
    cabinet.visual(
        Box((width, back_wall, height)),
        origin=Origin(xyz=(0.0, rear_y - back_wall / 2.0, height / 2.0)),
        material=dark_cavity,
        name="rear_liner",
    )
    cabinet.visual(
        Box((0.026, depth - back_wall, height - 2.0 * wall)),
        origin=Origin(xyz=(seam_x, front_y + (depth - back_wall) / 2.0, height / 2.0)),
        material=dark_cavity,
        name="center_divider",
    )
    cabinet.visual(
        Box((0.030, 0.024, height - 0.16)),
        origin=Origin(xyz=(seam_x, front_y + 0.012, height / 2.0)),
        material=dark_gasket,
        name="front_mullion",
    )
    cabinet.visual(
        Box((width - 0.08, 0.020, 0.060)),
        origin=Origin(xyz=(0.0, front_y + 0.010, 0.075)),
        material=dark_gasket,
        name="toe_kick_grille",
    )
    # Slim exposed hinge plates at both outer sides show the vertical axes on
    # which the tall doors rotate while remaining attached to the cabinet shell.
    for side_index, x in enumerate((left_hinge_x - 0.004, right_hinge_x + 0.004)):
        for plate_index, z in enumerate((0.42, 0.90, 1.38)):
            cabinet.visual(
                Box((0.018, 0.014, 0.24)),
                origin=Origin(xyz=(x, front_y - 0.002, z)),
                material=brushed_metal,
                name=f"side_hinge_{side_index}_{plate_index}",
            )

    left_door = model.part("left_door")
    left_door.visual(
        Box((left_door_width, door_thickness, door_height)),
        origin=Origin(xyz=(left_door_width / 2.0, 0.0, 0.0)),
        material=appliance_white,
        name="outer_panel",
    )
    left_door.visual(
        Box((0.012, 0.010, door_height - 0.10)),
        origin=Origin(xyz=(left_door_width - 0.006, -door_thickness / 2.0 - 0.005, 0.0)),
        material=dark_gasket,
        name="center_gasket",
    )
    left_door.visual(
        Box((0.030, 0.028, 1.10)),
        origin=Origin(xyz=(left_door_width - 0.055, -0.087, 0.02)),
        material=brushed_metal,
        name="pull_bar",
    )
    left_door.visual(
        Box((0.036, 0.054, 0.080)),
        origin=Origin(xyz=(left_door_width - 0.055, -0.058, 0.43)),
        material=brushed_metal,
        name="upper_handle_standoff",
    )
    left_door.visual(
        Box((0.036, 0.054, 0.080)),
        origin=Origin(xyz=(left_door_width - 0.055, -0.058, -0.39)),
        material=brushed_metal,
        name="lower_handle_standoff",
    )
    for leaf_index, z in enumerate((-0.48, 0.0, 0.48)):
        left_door.visual(
            Box((0.018, 0.014, 0.22)),
            origin=Origin(xyz=(0.004, 0.034, z)),
            material=brushed_metal,
            name=f"hinge_leaf_{leaf_index}",
        )

    right_door = model.part("right_door")
    service_center_x = -0.345
    service_center_z = 0.185
    service_opening_w = 0.250
    service_opening_h = 0.350
    service_left_x = service_center_x - service_opening_w / 2.0
    service_right_x = service_center_x + service_opening_w / 2.0
    service_bottom_z = service_center_z - service_opening_h / 2.0
    service_top_z = service_center_z + service_opening_h / 2.0
    # The wider right door is built as a single frame around the service opening.
    # The small moving flap below fills this full-depth opening instead of being
    # laid on top of an uncut solid panel.
    right_door.visual(
        Box((right_door_width + service_left_x, door_thickness, door_height)),
        origin=Origin(xyz=((-right_door_width + service_left_x) / 2.0, 0.0, 0.0)),
        material=appliance_white,
        name="outer_stile",
    )
    right_door.visual(
        Box((-service_right_x, door_thickness, door_height)),
        origin=Origin(xyz=(service_right_x / 2.0, 0.0, 0.0)),
        material=appliance_white,
        name="hinge_stile",
    )
    right_door.visual(
        Box((right_door_width, door_thickness, door_height / 2.0 - service_top_z)),
        origin=Origin(xyz=(-right_door_width / 2.0, 0.0, (door_height / 2.0 + service_top_z) / 2.0)),
        material=appliance_white,
        name="upper_rail",
    )
    right_door.visual(
        Box((right_door_width, door_thickness, service_bottom_z + door_height / 2.0)),
        origin=Origin(xyz=(-right_door_width / 2.0, 0.0, (-door_height / 2.0 + service_bottom_z) / 2.0)),
        material=appliance_white,
        name="lower_rail",
    )
    # Black trim surrounding the service flap reads as a gasketed insert.
    trim_y = -door_thickness / 2.0 - 0.006
    right_door.visual(
        Box((service_opening_w + 0.034, 0.014, 0.018)),
        origin=Origin(xyz=(service_center_x, trim_y, service_top_z + 0.009)),
        material=dark_gasket,
        name="flap_trim_top",
    )
    right_door.visual(
        Box((service_opening_w + 0.034, 0.014, 0.018)),
        origin=Origin(xyz=(service_center_x, trim_y, service_bottom_z - 0.009)),
        material=dark_gasket,
        name="flap_trim_bottom",
    )
    right_door.visual(
        Box((0.018, 0.014, service_opening_h + 0.034)),
        origin=Origin(xyz=(service_left_x - 0.009, trim_y, service_center_z)),
        material=dark_gasket,
        name="flap_trim_side_0",
    )
    right_door.visual(
        Box((0.018, 0.014, service_opening_h + 0.034)),
        origin=Origin(xyz=(service_right_x + 0.009, trim_y, service_center_z)),
        material=dark_gasket,
        name="flap_trim_side_1",
    )
    right_door.visual(
        Box((0.012, 0.010, door_height - 0.10)),
        origin=Origin(xyz=(-right_door_width + 0.006, -door_thickness / 2.0 - 0.005, 0.0)),
        material=dark_gasket,
        name="center_gasket",
    )
    right_door.visual(
        Box((0.030, 0.028, 1.06)),
        origin=Origin(xyz=(-right_door_width + 0.055, -0.087, -0.02)),
        material=brushed_metal,
        name="pull_bar",
    )
    right_door.visual(
        Box((0.036, 0.054, 0.080)),
        origin=Origin(xyz=(-right_door_width + 0.055, -0.058, 0.40)),
        material=brushed_metal,
        name="upper_handle_standoff",
    )
    right_door.visual(
        Box((0.036, 0.054, 0.080)),
        origin=Origin(xyz=(-right_door_width + 0.055, -0.058, -0.42)),
        material=brushed_metal,
        name="lower_handle_standoff",
    )
    right_door.visual(
        Box((0.010, 0.018, service_opening_h + 0.010)),
        origin=Origin(xyz=(service_left_x + 0.002, trim_y - 0.006, service_center_z)),
        material=brushed_metal,
        name="flap_hinge_leaf",
    )
    for leaf_index, z in enumerate((-0.48, 0.0, 0.48)):
        right_door.visual(
            Box((0.018, 0.014, 0.22)),
            origin=Origin(xyz=(-0.004, 0.034, z)),
            material=brushed_metal,
            name=f"hinge_leaf_{leaf_index}",
        )

    service_flap = model.part("service_flap")
    service_flap_w = 0.220
    service_flap_h = 0.300
    service_flap.visual(
        Box((service_flap_w, 0.026, service_flap_h)),
        origin=Origin(xyz=(service_flap_w / 2.0, 0.0, 0.0)),
        material=smoked_plastic,
        name="flap_panel",
    )
    service_flap.visual(
        Cylinder(radius=0.012, length=service_flap_h + 0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=brushed_metal,
        name="hinge_barrel",
    )
    service_flap.visual(
        Box((0.095, 0.010, 0.020)),
        origin=Origin(xyz=(service_flap_w * 0.58, -0.018, -service_flap_h * 0.36)),
        material=brushed_metal,
        name="pull_lip",
    )
    service_flap.visual(
        Box((0.120, 0.006, 0.055)),
        origin=Origin(xyz=(service_flap_w * 0.56, -0.015, service_flap_h * 0.18)),
        material=dark_gasket,
        name="dark_inset",
    )

    left_hinge = model.articulation(
        "cabinet_to_left_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=left_door,
        origin=Origin(xyz=(left_hinge_x, hinge_y, hinge_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.5, lower=0.0, upper=1.65),
    )
    right_hinge = model.articulation(
        "cabinet_to_right_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=right_door,
        origin=Origin(xyz=(right_hinge_x, hinge_y, hinge_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.5, lower=0.0, upper=1.65),
    )
    service_hinge = model.articulation(
        "right_door_to_service_flap",
        ArticulationType.REVOLUTE,
        parent=right_door,
        child=service_flap,
        origin=Origin(xyz=(service_center_x - service_flap_w / 2.0, -0.052, service_center_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.20),
    )
    # Keep names alive for lint-free readability when inspecting the script.
    assert left_hinge.name and right_hinge.name and service_hinge.name
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    service_flap = object_model.get_part("service_flap")
    left_hinge = object_model.get_articulation("cabinet_to_left_door")
    right_hinge = object_model.get_articulation("cabinet_to_right_door")
    service_hinge = object_model.get_articulation("right_door_to_service_flap")

    ctx.expect_gap(
        cabinet,
        left_door,
        axis="y",
        min_gap=0.004,
        max_gap=0.020,
        positive_elem="outer_side_0",
        negative_elem="outer_panel",
        name="left door sits just in front of cabinet",
    )
    ctx.expect_gap(
        cabinet,
        right_door,
        axis="y",
        min_gap=0.004,
        max_gap=0.020,
        positive_elem="outer_side_1",
        negative_elem="hinge_stile",
        name="right door sits just in front of cabinet",
    )
    ctx.expect_gap(
        right_door,
        left_door,
        axis="x",
        min_gap=0.003,
        max_gap=0.015,
        name="center seam separates the two tall doors",
    )
    ctx.expect_overlap(
        left_door,
        cabinet,
        axes="xz",
        min_overlap=0.30,
        name="left door covers the left compartment",
    )
    ctx.expect_overlap(
        right_door,
        cabinet,
        axes="xz",
        min_overlap=0.50,
        name="right door covers the wider compartment",
    )
    ctx.expect_within(
        service_flap,
        right_door,
        axes="xz",
        margin=0.002,
        name="service flap is built into the right door outline",
    )

    for joint, expected_axis, upper in (
        (left_hinge, (0.0, 0.0, -1.0), 1.65),
        (right_hinge, (0.0, 0.0, 1.0), 1.65),
        (service_hinge, (0.0, 0.0, -1.0), 1.20),
    ):
        ctx.check(
            f"{joint.name} has vertical hinge axis and realistic swing",
            tuple(round(v, 6) for v in joint.axis) == expected_axis
            and joint.motion_limits is not None
            and isclose(joint.motion_limits.lower, 0.0)
            and joint.motion_limits.upper is not None
            and joint.motion_limits.upper >= upper - 0.01,
            details=f"axis={joint.axis}, limits={joint.motion_limits}",
        )

    closed_left_aabb = ctx.part_world_aabb(left_door)
    closed_right_aabb = ctx.part_world_aabb(right_door)
    closed_flap_aabb = ctx.part_world_aabb(service_flap)
    with ctx.pose({left_hinge: 1.10, right_hinge: 1.10, service_hinge: 0.85}):
        open_left_aabb = ctx.part_world_aabb(left_door)
        open_right_aabb = ctx.part_world_aabb(right_door)
        open_flap_aabb = ctx.part_world_aabb(service_flap)

    ctx.check(
        "left main door swings outward toward the viewer",
        closed_left_aabb is not None
        and open_left_aabb is not None
        and open_left_aabb[0][1] < closed_left_aabb[0][1] - 0.08,
        details=f"closed={closed_left_aabb}, open={open_left_aabb}",
    )
    ctx.check(
        "right main door swings outward toward the viewer",
        closed_right_aabb is not None
        and open_right_aabb is not None
        and open_right_aabb[0][1] < closed_right_aabb[0][1] - 0.08,
        details=f"closed={closed_right_aabb}, open={open_right_aabb}",
    )
    ctx.check(
        "service flap opens outward from the door face",
        closed_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[0][1] < closed_flap_aabb[0][1] - 0.05,
        details=f"closed={closed_flap_aabb}, open={open_flap_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
