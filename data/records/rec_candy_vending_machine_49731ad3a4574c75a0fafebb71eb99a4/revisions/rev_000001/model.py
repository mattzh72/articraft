from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="boxy_candy_vending_cabinet")

    red = model.material("red_powder_coat", rgba=(0.78, 0.05, 0.04, 1.0))
    dark = model.material("matte_black_trim", rgba=(0.02, 0.018, 0.016, 1.0))
    clear = model.material("clear_polycarbonate", rgba=(0.62, 0.86, 1.0, 0.34))
    gray = model.material("smoky_flap_plastic", rgba=(0.25, 0.30, 0.33, 0.58))
    metal = model.material("brushed_steel", rgba=(0.63, 0.62, 0.58, 1.0))
    label = model.material("cream_label_print", rgba=(0.95, 0.86, 0.56, 1.0))
    yellow = model.material("candy_yellow", rgba=(1.0, 0.82, 0.05, 1.0))
    blue = model.material("candy_blue", rgba=(0.08, 0.35, 0.95, 1.0))
    green = model.material("candy_green", rgba=(0.02, 0.65, 0.18, 1.0))
    pink = model.material("candy_pink", rgba=(1.0, 0.18, 0.55, 1.0))
    orange = model.material("candy_orange", rgba=(1.0, 0.44, 0.04, 1.0))

    width = 0.50
    depth = 0.32
    height = 0.90
    wall = 0.028
    front_y = -depth / 2.0

    cabinet = model.part("cabinet")

    # Boxy sheet-metal cabinet shell: back, sides, top, bottom, and a framed front.
    cabinet.visual(
        Box((width, wall, height)),
        origin=Origin(xyz=(0.0, depth / 2.0 - wall / 2.0, height / 2.0)),
        material=red,
        name="back_panel",
    )
    cabinet.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, 0.0, height / 2.0)),
        material=red,
        name="side_wall_0",
    )
    cabinet.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, 0.0, height / 2.0)),
        material=red,
        name="side_wall_1",
    )
    cabinet.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, height - wall / 2.0)),
        material=red,
        name="top_panel",
    )
    cabinet.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall / 2.0)),
        material=red,
        name="bottom_panel",
    )
    cabinet.visual(
        Box((0.050, wall, height)),
        origin=Origin(xyz=(-width / 2.0 + 0.025, front_y + wall / 2.0, height / 2.0)),
        material=red,
        name="front_stile_0",
    )
    cabinet.visual(
        Box((0.050, wall, height)),
        origin=Origin(xyz=(width / 2.0 - 0.025, front_y + wall / 2.0, height / 2.0)),
        material=red,
        name="front_stile_1",
    )
    cabinet.visual(
        Box((width, wall, 0.070)),
        origin=Origin(xyz=(0.0, front_y + wall / 2.0, height - 0.035)),
        material=red,
        name="front_top_rail",
    )
    cabinet.visual(
        Box((width, wall, 0.045)),
        origin=Origin(xyz=(0.0, front_y + wall / 2.0, 0.365)),
        material=red,
        name="display_bottom_rail",
    )
    cabinet.visual(
        Box((width, wall, 0.320)),
        origin=Origin(xyz=(0.0, front_y + wall / 2.0, 0.160)),
        material=red,
        name="lower_front_panel",
    )

    # Clear viewing panel and colorful candy dots fixed against it so the front
    # reads as a candy display rather than a blank window.
    cabinet.visual(
        Box((0.345, 0.006, 0.415)),
        origin=Origin(xyz=(0.0, front_y - 0.003, 0.605)),
        material=clear,
        name="clear_front_panel",
    )
    candy_specs = [
        (-0.115, 0.515, yellow),
        (-0.050, 0.560, blue),
        (0.030, 0.520, pink),
        (0.105, 0.575, green),
        (-0.090, 0.665, orange),
        (-0.010, 0.705, yellow),
        (0.075, 0.665, blue),
        (0.130, 0.735, pink),
    ]
    for idx, (x, z, mat) in enumerate(candy_specs):
        cabinet.visual(
            Cylinder(radius=0.020, length=0.004),
            origin=Origin(xyz=(x, front_y - 0.0078, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=mat,
            name=f"candy_disc_{idx}",
        )

    # Dispensing shelf with lips; it is part of the fixed cabinet and protrudes
    # from the lower front under the hinged customer flap.
    cabinet.visual(
        Box((0.330, 0.092, 0.018)),
        origin=Origin(xyz=(0.0, front_y - 0.046, 0.130)),
        material=metal,
        name="dispensing_shelf",
    )
    cabinet.visual(
        Box((0.330, 0.012, 0.030)),
        origin=Origin(xyz=(0.0, front_y - 0.086, 0.154)),
        material=metal,
        name="shelf_front_lip",
    )
    cabinet.visual(
        Box((0.014, 0.092, 0.030)),
        origin=Origin(xyz=(-0.165, front_y - 0.046, 0.154)),
        material=metal,
        name="shelf_side_lip_0",
    )
    cabinet.visual(
        Box((0.014, 0.092, 0.030)),
        origin=Origin(xyz=(0.165, front_y - 0.046, 0.154)),
        material=metal,
        name="shelf_side_lip_1",
    )
    cabinet.visual(
        Box((0.275, 0.006, 0.118)),
        origin=Origin(xyz=(0.0, front_y + 0.003, 0.225)),
        material=dark,
        name="dispense_shadow",
    )

    # Printed tick marks around the rotary selector.
    for idx, (x, z, sx, sz) in enumerate(
        [
            (0.070, 0.338, 0.026, 0.012),
            (0.100, 0.385, 0.008, 0.030),
            (0.158, 0.392, 0.008, 0.030),
            (0.188, 0.338, 0.026, 0.012),
        ]
    ):
        cabinet.visual(
            Box((sx, 0.004, sz)),
            origin=Origin(xyz=(x, front_y - 0.002, z)),
            material=label,
            name=f"selector_tick_{idx}",
        )

    # Side service door, hinged on the rear vertical edge of the right side.
    service_door = model.part("service_door")
    door_thickness = 0.014
    door_depth = depth - 0.020
    door_height = 0.740
    service_door.visual(
        Box((door_thickness, door_depth, door_height)),
        origin=Origin(xyz=(door_thickness / 2.0, -door_depth / 2.0, door_height / 2.0)),
        material=red,
        name="door_slab",
    )
    service_door.visual(
        Box((0.018, 0.060, 0.120)),
        origin=Origin(xyz=(door_thickness + 0.009, -0.205, 0.405)),
        material=dark,
        name="recessed_pull",
    )
    for idx, z in enumerate((0.095, 0.350, 0.605)):
        service_door.visual(
            Cylinder(radius=0.008, length=0.118),
            origin=Origin(xyz=(door_thickness + 0.008, -0.008, z), rpy=(0.0, 0.0, 0.0)),
            material=metal,
            name=f"hinge_barrel_{idx}",
        )
        service_door.visual(
            Box((0.014, 0.038, 0.070)),
            origin=Origin(xyz=(door_thickness + 0.002, -0.018, z)),
            material=metal,
            name=f"hinge_leaf_{idx}",
        )

    model.articulation(
        "cabinet_to_service_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=service_door,
        origin=Origin(xyz=(width / 2.0, depth / 2.0 - 0.010, 0.080)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=0.0, upper=1.75),
    )

    # Customer dispensing flap: bottom hinged, smoky plastic, opening outward
    # over the shelf.
    flap = model.part("dispensing_flap")
    flap_width = 0.255
    flap_height = 0.105
    flap_thickness = 0.012
    flap.visual(
        Box((flap_width, flap_thickness, flap_height)),
        origin=Origin(xyz=(0.0, -flap_thickness / 2.0, flap_height / 2.0)),
        material=gray,
        name="flap_panel",
    )
    flap.visual(
        Cylinder(radius=0.006, length=flap_width + 0.022),
        origin=Origin(xyz=(0.0, -flap_thickness / 2.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="flap_hinge_barrel",
    )
    model.articulation(
        "cabinet_to_dispensing_flap",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=flap,
        origin=Origin(xyz=(0.0, front_y, 0.170)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.05),
    )

    # Continuous front selection knob on a shaft normal to the front panel.
    knob = model.part("selection_knob")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.070,
            0.036,
            body_style="faceted",
            base_diameter=0.074,
            top_diameter=0.058,
            edge_radius=0.0012,
            grip=KnobGrip(style="ribbed", count=16, depth=0.0013, width=0.0020),
            indicator=KnobIndicator(style="wedge", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "selection_knob_cap",
    )
    knob.visual(knob_mesh, origin=Origin(), material=dark, name="knob_cap")
    model.articulation(
        "cabinet_to_selection_knob",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=knob,
        origin=Origin(xyz=(0.130, front_y, 0.340), rpy=(math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    service_door = object_model.get_part("service_door")
    flap = object_model.get_part("dispensing_flap")
    knob = object_model.get_part("selection_knob")
    door_joint = object_model.get_articulation("cabinet_to_service_door")
    flap_joint = object_model.get_articulation("cabinet_to_dispensing_flap")
    knob_joint = object_model.get_articulation("cabinet_to_selection_knob")

    ctx.check(
        "selection knob is continuous",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={knob_joint.articulation_type}",
    )

    with ctx.pose({door_joint: 0.0, flap_joint: 0.0, knob_joint: 0.0}):
        ctx.expect_gap(
            service_door,
            cabinet,
            axis="x",
            max_gap=0.003,
            max_penetration=0.0,
            name="service door sits on side face",
        )
        ctx.expect_overlap(
            service_door,
            cabinet,
            axes="yz",
            min_overlap=0.20,
            name="service door covers side service opening",
        )
        ctx.expect_gap(
            cabinet,
            flap,
            axis="y",
            max_gap=0.003,
            max_penetration=0.0,
            positive_elem="lower_front_panel",
            name="dispensing flap closes at front opening",
        )
        ctx.expect_overlap(
            flap,
            cabinet,
            axes="xz",
            min_overlap=0.090,
            name="flap covers dispensing opening",
        )
        ctx.expect_gap(
            cabinet,
            knob,
            axis="y",
            max_gap=0.003,
            max_penetration=0.0,
            positive_elem="lower_front_panel",
            name="selection knob mounts to front panel",
        )
        closed_door_aabb = ctx.part_world_aabb(service_door)
        closed_flap_aabb = ctx.part_world_aabb(flap)

    with ctx.pose({door_joint: 1.05}):
        open_door_aabb = ctx.part_world_aabb(service_door)
    ctx.check(
        "service door swings outward from side",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][0] > closed_door_aabb[1][0] + 0.080,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    with ctx.pose({flap_joint: 0.85}):
        open_flap_aabb = ctx.part_world_aabb(flap)
    ctx.check(
        "dispensing flap swings outward over shelf",
        closed_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[0][1] < closed_flap_aabb[0][1] - 0.035,
        details=f"closed={closed_flap_aabb}, open={open_flap_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
