from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_rolling_site_toolbox")

    jobsite_yellow = model.material("jobsite_yellow", rgba=(0.96, 0.68, 0.08, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.04, 0.045, 0.045, 1.0))
    rubber = model.material("rubber_black", rgba=(0.01, 0.01, 0.01, 1.0))
    steel = model.material("galvanized_steel", rgba=(0.55, 0.56, 0.54, 1.0))
    latch_red = model.material("red_latch_tabs", rgba=(0.80, 0.04, 0.02, 1.0))

    def hollow_tube_mesh(name: str, outer_radius: float, inner_radius: float, length: float):
        tube = (
            cq.Workplane("XY")
            .circle(outer_radius)
            .circle(inner_radius)
            .extrude(length)
        )
        return mesh_from_cadquery(tube, name, tolerance=0.0008, angular_tolerance=0.08)

    body = model.part("body")

    # Deep molded rolling tub with a smaller upper tool tray compartment.
    body.visual(
        Box((0.84, 0.44, 0.40)),
        origin=Origin(xyz=(0.0, 0.0, 0.29)),
        material=jobsite_yellow,
        name="deep_tub",
    )
    body.visual(
        Box((0.76, 0.40, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.56)),
        material=jobsite_yellow,
        name="upper_compartment",
    )
    body.visual(
        Box((0.88, 0.48, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=dark_plastic,
        name="bottom_bumper",
    )
    body.visual(
        Box((0.88, 0.048, 0.46)),
        origin=Origin(xyz=(0.0, 0.246, 0.34)),
        material=dark_plastic,
        name="side_rail_0",
    )
    body.visual(
        Box((0.88, 0.048, 0.46)),
        origin=Origin(xyz=(0.0, -0.246, 0.34)),
        material=dark_plastic,
        name="side_rail_1",
    )
    for i, x in enumerate((-0.28, 0.0, 0.28)):
        body.visual(
            Box((0.035, 0.492, 0.34)),
            origin=Origin(xyz=(x, 0.0, 0.32)),
            material=dark_plastic,
            name=f"vertical_rib_{i}",
        )
    body.visual(
        Box((0.020, 0.420, 0.075)),
        origin=Origin(xyz=(-0.430, 0.0, 0.625)),
        material=dark_plastic,
        name="top_rear_hinge_band",
    )

    # Raised front frame around the lower drop-down access door.
    body.visual(
        Box((0.024, 0.405, 0.030)),
        origin=Origin(xyz=(0.431, 0.0, 0.468)),
        material=dark_plastic,
        name="door_top_jamb",
    )
    body.visual(
        Box((0.024, 0.030, 0.245)),
        origin=Origin(xyz=(0.431, 0.188, 0.345)),
        material=dark_plastic,
        name="door_jamb_0",
    )
    body.visual(
        Box((0.024, 0.030, 0.245)),
        origin=Origin(xyz=(0.431, -0.188, 0.345)),
        material=dark_plastic,
        name="door_jamb_1",
    )
    body.visual(
        Box((0.012, 0.300, 0.190)),
        origin=Origin(xyz=(0.425, 0.0, 0.342)),
        material=dark_plastic,
        name="access_bay_shadow",
    )
    body.visual(
        Box((0.010, 0.270, 0.018)),
        origin=Origin(xyz=(0.428, 0.0, 0.355)),
        material=steel,
        name="access_bay_shelf",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.430),
        origin=Origin(xyz=(0.438, 0.0, 0.222), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="door_hinge_pin",
    )
    for i, y in enumerate((-0.205, 0.205)):
        body.visual(
            Box((0.026, 0.034, 0.040)),
            origin=Origin(xyz=(0.426, y, 0.222)),
            material=dark_plastic,
            name=f"door_hinge_leaf_{i}",
        )

    # Rear tow-handle guide tubes: real hollow sleeves with side saddle ears.
    guide_mesh = hollow_tube_mesh("rear_guide_tube", 0.028, 0.018, 0.44)
    for i, y in enumerate((-0.160, 0.160)):
        body.visual(
            guide_mesh,
            origin=Origin(xyz=(-0.455, y, 0.320)),
            material=dark_plastic,
            name=f"guide_tube_{i}",
        )
        for z in (0.425, 0.665):
            for side in (-1.0, 1.0):
                body.visual(
                    Box((0.038, 0.012, 0.055)),
                    origin=Origin(xyz=(-0.436, y + side * 0.033, z)),
                    material=dark_plastic,
                    name=f"guide_saddle_{i}_{int(z * 1000)}_{0 if side < 0 else 1}",
                )

    # Wheel mounts and front skid feet.
    for i, y in enumerate((-0.256, 0.256)):
        body.visual(
            Box((0.115, 0.038, 0.075)),
            origin=Origin(xyz=(-0.325, y, 0.105)),
            material=dark_plastic,
            name=f"wheel_yoke_{i}",
        )
        body.visual(
            Cylinder(radius=0.010, length=0.070),
            origin=Origin(
                xyz=(-0.325, -0.288 if y < 0.0 else 0.288, 0.105),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=steel,
            name=f"axle_stub_{i}",
        )
    for i, y in enumerate((-0.145, 0.145)):
        body.visual(
            Box((0.155, 0.095, 0.065)),
            origin=Origin(xyz=(0.295, y, 0.058)),
            material=dark_plastic,
            name=f"front_foot_{i}",
        )

    # Telescoping tow handle sliding in the rear tubes.
    tow_handle = model.part("tow_handle")
    for i, y in enumerate((-0.160, 0.160)):
        tow_handle.visual(
            Cylinder(radius=0.014, length=0.620),
            origin=Origin(xyz=(0.0, y, -0.190)),
            material=steel,
            name=f"inner_rod_{i}",
        )
    tow_handle.visual(
        Cylinder(radius=0.026, length=0.405),
        origin=Origin(xyz=(0.0, 0.0, 0.158), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="grip_bar",
    )
    tow_handle.visual(
        Box((0.055, 0.345, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.122)),
        material=dark_plastic,
        name="handle_bridge",
    )
    for i, y in enumerate((-0.160, 0.160)):
        tow_handle.visual(
            Cylinder(radius=0.026, length=0.012),
            origin=Origin(xyz=(0.0, y, 0.006)),
            material=dark_plastic,
            name=f"stop_collar_{i}",
        )

    tow_slide = model.articulation(
        "body_to_tow_handle",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tow_handle,
        origin=Origin(xyz=(-0.455, 0.0, 0.760)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.420),
    )

    # Front lower access door, with molded ribs, latch pads, and hinge barrel.
    access_door = model.part("access_door")
    access_door.visual(
        Box((0.030, 0.324, 0.220)),
        origin=Origin(xyz=(0.017, 0.0, 0.112)),
        material=dark_plastic,
        name="door_panel",
    )
    access_door.visual(
        Box((0.010, 0.280, 0.024)),
        origin=Origin(xyz=(0.036, 0.0, 0.190)),
        material=jobsite_yellow,
        name="door_upper_rib",
    )
    access_door.visual(
        Box((0.010, 0.280, 0.024)),
        origin=Origin(xyz=(0.036, 0.0, 0.060)),
        material=jobsite_yellow,
        name="door_lower_rib",
    )
    access_door.visual(
        Cylinder(radius=0.014, length=0.340),
        origin=Origin(xyz=(0.018, 0.0, 0.000), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="door_hinge_barrel",
    )
    for i, y in enumerate((-0.105, 0.105)):
        access_door.visual(
            Box((0.014, 0.050, 0.040)),
            origin=Origin(xyz=(0.044, y, 0.207)),
            material=latch_red,
            name=f"door_latch_pad_{i}",
        )

    door_hinge = model.articulation(
        "body_to_access_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=access_door,
        origin=Origin(xyz=(0.430, 0.0, 0.225)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=0.0, upper=1.75),
    )

    # Upper tray lid hinged at the rear of the toolbox.
    tray_lid = model.part("tray_lid")
    tray_lid.visual(
        Box((0.755, 0.390, 0.034)),
        origin=Origin(xyz=(0.378, 0.0, 0.017)),
        material=jobsite_yellow,
        name="lid_slab",
    )
    tray_lid.visual(
        Box((0.690, 0.030, 0.043)),
        origin=Origin(xyz=(0.380, 0.195, 0.043)),
        material=dark_plastic,
        name="lid_side_lip_0",
    )
    tray_lid.visual(
        Box((0.690, 0.030, 0.043)),
        origin=Origin(xyz=(0.380, -0.195, 0.043)),
        material=dark_plastic,
        name="lid_side_lip_1",
    )
    tray_lid.visual(
        Box((0.040, 0.390, 0.043)),
        origin=Origin(xyz=(0.735, 0.0, 0.043)),
        material=dark_plastic,
        name="lid_front_lip",
    )
    tray_lid.visual(
        Cylinder(radius=0.014, length=0.395),
        origin=Origin(xyz=(-0.012, 0.0, 0.022), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="lid_hinge_barrel",
    )
    for i, y in enumerate((-0.115, 0.115)):
        tray_lid.visual(
            Box((0.045, 0.055, 0.026)),
            origin=Origin(xyz=(0.720, y, 0.064)),
            material=latch_red,
            name=f"lid_latch_tab_{i}",
        )

    lid_hinge = model.articulation(
        "body_to_tray_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=tray_lid,
        origin=Origin(xyz=(-0.378, 0.0, 0.640)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=1.5, lower=0.0, upper=1.25),
    )

    # Large rear wheels with rubber tires, hub, rim, spokes, and rotating joints.
    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.074,
            0.058,
            rim=WheelRim(inner_radius=0.045, flange_height=0.006, flange_thickness=0.004),
            hub=WheelHub(
                radius=0.024,
                width=0.044,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.030, hole_diameter=0.004),
            ),
            face=WheelFace(dish_depth=0.006, front_inset=0.004, rear_inset=0.003),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.004, window_radius=0.013),
            bore=WheelBore(style="round", diameter=0.013),
        ),
        "rear_wheel_rim",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.105,
            0.065,
            inner_radius=0.070,
            carcass=TireCarcass(belt_width_ratio=0.68, sidewall_bulge=0.05),
            tread=TireTread(style="block", depth=0.007, count=18, land_ratio=0.56),
            grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.003),),
            sidewall=TireSidewall(style="square", bulge=0.025),
            shoulder=TireShoulder(width=0.007, radius=0.003),
        ),
        "rear_wheel_tire",
    )

    for i, y in enumerate((-0.310, 0.310)):
        wheel = model.part(f"rear_wheel_{i}")
        wheel.visual(tire_mesh, material=rubber, name="tire")
        wheel.visual(wheel_mesh, material=steel, name="rim")
        model.articulation(
            f"body_to_rear_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(-0.325, y, 0.105), rpy=(0.0, 0.0, math.pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=20.0),
        )

    # Keep references used by tests easy to inspect in probes.
    model.meta["primary_mechanisms"] = {
        "tow_slide": tow_slide.name,
        "door_hinge": door_hinge.name,
        "lid_hinge": lid_hinge.name,
    }
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    tow_handle = object_model.get_part("tow_handle")
    access_door = object_model.get_part("access_door")
    tray_lid = object_model.get_part("tray_lid")
    rear_wheel_0 = object_model.get_part("rear_wheel_0")
    rear_wheel_1 = object_model.get_part("rear_wheel_1")

    tow_slide = object_model.get_articulation("body_to_tow_handle")
    door_hinge = object_model.get_articulation("body_to_access_door")
    lid_hinge = object_model.get_articulation("body_to_tray_lid")
    wheel_spin_0 = object_model.get_articulation("body_to_rear_wheel_0")
    wheel_spin_1 = object_model.get_articulation("body_to_rear_wheel_1")

    # Captured hinge/axle shafts intentionally pass through their mating barrels
    # or hubs; this is the local interpenetration used to represent the retained
    # mechanical joint.
    ctx.allow_overlap(
        access_door,
        body,
        elem_a="door_hinge_barrel",
        elem_b="door_hinge_pin",
        reason="The steel hinge pin is intentionally captured inside the drop-down door barrel.",
    )
    ctx.expect_overlap(
        access_door,
        body,
        axes="y",
        elem_a="door_hinge_barrel",
        elem_b="door_hinge_pin",
        min_overlap=0.30,
        name="drop door hinge pin runs through the barrel",
    )
    ctx.expect_overlap(
        access_door,
        body,
        axes="xz",
        elem_a="door_hinge_barrel",
        elem_b="door_hinge_pin",
        min_overlap=0.013,
        name="drop door hinge pin is concentric with the barrel",
    )

    for wheel, stub_name in ((rear_wheel_0, "axle_stub_0"), (rear_wheel_1, "axle_stub_1")):
        ctx.allow_overlap(
            body,
            wheel,
            elem_a=stub_name,
            elem_b="rim",
            reason="The fixed axle stub is intentionally captured in the wheel hub bore.",
        )
        ctx.expect_overlap(
            body,
            wheel,
            axes="y",
            elem_a=stub_name,
            elem_b="rim",
            min_overlap=0.025,
            name=f"{wheel.name} is retained on its axle stub",
        )
        ctx.expect_overlap(
            body,
            wheel,
            axes="xz",
            elem_a=stub_name,
            elem_b="rim",
            min_overlap=0.012,
            name=f"{wheel.name} axle is centered in the hub",
        )

    # Closed-state seating and fit checks for the visible mechanisms.
    ctx.expect_gap(
        access_door,
        body,
        axis="x",
        positive_elem="door_panel",
        negative_elem="deep_tub",
        min_gap=0.004,
        max_gap=0.018,
        name="access door sits proud of the front tub wall",
    )
    ctx.expect_overlap(
        access_door,
        body,
        axes="yz",
        elem_a="door_panel",
        elem_b="deep_tub",
        min_overlap=0.20,
        name="access door covers the lower front opening area",
    )
    ctx.expect_gap(
        tray_lid,
        body,
        axis="z",
        positive_elem="lid_slab",
        negative_elem="upper_compartment",
        max_gap=0.001,
        max_penetration=0.0,
        name="upper tray lid rests on the upper compartment",
    )
    ctx.expect_within(
        tray_lid,
        body,
        axes="xy",
        inner_elem="lid_slab",
        outer_elem="upper_compartment",
        margin=0.005,
        name="upper tray lid stays within the tray footprint",
    )

    for rod, tube in (("inner_rod_0", "guide_tube_0"), ("inner_rod_1", "guide_tube_1")):
        ctx.expect_within(
            tow_handle,
            body,
            axes="xy",
            inner_elem=rod,
            outer_elem=tube,
            margin=0.002,
            name=f"{rod} is centered in its rear guide tube",
        )
        ctx.expect_overlap(
            tow_handle,
            body,
            axes="z",
            elem_a=rod,
            elem_b=tube,
            min_overlap=0.25,
            name=f"{rod} has deep collapsed insertion",
        )

    with ctx.pose({tow_slide: 0.420}):
        for rod, tube in (("inner_rod_0", "guide_tube_0"), ("inner_rod_1", "guide_tube_1")):
            ctx.expect_within(
                tow_handle,
                body,
                axes="xy",
                inner_elem=rod,
                outer_elem=tube,
                margin=0.002,
                name=f"{rod} remains centered when extended",
            )
            ctx.expect_overlap(
                tow_handle,
                body,
                axes="z",
                elem_a=rod,
                elem_b=tube,
                min_overlap=0.075,
                name=f"{rod} retains insertion at full handle extension",
            )

    rest_handle_pos = ctx.part_world_position(tow_handle)
    with ctx.pose({tow_slide: 0.420}):
        extended_handle_pos = ctx.part_world_position(tow_handle)
    ctx.check(
        "tow handle extends upward on rear guide tubes",
        rest_handle_pos is not None
        and extended_handle_pos is not None
        and extended_handle_pos[2] > rest_handle_pos[2] + 0.40,
        details=f"rest={rest_handle_pos}, extended={extended_handle_pos}",
    )

    closed_door_aabb = ctx.part_element_world_aabb(access_door, elem="door_panel")
    with ctx.pose({door_hinge: 1.35}):
        open_door_aabb = ctx.part_element_world_aabb(access_door, elem="door_panel")
    ctx.check(
        "access door drops outward and downward",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][0] > closed_door_aabb[1][0] + 0.14
        and open_door_aabb[0][2] < closed_door_aabb[0][2] - 0.02,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(tray_lid, elem="lid_slab")
    with ctx.pose({lid_hinge: 1.05}):
        open_lid_aabb = ctx.part_element_world_aabb(tray_lid, elem="lid_slab")
    ctx.check(
        "upper tray lid rotates upward on rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.25,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    ctx.check(
        "primary mechanisms use the requested joint types",
        tow_slide.articulation_type == ArticulationType.PRISMATIC
        and door_hinge.articulation_type == ArticulationType.REVOLUTE
        and lid_hinge.articulation_type == ArticulationType.REVOLUTE
        and wheel_spin_0.articulation_type == ArticulationType.CONTINUOUS
        and wheel_spin_1.articulation_type == ArticulationType.CONTINUOUS,
        details=(
            f"tow={tow_slide.articulation_type}, door={door_hinge.articulation_type}, "
            f"lid={lid_hinge.articulation_type}, wheels="
            f"{wheel_spin_0.articulation_type}/{wheel_spin_1.articulation_type}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
