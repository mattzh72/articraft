from __future__ import annotations

import math

import cadquery as cq
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


def _open_box(length: float, width: float, height: float, wall: float, bottom_z: float) -> cq.Workplane:
    """A simple open-top sheet-metal tub in model coordinates."""
    outer = cq.Workplane("XY").box(length, width, height).translate((0.0, 0.0, bottom_z + height / 2.0))
    cutter = (
        cq.Workplane("XY")
        .box(length - 2.0 * wall, width - 2.0 * wall, height)
        .translate((0.0, 0.0, bottom_z + wall + height / 2.0))
    )
    return outer.cut(cutter)


def _drawer_pan() -> cq.Workplane:
    """Drawer geometry in the drawer part frame; the frame is at the front pull face."""
    length = 0.360
    width = 0.350
    height = 0.065
    wall = 0.008
    bottom_z = -height / 2.0
    tray_center_x = -0.200
    tray = cq.Workplane("XY").box(length, width, height).translate((tray_center_x, 0.0, 0.0))
    tray_cut = (
        cq.Workplane("XY")
        .box(length - 2.0 * wall, width - 2.0 * wall, height)
        .translate((tray_center_x, 0.0, bottom_z + wall + height / 2.0))
    )
    front = cq.Workplane("XY").box(0.040, 0.410, 0.090).translate((0.0, 0.0, 0.0))
    return tray.cut(tray_cut).union(front)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mechanics_rolling_toolbox")

    red = model.material("powder_coat_red", color=(0.72, 0.04, 0.025, 1.0))
    dark_red = model.material("dark_red_shadow", color=(0.42, 0.015, 0.012, 1.0))
    black = model.material("matte_black", color=(0.015, 0.015, 0.014, 1.0))
    rubber = model.material("black_rubber", color=(0.005, 0.005, 0.004, 1.0))
    steel = model.material("brushed_steel", color=(0.62, 0.62, 0.58, 1.0))
    rail = model.material("dark_runner_steel", color=(0.08, 0.08, 0.075, 1.0))

    body = model.part("lower_bin")

    # Rugged lower rolling bin: an actual open-topped tub is visible when the lid rotates up.
    body.visual(
        mesh_from_cadquery(_open_box(0.820, 0.460, 0.340, 0.026, 0.120), "lower_bin_shell"),
        material=red,
        name="bin_shell",
    )

    # Shallow front drawer housing under the lid.  The middle is deliberately left open.
    body.visual(Box((0.370, 0.050, 0.090)), origin=Origin(xyz=(0.235, 0.230, 0.505)), material=red, name="drawer_side_0")
    body.visual(Box((0.370, 0.050, 0.090)), origin=Origin(xyz=(0.235, -0.230, 0.505)), material=red, name="drawer_side_1")
    body.visual(Box((0.370, 0.460, 0.018)), origin=Origin(xyz=(0.235, 0.000, 0.454)), material=dark_red, name="drawer_floor")
    body.visual(Box((0.370, 0.460, 0.006)), origin=Origin(xyz=(0.235, 0.000, 0.552)), material=red, name="drawer_top_plate")
    body.visual(Box((0.025, 0.460, 0.110)), origin=Origin(xyz=(0.000, 0.000, 0.500)), material=red, name="rear_drawer_frame")
    body.visual(Box((0.020, 0.032, 0.026)), origin=Origin(xyz=(0.240, 0.220, 0.455)), material=rail, name="drawer_runner_0")
    body.visual(Box((0.020, 0.032, 0.026)), origin=Origin(xyz=(0.240, -0.220, 0.455)), material=rail, name="drawer_runner_1")
    body.visual(Box((0.335, 0.020, 0.018)), origin=Origin(xyz=(0.240, 0.220, 0.461)), material=steel, name="straight_runner_0")
    body.visual(Box((0.335, 0.020, 0.018)), origin=Origin(xyz=(0.240, -0.220, 0.461)), material=steel, name="straight_runner_1")

    # Raised shop-toolbox details and bumpers.
    body.visual(Box((0.820, 0.035, 0.055)), origin=Origin(xyz=(0.000, 0.246, 0.310)), material=black, name="side_bumper_0")
    body.visual(Box((0.820, 0.035, 0.055)), origin=Origin(xyz=(0.000, -0.246, 0.310)), material=black, name="side_bumper_1")
    body.visual(Box((0.090, 0.500, 0.090)), origin=Origin(xyz=(-0.410, 0.000, 0.175)), material=black, name="rear_bumper")
    body.visual(Box((0.070, 0.110, 0.080)), origin=Origin(xyz=(0.315, 0.160, 0.080)), material=rubber, name="front_foot_0")
    body.visual(Box((0.070, 0.110, 0.080)), origin=Origin(xyz=(0.315, -0.160, 0.080)), material=rubber, name="front_foot_1")

    # Fixed rear axle and two telescoping handle sleeves mounted to the rear housing.
    body.visual(
        Cylinder(radius=0.015, length=0.610),
        origin=Origin(xyz=(-0.340, 0.000, 0.095), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="rear_axle",
    )
    body.visual(Box((0.045, 0.030, 0.095)), origin=Origin(xyz=(-0.340, 0.218, 0.140)), material=steel, name="axle_bracket_0")
    body.visual(Box((0.045, 0.030, 0.095)), origin=Origin(xyz=(-0.340, -0.218, 0.140)), material=steel, name="axle_bracket_1")
    sleeve_height = 0.500
    sleeve_z = 0.380
    for i, y in enumerate((0.180, -0.180)):
        body.visual(Box((0.010, 0.070, sleeve_height)), origin=Origin(xyz=(-0.460, y, sleeve_z)), material=black, name=f"sleeve_front_{i}")
        body.visual(Box((0.010, 0.070, sleeve_height)), origin=Origin(xyz=(-0.520, y, sleeve_z)), material=black, name=f"sleeve_rear_{i}")
        body.visual(Box((0.050, 0.010, sleeve_height)), origin=Origin(xyz=(-0.490, y + 0.030, sleeve_z)), material=black, name=f"sleeve_side_{i}_0")
        body.visual(Box((0.050, 0.010, sleeve_height)), origin=Origin(xyz=(-0.490, y - 0.030, sleeve_z)), material=black, name=f"sleeve_side_{i}_1")
        body.visual(Box((0.055, 0.055, 0.100)), origin=Origin(xyz=(-0.4375, y, 0.560)), material=black, name=f"upper_sleeve_mount_{i}")
        body.visual(Box((0.055, 0.055, 0.120)), origin=Origin(xyz=(-0.4375, y, 0.230)), material=black, name=f"lower_sleeve_mount_{i}")

    # The rear lip is a hinge landing for the rotating lid.
    body.visual(Box((0.035, 0.430, 0.030)), origin=Origin(xyz=(-0.395, 0.000, 0.528)), material=red, name="rear_hinge_lip")

    drawer = model.part("drawer")
    drawer.visual(mesh_from_cadquery(_drawer_pan(), "drawer_pan"), origin=Origin(), material=dark_red, name="drawer_tray")
    drawer.visual(Box((0.026, 0.260, 0.030)), origin=Origin(xyz=(0.032, 0.000, 0.000)), material=steel, name="drawer_pull")
    drawer.visual(Box((0.270, 0.012, 0.014)), origin=Origin(xyz=(-0.190, 0.180, -0.020)), material=steel, name="drawer_slide_0")
    drawer.visual(Box((0.270, 0.012, 0.014)), origin=Origin(xyz=(-0.190, -0.180, -0.020)), material=steel, name="drawer_slide_1")

    lid = model.part("main_lid")
    lid.visual(Box((0.825, 0.505, 0.040)), origin=Origin(xyz=(0.412, 0.000, 0.020)), material=red, name="lid_panel")
    lid.visual(Box((0.680, 0.045, 0.020)), origin=Origin(xyz=(0.430, 0.185, 0.050)), material=dark_red, name="lid_rib_0")
    lid.visual(Box((0.680, 0.045, 0.020)), origin=Origin(xyz=(0.430, -0.185, 0.050)), material=dark_red, name="lid_rib_1")
    lid.visual(Box((0.045, 0.430, 0.030)), origin=Origin(xyz=(0.620, 0.000, 0.055)), material=black, name="lid_front_latch_bar")
    lid.visual(
        Cylinder(radius=0.014, length=0.300),
        origin=Origin(xyz=(0.000, 0.000, 0.020), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )

    handle = model.part("rear_handle")
    for i, y in enumerate((0.180, -0.180)):
        handle.visual(
            Cylinder(radius=0.012, length=0.560),
            origin=Origin(xyz=(0.000, y, -0.280)),
            material=steel,
            name=f"handle_tube_{i}",
        )
    handle.visual(
        Cylinder(radius=0.020, length=0.430),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="top_grip",
    )
    handle.visual(Box((0.035, 0.055, 0.035)), origin=Origin(xyz=(0.000, 0.180, -0.015)), material=black, name="grip_socket_0")
    handle.visual(Box((0.035, 0.055, 0.035)), origin=Origin(xyz=(0.000, -0.180, -0.015)), material=black, name="grip_socket_1")

    wheel_geom = WheelGeometry(
        0.070,
        0.050,
        rim=WheelRim(inner_radius=0.044, flange_height=0.006, flange_thickness=0.004, bead_seat_depth=0.003),
        hub=WheelHub(
            radius=0.026,
            width=0.040,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=5, circle_diameter=0.034, hole_diameter=0.004),
        ),
        face=WheelFace(dish_depth=0.005, front_inset=0.002, rear_inset=0.002),
        spokes=WheelSpokes(style="straight", count=5, thickness=0.004, window_radius=0.008),
        bore=WheelBore(style="round", diameter=0.030),
    )
    tire_geom = TireGeometry(
        0.095,
        0.058,
        inner_radius=0.066,
        carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.04),
        tread=TireTread(style="block", depth=0.006, count=18, land_ratio=0.58),
        grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.002),),
        sidewall=TireSidewall(style="square", bulge=0.02),
        shoulder=TireShoulder(width=0.006, radius=0.003),
    )
    for i, y in enumerate((0.286, -0.286)):
        wheel = model.part(f"wheel_{i}")
        wheel.visual(mesh_from_geometry(tire_geom, f"tire_{i}"), origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)), material=rubber, name="tire")
        wheel.visual(mesh_from_geometry(wheel_geom, f"wheel_rim_{i}"), origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)), material=steel, name="rim")
        model.articulation(
            f"wheel_{i}_spin",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(-0.340, y, 0.095)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=20.0),
        )

    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.415, 0.000, 0.505)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.35, lower=0.0, upper=0.240),
    )
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.410, 0.000, 0.558)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.6, lower=0.0, upper=1.25),
    )
    model.articulation(
        "handle_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=handle,
        origin=Origin(xyz=(-0.490, 0.000, 0.660)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.30, lower=0.0, upper=0.380),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("lower_bin")
    drawer = object_model.get_part("drawer")
    lid = object_model.get_part("main_lid")
    handle = object_model.get_part("rear_handle")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")

    drawer_slide = object_model.get_articulation("drawer_slide")
    lid_hinge = object_model.get_articulation("lid_hinge")
    handle_slide = object_model.get_articulation("handle_slide")
    wheel_0_spin = object_model.get_articulation("wheel_0_spin")
    wheel_1_spin = object_model.get_articulation("wheel_1_spin")

    for wheel in (wheel_0, wheel_1):
        ctx.allow_overlap(
            body,
            wheel,
            elem_a="rear_axle",
            elem_b="rim",
            reason="The wheel hub is intentionally captured around the solid axle proxy so the rear wheel can spin on its shaft.",
        )
        ctx.expect_overlap(
            body,
            wheel,
            axes="yz",
            elem_a="rear_axle",
            elem_b="rim",
            min_overlap=0.025,
            name=f"{wheel.name} axle passes through hub",
        )

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="drawer_top_plate",
        min_gap=0.001,
        max_gap=0.015,
        name="closed lid sits just above the drawer housing",
    )
    ctx.expect_overlap(
        drawer,
        body,
        axes="x",
        elem_a="drawer_tray",
        elem_b="straight_runner_0",
        min_overlap=0.250,
        name="closed drawer is retained on straight runners",
    )

    drawer_rest = ctx.part_world_position(drawer)
    handle_rest = ctx.part_world_position(handle)
    lid_rest_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")

    with ctx.pose({drawer_slide: 0.240}):
        drawer_ext = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            body,
            axes="x",
            elem_a="drawer_tray",
            elem_b="straight_runner_0",
            min_overlap=0.070,
            name="extended drawer keeps runner engagement",
        )

    with ctx.pose({handle_slide: 0.380}):
        handle_ext = ctx.part_world_position(handle)
        ctx.expect_overlap(
            handle,
            body,
            axes="z",
            elem_a="handle_tube_0",
            elem_b="sleeve_front_0",
            min_overlap=0.150,
            name="extended handle tubes remain captured in rear sleeves",
        )

    with ctx.pose({lid_hinge: 1.10}):
        lid_open_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")

    with ctx.pose({wheel_0_spin: math.tau, wheel_1_spin: -math.tau}):
        ctx.expect_origin_gap(wheel_0, wheel_1, axis="y", min_gap=0.50, max_gap=0.60, name="rear wheels share the axle line")

    ctx.check(
        "front drawer slides outward",
        drawer_rest is not None and drawer_ext is not None and drawer_ext[0] > drawer_rest[0] + 0.220,
        details=f"rest={drawer_rest}, extended={drawer_ext}",
    )
    ctx.check(
        "rear handle extends upward",
        handle_rest is not None and handle_ext is not None and handle_ext[2] > handle_rest[2] + 0.350,
        details=f"rest={handle_rest}, extended={handle_ext}",
    )
    ctx.check(
        "main lid rotates upward",
        lid_rest_aabb is not None
        and lid_open_aabb is not None
        and lid_open_aabb[1][2] > lid_rest_aabb[1][2] + 0.550,
        details=f"closed={lid_rest_aabb}, opened={lid_open_aabb}",
    )
    ctx.check(
        "rear wheels are continuous joints",
        wheel_0_spin.motion_limits.lower is None
        and wheel_0_spin.motion_limits.upper is None
        and wheel_1_spin.motion_limits.lower is None
        and wheel_1_spin.motion_limits.upper is None,
        details="continuous rear wheels should not have finite angle stops",
    )

    return ctx.report()


object_model = build_object_model()
