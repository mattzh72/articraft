from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)


def _cup_shell() -> cq.Workplane:
    """Open stainless chamber cup with a thick bottom, rolled rim, and bayonet lugs."""
    inner_radius = 0.072
    outer_radius = 0.080
    bottom = cq.Workplane("XY").circle(inner_radius).extrude(0.030)
    wall = _annular_cylinder(outer_radius, inner_radius, 0.232).translate((0.0, 0.0, 0.020))
    rolled_rim = _annular_cylinder(0.084, 0.070, 0.010).translate((0.0, 0.0, 0.250))
    lower_flange = _annular_cylinder(0.088, 0.056, 0.014)

    cup = bottom.union(wall).union(rolled_rim).union(lower_flange)
    for angle_deg in (0.0, 120.0, 240.0):
        lug = (
            cq.Workplane("XY")
            .box(0.018, 0.040, 0.010)
            .translate((0.092, 0.0, 0.012))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        )
        cup = cup.union(lug)
    return cup


def _rounded_housing() -> cq.Workplane:
    return cq.Workplane("XY").box(0.280, 0.320, 0.550).edges("|Z").fillet(0.018)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="laboratory_homogenizer_blender")

    powder_coat = model.material("warm_gray_powder_coat", rgba=(0.72, 0.74, 0.73, 1.0))
    dark_panel = model.material("dark_control_panel", rgba=(0.015, 0.018, 0.022, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.68, 0.70, 0.69, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.005, 0.005, 0.006, 1.0))
    clear_lid = model.material("clear_polycarbonate", rgba=(0.58, 0.82, 0.95, 0.46))
    yellow_mark = model.material("yellow_lock_mark", rgba=(1.0, 0.76, 0.08, 1.0))
    red_indicator = model.material("red_indicator", rgba=(0.80, 0.04, 0.03, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_rounded_housing(), "rounded_housing", tolerance=0.0012),
        origin=Origin(xyz=(0.0, 0.0, 0.275)),
        material=powder_coat,
        name="housing_shell",
    )
    housing.visual(
        Box((0.205, 0.006, 0.255)),
        origin=Origin(xyz=(0.0, -0.163, 0.330)),
        material=dark_panel,
        name="control_panel",
    )
    housing.visual(
        mesh_from_cadquery(_annular_cylinder(0.105, 0.080, 0.015), "bayonet_socket"),
        origin=Origin(xyz=(0.0, -0.020, 0.550)),
        material=stainless,
        name="bayonet_socket",
    )
    for idx, angle in enumerate((0.0, 120.0, 240.0)):
        radius = 0.094
        x = radius * math.cos(math.radians(angle))
        y = -0.020 + radius * math.sin(math.radians(angle))
        housing.visual(
            Box((0.014, 0.050, 0.001)),
            origin=Origin(xyz=(x, y, 0.5645), rpy=(0.0, 0.0, math.radians(angle))),
            material=dark_panel,
            name=f"bayonet_slot_{idx}",
        )
    housing.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(xyz=(0.0, -0.020, 0.555)),
        material=stainless,
        name="drive_coupler",
    )
    housing.visual(
        Box((0.004, 0.028, 0.018)),
        origin=Origin(xyz=(0.085, -0.168, 0.444)),
        material=yellow_mark,
        name="lock_tick",
    )

    chamber_cup = model.part("chamber_cup")
    chamber_cup.visual(
        mesh_from_cadquery(_cup_shell(), "stainless_chamber_cup", tolerance=0.0008),
        material=stainless,
        name="cup_shell",
    )
    chamber_cup.visual(
        Box((0.020, 0.008, 0.045)),
        origin=Origin(xyz=(0.0, -0.083, 0.125)),
        material=black_rubber,
        name="grip_strip",
    )

    lid_hinge = model.part("lid_hinge")
    lid_hinge.visual(
        Box((0.150, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, 0.089, 0.248)),
        material=stainless,
        name="hinge_bridge",
    )
    lid_hinge.visual(
        Box((0.025, 0.012, 0.034)),
        origin=Origin(xyz=(-0.060, 0.089, 0.267)),
        material=stainless,
        name="hinge_ear_0",
    )
    lid_hinge.visual(
        Box((0.025, 0.012, 0.034)),
        origin=Origin(xyz=(0.060, 0.089, 0.267)),
        material=stainless,
        name="hinge_ear_1",
    )

    chamber_lid = model.part("chamber_lid")
    chamber_lid.visual(
        Cylinder(radius=0.080, length=0.012),
        origin=Origin(xyz=(0.0, -0.088, -0.002)),
        material=clear_lid,
        name="lid_disk",
    )
    chamber_lid.visual(
        mesh_from_cadquery(_annular_cylinder(0.080, 0.066, 0.004), "lid_gasket"),
        origin=Origin(xyz=(0.0, -0.088, -0.008)),
        material=black_rubber,
        name="lid_gasket",
    )
    chamber_lid.visual(
        Cylinder(radius=0.008, length=0.070),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="hinge_barrel",
    )
    chamber_lid.visual(
        Box((0.070, 0.034, 0.010)),
        origin=Origin(xyz=(0.0, -0.022, -0.001)),
        material=clear_lid,
        name="hinge_leaf",
    )
    chamber_lid.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(xyz=(0.0, -0.088, 0.011)),
        material=black_rubber,
        name="lid_knob",
    )

    blade_drive = model.part("blade_drive")
    blade_drive.visual(
        Cylinder(radius=0.005, length=0.044),
        material=stainless,
        name="drive_shaft",
    )
    blade_drive.visual(
        Cylinder(radius=0.016, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=stainless,
        name="blade_hub",
    )
    for idx, yaw in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        blade_drive.visual(
            Box((0.052, 0.014, 0.004)),
            origin=Origin(xyz=(0.033 * math.cos(yaw), 0.033 * math.sin(yaw), 0.012), rpy=(0.0, 0.0, yaw)),
            material=stainless,
            name=f"blade_leaf_{idx}",
        )

    speed_dial = model.part("speed_dial")
    speed_dial.visual(
        Cylinder(radius=0.029, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=black_rubber,
        name="dial_cap",
    )
    speed_dial.visual(
        Box((0.004, 0.023, 0.002)),
        origin=Origin(xyz=(0.0, 0.020, 0.019)),
        material=yellow_mark,
        name="dial_pointer",
    )

    status_light = model.part("status_light")
    status_light.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=red_indicator,
        name="indicator_lens",
    )

    cup_twist = model.articulation(
        "housing_to_chamber_cup",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=chamber_cup,
        origin=Origin(xyz=(0.0, -0.020, 0.565)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=0.0, upper=0.42),
    )
    model.articulation(
        "cup_to_lid_hinge",
        ArticulationType.FIXED,
        parent=chamber_cup,
        child=lid_hinge,
    )
    model.articulation(
        "hinge_to_lid",
        ArticulationType.REVOLUTE,
        parent=lid_hinge,
        child=chamber_lid,
        origin=Origin(xyz=(0.0, 0.088, 0.268)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.4, lower=0.0, upper=1.25),
    )
    model.articulation(
        "cup_to_blade",
        ArticulationType.CONTINUOUS,
        parent=chamber_cup,
        child=blade_drive,
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=80.0),
    )
    model.articulation(
        "housing_to_speed_dial",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=speed_dial,
        origin=Origin(xyz=(0.0, -0.166, 0.325), rpy=(math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=4.0, lower=0.0, upper=4.7),
    )
    model.articulation(
        "housing_to_status_light",
        ArticulationType.FIXED,
        parent=housing,
        child=status_light,
        origin=Origin(xyz=(-0.065, -0.166, 0.405), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    cup_twist.meta["description"] = "Limited revolute bayonet twist lock for the removable stainless chamber cup."
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    cup = object_model.get_part("chamber_cup")
    lid_hinge = object_model.get_part("lid_hinge")
    lid = object_model.get_part("chamber_lid")
    blade = object_model.get_part("blade_drive")
    dial = object_model.get_part("speed_dial")

    cup_joint = object_model.get_articulation("housing_to_chamber_cup")
    lid_joint = object_model.get_articulation("hinge_to_lid")
    blade_joint = object_model.get_articulation("cup_to_blade")
    dial_joint = object_model.get_articulation("housing_to_speed_dial")

    housing_aabb = ctx.part_world_aabb(housing)
    ctx.check(
        "housing is tall rectangular appliance body",
        housing_aabb is not None
        and (housing_aabb[1][2] - housing_aabb[0][2]) > 0.50
        and (housing_aabb[1][0] - housing_aabb[0][0]) < 0.34,
        details=f"housing_aabb={housing_aabb}",
    )

    ctx.check(
        "cup uses limited revolute bayonet twist",
        cup_joint.articulation_type == ArticulationType.REVOLUTE
        and cup_joint.motion_limits is not None
        and cup_joint.motion_limits.lower == 0.0
        and cup_joint.motion_limits.upper is not None
        and 0.30 <= cup_joint.motion_limits.upper <= 0.55,
        details=f"type={cup_joint.articulation_type}, limits={cup_joint.motion_limits}",
    )
    ctx.expect_gap(
        cup,
        housing,
        axis="z",
        positive_elem="cup_shell",
        negative_elem="bayonet_socket",
        max_gap=0.0015,
        max_penetration=0.0005,
        name="cup flange seats on bayonet socket",
    )
    ctx.expect_overlap(
        cup,
        housing,
        axes="xy",
        elem_a="cup_shell",
        elem_b="bayonet_socket",
        min_overlap=0.12,
        name="cup remains centered over socket footprint",
    )
    with ctx.pose({cup_joint: 0.42}):
        ctx.expect_gap(
            cup,
            housing,
            axis="z",
            positive_elem="cup_shell",
            negative_elem="bayonet_socket",
            max_gap=0.0015,
            max_penetration=0.0005,
            name="twisted cup stays seated on socket",
        )

    ctx.allow_overlap(
        cup,
        lid_hinge,
        elem_a="cup_shell",
        elem_b="hinge_bridge",
        reason="The welded rear hinge bridge is intentionally seated into the stainless cup rim for a rigid mount.",
    )
    ctx.expect_overlap(
        lid_hinge,
        cup,
        axes="xz",
        elem_a="hinge_bridge",
        elem_b="cup_shell",
        min_overlap=0.015,
        name="hinge bridge is embedded along the rear cup rim",
    )

    ctx.expect_contact(
        lid,
        cup,
        elem_a="lid_gasket",
        elem_b="cup_shell",
        contact_tol=0.002,
        name="closed lid gasket sits on cup rim",
    )
    ctx.expect_overlap(
        lid,
        cup,
        axes="xy",
        elem_a="lid_disk",
        elem_b="cup_shell",
        min_overlap=0.13,
        name="closed lid covers the chamber opening",
    )
    closed_lid = ctx.part_element_world_aabb(lid, elem="lid_disk")
    with ctx.pose({lid_joint: 1.10}):
        open_lid = ctx.part_element_world_aabb(lid, elem="lid_disk")
    ctx.check(
        "lid hinge opens upward from rear edge",
        closed_lid is not None and open_lid is not None and open_lid[1][2] > closed_lid[1][2] + 0.08,
        details=f"closed={closed_lid}, open={open_lid}",
    )

    ctx.check(
        "blade drive is continuous about chamber axis",
        blade_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(blade_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={blade_joint.articulation_type}, axis={blade_joint.axis}",
    )
    ctx.expect_within(
        blade,
        cup,
        axes="xy",
        margin=0.003,
        name="rotating blade stays inside cup diameter",
    )
    ctx.allow_overlap(
        blade,
        cup,
        elem_a="drive_shaft",
        elem_b="cup_shell",
        reason="The rotating blade shaft is intentionally captured through the cup's sealed bottom bearing proxy.",
    )
    ctx.expect_overlap(
        blade,
        cup,
        axes="z",
        elem_a="drive_shaft",
        elem_b="cup_shell",
        min_overlap=0.030,
        name="blade shaft remains inserted through bottom bearing",
    )

    ctx.check(
        "front speed dial has realistic limited rotation",
        dial_joint.articulation_type == ArticulationType.REVOLUTE
        and dial_joint.motion_limits is not None
        and 4.0 <= (dial_joint.motion_limits.upper or 0.0) <= 5.2,
        details=f"type={dial_joint.articulation_type}, limits={dial_joint.motion_limits}",
    )
    ctx.expect_contact(
        dial,
        housing,
        elem_a="dial_cap",
        elem_b="control_panel",
        contact_tol=0.001,
        name="speed dial is mounted on front panel",
    )

    return ctx.report()


object_model = build_object_model()
