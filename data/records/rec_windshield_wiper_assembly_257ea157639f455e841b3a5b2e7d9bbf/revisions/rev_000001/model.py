from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _tapered_arm_mesh():
    """Flat stamped wiper arm: broad at the spindle and slimmer at the blade end."""
    profile = [
        (0.020, -0.024),
        (0.145, -0.020),
        (0.640, -0.014),
        (0.675, -0.018),
        (0.675, 0.018),
        (0.640, 0.014),
        (0.145, 0.020),
        (0.020, 0.024),
    ]
    return ExtrudeGeometry(profile, 0.014, center=True, closed=True)


def _rubber_blade_mesh(length: float, top_width: float, lip_width: float, depth: float):
    """Triangular squeegee lip extruded along blade length in local Y."""
    geom = MeshGeometry()
    y0 = -length * 0.5
    y1 = length * 0.5
    top_z = 0.0
    shoulder_z = -depth * 0.38
    apex_z = -depth
    half_top = top_width * 0.5
    half_lip = lip_width * 0.5

    section = [
        (-half_top, top_z),
        (half_top, top_z),
        (half_lip, shoulder_z),
        (0.0, apex_z),
        (-half_lip, shoulder_z),
    ]
    near = [geom.add_vertex(x, y0, z) for x, z in section]
    far = [geom.add_vertex(x, y1, z) for x, z in section]

    count = len(section)
    for index in range(count):
        next_index = (index + 1) % count
        geom.add_face(near[index], near[next_index], far[next_index])
        geom.add_face(near[index], far[next_index], far[index])

    # end caps
    for index in range(1, count - 1):
        geom.add_face(near[0], near[index], near[index + 1])
        geom.add_face(far[0], far[index + 1], far[index])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_arm_windshield_wiper")

    cast_black = model.material("cast_black", rgba=(0.055, 0.058, 0.060, 1.0))
    satin_graphite = model.material("satin_graphite", rgba=(0.20, 0.21, 0.22, 1.0))
    dark_powder = model.material("dark_powder", rgba=(0.10, 0.105, 0.11, 1.0))
    rubber = model.material("rubber_black", rgba=(0.018, 0.018, 0.018, 1.0))
    worn_metal = model.material("worn_metal", rgba=(0.47, 0.48, 0.46, 1.0))

    housing = model.part("motor_housing")
    housing.visual(
        Box((0.230, 0.165, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=cast_black,
        name="mounting_plate",
    )
    housing.visual(
        Cylinder(radius=0.066, length=0.056),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=cast_black,
        name="gear_cover",
    )
    housing.visual(
        Cylinder(radius=0.034, length=0.145),
        origin=Origin(xyz=(0.0, -0.083, 0.047), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_powder,
        name="motor_can",
    )
    housing.visual(
        Box((0.052, 0.030, 0.024)),
        origin=Origin(xyz=(0.0, -0.018, 0.052)),
        material=dark_powder,
        name="motor_neck",
    )
    housing.visual(
        Cylinder(radius=0.027, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.094)),
        material=worn_metal,
        name="spindle_pedestal",
    )
    housing.visual(
        Cylinder(radius=0.014, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.116)),
        material=worn_metal,
        name="spindle_spline",
    )
    housing.visual(
        Box((0.050, 0.014, 0.010)),
        origin=Origin(xyz=(0.065, 0.064, 0.027), rpy=(0.0, 0.0, 0.35)),
        material=worn_metal,
        name="mounting_ear_0",
    )
    housing.visual(
        Box((0.050, 0.014, 0.010)),
        origin=Origin(xyz=(-0.065, 0.064, 0.027), rpy=(0.0, 0.0, -0.35)),
        material=worn_metal,
        name="mounting_ear_1",
    )

    arm = model.part("sweep_arm")
    arm.visual(
        Cylinder(radius=0.040, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=satin_graphite,
        name="hub_clamp",
    )
    arm.visual(
        Cylinder(radius=0.019, length=0.011),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=worn_metal,
        name="retaining_nut",
    )
    arm.visual(
        mesh_from_geometry(_tapered_arm_mesh(), "wiper_tapered_sweep_arm"),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=satin_graphite,
        name="stamped_arm",
    )
    arm.visual(
        Box((0.520, 0.011, 0.006)),
        origin=Origin(xyz=(0.395, 0.0, 0.026)),
        material=dark_powder,
        name="raised_spine",
    )
    arm.visual(
        Box((0.055, 0.050, 0.020)),
        origin=Origin(xyz=(0.648, 0.0, 0.017)),
        material=satin_graphite,
        name="tip_fork",
    )
    arm.visual(
        Cylinder(radius=0.012, length=0.070),
        origin=Origin(xyz=(0.708, 0.0, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=worn_metal,
        name="tip_pin",
    )

    model.articulation(
        "housing_to_arm",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.124)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.2, lower=-1.05, upper=1.05),
    )

    blade = model.part("blade_carrier")
    blade.visual(
        Cylinder(radius=0.019, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_graphite,
        name="carrier_socket",
    )
    blade.visual(
        Box((0.060, 0.082, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.028)),
        material=satin_graphite,
        name="carrier_saddle",
    )
    blade.visual(
        Box((0.034, 0.486, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, -0.047)),
        material=worn_metal,
        name="blade_spine",
    )
    blade.visual(
        mesh_from_geometry(_rubber_blade_mesh(0.520, 0.020, 0.011, 0.050), "wiper_rubber_blade_lip"),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=rubber,
        name="rubber_lip",
    )
    blade.visual(
        Box((0.038, 0.026, 0.028)),
        origin=Origin(xyz=(0.0, -0.250, -0.049)),
        material=dark_powder,
        name="blade_end_cap_0",
    )
    blade.visual(
        Box((0.038, 0.026, 0.028)),
        origin=Origin(xyz=(0.0, 0.250, -0.049)),
        material=dark_powder,
        name="blade_end_cap_1",
    )

    model.articulation(
        "arm_to_blade",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=blade,
        origin=Origin(xyz=(0.708, 0.0, 0.018)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=2.0, lower=-0.34, upper=0.34),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("motor_housing")
    arm = object_model.get_part("sweep_arm")
    blade = object_model.get_part("blade_carrier")
    sweep_joint = object_model.get_articulation("housing_to_arm")
    roll_joint = object_model.get_articulation("arm_to_blade")

    ctx.allow_overlap(
        arm,
        blade,
        elem_a="tip_pin",
        elem_b="carrier_socket",
        reason="The solid tip pin is intentionally captured inside the blade-carrier roll socket.",
    )
    ctx.expect_within(
        arm,
        blade,
        axes="yz",
        inner_elem="tip_pin",
        outer_elem="carrier_socket",
        margin=0.001,
        name="roll pin sits inside socket bore envelope",
    )
    ctx.expect_overlap(
        arm,
        blade,
        axes="x",
        elem_a="tip_pin",
        elem_b="carrier_socket",
        min_overlap=0.045,
        name="roll socket remains captured along arm axis",
    )

    ctx.expect_contact(
        arm,
        housing,
        elem_a="hub_clamp",
        elem_b="spindle_spline",
        contact_tol=0.001,
        name="arm clamp seats on supported spindle",
    )
    ctx.expect_overlap(
        arm,
        housing,
        axes="xy",
        elem_a="hub_clamp",
        elem_b="spindle_spline",
        min_overlap=0.020,
        name="arm hub is centered over spindle",
    )

    arm_aabb = ctx.part_world_aabb(arm)
    blade_aabb = ctx.part_element_world_aabb(blade, elem="rubber_lip")
    if arm_aabb is not None and blade_aabb is not None:
        arm_length = arm_aabb[1][0] - arm_aabb[0][0]
        blade_length = blade_aabb[1][1] - blade_aabb[0][1]
    else:
        arm_length = blade_length = None
    ctx.check(
        "primary arm is longer than secondary blade link",
        arm_length is not None and blade_length is not None and arm_length > blade_length + 0.15,
        details=f"arm_length={arm_length}, blade_length={blade_length}",
    )

    rest_blade_pos = ctx.part_world_position(blade)
    with ctx.pose({sweep_joint: 0.90}):
        swept_blade_pos = ctx.part_world_position(blade)
    ctx.check(
        "spindle joint sweeps blade tip laterally",
        rest_blade_pos is not None
        and swept_blade_pos is not None
        and swept_blade_pos[1] > rest_blade_pos[1] + 0.45,
        details=f"rest={rest_blade_pos}, swept={swept_blade_pos}",
    )

    rest_blade_aabb = ctx.part_world_aabb(blade)
    with ctx.pose({roll_joint: 0.32}):
        rolled_blade_aabb = ctx.part_world_aabb(blade)
    if rest_blade_aabb is not None and rolled_blade_aabb is not None:
        rest_z_span = rest_blade_aabb[1][2] - rest_blade_aabb[0][2]
        rolled_z_span = rolled_blade_aabb[1][2] - rolled_blade_aabb[0][2]
    else:
        rest_z_span = rolled_z_span = None
    ctx.check(
        "carrier roll turns the blade about the arm axis",
        rest_z_span is not None and rolled_z_span is not None and rolled_z_span > rest_z_span + 0.05,
        details=f"rest_z_span={rest_z_span}, rolled_z_span={rolled_z_span}",
    )

    return ctx.report()


object_model = build_object_model()
