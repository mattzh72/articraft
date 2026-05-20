import cadquery as cq
from math import cos, pi, radians, sin, sqrt

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


def _tube_shell(length: float, radius: float, wall: float) -> cq.Workplane:
    """Open annular tube, authored along CadQuery Z and centered on its length."""
    return (
        cq.Workplane("XY")
        .circle(radius)
        .circle(radius - wall)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
    )


def _annular_sleeve(length: float, outer_radius: float, inner_radius: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
    )


def _dovetail_rail(length: float, top_width: float, bottom_width: float, height: float) -> cq.Workplane:
    # A trapezoidal prism extruded along local Y.  The wide face tucks slightly
    # into the optical tube wall so the rail reads as a bonded shoe.
    return (
        cq.Workplane("XZ")
        .polyline(
            [
                (-bottom_width / 2.0, 0.0),
                (bottom_width / 2.0, 0.0),
                (top_width / 2.0, height),
                (-top_width / 2.0, height),
            ]
        )
        .close()
        .extrude(length)
        .translate((0.0, -length / 2.0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="short_f5_newtonian_equatorial")

    matte_black = model.material("matte_black", rgba=(0.005, 0.006, 0.007, 1.0))
    satin_black = model.material("satin_black", rgba=(0.02, 0.022, 0.024, 1.0))
    white_enamel = model.material("white_enamel", rgba=(0.86, 0.88, 0.84, 1.0))
    dark_metal = model.material("dark_anodized_metal", rgba=(0.12, 0.125, 0.13, 1.0))
    steel = model.material("brushed_steel", rgba=(0.55, 0.56, 0.54, 1.0))
    glass = model.material("mirror_glass", rgba=(0.70, 0.78, 0.82, 1.0))
    red = model.material("red_index_marks", rgba=(0.85, 0.06, 0.04, 1.0))

    tube_length = 0.75
    tube_radius = 0.095
    wall = 0.006
    rail_length = 0.46
    rail_height = 0.030
    rail_bottom_z = -0.122

    tripod = model.part("tripod")
    tripod.visual(
        Cylinder(radius=0.040, length=0.72),
        origin=Origin(xyz=(0.0, 0.0, 0.36)),
        material=dark_metal,
        name="pier",
    )
    tripod.visual(
        Cylinder(radius=0.070, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.730)),
        material=dark_metal,
        name="pier_cap",
    )
    # Three broad tripod legs run from the pier foot to rubber pads.
    for idx, yaw in enumerate((0.0, 2.0 * pi / 3.0, 4.0 * pi / 3.0)):
        r = 0.34
        x = r * cos(yaw)
        y = r * sin(yaw)
        tripod.visual(
            Box((0.58, 0.045, 0.040)),
            origin=Origin(
                xyz=(x / 2.0, y / 2.0, 0.080),
                rpy=(0.0, radians(10.0), yaw),
            ),
            material=dark_metal,
            name=f"tripod_leg_{idx}",
        )
        tripod.visual(
            Box((0.12, 0.075, 0.022)),
            origin=Origin(xyz=(x, y, 0.024), rpy=(0.0, 0.0, yaw)),
            material=satin_black,
            name=f"rubber_foot_{idx}",
        )

    # A tilted polar wedge gives the right-ascension housing a real sloped seat.
    latitude_tilt = radians(55.0)  # polar axis is 35 degrees above the horizon
    ra_joint_world = (0.0, 0.0, 0.82)
    polar_axis = (0.0, -sin(latitude_tilt), cos(latitude_tilt))
    wedge_half_thick = 0.018
    wedge_center = (
        ra_joint_world[0] - wedge_half_thick * polar_axis[0],
        ra_joint_world[1] - wedge_half_thick * polar_axis[1],
        ra_joint_world[2] - wedge_half_thick * polar_axis[2],
    )
    tripod.visual(
        Box((0.24, 0.15, 0.036)),
        origin=Origin(xyz=wedge_center, rpy=(latitude_tilt, 0.0, 0.0)),
        material=dark_metal,
        name="polar_wedge",
    )

    ra_axis = model.part("ra_axis")
    ra_axis.visual(
        Cylinder(radius=0.076, length=0.240),
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=satin_black,
        name="polar_housing",
    )
    ra_axis.visual(
        Cylinder(radius=0.052, length=0.260),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=dark_metal,
        name="ra_shaft",
    )
    ra_axis.visual(
        Box((0.082, 0.090, 0.105)),
        origin=Origin(xyz=(0.105, 0.0, 0.120)),
        material=dark_metal,
        name="ra_motor_box",
    )
    ra_axis.visual(
        Cylinder(radius=0.028, length=0.020),
        origin=Origin(xyz=(0.105, -0.054, 0.120), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="ra_motor_cap",
    )
    ra_axis.visual(
        Box((0.105, 0.004, 0.018)),
        origin=Origin(xyz=(0.105, -0.047, 0.120)),
        material=red,
        name="ra_index_mark",
    )

    saddle = model.part("saddle")
    saddle.visual(
        Cylinder(radius=0.055, length=0.260),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_black,
        name="dec_hub",
    )
    saddle.visual(
        Box((0.250, 0.500, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0475)),
        material=dark_metal,
        name="saddle_plate",
    )
    saddle.visual(
        Box((0.024, 0.455, 0.040)),
        origin=Origin(xyz=(0.070, 0.0, 0.085)),
        material=satin_black,
        name="dovetail_jaw_0",
    )
    saddle.visual(
        Box((0.024, 0.455, 0.040)),
        origin=Origin(xyz=(-0.070, 0.0, 0.085)),
        material=satin_black,
        name="dovetail_jaw_1",
    )
    saddle.visual(
        Box((0.035, 0.075, 0.030)),
        origin=Origin(xyz=(0.092, 0.190, 0.093)),
        material=steel,
        name="clamp_block",
    )
    saddle.visual(
        Cylinder(radius=0.018, length=0.028),
        origin=Origin(xyz=(0.113, 0.190, 0.093), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="clamp_knob",
    )
    saddle.visual(
        Cylinder(radius=0.012, length=0.350),
        origin=Origin(xyz=(-0.300, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="counterweight_bar",
    )
    saddle.visual(
        Cylinder(radius=0.060, length=0.070),
        origin=Origin(xyz=(-0.400, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="counterweight",
    )
    saddle.visual(
        Box((0.078, 0.070, 0.060)),
        origin=Origin(xyz=(0.160, -0.165, 0.094)),
        material=dark_metal,
        name="dec_motor_box",
    )

    optical_tube = model.part("optical_tube")
    tube_mesh = mesh_from_cadquery(
        _tube_shell(tube_length, tube_radius, wall),
        "open_newtonian_tube",
        tolerance=0.0008,
        angular_tolerance=0.08,
    )
    optical_tube.visual(
        tube_mesh,
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=white_enamel,
        name="tube_shell",
    )
    optical_tube.visual(
        Cylinder(radius=0.099, length=0.030),
        origin=Origin(xyz=(0.0, tube_length / 2.0 - 0.015, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="front_rim",
    )
    optical_tube.visual(
        Cylinder(radius=0.098, length=0.028),
        origin=Origin(xyz=(0.0, -tube_length / 2.0 + 0.014, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="mirror_cell",
    )
    optical_tube.visual(
        Cylinder(radius=0.073, length=0.006),
        origin=Origin(xyz=(0.0, -tube_length / 2.0 + 0.029, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="primary_mirror",
    )
    # Spider vanes and secondary holder bridge the front aperture.
    optical_tube.visual(
        Box((0.180, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, tube_length / 2.0 - 0.035, 0.0)),
        material=satin_black,
        name="spider_vane_x",
    )
    optical_tube.visual(
        Box((0.006, 0.006, 0.180)),
        origin=Origin(xyz=(0.0, tube_length / 2.0 - 0.035, 0.0)),
        material=satin_black,
        name="spider_vane_z",
    )
    optical_tube.visual(
        Cylinder(radius=0.022, length=0.014),
        origin=Origin(xyz=(0.0, tube_length / 2.0 - 0.035, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="secondary_holder",
    )
    optical_tube.visual(
        Box((0.060, rail_length, rail_height)),
        origin=Origin(xyz=(0.0, 0.0, rail_bottom_z + rail_height / 2.0)),
        material=dark_metal,
        name="dovetail_rail",
    )

    focuser_body = model.part("focuser_body")
    focuser_body.visual(
        Box((0.120, 0.120, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=satin_black,
        name="curved_base_plate",
    )
    focuser_body.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=satin_black,
        name="mounting_foot",
    )
    focuser_body.visual(
        Box((0.016, 0.102, 0.095)),
        origin=Origin(xyz=(0.048, 0.0, 0.062)),
        material=dark_metal,
        name="focuser_cheek_0",
    )
    focuser_body.visual(
        Box((0.016, 0.102, 0.095)),
        origin=Origin(xyz=(-0.048, 0.0, 0.062)),
        material=dark_metal,
        name="focuser_cheek_1",
    )
    focuser_body.visual(
        Box((0.090, 0.016, 0.082)),
        origin=Origin(xyz=(0.0, 0.051, 0.057)),
        material=dark_metal,
        name="focuser_bridge",
    )
    collar_mesh = mesh_from_cadquery(
        _annular_sleeve(0.105, outer_radius=0.041, inner_radius=0.030),
        "focuser_collar",
        tolerance=0.0005,
        angular_tolerance=0.08,
    )
    focuser_body.visual(
        collar_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.076)),
        material=satin_black,
        name="collar",
    )
    focuser_body.visual(
        Box((0.085, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, -0.049, 0.092)),
        material=steel,
        name="rack_window",
    )

    drawtube = model.part("drawtube")
    drawtube.visual(
        Cylinder(radius=0.025, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=satin_black,
        name="drawtube_barrel",
    )
    drawtube.visual(
        Cylinder(radius=0.032, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=dark_metal,
        name="eyepiece_clamp",
    )
    drawtube.visual(
        Box((0.010, 0.006, 0.090)),
        origin=Origin(xyz=(0.0, -0.027, 0.053)),
        material=steel,
        name="rack_strip",
    )
    for idx in range(7):
        drawtube.visual(
            Box((0.018, 0.004, 0.003)),
            origin=Origin(xyz=(0.0, -0.031, 0.012 + idx * 0.010)),
            material=steel,
            name=f"rack_tooth_{idx}",
        )

    focus_knob = model.part("focus_knob")
    focus_knob.visual(
        Cylinder(radius=0.005, length=0.128),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="pinion_shaft",
    )
    focus_knob.visual(
        Cylinder(radius=0.025, length=0.022),
        origin=Origin(xyz=(0.073, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_black,
        name="knob_cap_0",
    )
    focus_knob.visual(
        Cylinder(radius=0.025, length=0.022),
        origin=Origin(xyz=(-0.073, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_black,
        name="knob_cap_1",
    )

    model.articulation(
        "ra_drive",
        ArticulationType.CONTINUOUS,
        parent=tripod,
        child=ra_axis,
        origin=Origin(xyz=ra_joint_world, rpy=(latitude_tilt, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.020),
    )
    model.articulation(
        "declination_axis",
        ArticulationType.REVOLUTE,
        parent=ra_axis,
        child=saddle,
        origin=Origin(xyz=(0.0, 0.0, 0.335)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=32.0, velocity=0.25, lower=-pi / 2.0, upper=pi / 2.0),
    )
    model.articulation(
        "tube_clamp",
        ArticulationType.FIXED,
        parent=saddle,
        child=optical_tube,
        origin=Origin(xyz=(0.0, 0.0, 0.187)),
    )
    model.articulation(
        "focuser_mount",
        ArticulationType.FIXED,
        parent=optical_tube,
        child=focuser_body,
        origin=Origin(xyz=(0.0, 0.190, tube_radius - 0.0005)),
    )
    model.articulation(
        "drawtube_slide",
        ArticulationType.PRISMATIC,
        parent=focuser_body,
        child=drawtube,
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.030, lower=0.0, upper=0.045),
    )
    model.articulation(
        "focus_pinion",
        ArticulationType.CONTINUOUS,
        parent=focuser_body,
        child=focus_knob,
        origin=Origin(xyz=(0.0, -0.055, 0.105)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=5.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    ra = object_model.get_articulation("ra_drive")
    dec = object_model.get_articulation("declination_axis")
    slide = object_model.get_articulation("drawtube_slide")
    pinion = object_model.get_articulation("focus_pinion")

    tube = object_model.get_part("optical_tube")
    saddle = object_model.get_part("saddle")
    focuser = object_model.get_part("focuser_body")
    drawtube = object_model.get_part("drawtube")
    focus_knob = object_model.get_part("focus_knob")

    ctx.allow_overlap(
        drawtube,
        focuser,
        elem_a="rack_strip",
        elem_b="collar",
        reason="The raised focuser rack slides through an uncut slot in the simplified collar proxy.",
    )
    ctx.allow_overlap(
        focus_knob,
        focuser,
        elem_a="pinion_shaft",
        elem_b="focuser_cheek_0",
        reason="The pinion shaft is intentionally captured in the focuser cheek bearing.",
    )
    ctx.allow_overlap(
        focus_knob,
        focuser,
        elem_a="pinion_shaft",
        elem_b="focuser_cheek_1",
        reason="The pinion shaft is intentionally captured in the opposite focuser cheek bearing.",
    )
    ctx.allow_overlap(
        focuser,
        tube,
        elem_a="mounting_foot",
        elem_b="tube_shell",
        reason="A tiny seated focuser foot embed represents the compressed adapter foot on the curved tube.",
    )

    ctx.check(
        "right ascension is continuous",
        ra.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={ra.articulation_type}",
    )
    ctx.check(
        "declination has revolute limits",
        dec.articulation_type == ArticulationType.REVOLUTE
        and dec.motion_limits is not None
        and dec.motion_limits.lower is not None
        and dec.motion_limits.upper is not None
        and dec.motion_limits.lower < 0.0
        and dec.motion_limits.upper > 0.0,
        details=f"type={dec.articulation_type}, limits={dec.motion_limits}",
    )
    ctx.check(
        "focuser drawtube is prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC
        and slide.motion_limits is not None
        and 0.035 <= (slide.motion_limits.upper or 0.0) <= 0.055,
        details=f"type={slide.articulation_type}, limits={slide.motion_limits}",
    )
    ctx.check(
        "pinion knob is rotary",
        pinion.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={pinion.articulation_type}",
    )

    ctx.expect_contact(
        tube,
        saddle,
        elem_a="dovetail_rail",
        elem_b="saddle_plate",
        contact_tol=0.003,
        name="dovetail rail seats in saddle",
    )
    ctx.expect_overlap(
        tube,
        saddle,
        axes="xy",
        elem_a="dovetail_rail",
        elem_b="saddle_plate",
        min_overlap=0.040,
        name="saddle supports the tube footprint",
    )
    ctx.expect_overlap(
        drawtube,
        focuser,
        axes="z",
        elem_a="drawtube_barrel",
        elem_b="collar",
        min_overlap=0.010,
        name="drawtube remains inside focuser collar",
    )
    ctx.expect_overlap(
        drawtube,
        focuser,
        axes="z",
        elem_a="rack_strip",
        elem_b="collar",
        min_overlap=0.030,
        name="rack remains engaged with collar slot",
    )
    ctx.expect_overlap(
        focus_knob,
        focuser,
        axes="x",
        elem_a="pinion_shaft",
        elem_b="focuser_cheek_0",
        min_overlap=0.006,
        name="pinion shaft crosses cheek bearing",
    )
    ctx.expect_overlap(
        focus_knob,
        focuser,
        axes="x",
        elem_a="pinion_shaft",
        elem_b="focuser_cheek_1",
        min_overlap=0.006,
        name="pinion shaft crosses opposite cheek",
    )
    ctx.expect_contact(
        focuser,
        tube,
        elem_a="mounting_foot",
        elem_b="tube_shell",
        contact_tol=0.001,
        name="focuser foot is seated on tube shell",
    )

    rest_pos = ctx.part_world_position(drawtube)
    with ctx.pose({slide: slide.motion_limits.upper}):
        ctx.expect_overlap(
            drawtube,
            focuser,
            axes="z",
            elem_a="drawtube_barrel",
            elem_b="collar",
            min_overlap=0.010,
            name="extended drawtube keeps retained insertion",
        )
        extended_pos = ctx.part_world_position(drawtube)
    if rest_pos is not None and extended_pos is not None:
        travel = sqrt(sum((extended_pos[i] - rest_pos[i]) ** 2 for i in range(3)))
    else:
        travel = 0.0
    ctx.check(
        "drawtube translates by focuser travel",
        travel > 0.035,
        details=f"rest={rest_pos}, extended={extended_pos}, travel={travel}",
    )

    return ctx.report()


object_model = build_object_model()
