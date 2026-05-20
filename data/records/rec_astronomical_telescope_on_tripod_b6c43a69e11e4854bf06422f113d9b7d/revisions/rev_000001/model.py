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
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)
import cadquery as cq


def _cylinder_between(part, start, end, radius, material, name, segments: int = 24):
    """Attach a native cylinder whose local +Z axis runs from start to end."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("Cylinder endpoints must be distinct")
    ux, uy, uz = dx / length, dy / length, dz / length
    yaw = math.atan2(uy, ux)
    pitch = math.atan2(math.sqrt(ux * ux + uy * uy), uz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def _annular_tube_mesh(name: str, *, outer_radius: float, inner_radius: float, length: float):
    shape = (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length, both=True)
    )
    return mesh_from_cadquery(shape, name, tolerance=0.0008, angular_tolerance=0.08)


def _dovetail_rail_mesh(name: str, *, length: float, top_width: float, bottom_width: float, height: float):
    half_top = top_width * 0.5
    half_bottom = bottom_width * 0.5
    profile = [
        (-half_bottom, -height * 0.5),
        (half_bottom, -height * 0.5),
        (half_top, height * 0.5),
        (-half_top, height * 0.5),
    ]
    shape = cq.Workplane("YZ").polyline(profile).close().extrude(length, both=True)
    return mesh_from_cadquery(shape, name, tolerance=0.0006, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="video_tracking_newtonian")

    matte_black = model.material("matte_black", rgba=(0.005, 0.006, 0.007, 1.0))
    tube_blue = model.material("deep_blue_tube", rgba=(0.02, 0.055, 0.13, 1.0))
    dark_grey = model.material("dark_grey_metal", rgba=(0.08, 0.085, 0.09, 1.0))
    graphite = model.material("graphite_casting", rgba=(0.16, 0.16, 0.17, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.48, 0.50, 0.52, 1.0))
    orange = model.material("anodized_orange", rgba=(0.95, 0.38, 0.06, 1.0))
    glass = model.material("dark_coated_glass", rgba=(0.03, 0.04, 0.055, 0.72))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))

    # --- Fixed tripod head and legs.  The root frame is on the ground at the
    # azimuth axis; the flat tripod head top is the azimuth bearing plane.
    tripod_head = model.part("tripod_head")
    tripod_head.visual(
        Cylinder(radius=0.16, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.70)),
        material=graphite,
        name="flat_head",
    )
    tripod_head.visual(
        Cylinder(radius=0.070, length=0.38),
        origin=Origin(xyz=(0.0, 0.0, 0.47)),
        material=dark_grey,
        name="center_column",
    )
    tripod_head.visual(
        Cylinder(radius=0.11, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
        material=graphite,
        name="leg_hub",
    )

    leg_tops = []
    leg_mids = []
    foot_centers = []
    for i, angle in enumerate((math.radians(90.0), math.radians(210.0), math.radians(330.0))):
        ca, sa = math.cos(angle), math.sin(angle)
        top = (0.11 * ca, 0.11 * sa, 0.58)
        mid = (0.335 * ca, 0.335 * sa, 0.318)
        foot = (0.56 * ca, 0.56 * sa, 0.055)
        leg_tops.append(top)
        leg_mids.append(mid)
        foot_centers.append(foot)
        _cylinder_between(
            tripod_head,
            top,
            foot,
            0.024,
            dark_grey,
            f"tripod_leg_{i}",
        )
        tripod_head.visual(
            Box((0.16, 0.060, 0.030)),
            origin=Origin(xyz=foot, rpy=(0.0, 0.0, angle)),
            material=rubber,
            name=f"rubber_foot_{i}",
        )
    for i in range(3):
        _cylinder_between(
            tripod_head,
            leg_mids[i],
            leg_mids[(i + 1) % 3],
            0.010,
            satin_metal,
            f"spreader_bar_{i}",
        )

    # --- Azimuth stage: a continuous motorized bearing and one upright arm.
    arm = model.part("azimuth_arm")
    arm.visual(
        Cylinder(radius=0.145, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=graphite,
        name="az_motor_bearing",
    )
    arm.visual(
        Cylinder(radius=0.116, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.081)),
        material=satin_metal,
        name="bearing_trim_ring",
    )
    arm.visual(
        Box((0.13, 0.25, 0.080)),
        origin=Origin(xyz=(0.0, 0.13, 0.075)),
        material=graphite,
        name="arm_foot",
    )
    arm.visual(
        Box((0.105, 0.090, 0.52)),
        origin=Origin(xyz=(0.0, 0.235, 0.33)),
        material=dark_grey,
        name="arm_column",
    )
    arm.visual(
        Box((0.030, 0.12, 0.37)),
        origin=Origin(xyz=(0.062, 0.215, 0.34), rpy=(0.0, 0.0, math.radians(8.0))),
        material=graphite,
        name="outer_rib",
    )
    arm.visual(
        Cylinder(radius=0.104, length=0.090),
        origin=Origin(xyz=(0.0, 0.20, 0.62), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="alt_motor_housing",
    )
    arm.visual(
        Cylinder(radius=0.062, length=0.098),
        origin=Origin(xyz=(0.0, 0.20, 0.62), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="alt_bearing_face",
    )

    model.articulation(
        "azimuth",
        ArticulationType.CONTINUOUS,
        parent=tripod_head,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.74)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2),
    )

    # --- Rotating saddle: a Vixen-style dovetail clamp carried on the altitude
    # axis, with a bridge from the side bearing to the saddle bed.
    saddle = model.part("saddle")
    saddle.visual(
        Cylinder(radius=0.070, length=0.038),
        origin=Origin(xyz=(0.0, -0.068, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="altitude_trunnion",
    )
    saddle.visual(
        Box((0.082, 0.046, 0.100)),
        origin=Origin(xyz=(0.0, -0.076, -0.095)),
        material=graphite,
        name="bearing_neck",
    )
    saddle.visual(
        Box((0.105, 0.112, 0.035)),
        origin=Origin(xyz=(0.0, -0.112, -0.135)),
        material=graphite,
        name="saddle_bridge",
    )
    saddle.visual(
        Box((0.36, 0.13, 0.018)),
        origin=Origin(xyz=(0.0, -0.23, -0.145)),
        material=dark_grey,
        name="saddle_bed",
    )
    saddle.visual(
        Box((0.36, 0.018, 0.045)),
        origin=Origin(xyz=(0.0, -0.166, -0.121)),
        material=dark_grey,
        name="fixed_jaw",
    )
    saddle.visual(
        Box((0.36, 0.018, 0.045)),
        origin=Origin(xyz=(0.0, -0.294, -0.121)),
        material=dark_grey,
        name="clamp_jaw",
    )
    saddle.visual(
        Box((0.12, 0.014, 0.040)),
        origin=Origin(xyz=(0.11, -0.152, -0.121)),
        material=satin_metal,
        name="clamp_pad",
    )

    model.articulation(
        "altitude",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=saddle,
        origin=Origin(xyz=(0.0, 0.20, 0.62)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=65.0, velocity=0.85, lower=-0.20, upper=1.25),
    )

    # --- Short, fast imaging Newtonian.  The main tube is a real hollow shell
    # with an open front, spider vanes, side focuser, camera, and a Vixen rail.
    tube = model.part("tube")
    tube_length = 0.64
    outer_radius = 0.110
    inner_radius = 0.101
    tube.visual(
        _annular_tube_mesh(
            "hollow_newtonian_tube",
            outer_radius=outer_radius,
            inner_radius=inner_radius,
            length=tube_length,
        ),
        origin=Origin(),
        material=tube_blue,
        name="hollow_tube",
    )
    tube.visual(
        _annular_tube_mesh("front_rolled_rim", outer_radius=0.118, inner_radius=0.098, length=0.026),
        origin=Origin(xyz=(tube_length * 0.5 + 0.005, 0.0, 0.0)),
        material=matte_black,
        name="front_rim",
    )
    tube.visual(
        Cylinder(radius=0.108, length=0.018),
        origin=Origin(xyz=(-tube_length * 0.5 - 0.002, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="rear_mirror_cell",
    )
    tube.visual(
        Cylinder(radius=0.084, length=0.006),
        origin=Origin(xyz=(-tube_length * 0.5 + 0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="primary_mirror",
    )
    tube.visual(
        Box((0.008, 0.205, 0.006)),
        origin=Origin(xyz=(tube_length * 0.5 - 0.030, 0.0, 0.0)),
        material=satin_metal,
        name="spider_vane_y",
    )
    tube.visual(
        Box((0.008, 0.006, 0.205)),
        origin=Origin(xyz=(tube_length * 0.5 - 0.030, 0.0, 0.0)),
        material=satin_metal,
        name="spider_vane_z",
    )
    tube.visual(
        Cylinder(radius=0.024, length=0.014),
        origin=Origin(xyz=(tube_length * 0.5 - 0.035, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="secondary_holder",
    )
    tube.visual(
        _dovetail_rail_mesh("vixen_dovetail_rail", length=0.46, top_width=0.050, bottom_width=0.074, height=0.018),
        origin=Origin(xyz=(-0.020, 0.0, -0.127)),
        material=orange,
        name="vixen_rail",
    )
    tube.visual(
        Box((0.070, 0.044, 0.020)),
        origin=Origin(xyz=(-0.18, 0.0, -0.111)),
        material=orange,
        name="rear_rail_foot",
    )
    tube.visual(
        Box((0.070, 0.044, 0.020)),
        origin=Origin(xyz=(0.14, 0.0, -0.111)),
        material=orange,
        name="front_rail_foot",
    )
    tube.visual(
        Box((0.105, 0.085, 0.036)),
        origin=Origin(xyz=(0.16, 0.0, 0.120)),
        material=matte_black,
        name="focuser_base",
    )
    tube.visual(
        Cylinder(radius=0.032, length=0.115),
        origin=Origin(xyz=(0.16, 0.0, 0.185)),
        material=matte_black,
        name="focuser_drawtube",
    )
    tube.visual(
        Cylinder(radius=0.046, length=0.046),
        origin=Origin(xyz=(0.16, 0.0, 0.265)),
        material=matte_black,
        name="camera_collar",
    )
    tube.visual(
        Box((0.078, 0.078, 0.048)),
        origin=Origin(xyz=(0.16, 0.0, 0.312)),
        material=matte_black,
        name="video_camera",
    )
    tube.visual(
        Cylinder(radius=0.008, length=0.18),
        origin=Origin(xyz=(0.265, 0.0, 0.312), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="camera_cable",
    )
    tube.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(xyz=(0.16, 0.0525, 0.145), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="focus_boss",
    )

    model.articulation(
        "saddle_to_tube",
        ArticulationType.FIXED,
        parent=saddle,
        child=tube,
        origin=Origin(xyz=(0.0, -0.23, 0.0)),
    )

    # Visible rotary controls are separate moving parts.
    clamp_knob = model.part("saddle_knob")
    clamp_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.052,
                0.020,
                body_style="lobed",
                base_diameter=0.036,
                top_diameter=0.046,
                grip=KnobGrip(style="ribbed", count=10, depth=0.0015),
                indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
            ),
            "saddle_clamp_knob",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="knob_cap",
    )
    clamp_knob.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.0, -0.005, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="knob_screw",
    )
    model.articulation(
        "saddle_knob_spin",
        ArticulationType.CONTINUOUS,
        parent=saddle,
        child=clamp_knob,
        origin=Origin(xyz=(0.11, -0.135, -0.121)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=8.0),
    )

    focus_knob = model.part("focus_knob")
    focus_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.034,
                0.016,
                body_style="cylindrical",
                grip=KnobGrip(style="fluted", count=18, depth=0.0011),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0006),
            ),
            "focus_knob_cap",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="knob_cap",
    )
    focus_knob.visual(
        Cylinder(radius=0.0045, length=0.008),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="focus_shaft",
    )
    model.articulation(
        "focus_knob_spin",
        ArticulationType.CONTINUOUS,
        parent=tube,
        child=focus_knob,
        origin=Origin(xyz=(0.16, 0.0705, 0.145)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tripod_head = object_model.get_part("tripod_head")
    arm = object_model.get_part("azimuth_arm")
    saddle = object_model.get_part("saddle")
    tube = object_model.get_part("tube")
    azimuth = object_model.get_articulation("azimuth")
    altitude = object_model.get_articulation("altitude")

    ctx.expect_gap(
        arm,
        tripod_head,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="az_motor_bearing",
        negative_elem="flat_head",
        name="azimuth bearing sits on flat tripod head",
    )
    ctx.expect_contact(
        tube,
        saddle,
        elem_a="vixen_rail",
        elem_b="saddle_bed",
        contact_tol=0.0015,
        name="vixen rail is seated in saddle bed",
    )
    ctx.expect_overlap(
        tube,
        saddle,
        axes="x",
        elem_a="vixen_rail",
        elem_b="saddle_bed",
        min_overlap=0.30,
        name="saddle grips a long section of the dovetail rail",
    )
    ctx.expect_overlap(
        tube,
        saddle,
        axes="y",
        elem_a="vixen_rail",
        elem_b="saddle_bed",
        min_overlap=0.070,
        name="dovetail rail is laterally captured by the saddle",
    )
    ctx.check(
        "altitude joint has realistic limited travel",
        altitude.articulation_type == ArticulationType.REVOLUTE
        and altitude.motion_limits is not None
        and altitude.motion_limits.lower <= -0.15
        and altitude.motion_limits.upper >= 1.1,
        details=str(altitude),
    )
    ctx.check(
        "azimuth joint is continuous",
        azimuth.articulation_type == ArticulationType.CONTINUOUS,
        details=str(azimuth.articulation_type),
    )

    def center_from_aabb(aabb):
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    front0 = center_from_aabb(ctx.part_element_world_aabb(tube, elem="front_rim"))
    column0 = center_from_aabb(ctx.part_element_world_aabb(arm, elem="arm_column"))
    with ctx.pose({altitude: 0.95}):
        front_up = center_from_aabb(ctx.part_element_world_aabb(tube, elem="front_rim"))
    with ctx.pose({azimuth: math.pi / 2.0}):
        column_rot = center_from_aabb(ctx.part_element_world_aabb(arm, elem="arm_column"))

    ctx.check(
        "positive altitude raises the front aperture",
        front_up[2] > front0[2] + 0.15,
        details=f"front0={front0}, front_up={front_up}",
    )
    ctx.check(
        "azimuth stage rotates the single arm about the vertical axis",
        column0[1] > 0.15 and column_rot[0] < -0.15,
        details=f"column0={column0}, column_rot={column_rot}",
    )

    return ctx.report()


object_model = build_object_model()
