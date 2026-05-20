from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
)


def _annular_cylinder_mesh(outer_radius: float, inner_radius: float, length: float, name: str):
    """A hollow cylinder centered on local Z, exported as a managed mesh."""

    body = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
    )
    return mesh_from_cadquery(body, name, tolerance=0.001, angular_tolerance=0.08)


def _leg_origin(angle: float, top_z: float, foot_radius: float, foot_z: float) -> tuple[Origin, float]:
    """Return a box origin whose local Z runs from the tripod hub to a foot."""

    x = foot_radius * math.cos(angle)
    y = foot_radius * math.sin(angle)
    dz = foot_z - top_z
    length = math.sqrt(x * x + y * y + dz * dz)
    center = (0.5 * x, 0.5 * y, 0.5 * (top_z + foot_z))
    pitch = math.acos(dz / length)
    return Origin(xyz=center, rpy=(0.0, pitch, angle)), length


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_sct_single_arm_equatorial")

    orange = model.material("powder_orange", rgba=(0.88, 0.33, 0.08, 1.0))
    black = model.material("satin_black", rgba=(0.015, 0.016, 0.018, 1.0))
    dark = model.material("matte_dark_gray", rgba=(0.07, 0.075, 0.08, 1.0))
    metal = model.material("brushed_metal", rgba=(0.56, 0.58, 0.60, 1.0))
    rubber = model.material("black_rubber", rgba=(0.0, 0.0, 0.0, 1.0))
    glass = model.material("faint_blue_glass", rgba=(0.55, 0.78, 0.95, 0.34))

    tilt = math.radians(35.0)

    tripod = model.part("tripod")
    tripod.visual(
        Cylinder(radius=0.065, length=0.50),
        origin=Origin(xyz=(0.0, 0.0, 0.285)),
        material=metal,
        name="center_column",
    )
    tripod.visual(
        Cylinder(radius=0.083, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.535)),
        material=metal,
        name="top_hub",
    )
    tripod.visual(
        Cylinder(radius=0.22, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.325)),
        material=metal,
        name="spreader_tray",
    )
    tripod.visual(
        Cylinder(radius=0.090, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.605)),
        material=metal,
        name="wedge_pier",
    )
    tripod.visual(
        Box((0.42, 0.31, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, 0.650)),
        material=dark,
        name="wedge_lower_plate",
    )

    top_z = 0.535
    foot_z = 0.030
    foot_radius = 0.50
    for idx, angle in enumerate((math.radians(90.0), math.radians(210.0), math.radians(330.0))):
        leg_origin, leg_length = _leg_origin(angle, top_z, foot_radius, foot_z)
        tripod.visual(
            Box((0.042, 0.034, leg_length)),
            origin=leg_origin,
            material=metal,
            name=f"tripod_leg_{idx}",
        )
        tripod.visual(
            Box((0.17, 0.085, 0.030)),
            origin=Origin(
                xyz=(foot_radius * math.cos(angle), foot_radius * math.sin(angle), 0.015),
                rpy=(0.0, 0.0, angle),
            ),
            material=rubber,
            name=f"foot_pad_{idx}",
        )

    wedge_top = (0.0, 0.0, 0.765)
    wedge_normal = (-math.sin(tilt), 0.0, math.cos(tilt))
    wedge_center = tuple(wedge_top[i] - 0.5 * 0.036 * wedge_normal[i] for i in range(3))
    tripod.visual(
        Box((0.38, 0.29, 0.036)),
        origin=Origin(xyz=wedge_center, rpy=(0.0, -tilt, 0.0)),
        material=orange,
        name="wedge_plate",
    )
    tripod.visual(
        Box((0.24, 0.035, 0.15)),
        origin=Origin(xyz=(-0.015, 0.145, 0.700), rpy=(0.0, -0.5 * tilt, 0.0)),
        material=orange,
        name="wedge_side_cheek_0",
    )
    tripod.visual(
        Box((0.24, 0.035, 0.15)),
        origin=Origin(xyz=(-0.015, -0.145, 0.700), rpy=(0.0, -0.5 * tilt, 0.0)),
        material=orange,
        name="wedge_side_cheek_1",
    )

    polar_head = model.part("polar_head")
    polar_head.visual(
        Cylinder(radius=0.095, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=dark,
        name="ra_bearing",
    )
    polar_head.visual(
        Cylinder(radius=0.025, length=0.240),
        origin=Origin(xyz=(-0.055, 0.0, 0.055), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="latitude_hinge_bar",
    )
    polar_head.visual(
        Box((0.18, 0.20, 0.075)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=orange,
        name="tilt_base_block",
    )
    polar_head.visual(
        Box((0.012, 0.10, 0.070)),
        origin=Origin(xyz=(0.094, 0.0, 0.070)),
        material=metal,
        name="polar_scale_plate",
    )

    fork_arm = model.part("fork_arm")
    fork_arm.visual(
        Cylinder(radius=0.105, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 0.0375)),
        material=orange,
        name="ra_collar",
    )
    fork_arm.visual(
        Box((0.090, 0.230, 0.090)),
        origin=Origin(xyz=(0.0, -0.095, 0.075)),
        material=orange,
        name="arm_root_web",
    )
    fork_arm.visual(
        Box((0.078, 0.050, 0.420)),
        origin=Origin(xyz=(0.0, -0.215, 0.260)),
        material=orange,
        name="single_fork_arm",
    )
    fork_arm.visual(
        Box((0.095, 0.070, 0.100)),
        origin=Origin(xyz=(0.0, -0.215, 0.470)),
        material=orange,
        name="declination_cap",
    )
    fork_arm.visual(
        Cylinder(radius=0.075, length=0.050),
        origin=Origin(xyz=(0.0, -0.205, 0.470), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="declination_boss",
    )

    optical_tube = model.part("optical_tube")
    tube_shell = _annular_cylinder_mesh(0.120, 0.105, 0.420, "stubby_tube_shell")
    front_ring = _annular_cylinder_mesh(0.128, 0.103, 0.025, "front_retain_ring")
    optical_tube.visual(
        tube_shell,
        origin=Origin(xyz=(0.0, 0.170, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=orange,
        name="tube_shell",
    )
    optical_tube.visual(
        front_ring,
        origin=Origin(xyz=(0.218, 0.170, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="front_retain_ring",
    )
    optical_tube.visual(
        Cylinder(radius=0.103, length=0.008),
        origin=Origin(xyz=(0.214, 0.170, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="front_corrector",
    )
    optical_tube.visual(
        Cylinder(radius=0.035, length=0.012),
        origin=Origin(xyz=(0.221, 0.170, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="secondary_mirror",
    )
    optical_tube.visual(
        Cylinder(radius=0.130, length=0.055),
        origin=Origin(xyz=(-0.2275, 0.170, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="rear_cell",
    )
    optical_tube.visual(
        Cylinder(radius=0.034, length=0.085),
        origin=Origin(xyz=(-0.2975, 0.170, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="rear_visual_back",
    )
    optical_tube.visual(
        Cylinder(radius=0.013, length=0.030),
        origin=Origin(xyz=(-0.270, 0.255, 0.035), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="focus_socket",
    )
    optical_tube.visual(
        Cylinder(radius=0.050, length=0.110),
        origin=Origin(xyz=(0.0, 0.055, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="declination_shaft",
    )
    optical_tube.visual(
        Cylinder(radius=0.066, length=0.030),
        origin=Origin(xyz=(0.0, 0.015, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=orange,
        name="declination_side_cap",
    )
    optical_tube.visual(
        Cylinder(radius=0.023, length=0.230),
        origin=Origin(xyz=(-0.015, 0.170, 0.190), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="finder_scope",
    )
    for idx, x in enumerate((-0.080, 0.060)):
        optical_tube.visual(
            Box((0.020, 0.030, 0.082)),
            origin=Origin(xyz=(x, 0.170, 0.153)),
            material=metal,
            name=f"finder_bracket_{idx}",
        )

    focus_knob = model.part("focus_knob")
    focus_knob.visual(
        Cylinder(radius=0.018, length=0.035),
        origin=Origin(xyz=(-0.0175, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="knob_cap",
    )

    model.articulation(
        "polar_angle",
        ArticulationType.REVOLUTE,
        parent=tripod,
        child=polar_head,
        origin=Origin(xyz=wedge_top, rpy=(0.0, -tilt, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.25, lower=-0.22, upper=0.30),
    )
    model.articulation(
        "right_ascension",
        ArticulationType.CONTINUOUS,
        parent=polar_head,
        child=fork_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.75),
    )
    model.articulation(
        "declination",
        ArticulationType.REVOLUTE,
        parent=fork_arm,
        child=optical_tube,
        origin=Origin(xyz=(0.0, -0.180, 0.470)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=0.65, lower=-0.75, upper=1.15),
    )
    model.articulation(
        "focus",
        ArticulationType.CONTINUOUS,
        parent=optical_tube,
        child=focus_knob,
        origin=Origin(xyz=(-0.285, 0.255, 0.035)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tripod = object_model.get_part("tripod")
    polar_head = object_model.get_part("polar_head")
    fork_arm = object_model.get_part("fork_arm")
    optical_tube = object_model.get_part("optical_tube")
    focus_knob = object_model.get_part("focus_knob")

    polar = object_model.get_articulation("polar_angle")
    ra = object_model.get_articulation("right_ascension")
    dec = object_model.get_articulation("declination")
    focus = object_model.get_articulation("focus")

    ctx.check(
        "equatorial axes use requested joint types",
        polar.articulation_type == ArticulationType.REVOLUTE
        and ra.articulation_type == ArticulationType.CONTINUOUS
        and dec.articulation_type == ArticulationType.REVOLUTE
        and focus.articulation_type == ArticulationType.CONTINUOUS,
        details=(
            f"polar={polar.articulation_type}, ra={ra.articulation_type}, "
            f"dec={dec.articulation_type}, focus={focus.articulation_type}"
        ),
    )

    ctx.expect_contact(
        polar_head,
        tripod,
        elem_a="ra_bearing",
        elem_b="wedge_plate",
        contact_tol=0.003,
        name="polar head sits on tilted wedge plate",
    )
    ctx.expect_contact(
        fork_arm,
        polar_head,
        elem_a="ra_collar",
        elem_b="ra_bearing",
        contact_tol=0.003,
        name="RA collar seats on polar bearing",
    )
    ctx.expect_contact(
        optical_tube,
        fork_arm,
        elem_a="declination_shaft",
        elem_b="declination_boss",
        contact_tol=0.003,
        name="tube declination shaft meets single fork tip",
    )
    ctx.expect_contact(
        focus_knob,
        optical_tube,
        elem_a="knob_cap",
        elem_b="focus_socket",
        contact_tol=0.003,
        name="focus knob is seated on rear cell socket",
    )

    def element_center_z(part, elem: str) -> float | None:
        bounds = ctx.part_element_world_aabb(part, elem=elem)
        if bounds is None:
            return None
        return 0.5 * (bounds[0][2] + bounds[1][2])

    def element_center_xy(part, elem: str) -> tuple[float, float] | None:
        bounds = ctx.part_element_world_aabb(part, elem=elem)
        if bounds is None:
            return None
        return (0.5 * (bounds[0][0] + bounds[1][0]), 0.5 * (bounds[0][1] + bounds[1][1]))

    front_rest_z = element_center_z(optical_tube, "front_corrector")
    with ctx.pose({dec: 0.60}):
        front_raised_z = element_center_z(optical_tube, "front_corrector")
    ctx.check(
        "positive declination raises the tube nose",
        front_rest_z is not None and front_raised_z is not None and front_raised_z > front_rest_z + 0.05,
        details=f"rest_z={front_rest_z}, raised_z={front_raised_z}",
    )

    arm_rest_xy = element_center_xy(fork_arm, "declination_cap")
    with ctx.pose({ra: 0.80}):
        arm_ra_xy = element_center_xy(fork_arm, "declination_cap")
    ctx.check(
        "RA axis swings the single fork arm around the polar bearing",
        arm_rest_xy is not None
        and arm_ra_xy is not None
        and math.hypot(arm_ra_xy[0] - arm_rest_xy[0], arm_ra_xy[1] - arm_rest_xy[1]) > 0.10,
        details=f"rest_xy={arm_rest_xy}, ra_xy={arm_ra_xy}",
    )

    bearing_rest_z = element_center_z(polar_head, "ra_bearing")
    with ctx.pose({polar: 0.25}):
        bearing_tilted_z = element_center_z(polar_head, "ra_bearing")
    ctx.check(
        "polar-angle joint changes the mount-base tilt",
        bearing_rest_z is not None
        and bearing_tilted_z is not None
        and abs(bearing_tilted_z - bearing_rest_z) > 0.005,
        details=f"rest_z={bearing_rest_z}, tilted_z={bearing_tilted_z}",
    )

    return ctx.report()


object_model = build_object_model()
