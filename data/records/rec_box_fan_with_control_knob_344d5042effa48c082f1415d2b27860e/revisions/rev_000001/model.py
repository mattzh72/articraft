from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _annulus_x(
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    x0: float,
    center_y: float = 0.0,
    center_z: float = 0.0,
) -> cq.Workplane:
    """CadQuery annular tube/ring extruded along +X."""
    return (
        cq.Workplane("YZ")
        .center(center_y, center_z)
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((x0, 0.0, 0.0))
    )


def _annulus_y(
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    y0: float,
    center_x: float = 0.0,
    center_z: float = 0.0,
) -> cq.Workplane:
    """CadQuery annular hinge barrel extruded along +Y."""
    return (
        cq.Workplane("XZ")
        .center(center_x, center_z)
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((0.0, y0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="through_wall_louver_fan")

    warm_white = model.material("warm_white_plastic", rgba=(0.86, 0.84, 0.77, 1.0))
    dark_plastic = model.material("dark_drum_plastic", rgba=(0.05, 0.055, 0.055, 1.0))
    blade_black = model.material("matte_black_blades", rgba=(0.01, 0.012, 0.014, 1.0))
    hinge_metal = model.material("brushed_hinge_metal", rgba=(0.58, 0.58, 0.54, 1.0))
    knob_blue = model.material("blue_speed_knob", rgba=(0.05, 0.18, 0.42, 1.0))
    mark_gray = model.material("printed_speed_marks", rgba=(0.03, 0.03, 0.03, 1.0))

    frame = model.part("frame")

    # A real through-wall fan has a hollow cylindrical sleeve, not a solid puck.
    frame.visual(
        mesh_from_cadquery(
            _annulus_x(outer_radius=0.165, inner_radius=0.148, length=0.305, x0=-0.002),
            "round_drum_shell",
            tolerance=0.0008,
            angular_tolerance=0.05,
        ),
        material=dark_plastic,
        name="drum_shell",
    )
    frame.visual(
        mesh_from_cadquery(
            _annulus_x(outer_radius=0.205, inner_radius=0.145, length=0.020, x0=-0.020),
            "front_frame_ring",
            tolerance=0.0008,
            angular_tolerance=0.05,
        ),
        material=warm_white,
        name="front_flange",
    )
    frame.visual(
        mesh_from_cadquery(
            _annulus_x(outer_radius=0.182, inner_radius=0.150, length=0.032, x0=0.258),
            "rear_wall_collar",
            tolerance=0.0008,
            angular_tolerance=0.05,
        ),
        material=warm_white,
        name="rear_collar",
    )

    # Stationary spider frame and motor pod inside the drum.
    frame.visual(
        Cylinder(radius=0.044, length=0.060),
        origin=Origin(xyz=(0.110, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="motor_pod",
    )
    for i, angle in enumerate((math.radians(35), math.radians(145), math.radians(215), math.radians(325))):
        frame.visual(
            Box((0.018, 0.116, 0.013)),
            origin=Origin(
                xyz=(0.110, 0.097 * math.cos(angle), 0.097 * math.sin(angle)),
                rpy=(angle, 0.0, 0.0),
            ),
            material=dark_plastic,
            name=f"spider_strut_{i}",
        )
    frame.visual(
        Cylinder(radius=0.006, length=0.040),
        origin=Origin(xyz=(0.098, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="motor_shaft",
    )

    # Lower-right control boss molded into the surrounding inner frame.
    frame.visual(
        Box((0.020, 0.098, 0.078)),
        origin=Origin(xyz=(-0.014, 0.170, -0.132)),
        material=warm_white,
        name="control_boss",
    )
    for i, (dy, dz) in enumerate(((-0.024, 0.024), (0.0, 0.032), (0.024, 0.024))):
        frame.visual(
            Cylinder(radius=0.0037, length=0.006),
            origin=Origin(
                xyz=(-0.026, 0.170 + dy, -0.132 + dz),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=mark_gray,
            name=f"speed_mark_{i}",
        )

    # Exposed split hinge knuckles fixed to the stationary frame. The moving
    # center knuckle lives on the louver cover part.
    hinge_x = -0.046
    hinge_z = 0.178
    for i, y in enumerate((-0.125, 0.125)):
        frame.visual(
            Box((0.034, 0.064, 0.024)),
            origin=Origin(xyz=(-0.037, y, hinge_z - 0.006)),
            material=hinge_metal,
            name=f"hinge_leaf_{i}",
        )
        frame.visual(
            Cylinder(radius=0.008, length=0.060),
            origin=Origin(xyz=(hinge_x, y, hinge_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hinge_metal,
            name=f"hinge_knuckle_{i}",
        )
    frame.visual(
        Cylinder(radius=0.0048, length=0.300),
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_metal,
        name="hinge_pin",
    )

    fan_blade = model.part("fan_blade")
    fan_blade.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                0.128,
                0.034,
                5,
                thickness=0.026,
                blade_pitch_deg=33.0,
                blade_sweep_deg=25.0,
                blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=14.0, camber=0.12, tip_clearance=0.002),
                hub=FanRotorHub(style="spinner", bore_diameter=0.014),
            ),
            "five_blade_rotor",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blade_black,
        name="rotor",
    )

    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.044,
                0.028,
                body_style="skirted",
                top_diameter=0.034,
                grip=KnobGrip(style="fluted", count=18, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                bore=KnobBore(style="round", diameter=0.007),
                center=False,
            ),
            "rotary_speed_knob",
        ),
        origin=Origin(rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=knob_blue,
        name="knob_body",
    )
    speed_knob.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="knob_shaft",
    )

    louver_cover = model.part("louver_cover")
    cover_outer = 0.178
    cover_inner = 0.142
    cover_thickness = 0.012
    louver_cover.visual(
        mesh_from_cadquery(
            _annulus_x(
                outer_radius=cover_outer,
                inner_radius=cover_inner,
                length=cover_thickness,
                x0=-cover_thickness / 2.0,
                center_z=-cover_outer,
            ),
            "round_louver_cover_ring",
            tolerance=0.0008,
            angular_tolerance=0.05,
        ),
        material=warm_white,
        name="cover_ring",
    )
    # Five rain-shedding horizontal louvers span into the circular frame.
    for i, z in enumerate((-0.067, -0.095, -0.123, -0.151, -0.179)):
        dz_from_center = z + cover_outer
        span = 2.0 * math.sqrt(max(0.0, cover_inner**2 - dz_from_center**2)) + 0.050
        louver_cover.visual(
            Box((0.026, span, 0.010)),
            origin=Origin(xyz=(-0.001, 0.0, z), rpy=(0.0, -0.42, 0.0)),
            material=warm_white,
            name=f"louver_{i}",
        )
    louver_cover.visual(
        Box((0.018, 0.020, 0.250)),
        origin=Origin(xyz=(-0.002, 0.0, -0.178)),
        material=warm_white,
        name="center_divider",
    )
    louver_cover.visual(
        Box((0.014, 0.094, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=hinge_metal,
        name="cover_hinge_leaf",
    )
    louver_cover.visual(
        mesh_from_cadquery(
            _annulus_y(outer_radius=0.008, inner_radius=0.0045, length=0.094, y0=0.047),
            "moving_hinge_barrel",
            tolerance=0.0005,
            angular_tolerance=0.05,
        ),
        material=hinge_metal,
        name="cover_knuckle",
    )
    louver_cover.visual(
        Box((0.016, 0.060, 0.018)),
        origin=Origin(xyz=(-0.010, 0.0, -2.0 * cover_outer + 0.005)),
        material=warm_white,
        name="pull_lip",
    )

    model.articulation(
        "frame_to_fan_blade",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=fan_blade,
        origin=Origin(xyz=(0.065, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=80.0),
        motion_properties=MotionProperties(damping=0.01, friction=0.0),
    )
    model.articulation(
        "frame_to_speed_knob",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=speed_knob,
        origin=Origin(xyz=(-0.025, 0.170, -0.132)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=10.0),
        motion_properties=MotionProperties(damping=0.02, friction=0.01),
    )
    model.articulation(
        "frame_to_louver_cover",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=louver_cover,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.25),
        motion_properties=MotionProperties(damping=0.04, friction=0.02),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    fan_blade = object_model.get_part("fan_blade")
    speed_knob = object_model.get_part("speed_knob")
    louver_cover = object_model.get_part("louver_cover")
    fan_joint = object_model.get_articulation("frame_to_fan_blade")
    knob_joint = object_model.get_articulation("frame_to_speed_knob")
    cover_joint = object_model.get_articulation("frame_to_louver_cover")

    ctx.allow_overlap(
        frame,
        speed_knob,
        elem_a="control_boss",
        elem_b="knob_shaft",
        reason="The knob shaft is intentionally captured slightly inside the fixed speed-control boss.",
    )
    ctx.allow_overlap(
        frame,
        louver_cover,
        elem_a="hinge_pin",
        elem_b="cover_knuckle",
        reason="The fixed hinge pin intentionally passes through the moving cover barrel with a slight captured fit.",
    )

    ctx.check(
        "fan blade is continuous",
        fan_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={fan_joint.articulation_type}",
    )
    ctx.check(
        "speed knob is continuous",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_joint.articulation_type}",
    )
    ctx.check(
        "cover hinge has top-edge drop range",
        cover_joint.motion_limits is not None
        and cover_joint.motion_limits.lower == 0.0
        and cover_joint.motion_limits.upper is not None
        and cover_joint.motion_limits.upper > 1.0,
        details=f"limits={cover_joint.motion_limits}",
    )

    with ctx.pose({cover_joint: 0.0}):
        ctx.expect_gap(
            frame,
            louver_cover,
            axis="x",
            positive_elem="front_flange",
            negative_elem="cover_ring",
            min_gap=0.010,
            max_gap=0.030,
            name="closed cover sits just in front of frame",
        )
        ctx.expect_overlap(
            louver_cover,
            frame,
            axes="yz",
            elem_a="cover_ring",
            elem_b="front_flange",
            min_overlap=0.32,
            name="closed circular cover spans the fan opening",
        )
        closed_aabb = ctx.part_world_aabb(louver_cover)

    with ctx.pose({cover_joint: 1.15}):
        open_aabb = ctx.part_world_aabb(louver_cover)

    ctx.check(
        "cover drops outward from top hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][0] < closed_aabb[0][0] - 0.11,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )
    ctx.expect_within(
        fan_blade,
        frame,
        axes="yz",
        inner_elem="rotor",
        outer_elem="drum_shell",
        margin=0.025,
        name="rotor has radial clearance inside drum",
    )
    ctx.expect_gap(
        frame,
        speed_knob,
        axis="x",
        positive_elem="control_boss",
        negative_elem="knob_body",
        min_gap=0.0,
        max_gap=0.004,
        name="speed knob is seated on the inner frame boss",
    )
    ctx.expect_within(
        speed_knob,
        frame,
        axes="yz",
        inner_elem="knob_shaft",
        outer_elem="control_boss",
        margin=0.001,
        name="speed knob shaft is centered in boss",
    )
    ctx.expect_overlap(
        speed_knob,
        frame,
        axes="x",
        elem_a="knob_shaft",
        elem_b="control_boss",
        min_overlap=0.006,
        name="speed knob shaft remains inserted",
    )
    ctx.expect_within(
        frame,
        louver_cover,
        axes="xz",
        inner_elem="hinge_pin",
        outer_elem="cover_knuckle",
        margin=0.001,
        name="hinge pin runs through moving barrel",
    )
    ctx.expect_overlap(
        frame,
        louver_cover,
        axes="y",
        elem_a="hinge_pin",
        elem_b="cover_knuckle",
        min_overlap=0.090,
        name="hinge barrel has retained pin length",
    )

    return ctx.report()


object_model = build_object_model()
