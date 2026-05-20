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


PITCH_SET_ANGLE = 0.14
SLIDE_TRAVEL = 0.25


def _cylinder_x(radius: float, length: float) -> Cylinder:
    return Cylinder(radius=radius, length=length)


def _origin_x(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0))


def _origin_y(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(-math.pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_calibration_vacuum")

    graphite = model.material("graphite", rgba=(0.12, 0.13, 0.15, 1.0))
    satin_white = model.material("satin_white", rgba=(0.86, 0.88, 0.86, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.035, 0.036, 0.038, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.58, 0.61, 0.62, 1.0))
    anodized_blue = model.material("anodized_blue", rgba=(0.08, 0.26, 0.72, 1.0))
    amber = model.material("amber_index", rgba=(0.95, 0.62, 0.12, 1.0))
    black_mark = model.material("black_mark", rgba=(0.0, 0.0, 0.0, 1.0))
    pale_gap = model.material("pale_gap", rgba=(0.78, 0.92, 0.98, 1.0))

    body = model.part("vacuum_body")
    body.visual(
        Box((0.62, 0.30, 0.13)),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=graphite,
        name="lower_chassis",
    )
    body.visual(
        Box((0.50, 0.24, 0.095)),
        origin=Origin(xyz=(-0.045, 0.0, 0.185)),
        material=satin_white,
        name="motor_cover",
    )
    body.visual(
        Box((0.42, 0.028, 0.014)),
        origin=Origin(xyz=(-0.02, -0.116, 0.235)),
        material=anodized_blue,
        name="side_datum_0",
    )
    body.visual(
        Box((0.42, 0.028, 0.014)),
        origin=Origin(xyz=(-0.02, 0.116, 0.235)),
        material=anodized_blue,
        name="side_datum_1",
    )
    body.visual(
        Box((0.40, 0.028, 0.006)),
        origin=Origin(xyz=(-0.03, 0.0, 0.235)),
        material=brushed_steel,
        name="top_datum_bar",
    )
    for i, x in enumerate((-0.16, -0.08, 0.0, 0.08, 0.16)):
        body.visual(
            Box((0.008, 0.034, 0.008)),
            origin=Origin(xyz=(x, 0.0, 0.237)),
            material=black_mark,
            name=f"body_index_{i}",
        )
    body.visual(
        Cylinder(radius=0.070, length=0.060),
        origin=_origin_x(0.318, 0.0, 0.205),
        material=brushed_steel,
        name="front_socket",
    )
    body.visual(
        Cylinder(radius=0.050, length=0.055),
        origin=_origin_x(0.350, 0.0, 0.225),
        material=graphite,
        name="socket_lip",
    )
    body.visual(
        Box((0.055, 0.050, 0.105)),
        origin=Origin(xyz=(0.430, -0.075, 0.254)),
        material=brushed_steel,
        name="front_yoke_arm_0",
    )
    body.visual(
        Box((0.055, 0.050, 0.105)),
        origin=Origin(xyz=(0.430, 0.075, 0.254)),
        material=brushed_steel,
        name="front_yoke_arm_1",
    )
    body.visual(
        Box((0.110, 0.200, 0.040)),
        origin=Origin(xyz=(0.365, 0.0, 0.195)),
        material=brushed_steel,
        name="front_yoke_bridge",
    )
    body.visual(
        Cylinder(radius=0.070, length=0.044),
        origin=_origin_y(-0.225, -0.170, 0.072),
        material=dark_rubber,
        name="wheel_0",
    )
    body.visual(
        Cylinder(radius=0.070, length=0.044),
        origin=_origin_y(-0.225, 0.170, 0.072),
        material=dark_rubber,
        name="wheel_1",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.360),
        origin=_origin_y(-0.225, 0.0, 0.072),
        material=brushed_steel,
        name="wheel_axle",
    )
    body.visual(
        Box((0.36, 0.24, 0.010)),
        origin=Origin(xyz=(0.045, 0.0, 0.014)),
        material=dark_rubber,
        name="base_traction_pad",
    )

    upper_wand = model.part("upper_wand")
    upper_wand.visual(
        Cylinder(radius=0.034, length=0.100),
        origin=_origin_y(0.0, 0.0, 0.0),
        material=brushed_steel,
        name="hinge_barrel",
    )
    upper_wand.visual(
        Cylinder(radius=0.035, length=0.430),
        origin=_origin_x(0.215, 0.0, 0.0),
        material=graphite,
        name="outer_sleeve",
    )
    upper_wand.visual(
        Cylinder(radius=0.040, length=0.036),
        origin=_origin_x(0.425, 0.0, 0.0),
        material=anodized_blue,
        name="lock_collar",
    )
    upper_wand.visual(
        Box((0.060, 0.018, 0.020)),
        origin=Origin(xyz=(0.425, 0.0, 0.043)),
        material=amber,
        name="lock_tab",
    )
    for i, x in enumerate((0.235, 0.265, 0.295, 0.325, 0.355, 0.385)):
        upper_wand.visual(
            Box((0.006, 0.012, 0.006)),
            origin=Origin(xyz=(x, 0.0, 0.036)),
            material=black_mark,
            name=f"sleeve_index_{i}",
        )

    lower_wand = model.part("lower_wand")
    lower_wand.visual(
        Cylinder(radius=0.022, length=0.810),
        origin=_origin_x(0.045, 0.0, 0.0),
        material=brushed_steel,
        name="inner_tube",
    )
    lower_wand.visual(
        Cylinder(radius=0.033, length=0.030),
        origin=_origin_x(0.060, 0.0, 0.0),
        material=anodized_blue,
        name="slide_stop_collar",
    )
    lower_wand.visual(
        Cylinder(radius=0.029, length=0.070),
        origin=_origin_x(0.440, 0.0, 0.0),
        material=graphite,
        name="nozzle_socket",
    )
    lower_wand.visual(
        Box((0.066, 0.050, 0.082)),
        origin=Origin(xyz=(0.508, -0.054, 0.0)),
        material=brushed_steel,
        name="nozzle_yoke_arm_0",
    )
    lower_wand.visual(
        Box((0.066, 0.050, 0.082)),
        origin=Origin(xyz=(0.508, 0.054, 0.0)),
        material=brushed_steel,
        name="nozzle_yoke_arm_1",
    )
    for i, x in enumerate((0.100, 0.175, 0.250, 0.325, 0.400)):
        lower_wand.visual(
            Box((0.012, 0.010, 0.005)),
            origin=Origin(xyz=(x, 0.0, 0.024)),
            material=amber,
            name=f"extension_mark_{i}",
        )

    nozzle = model.part("floor_nozzle")
    nozzle.visual(
        Cylinder(radius=0.032, length=0.058),
        origin=_origin_y(0.0, 0.0, 0.0),
        material=brushed_steel,
        name="pivot_barrel",
    )
    nozzle.visual(
        Box((0.165, 0.050, 0.046)),
        origin=Origin(xyz=(0.065, 0.0, -0.038)),
        material=graphite,
        name="neck_bridge",
    )
    nozzle.visual(
        Box((0.405, 0.240, 0.060)),
        origin=Origin(xyz=(0.185, 0.0, -0.078)),
        material=satin_white,
        name="nozzle_head",
    )
    nozzle.visual(
        Box((0.405, 0.030, 0.020)),
        origin=Origin(xyz=(0.185, -0.105, -0.039)),
        material=anodized_blue,
        name="edge_datum_0",
    )
    nozzle.visual(
        Box((0.405, 0.030, 0.020)),
        origin=Origin(xyz=(0.185, 0.105, -0.039)),
        material=anodized_blue,
        name="edge_datum_1",
    )
    nozzle.visual(
        Box((0.132, 0.055, 0.010)),
        origin=Origin(xyz=(0.040, -0.070, -0.113)),
        material=dark_rubber,
        name="bottom_datum_pad_0",
    )
    nozzle.visual(
        Box((0.132, 0.055, 0.010)),
        origin=Origin(xyz=(0.040, 0.070, -0.113)),
        material=dark_rubber,
        name="bottom_datum_pad_1",
    )
    nozzle.visual(
        Box((0.250, 0.010, 0.006)),
        origin=Origin(xyz=(0.225, 0.0, -0.042)),
        material=pale_gap,
        name="gap_reference_line",
    )
    for i, x in enumerate((0.010, 0.055, 0.100, 0.145, 0.190, 0.235, 0.280, 0.325)):
        nozzle.visual(
            Box((0.006, 0.044, 0.006)),
            origin=Origin(xyz=(x, -0.124, -0.036)),
            material=black_mark,
            name=f"nozzle_index_{i}",
        )
    nozzle.visual(
        Box((0.035, 0.200, 0.020)),
        origin=Origin(xyz=(0.390, 0.0, -0.054)),
        material=dark_rubber,
        name="front_squeegee",
    )

    height_dial = model.part("height_dial")
    height_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.056,
                0.022,
                body_style="faceted",
                top_diameter=0.045,
                grip=KnobGrip(style="ribbed", count=18, depth=0.0014),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "height_dial_cap",
        ),
        material=graphite,
        name="dial_cap",
    )

    model.articulation(
        "body_to_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=upper_wand,
        origin=Origin(xyz=(0.430, 0.0, 0.254), rpy=(0.0, PITCH_SET_ANGLE, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=-0.34, upper=0.50),
    )
    model.articulation(
        "wand_extension",
        ArticulationType.PRISMATIC,
        parent=upper_wand,
        child=lower_wand,
        origin=Origin(xyz=(0.425, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.18, lower=0.0, upper=SLIDE_TRAVEL),
    )
    model.articulation(
        "wand_to_nozzle",
        ArticulationType.REVOLUTE,
        parent=lower_wand,
        child=nozzle,
        origin=Origin(xyz=(0.508, 0.0, 0.0), rpy=(0.0, -PITCH_SET_ANGLE, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=-0.72, upper=0.72),
    )
    model.articulation(
        "nozzle_height",
        ArticulationType.REVOLUTE,
        parent=nozzle,
        child=height_dial,
        origin=Origin(xyz=(0.105, 0.065, -0.048)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=0.0, upper=math.tau),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    upper = object_model.get_part("upper_wand")
    lower = object_model.get_part("lower_wand")
    body = object_model.get_part("vacuum_body")
    nozzle = object_model.get_part("floor_nozzle")
    dial = object_model.get_part("height_dial")
    extension = object_model.get_articulation("wand_extension")
    shoulder = object_model.get_articulation("body_to_wand")
    nozzle_pitch = object_model.get_articulation("wand_to_nozzle")
    dial_joint = object_model.get_articulation("nozzle_height")

    ctx.allow_overlap(
        upper,
        lower,
        elem_a="outer_sleeve",
        elem_b="inner_tube",
        reason=(
            "The calibrated telescoping wand is represented as a nested sliding tube "
            "inside the outer sleeve; the overlap is the retained insertion length."
        ),
    )
    ctx.allow_overlap(
        upper,
        lower,
        elem_a="lock_collar",
        elem_b="inner_tube",
        reason=(
            "The split lock collar surrounds the same sliding inner tube as a clamp; "
            "the solid proxy intentionally captures the tube locally."
        ),
    )

    ctx.expect_contact(
        upper,
        body,
        elem_a="hinge_barrel",
        elem_b="front_yoke_arm_0",
        contact_tol=0.002,
        name="upper wand barrel seats against first body yoke cheek",
    )
    ctx.expect_contact(
        upper,
        body,
        elem_a="hinge_barrel",
        elem_b="front_yoke_arm_1",
        contact_tol=0.002,
        name="upper wand barrel seats against second body yoke cheek",
    )
    ctx.expect_contact(
        nozzle,
        lower,
        elem_a="pivot_barrel",
        elem_b="nozzle_yoke_arm_0",
        contact_tol=0.002,
        name="floor nozzle pivot seats against first wand yoke cheek",
    )
    ctx.expect_contact(
        nozzle,
        lower,
        elem_a="pivot_barrel",
        elem_b="nozzle_yoke_arm_1",
        contact_tol=0.002,
        name="floor nozzle pivot seats against second wand yoke cheek",
    )

    ctx.expect_within(
        lower,
        upper,
        axes="y",
        inner_elem="inner_tube",
        outer_elem="outer_sleeve",
        margin=0.004,
        name="inner wand tube remains laterally centered in sleeve",
    )
    ctx.expect_overlap(
        lower,
        upper,
        axes="x",
        elem_a="inner_tube",
        elem_b="outer_sleeve",
        min_overlap=0.080,
        name="collapsed wand retains long sleeve insertion",
    )
    ctx.expect_overlap(
        lower,
        upper,
        axes="x",
        elem_a="inner_tube",
        elem_b="lock_collar",
        min_overlap=0.020,
        name="lock collar surrounds the sliding tube",
    )
    with ctx.pose({extension: SLIDE_TRAVEL}):
        ctx.expect_within(
            lower,
            upper,
            axes="y",
            inner_elem="inner_tube",
            outer_elem="outer_sleeve",
            margin=0.004,
            name="extended wand remains laterally centered in sleeve",
        )
        ctx.expect_overlap(
            lower,
            upper,
            axes="x",
            elem_a="inner_tube",
            elem_b="outer_sleeve",
            min_overlap=0.070,
            name="extended wand keeps retained insertion",
        )

    ctx.expect_contact(
        dial,
        nozzle,
        elem_a="dial_cap",
        elem_b="nozzle_head",
        contact_tol=0.0015,
        name="height dial sits on nozzle datum surface",
    )
    ctx.check(
        "explicit calibrated wand and nozzle motion limits",
        shoulder.motion_limits is not None
        and shoulder.motion_limits.lower <= -0.30
        and shoulder.motion_limits.upper >= 0.45
        and extension.motion_limits is not None
        and abs(extension.motion_limits.upper - SLIDE_TRAVEL) < 1e-6
        and nozzle_pitch.motion_limits is not None
        and nozzle_pitch.motion_limits.lower <= -0.70
        and nozzle_pitch.motion_limits.upper >= 0.70
        and dial_joint.motion_limits is not None
        and dial_joint.motion_limits.upper >= math.tau - 1e-6,
        details="wand pitch, telescoping slide, nozzle pitch, and height dial must all be explicitly limited",
    )

    return ctx.report()


object_model = build_object_model()
