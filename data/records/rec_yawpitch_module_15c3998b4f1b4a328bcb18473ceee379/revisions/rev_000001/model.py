from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_FLANGE_RADIUS = 0.17
BASE_FLANGE_THICKNESS = 0.018
BASE_HOUSING_RADIUS = 0.145
BASE_HOUSING_HEIGHT = 0.055
BASE_COLLAR_RADIUS = 0.128
BASE_COLLAR_HEIGHT = 0.012
BASE_BOLT_CIRCLE_RADIUS = 0.132
BASE_BOLT_DIAMETER = 0.012
BASE_HEIGHT = BASE_FLANGE_THICKNESS + BASE_HOUSING_HEIGHT + BASE_COLLAR_HEIGHT

TURNTABLE_RADIUS = 0.155
TURNTABLE_THICKNESS = 0.03
SERVICE_CAP_RADIUS = 0.06
SERVICE_CAP_HEIGHT = 0.01

ARM_DEPTH = 0.09
ARM_THICKNESS = 0.03
ARM_HEIGHT = 0.35
ARM_CENTER_Y = 0.16
PITCH_AXIS_Z = 0.27

FRAME_DEPTH = 0.05
FRAME_WIDTH = 0.22
FRAME_HEIGHT = 0.225
FRAME_APERTURE_WIDTH = 0.14
FRAME_APERTURE_HEIGHT = 0.145
FRAME_CENTER_X = 0.105
TRUNNION_RADIUS = 0.012
HUB_CENTER_Y = 0.09
HUB_WIDTH_Y = 0.04
TRUNNION_STUB_LENGTH = ARM_CENTER_Y - ARM_THICKNESS / 2.0 - (HUB_CENTER_Y + HUB_WIDTH_Y / 2.0)
CORE_DEPTH = 0.078
CORE_WIDTH = 0.145
CORE_HEIGHT = 0.115


def make_base_shape() -> cq.Workplane:
    flange = cq.Workplane("XY").circle(BASE_FLANGE_RADIUS).extrude(BASE_FLANGE_THICKNESS)
    housing = (
        cq.Workplane("XY")
        .circle(BASE_HOUSING_RADIUS)
        .extrude(BASE_HOUSING_HEIGHT)
        .translate((0.0, 0.0, BASE_FLANGE_THICKNESS))
    )
    collar = (
        cq.Workplane("XY")
        .circle(BASE_COLLAR_RADIUS)
        .extrude(BASE_COLLAR_HEIGHT)
        .translate((0.0, 0.0, BASE_FLANGE_THICKNESS + BASE_HOUSING_HEIGHT))
    )
    bolt_holes = (
        cq.Workplane("XY")
        .polarArray(BASE_BOLT_CIRCLE_RADIUS, 0.0, 360.0, 8)
        .circle(BASE_BOLT_DIAMETER / 2.0)
        .extrude(BASE_FLANGE_THICKNESS + 0.004)
        .translate((0.0, 0.0, -0.002))
    )
    return flange.union(housing).union(collar).cut(bolt_holes)


def make_turntable_shape() -> cq.Workplane:
    disc = cq.Workplane("XY").circle(TURNTABLE_RADIUS).extrude(TURNTABLE_THICKNESS)
    service_cap = (
        cq.Workplane("XY")
        .circle(SERVICE_CAP_RADIUS)
        .extrude(SERVICE_CAP_HEIGHT)
        .translate((0.0, 0.0, TURNTABLE_THICKNESS))
    )
    return disc.union(service_cap)


def make_fork_side_shape(side_sign: float) -> cq.Workplane:
    arm_plate = (
        cq.Workplane("XY")
        .box(0.04, ARM_THICKNESS, ARM_HEIGHT)
        .translate((-0.015, side_sign * ARM_CENTER_Y, TURNTABLE_THICKNESS + ARM_HEIGHT / 2.0))
    )
    window = (
        cq.Workplane("XY")
        .box(0.02, ARM_THICKNESS + 0.004, 0.17)
        .translate((-0.015, side_sign * ARM_CENTER_Y, TURNTABLE_THICKNESS + 0.17))
    )
    neck = (
        cq.Workplane("XY")
        .box(0.045, ARM_THICKNESS, 0.045)
        .translate((0.0175, side_sign * ARM_CENTER_Y, PITCH_AXIS_Z))
    )
    trunnion_pad = (
        cq.Workplane("XY")
        .box(0.032, ARM_THICKNESS, 0.08)
        .translate((0.0, side_sign * ARM_CENTER_Y, PITCH_AXIS_Z))
    )
    top_cap = (
        cq.Workplane("XY")
        .box(0.05, ARM_THICKNESS, 0.04)
        .translate((-0.012, side_sign * ARM_CENTER_Y, TURNTABLE_THICKNESS + ARM_HEIGHT - 0.02))
    )
    return arm_plate.cut(window).union(neck).union(trunnion_pad).union(top_cap)


def make_yoke_bridge_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.06, 2.0 * ARM_CENTER_Y - ARM_THICKNESS, 0.055)
        .translate((-0.028, 0.0, TURNTABLE_THICKNESS + 0.0275))
    )


def make_fork_assembly_shape() -> cq.Workplane:
    return (
        make_fork_side_shape(1.0)
        .union(make_fork_side_shape(-1.0))
        .union(make_yoke_bridge_shape())
    )


def make_sensor_head_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").box(FRAME_DEPTH, FRAME_WIDTH, FRAME_HEIGHT).translate(
        (FRAME_CENTER_X, 0.0, 0.0)
    )
    inner = cq.Workplane("XY").box(
        FRAME_DEPTH + 0.01,
        FRAME_APERTURE_WIDTH,
        FRAME_APERTURE_HEIGHT,
    ).translate((FRAME_CENTER_X, 0.0, 0.0))
    ring = outer.cut(inner)
    housing_shell = (
        cq.Workplane("XY")
        .box(CORE_DEPTH, CORE_WIDTH, CORE_HEIGHT)
        .translate((FRAME_CENTER_X + 0.01, 0.0, 0.0))
        .faces(">X")
        .workplane(centerOption="CenterOfBoundBox")
        .rect(CORE_WIDTH * 0.72, CORE_HEIGHT * 0.54)
        .cutBlind(-CORE_DEPTH * 0.42)
    )
    hub_carrier = (
        cq.Workplane("XY")
        .box(0.028, 2.0 * HUB_CENTER_Y, 0.065)
        .translate((0.014, 0.0, 0.0))
    )
    top_rail = (
        cq.Workplane("XY")
        .box(FRAME_CENTER_X + FRAME_DEPTH / 2.0, FRAME_WIDTH - 0.03, 0.018)
        .translate((FRAME_CENTER_X / 2.0 + FRAME_DEPTH / 4.0, 0.0, FRAME_APERTURE_HEIGHT / 2.0 + 0.018))
    )
    bottom_rail = top_rail.mirror("XY")
    rear_spine = (
        cq.Workplane("XY")
        .box(FRAME_CENTER_X + 0.01, 0.04, 0.055)
        .translate((FRAME_CENTER_X / 2.0 + 0.005, 0.0, 0.0))
    )
    hub_right = (
        cq.Workplane("XY")
        .box(0.024, HUB_WIDTH_Y, 0.085)
        .translate((0.0, HUB_CENTER_Y, 0.0))
    )
    hub_left = hub_right.mirror("XZ")
    right_trunnion = cq.Workplane(
        obj=cq.Solid.makeCylinder(
            TRUNNION_RADIUS,
            TRUNNION_STUB_LENGTH,
            cq.Vector(0.0, HUB_CENTER_Y + HUB_WIDTH_Y / 2.0, 0.0),
            cq.Vector(0.0, 1.0, 0.0),
        )
    )
    left_trunnion = cq.Workplane(
        obj=cq.Solid.makeCylinder(
            TRUNNION_RADIUS,
            TRUNNION_STUB_LENGTH,
            cq.Vector(0.0, -(HUB_CENTER_Y + HUB_WIDTH_Y / 2.0), 0.0),
            cq.Vector(0.0, -1.0, 0.0),
        )
    )
    return (
        ring.union(housing_shell)
        .union(hub_carrier)
        .union(top_rail)
        .union(bottom_rail)
        .union(rear_spine)
        .union(hub_right)
        .union(hub_left)
        .union(right_trunnion)
        .union(left_trunnion)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="marine_sensor_pan_tilt_head")

    marine_white = model.material("marine_white", color=(0.93, 0.94, 0.95))
    dark_paint = model.material("dark_paint", color=(0.18, 0.19, 0.22))
    sensor_black = model.material("sensor_black", color=(0.08, 0.09, 0.10))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(make_base_shape(), "base_shell"),
        material=marine_white,
        name="base_shell",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_FLANGE_RADIUS, length=BASE_HEIGHT),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
    )

    yoke = model.part("yoke")
    yoke.visual(
        mesh_from_cadquery(make_turntable_shape(), "turntable"),
        material=marine_white,
        name="turntable",
    )
    yoke.visual(
        mesh_from_cadquery(make_fork_assembly_shape(), "fork_assembly"),
        material=marine_white,
        name="fork_assembly",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.12, 0.38, 0.40)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
    )

    sensor_frame = model.part("sensor_frame")
    sensor_frame.visual(
        mesh_from_cadquery(make_sensor_head_shape(), "sensor_head"),
        material=marine_white,
        name="sensor_head",
    )
    sensor_frame.inertial = Inertial.from_geometry(
        Box((0.14, FRAME_WIDTH + 2.0 * TRUNNION_STUB_LENGTH, FRAME_HEIGHT)),
        mass=11.0,
        origin=Origin(),
    )

    model.articulation(
        "base_to_yoke",
        ArticulationType.REVOLUTE,
        parent=base,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.5,
            lower=-3.0,
            upper=3.0,
        ),
    )
    model.articulation(
        "yoke_to_sensor",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=sensor_frame,
        origin=Origin(xyz=(0.0, 0.0, PITCH_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.2,
            lower=-0.75,
            upper=1.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    yoke = object_model.get_part("yoke")
    sensor_frame = object_model.get_part("sensor_frame")
    yaw = object_model.get_articulation("base_to_yoke")
    pitch = object_model.get_articulation("yoke_to_sensor")
    turntable = yoke.get_visual("turntable")
    sensor_head = sensor_frame.get_visual("sensor_head")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.allow_overlap(
        sensor_frame,
        yoke,
        elem_a=sensor_head,
        elem_b="fork_assembly",
        reason="Pitch trunnion bearing engagement is represented as a captured nested interface inside the fork bosses.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "yaw joint uses vertical axis",
        tuple(yaw.axis) == (0.0, 0.0, 1.0),
        details=f"expected (0, 0, 1), got {yaw.axis}",
    )
    ctx.check(
        "pitch joint uses lateral axis",
        tuple(pitch.axis) == (0.0, 1.0, 0.0),
        details=f"expected (0, 1, 0), got {pitch.axis}",
    )
    ctx.check(
        "yaw limits cover broad deck sweep",
        yaw.motion_limits is not None
        and yaw.motion_limits.lower is not None
        and yaw.motion_limits.upper is not None
        and yaw.motion_limits.lower <= -2.5
        and yaw.motion_limits.upper >= 2.5,
        details="yaw range should read as a wide turntable sweep",
    )
    ctx.check(
        "pitch limits allow down and up look",
        pitch.motion_limits is not None
        and pitch.motion_limits.lower is not None
        and pitch.motion_limits.upper is not None
        and pitch.motion_limits.lower < 0.0
        and pitch.motion_limits.upper > 0.7,
        details="pitch range should include negative and positive elevation",
    )
    ctx.expect_contact(
        base,
        yoke,
        contact_tol=0.001,
        name="turntable sits on base housing",
    )
    ctx.expect_contact(
        yoke,
        sensor_frame,
        contact_tol=0.001,
        name="sensor frame is supported by fork trunnions",
    )
    ctx.expect_gap(
        sensor_frame,
        yoke,
        axis="z",
        positive_elem=sensor_head,
        negative_elem=turntable,
        min_gap=0.08,
        name="sensor frame reads above turntable",
    )
    ctx.expect_overlap(
        sensor_frame,
        yoke,
        axes="xy",
        elem_a=sensor_head,
        elem_b=turntable,
        min_overlap=0.05,
        name="sensor frame footprint nests over the rotating base",
    )

    lower_pitch = pitch.motion_limits.lower if pitch.motion_limits is not None else None
    if lower_pitch is not None:
        with ctx.pose({pitch: lower_pitch}):
            ctx.expect_contact(
                yoke,
                sensor_frame,
                contact_tol=0.001,
                name="trunnion contact holds at lower pitch stop",
            )
            ctx.expect_gap(
                sensor_frame,
                yoke,
                axis="z",
                positive_elem=sensor_head,
                negative_elem=turntable,
                min_gap=0.04,
                name="lower pitch pose still clears the turntable",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
