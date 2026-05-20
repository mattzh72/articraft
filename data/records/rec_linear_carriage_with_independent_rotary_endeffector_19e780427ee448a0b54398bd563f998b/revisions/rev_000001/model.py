from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


BEAM_LENGTH = 0.72
BEAM_WIDTH = 0.072
BEAM_HEIGHT = 0.048
WEAR_STRIP_THICKNESS = 0.006
WEAR_STRIP_HEIGHT = 0.022
WEAR_STRIP_LENGTH = 0.62
WEAR_STRIP_Z = -0.004
END_CAP_THICKNESS = 0.014
END_CAP_WIDTH = 0.088
END_CAP_HEIGHT = 0.060
SUSPENSION_LUG_SPACING = 0.46

CARRIAGE_LENGTH = 0.170
CARRIAGE_WIDTH = 0.118
CARRIAGE_TOP_Z = 0.050
CARRIAGE_BOTTOM_Z = -0.110
CARRIAGE_UPPER_HEIGHT = 0.085
CARRIAGE_UPPER_Z = 0.0075
CARRIAGE_LOWER_WIDTH = 0.074
CARRIAGE_LOWER_HEIGHT = 0.075
CARRIAGE_LOWER_Z = -0.0725
CARRIAGE_BRIDGE_WIDTH = 0.140
CARRIAGE_BRIDGE_THICKNESS = 0.014
CARRIAGE_BRIDGE_Z = 0.040
CARRIAGE_SIDE_THICKNESS = 0.012
CARRIAGE_SIDE_CENTER_Y = 0.061
CARRIAGE_SIDE_HEIGHT = 0.074
CARRIAGE_SIDE_CENTER_Z = 0.012
NOSE_BLOCK_LENGTH = 0.112
NOSE_BLOCK_DEPTH = 0.030
NOSE_BLOCK_HEIGHT = 0.088
NOSE_BLOCK_CENTER_Y = 0.060
NOSE_BLOCK_CENTER_Z = -0.011
TOP_PAD_LENGTH = 0.028
TOP_PAD_WIDTH = 0.016
TOP_PAD_THICKNESS = 0.004
TOP_PAD_X_OFFSET = 0.045
TOP_PAD_Y_OFFSET = 0.022
BEAM_SLOT_WIDTH = BEAM_WIDTH + (2.0 * WEAR_STRIP_THICKNESS) + 0.004
BEAM_SLOT_HEIGHT = 0.066
BEAM_SLOT_Z = -0.005
WIPER_THICKNESS = 0.004
WIPER_HEIGHT = 0.042
WIPER_SLOT_WIDTH = BEAM_WIDTH + (2.0 * WEAR_STRIP_THICKNESS)

GUIDE_WHEEL_RADIUS = 0.010
GUIDE_WHEEL_THICKNESS = 0.008
GUIDE_WHEEL_Z = WEAR_STRIP_Z
GUIDE_WHEEL_X_OFFSET = 0.050
GUIDE_WHEEL_CENTER_Y = (BEAM_WIDTH / 2.0) + WEAR_STRIP_THICKNESS + GUIDE_WHEEL_RADIUS
GUIDE_GUARD_DEPTH = 0.010
GUIDE_AXLE_RADIUS = 0.0042
GUIDE_BOLT_HEAD_RADIUS = 0.0066
GUIDE_BOLT_HEAD_LENGTH = 0.006

FORK_ARM_THICKNESS = 0.014
FORK_ARM_DEPTH = 0.028
FORK_ARM_HEIGHT = 0.050
FORK_ARM_X = 0.050
FORK_ARM_CENTER_Y = 0.075
FORK_RIB_LENGTH = 0.016
FORK_RIB_DEPTH = 0.020
FORK_RIB_HEIGHT = 0.010
FORK_RIB_X = 0.048
FORK_RIB_CENTER_Y = 0.061
FORK_RIB_CENTER_Z = -0.056
ROLLER_CENTER_Y = 0.095
ROLLER_CENTER_Z = -0.082
FORK_BORE_RADIUS = 0.015
ROLLER_BOLT_HEAD_RADIUS = 0.008
ROLLER_BOLT_HEAD_LENGTH = 0.008

ROLLER_TREAD_LENGTH = 0.056
ROLLER_TREAD_RADIUS = 0.033
ROLLER_END_CAP_LENGTH = 0.004
ROLLER_END_CAP_RADIUS = 0.027
ROLLER_JOURNAL_LENGTH = 0.086
ROLLER_JOURNAL_RADIUS = 0.015
ROLLER_SHOULDER_LENGTH = 0.004
ROLLER_SHOULDER_RADIUS = 0.022

SLIDE_TRAVEL = 0.23


def _cylinder_x(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane(
            "YZ",
            origin=(center[0] - (length / 2.0), center[1], center[2]),
        )
        .circle(radius)
        .extrude(length)
    )


def _cylinder_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane(
            "XZ",
            origin=(center[0], center[1] - (length / 2.0), center[2]),
        )
        .circle(radius)
        .extrude(length)
    )


def _cylinder_z(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane(
            "XY",
            origin=(center[0], center[1], center[2] - (length / 2.0)),
        )
        .circle(radius)
        .extrude(length)
    )


def _beam_body_shape() -> cq.Workplane:
    beam = cq.Workplane("XY").box(BEAM_LENGTH, BEAM_WIDTH, BEAM_HEIGHT)
    beam = beam.edges("|X").fillet(0.004)

    for side in (-1.0, 1.0):
        strip = cq.Workplane("XY").box(
            WEAR_STRIP_LENGTH,
            WEAR_STRIP_THICKNESS,
            WEAR_STRIP_HEIGHT,
        )
        strip = strip.translate(
            (
                0.0,
                side * ((BEAM_WIDTH / 2.0) + (WEAR_STRIP_THICKNESS / 2.0)),
                WEAR_STRIP_Z,
            )
        )
        beam = beam.union(strip)

    return beam


def _hanger_shape(x: float) -> cq.Workplane:
    lug = cq.Workplane("XY").box(0.060, 0.014, 0.048)
    lug = lug.translate((x, 0.0, (BEAM_HEIGHT / 2.0) + 0.024))
    lug_hole = _cylinder_y(0.007, 0.020, (x, 0.0, (BEAM_HEIGHT / 2.0) + 0.028))
    gusset = cq.Workplane("XY").box(0.030, 0.024, 0.016)
    gusset = gusset.translate((x, 0.0, (BEAM_HEIGHT / 2.0) + 0.008))
    return lug.cut(lug_hole).union(gusset)


def _end_cap_shape(side: float) -> cq.Workplane:
    cap = cq.Workplane("XY").box(
        END_CAP_THICKNESS,
        END_CAP_WIDTH,
        END_CAP_HEIGHT,
    )
    cap = cap.edges("|X").fillet(0.005)
    cap = cap.translate(
        (
            side * ((BEAM_LENGTH / 2.0) + (END_CAP_THICKNESS / 2.0) - 0.001),
            0.0,
            0.0,
        )
    )

    face_pocket = cq.Workplane("XY").box(
        END_CAP_THICKNESS * 0.55,
        END_CAP_WIDTH * 0.62,
        END_CAP_HEIGHT * 0.58,
    )
    face_pocket = face_pocket.translate(
        (
            side * ((BEAM_LENGTH / 2.0) + (END_CAP_THICKNESS * 0.30)),
            0.0,
            0.0,
        )
    )
    cap = cap.cut(face_pocket)

    for y in (-0.022, 0.022):
        for z in (-0.016, 0.016):
            bolt = _cylinder_x(
                0.0043,
                0.005,
                (
                    side * ((BEAM_LENGTH / 2.0) + END_CAP_THICKNESS - 0.0025),
                    y,
                    z,
                ),
            )
            cap = cap.union(bolt)

    return cap


def _wiper_shape(side: float) -> cq.Workplane:
    wiper = cq.Workplane("XY").box(
        WIPER_THICKNESS,
        CARRIAGE_BRIDGE_WIDTH,
        WIPER_HEIGHT,
    )
    wiper = wiper.translate(
        (
            side * ((CARRIAGE_LENGTH / 2.0) - (WIPER_THICKNESS / 2.0)),
            0.0,
            0.012,
        )
    )
    slot = cq.Workplane("XY").box(
        WIPER_THICKNESS + 0.002,
        WIPER_SLOT_WIDTH,
        WIPER_HEIGHT + 0.002,
    )
    slot = slot.translate(
        (
            side * ((CARRIAGE_LENGTH / 2.0) - (WIPER_THICKNESS / 2.0)),
            0.0,
            0.006,
        )
    )
    return wiper.cut(slot)


def _carriage_body_shape() -> cq.Workplane:
    roof = cq.Workplane("XY").box(
        CARRIAGE_LENGTH,
        CARRIAGE_BRIDGE_WIDTH,
        CARRIAGE_BRIDGE_THICKNESS,
    )
    roof = roof.translate((0.0, 0.0, CARRIAGE_BRIDGE_Z))

    carriage = roof

    for side in (-1.0, 1.0):
        cheek = cq.Workplane("XY").box(
            CARRIAGE_LENGTH,
            CARRIAGE_SIDE_THICKNESS,
            CARRIAGE_SIDE_HEIGHT,
        )
        cheek = cheek.translate(
            (
                0.0,
                side * CARRIAGE_SIDE_CENTER_Y,
                CARRIAGE_SIDE_CENTER_Z,
            )
        )
        carriage = carriage.union(cheek)

    nose = cq.Workplane("XY").box(
        NOSE_BLOCK_LENGTH,
        NOSE_BLOCK_DEPTH,
        NOSE_BLOCK_HEIGHT,
    )
    nose = nose.translate(
        (0.0, NOSE_BLOCK_CENTER_Y, NOSE_BLOCK_CENTER_Z)
    )
    carriage = carriage.union(nose)

    return carriage


def _top_pad_shape(x: float, y: float) -> cq.Workplane:
    return cq.Workplane("XY").box(
        TOP_PAD_LENGTH,
        TOP_PAD_WIDTH,
        TOP_PAD_THICKNESS,
    ).translate((x, y, (BEAM_HEIGHT / 2.0) + (TOP_PAD_THICKNESS / 2.0)))


def _guide_guard_shape(x: float, side: float) -> cq.Workplane:
    return cq.Workplane("XY").box(
        0.030,
        GUIDE_GUARD_DEPTH,
        0.014,
    ).translate((x, side * 0.050, 0.004))


def _guide_wheel_shape(x: float, side: float) -> cq.Workplane:
    return _cylinder_z(
        GUIDE_WHEEL_RADIUS,
        GUIDE_WHEEL_THICKNESS,
        (x, side * GUIDE_WHEEL_CENTER_Y, GUIDE_WHEEL_Z),
    )


def _guide_axle_shape(x: float, side: float) -> cq.Workplane:
    return _cylinder_y(
        GUIDE_AXLE_RADIUS,
        0.020,
        (
            x,
            side * (0.058),
            GUIDE_WHEEL_Z,
        ),
    )


def _guide_bolt_head_shape(x: float, side: float) -> cq.Workplane:
    return _cylinder_y(
        GUIDE_BOLT_HEAD_RADIUS,
        GUIDE_BOLT_HEAD_LENGTH,
        (
            x,
            side * ((CARRIAGE_SIDE_CENTER_Y - (CARRIAGE_SIDE_THICKNESS / 2.0)) + (GUIDE_BOLT_HEAD_LENGTH / 2.0)),
            GUIDE_WHEEL_Z,
        ),
    )


def _fork_arm_shape(side: float) -> cq.Workplane:
    arm = cq.Workplane("XY").box(
        FORK_ARM_THICKNESS,
        FORK_ARM_DEPTH,
        FORK_ARM_HEIGHT,
    )
    arm = arm.translate(
        (
            side * FORK_ARM_X,
            FORK_ARM_CENTER_Y,
            ROLLER_CENTER_Z,
        )
    )
    bore = _cylinder_x(
        FORK_BORE_RADIUS,
        FORK_ARM_THICKNESS + 0.010,
        (
            side * FORK_ARM_X,
            ROLLER_CENTER_Y,
            ROLLER_CENTER_Z,
        ),
    )
    return arm.cut(bore)


def _roller_tread_shape() -> cq.Workplane:
    return _cylinder_x(ROLLER_TREAD_RADIUS, ROLLER_TREAD_LENGTH, (0.0, 0.0, 0.0))


def _roller_end_cap_shape(side: float) -> cq.Workplane:
    return _cylinder_x(
        ROLLER_END_CAP_RADIUS,
        ROLLER_END_CAP_LENGTH,
        (
            side * ((ROLLER_TREAD_LENGTH / 2.0) + (ROLLER_END_CAP_LENGTH / 2.0)),
            0.0,
            0.0,
        ),
    )


def _roller_journal_shape(side: float) -> cq.Workplane:
    return _cylinder_x(
        ROLLER_JOURNAL_RADIUS,
        0.014,
        (
            side * 0.039,
            0.0,
            0.0,
        ),
    )


def _roller_shoulder_shape(side: float) -> cq.Workplane:
    return _cylinder_x(
        ROLLER_SHOULDER_RADIUS,
        ROLLER_SHOULDER_LENGTH,
        (
            side * ((ROLLER_TREAD_LENGTH / 2.0) + (ROLLER_SHOULDER_LENGTH / 2.0)),
            0.0,
            0.0,
        ),
    )


def _roller_support_bolt_shape(side: float) -> cq.Workplane:
    return _cylinder_x(
        ROLLER_BOLT_HEAD_RADIUS,
        ROLLER_BOLT_HEAD_LENGTH,
        (
            side * (FORK_ARM_X + (FORK_ARM_THICKNESS / 2.0) + (ROLLER_BOLT_HEAD_LENGTH / 2.0)),
            ROLLER_CENTER_Y,
            ROLLER_CENTER_Z,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hanging_beam_slide")

    model.material("beam_steel", rgba=(0.60, 0.63, 0.68, 1.0))
    model.material("cap_dark", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("carriage_alloy", rgba=(0.73, 0.75, 0.79, 1.0))
    model.material("wiper_black", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("fastener_steel", rgba=(0.55, 0.57, 0.61, 1.0))
    model.material("roller_poly", rgba=(0.18, 0.19, 0.20, 1.0))

    beam = model.part("beam")
    beam.visual(
        mesh_from_cadquery(_beam_body_shape(), "beam_body"),
        material="beam_steel",
        name="beam_body",
    )
    beam.visual(
        mesh_from_cadquery(_hanger_shape(-SUSPENSION_LUG_SPACING / 2.0), "beam_hanger_left"),
        material="beam_steel",
        name="left_hanger",
    )
    beam.visual(
        mesh_from_cadquery(_hanger_shape(SUSPENSION_LUG_SPACING / 2.0), "beam_hanger_right"),
        material="beam_steel",
        name="right_hanger",
    )
    beam.visual(
        mesh_from_cadquery(_end_cap_shape(-1.0), "beam_cap_left"),
        material="cap_dark",
        name="left_end_cap",
    )
    beam.visual(
        mesh_from_cadquery(_end_cap_shape(1.0), "beam_cap_right"),
        material="cap_dark",
        name="right_end_cap",
    )
    beam.inertial = Inertial.from_geometry(
        Box(
            (
                BEAM_LENGTH + (2.0 * END_CAP_THICKNESS),
                END_CAP_WIDTH,
                max(END_CAP_HEIGHT, BEAM_HEIGHT + 0.048),
            )
        ),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((CARRIAGE_LENGTH, CARRIAGE_BRIDGE_WIDTH, CARRIAGE_BRIDGE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_BRIDGE_Z)),
        material="carriage_alloy",
        name="carriage_roof",
    )
    carriage.visual(
        Box((CARRIAGE_LENGTH, CARRIAGE_SIDE_THICKNESS, CARRIAGE_SIDE_HEIGHT)),
        origin=Origin(xyz=(0.0, CARRIAGE_SIDE_CENTER_Y, CARRIAGE_SIDE_CENTER_Z)),
        material="carriage_alloy",
        name="right_cheek",
    )
    carriage.visual(
        Box((CARRIAGE_LENGTH, CARRIAGE_SIDE_THICKNESS, CARRIAGE_SIDE_HEIGHT)),
        origin=Origin(xyz=(0.0, -CARRIAGE_SIDE_CENTER_Y, CARRIAGE_SIDE_CENTER_Z)),
        material="carriage_alloy",
        name="left_cheek",
    )
    carriage.visual(
        Box((NOSE_BLOCK_LENGTH, NOSE_BLOCK_DEPTH, NOSE_BLOCK_HEIGHT)),
        origin=Origin(xyz=(0.0, NOSE_BLOCK_CENTER_Y, NOSE_BLOCK_CENTER_Z)),
        material="carriage_alloy",
        name="nose_block",
    )
    carriage.visual(
        mesh_from_cadquery(_wiper_shape(-1.0), "carriage_wiper_left"),
        material="wiper_black",
        name="left_wiper",
    )
    carriage.visual(
        mesh_from_cadquery(_wiper_shape(1.0), "carriage_wiper_right"),
        material="wiper_black",
        name="right_wiper",
    )
    carriage.visual(
        Box((FORK_ARM_THICKNESS, FORK_ARM_DEPTH, FORK_ARM_HEIGHT)),
        origin=Origin(xyz=(-FORK_ARM_X, FORK_ARM_CENTER_Y, ROLLER_CENTER_Z)),
        material="carriage_alloy",
        name="left_fork_arm",
    )
    carriage.visual(
        Box((FORK_ARM_THICKNESS, FORK_ARM_DEPTH, FORK_ARM_HEIGHT)),
        origin=Origin(xyz=(FORK_ARM_X, FORK_ARM_CENTER_Y, ROLLER_CENTER_Z)),
        material="carriage_alloy",
        name="right_fork_arm",
    )
    carriage.visual(
        Box((FORK_RIB_LENGTH, FORK_RIB_DEPTH, FORK_RIB_HEIGHT)),
        origin=Origin(xyz=(-FORK_RIB_X, FORK_RIB_CENTER_Y, FORK_RIB_CENTER_Z)),
        material="carriage_alloy",
        name="left_fork_rib",
    )
    carriage.visual(
        Box((FORK_RIB_LENGTH, FORK_RIB_DEPTH, FORK_RIB_HEIGHT)),
        origin=Origin(xyz=(FORK_RIB_X, FORK_RIB_CENTER_Y, FORK_RIB_CENTER_Z)),
        material="carriage_alloy",
        name="right_fork_rib",
    )
    for x in (-GUIDE_WHEEL_X_OFFSET, GUIDE_WHEEL_X_OFFSET):
        for side in (-1.0, 1.0):
            side_name = "r" if side > 0 else "l"
            x_name = "f" if x > 0 else "b"
            carriage.visual(
                Box((0.030, 0.008, 0.014)),
                origin=Origin(xyz=(x, side * 0.055, 0.004)),
                material="carriage_alloy",
                name=f"wheel_guard_{side_name}_{x_name}",
            )
            carriage.visual(
                Cylinder(radius=GUIDE_WHEEL_RADIUS, length=GUIDE_WHEEL_THICKNESS),
                origin=Origin(xyz=(x, side * GUIDE_WHEEL_CENTER_Y, GUIDE_WHEEL_Z)),
                material="fastener_steel",
                name=f"guide_wheel_{side_name}_{x_name}",
            )
            carriage.visual(
                Cylinder(radius=GUIDE_AXLE_RADIUS, length=0.020),
                origin=Origin(
                    xyz=(x, side * 0.057, GUIDE_WHEEL_Z),
                    rpy=(-pi / 2.0, 0.0, 0.0),
                ),
                material="fastener_steel",
                name=f"guide_axle_{side_name}_{x_name}",
            )
            carriage.visual(
                Cylinder(radius=GUIDE_BOLT_HEAD_RADIUS, length=GUIDE_BOLT_HEAD_LENGTH),
                origin=Origin(
                    xyz=(x, side * 0.066, GUIDE_WHEEL_Z),
                    rpy=(-pi / 2.0, 0.0, 0.0),
                ),
                material="fastener_steel",
                name=f"guide_bolt_{side_name}_{x_name}",
            )
    carriage.visual(
        Cylinder(radius=ROLLER_BOLT_HEAD_RADIUS, length=ROLLER_BOLT_HEAD_LENGTH),
        origin=Origin(
            xyz=(-0.059, ROLLER_CENTER_Y, ROLLER_CENTER_Z),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="fastener_steel",
        name="left_shoulder_bolt",
    )
    carriage.visual(
        Cylinder(radius=ROLLER_BOLT_HEAD_RADIUS, length=ROLLER_BOLT_HEAD_LENGTH),
        origin=Origin(
            xyz=(0.059, ROLLER_CENTER_Y, ROLLER_CENTER_Z),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="fastener_steel",
        name="right_shoulder_bolt",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_LENGTH, 0.150, CARRIAGE_TOP_Z - CARRIAGE_BOTTOM_Z)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
    )

    roller_head = model.part("roller_head")
    roller_head.visual(
        Cylinder(radius=ROLLER_TREAD_RADIUS, length=ROLLER_TREAD_LENGTH),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="roller_poly",
        name="roller_tread",
    )
    roller_head.visual(
        Cylinder(radius=ROLLER_END_CAP_RADIUS, length=ROLLER_END_CAP_LENGTH),
        origin=Origin(
            xyz=(
                -((ROLLER_TREAD_LENGTH / 2.0) + (ROLLER_END_CAP_LENGTH / 2.0)),
                0.0,
                0.0,
            ),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="fastener_steel",
        name="left_end_cap",
    )
    roller_head.visual(
        Cylinder(radius=ROLLER_END_CAP_RADIUS, length=ROLLER_END_CAP_LENGTH),
        origin=Origin(
            xyz=(
                (ROLLER_TREAD_LENGTH / 2.0) + (ROLLER_END_CAP_LENGTH / 2.0),
                0.0,
                0.0,
            ),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="fastener_steel",
        name="right_end_cap",
    )
    roller_head.visual(
        Cylinder(radius=ROLLER_JOURNAL_RADIUS, length=0.015),
        origin=Origin(xyz=(-0.0355, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="fastener_steel",
        name="left_journal",
    )
    roller_head.visual(
        Cylinder(radius=ROLLER_JOURNAL_RADIUS, length=0.015),
        origin=Origin(xyz=(0.0355, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="fastener_steel",
        name="right_journal",
    )
    roller_head.inertial = Inertial.from_geometry(
        Cylinder(radius=ROLLER_TREAD_RADIUS, length=ROLLER_JOURNAL_LENGTH),
        mass=0.75,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    model.articulation(
        "beam_to_carriage",
        ArticulationType.PRISMATIC,
        parent=beam,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-SLIDE_TRAVEL,
            upper=SLIDE_TRAVEL,
            effort=1800.0,
            velocity=0.35,
        ),
    )
    model.articulation(
        "carriage_to_roller",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=roller_head,
        origin=Origin(xyz=(0.0, ROLLER_CENTER_Y, ROLLER_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=12.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    beam = object_model.get_part("beam")
    carriage = object_model.get_part("carriage")
    roller_head = object_model.get_part("roller_head")
    slide = object_model.get_articulation("beam_to_carriage")
    roller_spin = object_model.get_articulation("carriage_to_roller")

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
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "slide_axis_aligned_to_beam",
        tuple(slide.axis) == (1.0, 0.0, 0.0),
        details=f"expected beam-axis slide, got {slide.axis}",
    )
    ctx.check(
        "roller_uses_axial_spin",
        tuple(roller_spin.axis) == (1.0, 0.0, 0.0),
        details=f"expected roller axle on x, got {roller_spin.axis}",
    )

    ctx.expect_contact(
        carriage,
        beam,
        name="carriage_is_supported_on_beam",
    )
    ctx.expect_overlap(
        carriage,
        beam,
        axes="xy",
        min_overlap=0.070,
        name="carriage_visually_straddles_beam",
    )
    ctx.expect_origin_gap(
        roller_head,
        carriage,
        axis="y",
        min_gap=0.075,
        max_gap=0.095,
        name="roller_projects_from_carriage_face",
    )
    ctx.expect_overlap(
        roller_head,
        carriage,
        axes="xz",
        min_overlap=0.050,
        name="roller_sits_within_fork_span",
    )
    ctx.expect_contact(
        roller_head,
        carriage,
        elem_a="left_journal",
        elem_b="left_fork_arm",
        name="left_journal_is_borne_by_fork",
    )
    ctx.expect_contact(
        roller_head,
        carriage,
        elem_a="right_journal",
        elem_b="right_fork_arm",
        name="right_journal_is_borne_by_fork",
    )

    with ctx.pose({slide: slide.motion_limits.lower}):
        ctx.expect_gap(
            carriage,
            beam,
            axis="x",
            negative_elem="left_end_cap",
            min_gap=0.020,
            name="left_limit_clears_end_cap",
        )

    with ctx.pose({slide: slide.motion_limits.upper}):
        ctx.expect_gap(
            beam,
            carriage,
            axis="x",
            positive_elem="right_end_cap",
            min_gap=0.020,
            name="right_limit_clears_end_cap",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
