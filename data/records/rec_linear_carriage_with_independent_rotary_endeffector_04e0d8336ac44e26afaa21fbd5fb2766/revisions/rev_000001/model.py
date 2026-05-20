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


BASE_LENGTH = 0.64
BASE_WIDTH = 0.16
BASE_THICKNESS = 0.016
RAIL_BODY_LENGTH = 0.56
RAIL_BODY_WIDTH = 0.082
RAIL_BODY_HEIGHT = 0.014
COVER_STRIP_LENGTH = 0.50
COVER_STRIP_WIDTH = 0.018
COVER_STRIP_HEIGHT = 0.004
COVER_STRIP_OFFSET_Y = 0.022
STOP_BLOCK_X = 0.286
STOP_BLOCK_Y = 0.045
STOP_BLOCK_LENGTH = 0.012
STOP_BLOCK_WIDTH = 0.028
STOP_BLOCK_HEIGHT = 0.016
MOUNT_HOLE_X = 0.24
MOUNT_HOLE_Y = 0.055

PRISMATIC_HOME_Z = BASE_THICKNESS + RAIL_BODY_HEIGHT + COVER_STRIP_HEIGHT
TRAVEL_HALF_RANGE = 0.14

CARRIAGE_LENGTH = 0.22
CARRIAGE_WIDTH = 0.19
CARRIAGE_BEARING_LENGTH = 0.19
CARRIAGE_BEARING_WIDTH = 0.040
CARRIAGE_BEARING_Y = 0.039
CARRIAGE_RUNNER_LENGTH = 0.18
CARRIAGE_RUNNER_WIDTH = 0.018
CARRIAGE_RUNNER_HEIGHT = 0.006
CARRIAGE_RUNNER_Y = COVER_STRIP_OFFSET_Y
CARRIAGE_BRIDGE_HEIGHT = 0.040
CARRIAGE_BRIDGE_Z = 0.046
CARRIAGE_SUPPORT_HEIGHT = 0.018
CARRIAGE_SUPPORT_Z = 0.076
CARRIAGE_PEDESTAL_RADIUS = 0.064
CARRIAGE_PEDESTAL_HEIGHT = 0.018
CARRIAGE_TOP_Z = CARRIAGE_BRIDGE_Z + CARRIAGE_BRIDGE_HEIGHT + CARRIAGE_PEDESTAL_HEIGHT
SPINDLE_OPENING_RADIUS = 0.032
WIPER_THICKNESS = 0.004
WIPER_WIDTH = 0.12
WIPER_HEIGHT = 0.030
WIPER_Z = 0.021

SPINDLE_FLANGE_RADIUS = 0.056
SPINDLE_FLANGE_HEIGHT = 0.012
SPINDLE_HOUSING_RADIUS = 0.044
SPINDLE_HOUSING_HEIGHT = 0.070
SPINDLE_CAP_RADIUS = 0.048
SPINDLE_CAP_HEIGHT = 0.014
SPINDLE_CAP_Z = SPINDLE_FLANGE_HEIGHT + SPINDLE_HOUSING_HEIGHT
SPINDLE_SERVICE_BOX_X = 0.028
SPINDLE_SERVICE_BOX_Y = 0.040
SPINDLE_SERVICE_BOX_Z = 0.022
SPINDLE_SERVICE_BOX_OFFSET_X = 0.052
SPINDLE_SERVICE_BOX_OFFSET_Z = 0.060
SPINDLE_SERVICE_NECK_X = 0.014
SPINDLE_SERVICE_NECK_Y = 0.024
SPINDLE_SERVICE_NECK_Z = 0.022
SPINDLE_SERVICE_NECK_OFFSET_X = 0.038

SPINDLE_NOSE_COLLAR_RADIUS = 0.030
SPINDLE_NOSE_COLLAR_HEIGHT = 0.012
SPINDLE_NOSE_BODY_RADIUS = 0.023
SPINDLE_NOSE_BODY_HEIGHT = 0.030
SPINDLE_COLLET_FLAT = 0.038
SPINDLE_COLLET_HEIGHT = 0.010
SPINDLE_NOSE_MIN_Z = -(
    SPINDLE_NOSE_COLLAR_HEIGHT + SPINDLE_NOSE_BODY_HEIGHT + SPINDLE_COLLET_HEIGHT
)


def _box_at(length: float, width: float, height: float, *, x: float, y: float, z: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(length, width, height, centered=(True, True, False))
        .translate((x, y, z))
    )


def _cylinder_at(radius: float, height: float, *, x: float, y: float, z: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(height).translate((x, y, z))


def _base_guide_shape() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS, centered=(True, True, False))
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-MOUNT_HOLE_X, -MOUNT_HOLE_Y),
                (-MOUNT_HOLE_X, MOUNT_HOLE_Y),
                (MOUNT_HOLE_X, -MOUNT_HOLE_Y),
                (MOUNT_HOLE_X, MOUNT_HOLE_Y),
            ]
        )
        .hole(0.009)
    )

    rail = _box_at(
        RAIL_BODY_LENGTH,
        RAIL_BODY_WIDTH,
        RAIL_BODY_HEIGHT,
        x=0.0,
        y=0.0,
        z=BASE_THICKNESS,
    )

    stop_blocks = (
        _box_at(
            STOP_BLOCK_LENGTH,
            STOP_BLOCK_WIDTH,
            STOP_BLOCK_HEIGHT,
            x=STOP_BLOCK_X,
            y=STOP_BLOCK_Y,
            z=BASE_THICKNESS,
        )
        .union(
            _box_at(
                STOP_BLOCK_LENGTH,
                STOP_BLOCK_WIDTH,
                STOP_BLOCK_HEIGHT,
                x=STOP_BLOCK_X,
                y=-STOP_BLOCK_Y,
                z=BASE_THICKNESS,
            )
        )
        .union(
            _box_at(
                STOP_BLOCK_LENGTH,
                STOP_BLOCK_WIDTH,
                STOP_BLOCK_HEIGHT,
                x=-STOP_BLOCK_X,
                y=STOP_BLOCK_Y,
                z=BASE_THICKNESS,
            )
        )
        .union(
            _box_at(
                STOP_BLOCK_LENGTH,
                STOP_BLOCK_WIDTH,
                STOP_BLOCK_HEIGHT,
                x=-STOP_BLOCK_X,
                y=-STOP_BLOCK_Y,
                z=BASE_THICKNESS,
            )
        )
    )

    return base.union(rail).union(stop_blocks)


def _carriage_body_shape() -> cq.Workplane:
    upper_bridge = _box_at(
        CARRIAGE_LENGTH,
        CARRIAGE_WIDTH,
        CARRIAGE_BRIDGE_HEIGHT,
        x=0.0,
        y=0.0,
        z=CARRIAGE_BRIDGE_Z,
    )
    left_bearing = _box_at(
        CARRIAGE_BEARING_LENGTH,
        CARRIAGE_BEARING_WIDTH,
        CARRIAGE_BRIDGE_Z - CARRIAGE_RUNNER_HEIGHT,
        x=0.0,
        y=CARRIAGE_BEARING_Y,
        z=CARRIAGE_RUNNER_HEIGHT,
    )
    right_bearing = _box_at(
        CARRIAGE_BEARING_LENGTH,
        CARRIAGE_BEARING_WIDTH,
        CARRIAGE_BRIDGE_Z - CARRIAGE_RUNNER_HEIGHT,
        x=0.0,
        y=-CARRIAGE_BEARING_Y,
        z=CARRIAGE_RUNNER_HEIGHT,
    )
    left_runner = _box_at(
        CARRIAGE_RUNNER_LENGTH,
        CARRIAGE_RUNNER_WIDTH,
        CARRIAGE_RUNNER_HEIGHT,
        x=0.0,
        y=CARRIAGE_RUNNER_Y,
        z=0.0,
    )
    right_runner = _box_at(
        CARRIAGE_RUNNER_LENGTH,
        CARRIAGE_RUNNER_WIDTH,
        CARRIAGE_RUNNER_HEIGHT,
        x=0.0,
        y=-CARRIAGE_RUNNER_Y,
        z=0.0,
    )
    front_endcap = _box_at(
        0.012,
        0.124,
        0.036,
        x=CARRIAGE_LENGTH / 2.0 - 0.006,
        y=0.0,
        z=0.006,
    )
    rear_endcap = _box_at(
        0.012,
        0.124,
        0.036,
        x=-(CARRIAGE_LENGTH / 2.0 - 0.006),
        y=0.0,
        z=0.006,
    )
    support_cross_x = _box_at(
        0.110,
        0.034,
        CARRIAGE_SUPPORT_HEIGHT,
        x=0.0,
        y=0.0,
        z=CARRIAGE_SUPPORT_Z,
    )
    support_cross_y = _box_at(
        0.034,
        0.110,
        CARRIAGE_SUPPORT_HEIGHT,
        x=0.0,
        y=0.0,
        z=CARRIAGE_SUPPORT_Z,
    )
    pedestal_ring = _cylinder_at(
        CARRIAGE_PEDESTAL_RADIUS,
        CARRIAGE_PEDESTAL_HEIGHT,
        x=0.0,
        y=0.0,
        z=CARRIAGE_BRIDGE_Z + CARRIAGE_BRIDGE_HEIGHT,
    )

    body = (
        upper_bridge.union(left_bearing)
        .union(right_bearing)
        .union(left_runner)
        .union(right_runner)
        .union(front_endcap)
        .union(rear_endcap)
        .union(support_cross_x)
        .union(support_cross_y)
        .union(pedestal_ring)
    )

    spindle_opening = _cylinder_at(
        SPINDLE_OPENING_RADIUS,
        CARRIAGE_TOP_Z + 0.004,
        x=0.0,
        y=0.0,
        z=0.0,
    )
    left_side_pocket = _box_at(
        0.130,
        0.014,
        0.028,
        x=0.0,
        y=CARRIAGE_WIDTH / 2.0 - 0.007,
        z=0.054,
    )
    right_side_pocket = _box_at(
        0.130,
        0.014,
        0.028,
        x=0.0,
        y=-(CARRIAGE_WIDTH / 2.0 - 0.007),
        z=0.054,
    )

    return body.cut(spindle_opening)


def _spindle_housing_shape() -> cq.Workplane:
    housing = _cylinder_at(
        SPINDLE_HOUSING_RADIUS,
        SPINDLE_HOUSING_HEIGHT,
        x=0.0,
        y=0.0,
        z=SPINDLE_FLANGE_HEIGHT,
    )
    service_neck = _box_at(
        SPINDLE_SERVICE_NECK_X,
        SPINDLE_SERVICE_NECK_Y,
        SPINDLE_SERVICE_NECK_Z,
        x=SPINDLE_SERVICE_NECK_OFFSET_X,
        y=0.0,
        z=SPINDLE_SERVICE_BOX_OFFSET_Z,
    )
    service_box = _box_at(
        SPINDLE_SERVICE_BOX_X,
        SPINDLE_SERVICE_BOX_Y,
        SPINDLE_SERVICE_BOX_Z,
        x=SPINDLE_SERVICE_BOX_OFFSET_X,
        y=0.0,
        z=SPINDLE_SERVICE_BOX_OFFSET_Z,
    )
    return housing.union(service_neck).union(service_box)


def _spindle_nose_shape() -> cq.Workplane:
    collar = _cylinder_at(
        SPINDLE_NOSE_COLLAR_RADIUS,
        SPINDLE_NOSE_COLLAR_HEIGHT,
        x=0.0,
        y=0.0,
        z=-SPINDLE_NOSE_COLLAR_HEIGHT,
    )
    nose_body = _cylinder_at(
        SPINDLE_NOSE_BODY_RADIUS,
        SPINDLE_NOSE_BODY_HEIGHT,
        x=0.0,
        y=0.0,
        z=-(SPINDLE_NOSE_COLLAR_HEIGHT + SPINDLE_NOSE_BODY_HEIGHT),
    )
    collet_nut = (
        cq.Workplane("XY")
        .polygon(6, SPINDLE_COLLET_FLAT)
        .extrude(SPINDLE_COLLET_HEIGHT)
        .translate((0.0, 0.0, SPINDLE_NOSE_MIN_Z))
    )
    return collar.union(nose_body).union(collet_nut)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inspection_rail_module")

    model.material("base_metal", rgba=(0.63, 0.66, 0.70, 1.0))
    model.material("carriage_metal", rgba=(0.76, 0.78, 0.81, 1.0))
    model.material("cover_dark", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("spindle_body", rgba=(0.54, 0.56, 0.60, 1.0))
    model.material("spindle_cap_dark", rgba=(0.20, 0.21, 0.24, 1.0))
    model.material("tool_steel", rgba=(0.72, 0.73, 0.75, 1.0))

    base_guide = model.part("base_guide")
    base_guide.visual(
        mesh_from_cadquery(_base_guide_shape(), "base_guide"),
        material="base_metal",
        name="guide_body",
    )
    base_guide.visual(
        Box((COVER_STRIP_LENGTH, COVER_STRIP_WIDTH, COVER_STRIP_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                COVER_STRIP_OFFSET_Y,
                BASE_THICKNESS + RAIL_BODY_HEIGHT + COVER_STRIP_HEIGHT / 2.0,
            )
        ),
        material="cover_dark",
        name="left_cover_strip",
    )
    base_guide.visual(
        Box((COVER_STRIP_LENGTH, COVER_STRIP_WIDTH, COVER_STRIP_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -COVER_STRIP_OFFSET_Y,
                BASE_THICKNESS + RAIL_BODY_HEIGHT + COVER_STRIP_HEIGHT / 2.0,
            )
        ),
        material="cover_dark",
        name="right_cover_strip",
    )
    base_guide.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS + RAIL_BODY_HEIGHT + STOP_BLOCK_HEIGHT)),
        mass=11.5,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                (BASE_THICKNESS + RAIL_BODY_HEIGHT + STOP_BLOCK_HEIGHT) / 2.0,
            )
        ),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_body_shape(), "carriage_body"),
        material="carriage_metal",
        name="carriage_body",
    )
    carriage.visual(
        Box((WIPER_THICKNESS, WIPER_WIDTH, WIPER_HEIGHT)),
        origin=Origin(
            xyz=(CARRIAGE_LENGTH / 2.0 - WIPER_THICKNESS / 2.0, 0.0, WIPER_Z)
        ),
        material="cover_dark",
        name="front_wiper",
    )
    carriage.visual(
        Box((WIPER_THICKNESS, WIPER_WIDTH, WIPER_HEIGHT)),
        origin=Origin(
            xyz=(-CARRIAGE_LENGTH / 2.0 + WIPER_THICKNESS / 2.0, 0.0, WIPER_Z)
        ),
        material="cover_dark",
        name="rear_wiper",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_TOP_Z)),
        mass=6.2,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_TOP_Z / 2.0)),
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=SPINDLE_FLANGE_RADIUS, length=SPINDLE_FLANGE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, SPINDLE_FLANGE_HEIGHT / 2.0)),
        material="spindle_body",
        name="spindle_flange",
    )
    spindle.visual(
        mesh_from_cadquery(_spindle_housing_shape(), "spindle_housing"),
        material="spindle_body",
        name="spindle_housing",
    )
    spindle.visual(
        Cylinder(radius=SPINDLE_CAP_RADIUS, length=SPINDLE_CAP_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, SPINDLE_CAP_Z + SPINDLE_CAP_HEIGHT / 2.0)),
        material="spindle_cap_dark",
        name="spindle_cap",
    )
    spindle.visual(
        mesh_from_cadquery(_spindle_nose_shape(), "spindle_nose"),
        material="tool_steel",
        name="spindle_nose",
    )
    spindle.inertial = Inertial.from_geometry(
        Box((0.14, 0.12, 0.148)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base_guide,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, PRISMATIC_HOME_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-TRAVEL_HALF_RANGE,
            upper=TRAVEL_HALF_RANGE,
            effort=1400.0,
            velocity=0.45,
        ),
    )
    model.articulation(
        "carriage_to_spindle",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-pi,
            upper=pi,
            effort=22.0,
            velocity=5.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_guide = object_model.get_part("base_guide")
    carriage = object_model.get_part("carriage")
    spindle = object_model.get_part("spindle")
    slide = object_model.get_articulation("base_to_carriage")
    head = object_model.get_articulation("carriage_to_spindle")

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
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=20,
        ignore_adjacent=False,
        ignore_fixed=True,
    )

    ctx.check(
        "primary_parts_present",
        all(part is not None for part in (base_guide, carriage, spindle)),
        details="base_guide, carriage, and spindle must all resolve",
    )
    ctx.check(
        "slide_axis_is_rail_x",
        tuple(slide.axis) == (1.0, 0.0, 0.0),
        details=f"expected prismatic axis (1,0,0), got {slide.axis}",
    )
    ctx.check(
        "spindle_axis_is_vertical",
        tuple(head.axis) == (0.0, 0.0, 1.0),
        details=f"expected spindle axis (0,0,1), got {head.axis}",
    )

    ctx.expect_contact(
        carriage,
        base_guide,
        elem_a="carriage_body",
        elem_b="left_cover_strip",
        name="left_bearing_runner_is_supported",
    )
    ctx.expect_contact(
        carriage,
        base_guide,
        elem_a="carriage_body",
        elem_b="right_cover_strip",
        name="right_bearing_runner_is_supported",
    )
    ctx.expect_contact(
        spindle,
        carriage,
        elem_a="spindle_flange",
        elem_b="carriage_body",
        name="spindle_flange_seats_on_pedestal",
    )
    ctx.expect_gap(
        spindle,
        carriage,
        axis="z",
        positive_elem="spindle_housing",
        negative_elem="carriage_body",
        min_gap=0.008,
        name="spindle_housing_clears_carriage_mass",
    )
    ctx.expect_overlap(
        spindle,
        carriage,
        axes="xy",
        elem_a="spindle_flange",
        elem_b="carriage_body",
        min_overlap=0.09,
        name="spindle_mount_stays_over_carriage_support",
    )

    with ctx.pose({slide: TRAVEL_HALF_RANGE, head: pi}):
        ctx.expect_within(
            carriage,
            base_guide,
            axes="x",
            margin=0.0,
            name="carriage_positive_travel_stays_over_base",
        )
        ctx.expect_gap(
            spindle,
            base_guide,
            axis="z",
            positive_elem="spindle_nose",
            negative_elem="guide_body",
            min_gap=0.045,
            name="spindle_nose_clears_guide_at_positive_end",
        )

    with ctx.pose({slide: -TRAVEL_HALF_RANGE, head: -pi}):
        ctx.expect_within(
            carriage,
            base_guide,
            axes="x",
            margin=0.0,
            name="carriage_negative_travel_stays_over_base",
        )
        ctx.expect_gap(
            spindle,
            base_guide,
            axis="z",
            positive_elem="spindle_nose",
            negative_elem="guide_body",
            min_gap=0.045,
            name="spindle_nose_clears_guide_at_negative_end",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
