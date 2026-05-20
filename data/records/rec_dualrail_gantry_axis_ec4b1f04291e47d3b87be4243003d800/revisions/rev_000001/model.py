from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


RAIL_LENGTH = 2.40
RAIL_CENTER_SPAN = 1.64
RAIL_X = RAIL_CENTER_SPAN / 2.0

PLINTH_SIZE = (0.16, RAIL_LENGTH, 0.08)
TRACK_SIZE = (0.09, RAIL_LENGTH, 0.03)
TRACK_TOP_Z = PLINTH_SIZE[2] + TRACK_SIZE[2]

CROSS_TIE_SIZE = (RAIL_CENTER_SPAN + PLINTH_SIZE[0], 0.12, 0.08)
FOOT_SIZE = (0.26, 0.24, 0.02)
END_STOP_SIZE = (0.08, 0.03, 0.05)

TRUCK_SIZE = (0.18, 0.28, 0.08)
UPRIGHT_SIZE = (0.10, 0.18, 1.14)
BEAM_SIZE = (1.58, 0.20, 0.18)
BEAM_BOTTOM_Z = TRUCK_SIZE[2] + UPRIGHT_SIZE[2]
GUIDE_SIZE = (1.26, 0.03, 0.03)
GUIDE_Y = 0.06
GUIDE_BOTTOM_Z = BEAM_BOTTOM_Z - GUIDE_SIZE[2]
CABLE_TRAY_SIZE = (1.10, 0.04, 0.05)

SLIDER_SIZE = (0.14, 0.024, 0.03)
SADDLE_SIZE = (0.18, 0.15, 0.07)
SIDE_PLATE_SIZE = (0.16, 0.012, 0.22)
POD_SIZE = (0.22, 0.10, 0.12)
MOTOR_BOX_SIZE = (0.09, 0.05, 0.08)
PAD_SIZE = (0.12, 0.06, 0.02)

BRIDGE_TRAVEL = 0.78
CARRIAGE_TRAVEL = 0.52


def _add_box(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    *,
    material: str,
    name: str,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_beam_gantry_shuttle")

    model.material("frame_graphite", rgba=(0.24, 0.26, 0.29, 1.0))
    model.material("rail_steel", rgba=(0.58, 0.61, 0.66, 1.0))
    model.material("bridge_aluminum", rgba=(0.77, 0.79, 0.81, 1.0))
    model.material("carriage_dark", rgba=(0.17, 0.18, 0.21, 1.0))
    model.material("safety_yellow", rgba=(0.84, 0.69, 0.15, 1.0))
    model.material("foot_black", rgba=(0.10, 0.11, 0.12, 1.0))

    base = model.part("base")
    _add_box(
        base,
        PLINTH_SIZE,
        (-RAIL_X, 0.0, PLINTH_SIZE[2] / 2.0),
        material="frame_graphite",
        name="left_plinth",
    )
    _add_box(
        base,
        PLINTH_SIZE,
        (RAIL_X, 0.0, PLINTH_SIZE[2] / 2.0),
        material="frame_graphite",
        name="right_plinth",
    )
    _add_box(
        base,
        TRACK_SIZE,
        (-RAIL_X, 0.0, PLINTH_SIZE[2] + (TRACK_SIZE[2] / 2.0)),
        material="rail_steel",
        name="left_track",
    )
    _add_box(
        base,
        TRACK_SIZE,
        (RAIL_X, 0.0, PLINTH_SIZE[2] + (TRACK_SIZE[2] / 2.0)),
        material="rail_steel",
        name="right_track",
    )
    for side, y in (("front", -1.04), ("rear", 1.04)):
        _add_box(
            base,
            CROSS_TIE_SIZE,
            (0.0, y, CROSS_TIE_SIZE[2] / 2.0),
            material="frame_graphite",
            name=f"{side}_cross_tie",
        )
    for x in (-RAIL_X, RAIL_X):
        for side, y in (("front", -1.04), ("rear", 1.04)):
            _add_box(
                base,
                FOOT_SIZE,
                (x, y, FOOT_SIZE[2] / 2.0),
                material="foot_black",
                name=f"{'left' if x < 0.0 else 'right'}_{side}_foot",
            )
    for x, label in ((-RAIL_X, "left"), (RAIL_X, "right")):
        for side, y in (("front", -0.98), ("rear", 0.98)):
            _add_box(
                base,
                END_STOP_SIZE,
                (x, y, TRACK_TOP_Z - (END_STOP_SIZE[2] / 2.0)),
                material="rail_steel",
                name=f"{label}_{side}_stop",
            )
    base.inertial = Inertial.from_geometry(
        Box((RAIL_CENTER_SPAN + 0.26, RAIL_LENGTH, TRACK_TOP_Z)),
        mass=210.0,
        origin=Origin(xyz=(0.0, 0.0, TRACK_TOP_Z / 2.0)),
    )

    bridge = model.part("bridge")
    _add_box(
        bridge,
        TRUCK_SIZE,
        (-RAIL_X, 0.0, TRUCK_SIZE[2] / 2.0),
        material="bridge_aluminum",
        name="left_truck",
    )
    _add_box(
        bridge,
        TRUCK_SIZE,
        (RAIL_X, 0.0, TRUCK_SIZE[2] / 2.0),
        material="bridge_aluminum",
        name="right_truck",
    )
    _add_box(
        bridge,
        UPRIGHT_SIZE,
        (-RAIL_X, 0.0, TRUCK_SIZE[2] + (UPRIGHT_SIZE[2] / 2.0)),
        material="bridge_aluminum",
        name="left_upright",
    )
    _add_box(
        bridge,
        UPRIGHT_SIZE,
        (RAIL_X, 0.0, TRUCK_SIZE[2] + (UPRIGHT_SIZE[2] / 2.0)),
        material="bridge_aluminum",
        name="right_upright",
    )
    _add_box(
        bridge,
        BEAM_SIZE,
        (0.0, 0.0, BEAM_BOTTOM_Z + (BEAM_SIZE[2] / 2.0)),
        material="bridge_aluminum",
        name="beam",
    )
    _add_box(
        bridge,
        GUIDE_SIZE,
        (0.0, GUIDE_Y, BEAM_BOTTOM_Z - (GUIDE_SIZE[2] / 2.0)),
        material="rail_steel",
        name="front_guide",
    )
    _add_box(
        bridge,
        GUIDE_SIZE,
        (0.0, -GUIDE_Y, BEAM_BOTTOM_Z - (GUIDE_SIZE[2] / 2.0)),
        material="rail_steel",
        name="rear_guide",
    )
    _add_box(
        bridge,
        CABLE_TRAY_SIZE,
        (
            0.0,
            (BEAM_SIZE[1] / 2.0) + (CABLE_TRAY_SIZE[1] / 2.0),
            BEAM_BOTTOM_Z + 0.02,
        ),
        material="carriage_dark",
        name="cable_tray",
    )
    bridge.inertial = Inertial.from_geometry(
        Box((RAIL_CENTER_SPAN + TRUCK_SIZE[0], TRUCK_SIZE[1], BEAM_BOTTOM_Z + BEAM_SIZE[2])),
        mass=95.0,
        origin=Origin(xyz=(0.0, 0.0, (BEAM_BOTTOM_Z + BEAM_SIZE[2]) / 2.0)),
    )

    carriage = model.part("carriage")
    _add_box(
        carriage,
        SLIDER_SIZE,
        (0.0, GUIDE_Y, -(SLIDER_SIZE[2] / 2.0)),
        material="rail_steel",
        name="front_slider",
    )
    _add_box(
        carriage,
        SLIDER_SIZE,
        (0.0, -GUIDE_Y, -(SLIDER_SIZE[2] / 2.0)),
        material="rail_steel",
        name="rear_slider",
    )
    _add_box(
        carriage,
        SADDLE_SIZE,
        (0.0, 0.0, -0.065),
        material="carriage_dark",
        name="saddle_block",
    )
    _add_box(
        carriage,
        SIDE_PLATE_SIZE,
        (0.0, 0.053, -0.185),
        material="carriage_dark",
        name="front_hanger",
    )
    _add_box(
        carriage,
        SIDE_PLATE_SIZE,
        (0.0, -0.053, -0.185),
        material="carriage_dark",
        name="rear_hanger",
    )
    _add_box(
        carriage,
        POD_SIZE,
        (0.0, 0.0, -0.33),
        material="safety_yellow",
        name="pod",
    )
    _add_box(
        carriage,
        MOTOR_BOX_SIZE,
        (0.0, (POD_SIZE[1] / 2.0) + (MOTOR_BOX_SIZE[1] / 2.0), -0.32),
        material="carriage_dark",
        name="drive_box",
    )
    _add_box(
        carriage,
        PAD_SIZE,
        (0.0, 0.0, -0.40),
        material="rail_steel",
        name="pickup_pad",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((POD_SIZE[0], 0.16, 0.40)),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.0, -0.20)),
    )

    model.articulation(
        "base_to_bridge",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bridge,
        origin=Origin(xyz=(0.0, 0.0, TRACK_TOP_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-BRIDGE_TRAVEL,
            upper=BRIDGE_TRAVEL,
            effort=1800.0,
            velocity=0.7,
        ),
    )
    model.articulation(
        "bridge_to_carriage",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, GUIDE_BOTTOM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-CARRIAGE_TRAVEL,
            upper=CARRIAGE_TRAVEL,
            effort=650.0,
            velocity=0.8,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    bridge = object_model.get_part("bridge")
    carriage = object_model.get_part("carriage")
    bridge_joint = object_model.get_articulation("base_to_bridge")
    carriage_joint = object_model.get_articulation("bridge_to_carriage")

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

    ctx.expect_contact(base, bridge, elem_a="left_track", elem_b="left_truck")
    ctx.expect_contact(base, bridge, elem_a="right_track", elem_b="right_truck")
    ctx.expect_contact(bridge, carriage, elem_a="front_guide", elem_b="front_slider")
    ctx.expect_contact(bridge, carriage, elem_a="rear_guide", elem_b="rear_slider")
    ctx.expect_gap(
        bridge,
        carriage,
        axis="z",
        positive_elem="beam",
        negative_elem="pod",
        min_gap=0.22,
        name="pod_hangs_below_beam",
    )
    ctx.check(
        "prismatic_axes_are_perpendicular",
        bridge_joint.articulation_type == ArticulationType.PRISMATIC
        and carriage_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(bridge_joint.axis) == (0.0, 1.0, 0.0)
        and tuple(carriage_joint.axis) == (1.0, 0.0, 0.0),
        details=(
            f"expected bridge axis (0,1,0) and carriage axis (1,0,0), got "
            f"{bridge_joint.axis} and {carriage_joint.axis}"
        ),
    )

    with ctx.pose({bridge_joint: 0.0, carriage_joint: 0.0}):
        bridge_home = ctx.part_world_position(bridge)
        carriage_home = ctx.part_world_position(carriage)
    with ctx.pose({bridge_joint: 0.35, carriage_joint: 0.0}):
        bridge_shifted = ctx.part_world_position(bridge)
    with ctx.pose({bridge_joint: 0.0, carriage_joint: 0.25}):
        carriage_shifted = ctx.part_world_position(carriage)

    bridge_delta = (
        bridge_shifted[0] - bridge_home[0],
        bridge_shifted[1] - bridge_home[1],
        bridge_shifted[2] - bridge_home[2],
    )
    carriage_delta = (
        carriage_shifted[0] - carriage_home[0],
        carriage_shifted[1] - carriage_home[1],
        carriage_shifted[2] - carriage_home[2],
    )
    ctx.check(
        "bridge_positive_travel_runs_along_y",
        abs(bridge_delta[0]) < 1e-6
        and bridge_delta[1] > 0.34
        and abs(bridge_delta[2]) < 1e-6,
        details=f"bridge delta was {bridge_delta}",
    )
    ctx.check(
        "carriage_positive_travel_runs_along_x",
        carriage_delta[0] > 0.24
        and abs(carriage_delta[1]) < 1e-6
        and abs(carriage_delta[2]) < 1e-6,
        details=f"carriage delta was {carriage_delta}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
