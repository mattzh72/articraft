from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


SHOULDER_NEUTRAL_PITCH = 0.70
ELBOW_NEUTRAL_PITCH = 0.18
NOZZLE_NEUTRAL_PITCH = -0.88

PROXIMAL_WAND_LENGTH = 0.34
DISTAL_WAND_LENGTH = 0.31

SHOULDER_GAP = 0.028
ELBOW_GAP = 0.028
NOZZLE_GAP = 0.024

PROXIMAL_TUBE_START = 0.045
PROXIMAL_TUBE_LENGTH = 0.245
DISTAL_TUBE_START = 0.040
DISTAL_TUBE_LENGTH = 0.220


def _add_cylinder_x(
    part,
    name: str,
    *,
    radius: float,
    length: float,
    center: tuple[float, float, float],
    material: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(0.0, pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _add_cylinder_y(
    part,
    name: str,
    *,
    radius: float,
    length: float,
    center: tuple[float, float, float],
    material: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_professional_vacuum")

    model.material("housing_dark", rgba=(0.15, 0.17, 0.19, 1.0))
    model.material("housing_mid", rgba=(0.28, 0.31, 0.34, 1.0))
    model.material("aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("datum_orange", rgba=(0.92, 0.42, 0.10, 1.0))
    model.material("datum_yellow", rgba=(0.93, 0.82, 0.16, 1.0))
    model.material("elastomer", rgba=(0.10, 0.10, 0.11, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.30, 0.20, 0.07)),
        origin=Origin(xyz=(-0.02, 0.0, 0.035)),
        material="housing_dark",
        name="base_chassis",
    )
    body.visual(
        Box((0.18, 0.14, 0.37)),
        origin=Origin(xyz=(-0.03, 0.0, 0.255)),
        material="housing_mid",
        name="main_body",
    )
    body.visual(
        Box((0.09, 0.18, 0.14)),
        origin=Origin(xyz=(-0.105, 0.0, 0.12)),
        material="housing_mid",
        name="rear_battery",
    )
    _add_cylinder_y(
        body,
        "motor_pod",
        radius=0.067,
        length=0.14,
        center=(0.0, 0.0, 0.50),
        material="housing_dark",
    )
    body.visual(
        Box((0.08, 0.09, 0.15)),
        origin=Origin(xyz=(0.057, 0.0, 0.515)),
        material="housing_mid",
        name="shoulder_tower",
    )
    body.visual(
        Box((0.026, 0.056, 0.012)),
        origin=Origin(xyz=(0.100, 0.0, 0.550)),
        material="housing_dark",
        name="shoulder_bridge",
    )
    body.visual(
        Box((0.018, 0.008, 0.05)),
        origin=Origin(xyz=(0.115, 0.018, 0.58)),
        material="aluminum",
        name="shoulder_ear_left",
    )
    body.visual(
        Box((0.018, 0.008, 0.05)),
        origin=Origin(xyz=(0.115, -0.018, 0.58)),
        material="aluminum",
        name="shoulder_ear_right",
    )
    body.visual(
        Box((0.028, 0.016, 0.05)),
        origin=Origin(xyz=(0.124, 0.041, 0.575)),
        material="datum_yellow",
        name="body_datum_pad",
    )
    body.visual(
        Box((0.020, 0.014, 0.050)),
        origin=Origin(xyz=(0.114, 0.034, 0.575)),
        material="housing_mid",
        name="body_datum_support",
    )
    body.visual(
        Box((0.025, 0.10, 0.04)),
        origin=Origin(xyz=(0.128, 0.0, 0.05)),
        material="elastomer",
        name="front_bumper",
    )
    _add_cylinder_y(
        body,
        "wheel_left",
        radius=0.047,
        length=0.022,
        center=(-0.125, 0.108, 0.047),
        material="elastomer",
    )
    _add_cylinder_y(
        body,
        "wheel_right",
        radius=0.047,
        length=0.022,
        center=(-0.125, -0.108, 0.047),
        material="elastomer",
    )

    proximal_wand = model.part("proximal_wand")
    _add_cylinder_y(
        proximal_wand,
        "shoulder_barrel",
        radius=0.016,
        length=SHOULDER_GAP,
        center=(0.0, 0.0, 0.0),
        material="aluminum",
    )
    proximal_wand.visual(
        Box((0.034, 0.022, 0.044)),
        origin=Origin(xyz=(0.032, 0.0, 0.0)),
        material="housing_mid",
        name="shoulder_housing",
    )
    _add_cylinder_x(
        proximal_wand,
        "proximal_tube",
        radius=0.015,
        length=PROXIMAL_TUBE_LENGTH,
        center=(PROXIMAL_TUBE_START + PROXIMAL_TUBE_LENGTH / 2.0, 0.0, 0.0),
        material="aluminum",
    )
    _add_cylinder_x(
        proximal_wand,
        "calibration_collar",
        radius=0.0185,
        length=0.045,
        center=(0.095, 0.0, 0.0),
        material="housing_dark",
    )
    proximal_wand.visual(
        Box((0.18, 0.006, 0.004)),
        origin=Origin(xyz=(0.16, 0.0, -0.013)),
        material="datum_yellow",
        name="wand_datum_flat",
    )
    proximal_wand.visual(
        Box((0.022, 0.003, 0.010)),
        origin=Origin(xyz=(0.016, 0.0125, 0.018)),
        material="datum_orange",
        name="shoulder_index",
    )
    proximal_wand.visual(
        Box((0.022, 0.028, 0.046)),
        origin=Origin(xyz=(0.301, 0.0, 0.0)),
        material="housing_mid",
        name="elbow_block",
    )
    proximal_wand.visual(
        Box((0.050, 0.036, 0.006)),
        origin=Origin(xyz=(0.315, 0.0, 0.020)),
        material="housing_mid",
        name="elbow_bridge_top",
    )
    proximal_wand.visual(
        Box((0.050, 0.036, 0.006)),
        origin=Origin(xyz=(0.315, 0.0, -0.020)),
        material="housing_mid",
        name="elbow_bridge_bottom",
    )
    proximal_wand.visual(
        Box((0.018, 0.008, 0.046)),
        origin=Origin(xyz=(PROXIMAL_WAND_LENGTH, 0.018, 0.0)),
        material="aluminum",
        name="elbow_ear_left",
    )
    proximal_wand.visual(
        Box((0.018, 0.008, 0.046)),
        origin=Origin(xyz=(PROXIMAL_WAND_LENGTH, -0.018, 0.0)),
        material="aluminum",
        name="elbow_ear_right",
    )
    proximal_wand.visual(
        Box((0.020, 0.003, 0.012)),
        origin=Origin(xyz=(0.304, -0.0125, 0.018)),
        material="datum_orange",
        name="elbow_scale",
    )

    distal_wand = model.part("distal_wand")
    _add_cylinder_y(
        distal_wand,
        "elbow_barrel",
        radius=0.015,
        length=ELBOW_GAP,
        center=(0.0, 0.0, 0.0),
        material="aluminum",
    )
    distal_wand.visual(
        Box((0.034, 0.022, 0.042)),
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
        material="housing_mid",
        name="elbow_housing",
    )
    _add_cylinder_x(
        distal_wand,
        "distal_tube",
        radius=0.014,
        length=DISTAL_TUBE_LENGTH,
        center=(DISTAL_TUBE_START + DISTAL_TUBE_LENGTH / 2.0, 0.0, 0.0),
        material="aluminum",
    )
    _add_cylinder_x(
        distal_wand,
        "adjustment_collar",
        radius=0.0175,
        length=0.040,
        center=(0.10, 0.0, 0.0),
        material="housing_dark",
    )
    distal_wand.visual(
        Box((0.16, 0.006, 0.004)),
        origin=Origin(xyz=(0.15, 0.0, -0.012)),
        material="datum_yellow",
        name="distal_datum_flat",
    )
    distal_wand.visual(
        Box((0.016, 0.003, 0.010)),
        origin=Origin(xyz=(0.092, 0.011, 0.016)),
        material="datum_orange",
        name="distal_index",
    )
    distal_wand.visual(
        Box((0.028, 0.024, 0.040)),
        origin=Origin(xyz=(0.274, 0.0, 0.0)),
        material="housing_mid",
        name="nozzle_block",
    )
    distal_wand.visual(
        Box((0.026, 0.032, 0.006)),
        origin=Origin(xyz=(0.286, 0.0, 0.017)),
        material="housing_mid",
        name="nozzle_bridge_top",
    )
    distal_wand.visual(
        Box((0.026, 0.032, 0.006)),
        origin=Origin(xyz=(0.286, 0.0, -0.017)),
        material="housing_mid",
        name="nozzle_bridge_bottom",
    )
    distal_wand.visual(
        Box((0.018, 0.008, 0.040)),
        origin=Origin(xyz=(0.300, 0.016, 0.0)),
        material="aluminum",
        name="nozzle_ear_left",
    )
    distal_wand.visual(
        Box((0.018, 0.008, 0.040)),
        origin=Origin(xyz=(0.300, -0.016, 0.0)),
        material="aluminum",
        name="nozzle_ear_right",
    )

    floor_nozzle = model.part("floor_nozzle")
    _add_cylinder_y(
        floor_nozzle,
        "pivot_barrel",
        radius=0.015,
        length=NOZZLE_GAP,
        center=(0.0, 0.0, 0.0),
        material="aluminum",
    )
    floor_nozzle.visual(
        Box((0.028, 0.018, 0.036)),
        origin=Origin(xyz=(0.030, 0.0, -0.010)),
        material="housing_mid",
        name="pivot_housing",
    )
    floor_nozzle.visual(
        Box((0.086, 0.036, 0.118)),
        origin=Origin(xyz=(0.028, 0.0, -0.068)),
        material="housing_mid",
        name="neck",
    )
    floor_nozzle.visual(
        Box((0.050, 0.20, 0.022)),
        origin=Origin(xyz=(-0.008, 0.0, -0.113)),
        material="housing_dark",
        name="rear_shroud",
    )
    floor_nozzle.visual(
        Box((0.110, 0.34, 0.028)),
        origin=Origin(xyz=(0.045, 0.0, -0.128)),
        material="housing_dark",
        name="head_shell",
    )
    floor_nozzle.visual(
        Box((0.095, 0.022, 0.010)),
        origin=Origin(xyz=(0.045, 0.118, -0.140)),
        material="datum_yellow",
        name="datum_skid_left",
    )
    floor_nozzle.visual(
        Box((0.095, 0.022, 0.010)),
        origin=Origin(xyz=(0.045, -0.118, -0.140)),
        material="datum_yellow",
        name="datum_skid_right",
    )
    floor_nozzle.visual(
        Box((0.016, 0.300, 0.012)),
        origin=Origin(xyz=(0.095, 0.0, -0.139)),
        material="datum_orange",
        name="front_datum_rail",
    )
    _add_cylinder_y(
        floor_nozzle,
        "height_adjust_knob",
        radius=0.010,
        length=0.018,
        center=(0.0, -0.030, -0.035),
        material="datum_orange",
    )
    floor_nozzle.visual(
        Box((0.008, 0.016, 0.008)),
        origin=Origin(xyz=(0.0, -0.015, -0.035)),
        material="housing_mid",
        name="height_adjust_stem",
    )
    floor_nozzle.visual(
        Box((0.014, 0.003, 0.010)),
        origin=Origin(xyz=(0.004, 0.0105, 0.018)),
        material="datum_orange",
        name="nozzle_index",
    )

    model.articulation(
        "body_to_proximal_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=proximal_wand,
        origin=Origin(xyz=(0.115, 0.0, 0.58), rpy=(0.0, SHOULDER_NEUTRAL_PITCH, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=-0.35,
            upper=0.55,
        ),
    )
    model.articulation(
        "proximal_to_distal_wand",
        ArticulationType.REVOLUTE,
        parent=proximal_wand,
        child=distal_wand,
        origin=Origin(xyz=(PROXIMAL_WAND_LENGTH, 0.0, 0.0), rpy=(0.0, ELBOW_NEUTRAL_PITCH, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.8,
            lower=-0.20,
            upper=0.75,
        ),
    )
    model.articulation(
        "distal_to_floor_nozzle",
        ArticulationType.REVOLUTE,
        parent=distal_wand,
        child=floor_nozzle,
        origin=Origin(xyz=(DISTAL_WAND_LENGTH, 0.0, 0.0), rpy=(0.0, NOZZLE_NEUTRAL_PITCH, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=-0.45,
            upper=0.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    proximal_wand = object_model.get_part("proximal_wand")
    distal_wand = object_model.get_part("distal_wand")
    floor_nozzle = object_model.get_part("floor_nozzle")

    shoulder = object_model.get_articulation("body_to_proximal_wand")
    elbow = object_model.get_articulation("proximal_to_distal_wand")
    nozzle_pitch = object_model.get_articulation("distal_to_floor_nozzle")

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

    ctx.expect_contact(
        body,
        proximal_wand,
        elem_b="shoulder_barrel",
        name="shoulder_pivot_is_physically_supported",
    )
    ctx.expect_contact(
        proximal_wand,
        distal_wand,
        elem_b="elbow_barrel",
        name="elbow_pivot_is_physically_supported",
    )
    ctx.expect_contact(
        distal_wand,
        floor_nozzle,
        elem_b="pivot_barrel",
        name="nozzle_pivot_is_physically_supported",
    )

    left_skid = ctx.part_element_world_aabb(floor_nozzle, elem="datum_skid_left")
    right_skid = ctx.part_element_world_aabb(floor_nozzle, elem="datum_skid_right")
    front_rail = ctx.part_element_world_aabb(floor_nozzle, elem="front_datum_rail")
    if left_skid is not None and right_skid is not None and front_rail is not None:
        datum_plane_error = max(
            abs(left_skid[0][2] - right_skid[0][2]),
            abs(left_skid[0][2] - front_rail[0][2]),
            abs(right_skid[0][2] - front_rail[0][2]),
        )
        ctx.check(
            "datum_surfaces_share_a_repeatable_plane",
            datum_plane_error <= 0.004,
            details=f"datum plane mismatch {datum_plane_error:.4f} m",
        )
        lowest_datum = min(left_skid[0][2], right_skid[0][2], front_rail[0][2])
        body_aabb = ctx.part_world_aabb(body)
        proximal_aabb = ctx.part_world_aabb(proximal_wand)
        distal_aabb = ctx.part_world_aabb(distal_wand)
        nozzle_aabb = ctx.part_world_aabb(floor_nozzle)
        model_lowest_z = min(
            aabb[0][2]
            for aabb in (body_aabb, proximal_aabb, distal_aabb, nozzle_aabb)
            if aabb is not None
        )
        ctx.check(
            "datum_plane_defines_the_lowest_running_reference",
            abs(lowest_datum - model_lowest_z) <= 0.006,
            details=f"lowest datum z={lowest_datum:.4f} m, model lowest z={model_lowest_z:.4f} m",
        )
    else:
        ctx.fail("datum_features_resolve", "one or more floor nozzle datum features are missing")

    rest_nozzle_position = ctx.part_world_position(floor_nozzle)
    with ctx.pose({shoulder: shoulder.motion_limits.upper}):
        shoulder_lift_position = ctx.part_world_position(floor_nozzle)
    if rest_nozzle_position is not None and shoulder_lift_position is not None:
        ctx.check(
            "shoulder_positive_motion_lifts_the_wand_stack",
            shoulder_lift_position[2] > rest_nozzle_position[2] + 0.08,
            details=(
                f"rest z={rest_nozzle_position[2]:.4f} m, "
                f"lifted z={shoulder_lift_position[2]:.4f} m"
            ),
        )
    else:
        ctx.fail("shoulder_motion_probe", "could not resolve floor nozzle position")

    with ctx.pose({elbow: elbow.motion_limits.upper}):
        elbow_lift_position = ctx.part_world_position(floor_nozzle)
    if rest_nozzle_position is not None and elbow_lift_position is not None:
        ctx.check(
            "elbow_positive_motion_lifts_the_floor_nozzle",
            elbow_lift_position[2] > rest_nozzle_position[2] + 0.04,
            details=(
                f"rest z={rest_nozzle_position[2]:.4f} m, "
                f"lifted z={elbow_lift_position[2]:.4f} m"
            ),
        )
    else:
        ctx.fail("elbow_motion_probe", "could not resolve floor nozzle position")

    rest_front_rail = ctx.part_element_world_aabb(floor_nozzle, elem="front_datum_rail")
    with ctx.pose({nozzle_pitch: nozzle_pitch.motion_limits.upper}):
        pitched_front_rail = ctx.part_element_world_aabb(floor_nozzle, elem="front_datum_rail")
    if rest_front_rail is not None and pitched_front_rail is not None:
        ctx.check(
            "nozzle_pitch_lifts_the_front_datum_rail",
            pitched_front_rail[0][2] > rest_front_rail[0][2] + 0.015,
            details=(
                f"rest rail z={rest_front_rail[0][2]:.4f} m, "
                f"pitched rail z={pitched_front_rail[0][2]:.4f} m"
            ),
        )
    else:
        ctx.fail("nozzle_pitch_probe", "could not resolve front datum rail")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
