from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq
from math import pi

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


BASE_W = 0.112
BASE_D = 0.094
BASE_H = 0.014
MID_W = 0.078
MID_D = 0.066
MID_H = 0.018
HOUSING_R = 0.028
HOUSING_H = 0.022
HOUSING_Z0 = BASE_H + MID_H
TOP_COLLAR_OUTER_R = 0.025
YAW_BORE_R = 0.0205
TOP_COLLAR_H = 0.014
TOP_COLLAR_Z0 = HOUSING_Z0 + HOUSING_H
YAW_ORIGIN_Z = 0.068

PITCH_Y = 0.070
PITCH_Z = 0.044
PITCH_EAR_OUTER_R = 0.015
PITCH_EAR_BORE_R = 0.0095
PITCH_EAR_THICKNESS = 0.008
PITCH_EAR_CENTER_X = 0.039

ROLL_OFFSET_Y = 0.030
ROLL_BORE_R = 0.0186


def _box(
    sx: float,
    sy: float,
    sz: float,
    *,
    center: tuple[float, float, float] = (0.0, 0.0, 0.0),
    chamfer: float | None = None,
    fillet: float | None = None,
) -> cq.Workplane:
    shape = cq.Workplane("XY").box(sx, sy, sz)
    if chamfer is not None:
        shape = shape.edges("|Z").chamfer(chamfer)
    if fillet is not None:
        shape = shape.edges("|Z").fillet(fillet)
    return shape.translate(center)


def _centered_cylinder_z(radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(length).translate((0.0, 0.0, -length / 2.0))


def _cyl_z(radius: float, length: float, *, center: tuple[float, float, float]) -> cq.Workplane:
    return _centered_cylinder_z(radius, length).translate(center)


def _cyl_x(radius: float, length: float, *, center: tuple[float, float, float]) -> cq.Workplane:
    return _centered_cylinder_z(radius, length).rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0).translate(center)


def _cyl_y(radius: float, length: float, *, center: tuple[float, float, float]) -> cq.Workplane:
    return _centered_cylinder_z(radius, length).rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0).translate(center)


def _ring_z(
    outer_radius: float,
    inner_radius: float,
    length: float,
    *,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return _cyl_z(outer_radius, length, center=center).cut(
        _cyl_z(inner_radius, length + 0.002, center=center)
    )


def _ring_x(
    outer_radius: float,
    inner_radius: float,
    length: float,
    *,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return _cyl_x(outer_radius, length, center=center).cut(
        _cyl_x(inner_radius, length + 0.002, center=center)
    )


def _ring_y(
    outer_radius: float,
    inner_radius: float,
    length: float,
    *,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return _cyl_y(outer_radius, length, center=center).cut(
        _cyl_y(inner_radius, length + 0.002, center=center)
    )


def _build_pedestal_shape() -> cq.Workplane:
    base = _box(
        BASE_W,
        BASE_D,
        BASE_H,
        center=(0.0, 0.0, BASE_H / 2.0),
        chamfer=0.0035,
    )
    mid = _box(
        MID_W,
        MID_D,
        MID_H,
        center=(0.0, 0.0, BASE_H + MID_H / 2.0),
        chamfer=0.0025,
    )
    housing = _cyl_z(
        HOUSING_R,
        HOUSING_H,
        center=(0.0, 0.0, HOUSING_Z0 + HOUSING_H / 2.0),
    ).intersect(
        _box(
            0.056,
            0.068,
            HOUSING_H,
            center=(0.0, 0.0, HOUSING_Z0 + HOUSING_H / 2.0),
        )
    )
    top_collar = _ring_z(
        TOP_COLLAR_OUTER_R,
        YAW_BORE_R,
        TOP_COLLAR_H,
        center=(0.0, 0.0, TOP_COLLAR_Z0 + TOP_COLLAR_H / 2.0),
    )
    body = base.union(mid).union(housing).union(top_collar)
    body = body.cut(_cyl_z(YAW_BORE_R, 0.024, center=(0.0, 0.0, 0.058)))
    body = body.cut(
        _box(
            0.070,
            0.020,
            0.010,
            center=(0.0, MID_D / 2.0 - 0.015, BASE_H + 0.006),
        )
    )
    body = body.cut(
        _box(
            0.070,
            0.020,
            0.010,
            center=(0.0, -MID_D / 2.0 + 0.015, BASE_H + 0.006),
        )
    )
    return body


def _build_yaw_carrier_shape() -> cq.Workplane:
    spindle = _cyl_z(YAW_BORE_R - 0.0008, 0.018, center=(0.0, 0.0, -0.009))
    thrust_plate = _ring_z(TOP_COLLAR_OUTER_R - 0.0008, YAW_BORE_R - 0.0017, 0.006, center=(0.0, 0.0, 0.003))
    neck = _box(0.026, 0.018, 0.022, center=(0.0, 0.010, 0.014), fillet=0.0015)
    beam = _box(0.024, 0.036, 0.016, center=(0.0, 0.030, 0.026), chamfer=0.0012)
    upper_bridge = _box(0.026, 0.014, 0.010, center=(0.0, 0.050, 0.040), chamfer=0.001)
    left_rib = _box(0.016, 0.032, 0.020, center=(-0.029, 0.044, 0.034), fillet=0.001)
    right_rib = _box(0.016, 0.032, 0.020, center=(0.029, 0.044, 0.034), fillet=0.001)
    left_ear = _ring_x(
        PITCH_EAR_OUTER_R,
        PITCH_EAR_BORE_R,
        PITCH_EAR_THICKNESS,
        center=(-PITCH_EAR_CENTER_X, PITCH_Y, PITCH_Z),
    )
    right_ear = _ring_x(
        PITCH_EAR_OUTER_R,
        PITCH_EAR_BORE_R,
        PITCH_EAR_THICKNESS,
        center=(PITCH_EAR_CENTER_X, PITCH_Y, PITCH_Z),
    )

    body = spindle.union(thrust_plate).union(neck).union(beam).union(upper_bridge)
    body = body.union(left_rib).union(right_rib).union(left_ear).union(right_ear)
    return body


def _build_pitch_frame_shape() -> cq.Workplane:
    left_cheek = _box(0.013, 0.060, 0.044, center=(-0.0285, 0.026, 0.0), fillet=0.0012)
    right_cheek = _box(0.013, 0.060, 0.044, center=(0.0285, 0.026, 0.0), fillet=0.0012)
    top_bridge = _box(0.046, 0.018, 0.006, center=(0.0, 0.032, 0.022), chamfer=0.0008)
    bottom_bridge = _box(0.046, 0.018, 0.006, center=(0.0, 0.032, -0.022), chamfer=0.0008)
    rear_ring = _ring_y(0.0226, 0.0187, 0.008, center=(0.0, 0.022, 0.0))
    front_ring = _ring_y(0.0226, 0.0187, 0.008, center=(0.0, 0.050, 0.0))
    left_rear_web = _box(0.010, 0.014, 0.018, center=(-0.018, 0.022, 0.0), fillet=0.0008)
    right_rear_web = _box(0.010, 0.014, 0.018, center=(0.018, 0.022, 0.0), fillet=0.0008)
    left_front_web = _box(0.010, 0.014, 0.018, center=(-0.018, 0.050, 0.0), fillet=0.0008)
    right_front_web = _box(0.010, 0.014, 0.018, center=(0.018, 0.050, 0.0), fillet=0.0008)
    trunnion_land = _cyl_x(PITCH_EAR_BORE_R, 0.086, center=(0.0, 0.0, 0.0))

    body = left_cheek.union(right_cheek).union(top_bridge).union(bottom_bridge)
    body = body.union(rear_ring).union(front_ring)
    body = body.union(left_rear_web).union(right_rear_web).union(left_front_web).union(right_front_web)
    body = body.union(trunnion_land)
    return body


def _build_roll_nose_shape() -> cq.Workplane:
    spindle = _cyl_y(0.0184, 0.028, center=(0.0, 0.032, 0.0))
    rear_collar = _cyl_y(0.0206, 0.008, center=(0.0, 0.014, 0.0))
    front_collar = _cyl_y(0.0206, 0.008, center=(0.0, 0.042, 0.0))
    cartridge = _cyl_y(0.0188, 0.028, center=(0.0, 0.060, 0.0))
    front_land = _cyl_y(0.0155, 0.012, center=(0.0, 0.080, 0.0))
    nose_cap = _cyl_y(0.0125, 0.016, center=(0.0, 0.094, 0.0))
    return spindle.union(rear_collar).union(front_collar).union(cartridge).union(front_land).union(nose_cap)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_robotic_wrist")

    model.material("machined_base", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("carrier_gray", rgba=(0.32, 0.34, 0.37, 1.0))
    model.material("frame_silver", rgba=(0.63, 0.66, 0.70, 1.0))
    model.material("nose_steel", rgba=(0.16, 0.18, 0.20, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((BASE_W, BASE_D, BASE_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_H / 2.0)),
        material="machined_base",
        name="base_foot",
    )
    pedestal.visual(
        Box((MID_W, MID_D, MID_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_H + MID_H / 2.0)),
        material="machined_base",
        name="mid_stage",
    )
    pedestal.visual(
        Box((0.050, 0.054, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, BASE_H + MID_H + 0.007)),
        material="machined_base",
        name="head_block",
    )
    pedestal.visual(
        Box((0.024, 0.024, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material="machined_base",
        name="bearing_pedestal",
    )
    pedestal.visual(
        Cylinder(radius=TOP_COLLAR_OUTER_R, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, YAW_ORIGIN_Z - 0.005)),
        material="machined_base",
        name="yaw_bearing_land",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((BASE_W, BASE_D, TOP_COLLAR_Z0 + TOP_COLLAR_H)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, (TOP_COLLAR_Z0 + TOP_COLLAR_H) / 2.0)),
    )

    yaw_carrier = model.part("yaw_carrier")
    yaw_carrier.visual(
        Cylinder(radius=TOP_COLLAR_OUTER_R - 0.001, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material="carrier_gray",
        name="thrust_plate",
    )
    yaw_carrier.visual(
        Box((0.024, 0.020, 0.022)),
        origin=Origin(xyz=(0.0, 0.010, 0.017)),
        material="carrier_gray",
        name="neck",
    )
    yaw_carrier.visual(
        Box((0.046, 0.030, 0.014)),
        origin=Origin(xyz=(0.0, 0.030, 0.028)),
        material="carrier_gray",
        name="beam",
    )
    yaw_carrier.visual(
        Box((0.016, 0.022, 0.020)),
        origin=Origin(xyz=(-0.029, 0.046, 0.034)),
        material="carrier_gray",
        name="left_rib",
    )
    yaw_carrier.visual(
        Box((0.016, 0.022, 0.020)),
        origin=Origin(xyz=(0.029, 0.046, 0.034)),
        material="carrier_gray",
        name="right_rib",
    )
    yaw_carrier.visual(
        mesh_from_cadquery(
            _ring_x(
                PITCH_EAR_OUTER_R,
                PITCH_EAR_BORE_R,
                PITCH_EAR_THICKNESS,
                center=(-PITCH_EAR_CENTER_X, PITCH_Y, PITCH_Z),
            ),
            "yaw_carrier_left_ear",
        ),
        material="carrier_gray",
        name="left_ear",
    )
    yaw_carrier.visual(
        mesh_from_cadquery(
            _ring_x(
                PITCH_EAR_OUTER_R,
                PITCH_EAR_BORE_R,
                PITCH_EAR_THICKNESS,
                center=(PITCH_EAR_CENTER_X, PITCH_Y, PITCH_Z),
            ),
            "yaw_carrier_right_ear",
        ),
        material="carrier_gray",
        name="right_ear",
    )
    yaw_carrier.inertial = Inertial.from_geometry(
        Box((0.080, 0.070, 0.060)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.030, 0.028)),
    )

    pitch_frame = model.part("pitch_frame")
    pitch_frame.visual(
        mesh_from_cadquery(_build_pitch_frame_shape(), "pitch_frame_body"),
        material="frame_silver",
        name="body",
    )
    pitch_frame.inertial = Inertial.from_geometry(
        Box((0.100, 0.074, 0.064)),
        mass=0.42,
        origin=Origin(xyz=(0.0, ROLL_OFFSET_Y, 0.0)),
    )

    roll_nose = model.part("roll_nose")
    roll_nose.visual(
        mesh_from_cadquery(_build_roll_nose_shape(), "roll_nose_body"),
        material="nose_steel",
        name="body",
    )
    roll_nose.inertial = Inertial.from_geometry(
        Box((0.044, 0.130, 0.044)),
        mass=0.24,
        origin=Origin(xyz=(0.0, 0.070, 0.0)),
    )

    model.articulation(
        "pedestal_yaw",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=yaw_carrier,
        origin=Origin(xyz=(0.0, 0.0, YAW_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=48.0, velocity=2.8, lower=-2.6, upper=2.6),
    )
    model.articulation(
        "carrier_pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_carrier,
        child=pitch_frame,
        origin=Origin(xyz=(0.0, PITCH_Y, PITCH_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=22.0, velocity=2.4, lower=-1.05, upper=1.20),
    )
    model.articulation(
        "nose_roll",
        ArticulationType.REVOLUTE,
        parent=pitch_frame,
        child=roll_nose,
        origin=Origin(xyz=(0.0, ROLL_OFFSET_Y, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=3.5, lower=-3.0, upper=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    yaw_carrier = object_model.get_part("yaw_carrier")
    pitch_frame = object_model.get_part("pitch_frame")
    roll_nose = object_model.get_part("roll_nose")
    yaw = object_model.get_articulation("pedestal_yaw")
    pitch = object_model.get_articulation("carrier_pitch")
    roll = object_model.get_articulation("nose_roll")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        yaw_carrier,
        pitch_frame,
        reason="Captured trunnion bearing interface is modeled as a nested ear-and-shaft assembly within the pitch joint envelope.",
    )
    ctx.allow_overlap(
        pitch_frame,
        roll_nose,
        reason="Roll cartridge is intentionally nested inside the pitch-frame bearing cradle and represented with slight bearing-surface interpenetration.",
    )

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

    ctx.expect_contact(yaw_carrier, pedestal, name="yaw_carrier_is_grounded_on_pedestal")
    ctx.expect_contact(pitch_frame, yaw_carrier, name="pitch_frame_is_grounded_on_trunnion_supports")
    ctx.expect_contact(roll_nose, pitch_frame, name="roll_nose_is_grounded_in_frame_cartridge")

    ctx.check(
        "yaw_axis_is_vertical",
        tuple(round(v, 3) for v in yaw.axis) == (0.0, 0.0, 1.0),
        details=f"unexpected yaw axis: {yaw.axis}",
    )
    ctx.check(
        "pitch_axis_is_lateral",
        tuple(round(v, 3) for v in pitch.axis) == (1.0, 0.0, 0.0),
        details=f"unexpected pitch axis: {pitch.axis}",
    )
    ctx.check(
        "roll_axis_is_forward",
        tuple(round(v, 3) for v in roll.axis) == (0.0, 1.0, 0.0),
        details=f"unexpected roll axis: {roll.axis}",
    )

    with ctx.pose({pitch: -0.90}):
        ctx.expect_gap(
            pitch_frame,
            pedestal,
            axis="y",
            min_gap=0.003,
            name="pitched_frame_clears_pedestal_front",
        )

    with ctx.pose({yaw: 0.80, pitch: 1.00, roll: 1.60}):
        ctx.fail_if_parts_overlap_in_current_pose(name="articulated_pose_stays_clear")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
