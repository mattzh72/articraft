from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_W = 0.280
BASE_D = 0.190
BASE_H = 0.038
TOP_W = 0.225
TOP_D = 0.145
TOP_H = 0.022

PLATE_T = 0.018
PLATE_D = 0.108
PLATE_H = 0.125
PLATE_CENTER_X = 0.089
PLATE_BASE_Z = BASE_H + TOP_H
PIVOT_Z = 0.145

FOOT_H = 0.010
FOOT_W = PLATE_T + 0.030
FOOT_D = PLATE_D + 0.020

INNER_BOSS_LEN = 0.006
INNER_BOSS_R = 0.026
OUTER_BOSS_LEN = 0.014
OUTER_BOSS_R = 0.030
HOLE_R = 0.0128

RIB_T = 0.018
RIB_Y_OFFSET = 0.030
RIB_BASE_EXT = 0.028

CHEEK_T = 0.008
CHEEK_CENTER_X = 0.066
CHEEK_D = 0.090
CHEEK_H = 0.110
CHEEK_CENTER_Y = 0.006
CHEEK_CENTER_Z = -0.005
CHEEK_BOSS_R = 0.028

CHEEK_WINDOW_D = 0.040
CHEEK_WINDOW_H = 0.054
CHEEK_WINDOW_Y = 0.012
CHEEK_WINDOW_Z = -0.016

MOUNT_W = 0.126
MOUNT_T = 0.008
MOUNT_H = 0.094
MOUNT_CENTER_Y = 0.042
MOUNT_CENTER_Z = -0.004
MOUNT_WINDOW_W = 0.078
MOUNT_WINDOW_H = 0.052
MOUNT_HOLE_R = 0.0045

TOP_RAIL_W = 0.106
TOP_RAIL_D = 0.020
TOP_RAIL_H = 0.014
TOP_RAIL_Y = 0.016
TOP_RAIL_Z = 0.038

BOTTOM_RAIL_W = 0.106
BOTTOM_RAIL_D = 0.018
BOTTOM_RAIL_H = 0.012
BOTTOM_RAIL_Y = 0.006
BOTTOM_RAIL_Z = -0.046

REAR_TIE_W = 0.094
REAR_TIE_D = 0.012
REAR_TIE_H = 0.018
REAR_TIE_Y = -0.030
REAR_TIE_Z = 0.012

CENTER_SPINE_W = 0.018
CENTER_SPINE_D = 0.022
CENTER_SPINE_H = 0.074
CENTER_SPINE_Y = 0.018
CENTER_SPINE_Z = -0.006

SUPPORT_PLATE_INNER_X = PLATE_CENTER_X - (PLATE_T / 2.0)
SUPPORT_PLATE_OUTER_X = PLATE_CENTER_X + (PLATE_T / 2.0)
SUPPORT_INNER_FACE_X = SUPPORT_PLATE_INNER_X - INNER_BOSS_LEN
SUPPORT_OUTER_FACE_X = SUPPORT_PLATE_OUTER_X + OUTER_BOSS_LEN
CHEEK_OUTER_FACE_X = CHEEK_CENTER_X + (CHEEK_T / 2.0)
TRUNNION_CLEARANCE = 0.0008
TRUNNION_BOSS_R = 0.021
TRUNNION_BOSS_LEN = SUPPORT_INNER_FACE_X - CHEEK_OUTER_FACE_X - TRUNNION_CLEARANCE


def _box(size_x: float, size_y: float, size_z: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(size_x, size_y, size_z).translate(center)


def _x_cylinder(
    radius: float,
    length: float,
    x_start: float,
    *,
    y: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    return cq.Workplane("YZ", origin=(x_start, y, z)).circle(radius).extrude(length)


def _y_cylinder(
    radius: float,
    length: float,
    y_start: float,
    *,
    x: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    return cq.Workplane("XZ", origin=(x, y_start, z)).circle(radius).extrude(length)


def _x_ring(
    outer_radius: float,
    inner_radius: float,
    x0: float,
    x1: float,
    *,
    y: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    ring = _x_cylinder(outer_radius, x1 - x0, x0, y=y, z=z)
    if inner_radius > 0.0:
        extension = 0.002 if x1 >= x0 else -0.002
        ring = ring.cut(
            _x_cylinder(
                inner_radius,
                (x1 - x0) + (2.0 * extension),
                x0 - extension,
                y=y,
                z=z,
            )
        )
    return ring


def _fuse_all(shapes: list[cq.Workplane]) -> cq.Workplane:
    fused = shapes[0]
    for shape in shapes[1:]:
        fused = fused.union(shape)
    return fused


def _make_base_mass() -> cq.Workplane:
    base = _box(BASE_W, BASE_D, BASE_H, (0.0, 0.0, BASE_H / 2.0))
    top = _box(TOP_W, TOP_D, TOP_H, (0.0, 0.0, BASE_H + (TOP_H / 2.0)))

    front_relief = _box(
        TOP_W - 0.030,
        0.024,
        0.012,
        (0.0, (BASE_D / 2.0) - 0.022, BASE_H * 0.52),
    )
    rear_relief = _box(
        TOP_W - 0.030,
        0.024,
        0.012,
        (0.0, -(BASE_D / 2.0) + 0.022, BASE_H * 0.52),
    )

    return base.union(top).cut(front_relief).cut(rear_relief)


def _make_support_structure(side: float) -> cq.Workplane:
    center_x = side * PLATE_CENTER_X
    plate_inner_x = side * SUPPORT_PLATE_INNER_X
    plate_outer_x = side * SUPPORT_PLATE_OUTER_X
    rib_base_x = side * (SUPPORT_OUTER_FACE_X + RIB_BASE_EXT)

    foot = _box(
        FOOT_W,
        FOOT_D,
        FOOT_H,
        (center_x, 0.0, BASE_H + (FOOT_H / 2.0)),
    )
    plate = _box(
        PLATE_T,
        PLATE_D,
        PLATE_H,
        (center_x, 0.0, PLATE_BASE_Z + (PLATE_H / 2.0)),
    )
    bore = _x_cylinder(
        HOLE_R,
        plate_outer_x - plate_inner_x,
        plate_inner_x,
        z=PIVOT_Z,
    )

    shapes: list[cq.Workplane] = [foot, plate.cut(bore)]

    for y_center in (-RIB_Y_OFFSET, RIB_Y_OFFSET):
        rib = (
            cq.Workplane("XZ", origin=(0.0, y_center, 0.0))
            .polyline(
                [
                    (plate_outer_x, PLATE_BASE_Z),
                    (plate_outer_x, PIVOT_Z - 0.018),
                    (rib_base_x, PLATE_BASE_Z),
                ]
            )
            .close()
            .extrude(RIB_T, both=True)
        )
        shapes.append(rib)

    return _fuse_all(shapes)


def _make_support_bearing(side: float) -> cq.Workplane:
    plate_inner_x = side * SUPPORT_PLATE_INNER_X
    plate_outer_x = side * SUPPORT_PLATE_OUTER_X
    inner_face_x = side * SUPPORT_INNER_FACE_X
    outer_face_x = side * SUPPORT_OUTER_FACE_X

    inner_flange = _x_ring(
        INNER_BOSS_R,
        HOLE_R,
        plate_inner_x,
        inner_face_x,
        z=PIVOT_Z,
    )
    outer_cap = _x_ring(
        OUTER_BOSS_R,
        HOLE_R,
        plate_outer_x,
        outer_face_x,
        z=PIVOT_Z,
    )
    return inner_flange.union(outer_cap)


def _make_cheek(side: float) -> cq.Workplane:
    cheek = _box(
        CHEEK_T,
        CHEEK_D,
        CHEEK_H,
        (side * CHEEK_CENTER_X, CHEEK_CENTER_Y, CHEEK_CENTER_Z),
    )
    cheek_boss = _x_cylinder(
        CHEEK_BOSS_R,
        CHEEK_T,
        (side * CHEEK_CENTER_X) - (CHEEK_T / 2.0),
    )
    window = _box(
        CHEEK_T + 0.004,
        CHEEK_WINDOW_D,
        CHEEK_WINDOW_H,
        (side * CHEEK_CENTER_X, CHEEK_WINDOW_Y, CHEEK_WINDOW_Z),
    )

    return cheek.union(cheek_boss).cut(window)


def _make_frame_body() -> cq.Workplane:
    front_plate = _box(
        MOUNT_W,
        MOUNT_T,
        MOUNT_H,
        (0.0, MOUNT_CENTER_Y, MOUNT_CENTER_Z),
    ).cut(
        _box(
            MOUNT_WINDOW_W,
            MOUNT_T + 0.004,
            MOUNT_WINDOW_H,
            (0.0, MOUNT_CENTER_Y, MOUNT_CENTER_Z),
        )
    )

    for x_center in (-0.041, 0.041):
        for z_center in (-0.024, 0.024):
            front_plate = front_plate.cut(
                _y_cylinder(
                    MOUNT_HOLE_R,
                    MOUNT_T + 0.020,
                    MOUNT_CENTER_Y - ((MOUNT_T + 0.020) / 2.0),
                    x=x_center,
                    z=MOUNT_CENTER_Z + z_center,
                )
            )

    body_shapes = [
        _make_cheek(-1.0),
        _make_cheek(1.0),
        front_plate,
        _box(TOP_RAIL_W, TOP_RAIL_D, TOP_RAIL_H, (0.0, TOP_RAIL_Y, TOP_RAIL_Z)),
        _box(
            BOTTOM_RAIL_W,
            BOTTOM_RAIL_D,
            BOTTOM_RAIL_H,
            (0.0, BOTTOM_RAIL_Y, BOTTOM_RAIL_Z),
        ),
        _box(REAR_TIE_W, REAR_TIE_D, REAR_TIE_H, (0.0, REAR_TIE_Y, REAR_TIE_Z)),
        _box(
            CENTER_SPINE_W,
            CENTER_SPINE_D,
            CENTER_SPINE_H,
            (0.0, CENTER_SPINE_Y, CENTER_SPINE_Z),
        ),
    ]

    return _fuse_all(body_shapes)


def _make_trunnion(side: float) -> cq.Workplane:
    return _x_cylinder(
        TRUNNION_BOSS_R,
        side * TRUNNION_BOSS_LEN,
        side * CHEEK_OUTER_FACE_X,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_axis_tilt_module")

    model.material("pedestal_graphite", rgba=(0.20, 0.21, 0.23, 1.0))
    model.material("support_graphite", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("frame_alloy", rgba=(0.72, 0.74, 0.76, 1.0))
    model.material("trunnion_steel", rgba=(0.55, 0.58, 0.61, 1.0))

    base_mass = _make_base_mass()
    support_left = _make_support_structure(-1.0)
    support_right = _make_support_structure(1.0)
    bearing_left = _make_support_bearing(-1.0)
    bearing_right = _make_support_bearing(1.0)
    frame_body = _make_frame_body()
    trunnion_left = _make_trunnion(-1.0)
    trunnion_right = _make_trunnion(1.0)

    pedestal = model.part("pedestal")
    pedestal.visual(
        mesh_from_cadquery(base_mass, "pedestal_mass"),
        name="base_mass",
        material="pedestal_graphite",
    )
    pedestal.visual(
        mesh_from_cadquery(support_left, "support_left"),
        name="support_left",
        material="support_graphite",
    )
    pedestal.visual(
        mesh_from_cadquery(support_right, "support_right"),
        name="support_right",
        material="support_graphite",
    )
    pedestal.visual(
        mesh_from_cadquery(bearing_left, "support_bearing_left"),
        name="support_bearing_left",
        material="support_graphite",
    )
    pedestal.visual(
        mesh_from_cadquery(bearing_right, "support_bearing_right"),
        name="support_bearing_right",
        material="support_graphite",
    )
    pedestal.visual(
        mesh_from_cadquery(bearing_left.union(bearing_right), "support_carriage"),
        name="support_carriage",
        material="support_graphite",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((BASE_W, BASE_D, PIVOT_Z + 0.040)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, (PIVOT_Z + 0.040) / 2.0)),
    )

    payload = model.part("payload_frame")
    payload.visual(
        mesh_from_cadquery(frame_body, "payload_frame_body"),
        name="frame_body",
        material="frame_alloy",
    )
    payload.visual(
        mesh_from_cadquery(trunnion_left, "payload_trunnion_left"),
        name="trunnion_left",
        material="trunnion_steel",
    )
    payload.visual(
        mesh_from_cadquery(trunnion_right, "payload_trunnion_right"),
        name="trunnion_right",
        material="trunnion_steel",
    )
    payload.visual(
        mesh_from_cadquery(trunnion_left.union(trunnion_right), "payload_trunnions"),
        name="trunnions",
        material="trunnion_steel",
    )
    payload.inertial = Inertial.from_geometry(
        Box((0.150, 0.102, 0.120)),
        mass=2.1,
        origin=Origin(xyz=(0.0, 0.020, -0.004)),
    )

    model.articulation(
        "pedestal_to_payload",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=payload,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.2,
            lower=-0.55,
            upper=0.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    payload = object_model.get_part("payload_frame")
    tilt = object_model.get_articulation("pedestal_to_payload")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.allow_isolated_part(
        payload,
        reason="payload journal bosses are intentionally modeled with small bearing clearances to the side supports",
    )
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

    axis = tilt.axis
    ctx.check(
        "pitch_axis_runs_through_support_plates",
        axis is not None
        and abs(axis[0] - 1.0) < 1e-6
        and abs(axis[1]) < 1e-6
        and abs(axis[2]) < 1e-6,
        f"expected horizontal x-axis pitch joint, got axis={axis}",
    )

    ctx.expect_gap(
        pedestal,
        payload,
        axis="x",
        positive_elem="support_bearing_right",
        negative_elem="trunnion_right",
        min_gap=0.0004,
        max_gap=0.0015,
        name="right_trunnion_has_small_bearing_clearance",
    )
    ctx.expect_gap(
        payload,
        pedestal,
        axis="x",
        positive_elem="trunnion_left",
        negative_elem="support_bearing_left",
        min_gap=0.0004,
        max_gap=0.0015,
        name="left_trunnion_has_small_bearing_clearance",
    )
    ctx.expect_overlap(
        payload,
        pedestal,
        elem_a="trunnion_left",
        elem_b="support_bearing_left",
        axes="yz",
        min_overlap=0.040,
        name="left_trunnion_stays_registered_in_left_support_bore",
    )
    ctx.expect_overlap(
        payload,
        pedestal,
        elem_a="trunnion_right",
        elem_b="support_bearing_right",
        axes="yz",
        min_overlap=0.040,
        name="right_trunnion_stays_registered_in_right_support_bore",
    )
    ctx.expect_within(
        payload,
        pedestal,
        axes="x",
        inner_elem="trunnions",
        outer_elem="support_carriage",
        margin=0.040,
        name="payload_pivot_features_stay_between_support_faces",
    )
    ctx.expect_overlap(
        payload,
        pedestal,
        elem_a="trunnions",
        elem_b="support_carriage",
        axes="yz",
        min_overlap=0.040,
        name="trunnions_stay_registered_inside_support_boss_window",
    )

    with ctx.pose({tilt: tilt.motion_limits.lower}):
        ctx.expect_gap(
            payload,
            pedestal,
            axis="z",
            positive_elem="frame_body",
            negative_elem="base_mass",
            min_gap=0.006,
            name="frame_body_clears_pedestal_at_down_tilt",
        )

    ctx.fail_if_articulation_overlaps(
        max_pose_samples=24,
        name="payload_sweeps_clear_through_motion_range",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
