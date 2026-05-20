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


RAIL_LENGTH = 1.40
RAIL_SPACING = 0.82

FOOT_HEIGHT = 0.020
RAIL_BASE_WIDTH = 0.140
RAIL_BASE_HEIGHT = 0.055
RAIL_CAP_WIDTH = 0.090
RAIL_CAP_HEIGHT = 0.020

RAIL_TOP_Z = FOOT_HEIGHT + RAIL_BASE_HEIGHT + RAIL_CAP_HEIGHT
TRACK_HEIGHT = 0.008
TRACK_SINK = 0.001
TRACK_TOP_Z = RAIL_TOP_Z + TRACK_HEIGHT - TRACK_SINK

TRACK_LENGTH = 1.26
TRACK_WIDTH = 0.060

FOOT_PAD_SIZE = (0.22, 0.18, FOOT_HEIGHT)
FOOT_XS = (-0.58, 0.58)

CROSS_TIE_SIZE = (0.11, RAIL_SPACING + 0.18, 0.040)
CROSS_TIE_XS = (-0.58, 0.0, 0.58)

BRIDGE_FRAME_Z = 0.290
BRIDGE_TRAVEL = 0.44

BRIDGE_SHOE_SIZE = (0.22, 0.11, 0.050)
BRIDGE_UPPER_BLOCK_SIZE = (0.18, 0.16, 0.030)
BRIDGE_BEAM_OUTER = (0.12, 0.72, 0.14)
BRIDGE_BEAM_INNER = (0.08, 0.68, 0.10)
BRIDGE_GUIDE_SIZE = (0.020, 0.58, 0.018)
BRIDGE_GUIDE_X = 0.040

TRUCK_FRAME_Z = 0.430
TRUCK_TRAVEL = 0.22

TRUCK_BODY_SIZE = (0.18, 0.14, 0.070)
TRUCK_TOP_CAP_SIZE = (0.12, 0.14, 0.020)
TRUCK_PAD_SIZE = (0.030, 0.12, 0.016)
TRUCK_FACE_PLATE_SIZE = (0.012, 0.10, 0.11)


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _combine(*solids: cq.Workplane) -> cq.Workplane:
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid)
    return result


def _bridge_local_z(world_z: float) -> float:
    return world_z - BRIDGE_FRAME_Z


def _truck_local_z(world_z: float) -> float:
    return world_z - TRUCK_FRAME_Z


def _make_base_body() -> cq.Workplane:
    rail_y = RAIL_SPACING / 2.0
    rail_base_z = FOOT_HEIGHT + RAIL_BASE_HEIGHT / 2.0
    rail_cap_z = FOOT_HEIGHT + RAIL_BASE_HEIGHT + RAIL_CAP_HEIGHT / 2.0

    solids = [
        _box((RAIL_LENGTH, RAIL_BASE_WIDTH, RAIL_BASE_HEIGHT), (0.0, -rail_y, rail_base_z)),
        _box((RAIL_LENGTH, RAIL_BASE_WIDTH, RAIL_BASE_HEIGHT), (0.0, rail_y, rail_base_z)),
        _box((RAIL_LENGTH, RAIL_CAP_WIDTH, RAIL_CAP_HEIGHT), (0.0, -rail_y, rail_cap_z)),
        _box((RAIL_LENGTH, RAIL_CAP_WIDTH, RAIL_CAP_HEIGHT), (0.0, rail_y, rail_cap_z)),
    ]

    for rail_sign in (-1.0, 1.0):
        for foot_x in FOOT_XS:
            solids.append(
                _box(
                    FOOT_PAD_SIZE,
                    (foot_x, rail_sign * rail_y, FOOT_PAD_SIZE[2] / 2.0),
                )
            )

    for tie_x in CROSS_TIE_XS:
        solids.append(_box(CROSS_TIE_SIZE, (tie_x, 0.0, CROSS_TIE_SIZE[2] / 2.0)))

    return _combine(*solids)


def _make_base_track(y_sign: float) -> cq.Workplane:
    rail_y = y_sign * RAIL_SPACING / 2.0
    center_z = RAIL_TOP_Z - TRACK_SINK + TRACK_HEIGHT / 2.0
    return _box((TRACK_LENGTH, TRACK_WIDTH, TRACK_HEIGHT), (0.0, rail_y, center_z))


def _make_bridge_support(y_sign: float) -> cq.Workplane:
    rail_y = y_sign * RAIL_SPACING / 2.0
    shoe_center_z = _bridge_local_z(TRACK_TOP_Z + BRIDGE_SHOE_SIZE[2] / 2.0)
    upper_center_z = _bridge_local_z(
        TRACK_TOP_Z + BRIDGE_SHOE_SIZE[2] + BRIDGE_UPPER_BLOCK_SIZE[2] / 2.0 - 0.002
    )

    shoe = _box(BRIDGE_SHOE_SIZE, (0.0, rail_y, shoe_center_z))
    upper = _box(BRIDGE_UPPER_BLOCK_SIZE, (0.0, rail_y, upper_center_z))

    plate = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.10, _bridge_local_z(TRACK_TOP_Z + 0.050)),
                (-0.10, _bridge_local_z(0.200)),
                (-0.065, _bridge_local_z(0.245)),
                (-0.065, _bridge_local_z(0.345)),
                (0.065, _bridge_local_z(0.345)),
                (0.065, _bridge_local_z(0.245)),
                (0.10, _bridge_local_z(0.200)),
                (0.10, _bridge_local_z(TRACK_TOP_Z + 0.050)),
            ]
        )
        .close()
        .extrude(0.040, both=True)
        .translate((0.0, y_sign * 0.365, 0.0))
    )

    return _combine(shoe, upper, plate)


def _make_bridge_beam() -> cq.Workplane:
    outer = _box(BRIDGE_BEAM_OUTER, (0.0, 0.0, 0.0))
    inner = _box(BRIDGE_BEAM_INNER, (0.0, 0.0, 0.0))
    return outer.cut(inner)


def _make_bridge_guide(x_sign: float) -> cq.Workplane:
    center_z = _bridge_local_z(0.360 - 0.002 + BRIDGE_GUIDE_SIZE[2] / 2.0)
    return _box(BRIDGE_GUIDE_SIZE, (x_sign * BRIDGE_GUIDE_X, 0.0, center_z))


def _make_truck_body() -> cq.Workplane:
    body = _box(TRUCK_BODY_SIZE, (0.0, 0.0, 0.0))
    top_cap = _box(TRUCK_TOP_CAP_SIZE, (0.0, 0.0, TRUCK_BODY_SIZE[2] / 2.0))
    rear_rib = _box((0.050, 0.14, 0.050), (-0.040, 0.0, -0.010))
    pad_connector_z = _truck_local_z(
        _bridge_local_z(0.0) + BRIDGE_FRAME_Z + 0.376 + TRUCK_PAD_SIZE[2] / 2.0 + 0.005
    )
    left_connector = _box((0.024, 0.080, 0.022), (-BRIDGE_GUIDE_X, 0.0, pad_connector_z))
    right_connector = _box((0.024, 0.080, 0.022), (BRIDGE_GUIDE_X, 0.0, pad_connector_z))
    return _combine(body, top_cap, rear_rib, left_connector, right_connector)


def _make_truck_pad(x_sign: float) -> cq.Workplane:
    center_z = _truck_local_z(_bridge_local_z(0.0) + BRIDGE_FRAME_Z + 0.376 + TRUCK_PAD_SIZE[2] / 2.0)
    return _box(TRUCK_PAD_SIZE, (x_sign * BRIDGE_GUIDE_X, 0.0, center_z))


def _make_truck_face_plate() -> cq.Workplane:
    x_center = TRUCK_BODY_SIZE[0] / 2.0 - TRUCK_FACE_PLATE_SIZE[0] / 2.0 + 0.003
    return _box(TRUCK_FACE_PLATE_SIZE, (x_center, 0.0, -0.015))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_rider_dual_rail_gantry")

    model.material("frame_graphite", rgba=(0.21, 0.23, 0.26, 1.0))
    model.material("rail_steel", rgba=(0.63, 0.66, 0.70, 1.0))
    model.material("bridge_aluminum", rgba=(0.82, 0.84, 0.86, 1.0))
    model.material("truck_graphite", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("mount_aluminum", rgba=(0.76, 0.78, 0.81, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_body(), "gantry_base_body"),
        material="frame_graphite",
        name="base_body",
    )
    base.visual(
        mesh_from_cadquery(_make_base_track(-1.0), "gantry_left_track"),
        material="rail_steel",
        name="left_track",
    )
    base.visual(
        mesh_from_cadquery(_make_base_track(1.0), "gantry_right_track"),
        material="rail_steel",
        name="right_track",
    )
    base.inertial = Inertial.from_geometry(
        Box((RAIL_LENGTH, RAIL_SPACING + 0.20, TRACK_TOP_Z)),
        mass=46.0,
        origin=Origin(xyz=(0.0, 0.0, TRACK_TOP_Z / 2.0)),
    )

    bridge = model.part("bridge")
    bridge.visual(
        mesh_from_cadquery(_make_bridge_support(-1.0), "gantry_left_bridge_support"),
        material="bridge_aluminum",
        name="left_support",
    )
    bridge.visual(
        mesh_from_cadquery(_make_bridge_support(1.0), "gantry_right_bridge_support"),
        material="bridge_aluminum",
        name="right_support",
    )
    bridge.visual(
        mesh_from_cadquery(_make_bridge_beam(), "gantry_bridge_beam"),
        material="bridge_aluminum",
        name="beam_body",
    )
    bridge.visual(
        mesh_from_cadquery(_make_bridge_guide(-1.0), "gantry_left_bridge_guide"),
        material="rail_steel",
        name="left_guide",
    )
    bridge.visual(
        mesh_from_cadquery(_make_bridge_guide(1.0), "gantry_right_bridge_guide"),
        material="rail_steel",
        name="right_guide",
    )
    bridge.inertial = Inertial.from_geometry(
        Box((0.22, 0.98, 0.274)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, -0.051)),
    )

    truck = model.part("truck")
    truck.visual(
        mesh_from_cadquery(_make_truck_body(), "gantry_truck_body"),
        material="truck_graphite",
        name="truck_body",
    )
    truck.visual(
        mesh_from_cadquery(_make_truck_pad(-1.0), "gantry_left_truck_pad"),
        material="truck_graphite",
        name="left_pad",
    )
    truck.visual(
        mesh_from_cadquery(_make_truck_pad(1.0), "gantry_right_truck_pad"),
        material="truck_graphite",
        name="right_pad",
    )
    truck.visual(
        mesh_from_cadquery(_make_truck_face_plate(), "gantry_truck_face_plate"),
        material="mount_aluminum",
        name="mount_plate",
    )
    truck.inertial = Inertial.from_geometry(
        Box((0.19, 0.14, 0.11)),
        mass=4.2,
        origin=Origin(),
    )

    model.articulation(
        "base_to_bridge",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bridge,
        origin=Origin(xyz=(0.0, 0.0, BRIDGE_FRAME_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.70,
            lower=-BRIDGE_TRAVEL,
            upper=BRIDGE_TRAVEL,
        ),
    )
    model.articulation(
        "bridge_to_truck",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=truck,
        origin=Origin(xyz=(0.0, 0.0, TRUCK_FRAME_Z - BRIDGE_FRAME_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=280.0,
            velocity=0.55,
            lower=-TRUCK_TRAVEL,
            upper=TRUCK_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    bridge = object_model.get_part("bridge")
    truck = object_model.get_part("truck")

    left_track = base.get_visual("left_track")
    right_track = base.get_visual("right_track")
    left_support = bridge.get_visual("left_support")
    right_support = bridge.get_visual("right_support")
    left_guide = bridge.get_visual("left_guide")
    right_guide = bridge.get_visual("right_guide")
    left_pad = truck.get_visual("left_pad")
    right_pad = truck.get_visual("right_pad")

    bridge_slide = object_model.get_articulation("base_to_bridge")
    truck_slide = object_model.get_articulation("bridge_to_truck")

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
        "stacked_prismatic_axes_are_perpendicular",
        bridge_slide.articulation_type == ArticulationType.PRISMATIC
        and truck_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(bridge_slide.axis) == (1.0, 0.0, 0.0)
        and tuple(truck_slide.axis) == (0.0, 1.0, 0.0),
        details=(
            f"bridge axis={bridge_slide.axis}, truck axis={truck_slide.axis}, "
            f"bridge type={bridge_slide.articulation_type}, truck type={truck_slide.articulation_type}"
        ),
    )

    with ctx.pose({bridge_slide: 0.0, truck_slide: 0.0}):
        ctx.expect_contact(
            bridge,
            base,
            elem_a=left_support,
            elem_b=left_track,
            name="left_bridge_support_contacts_left_rail",
        )
        ctx.expect_contact(
            bridge,
            base,
            elem_a=right_support,
            elem_b=right_track,
            name="right_bridge_support_contacts_right_rail",
        )
        ctx.expect_contact(
            truck,
            bridge,
            elem_a=left_pad,
            elem_b=left_guide,
            name="left_truck_pad_contacts_left_bridge_guide",
        )
        ctx.expect_contact(
            truck,
            bridge,
            elem_a=right_pad,
            elem_b=right_guide,
            name="right_truck_pad_contacts_right_bridge_guide",
        )

    with ctx.pose({bridge_slide: bridge_slide.motion_limits.upper, truck_slide: 0.0}):
        ctx.expect_contact(
            bridge,
            base,
            elem_a=left_support,
            elem_b=left_track,
            name="bridge_stays_supported_at_positive_x_limit_left",
        )
        ctx.expect_contact(
            bridge,
            base,
            elem_a=right_support,
            elem_b=right_track,
            name="bridge_stays_supported_at_positive_x_limit_right",
        )

    with ctx.pose({bridge_slide: 0.0, truck_slide: truck_slide.motion_limits.upper}):
        ctx.expect_contact(
            truck,
            bridge,
            elem_a=left_pad,
            elem_b=left_guide,
            name="truck_stays_supported_at_positive_y_limit_left",
        )
        ctx.expect_contact(
            truck,
            bridge,
            elem_a=right_pad,
            elem_b=right_guide,
            name="truck_stays_supported_at_positive_y_limit_right",
        )

    with ctx.pose({bridge_slide: 0.0, truck_slide: 0.0}):
        bridge_home = ctx.part_world_position(bridge)
        truck_home = ctx.part_world_position(truck)
    with ctx.pose({bridge_slide: bridge_slide.motion_limits.upper, truck_slide: 0.0}):
        bridge_far = ctx.part_world_position(bridge)
    with ctx.pose({bridge_slide: 0.0, truck_slide: truck_slide.motion_limits.upper}):
        truck_far = ctx.part_world_position(truck)

    bridge_moves_correctly = (
        bridge_home is not None
        and bridge_far is not None
        and bridge_far[0] > bridge_home[0] + 0.30
        and abs(bridge_far[1] - bridge_home[1]) < 1e-6
        and abs(bridge_far[2] - bridge_home[2]) < 1e-6
    )
    ctx.check(
        "bridge_positive_q_moves_along_world_x",
        bridge_moves_correctly,
        details=f"bridge_home={bridge_home}, bridge_far={bridge_far}",
    )

    truck_moves_correctly = (
        truck_home is not None
        and truck_far is not None
        and truck_far[1] > truck_home[1] + 0.15
        and abs(truck_far[0] - truck_home[0]) < 1e-6
        and abs(truck_far[2] - truck_home[2]) < 1e-6
    )
    ctx.check(
        "truck_positive_q_moves_along_world_y",
        truck_moves_correctly,
        details=f"truck_home={truck_home}, truck_far={truck_far}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
