from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


COLUMN_CENTER_X = 0.22
COLUMN_WIDTH_X = 0.09
COLUMN_DEPTH_Y = 0.12
COLUMN_BASE_Z = 0.12
COLUMN_HEIGHT_Z = 1.08

GUIDE_STRIP_THICKNESS = 0.006
GUIDE_STRIP_WIDTH_X = 0.058
GUIDE_STRIP_BASE_Z = 0.32
GUIDE_STRIP_HEIGHT_Z = 0.79

BASE_FOOT_WIDTH_X = 0.19
BASE_FOOT_DEPTH_Y = 0.40
BASE_FOOT_HEIGHT_Z = 0.10
BASE_CROSS_WIDTH_X = 0.58
BASE_CROSS_DEPTH_Y = 0.08
BASE_CROSS_HEIGHT_Z = 0.08
BASE_CROSS_OFFSET_Y = 0.15

COLUMN_PLINTH_WIDTH_X = 0.14
COLUMN_PLINTH_DEPTH_Y = 0.16
COLUMN_PLINTH_HEIGHT_Z = 0.08

UPPER_BEAM_WIDTH_X = 0.62
UPPER_BEAM_DEPTH_Y = 0.18
UPPER_BEAM_HEIGHT_Z = 0.12
UPPER_BEAM_CENTER_Z = 1.28

BASE_WEB_WIDTH_X = 0.14
BASE_WEB_DEPTH_Y = 0.03
BASE_WEB_HEIGHT_Z = 0.18
BASE_WEB_CENTER_Z = 0.19
BASE_WEB_OFFSET_Y = 0.07

CARRIAGE_PLATE_WIDTH_X = 0.70
CARRIAGE_PLATE_DEPTH_Y = 0.045
CARRIAGE_PLATE_HEIGHT_Z = 0.36
CARRIAGE_PLATE_CENTER_Y = 0.09

CARRIAGE_REAR_BOX_WIDTH_X = 0.34
CARRIAGE_REAR_BOX_DEPTH_Y = 0.045
CARRIAGE_REAR_BOX_HEIGHT_Z = 0.28
CARRIAGE_REAR_BOX_CENTER_Y = -0.07

CARRIAGE_BRIDGE_WIDTH_X = 0.40
CARRIAGE_BRIDGE_DEPTH_Y = 0.10
CARRIAGE_BRIDGE_HEIGHT_Z = 0.035
CARRIAGE_BRIDGE_CENTER_Y = -0.01

CARRIAGE_SIDE_RIB_WIDTH_X = 0.035
CARRIAGE_SIDE_RIB_DEPTH_Y = 0.16
CARRIAGE_SIDE_RIB_HEIGHT_Z = 0.16
CARRIAGE_SIDE_RIB_CENTER_X = 0.155
CARRIAGE_SIDE_RIB_CENTER_Y = -0.015

CARRIAGE_CENTER_RIB_WIDTH_X = 0.045
CARRIAGE_CENTER_RIB_DEPTH_Y = 0.10
CARRIAGE_CENTER_RIB_HEIGHT_Z = 0.22
CARRIAGE_CENTER_RIB_CENTER_Y = 0.00

SHOE_OUTER_WIDTH_X = 0.17
SHOE_OUTER_DEPTH_Y = 0.22
SHOE_OUTER_HEIGHT_Z = 0.12
SHOE_CENTER_OFFSET_Z = 0.11

SHOE_CHANNEL_WIDTH_X = 0.104
SHOE_CHANNEL_DEPTH_Y = 0.136
SHOE_CHANNEL_HEIGHT_Z = 0.18

GUIDE_PAD_WIDTH_X = 0.060
FRONT_PAD_DEPTH_Y = 0.0025
REAR_KEEPER_DEPTH_Y = 0.0015
GUIDE_PAD_HEIGHT_Z = 0.10
GUIDE_PAD_CENTER_OFFSET_Y = 0.06725

STRIKER_WIDTH_X = 0.070
STRIKER_DEPTH_Y = 0.036
STRIKER_HEIGHT_Z = 0.022
STRIKER_CENTER_Y = 0.0845
STRIKER_CENTER_OFFSET_Z = 0.18

STOP_PAD_WIDTH_X = 0.070
STOP_PAD_DEPTH_Y = 0.018
STOP_PAD_HEIGHT_Z = 0.022
STOP_PAD_CENTER_Y = (COLUMN_DEPTH_Y / 2.0) + (STOP_PAD_DEPTH_Y / 2.0)
LOWER_STOP_CENTER_Z = 0.289
UPPER_STOP_CENTER_Z = 1.151

JOINT_ORIGIN_Z = 0.50
CARRIAGE_TRAVEL_Z = 0.44


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    cx, cy, cz = center
    return cq.Workplane("XY").box(sx, sy, sz).translate((cx, cy, cz))


def _fuse_all(solids: list[cq.Workplane]) -> cq.Workplane:
    fused = solids[0]
    for solid in solids[1:]:
        fused = fused.union(solid)
    return fused


def make_frame_body() -> cq.Workplane:
    solids: list[cq.Workplane] = [
        _box(
            (BASE_FOOT_WIDTH_X, BASE_FOOT_DEPTH_Y, BASE_FOOT_HEIGHT_Z),
            (-COLUMN_CENTER_X, 0.0, BASE_FOOT_HEIGHT_Z / 2.0),
        ),
        _box(
            (BASE_FOOT_WIDTH_X, BASE_FOOT_DEPTH_Y, BASE_FOOT_HEIGHT_Z),
            (COLUMN_CENTER_X, 0.0, BASE_FOOT_HEIGHT_Z / 2.0),
        ),
        _box(
            (BASE_CROSS_WIDTH_X, BASE_CROSS_DEPTH_Y, BASE_CROSS_HEIGHT_Z),
            (0.0, BASE_CROSS_OFFSET_Y, 0.08),
        ),
        _box(
            (BASE_CROSS_WIDTH_X, BASE_CROSS_DEPTH_Y, BASE_CROSS_HEIGHT_Z),
            (0.0, -BASE_CROSS_OFFSET_Y, 0.08),
        ),
        _box(
            (COLUMN_PLINTH_WIDTH_X, COLUMN_PLINTH_DEPTH_Y, COLUMN_PLINTH_HEIGHT_Z),
            (-COLUMN_CENTER_X, 0.0, BASE_FOOT_HEIGHT_Z + (COLUMN_PLINTH_HEIGHT_Z / 2.0)),
        ),
        _box(
            (COLUMN_PLINTH_WIDTH_X, COLUMN_PLINTH_DEPTH_Y, COLUMN_PLINTH_HEIGHT_Z),
            (COLUMN_CENTER_X, 0.0, BASE_FOOT_HEIGHT_Z + (COLUMN_PLINTH_HEIGHT_Z / 2.0)),
        ),
        _box(
            (COLUMN_WIDTH_X, COLUMN_DEPTH_Y, COLUMN_HEIGHT_Z),
            (-COLUMN_CENTER_X, 0.0, COLUMN_BASE_Z + (COLUMN_HEIGHT_Z / 2.0)),
        ),
        _box(
            (COLUMN_WIDTH_X, COLUMN_DEPTH_Y, COLUMN_HEIGHT_Z),
            (COLUMN_CENTER_X, 0.0, COLUMN_BASE_Z + (COLUMN_HEIGHT_Z / 2.0)),
        ),
        _box(
            (UPPER_BEAM_WIDTH_X, UPPER_BEAM_DEPTH_Y, UPPER_BEAM_HEIGHT_Z),
            (0.0, 0.0, UPPER_BEAM_CENTER_Z),
        ),
    ]

    for x_sign in (-1.0, 1.0):
        x_pos = x_sign * COLUMN_CENTER_X
        for y_sign in (-1.0, 1.0):
            solids.append(
                _box(
                    (BASE_WEB_WIDTH_X, BASE_WEB_DEPTH_Y, BASE_WEB_HEIGHT_Z),
                    (x_pos, y_sign * BASE_WEB_OFFSET_Y, BASE_WEB_CENTER_Z),
                )
            )

    return _fuse_all(solids)


def make_frame_guide_strips(y_sign: float) -> cq.Workplane:
    strips: list[cq.Workplane] = []
    strip_center_z = GUIDE_STRIP_BASE_Z + (GUIDE_STRIP_HEIGHT_Z / 2.0)
    for x_sign in (-1.0, 1.0):
        x_pos = x_sign * COLUMN_CENTER_X
        y_pos = y_sign * ((COLUMN_DEPTH_Y / 2.0) + (GUIDE_STRIP_THICKNESS / 2.0))
        strips.append(
            _box(
                (GUIDE_STRIP_WIDTH_X, GUIDE_STRIP_THICKNESS, GUIDE_STRIP_HEIGHT_Z),
                (x_pos, y_pos, strip_center_z),
            )
        )
    return _fuse_all(strips)


def make_frame_stop_pads(center_z: float) -> cq.Workplane:
    pads: list[cq.Workplane] = []
    for x_sign in (-1.0, 1.0):
        pads.append(
            _box(
                (STOP_PAD_WIDTH_X, STOP_PAD_DEPTH_Y, STOP_PAD_HEIGHT_Z),
                (x_sign * COLUMN_CENTER_X, STOP_PAD_CENTER_Y, center_z),
            )
        )
    return _fuse_all(pads)


def make_carriage_body() -> cq.Workplane:
    outer_solids: list[cq.Workplane] = [
        _box(
            (CARRIAGE_PLATE_WIDTH_X, CARRIAGE_PLATE_DEPTH_Y, CARRIAGE_PLATE_HEIGHT_Z),
            (0.0, CARRIAGE_PLATE_CENTER_Y, 0.0),
        ),
        _box(
            (CARRIAGE_REAR_BOX_WIDTH_X, CARRIAGE_REAR_BOX_DEPTH_Y, CARRIAGE_REAR_BOX_HEIGHT_Z),
            (0.0, CARRIAGE_REAR_BOX_CENTER_Y, 0.0),
        ),
        _box(
            (CARRIAGE_BRIDGE_WIDTH_X, CARRIAGE_BRIDGE_DEPTH_Y, CARRIAGE_BRIDGE_HEIGHT_Z),
            (0.0, CARRIAGE_BRIDGE_CENTER_Y, SHOE_CENTER_OFFSET_Z),
        ),
        _box(
            (CARRIAGE_BRIDGE_WIDTH_X, CARRIAGE_BRIDGE_DEPTH_Y, CARRIAGE_BRIDGE_HEIGHT_Z),
            (0.0, CARRIAGE_BRIDGE_CENTER_Y, -SHOE_CENTER_OFFSET_Z),
        ),
        _box(
            (CARRIAGE_CENTER_RIB_WIDTH_X, CARRIAGE_CENTER_RIB_DEPTH_Y, CARRIAGE_CENTER_RIB_HEIGHT_Z),
            (0.0, CARRIAGE_CENTER_RIB_CENTER_Y, 0.0),
        ),
    ]

    for x_sign in (-1.0, 1.0):
        x_pos = x_sign * COLUMN_CENTER_X
        outer_solids.append(
            _box(
                (CARRIAGE_SIDE_RIB_WIDTH_X, CARRIAGE_SIDE_RIB_DEPTH_Y, CARRIAGE_SIDE_RIB_HEIGHT_Z),
                (x_sign * CARRIAGE_SIDE_RIB_CENTER_X, CARRIAGE_SIDE_RIB_CENTER_Y, 0.0),
            )
        )
        for z_sign in (-1.0, 1.0):
            outer_solids.append(
                _box(
                    (SHOE_OUTER_WIDTH_X, SHOE_OUTER_DEPTH_Y, SHOE_OUTER_HEIGHT_Z),
                    (x_pos, 0.0, z_sign * SHOE_CENTER_OFFSET_Z),
                )
            )

    carriage = _fuse_all(outer_solids)

    for x_sign in (-1.0, 1.0):
        x_pos = x_sign * COLUMN_CENTER_X
        for z_sign in (-1.0, 1.0):
            carriage = carriage.cut(
                _box(
                    (SHOE_CHANNEL_WIDTH_X, SHOE_CHANNEL_DEPTH_Y, SHOE_CHANNEL_HEIGHT_Z),
                    (x_pos, 0.0, z_sign * SHOE_CENTER_OFFSET_Z),
                )
            )

    front_relief = _box((0.32, 0.014, 0.20), (0.0, 0.105, 0.0))
    return carriage.cut(front_relief)


def make_carriage_front_bearing_pads() -> cq.Workplane:
    pads: list[cq.Workplane] = []
    for x_sign in (-1.0, 1.0):
        x_pos = x_sign * COLUMN_CENTER_X
        for z_sign in (-1.0, 1.0):
            z_pos = z_sign * SHOE_CENTER_OFFSET_Z
            pads.append(
                _box(
                    (GUIDE_PAD_WIDTH_X, FRONT_PAD_DEPTH_Y, GUIDE_PAD_HEIGHT_Z),
                    (x_pos, GUIDE_PAD_CENTER_OFFSET_Y, z_pos),
                )
            )
    return _fuse_all(pads)


def make_carriage_rear_keepers() -> cq.Workplane:
    keepers: list[cq.Workplane] = []
    for x_sign in (-1.0, 1.0):
        x_pos = x_sign * COLUMN_CENTER_X
        for z_sign in (-1.0, 1.0):
            z_pos = z_sign * SHOE_CENTER_OFFSET_Z
            keepers.append(
                _box(
                    (GUIDE_PAD_WIDTH_X, REAR_KEEPER_DEPTH_Y, GUIDE_PAD_HEIGHT_Z),
                    (x_pos, -GUIDE_PAD_CENTER_OFFSET_Y, z_pos),
                )
            )
    return _fuse_all(keepers)


def make_carriage_strikers(center_z: float) -> cq.Workplane:
    strikers: list[cq.Workplane] = []
    for x_sign in (-1.0, 1.0):
        strikers.append(
            _box(
                (STRIKER_WIDTH_X, STRIKER_DEPTH_Y, STRIKER_HEIGHT_Z),
                (x_sign * COLUMN_CENTER_X, STRIKER_CENTER_Y, center_z),
            )
        )
    return _fuse_all(strikers)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_lift_carriage_module")

    frame_blue = model.material("frame_blue", rgba=(0.15, 0.23, 0.36, 1.0))
    carriage_gray = model.material("carriage_gray", rgba=(0.66, 0.68, 0.70, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.78, 0.80, 0.82, 1.0))
    liner_bronze = model.material("liner_bronze", rgba=(0.70, 0.56, 0.28, 1.0))
    stop_black = model.material("stop_black", rgba=(0.16, 0.16, 0.17, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(make_frame_body(), "frame_body"),
        material=frame_blue,
        name="frame_body",
    )
    frame.visual(
        mesh_from_cadquery(make_frame_guide_strips(1.0), "front_guides"),
        material=machined_steel,
        name="front_guides",
    )
    frame.visual(
        mesh_from_cadquery(make_frame_guide_strips(-1.0), "rear_guides"),
        material=machined_steel,
        name="rear_guides",
    )
    frame.visual(
        mesh_from_cadquery(make_frame_stop_pads(LOWER_STOP_CENTER_Z), "lower_stops"),
        material=stop_black,
        name="lower_stops",
    )
    frame.visual(
        mesh_from_cadquery(make_frame_stop_pads(UPPER_STOP_CENTER_Z), "upper_stops"),
        material=stop_black,
        name="upper_stops",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(make_carriage_body(), "carriage_body"),
        material=carriage_gray,
        name="carriage_body",
    )
    carriage.visual(
        mesh_from_cadquery(make_carriage_front_bearing_pads(), "front_bearing_pads"),
        material=liner_bronze,
        name="front_bearing_pads",
    )
    carriage.visual(
        mesh_from_cadquery(make_carriage_rear_keepers(), "rear_keepers"),
        material=liner_bronze,
        name="rear_keepers",
    )
    carriage.visual(
        mesh_from_cadquery(make_carriage_strikers(-STRIKER_CENTER_OFFSET_Z), "lower_strikers"),
        material=machined_steel,
        name="lower_strikers",
    )
    carriage.visual(
        mesh_from_cadquery(make_carriage_strikers(STRIKER_CENTER_OFFSET_Z), "upper_strikers"),
        material=machined_steel,
        name="upper_strikers",
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, JOINT_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4000.0,
            velocity=0.35,
            lower=0.0,
            upper=CARRIAGE_TRAVEL_Z,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("frame_to_carriage")
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

    limits = lift.motion_limits
    ctx.check(
        "prismatic_axis_is_vertical",
        tuple(lift.axis) == (0.0, 0.0, 1.0),
        details=f"Expected vertical axis, got {lift.axis!r}",
    )
    ctx.check(
        "prismatic_limits_match_design_travel",
        limits is not None
        and limits.lower == 0.0
        and limits.upper == CARRIAGE_TRAVEL_Z
        and lift.articulation_type == ArticulationType.PRISMATIC,
        details="Carriage should ride on one vertical prismatic joint with the authored travel range.",
    )

    with ctx.pose({lift: 0.0}):
        ctx.expect_gap(
            carriage,
            frame,
            axis="y",
            positive_elem="front_bearing_pads",
            negative_elem="front_guides",
            min_gap=0.0,
            max_gap=0.0002,
            name="lower_pose_front_bearing_faces_seat",
        )
        ctx.expect_contact(
            carriage,
            frame,
            elem_a="front_bearing_pads",
            elem_b="front_guides",
            contact_tol=0.0002,
            name="lower_pose_carriage_is_grounded_on_front_guides",
        )
        ctx.expect_gap(
            frame,
            carriage,
            axis="y",
            positive_elem="rear_guides",
            negative_elem="rear_keepers",
            min_gap=0.0003,
            max_gap=0.0012,
            name="lower_pose_rear_running_clearance",
        )
        ctx.expect_overlap(
            carriage,
            frame,
            elem_a="front_bearing_pads",
            elem_b="front_guides",
            axes="xz",
            min_overlap=0.05,
            name="lower_pose_front_guides_remain_registered",
        )
        ctx.expect_overlap(
            carriage,
            frame,
            elem_a="rear_keepers",
            elem_b="rear_guides",
            axes="xz",
            min_overlap=0.05,
            name="lower_pose_rear_guides_remain_registered",
        )
        ctx.expect_gap(
            carriage,
            frame,
            axis="z",
            positive_elem="lower_strikers",
            negative_elem="lower_stops",
            min_gap=0.006,
            max_gap=0.020,
            name="lower_pose_hard_stop_clearance",
        )
        ctx.expect_overlap(
            carriage,
            frame,
            elem_a="lower_strikers",
            elem_b="lower_stops",
            axes="xy",
            min_overlap=0.010,
            name="lower_stop_faces_stay_aligned",
        )

    with ctx.pose({lift: CARRIAGE_TRAVEL_Z}):
        ctx.expect_gap(
            carriage,
            frame,
            axis="y",
            positive_elem="front_bearing_pads",
            negative_elem="front_guides",
            min_gap=0.0,
            max_gap=0.0002,
            name="upper_pose_front_bearing_faces_seat",
        )
        ctx.expect_contact(
            carriage,
            frame,
            elem_a="front_bearing_pads",
            elem_b="front_guides",
            contact_tol=0.0002,
            name="upper_pose_carriage_remains_grounded_on_front_guides",
        )
        ctx.expect_gap(
            frame,
            carriage,
            axis="y",
            positive_elem="rear_guides",
            negative_elem="rear_keepers",
            min_gap=0.0003,
            max_gap=0.0012,
            name="upper_pose_rear_running_clearance",
        )
        ctx.expect_overlap(
            carriage,
            frame,
            elem_a="front_bearing_pads",
            elem_b="front_guides",
            axes="xz",
            min_overlap=0.05,
            name="upper_pose_front_guides_remain_registered",
        )
        ctx.expect_overlap(
            carriage,
            frame,
            elem_a="rear_keepers",
            elem_b="rear_guides",
            axes="xz",
            min_overlap=0.05,
            name="upper_pose_rear_guides_remain_registered",
        )
        ctx.expect_gap(
            frame,
            carriage,
            axis="z",
            positive_elem="upper_stops",
            negative_elem="upper_strikers",
            min_gap=0.006,
            max_gap=0.020,
            name="upper_pose_hard_stop_clearance",
        )
        ctx.expect_overlap(
            carriage,
            frame,
            elem_a="upper_strikers",
            elem_b="upper_stops",
            axes="xy",
            min_overlap=0.010,
            name="upper_stop_faces_stay_aligned",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
