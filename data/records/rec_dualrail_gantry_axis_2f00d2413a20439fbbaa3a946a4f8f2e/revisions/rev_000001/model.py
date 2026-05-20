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


BASE_LENGTH = 1.80
BASE_WIDTH = 0.92
DECK_THICKNESS = 0.04
SIDE_FRAME_WIDTH = 0.06
SIDE_FRAME_HEIGHT = 0.16
SIDE_FRAME_LENGTH = 1.62
SIDE_FRAME_X = 0.43
END_TIE_LENGTH = 0.10

LOWER_RAIL_X = 0.29
LOWER_RAIL_LENGTH = 1.55
LOWER_RAIL_WIDTH = 0.06
LOWER_RAIL_HEIGHT = 0.026
RAIL_SUPPORT_Y = 0.735
RAIL_SUPPORT_LENGTH = 0.08
RAIL_SUPPORT_WIDTH = 0.10
RAIL_SUPPORT_HEIGHT = 0.018

BRIDGE_BEARING_WIDTH = 0.12
BRIDGE_BEARING_LENGTH = 0.24
BRIDGE_BEARING_HEIGHT = 0.07
BRIDGE_SIDE_X = 0.34
BRIDGE_UPRIGHT_WIDTH = 0.10
BRIDGE_UPRIGHT_DEPTH = 0.18
BRIDGE_UPRIGHT_HEIGHT = 0.56
BRIDGE_SHOULDER_WIDTH = 0.16
BRIDGE_SHOULDER_DEPTH = 0.22
BRIDGE_SHOULDER_HEIGHT = 0.10
BRIDGE_BEAM_LENGTH = 0.82
BRIDGE_BEAM_DEPTH = 0.16
BRIDGE_BEAM_HEIGHT = 0.18
BRIDGE_BEAM_CENTER_Z = 0.72
BRIDGE_TIE_LENGTH = 0.56
BRIDGE_TIE_DEPTH = 0.10
BRIDGE_TIE_HEIGHT = 0.06
BRIDGE_TIE_CENTER_Z = 0.11

UPPER_GUIDE_LENGTH = 0.68
UPPER_GUIDE_DEPTH = 0.022
UPPER_GUIDE_HEIGHT = 0.03
UPPER_GUIDE_UPPER_Z = BRIDGE_BEAM_CENTER_Z + 0.04
UPPER_GUIDE_LOWER_Z = BRIDGE_BEAM_CENTER_Z - 0.04
UPPER_GUIDE_CENTER_Y = BRIDGE_BEAM_DEPTH / 2.0 + UPPER_GUIDE_DEPTH / 2.0
UPPER_GUIDE_FRONT_Y = BRIDGE_BEAM_DEPTH / 2.0 + UPPER_GUIDE_DEPTH

BRIDGE_TRAVEL = 0.52
TRUCK_TRAVEL = 0.24


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    cx, cy, cz = center
    return cq.Workplane("XY").box(sx, sy, sz).translate((cx, cy, cz))


def _make_base_shape() -> cq.Workplane:
    return _box((BASE_WIDTH, BASE_LENGTH, DECK_THICKNESS), (0.0, 0.0, DECK_THICKNESS / 2.0))


def _make_side_frame_shape() -> cq.Workplane:
    frame = _box(
        (SIDE_FRAME_WIDTH, SIDE_FRAME_LENGTH, SIDE_FRAME_HEIGHT),
        (0.0, 0.0, SIDE_FRAME_HEIGHT / 2.0),
    )
    relief = _box(
        (SIDE_FRAME_WIDTH * 0.55, SIDE_FRAME_LENGTH - 0.22, SIDE_FRAME_HEIGHT * 0.55),
        (0.0, 0.0, SIDE_FRAME_HEIGHT * 0.55),
    )
    return frame.cut(relief)


def _make_rail_support_shape() -> cq.Workplane:
    return _box(
        (RAIL_SUPPORT_WIDTH, RAIL_SUPPORT_LENGTH, RAIL_SUPPORT_HEIGHT),
        (0.0, 0.0, RAIL_SUPPORT_HEIGHT / 2.0),
    )


def _make_lower_rail_shape() -> cq.Workplane:
    rail = _box((LOWER_RAIL_WIDTH, LOWER_RAIL_LENGTH, 0.012), (0.0, 0.0, 0.006))
    rail = rail.union(_box((0.032, LOWER_RAIL_LENGTH, 0.014), (0.0, 0.0, 0.019)))
    return rail


def _make_bridge_frame_shape() -> cq.Workplane:
    frame = _box(
        (BRIDGE_TIE_LENGTH, BRIDGE_TIE_DEPTH, BRIDGE_TIE_HEIGHT),
        (0.0, 0.0, BRIDGE_TIE_CENTER_Z),
    )
    for side in (-1.0, 1.0):
        frame = frame.union(
            _box(
                (BRIDGE_UPRIGHT_WIDTH, BRIDGE_UPRIGHT_DEPTH, BRIDGE_UPRIGHT_HEIGHT),
                (
                    side * BRIDGE_SIDE_X,
                    0.0,
                    BRIDGE_BEARING_HEIGHT + BRIDGE_UPRIGHT_HEIGHT / 2.0 - 0.01,
                ),
            )
        )
        frame = frame.union(
            _box(
                (BRIDGE_SHOULDER_WIDTH, BRIDGE_SHOULDER_DEPTH, BRIDGE_SHOULDER_HEIGHT),
                (
                    side * BRIDGE_SIDE_X,
                    0.0,
                    BRIDGE_BEAM_CENTER_Z - 0.02,
                ),
            )
        )
    frame = frame.union(
        _box(
            (BRIDGE_BEAM_LENGTH, BRIDGE_BEAM_DEPTH, BRIDGE_BEAM_HEIGHT),
            (0.0, 0.0, BRIDGE_BEAM_CENTER_Z),
        )
    )
    return frame


def _make_bearing_shape() -> cq.Workplane:
    return _box(
        (BRIDGE_BEARING_WIDTH, BRIDGE_BEARING_LENGTH, BRIDGE_BEARING_HEIGHT),
        (0.0, 0.0, BRIDGE_BEARING_HEIGHT / 2.0),
    )


def _make_upper_guide_shape() -> cq.Workplane:
    return _box((UPPER_GUIDE_LENGTH, UPPER_GUIDE_DEPTH, UPPER_GUIDE_HEIGHT), (0.0, 0.0, 0.0))


def _make_truck_body_shape() -> cq.Workplane:
    body = _box((0.20, 0.12, 0.18), (0.0, 0.06, -0.02))
    body = body.union(_box((0.11, 0.09, 0.14), (0.0, 0.125, -0.12)))
    body = body.union(_box((0.12, 0.05, 0.05), (0.0, 0.095, 0.075)))
    return body


def _make_truck_shoe_shape(z_center: float) -> cq.Workplane:
    shoe = _box((0.05, 0.022, 0.03), (-0.06, 0.011, z_center))
    shoe = shoe.union(_box((0.05, 0.022, 0.03), (0.06, 0.011, z_center)))
    return shoe


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="metrology_bridge_axis")

    painted = model.material("painted_machine_frame", rgba=(0.77, 0.79, 0.81, 1.0))
    dark = model.material("dark_carriage", rgba=(0.20, 0.22, 0.24, 1.0))
    steel = model.material("ground_steel", rgba=(0.64, 0.67, 0.70, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_shape(), "base"),
        material=painted,
        name="base_structure",
    )

    left_side_frame = model.part("left_side_frame")
    left_side_frame.visual(
        mesh_from_cadquery(_make_side_frame_shape(), "left_side_frame"),
        material=painted,
        name="frame",
    )

    right_side_frame = model.part("right_side_frame")
    right_side_frame.visual(
        mesh_from_cadquery(_make_side_frame_shape(), "right_side_frame"),
        material=painted,
        name="frame",
    )

    left_support_front = model.part("left_support_front")
    left_support_front.visual(
        mesh_from_cadquery(_make_rail_support_shape(), "left_support_front"),
        material=painted,
        name="support",
    )

    left_support_rear = model.part("left_support_rear")
    left_support_rear.visual(
        mesh_from_cadquery(_make_rail_support_shape(), "left_support_rear"),
        material=painted,
        name="support",
    )

    right_support_front = model.part("right_support_front")
    right_support_front.visual(
        mesh_from_cadquery(_make_rail_support_shape(), "right_support_front"),
        material=painted,
        name="support",
    )

    right_support_rear = model.part("right_support_rear")
    right_support_rear.visual(
        mesh_from_cadquery(_make_rail_support_shape(), "right_support_rear"),
        material=painted,
        name="support",
    )

    left_rail = model.part("left_rail")
    left_rail.visual(
        mesh_from_cadquery(_make_lower_rail_shape(), "left_rail"),
        material=steel,
        name="rail",
    )

    right_rail = model.part("right_rail")
    right_rail.visual(
        mesh_from_cadquery(_make_lower_rail_shape(), "right_rail"),
        material=steel,
        name="rail",
    )

    bridge = model.part("bridge")
    bridge.visual(
        mesh_from_cadquery(_make_bridge_frame_shape(), "bridge_frame"),
        material=painted,
        name="frame",
    )
    bridge.visual(
        mesh_from_cadquery(_make_bearing_shape(), "bridge_left_bearing"),
        origin=Origin(xyz=(-LOWER_RAIL_X, 0.0, 0.0)),
        material=dark,
        name="left_bearing",
    )
    bridge.visual(
        mesh_from_cadquery(_make_bearing_shape(), "bridge_right_bearing"),
        origin=Origin(xyz=(LOWER_RAIL_X, 0.0, 0.0)),
        material=dark,
        name="right_bearing",
    )
    bridge.visual(
        mesh_from_cadquery(_make_upper_guide_shape(), "beam_upper_guide"),
        origin=Origin(xyz=(0.0, UPPER_GUIDE_CENTER_Y, UPPER_GUIDE_UPPER_Z)),
        material=steel,
        name="beam_upper_guide",
    )
    bridge.visual(
        mesh_from_cadquery(_make_upper_guide_shape(), "beam_lower_guide"),
        origin=Origin(xyz=(0.0, UPPER_GUIDE_CENTER_Y, UPPER_GUIDE_LOWER_Z)),
        material=steel,
        name="beam_lower_guide",
    )

    truck = model.part("truck")
    truck.visual(
        mesh_from_cadquery(_make_truck_body_shape(), "truck_body"),
        material=dark,
        name="truck_body",
    )
    truck.visual(
        mesh_from_cadquery(_make_truck_shoe_shape(0.04), "truck_upper_shoes"),
        material=steel,
        name="upper_shoes",
    )
    truck.visual(
        mesh_from_cadquery(_make_truck_shoe_shape(-0.04), "truck_lower_shoes"),
        material=steel,
        name="lower_shoes",
    )

    model.articulation(
        "base_to_left_side_frame",
        ArticulationType.FIXED,
        parent=base,
        child=left_side_frame,
        origin=Origin(xyz=(-SIDE_FRAME_X, 0.0, DECK_THICKNESS)),
    )
    model.articulation(
        "base_to_right_side_frame",
        ArticulationType.FIXED,
        parent=base,
        child=right_side_frame,
        origin=Origin(xyz=(SIDE_FRAME_X, 0.0, DECK_THICKNESS)),
    )
    model.articulation(
        "base_to_left_support_front",
        ArticulationType.FIXED,
        parent=base,
        child=left_support_front,
        origin=Origin(xyz=(-LOWER_RAIL_X, RAIL_SUPPORT_Y, DECK_THICKNESS)),
    )
    model.articulation(
        "base_to_left_support_rear",
        ArticulationType.FIXED,
        parent=base,
        child=left_support_rear,
        origin=Origin(xyz=(-LOWER_RAIL_X, -RAIL_SUPPORT_Y, DECK_THICKNESS)),
    )
    model.articulation(
        "base_to_right_support_front",
        ArticulationType.FIXED,
        parent=base,
        child=right_support_front,
        origin=Origin(xyz=(LOWER_RAIL_X, RAIL_SUPPORT_Y, DECK_THICKNESS)),
    )
    model.articulation(
        "base_to_right_support_rear",
        ArticulationType.FIXED,
        parent=base,
        child=right_support_rear,
        origin=Origin(xyz=(LOWER_RAIL_X, -RAIL_SUPPORT_Y, DECK_THICKNESS)),
    )
    model.articulation(
        "base_to_left_rail",
        ArticulationType.FIXED,
        parent=base,
        child=left_rail,
        origin=Origin(xyz=(-LOWER_RAIL_X, 0.0, DECK_THICKNESS + RAIL_SUPPORT_HEIGHT)),
    )
    model.articulation(
        "base_to_right_rail",
        ArticulationType.FIXED,
        parent=base,
        child=right_rail,
        origin=Origin(xyz=(LOWER_RAIL_X, 0.0, DECK_THICKNESS + RAIL_SUPPORT_HEIGHT)),
    )
    model.articulation(
        "base_to_bridge",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bridge,
        origin=Origin(xyz=(0.0, 0.0, DECK_THICKNESS + RAIL_SUPPORT_HEIGHT + LOWER_RAIL_HEIGHT)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.8,
            lower=-BRIDGE_TRAVEL,
            upper=BRIDGE_TRAVEL,
        ),
    )
    model.articulation(
        "bridge_to_truck",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=truck,
        origin=Origin(xyz=(0.0, UPPER_GUIDE_FRONT_Y, BRIDGE_BEAM_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=350.0,
            velocity=0.6,
            lower=-TRUCK_TRAVEL,
            upper=TRUCK_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    left_side_frame = object_model.get_part("left_side_frame")
    right_side_frame = object_model.get_part("right_side_frame")
    left_support_front = object_model.get_part("left_support_front")
    left_support_rear = object_model.get_part("left_support_rear")
    right_support_front = object_model.get_part("right_support_front")
    right_support_rear = object_model.get_part("right_support_rear")
    left_rail = object_model.get_part("left_rail")
    right_rail = object_model.get_part("right_rail")
    bridge = object_model.get_part("bridge")
    truck = object_model.get_part("truck")

    bridge_axis = object_model.get_articulation("base_to_bridge")
    truck_axis = object_model.get_articulation("bridge_to_truck")

    left_bearing = bridge.get_visual("left_bearing")
    right_bearing = bridge.get_visual("right_bearing")
    upper_guide = bridge.get_visual("beam_upper_guide")
    lower_guide = bridge.get_visual("beam_lower_guide")
    upper_shoes = truck.get_visual("upper_shoes")
    lower_shoes = truck.get_visual("lower_shoes")
    rail_visual = left_rail.get_visual("rail")

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
        "bridge_axis_is_longitudinal_prismatic",
        bridge_axis.articulation_type == ArticulationType.PRISMATIC
        and tuple(float(v) for v in bridge_axis.axis) == (0.0, 1.0, 0.0),
        details=f"type={bridge_axis.articulation_type}, axis={bridge_axis.axis}",
    )
    ctx.check(
        "truck_axis_is_crossbeam_prismatic",
        truck_axis.articulation_type == ArticulationType.PRISMATIC
        and tuple(float(v) for v in truck_axis.axis) == (1.0, 0.0, 0.0),
        details=f"type={truck_axis.articulation_type}, axis={truck_axis.axis}",
    )

    ctx.expect_contact(left_side_frame, base, name="left_side_frame_attached_to_base")
    ctx.expect_contact(right_side_frame, base, name="right_side_frame_attached_to_base")
    ctx.expect_contact(left_support_front, base, name="left_front_support_attached_to_base")
    ctx.expect_contact(left_support_rear, base, name="left_rear_support_attached_to_base")
    ctx.expect_contact(right_support_front, base, name="right_front_support_attached_to_base")
    ctx.expect_contact(right_support_rear, base, name="right_rear_support_attached_to_base")
    ctx.expect_contact(left_rail, left_support_front, name="left_rail_on_front_support")
    ctx.expect_contact(left_rail, left_support_rear, name="left_rail_on_rear_support")
    ctx.expect_contact(right_rail, right_support_front, name="right_rail_on_front_support")
    ctx.expect_contact(right_rail, right_support_rear, name="right_rail_on_rear_support")
    ctx.expect_contact(bridge, left_rail, name="bridge_contacts_left_rail")
    ctx.expect_contact(bridge, right_rail, name="bridge_contacts_right_rail")

    ctx.expect_gap(
        truck,
        bridge,
        axis="y",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem=upper_shoes,
        negative_elem=upper_guide,
        name="truck_upper_shoes_seat_on_upper_guide",
    )
    ctx.expect_gap(
        truck,
        bridge,
        axis="y",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem=lower_shoes,
        negative_elem=lower_guide,
        name="truck_lower_shoes_seat_on_lower_guide",
    )
    ctx.expect_within(
        truck,
        bridge,
        axes="x",
        margin=0.002,
        inner_elem=upper_shoes,
        outer_elem=upper_guide,
        name="truck_upper_shoes_within_upper_guide_span",
    )
    ctx.expect_within(
        truck,
        bridge,
        axes="x",
        margin=0.002,
        inner_elem=lower_shoes,
        outer_elem=lower_guide,
        name="truck_lower_shoes_within_lower_guide_span",
    )

    with ctx.pose({bridge_axis: bridge_axis.motion_limits.lower}):
        ctx.expect_within(
            bridge,
            left_rail,
            axes="y",
            margin=0.002,
            inner_elem=left_bearing,
            outer_elem=rail_visual,
            name="left_bearing_within_left_rail_at_negative_travel",
        )
        ctx.expect_within(
            bridge,
            right_rail,
            axes="y",
            margin=0.002,
            inner_elem=right_bearing,
            outer_elem=rail_visual,
            name="right_bearing_within_right_rail_at_negative_travel",
        )

    with ctx.pose({bridge_axis: bridge_axis.motion_limits.upper}):
        ctx.expect_within(
            bridge,
            left_rail,
            axes="y",
            margin=0.002,
            inner_elem=left_bearing,
            outer_elem=rail_visual,
            name="left_bearing_within_left_rail_at_positive_travel",
        )
        ctx.expect_within(
            bridge,
            right_rail,
            axes="y",
            margin=0.002,
            inner_elem=right_bearing,
            outer_elem=rail_visual,
            name="right_bearing_within_right_rail_at_positive_travel",
        )

    with ctx.pose({truck_axis: truck_axis.motion_limits.lower}):
        ctx.expect_within(
            truck,
            bridge,
            axes="x",
            margin=0.002,
            inner_elem=upper_shoes,
            outer_elem=upper_guide,
            name="truck_within_guides_at_negative_crossbeam_travel",
        )

    with ctx.pose({truck_axis: truck_axis.motion_limits.upper}):
        ctx.expect_within(
            truck,
            bridge,
            axes="x",
            margin=0.002,
            inner_elem=upper_shoes,
            outer_elem=upper_guide,
            name="truck_within_guides_at_positive_crossbeam_travel",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
