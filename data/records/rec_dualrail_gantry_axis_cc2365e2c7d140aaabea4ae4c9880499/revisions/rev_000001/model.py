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


RAIL_CENTER_X = 0.34
SUPPORT_LEN = 1.02
SUPPORT_BASE_W = 0.10
SUPPORT_TOP_W = 0.075
SUPPORT_LOWER_H = 0.055
SUPPORT_TOP_H = 0.025
SUPPORT_H = SUPPORT_LOWER_H + SUPPORT_TOP_H
FRAME_OUTER_W = (2.0 * RAIL_CENTER_X) + SUPPORT_BASE_W
TIE_W = FRAME_OUTER_W - 0.05
TIE_D = 0.08
TIE_H = 0.03

RAIL_LEN = 0.88
RAIL_W = 0.028
RAIL_H = 0.018
RAIL_TOP_Z = SUPPORT_H + RAIL_H

BASE_STOP_X = 0.050
BASE_STOP_Y = 0.020
BASE_STOP_Z = 0.040
BASE_STOP_CENTER_Y = (RAIL_LEN / 2.0) + 0.025
BASE_STOP_PAD_T = 0.006

BRIDGE_BLOCK_X = 0.076
BRIDGE_BLOCK_Y = 0.095
BRIDGE_BLOCK_Z = 0.048
BRIDGE_BLOCK_CENTER_Y = 0.085
BRIDGE_CHEEK_T = 0.024
BRIDGE_CHEEK_Y = 0.170
BRIDGE_CHEEK_Z0 = 0.036
BRIDGE_CHEEK_Z = 0.175
BRIDGE_CHEEK_X = 0.305

BRIDGE_BEAM_X = 0.700
BRIDGE_BEAM_Y = 0.100
BRIDGE_BEAM_Z = 0.120
BRIDGE_BEAM_BOTTOM_Z = 0.110
BRIDGE_BEAM_WALL = 0.010

BRIDGE_BUMPER_Y = 0.010
BRIDGE_BUMPER_Z = 0.026
BRIDGE_BUMPER_PAD_T = 0.006
BRIDGE_BUMPER_CENTER_Y = 0.168

SLED_RAIL_X = 0.560
SLED_RAIL_Y = 0.018
SLED_RAIL_Z = 0.014
SLED_RAIL_CENTER_Y = 0.026
SLED_RAIL_BOTTOM_Z = BRIDGE_BEAM_BOTTOM_Z - SLED_RAIL_Z

SLED_STOP_X = 0.022
SLED_STOP_Y = 0.055
SLED_STOP_Z = 0.035
SLED_STOP_CENTER_X = 0.305
SLED_STOP_PAD_T = 0.006

SLED_BLOCK_X = 0.090
SLED_BLOCK_Y = 0.032
SLED_BLOCK_Z = 0.048
SLED_WIPER_T = 0.006
SLED_WIPER_Z = 0.024

TOOL_RADIUS = 0.018
TOOL_LENGTH = 0.075

BRIDGE_TRAVEL = 0.260
SLED_TRAVEL = 0.220


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]):
    return cq.Workplane("XY").box(*size).translate(center)


def _add_box_visual(
    part,
    *,
    name: str,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material: str,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def _build_base_frame_shape() -> cq.Workplane:
    shape = None

    for sign in (-1.0, 1.0):
        lower = _cq_box(
            (SUPPORT_BASE_W, SUPPORT_LEN, SUPPORT_LOWER_H),
            (sign * RAIL_CENTER_X, 0.0, SUPPORT_LOWER_H / 2.0),
        )
        upper = _cq_box(
            (SUPPORT_TOP_W, SUPPORT_LEN, SUPPORT_TOP_H),
            (
                sign * RAIL_CENTER_X,
                0.0,
                SUPPORT_LOWER_H + (SUPPORT_TOP_H / 2.0),
            ),
        )
        relief = _cq_box(
            (0.022, SUPPORT_LEN * 0.72, SUPPORT_TOP_H + 0.010),
            (
                sign * (RAIL_CENTER_X - 0.020),
                0.0,
                SUPPORT_LOWER_H + (SUPPORT_TOP_H / 2.0) - 0.004,
            ),
        )
        side = lower.union(upper).cut(relief)
        shape = side if shape is None else shape.union(side)

    for y in (
        (SUPPORT_LEN / 2.0) - (TIE_D / 2.0),
        -(SUPPORT_LEN / 2.0) + (TIE_D / 2.0),
    ):
        tie = _cq_box((TIE_W, TIE_D, TIE_H), (0.0, y, TIE_H / 2.0))
        shape = shape.union(tie)

    for sign in (-1.0, 1.0):
        inner_rib = _cq_box((0.110, 0.060, 0.025), (sign * 0.235, 0.0, 0.0125))
        shape = shape.union(inner_rib)

    return shape


def _build_bridge_body_shape() -> cq.Workplane:
    beam_outer = _cq_box(
        (BRIDGE_BEAM_X, BRIDGE_BEAM_Y, BRIDGE_BEAM_Z),
        (0.0, 0.0, BRIDGE_BEAM_BOTTOM_Z + (BRIDGE_BEAM_Z / 2.0)),
    )
    beam_inner = _cq_box(
        (
            BRIDGE_BEAM_X - (2.0 * BRIDGE_BEAM_WALL),
            BRIDGE_BEAM_Y - (2.0 * BRIDGE_BEAM_WALL),
            BRIDGE_BEAM_Z - (2.0 * BRIDGE_BEAM_WALL),
        ),
        (0.0, 0.0, BRIDGE_BEAM_BOTTOM_Z + (BRIDGE_BEAM_Z / 2.0)),
    )
    shape = beam_outer.cut(beam_inner)

    for sign in (-1.0, 1.0):
        cheek = _cq_box(
            (BRIDGE_CHEEK_T, BRIDGE_CHEEK_Y, BRIDGE_CHEEK_Z),
            (
                sign * BRIDGE_CHEEK_X,
                0.0,
                BRIDGE_CHEEK_Z0 + (BRIDGE_CHEEK_Z / 2.0),
            ),
        )
        cheek_window = _cq_box(
            (BRIDGE_CHEEK_T + 0.010, 0.070, 0.070),
            (sign * BRIDGE_CHEEK_X, 0.0, 0.125),
        )
        cheek = cheek.cut(cheek_window)

        shape = shape.union(cheek)
        shape = shape.union(
            _cq_box((0.050, 0.080, 0.038), (sign * BRIDGE_CHEEK_X, 0.125, 0.072))
        )
        shape = shape.union(
            _cq_box((0.050, 0.080, 0.038), (sign * BRIDGE_CHEEK_X, -0.125, 0.072))
        )
        shape = shape.union(
            _cq_box((0.028, 0.040, 0.085), (sign * 0.286, 0.034, 0.073))
        )
        shape = shape.union(
            _cq_box((0.028, 0.040, 0.085), (sign * 0.286, -0.034, 0.073))
        )

    return shape


def _build_sled_body_shape() -> cq.Workplane:
    shape = _cq_box((0.150, 0.070, 0.012), (0.0, 0.0, -0.054))
    shape = shape.union(_cq_box((0.125, 0.018, 0.170), (0.0, -0.010, -0.145)))
    shape = shape.union(_cq_box((0.100, 0.060, 0.080), (0.0, -0.028, -0.195)))
    shape = shape.union(_cq_box((0.070, 0.045, 0.030), (0.0, -0.022, -0.255)))
    shape = shape.union(_cq_box((0.012, 0.045, 0.105), (0.040, -0.020, -0.118)))
    shape = shape.union(_cq_box((0.012, 0.045, 0.105), (-0.040, -0.020, -0.118)))
    shape = shape.cut(_cq_box((0.050, 0.030, 0.090), (0.0, -0.009, -0.150)))
    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tabletop_gantry_axis")

    model.material("machined_aluminum", rgba=(0.76, 0.78, 0.80, 1.0))
    model.material("carriage_gray", rgba=(0.62, 0.65, 0.69, 1.0))
    model.material("anodized_blue", rgba=(0.26, 0.35, 0.52, 1.0))
    model.material("rail_steel", rgba=(0.30, 0.33, 0.36, 1.0))
    model.material("guide_block", rgba=(0.50, 0.53, 0.58, 1.0))
    model.material("wiper_black", rgba=(0.07, 0.07, 0.08, 1.0))
    model.material("tool_steel", rgba=(0.40, 0.42, 0.46, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_frame_shape(), "gantry_base_frame"),
        material="machined_aluminum",
        name="frame_body",
    )
    _add_box_visual(
        base,
        name="left_rail",
        size=(RAIL_W, RAIL_LEN, RAIL_H),
        center=(RAIL_CENTER_X, 0.0, SUPPORT_H + (RAIL_H / 2.0)),
        material="rail_steel",
    )
    _add_box_visual(
        base,
        name="right_rail",
        size=(RAIL_W, RAIL_LEN, RAIL_H),
        center=(-RAIL_CENTER_X, 0.0, SUPPORT_H + (RAIL_H / 2.0)),
        material="rail_steel",
    )

    for x_sign, rail_name in ((1.0, "left"), (-1.0, "right")):
        rail_x = x_sign * RAIL_CENTER_X
        for y_sign, travel_name in ((1.0, "positive"), (-1.0, "negative")):
            stop_center_y = y_sign * BASE_STOP_CENTER_Y
            body_name = f"{rail_name}_{travel_name}_stop_body"
            pad_name = f"{rail_name}_{travel_name}_stop_pad"
            _add_box_visual(
                base,
                name=body_name,
                size=(BASE_STOP_X, BASE_STOP_Y, BASE_STOP_Z),
                center=(
                    rail_x,
                    stop_center_y,
                    SUPPORT_H + (BASE_STOP_Z / 2.0),
                ),
                material="carriage_gray",
            )
            _add_box_visual(
                base,
                name=pad_name,
                size=(BASE_STOP_X - 0.010, BASE_STOP_PAD_T, BASE_STOP_Z - 0.014),
                center=(
                    rail_x,
                    stop_center_y - (y_sign * ((BASE_STOP_Y / 2.0) - (BASE_STOP_PAD_T / 2.0))),
                    SUPPORT_H + (BASE_STOP_Z / 2.0),
                ),
                material="wiper_black",
            )

    base.inertial = Inertial.from_geometry(
        Box((FRAME_OUTER_W, SUPPORT_LEN, 0.12)),
        mass=23.0,
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
    )

    bridge = model.part("bridge")
    bridge.visual(
        mesh_from_cadquery(_build_bridge_body_shape(), "gantry_bridge_body"),
        material="machined_aluminum",
        name="bridge_body",
    )

    for x_sign, rail_name in ((1.0, "left"), (-1.0, "right")):
        rail_x = x_sign * RAIL_CENTER_X
        for y_sign, travel_name in ((1.0, "front"), (-1.0, "rear")):
            center_y = y_sign * BRIDGE_BLOCK_CENTER_Y
            block_name = f"{rail_name}_{travel_name}_block"
            _add_box_visual(
                bridge,
                name=block_name,
                size=(BRIDGE_BLOCK_X, BRIDGE_BLOCK_Y, BRIDGE_BLOCK_Z),
                center=(rail_x, center_y, BRIDGE_BLOCK_Z / 2.0),
                material="guide_block",
            )

            for wiper_sign, wiper_name in ((1.0, "positive"), (-1.0, "negative")):
                _add_box_visual(
                    bridge,
                    name=f"{block_name}_{wiper_name}_wiper",
                    size=(BRIDGE_BLOCK_X - 0.010, 0.006, 0.020),
                    center=(
                        rail_x,
                        center_y + (wiper_sign * ((BRIDGE_BLOCK_Y / 2.0) - 0.003)),
                        0.016,
                    ),
                    material="wiper_black",
                )

    _add_box_visual(
        bridge,
        name="front_sled_rail",
        size=(SLED_RAIL_X, SLED_RAIL_Y, SLED_RAIL_Z),
        center=(0.0, SLED_RAIL_CENTER_Y, BRIDGE_BEAM_BOTTOM_Z - (SLED_RAIL_Z / 2.0)),
        material="rail_steel",
    )
    _add_box_visual(
        bridge,
        name="rear_sled_rail",
        size=(SLED_RAIL_X, SLED_RAIL_Y, SLED_RAIL_Z),
        center=(0.0, -SLED_RAIL_CENTER_Y, BRIDGE_BEAM_BOTTOM_Z - (SLED_RAIL_Z / 2.0)),
        material="rail_steel",
    )

    for y_sign, travel_name in ((1.0, "positive"), (-1.0, "negative")):
        _add_box_visual(
            bridge,
            name=f"left_{travel_name}_bumper_pad",
            size=(0.040, BRIDGE_BUMPER_PAD_T, BRIDGE_BUMPER_Z),
            center=(
                RAIL_CENTER_X,
                y_sign * BRIDGE_BUMPER_CENTER_Y,
                0.072,
            ),
            material="wiper_black",
        )

    _add_box_visual(
        bridge,
        name="positive_sled_stop_body",
        size=(SLED_STOP_X, SLED_STOP_Y, SLED_STOP_Z),
        center=(SLED_STOP_CENTER_X, 0.0, BRIDGE_BEAM_BOTTOM_Z - (SLED_STOP_Z / 2.0)),
        material="carriage_gray",
    )
    _add_box_visual(
        bridge,
        name="negative_sled_stop_body",
        size=(SLED_STOP_X, SLED_STOP_Y, SLED_STOP_Z),
        center=(-SLED_STOP_CENTER_X, 0.0, BRIDGE_BEAM_BOTTOM_Z - (SLED_STOP_Z / 2.0)),
        material="carriage_gray",
    )
    _add_box_visual(
        bridge,
        name="positive_sled_stop_pad",
        size=(SLED_STOP_PAD_T, SLED_STOP_Y - 0.010, SLED_STOP_Z - 0.013),
        center=(
            SLED_STOP_CENTER_X - ((SLED_STOP_X / 2.0) - (SLED_STOP_PAD_T / 2.0)),
            0.0,
            BRIDGE_BEAM_BOTTOM_Z - (SLED_STOP_Z / 2.0),
        ),
        material="wiper_black",
    )
    _add_box_visual(
        bridge,
        name="negative_sled_stop_pad",
        size=(SLED_STOP_PAD_T, SLED_STOP_Y - 0.010, SLED_STOP_Z - 0.013),
        center=(
            -SLED_STOP_CENTER_X + ((SLED_STOP_X / 2.0) - (SLED_STOP_PAD_T / 2.0)),
            0.0,
            BRIDGE_BEAM_BOTTOM_Z - (SLED_STOP_Z / 2.0),
        ),
        material="wiper_black",
    )

    bridge.inertial = Inertial.from_geometry(
        Box((0.78, 0.20, 0.24)),
        mass=10.5,
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
    )

    sled = model.part("sled")
    sled.visual(
        mesh_from_cadquery(_build_sled_body_shape(), "gantry_tool_sled_body"),
        material="anodized_blue",
        name="sled_body",
    )
    _add_box_visual(
        sled,
        name="front_block",
        size=(SLED_BLOCK_X, SLED_BLOCK_Y, SLED_BLOCK_Z),
        center=(0.0, SLED_RAIL_CENTER_Y, -(SLED_BLOCK_Z / 2.0)),
        material="guide_block",
    )
    _add_box_visual(
        sled,
        name="rear_block",
        size=(SLED_BLOCK_X, SLED_BLOCK_Y, SLED_BLOCK_Z),
        center=(0.0, -SLED_RAIL_CENTER_Y, -(SLED_BLOCK_Z / 2.0)),
        material="guide_block",
    )

    for rail_y, prefix in ((SLED_RAIL_CENTER_Y, "front"), (-SLED_RAIL_CENTER_Y, "rear")):
        for x_sign, travel_name in ((1.0, "positive"), (-1.0, "negative")):
            _add_box_visual(
                sled,
                name=f"{prefix}_{travel_name}_wiper",
                size=(SLED_WIPER_T, SLED_BLOCK_Y - 0.004, SLED_WIPER_Z),
                center=(
                    x_sign * ((SLED_BLOCK_X / 2.0) - (SLED_WIPER_T / 2.0)),
                    rail_y,
                    -0.020,
                ),
                material="wiper_black",
            )

    sled.visual(
        Cylinder(radius=TOOL_RADIUS, length=TOOL_LENGTH),
        origin=Origin(xyz=(0.0, -0.022, -0.3075)),
        material="tool_steel",
        name="tool_body",
    )
    sled.inertial = Inertial.from_geometry(
        Box((0.16, 0.08, 0.34)),
        mass=3.4,
        origin=Origin(xyz=(0.0, -0.01, -0.17)),
    )

    model.articulation(
        "base_to_bridge",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bridge,
        origin=Origin(xyz=(0.0, 0.0, RAIL_TOP_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.65,
            lower=-BRIDGE_TRAVEL,
            upper=BRIDGE_TRAVEL,
        ),
    )
    model.articulation(
        "bridge_to_sled",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=sled,
        origin=Origin(xyz=(0.0, 0.0, SLED_RAIL_BOTTOM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=240.0,
            velocity=0.45,
            lower=-SLED_TRAVEL,
            upper=SLED_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    bridge = object_model.get_part("bridge")
    sled = object_model.get_part("sled")
    bridge_slide = object_model.get_articulation("base_to_bridge")
    sled_slide = object_model.get_articulation("bridge_to_sled")

    left_rail = base.get_visual("left_rail")
    right_rail = base.get_visual("right_rail")
    left_front_block = bridge.get_visual("left_front_block")
    right_rear_block = bridge.get_visual("right_rear_block")
    front_sled_rail = bridge.get_visual("front_sled_rail")
    rear_sled_rail = bridge.get_visual("rear_sled_rail")
    front_block = sled.get_visual("front_block")
    rear_block = sled.get_visual("rear_block")
    left_positive_stop_pad = base.get_visual("left_positive_stop_pad")
    left_negative_stop_pad = base.get_visual("left_negative_stop_pad")
    left_positive_bumper_pad = bridge.get_visual("left_positive_bumper_pad")
    left_negative_bumper_pad = bridge.get_visual("left_negative_bumper_pad")
    positive_sled_stop_pad = bridge.get_visual("positive_sled_stop_pad")
    negative_sled_stop_pad = bridge.get_visual("negative_sled_stop_pad")

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

    bridge_limits = bridge_slide.motion_limits
    sled_limits = sled_slide.motion_limits
    ctx.check(
        "bridge_joint_axis_is_rail_direction",
        bridge_slide.axis == (0.0, 1.0, 0.0)
        and bridge_limits is not None
        and bridge_limits.lower == -BRIDGE_TRAVEL
        and bridge_limits.upper == BRIDGE_TRAVEL,
        details=f"axis={bridge_slide.axis}, limits={bridge_limits}",
    )
    ctx.check(
        "sled_joint_axis_is_beam_direction",
        sled_slide.axis == (1.0, 0.0, 0.0)
        and sled_limits is not None
        and sled_limits.lower == -SLED_TRAVEL
        and sled_limits.upper == SLED_TRAVEL,
        details=f"axis={sled_slide.axis}, limits={sled_limits}",
    )

    ctx.expect_contact(
        bridge,
        base,
        elem_a=left_front_block,
        elem_b=left_rail,
        name="left_front_bridge_block_contacts_left_rail",
    )
    ctx.expect_contact(
        bridge,
        base,
        elem_a=right_rear_block,
        elem_b=right_rail,
        name="right_rear_bridge_block_contacts_right_rail",
    )
    ctx.expect_contact(
        sled,
        bridge,
        elem_a=front_block,
        elem_b=front_sled_rail,
        name="front_sled_block_contacts_front_beam_rail",
    )
    ctx.expect_contact(
        sled,
        bridge,
        elem_a=rear_block,
        elem_b=rear_sled_rail,
        name="rear_sled_block_contacts_rear_beam_rail",
    )

    bridge_aabb = ctx.part_world_aabb(bridge)
    sled_aabb = ctx.part_world_aabb(sled)
    if bridge_aabb is None or sled_aabb is None:
        ctx.fail("bridge_and_sled_aabbs_available", "Expected both moving parts to have world AABBs.")
    else:
        bridge_x = bridge_aabb[1][0] - bridge_aabb[0][0]
        sled_x = sled_aabb[1][0] - sled_aabb[0][0]
        sled_z = sled_aabb[1][2] - sled_aabb[0][2]
        ctx.check(
            "bridge_reads_larger_than_tool_sled",
            bridge_x > (3.0 * sled_x) and bridge_x > 0.70 and sled_z > 0.25,
            details=f"bridge_x={bridge_x:.3f}, sled_x={sled_x:.3f}, sled_z={sled_z:.3f}",
        )

    with ctx.pose({bridge_slide: BRIDGE_TRAVEL}):
        ctx.expect_gap(
            base,
            bridge,
            axis="y",
            min_gap=0.015,
            max_gap=0.035,
            positive_elem=left_positive_stop_pad,
            negative_elem=left_positive_bumper_pad,
            name="bridge_positive_endstop_clearance",
        )

    with ctx.pose({bridge_slide: -BRIDGE_TRAVEL}):
        ctx.expect_gap(
            bridge,
            base,
            axis="y",
            min_gap=0.015,
            max_gap=0.035,
            positive_elem=left_negative_bumper_pad,
            negative_elem=left_negative_stop_pad,
            name="bridge_negative_endstop_clearance",
        )

    with ctx.pose({sled_slide: SLED_TRAVEL}):
        ctx.expect_gap(
            bridge,
            sled,
            axis="x",
            min_gap=0.020,
            max_gap=0.040,
            positive_elem=positive_sled_stop_pad,
            negative_elem=front_block,
            name="sled_positive_endstop_clearance",
        )

    with ctx.pose({sled_slide: -SLED_TRAVEL}):
        ctx.expect_gap(
            sled,
            bridge,
            axis="x",
            min_gap=0.020,
            max_gap=0.040,
            positive_elem=front_block,
            negative_elem=negative_sled_stop_pad,
            name="sled_negative_endstop_clearance",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
