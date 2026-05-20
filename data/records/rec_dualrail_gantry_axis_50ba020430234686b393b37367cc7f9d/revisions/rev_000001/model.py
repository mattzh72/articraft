from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 1.08
BASE_WIDTH = 0.58
FOOT_SIZE = 0.08
FOOT_HEIGHT = 0.02
FRAME_BEAM_HEIGHT = 0.06
FRAME_SIDE_WIDTH = 0.08
FRAME_END_WIDTH = 0.08
RAIL_LENGTH = 0.92
RAIL_WIDTH = 0.04
RAIL_HEIGHT = 0.028
RAIL_Y = 0.22
RAIL_TOP_Z = FOOT_HEIGHT + FRAME_BEAM_HEIGHT + RAIL_HEIGHT

BRIDGE_TRAVEL = 0.18
SHUTTLE_TRAVEL = 0.14

BRIDGE_SHOE_X = 0.16
BRIDGE_SHOE_Y = 0.085
BRIDGE_SHOE_Z = 0.04
BRIDGE_UPRIGHT_X = 0.10
BRIDGE_UPRIGHT_Y = 0.05
BRIDGE_UPRIGHT_Z = 0.312
BRIDGE_ROOF_X = 0.10
BRIDGE_ROOF_Y = 0.50
BRIDGE_ROOF_Z = 0.028
BRIDGE_ROOF_CENTER_Z = 0.366
BRIDGE_GUIDE_X = 0.014
BRIDGE_GUIDE_Y = 0.44
BRIDGE_GUIDE_Z = 0.050
BRIDGE_GUIDE_CENTER_Z = 0.327
BRIDGE_GUIDE_OFFSET_X = 0.040

SHUTTLE_HEAD_X = 0.066
SHUTTLE_HEAD_Y = 0.11
SHUTTLE_HEAD_Z = 0.026
SHUTTLE_HEAD_CENTER_Z = BRIDGE_GUIDE_CENTER_Z
SHUTTLE_STEM_X = 0.040
SHUTTLE_STEM_Y = 0.094
SHUTTLE_STEM_Z = 0.070
SHUTTLE_STEM_CENTER_Z = SHUTTLE_HEAD_CENTER_Z - 0.048
SHUTTLE_BODY_X = 0.085
SHUTTLE_BODY_Y = 0.12
SHUTTLE_BODY_Z = 0.10
SHUTTLE_BODY_CENTER_Z = SHUTTLE_HEAD_CENTER_Z - 0.133
SHUTTLE_NOSE_X = 0.060
SHUTTLE_NOSE_Y = 0.060
SHUTTLE_NOSE_Z = 0.060
SHUTTLE_NOSE_CENTER_Z = SHUTTLE_HEAD_CENTER_Z - 0.213


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _union(workplanes: list[cq.Workplane]) -> cq.Workplane:
    shape = workplanes[0]
    for other in workplanes[1:]:
        shape = shape.union(other)
    return shape


def _base_frame_shape() -> cq.Workplane:
    half_x = (BASE_LENGTH / 2.0) - (FRAME_END_WIDTH / 2.0)
    half_y = RAIL_Y
    beam_center_z = FOOT_HEIGHT + (FRAME_BEAM_HEIGHT / 2.0)
    foot_center_z = FOOT_HEIGHT / 2.0

    return _union(
        [
            _box((BASE_LENGTH, FRAME_SIDE_WIDTH, FRAME_BEAM_HEIGHT), (0.0, half_y, beam_center_z)),
            _box((BASE_LENGTH, FRAME_SIDE_WIDTH, FRAME_BEAM_HEIGHT), (0.0, -half_y, beam_center_z)),
            _box(
                (FRAME_END_WIDTH, (2.0 * RAIL_Y) + FRAME_SIDE_WIDTH, FRAME_BEAM_HEIGHT),
                (half_x, 0.0, beam_center_z),
            ),
            _box(
                (FRAME_END_WIDTH, (2.0 * RAIL_Y) + FRAME_SIDE_WIDTH, FRAME_BEAM_HEIGHT),
                (-half_x, 0.0, beam_center_z),
            ),
            _box((FOOT_SIZE, FOOT_SIZE, FOOT_HEIGHT), (half_x, half_y, foot_center_z)),
            _box((FOOT_SIZE, FOOT_SIZE, FOOT_HEIGHT), (half_x, -half_y, foot_center_z)),
            _box((FOOT_SIZE, FOOT_SIZE, FOOT_HEIGHT), (-half_x, half_y, foot_center_z)),
            _box((FOOT_SIZE, FOOT_SIZE, FOOT_HEIGHT), (-half_x, -half_y, foot_center_z)),
        ]
    )


def _base_rails_shape() -> cq.Workplane:
    rail_center_z = FOOT_HEIGHT + FRAME_BEAM_HEIGHT + (RAIL_HEIGHT / 2.0)
    return _union(
        [
            _box((RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT), (0.0, RAIL_Y, rail_center_z)),
            _box((RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT), (0.0, -RAIL_Y, rail_center_z)),
        ]
    )


def _shuttle_shape() -> cq.Workplane:
    return _union(
        [
            _box((SHUTTLE_HEAD_X, SHUTTLE_HEAD_Y, SHUTTLE_HEAD_Z), (0.0, 0.0, 0.0)),
            _box(
                (SHUTTLE_STEM_X, SHUTTLE_STEM_Y, SHUTTLE_STEM_Z),
                (0.0, 0.0, SHUTTLE_STEM_CENTER_Z - SHUTTLE_HEAD_CENTER_Z),
            ),
            _box(
                (SHUTTLE_BODY_X, SHUTTLE_BODY_Y, SHUTTLE_BODY_Z),
                (0.0, 0.0, SHUTTLE_BODY_CENTER_Z - SHUTTLE_HEAD_CENTER_Z),
            ),
            _box(
                (SHUTTLE_NOSE_X, SHUTTLE_NOSE_Y, SHUTTLE_NOSE_Z),
                (0.0, 0.0, SHUTTLE_NOSE_CENTER_Z - SHUTTLE_HEAD_CENTER_Z),
            ),
        ]
    )


def _shuttle_faceplate_shape() -> cq.Workplane:
    return _box(
        (0.006, SHUTTLE_BODY_Y * 0.92, SHUTTLE_BODY_Z * 0.76),
        (
            (SHUTTLE_BODY_X / 2.0) - 0.003,
            0.0,
            (SHUTTLE_BODY_CENTER_Z - SHUTTLE_HEAD_CENTER_Z) - 0.004,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_automation_gantry")

    model.material("frame_graphite", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("rail_steel", rgba=(0.65, 0.67, 0.70, 1.0))
    model.material("bridge_aluminum", rgba=(0.78, 0.80, 0.83, 1.0))
    model.material("shuttle_dark", rgba=(0.18, 0.19, 0.22, 1.0))
    model.material("accent_blue", rgba=(0.16, 0.42, 0.80, 1.0))

    base = model.part("base_frame")
    base.visual(
        mesh_from_cadquery(_base_frame_shape(), "base_frame_shell"),
        material="frame_graphite",
        name="frame_shell",
    )
    base.visual(
        mesh_from_cadquery(_base_rails_shape(), "base_rails"),
        material="rail_steel",
        name="rails",
    )

    bridge = model.part("bridge_carriage")
    bridge.visual(
        Box((BRIDGE_SHOE_X, BRIDGE_SHOE_Y, BRIDGE_SHOE_Z)),
        origin=Origin(xyz=(0.0, RAIL_Y, BRIDGE_SHOE_Z / 2.0)),
        material="bridge_aluminum",
        name="upper_shoe",
    )
    bridge.visual(
        Box((BRIDGE_SHOE_X, BRIDGE_SHOE_Y, BRIDGE_SHOE_Z)),
        origin=Origin(xyz=(0.0, -RAIL_Y, BRIDGE_SHOE_Z / 2.0)),
        material="bridge_aluminum",
        name="lower_shoe",
    )
    bridge.visual(
        Box((BRIDGE_UPRIGHT_X, BRIDGE_UPRIGHT_Y, BRIDGE_UPRIGHT_Z)),
        origin=Origin(xyz=(0.0, RAIL_Y, BRIDGE_SHOE_Z + (BRIDGE_UPRIGHT_Z / 2.0))),
        material="bridge_aluminum",
        name="upper_upright",
    )
    bridge.visual(
        Box((BRIDGE_UPRIGHT_X, BRIDGE_UPRIGHT_Y, BRIDGE_UPRIGHT_Z)),
        origin=Origin(xyz=(0.0, -RAIL_Y, BRIDGE_SHOE_Z + (BRIDGE_UPRIGHT_Z / 2.0))),
        material="bridge_aluminum",
        name="lower_upright",
    )
    bridge.visual(
        Box((BRIDGE_ROOF_X, BRIDGE_ROOF_Y, BRIDGE_ROOF_Z)),
        origin=Origin(xyz=(0.0, 0.0, BRIDGE_ROOF_CENTER_Z)),
        material="bridge_aluminum",
        name="roof",
    )
    bridge.visual(
        Box((BRIDGE_GUIDE_X, BRIDGE_GUIDE_Y, BRIDGE_GUIDE_Z)),
        origin=Origin(xyz=(BRIDGE_GUIDE_OFFSET_X, 0.0, BRIDGE_GUIDE_CENTER_Z)),
        material="rail_steel",
        name="front_guide",
    )
    bridge.visual(
        Box((BRIDGE_GUIDE_X, BRIDGE_GUIDE_Y, BRIDGE_GUIDE_Z)),
        origin=Origin(xyz=(-BRIDGE_GUIDE_OFFSET_X, 0.0, BRIDGE_GUIDE_CENTER_Z)),
        material="rail_steel",
        name="rear_guide",
    )

    shuttle = model.part("center_shuttle")
    shuttle.visual(
        Box((SHUTTLE_HEAD_X, SHUTTLE_HEAD_Y, SHUTTLE_HEAD_Z)),
        origin=Origin(),
        material="shuttle_dark",
        name="head",
    )
    shuttle.visual(
        Box((SHUTTLE_STEM_X, SHUTTLE_STEM_Y, SHUTTLE_STEM_Z)),
        origin=Origin(xyz=(0.0, 0.0, SHUTTLE_STEM_CENTER_Z - SHUTTLE_HEAD_CENTER_Z)),
        material="shuttle_dark",
        name="stem",
    )
    shuttle.visual(
        Box((SHUTTLE_BODY_X, SHUTTLE_BODY_Y, SHUTTLE_BODY_Z)),
        origin=Origin(xyz=(0.0, 0.0, SHUTTLE_BODY_CENTER_Z - SHUTTLE_HEAD_CENTER_Z)),
        material="shuttle_dark",
        name="shuttle_body",
    )
    shuttle.visual(
        Box((SHUTTLE_NOSE_X, SHUTTLE_NOSE_Y, SHUTTLE_NOSE_Z)),
        origin=Origin(xyz=(0.0, 0.0, SHUTTLE_NOSE_CENTER_Z - SHUTTLE_HEAD_CENTER_Z)),
        material="shuttle_dark",
        name="nose",
    )
    shuttle.visual(
        mesh_from_cadquery(_shuttle_faceplate_shape(), "shuttle_faceplate"),
        material="accent_blue",
        name="faceplate",
    )

    model.articulation(
        "base_to_bridge",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bridge,
        origin=Origin(xyz=(0.0, 0.0, RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=320.0,
            velocity=0.55,
            lower=-BRIDGE_TRAVEL,
            upper=BRIDGE_TRAVEL,
        ),
    )
    model.articulation(
        "bridge_to_shuttle",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=shuttle,
        origin=Origin(xyz=(0.0, 0.0, SHUTTLE_HEAD_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=160.0,
            velocity=0.45,
            lower=-SHUTTLE_TRAVEL,
            upper=SHUTTLE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    bridge = object_model.get_part("bridge_carriage")
    shuttle = object_model.get_part("center_shuttle")
    bridge_axis = object_model.get_articulation("base_to_bridge")
    shuttle_axis = object_model.get_articulation("bridge_to_shuttle")
    rails = base.get_visual("rails")
    upper_shoe = bridge.get_visual("upper_shoe")
    lower_shoe = bridge.get_visual("lower_shoe")
    upper_upright = bridge.get_visual("upper_upright")
    lower_upright = bridge.get_visual("lower_upright")
    front_guide = bridge.get_visual("front_guide")
    rear_guide = bridge.get_visual("rear_guide")
    roof = bridge.get_visual("roof")
    shuttle_head = shuttle.get_visual("head")
    shuttle_body = shuttle.get_visual("shuttle_body")

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
        bridge,
        base,
        elem_a=upper_shoe,
        elem_b=rails,
        name="upper bridge shoe sits on the twin base rails",
    )
    ctx.expect_contact(
        bridge,
        base,
        elem_a=lower_shoe,
        elem_b=rails,
        name="lower bridge shoe sits on the twin base rails",
    )
    ctx.expect_contact(
        bridge,
        bridge,
        elem_a=upper_upright,
        elem_b=roof,
        name="upper upright ties into the bridge roof",
    )
    ctx.expect_contact(
        bridge,
        bridge,
        elem_a=lower_upright,
        elem_b=roof,
        name="lower upright ties into the bridge roof",
    )
    ctx.expect_contact(
        shuttle,
        bridge,
        elem_a=shuttle_head,
        elem_b=front_guide,
        name="center shuttle head bears on the front bridge guide",
    )
    ctx.expect_contact(
        shuttle,
        bridge,
        elem_a=shuttle_head,
        elem_b=rear_guide,
        name="center shuttle head bears on the rear bridge guide",
    )
    ctx.expect_within(
        shuttle,
        bridge,
        axes="y",
        margin=0.0,
        inner_elem=shuttle_body,
        outer_elem=roof,
        name="center shuttle stays nested within bridge span",
    )

    rest_bridge_pos = ctx.part_world_position(bridge)
    rest_shuttle_pos = ctx.part_world_position(shuttle)

    with ctx.pose({bridge_axis: BRIDGE_TRAVEL}):
        moved_bridge_pos = ctx.part_world_position(bridge)
        bridge_ok = (
            moved_bridge_pos is not None
            and rest_bridge_pos is not None
            and abs((moved_bridge_pos[0] - rest_bridge_pos[0]) - BRIDGE_TRAVEL) <= 0.001
            and abs(moved_bridge_pos[1] - rest_bridge_pos[1]) <= 0.001
            and abs(moved_bridge_pos[2] - rest_bridge_pos[2]) <= 0.001
        )
        ctx.check(
            "bridge slide follows only the base rail direction",
            bridge_ok,
            details=(
                f"rest={rest_bridge_pos} moved={moved_bridge_pos} expected_dx={BRIDGE_TRAVEL}"
            ),
        )

    with ctx.pose({shuttle_axis: SHUTTLE_TRAVEL}):
        moved_shuttle_pos = ctx.part_world_position(shuttle)
        shuttle_ok = (
            moved_shuttle_pos is not None
            and rest_shuttle_pos is not None
            and abs(moved_shuttle_pos[0] - rest_shuttle_pos[0]) <= 0.001
            and abs((moved_shuttle_pos[1] - rest_shuttle_pos[1]) - SHUTTLE_TRAVEL) <= 0.001
            and abs(moved_shuttle_pos[2] - rest_shuttle_pos[2]) <= 0.001
        )
        ctx.check(
            "shuttle slide follows only the bridge direction",
            shuttle_ok,
            details=(
                f"rest={rest_shuttle_pos} moved={moved_shuttle_pos} expected_dy={SHUTTLE_TRAVEL}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
