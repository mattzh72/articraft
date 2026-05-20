from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


FRAME_TOTAL_WIDTH = 1.08
FRAME_TOTAL_HEIGHT = 1.44
FRAME_DEPTH = 0.075
OPENING_WIDTH = 0.96
OPENING_HEIGHT = 1.32
PANEL_HEIGHT = 1.30
PANEL_THICKNESS = 0.032
CENTER_GAP = 0.006
PANEL_WIDTH = (OPENING_WIDTH * 0.5) - (CENTER_GAP * 0.5)
OUTER_STILE_WIDTH = 0.055
MEETING_STILE_WIDTH = 0.060
RAIL_HEIGHT = 0.090
FRAME_MEMBER_OVERLAP = 0.004
LOUVER_COUNT = 8
LOUVER_SIDE_CLEARANCE = 0.0
LOUVER_WIDTH = 0.105
LOUVER_THICKNESS = 0.012
LOUVER_REST_TILT = 0.18


def _x_along_panel(direction_sign: float, distance_from_hinge: float) -> float:
    return direction_sign * distance_from_hinge


def _aabb_center_y(aabb):
    if aabb is None:
        return None
    return 0.5 * (aabb[0][1] + aabb[1][1])


def _aabb_extent(aabb, axis_index: int):
    if aabb is None:
        return None
    return aabb[1][axis_index] - aabb[0][axis_index]


def _add_louver_part(
    model: ArticulatedObject,
    *,
    name: str,
    span: float,
    width: float,
    thickness: float,
    material,
    rest_tilt: float,
):
    louver = model.part(name)
    louver.visual(
        Box((span, thickness, width)),
        origin=Origin(rpy=(rest_tilt, 0.0, 0.0)),
        material=material,
        name="slat_body",
    )
    louver.inertial = Inertial.from_geometry(
        Box((span, thickness, width)),
        mass=0.16,
        origin=Origin(),
    )
    return louver


def _add_panel(
    model: ArticulatedObject,
    *,
    panel_name: str,
    side: str,
    frame_material,
    louver_material,
):
    direction_sign = 1.0 if side == "left" else -1.0
    panel = model.part(panel_name)

    rail_center = (OUTER_STILE_WIDTH + PANEL_WIDTH - MEETING_STILE_WIDTH) * 0.5
    rail_length = (
        PANEL_WIDTH
        - OUTER_STILE_WIDTH
        - MEETING_STILE_WIDTH
        + (2.0 * FRAME_MEMBER_OVERLAP)
    )

    panel.visual(
        Box((OUTER_STILE_WIDTH, PANEL_THICKNESS, PANEL_HEIGHT)),
        origin=Origin(xyz=(_x_along_panel(direction_sign, OUTER_STILE_WIDTH * 0.5), 0.0, 0.0)),
        material=frame_material,
        name="outer_stile",
    )
    panel.visual(
        Box((MEETING_STILE_WIDTH, PANEL_THICKNESS, PANEL_HEIGHT)),
        origin=Origin(
            xyz=(
                _x_along_panel(direction_sign, PANEL_WIDTH - (MEETING_STILE_WIDTH * 0.5)),
                0.0,
                0.0,
            )
        ),
        material=frame_material,
        name="meeting_stile",
    )
    panel.visual(
        Box((rail_length, PANEL_THICKNESS, RAIL_HEIGHT)),
        origin=Origin(
            xyz=(
                _x_along_panel(direction_sign, rail_center),
                0.0,
                (PANEL_HEIGHT * 0.5) - (RAIL_HEIGHT * 0.5),
            )
        ),
        material=frame_material,
        name="top_rail",
    )
    panel.visual(
        Box((rail_length, PANEL_THICKNESS, RAIL_HEIGHT)),
        origin=Origin(
            xyz=(
                _x_along_panel(direction_sign, rail_center),
                0.0,
                -(PANEL_HEIGHT * 0.5) + (RAIL_HEIGHT * 0.5),
            )
        ),
        material=frame_material,
        name="bottom_rail",
    )
    panel.inertial = Inertial.from_geometry(
        Box((PANEL_WIDTH, PANEL_THICKNESS, PANEL_HEIGHT)),
        mass=4.2,
        origin=Origin(xyz=(_x_along_panel(direction_sign, PANEL_WIDTH * 0.5), 0.0, 0.0)),
    )

    louver_zone_height = PANEL_HEIGHT - (2.0 * RAIL_HEIGHT)
    louver_pitch = louver_zone_height / LOUVER_COUNT
    louver_span = (
        PANEL_WIDTH
        - OUTER_STILE_WIDTH
        - MEETING_STILE_WIDTH
        - (2.0 * LOUVER_SIDE_CLEARANCE)
    )
    louver_center_x = OUTER_STILE_WIDTH + LOUVER_SIDE_CLEARANCE + (louver_span * 0.5)
    top_zone_z = (PANEL_HEIGHT * 0.5) - RAIL_HEIGHT

    louver_parts = []
    louver_joint_names = []
    for index in range(LOUVER_COUNT):
        louver_name = f"{side}_louver_{index:02d}"
        louver = _add_louver_part(
            model,
            name=louver_name,
            span=louver_span,
            width=LOUVER_WIDTH,
            thickness=LOUVER_THICKNESS,
            material=louver_material,
            rest_tilt=LOUVER_REST_TILT,
        )
        louver_parts.append(louver)
        louver_z = top_zone_z - (louver_pitch * (index + 0.5))
        joint_name = f"{panel_name}_to_{louver_name}"
        louver_joint_names.append(joint_name)
        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=panel,
            child=louver,
            origin=Origin(
                xyz=(
                    _x_along_panel(direction_sign, louver_center_x),
                    0.0,
                    louver_z,
                )
            ),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.8,
                velocity=2.5,
                lower=-0.85,
                upper=0.85,
            ),
        )

    return panel, louver_parts, louver_joint_names


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="louvered_shutter_assembly")

    frame_paint = model.material("frame_paint", rgba=(0.95, 0.95, 0.93, 1.0))
    panel_paint = model.material("panel_paint", rgba=(0.97, 0.97, 0.95, 1.0))
    louver_paint = model.material("louver_paint", rgba=(0.93, 0.94, 0.92, 1.0))
    opening_shadow = model.material("opening_shadow", rgba=(0.18, 0.19, 0.20, 1.0))

    opening_frame = model.part("opening_frame")
    jamb_width = (FRAME_TOTAL_WIDTH - OPENING_WIDTH) * 0.5
    head_height = (FRAME_TOTAL_HEIGHT - OPENING_HEIGHT) * 0.5

    opening_frame.visual(
        Box((jamb_width, FRAME_DEPTH, FRAME_TOTAL_HEIGHT)),
        origin=Origin(xyz=(-(OPENING_WIDTH * 0.5) - (jamb_width * 0.5), 0.0, 0.0)),
        material=frame_paint,
        name="left_jamb",
    )
    opening_frame.visual(
        Box((jamb_width, FRAME_DEPTH, FRAME_TOTAL_HEIGHT)),
        origin=Origin(xyz=((OPENING_WIDTH * 0.5) + (jamb_width * 0.5), 0.0, 0.0)),
        material=frame_paint,
        name="right_jamb",
    )
    opening_frame.visual(
        Box((FRAME_TOTAL_WIDTH, FRAME_DEPTH, head_height)),
        origin=Origin(xyz=(0.0, 0.0, (OPENING_HEIGHT * 0.5) + (head_height * 0.5))),
        material=frame_paint,
        name="head",
    )
    opening_frame.visual(
        Box((FRAME_TOTAL_WIDTH, FRAME_DEPTH, head_height)),
        origin=Origin(xyz=(0.0, 0.0, -(OPENING_HEIGHT * 0.5) - (head_height * 0.5))),
        material=frame_paint,
        name="sill",
    )
    opening_frame.inertial = Inertial.from_geometry(
        Box((FRAME_TOTAL_WIDTH, FRAME_DEPTH, FRAME_TOTAL_HEIGHT)),
        mass=14.0,
        origin=Origin(),
    )

    left_panel, _, _ = _add_panel(
        model,
        panel_name="left_panel",
        side="left",
        frame_material=panel_paint,
        louver_material=louver_paint,
    )
    right_panel, _, _ = _add_panel(
        model,
        panel_name="right_panel",
        side="right",
        frame_material=panel_paint,
        louver_material=louver_paint,
    )

    model.articulation(
        "opening_frame_to_left_panel",
        ArticulationType.REVOLUTE,
        parent=opening_frame,
        child=left_panel,
        origin=Origin(xyz=(-(OPENING_WIDTH * 0.5), 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.4,
            lower=0.0,
            upper=1.75,
        ),
    )
    model.articulation(
        "opening_frame_to_right_panel",
        ArticulationType.REVOLUTE,
        parent=opening_frame,
        child=right_panel,
        origin=Origin(xyz=((OPENING_WIDTH * 0.5), 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.4,
            lower=0.0,
            upper=1.75,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    def require_part(name: str):
        try:
            part = object_model.get_part(name)
        except Exception as exc:  # pragma: no cover - defensive authored test helper
            ctx.fail(f"{name} exists", str(exc))
            return None
        ctx.check(f"{name} exists", True)
        return part

    def require_joint(name: str):
        try:
            articulation = object_model.get_articulation(name)
        except Exception as exc:  # pragma: no cover - defensive authored test helper
            ctx.fail(f"{name} exists", str(exc))
            return None
        ctx.check(f"{name} exists", True)
        return articulation

    required_part_names = [
        "opening_frame",
        "left_panel",
        "right_panel",
        *[f"left_louver_{index:02d}" for index in range(LOUVER_COUNT)],
        *[f"right_louver_{index:02d}" for index in range(LOUVER_COUNT)],
    ]
    required_joint_names = [
        "opening_frame_to_left_panel",
        "opening_frame_to_right_panel",
        *[f"left_panel_to_left_louver_{index:02d}" for index in range(LOUVER_COUNT)],
        *[f"right_panel_to_right_louver_{index:02d}" for index in range(LOUVER_COUNT)],
    ]

    resolved_parts = {name: require_part(name) for name in required_part_names}
    resolved_joints = {name: require_joint(name) for name in required_joint_names}

    opening_frame = resolved_parts["opening_frame"]
    left_panel = resolved_parts["left_panel"]
    right_panel = resolved_parts["right_panel"]
    left_mid_louver = resolved_parts["left_louver_03"]
    right_mid_louver = resolved_parts["right_louver_03"]
    left_hinge = resolved_joints["opening_frame_to_left_panel"]
    right_hinge = resolved_joints["opening_frame_to_right_panel"]
    left_mid_joint = resolved_joints["left_panel_to_left_louver_03"]
    right_mid_joint = resolved_joints["right_panel_to_right_louver_03"]

    critical_items = [
        opening_frame,
        left_panel,
        right_panel,
        left_mid_louver,
        right_mid_louver,
        left_hinge,
        right_hinge,
        left_mid_joint,
        right_mid_joint,
    ]
    if any(item is None for item in critical_items):
        return ctx.report()

    ctx.expect_contact(
        left_panel,
        opening_frame,
        elem_a="outer_stile",
        elem_b="left_jamb",
        name="left panel outer stile mounts to left jamb",
    )
    ctx.expect_contact(
        right_panel,
        opening_frame,
        elem_a="outer_stile",
        elem_b="right_jamb",
        name="right panel outer stile mounts to right jamb",
    )
    ctx.expect_gap(
        right_panel,
        left_panel,
        axis="x",
        positive_elem="meeting_stile",
        negative_elem="meeting_stile",
        min_gap=0.0,
        max_gap=0.010,
        name="meeting stiles close with a narrow center gap",
    )

    for index in range(LOUVER_COUNT):
        left_louver = resolved_parts[f"left_louver_{index:02d}"]
        right_louver = resolved_parts[f"right_louver_{index:02d}"]
        if left_louver is not None:
            ctx.expect_within(
                left_louver,
                left_panel,
                axes="xz",
                margin=0.002,
                name=f"left louver {index:02d} stays within left panel frame",
            )
        if right_louver is not None:
            ctx.expect_within(
                right_louver,
                right_panel,
                axes="xz",
                margin=0.002,
                name=f"right louver {index:02d} stays within right panel frame",
            )

    left_meeting_rest = ctx.part_element_world_aabb(left_panel, elem="meeting_stile")
    right_meeting_rest = ctx.part_element_world_aabb(right_panel, elem="meeting_stile")
    left_louver_rest = ctx.part_world_aabb(left_mid_louver)
    right_louver_rest = ctx.part_world_aabb(right_mid_louver)

    with ctx.pose({left_hinge: 1.15, right_hinge: 1.15}):
        left_meeting_open = ctx.part_element_world_aabb(left_panel, elem="meeting_stile")
        right_meeting_open = ctx.part_element_world_aabb(right_panel, elem="meeting_stile")

    ctx.check(
        "both panels swing outward from the opening",
        (
            _aabb_center_y(left_meeting_rest) is not None
            and _aabb_center_y(right_meeting_rest) is not None
            and _aabb_center_y(left_meeting_open) is not None
            and _aabb_center_y(right_meeting_open) is not None
            and _aabb_center_y(left_meeting_open) > _aabb_center_y(left_meeting_rest) + 0.16
            and _aabb_center_y(right_meeting_open) > _aabb_center_y(right_meeting_rest) + 0.16
        ),
        details=(
            f"left_rest_y={_aabb_center_y(left_meeting_rest)}, "
            f"left_open_y={_aabb_center_y(left_meeting_open)}, "
            f"right_rest_y={_aabb_center_y(right_meeting_rest)}, "
            f"right_open_y={_aabb_center_y(right_meeting_open)}"
        ),
    )

    with ctx.pose({left_mid_joint: 0.72, right_mid_joint: -0.72}):
        left_louver_tilted = ctx.part_world_aabb(left_mid_louver)
        right_louver_tilted = ctx.part_world_aabb(right_mid_louver)
        ctx.expect_within(
            left_mid_louver,
            left_panel,
            axes="xz",
            margin=0.002,
            name="representative left louver stays captured when tilted",
        )
        ctx.expect_within(
            right_mid_louver,
            right_panel,
            axes="xz",
            margin=0.002,
            name="representative right louver stays captured when tilted",
        )

    ctx.check(
        "representative louvers rotate about their long axes",
        (
            _aabb_extent(left_louver_rest, 1) is not None
            and _aabb_extent(right_louver_rest, 1) is not None
            and _aabb_extent(left_louver_tilted, 1) is not None
            and _aabb_extent(right_louver_tilted, 1) is not None
            and _aabb_extent(left_louver_tilted, 1) > _aabb_extent(left_louver_rest, 1) + 0.025
            and _aabb_extent(right_louver_tilted, 1) > _aabb_extent(right_louver_rest, 1) + 0.025
        ),
        details=(
            f"left_rest_dy={_aabb_extent(left_louver_rest, 1)}, "
            f"left_tilted_dy={_aabb_extent(left_louver_tilted, 1)}, "
            f"right_rest_dy={_aabb_extent(right_louver_rest, 1)}, "
            f"right_tilted_dy={_aabb_extent(right_louver_tilted, 1)}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
