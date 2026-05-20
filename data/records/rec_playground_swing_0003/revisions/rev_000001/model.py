from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)

BEAM_Z = 2.18
BEAM_RADIUS = 0.06
BEAM_HALF_SPAN = 1.18
FRAME_HALF_DEPTH = 0.56
PIVOT_Z = 2.08
TIRE_CENTER_DROP = 1.26
TIRE_OUTER_RADIUS = 0.285
TIRE_INNER_RADIUS = 0.155
TIRE_HALF_WIDTH = 0.060
SUPPORT_XS = (-0.20, 0.0, 0.20)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _segment_pose(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[Origin, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    center = (
        0.5 * (start[0] + end[0]),
        0.5 * (start[1] + end[1]),
        0.5 * (start[2] + end[2]),
    )
    return Origin(xyz=center, rpy=(0.0, pitch, yaw)), length


def _extended_segment(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    embed: float,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    ux = dx / length
    uy = dy / length
    uz = dz / length
    return (
        (start[0] - ux * embed, start[1] - uy * embed, start[2] - uz * embed),
        (end[0] + ux * embed, end[1] + uy * embed, end[2] + uz * embed),
    )


def _add_segment_visual(
    part,
    name: str,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    radius: float,
    material,
) -> None:
    origin, length = _segment_pose(start, end)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=origin,
        material=material,
        name=name,
    )


def _build_tire_geometry() -> LatheGeometry:
    profile = [
        (TIRE_INNER_RADIUS + 0.010, -TIRE_HALF_WIDTH * 0.92),
        (TIRE_INNER_RADIUS + 0.050, -TIRE_HALF_WIDTH * 0.98),
        (TIRE_OUTER_RADIUS - 0.050, -TIRE_HALF_WIDTH * 0.94),
        (TIRE_OUTER_RADIUS - 0.016, -TIRE_HALF_WIDTH * 0.66),
        (TIRE_OUTER_RADIUS, -TIRE_HALF_WIDTH * 0.18),
        (TIRE_OUTER_RADIUS, TIRE_HALF_WIDTH * 0.18),
        (TIRE_OUTER_RADIUS - 0.016, TIRE_HALF_WIDTH * 0.66),
        (TIRE_OUTER_RADIUS - 0.050, TIRE_HALF_WIDTH * 0.94),
        (TIRE_INNER_RADIUS + 0.050, TIRE_HALF_WIDTH * 0.98),
        (TIRE_INNER_RADIUS + 0.010, TIRE_HALF_WIDTH * 0.92),
        (TIRE_INNER_RADIUS, TIRE_HALF_WIDTH * 0.32),
        (TIRE_INNER_RADIUS - 0.004, 0.0),
        (TIRE_INNER_RADIUS, -TIRE_HALF_WIDTH * 0.32),
    ]
    return LatheGeometry(profile, segments=72)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tire_playground_swing", assets=ASSETS)

    galvanized = model.material("galvanized_steel", rgba=(0.66, 0.69, 0.72, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.22, 0.23, 0.25, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    bushing = model.material("bushing_black", rgba=(0.06, 0.06, 0.07, 1.0))

    frame = model.part("gantry_frame")
    frame.inertial = Inertial.from_geometry(
        Box((2.60, 1.20, 2.30)),
        mass=96.0,
        origin=Origin(xyz=(0.0, 0.0, 1.15)),
    )

    _add_segment_visual(
        frame,
        "top_beam",
        (-BEAM_HALF_SPAN, 0.0, BEAM_Z),
        (BEAM_HALF_SPAN, 0.0, BEAM_Z),
        radius=BEAM_RADIUS,
        material=galvanized,
    )

    left_top = (-BEAM_HALF_SPAN + 0.02, 0.0, BEAM_Z)
    right_top = (BEAM_HALF_SPAN - 0.02, 0.0, BEAM_Z)
    left_front_foot = (-BEAM_HALF_SPAN + 0.02, FRAME_HALF_DEPTH, 0.08)
    left_rear_foot = (-BEAM_HALF_SPAN + 0.02, -FRAME_HALF_DEPTH, 0.08)
    right_front_foot = (BEAM_HALF_SPAN - 0.02, FRAME_HALF_DEPTH, 0.08)
    right_rear_foot = (BEAM_HALF_SPAN - 0.02, -FRAME_HALF_DEPTH, 0.08)

    for name, start, end in (
        ("left_leg_front", left_top, left_front_foot),
        ("left_leg_rear", left_top, left_rear_foot),
        ("right_leg_front", right_top, right_front_foot),
        ("right_leg_rear", right_top, right_rear_foot),
    ):
        start_pt, end_pt = _extended_segment(start, end, 0.020)
        _add_segment_visual(
            frame,
            name,
            start_pt,
            end_pt,
            radius=0.045,
            material=galvanized,
        )

    frame.visual(
        Box((0.12, 1.18, 0.08)),
        origin=Origin(xyz=(left_top[0], 0.0, 0.04)),
        material=dark_metal,
        name="left_ground_skid",
    )
    frame.visual(
        Box((0.12, 1.18, 0.08)),
        origin=Origin(xyz=(right_top[0], 0.0, 0.04)),
        material=dark_metal,
        name="right_ground_skid",
    )
    frame.visual(
        Box((0.06, 0.72, 0.06)),
        origin=Origin(xyz=(left_top[0], 0.0, 1.08)),
        material=galvanized,
        name="left_side_brace",
    )
    frame.visual(
        Box((0.06, 0.72, 0.06)),
        origin=Origin(xyz=(right_top[0], 0.0, 1.08)),
        material=galvanized,
        name="right_side_brace",
    )

    for support_name, support_x in zip(("left", "center", "right"), SUPPORT_XS):
        for side_name, side_y in (("front", 0.020), ("rear", -0.020)):
            frame.visual(
                Box((0.032, 0.008, 0.090)),
                origin=Origin(
                    xyz=(
                        support_x,
                        side_y,
                        PIVOT_Z + 0.025,
                    )
                ),
                material=dark_metal,
                name=f"pivot_bracket_{support_name}_{side_name}",
            )

    swing = model.part("swing_assembly")
    swing.inertial = Inertial.from_geometry(
        Box((0.62, 0.62, 1.42)),
        mass=19.0,
        origin=Origin(xyz=(0.0, 0.0, -0.71)),
    )

    tire_mesh = _save_mesh("playground_tire.obj", _build_tire_geometry())
    swing.visual(
        tire_mesh,
        origin=Origin(xyz=(0.0, 0.0, -TIRE_CENTER_DROP)),
        material=rubber,
        name="tire_shell",
    )

    lower_anchors = {
        "center": (0.0, 0.215, -TIRE_CENTER_DROP + 0.060),
        "left": (-0.182, -0.105, -TIRE_CENTER_DROP + 0.058),
        "right": (0.182, -0.105, -TIRE_CENTER_DROP + 0.058),
    }
    top_supports = {
        "left": (SUPPORT_XS[0], 0.0, 0.0),
        "center": (SUPPORT_XS[1], 0.0, 0.0),
        "right": (SUPPORT_XS[2], 0.0, 0.0),
    }

    for support_name in ("left", "center", "right"):
        top_pt = top_supports[support_name]
        lower_pt = lower_anchors[support_name]
        swing.visual(
            Box((0.012, 0.016, 0.090)),
            origin=Origin(
                xyz=(top_pt[0], 0.0, -0.040),
            ),
            material=dark_metal,
            name=f"hanger_head_{support_name}",
        )
        swing.visual(
            Box((0.052, 0.028, 0.026)),
            origin=Origin(
                xyz=(lower_pt[0], lower_pt[1], lower_pt[2] - 0.004),
                rpy=(0.0, 0.0, 0.0),
            ),
            material=dark_metal,
            name=f"tire_anchor_{support_name}",
        )
        rod_start = (top_pt[0], 0.0, -0.075)
        link_dx = lower_pt[0] - rod_start[0]
        link_dy = lower_pt[1] - rod_start[1]
        link_dz = lower_pt[2] - rod_start[2]
        link_length = math.sqrt(link_dx * link_dx + link_dy * link_dy + link_dz * link_dz)
        rod_end = (
            lower_pt[0] + 0.012 * link_dx / link_length,
            lower_pt[1] + 0.012 * link_dy / link_length,
            lower_pt[2] + 0.012 * link_dz / link_length,
        )
        _add_segment_visual(
            swing,
            f"hanger_link_{support_name}",
            rod_start,
            rod_end,
            radius=0.013,
            material=galvanized,
        )

    # The real swing uses three parallel top pivots on the same beam axis.
    # The SDK articulation graph cannot encode one rigid body with three parents,
    # so the three physical pivots are modeled visually while one equivalent
    # revolute articulation carries the shared pendulum motion.
    model.articulation(
        "swing_pivot",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=swing,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=650.0,
            velocity=1.4,
            lower=-math.radians(35.0),
            upper=math.radians(35.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    frame = object_model.get_part("gantry_frame")
    swing = object_model.get_part("swing_assembly")
    swing_pivot = object_model.get_articulation("swing_pivot")

    top_beam = frame.get_visual("top_beam")
    pivot_bracket_left_front = frame.get_visual("pivot_bracket_left_front")
    pivot_bracket_left_rear = frame.get_visual("pivot_bracket_left_rear")
    pivot_bracket_center_front = frame.get_visual("pivot_bracket_center_front")
    pivot_bracket_center_rear = frame.get_visual("pivot_bracket_center_rear")
    pivot_bracket_right_front = frame.get_visual("pivot_bracket_right_front")
    pivot_bracket_right_rear = frame.get_visual("pivot_bracket_right_rear")

    tire_shell = swing.get_visual("tire_shell")
    hanger_link_left = swing.get_visual("hanger_link_left")
    hanger_link_center = swing.get_visual("hanger_link_center")
    hanger_link_right = swing.get_visual("hanger_link_right")
    hanger_head_left = swing.get_visual("hanger_head_left")
    hanger_head_center = swing.get_visual("hanger_head_center")
    hanger_head_right = swing.get_visual("hanger_head_right")

    ctx.check(
        "parts_present",
        frame is not None and swing is not None and swing_pivot is not None,
        "Expected gantry frame, swing assembly, and pendulum articulation.",
    )
    ctx.check(
        "key_visuals_present",
        all(
            visual is not None
            for visual in (
                top_beam,
                tire_shell,
                hanger_link_left,
                hanger_link_center,
                hanger_link_right,
                hanger_head_left,
                hanger_head_center,
                hanger_head_right,
            )
        ),
        "Expected top beam, tire, clevis-mounted hanger heads, and all three rigid hangers.",
    )
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.allow_isolated_part(
        frame,
        reason="Frame and swing are connected by a working-clearance revolute pivot; the articulation encodes the mechanical support.",
    )
    ctx.allow_isolated_part(
        swing,
        reason="Swing hangs from a working-clearance revolute pivot; small modeled pivot clearance avoids false contact/overlap.",
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

    ctx.expect_gap(
        frame,
        swing,
        axis="y",
        positive_elem=pivot_bracket_left_front,
        negative_elem=hanger_head_left,
        min_gap=0.006,
        max_gap=0.011,
        name="left_hanger_head_clear_of_front_clevis_plate",
    )
    ctx.expect_gap(
        swing,
        frame,
        axis="y",
        positive_elem=hanger_head_left,
        negative_elem=pivot_bracket_left_rear,
        min_gap=0.006,
        max_gap=0.011,
        name="left_hanger_head_clear_of_rear_clevis_plate",
    )
    ctx.expect_gap(
        frame,
        swing,
        axis="y",
        positive_elem=pivot_bracket_center_front,
        negative_elem=hanger_head_center,
        min_gap=0.006,
        max_gap=0.011,
        name="center_hanger_head_clear_of_front_clevis_plate",
    )
    ctx.expect_gap(
        swing,
        frame,
        axis="y",
        positive_elem=hanger_head_center,
        negative_elem=pivot_bracket_center_rear,
        min_gap=0.006,
        max_gap=0.011,
        name="center_hanger_head_clear_of_rear_clevis_plate",
    )
    ctx.expect_gap(
        frame,
        swing,
        axis="y",
        positive_elem=pivot_bracket_right_front,
        negative_elem=hanger_head_right,
        min_gap=0.006,
        max_gap=0.011,
        name="right_hanger_head_clear_of_front_clevis_plate",
    )
    ctx.expect_gap(
        swing,
        frame,
        axis="y",
        positive_elem=hanger_head_right,
        negative_elem=pivot_bracket_right_rear,
        min_gap=0.006,
        max_gap=0.011,
        name="right_hanger_head_clear_of_rear_clevis_plate",
    )
    ctx.expect_gap(
        frame,
        swing,
        axis="z",
        positive_elem=top_beam,
        negative_elem=tire_shell,
        min_gap=1.15,
        max_gap=1.36,
        name="tire_hangs_well_below_top_beam",
    )
    ctx.expect_overlap(
        frame,
        swing,
        axes="x",
        elem_a=top_beam,
        elem_b=tire_shell,
        min_overlap=0.50,
        name="tire_stays_under_beam_span",
    )

    with ctx.pose({swing_pivot: math.radians(35.0)}):
        ctx.expect_gap(
            swing,
            frame,
            axis="y",
            positive_elem=tire_shell,
            negative_elem=top_beam,
            min_gap=0.35,
            max_gap=1.05,
            name="positive_pose_swings_tire_forward",
        )
        ctx.expect_gap(
            frame,
            swing,
            axis="z",
            positive_elem=top_beam,
            negative_elem=tire_shell,
            min_gap=0.84,
            max_gap=1.28,
            name="positive_pose_keeps_tire_clear_of_beam",
        )

    with ctx.pose({swing_pivot: -math.radians(35.0)}):
        ctx.expect_gap(
            frame,
            swing,
            axis="y",
            positive_elem=top_beam,
            negative_elem=tire_shell,
            min_gap=0.35,
            max_gap=1.05,
            name="negative_pose_swings_tire_backward",
        )
        ctx.expect_gap(
            frame,
            swing,
            axis="z",
            positive_elem=top_beam,
            negative_elem=tire_shell,
            min_gap=0.84,
            max_gap=1.28,
            name="negative_pose_keeps_tire_clear_of_beam",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
