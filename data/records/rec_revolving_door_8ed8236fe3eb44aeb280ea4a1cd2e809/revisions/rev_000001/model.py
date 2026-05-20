from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


def _circle_profile(radius: float, *, segments: int = 72) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * i) / segments),
            radius * math.sin((2.0 * math.pi * i) / segments),
        )
        for i in range(segments)
    ]


def _sector_loop(
    *,
    inner_radius: float,
    outer_radius: float,
    start_angle: float,
    end_angle: float,
    z: float,
    samples: int = 20,
) -> list[tuple[float, float, float]]:
    outer = []
    inner = []
    for i in range(samples + 1):
        t = i / samples
        angle = start_angle + (end_angle - start_angle) * t
        outer.append((outer_radius * math.cos(angle), outer_radius * math.sin(angle), z))
    for i in range(samples + 1):
        t = i / samples
        angle = end_angle + (start_angle - end_angle) * t
        inner.append((inner_radius * math.cos(angle), inner_radius * math.sin(angle), z))
    return outer + inner


def _ring_mesh(*, inner_radius: float, outer_radius: float, height: float):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius, segments=80),
            [_circle_profile(inner_radius, segments=80)],
            height,
            center=True,
        ),
        f"ring_{int(outer_radius * 1000)}_{int(inner_radius * 1000)}_{int(height * 1000)}",
    )


def _sector_wall_mesh(
    *,
    inner_radius: float,
    outer_radius: float,
    start_angle_deg: float,
    end_angle_deg: float,
    height: float,
    samples: int = 24,
    name: str,
):
    start = math.radians(start_angle_deg)
    end = math.radians(end_angle_deg)
    lower = _sector_loop(
        inner_radius=inner_radius,
        outer_radius=outer_radius,
        start_angle=start,
        end_angle=end,
        z=0.0,
        samples=samples,
    )
    upper = _sector_loop(
        inner_radius=inner_radius,
        outer_radius=outer_radius,
        start_angle=start,
        end_angle=end,
        z=height,
        samples=samples,
    )
    return mesh_from_geometry(section_loft([lower, upper]), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="revolving_door_with_bypass_panel")

    aluminum = model.material("aluminum", rgba=(0.52, 0.54, 0.57, 1.0))
    stainless = model.material("stainless", rgba=(0.68, 0.70, 0.73, 1.0))
    dark_track = model.material("dark_track", rgba=(0.19, 0.20, 0.22, 1.0))
    glass = model.material("glass", rgba=(0.69, 0.84, 0.93, 0.28))
    floor_stone = model.material("floor_stone", rgba=(0.58, 0.58, 0.60, 1.0))

    frame = model.part("entrance_frame")

    floor_top = 0.06
    canopy_bottom = 2.52
    canopy_top = 2.70
    clear_height = canopy_bottom - floor_top

    frame.visual(
        Box((5.92, 2.90, 0.06)),
        origin=Origin(xyz=(1.28, 0.0, 0.03)),
        material=floor_stone,
        name="floor_plinth",
    )
    frame.visual(
        Box((5.92, 2.90, canopy_top - canopy_bottom)),
        origin=Origin(xyz=(1.28, 0.0, (canopy_bottom + canopy_top) / 2.0)),
        material=aluminum,
        name="ceiling_canopy",
    )

    for name, x_pos in (
        ("left_outer_post", -1.62),
        ("bypass_left_jamb", 1.58),
        ("bypass_right_post", 4.18),
    ):
        frame.visual(
            Box((0.12, 0.24, clear_height)),
            origin=Origin(xyz=(x_pos, 0.0, floor_top + clear_height / 2.0)),
            material=aluminum,
            name=name,
        )

    frame.visual(
        _ring_mesh(inner_radius=1.06, outer_radius=1.28, height=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=dark_track,
        name="drum_floor_ring",
    )
    frame.visual(
        _ring_mesh(inner_radius=1.06, outer_radius=1.28, height=0.16),
        origin=Origin(xyz=(0.0, 0.0, 2.44)),
        material=dark_track,
        name="drum_header_ring",
    )
    frame.visual(
        _sector_wall_mesh(
            inner_radius=1.16,
            outer_radius=1.20,
            start_angle_deg=-55.0,
            end_angle_deg=55.0,
            height=2.24,
            samples=28,
            name="east_drum_glass_mesh",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=glass,
        name="east_drum_glass",
    )
    frame.visual(
        _sector_wall_mesh(
            inner_radius=1.16,
            outer_radius=1.20,
            start_angle_deg=125.0,
            end_angle_deg=235.0,
            height=2.24,
            samples=28,
            name="west_drum_glass_mesh",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=glass,
        name="west_drum_glass",
    )

    frame.visual(
        Box((2.48, 0.18, 0.18)),
        origin=Origin(xyz=(2.88, 0.0, 2.43)),
        material=dark_track,
        name="bypass_header_track",
    )
    frame.visual(
        Box((2.48, 0.02, 0.04)),
        origin=Origin(xyz=(2.88, 0.045, 0.08)),
        material=dark_track,
        name="bypass_floor_guide_front",
    )
    frame.visual(
        Box((2.48, 0.02, 0.04)),
        origin=Origin(xyz=(2.88, -0.045, 0.08)),
        material=dark_track,
        name="bypass_floor_guide_rear",
    )

    rotor = model.part("drum_rotor")
    rotor.visual(
        Cylinder(radius=0.05, length=2.28),
        origin=Origin(xyz=(0.0, 0.0, 1.14)),
        material=stainless,
        name="shaft",
    )
    rotor.visual(
        Cylinder(radius=0.16, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=dark_track,
        name="lower_hub",
    )
    rotor.visual(
        Cylinder(radius=0.16, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 2.17)),
        material=dark_track,
        name="upper_hub",
    )
    rotor.visual(
        Box((1.08, 0.032, 2.20)),
        origin=Origin(xyz=(0.54, 0.0, 1.18)),
        material=glass,
        name="wing_pos_glass",
    )
    rotor.visual(
        Box((1.08, 0.032, 2.20)),
        origin=Origin(xyz=(-0.54, 0.0, 1.18)),
        material=glass,
        name="wing_neg_glass",
    )
    rotor.visual(
        Box((0.05, 0.06, 2.22)),
        origin=Origin(xyz=(1.055, 0.0, 1.18)),
        material=aluminum,
        name="wing_pos_edge",
    )
    rotor.visual(
        Box((0.05, 0.06, 2.22)),
        origin=Origin(xyz=(-1.055, 0.0, 1.18)),
        material=aluminum,
        name="wing_neg_edge",
    )
    rotor.visual(
        Box((1.02, 0.05, 0.05)),
        origin=Origin(xyz=(0.51, 0.0, 0.12)),
        material=aluminum,
        name="wing_pos_bottom_rail",
    )
    rotor.visual(
        Box((1.02, 0.05, 0.05)),
        origin=Origin(xyz=(0.51, 0.0, 2.24)),
        material=aluminum,
        name="wing_pos_top_rail",
    )
    rotor.visual(
        Box((1.02, 0.05, 0.05)),
        origin=Origin(xyz=(-0.51, 0.0, 0.12)),
        material=aluminum,
        name="wing_neg_bottom_rail",
    )
    rotor.visual(
        Box((1.02, 0.05, 0.05)),
        origin=Origin(xyz=(-0.51, 0.0, 2.24)),
        material=aluminum,
        name="wing_neg_top_rail",
    )

    bypass_panel = model.part("bypass_panel")
    bypass_panel.visual(
        Box((1.20, 0.024, 2.18)),
        origin=Origin(xyz=(0.0, 0.0, 1.15)),
        material=glass,
        name="panel_glass",
    )
    bypass_panel.visual(
        Box((0.06, 0.05, 2.24)),
        origin=Origin(xyz=(-0.57, 0.0, 1.15)),
        material=aluminum,
        name="panel_left_stile",
    )
    bypass_panel.visual(
        Box((0.06, 0.05, 2.24)),
        origin=Origin(xyz=(0.57, 0.0, 1.15)),
        material=aluminum,
        name="panel_right_stile",
    )
    bypass_panel.visual(
        Box((1.20, 0.05, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=aluminum,
        name="panel_bottom_rail",
    )
    bypass_panel.visual(
        Box((1.20, 0.05, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 2.19)),
        material=aluminum,
        name="panel_top_rail",
    )
    bypass_panel.visual(
        Box((0.10, 0.07, 0.10)),
        origin=Origin(xyz=(-0.36, 0.0, 2.23)),
        material=dark_track,
        name="panel_left_top_guide",
    )
    bypass_panel.visual(
        Box((0.10, 0.07, 0.10)),
        origin=Origin(xyz=(0.36, 0.0, 2.23)),
        material=dark_track,
        name="panel_right_top_guide",
    )
    bypass_panel.visual(
        Box((0.12, 0.03, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=dark_track,
        name="panel_bottom_guide",
    )

    model.articulation(
        "frame_to_drum_rotor",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, floor_top)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.6),
    )
    model.articulation(
        "frame_to_bypass_panel",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=bypass_panel,
        origin=Origin(xyz=(2.27, 0.0, floor_top)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.9,
            lower=0.0,
            upper=1.22,
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

    frame = object_model.get_part("entrance_frame")
    rotor = object_model.get_part("drum_rotor")
    bypass_panel = object_model.get_part("bypass_panel")
    drum_joint = object_model.get_articulation("frame_to_drum_rotor")
    bypass_joint = object_model.get_articulation("frame_to_bypass_panel")

    ctx.check(
        "rotor uses continuous vertical articulation",
        drum_joint.articulation_type == ArticulationType.CONTINUOUS and drum_joint.axis == (0.0, 0.0, 1.0),
        details=f"type={drum_joint.articulation_type}, axis={drum_joint.axis}",
    )
    bypass_limits = bypass_joint.motion_limits
    ctx.check(
        "bypass panel uses rightward prismatic travel",
        bypass_joint.articulation_type == ArticulationType.PRISMATIC
        and bypass_joint.axis == (1.0, 0.0, 0.0)
        and bypass_limits is not None
        and bypass_limits.lower == 0.0
        and bypass_limits.upper is not None
        and bypass_limits.upper > 1.0,
        details=(
            f"type={bypass_joint.articulation_type}, axis={bypass_joint.axis}, "
            f"limits={None if bypass_limits is None else (bypass_limits.lower, bypass_limits.upper)}"
        ),
    )

    with ctx.pose({drum_joint: 0.0, bypass_joint: 0.0}):
        ctx.expect_gap(
            rotor,
            frame,
            axis="z",
            positive_elem="wing_pos_glass",
            negative_elem="drum_floor_ring",
            min_gap=0.015,
            max_gap=0.06,
            name="rotor wing clears drum floor ring",
        )
        ctx.expect_gap(
            frame,
            rotor,
            axis="z",
            positive_elem="drum_header_ring",
            negative_elem="wing_pos_glass",
            min_gap=0.015,
            max_gap=0.06,
            name="rotor wing clears drum header ring",
        )

    slide_upper = 1.22
    for label, q in (("closed", 0.0), ("open", slide_upper)):
        with ctx.pose({bypass_joint: q}):
            ctx.expect_gap(
                frame,
                bypass_panel,
                axis="z",
                positive_elem="bypass_header_track",
                negative_elem="panel_left_top_guide",
                min_gap=0.0,
                max_gap=0.12,
                name=f"bypass panel stays below header track ({label})",
            )
            ctx.expect_overlap(
                bypass_panel,
                frame,
                axes="x",
                elem_a="panel_top_rail",
                elem_b="bypass_header_track",
                min_overlap=0.40,
                name=f"bypass panel remains under header span ({label})",
            )

    closed_panel_pos = ctx.part_world_position(bypass_panel)
    with ctx.pose({bypass_joint: slide_upper}):
        open_panel_pos = ctx.part_world_position(bypass_panel)
    ctx.check(
        "bypass panel translates to the right when opened",
        closed_panel_pos is not None
        and open_panel_pos is not None
        and open_panel_pos[0] > closed_panel_pos[0] + 1.0,
        details=f"closed={closed_panel_pos}, open={open_panel_pos}",
    )

    def _span(aabb, axis_index: int) -> float | None:
        if aabb is None:
            return None
        return aabb[1][axis_index] - aabb[0][axis_index]

    rest_wing_aabb = ctx.part_element_world_aabb(rotor, elem="wing_pos_glass")
    with ctx.pose({drum_joint: math.pi / 2.0}):
        quarter_turn_wing_aabb = ctx.part_element_world_aabb(rotor, elem="wing_pos_glass")

    rest_x = _span(rest_wing_aabb, 0)
    rest_y = _span(rest_wing_aabb, 1)
    turned_x = _span(quarter_turn_wing_aabb, 0)
    turned_y = _span(quarter_turn_wing_aabb, 1)
    ctx.check(
        "rotor wing sweeps ninety degrees around the central shaft",
        rest_x is not None
        and rest_y is not None
        and turned_x is not None
        and turned_y is not None
        and rest_x > 0.9
        and rest_y < 0.08
        and turned_y > 0.9
        and turned_x < 0.08,
        details=(
            f"rest_spans=({rest_x}, {rest_y}), "
            f"turned_spans=({turned_x}, {turned_y})"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
