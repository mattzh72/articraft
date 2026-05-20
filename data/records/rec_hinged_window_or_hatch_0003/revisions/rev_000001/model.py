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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


def _xy_section(width: float, length: float, radius: float, z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, length, radius, corner_segments=8)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="marine_deck_hatch", assets=ASSETS)

    deck_white = model.material("deck_white", rgba=(0.91, 0.92, 0.89, 1.0))
    hatch_aluminum = model.material("hatch_aluminum", rgba=(0.68, 0.72, 0.75, 1.0))
    lid_white = model.material("lid_white", rgba=(0.94, 0.95, 0.93, 1.0))
    latch_black = model.material("latch_black", rgba=(0.12, 0.13, 0.14, 1.0))
    gasket_dark = model.material("gasket_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    smoked_acrylic = model.material("smoked_acrylic", rgba=(0.24, 0.33, 0.36, 0.60))

    deck_length = 0.66
    deck_width = 0.78
    deck_thickness = 0.012

    opening_length = 0.35
    opening_width = 0.46

    frame_outer_length = 0.45
    frame_outer_width = 0.56
    frame_wall = 0.05
    frame_height = 0.040

    lid_length = 0.51
    lid_width = 0.62
    lid_base_thickness = 0.018
    lid_crown_length = 0.41
    lid_crown_width = 0.50
    lid_crown_thickness = 0.012
    lid_rear_inset = 0.030

    hinge_radius = 0.011
    hinge_axis_y = -(frame_outer_length * 0.5) - 0.025
    hinge_axis_z = (deck_thickness * 0.5) + frame_height + hinge_radius

    deck = model.part("deck_panel")
    deck.inertial = Inertial.from_geometry(
        Box((deck_width, deck_length, deck_thickness)),
        mass=12.0,
        origin=Origin(),
    )
    side_span = (deck_width - opening_width) * 0.5
    end_span = (deck_length - opening_length) * 0.5
    deck.visual(
        Box((deck_width, end_span, deck_thickness)),
        origin=Origin(xyz=(0.0, (opening_length * 0.5) + (end_span * 0.5), 0.0)),
        material=deck_white,
        name="deck_fore",
    )
    deck.visual(
        Box((deck_width, end_span, deck_thickness)),
        origin=Origin(xyz=(0.0, -((opening_length * 0.5) + (end_span * 0.5)), 0.0)),
        material=deck_white,
        name="deck_aft",
    )
    deck.visual(
        Box((side_span, opening_length, deck_thickness)),
        origin=Origin(xyz=(-((opening_width * 0.5) + (side_span * 0.5)), 0.0, 0.0)),
        material=deck_white,
        name="deck_port",
    )
    deck.visual(
        Box((side_span, opening_length, deck_thickness)),
        origin=Origin(xyz=((opening_width * 0.5) + (side_span * 0.5), 0.0, 0.0)),
        material=deck_white,
        name="deck_starboard",
    )

    frame = model.part("curb_frame")
    frame.inertial = Inertial.from_geometry(
        Box((frame_outer_width, frame_outer_length, frame_height)),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.0, (deck_thickness * 0.5) + (frame_height * 0.5))),
    )
    frame_z = (deck_thickness * 0.5) + (frame_height * 0.5)
    front_back_span = (frame_outer_length - opening_length) * 0.5
    side_wall_span = (frame_outer_width - opening_width) * 0.5
    frame.visual(
        Box((frame_outer_width, front_back_span, frame_height)),
        origin=Origin(xyz=(0.0, (opening_length * 0.5) + (front_back_span * 0.5), frame_z)),
        material=hatch_aluminum,
        name="frame_fore",
    )
    frame.visual(
        Box((frame_outer_width, front_back_span, frame_height)),
        origin=Origin(xyz=(0.0, -((opening_length * 0.5) + (front_back_span * 0.5)), frame_z)),
        material=hatch_aluminum,
        name="frame_aft",
    )
    frame.visual(
        Box((side_wall_span, opening_length, frame_height)),
        origin=Origin(xyz=(-((opening_width * 0.5) + (side_wall_span * 0.5)), 0.0, frame_z)),
        material=hatch_aluminum,
        name="frame_port",
    )
    frame.visual(
        Box((side_wall_span, opening_length, frame_height)),
        origin=Origin(xyz=((opening_width * 0.5) + (side_wall_span * 0.5), 0.0, frame_z)),
        material=hatch_aluminum,
        name="frame_starboard",
    )

    hinge_segment_length = 0.18
    hinge_segment_offset = 0.19
    hinge_bridge_size = (0.050, 0.076, 0.024)
    frame.visual(
        Cylinder(radius=hinge_radius, length=hinge_segment_length),
        origin=Origin(xyz=(-hinge_segment_offset, hinge_axis_y, hinge_axis_z), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=gasket_dark,
        name="port_hinge_barrel",
    )
    frame.visual(
        Cylinder(radius=hinge_radius, length=hinge_segment_length),
        origin=Origin(xyz=(hinge_segment_offset, hinge_axis_y, hinge_axis_z), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=gasket_dark,
        name="starboard_hinge_barrel",
    )
    frame.visual(
        Box(hinge_bridge_size),
        origin=Origin(xyz=(-hinge_segment_offset, -0.213, 0.034)),
        material=hatch_aluminum,
        name="port_hinge_bridge",
    )
    frame.visual(
        Box(hinge_bridge_size),
        origin=Origin(xyz=(hinge_segment_offset, -0.213, 0.034)),
        material=hatch_aluminum,
        name="starboard_hinge_bridge",
    )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((lid_width, lid_length, 0.040)),
        mass=6.5,
        origin=Origin(xyz=(0.0, lid_rear_inset + (lid_length * 0.5), 0.010)),
    )
    crown_base_z = (deck_thickness * 0.5) + frame_height + lid_base_thickness - hinge_axis_z
    lid.visual(
        Box((lid_width, lid_length, lid_base_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                lid_rear_inset + (lid_length * 0.5),
                (deck_thickness * 0.5) + frame_height + (lid_base_thickness * 0.5) - hinge_axis_z,
            )
        ),
        material=lid_white,
        name="lid_flange",
    )
    crown_mesh = mesh_from_geometry(
        section_loft(
            [
                _xy_section(lid_crown_width, lid_crown_length, 0.048, 0.0),
                _xy_section(lid_crown_width - 0.024, lid_crown_length - 0.024, 0.054, lid_crown_thickness * 0.58),
                _xy_section(lid_crown_width - 0.044, lid_crown_length - 0.040, 0.060, lid_crown_thickness),
            ]
        ),
        ASSETS.mesh_path("lid_crown.obj"),
    )
    lid.visual(
        crown_mesh,
        origin=Origin(
            xyz=(
                0.0,
                lid_rear_inset + (lid_length * 0.5) + 0.005,
                crown_base_z,
            )
        ),
        material=lid_white,
        name="lid_crown",
    )
    lid.visual(
        Box((0.390, 0.300, 0.008)),
        origin=Origin(
            xyz=(
                0.0,
                lid_rear_inset + (lid_length * 0.5) + 0.015,
                crown_base_z + lid_crown_thickness + 0.004,
            )
        ),
        material=smoked_acrylic,
        name="lid_lens",
    )
    lid.visual(
        Cylinder(radius=hinge_radius, length=0.16),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=gasket_dark,
        name="lid_hinge_barrel",
    )
    lid.visual(
        Box((0.140, lid_rear_inset, 0.018)),
        origin=Origin(xyz=(0.0, lid_rear_inset * 0.5, (deck_thickness * 0.5) + frame_height + 0.009 - hinge_axis_z)),
        material=hatch_aluminum,
        name="rear_hinge_leaf",
    )
    latch_z = (
        (deck_thickness * 0.5)
        + frame_height
        + lid_base_thickness
        + (0.018 * 0.5)
        - hinge_axis_z
    )
    lid.visual(
        Box((0.060, 0.040, 0.018)),
        origin=Origin(xyz=(-0.220, 0.448, latch_z)),
        material=latch_black,
        name="port_latch_block",
    )
    lid.visual(
        Box((0.060, 0.040, 0.018)),
        origin=Origin(xyz=(0.220, 0.448, latch_z)),
        material=latch_black,
        name="starboard_latch_block",
    )
    lid.visual(
        Box((0.160, 0.020, 0.008)),
        origin=Origin(
            xyz=(
                0.0,
                0.513,
                (deck_thickness * 0.5) + frame_height + lid_base_thickness + 0.004 - hinge_axis_z,
            )
        ),
        material=hatch_aluminum,
        name="front_pull_lip",
    )

    model.articulation(
        "deck_to_frame",
        ArticulationType.FIXED,
        parent=deck,
        child=frame,
        origin=Origin(),
    )
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.4,
            lower=0.0,
            upper=math.radians(75.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    deck = object_model.get_part("deck_panel")
    frame = object_model.get_part("curb_frame")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("lid_hinge")

    frame_fore = frame.get_visual("frame_fore")
    frame_port = frame.get_visual("frame_port")
    frame_starboard = frame.get_visual("frame_starboard")
    lid_flange = lid.get_visual("lid_flange")
    lid_crown = lid.get_visual("lid_crown")
    port_latch = lid.get_visual("port_latch_block")
    starboard_latch = lid.get_visual("starboard_latch_block")
    pull_lip = lid.get_visual("front_pull_lip")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    ctx.expect_contact(frame, deck, name="curb frame sits on deck panel")
    ctx.expect_gap(
        frame,
        deck,
        axis="z",
        min_gap=0.0,
        max_gap=0.001,
        max_penetration=0.0,
        name="curb frame base is flush on deck top",
    )
    ctx.expect_contact(
        lid,
        frame,
        elem_a=lid_flange,
        elem_b=frame_fore,
        name="lid front flange seats on curb frame",
    )
    ctx.expect_overlap(
        lid,
        frame,
        axes="xy",
        min_overlap=0.44,
        name="lid footprint overlaps curb frame",
    )
    ctx.expect_overlap(
        lid,
        lid,
        axes="xy",
        min_overlap=0.03,
        elem_a=port_latch,
        elem_b=lid_flange,
        name="port latch block sits on lid top",
    )
    ctx.expect_overlap(
        lid,
        lid,
        axes="xy",
        min_overlap=0.03,
        elem_a=starboard_latch,
        elem_b=lid_flange,
        name="starboard latch block sits on lid top",
    )
    ctx.expect_overlap(
        lid,
        lid,
        axes="x",
        min_overlap=0.12,
        elem_a=pull_lip,
        elem_b=lid_flange,
        name="front pull lip stays centered across the lid width",
    )

    port_latch_aabb = ctx.part_element_world_aabb(lid, elem=port_latch)
    starboard_latch_aabb = ctx.part_element_world_aabb(lid, elem=starboard_latch)
    if port_latch_aabb is None or starboard_latch_aabb is None:
        ctx.fail("latch block AABBs available", "Expected both latch block visuals to have measurable AABBs.")
    else:
        port_center = tuple((lo + hi) * 0.5 for lo, hi in zip(port_latch_aabb[0], port_latch_aabb[1]))
        starboard_center = tuple((lo + hi) * 0.5 for lo, hi in zip(starboard_latch_aabb[0], starboard_latch_aabb[1]))
        ctx.check(
            "port latch near forward port corner",
            port_center[0] < -0.16 and port_center[1] > 0.14,
            details=f"port latch center={port_center}",
        )
        ctx.check(
            "starboard latch near forward starboard corner",
            starboard_center[0] > 0.16 and starboard_center[1] > 0.14,
            details=f"starboard latch center={starboard_center}",
        )

    pull_rest_aabb = ctx.part_element_world_aabb(lid, elem=pull_lip)
    crown_rest_aabb = ctx.part_element_world_aabb(lid, elem=lid_crown)
    if pull_rest_aabb is None or crown_rest_aabb is None:
        ctx.fail("lid feature AABBs available", "Expected front pull lip and lid crown to have measurable AABBs.")
    else:
        pull_rest_center = tuple((lo + hi) * 0.5 for lo, hi in zip(pull_rest_aabb[0], pull_rest_aabb[1]))
        crown_rest_center = tuple((lo + hi) * 0.5 for lo, hi in zip(crown_rest_aabb[0], crown_rest_aabb[1]))
        ctx.check(
            "lid crown is above main flange",
            crown_rest_center[2] > pull_rest_center[2] - 0.01,
            details=f"crown={crown_rest_center} pull={pull_rest_center}",
        )
        ctx.check(
            "front pull lip sits near the forward edge",
            pull_rest_center[1] > crown_rest_center[1] + 0.18,
            details=f"crown={crown_rest_center} pull={pull_rest_center}",
        )

    with ctx.pose({lid_hinge: math.radians(75.0)}):
        ctx.expect_gap(
            lid,
            frame,
            axis="z",
            min_gap=0.12,
            positive_elem=port_latch,
            negative_elem=frame_port,
            name="opened lid lifts port latch well clear of frame",
        )
        ctx.expect_gap(
            lid,
            frame,
            axis="z",
            min_gap=0.12,
            positive_elem=starboard_latch,
            negative_elem=frame_starboard,
            name="opened lid lifts starboard latch well clear of frame",
        )
        ctx.expect_overlap(
            lid,
            frame,
            axes="x",
            min_overlap=0.50,
            name="rear hinge keeps lid aligned across hatch width when open",
        )

        pull_open_aabb = ctx.part_element_world_aabb(lid, elem=pull_lip)
        if pull_open_aabb is None:
            ctx.fail("open pull lip AABB available", "Expected pull lip to remain measurable in open pose.")
        else:
            pull_open_center = tuple((lo + hi) * 0.5 for lo, hi in zip(pull_open_aabb[0], pull_open_aabb[1]))
            ctx.check(
                "front edge rises substantially when lid opens",
                pull_open_center[2] > 0.18,
                details=f"open pull lip center={pull_open_center}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
