from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)

RAIL_SPAN_X = 0.220
RAIL_LENGTH = 0.420
RAIL_WIDTH = 0.028
RAIL_HEIGHT = 0.018
SUPPORT_WIDTH = 0.040
SUPPORT_HEIGHT = 0.012
CROSS_TIE_LENGTH_Y = 0.060
CROSS_TIE_HEIGHT = SUPPORT_HEIGHT

BRIDGE_TRAVEL = 0.300
TRUCK_TRAVEL = 0.150

BRIDGE_PAD_SIZE = (0.028, 0.072, 0.012)
BRIDGE_TOWER_SIZE = (0.024, 0.050, 0.040)
BRIDGE_BEAM_SIZE = (0.250, 0.050, 0.024)

TRUCK_BODY_SIZE = (0.042, 0.040, 0.050)
TRUCK_SLIDER_SIZE = (0.022, 0.012, 0.008)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_rail_gantry_axis", assets=ASSETS)

    dark_base = model.material("dark_base", rgba=(0.18, 0.19, 0.21, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    carriage_gray = model.material("carriage_gray", rgba=(0.58, 0.60, 0.63, 1.0))
    accent_blue = model.material("accent_blue", rgba=(0.17, 0.33, 0.63, 1.0))
    truck_orange = model.material("truck_orange", rgba=(0.90, 0.47, 0.15, 1.0))

    rail_top_z = SUPPORT_HEIGHT + RAIL_HEIGHT

    base = model.part("base")
    base.visual(
        Box((SUPPORT_WIDTH, RAIL_LENGTH, SUPPORT_HEIGHT)),
        origin=Origin(xyz=(-RAIL_SPAN_X / 2.0, 0.0, SUPPORT_HEIGHT / 2.0)),
        material=dark_base,
        name="left_support",
    )
    base.visual(
        Box((SUPPORT_WIDTH, RAIL_LENGTH, SUPPORT_HEIGHT)),
        origin=Origin(xyz=(RAIL_SPAN_X / 2.0, 0.0, SUPPORT_HEIGHT / 2.0)),
        material=dark_base,
        name="right_support",
    )
    base.visual(
        Box((RAIL_WIDTH, RAIL_LENGTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(-RAIL_SPAN_X / 2.0, 0.0, SUPPORT_HEIGHT + RAIL_HEIGHT / 2.0)),
        material=rail_steel,
        name="left_rail",
    )
    base.visual(
        Box((RAIL_WIDTH, RAIL_LENGTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(RAIL_SPAN_X / 2.0, 0.0, SUPPORT_HEIGHT + RAIL_HEIGHT / 2.0)),
        material=rail_steel,
        name="right_rail",
    )
    base.visual(
        Box((RAIL_SPAN_X + SUPPORT_WIDTH, CROSS_TIE_LENGTH_Y, CROSS_TIE_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -(RAIL_LENGTH / 2.0 - CROSS_TIE_LENGTH_Y / 2.0),
                CROSS_TIE_HEIGHT / 2.0,
            )
        ),
        material=dark_base,
        name="front_tie",
    )
    base.visual(
        Box((RAIL_SPAN_X + SUPPORT_WIDTH, CROSS_TIE_LENGTH_Y, CROSS_TIE_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                RAIL_LENGTH / 2.0 - CROSS_TIE_LENGTH_Y / 2.0,
                CROSS_TIE_HEIGHT / 2.0,
            )
        ),
        material=dark_base,
        name="rear_tie",
    )
    base.inertial = Inertial.from_geometry(
        Box((RAIL_SPAN_X + SUPPORT_WIDTH, RAIL_LENGTH, rail_top_z)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, rail_top_z / 2.0)),
    )

    bridge = model.part("bridge")
    bridge.visual(
        Box(BRIDGE_PAD_SIZE),
        origin=Origin(
            xyz=(-RAIL_SPAN_X / 2.0, 0.0, rail_top_z + BRIDGE_PAD_SIZE[2] / 2.0)
        ),
        material=accent_blue,
        name="left_pad",
    )
    bridge.visual(
        Box(BRIDGE_PAD_SIZE),
        origin=Origin(
            xyz=(RAIL_SPAN_X / 2.0, 0.0, rail_top_z + BRIDGE_PAD_SIZE[2] / 2.0)
        ),
        material=accent_blue,
        name="right_pad",
    )
    bridge.visual(
        Box(BRIDGE_TOWER_SIZE),
        origin=Origin(
            xyz=(
                -RAIL_SPAN_X / 2.0,
                0.0,
                rail_top_z + BRIDGE_PAD_SIZE[2] + BRIDGE_TOWER_SIZE[2] / 2.0,
            )
        ),
        material=carriage_gray,
        name="left_tower",
    )
    bridge.visual(
        Box(BRIDGE_TOWER_SIZE),
        origin=Origin(
            xyz=(
                RAIL_SPAN_X / 2.0,
                0.0,
                rail_top_z + BRIDGE_PAD_SIZE[2] + BRIDGE_TOWER_SIZE[2] / 2.0,
            )
        ),
        material=carriage_gray,
        name="right_tower",
    )
    bridge.visual(
        Box(BRIDGE_BEAM_SIZE),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                rail_top_z + BRIDGE_PAD_SIZE[2] + BRIDGE_TOWER_SIZE[2] + BRIDGE_BEAM_SIZE[2] / 2.0,
            )
        ),
        material=carriage_gray,
        name="beam",
    )
    bridge.inertial = Inertial.from_geometry(
        Box((BRIDGE_BEAM_SIZE[0], BRIDGE_PAD_SIZE[1], rail_top_z + 0.080)),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.0, (rail_top_z + 0.080) / 2.0)),
    )

    truck = model.part("truck")
    truck.visual(
        Box(TRUCK_BODY_SIZE),
        origin=Origin(xyz=(0.0, 0.0, TRUCK_BODY_SIZE[2] / 2.0)),
        material=truck_orange,
        name="body",
    )
    truck.visual(
        Box(TRUCK_SLIDER_SIZE),
        origin=Origin(xyz=(0.0, -0.014, TRUCK_BODY_SIZE[2] + TRUCK_SLIDER_SIZE[2] / 2.0)),
        material=rail_steel,
        name="left_slider",
    )
    truck.visual(
        Box(TRUCK_SLIDER_SIZE),
        origin=Origin(xyz=(0.0, 0.014, TRUCK_BODY_SIZE[2] + TRUCK_SLIDER_SIZE[2] / 2.0)),
        material=rail_steel,
        name="right_slider",
    )
    truck.visual(
        Box((0.028, 0.050, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=dark_base,
        name="tool_plate",
    )
    truck.inertial = Inertial.from_geometry(
        Box((0.050, 0.050, TRUCK_BODY_SIZE[2] + TRUCK_SLIDER_SIZE[2])),
        mass=1.5,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )

    model.articulation(
        "base_to_bridge",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bridge,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=150.0,
            velocity=0.50,
            lower=-BRIDGE_TRAVEL / 2.0,
            upper=BRIDGE_TRAVEL / 2.0,
        ),
    )
    model.articulation(
        "bridge_to_truck",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=truck,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.35,
            lower=-TRUCK_TRAVEL / 2.0,
            upper=TRUCK_TRAVEL / 2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    bridge = object_model.get_part("bridge")
    truck = object_model.get_part("truck")
    base_to_bridge = object_model.get_articulation("base_to_bridge")
    bridge_to_truck = object_model.get_articulation("bridge_to_truck")

    left_rail = base.get_visual("left_rail")
    right_rail = base.get_visual("right_rail")
    left_pad = bridge.get_visual("left_pad")
    right_pad = bridge.get_visual("right_pad")
    beam = bridge.get_visual("beam")
    left_slider = truck.get_visual("left_slider")
    right_slider = truck.get_visual("right_slider")

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
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=16,
        ignore_adjacent=False,
        ignore_fixed=True,
    )

    ctx.check(
        "parts_present",
        all(part is not None for part in (base, bridge, truck)),
        "base, bridge, and truck parts must all be present",
    )
    ctx.check(
        "articulations_present",
        base_to_bridge is not None and bridge_to_truck is not None,
        "Both prismatic stages must be defined",
    )

    ctx.expect_gap(
        bridge,
        base,
        axis="z",
        max_gap=0.0,
        max_penetration=0.0,
        positive_elem=left_pad,
        negative_elem=left_rail,
        name="left_bridge_pad_seats_on_left_rail",
    )
    ctx.expect_gap(
        bridge,
        base,
        axis="z",
        max_gap=0.0,
        max_penetration=0.0,
        positive_elem=right_pad,
        negative_elem=right_rail,
        name="right_bridge_pad_seats_on_right_rail",
    )
    ctx.expect_overlap(
        bridge,
        base,
        axes="y",
        min_overlap=0.070,
        elem_a=left_pad,
        elem_b=left_rail,
        name="left_pad_has_guide_overlap_on_left_rail",
    )
    ctx.expect_overlap(
        bridge,
        base,
        axes="y",
        min_overlap=0.070,
        elem_a=right_pad,
        elem_b=right_rail,
        name="right_pad_has_guide_overlap_on_right_rail",
    )
    ctx.expect_origin_distance(
        bridge,
        base,
        axes="x",
        max_dist=1e-6,
        name="bridge_is_centered_between_rails",
    )

    ctx.expect_contact(
        bridge,
        truck,
        contact_tol=1e-5,
        elem_a=beam,
        elem_b=left_slider,
        name="left_truck_slider_contacts_bridge_beam",
    )
    ctx.expect_contact(
        bridge,
        truck,
        contact_tol=1e-5,
        elem_a=beam,
        elem_b=right_slider,
        name="right_truck_slider_contacts_bridge_beam",
    )
    ctx.expect_overlap(
        bridge,
        truck,
        axes="x",
        min_overlap=0.020,
        elem_a=beam,
        elem_b=left_slider,
        name="truck_left_slider_stays_under_beam",
    )
    ctx.expect_overlap(
        bridge,
        truck,
        axes="x",
        min_overlap=0.020,
        elem_a=beam,
        elem_b=right_slider,
        name="truck_right_slider_stays_under_beam",
    )

    rest_bridge_pos = ctx.part_world_position(bridge)
    rest_truck_pos = ctx.part_world_position(truck)

    with ctx.pose({base_to_bridge: BRIDGE_TRAVEL / 2.0}):
        pos = ctx.part_world_position(bridge)
        ctx.check(
            "bridge_moves_along_rail_axis_positive",
            pos is not None
            and rest_bridge_pos is not None
            and math.isclose(pos[1] - rest_bridge_pos[1], BRIDGE_TRAVEL / 2.0, abs_tol=1e-6)
            and math.isclose(pos[0], rest_bridge_pos[0], abs_tol=1e-6)
            and math.isclose(pos[2], rest_bridge_pos[2], abs_tol=1e-6),
            f"Expected +Y bridge travel of {BRIDGE_TRAVEL / 2.0:.3f} m with no X/Z drift, got {pos}",
        )
        ctx.expect_gap(
            bridge,
            base,
            axis="z",
            max_gap=0.0,
            max_penetration=0.0,
            positive_elem=left_pad,
            negative_elem=left_rail,
            name="left_pad_stays_seated_at_positive_bridge_travel",
        )
        ctx.expect_gap(
            bridge,
            base,
            axis="z",
            max_gap=0.0,
            max_penetration=0.0,
            positive_elem=right_pad,
            negative_elem=right_rail,
            name="right_pad_stays_seated_at_positive_bridge_travel",
        )

    with ctx.pose({base_to_bridge: -BRIDGE_TRAVEL / 2.0}):
        pos = ctx.part_world_position(bridge)
        ctx.check(
            "bridge_moves_along_rail_axis_negative",
            pos is not None
            and rest_bridge_pos is not None
            and math.isclose(pos[1] - rest_bridge_pos[1], -BRIDGE_TRAVEL / 2.0, abs_tol=1e-6)
            and math.isclose(pos[0], rest_bridge_pos[0], abs_tol=1e-6)
            and math.isclose(pos[2], rest_bridge_pos[2], abs_tol=1e-6),
            f"Expected -Y bridge travel of {BRIDGE_TRAVEL / 2.0:.3f} m with no X/Z drift, got {pos}",
        )

    with ctx.pose({bridge_to_truck: TRUCK_TRAVEL / 2.0}):
        pos = ctx.part_world_position(truck)
        ctx.check(
            "truck_moves_along_bridge_axis_positive",
            pos is not None
            and rest_truck_pos is not None
            and math.isclose(pos[0] - rest_truck_pos[0], TRUCK_TRAVEL / 2.0, abs_tol=1e-6)
            and math.isclose(pos[1], rest_truck_pos[1], abs_tol=1e-6),
            f"Expected +X truck travel of {TRUCK_TRAVEL / 2.0:.3f} m with fixed Y, got {pos}",
        )
        ctx.expect_contact(
            bridge,
            truck,
            contact_tol=1e-5,
            elem_a=beam,
            elem_b=left_slider,
            name="left_slider_stays_on_beam_at_positive_truck_travel",
        )
        ctx.expect_contact(
            bridge,
            truck,
            contact_tol=1e-5,
            elem_a=beam,
            elem_b=right_slider,
            name="right_slider_stays_on_beam_at_positive_truck_travel",
        )

    with ctx.pose({base_to_bridge: 0.120, bridge_to_truck: -TRUCK_TRAVEL / 2.0}):
        pos = ctx.part_world_position(truck)
        ctx.check(
            "truck_combined_pose_tracks_bridge_and_cross_axis",
            pos is not None
            and rest_truck_pos is not None
            and math.isclose(pos[0] - rest_truck_pos[0], -TRUCK_TRAVEL / 2.0, abs_tol=1e-6)
            and math.isclose(pos[1] - rest_truck_pos[1], 0.120, abs_tol=1e-6),
            f"Expected truck to inherit bridge Y motion and its own X motion, got {pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
