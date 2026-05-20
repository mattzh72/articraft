from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    mesh_from_geometry,
    wire_from_points,
)


DECK_WIDTH = 0.62
DECK_LENGTH = 0.98
DECK_THICKNESS = 0.040
DECK_TOP_Z = 0.158
DECK_BOTTOM_Z = DECK_TOP_Z - DECK_THICKNESS
DECK_CENTER_Z = (DECK_TOP_Z + DECK_BOTTOM_Z) * 0.5

CASTER_X = 0.235
CASTER_Y = 0.365
CASTER_SWIVEL_Z = 0.100
WHEEL_RADIUS = 0.050
WHEEL_WIDTH = 0.026


def _add_wheel_visuals(part, *, rubber, hub_steel) -> None:
    spin_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))
    part.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        origin=spin_origin,
        material=rubber,
        name="tread",
    )
    part.visual(
        Cylinder(radius=0.031, length=0.018),
        origin=spin_origin,
        material=hub_steel,
        name="hub_barrel",
    )
    part.visual(
        Cylinder(radius=0.034, length=0.004),
        origin=Origin(xyz=(-0.011, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_steel,
        name="left_hub_cap",
    )
    part.visual(
        Cylinder(radius=0.034, length=0.004),
        origin=Origin(xyz=(0.011, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_steel,
        name="right_hub_cap",
    )


def _add_caster_visuals(part, *, steel, dark_steel) -> None:
    part.visual(
        Box((0.100, 0.100, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=steel,
        name="top_plate",
    )
    part.visual(
        Cylinder(radius=0.013, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, -0.021)),
        material=dark_steel,
        name="swivel_stem",
    )
    part.visual(
        Box((0.050, 0.048, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.031)),
        material=dark_steel,
        name="fork_crown",
    )
    part.visual(
        Box((0.010, 0.066, 0.056)),
        origin=Origin(xyz=(-0.025, 0.0, -0.060)),
        material=steel,
        name="left_fork_arm",
    )
    part.visual(
        Box((0.010, 0.066, 0.056)),
        origin=Origin(xyz=(0.025, 0.0, -0.060)),
        material=steel,
        name="right_fork_arm",
    )
    part.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(xyz=(-0.017, 0.0, -0.088), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="left_axle_boss",
    )
    part.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(xyz=(0.017, 0.0, -0.088), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="right_axle_boss",
    )


def _add_caster(
    model: ArticulatedObject,
    *,
    name_prefix: str,
    position: tuple[float, float],
    deck,
    steel,
    dark_steel,
    rubber,
) -> None:
    caster = model.part(f"{name_prefix}_caster")
    caster.inertial = Inertial.from_geometry(
        Box((0.100, 0.100, 0.108)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
    )
    _add_caster_visuals(caster, steel=steel, dark_steel=dark_steel)

    wheel = model.part(f"{name_prefix}_wheel")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=1.1,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_wheel_visuals(wheel, rubber=rubber, hub_steel=steel)

    x_pos, y_pos = position
    model.articulation(
        f"{name_prefix}_swivel",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=caster,
        origin=Origin(xyz=(x_pos, y_pos, CASTER_SWIVEL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=8.0),
    )
    model.articulation(
        f"{name_prefix}_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=caster,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, -0.088)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=24.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_platform_cart")

    deck_blue = model.material("deck_blue", rgba=(0.22, 0.37, 0.59, 1.0))
    frame_gray = model.material("frame_gray", rgba=(0.20, 0.22, 0.24, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.31, 0.33, 0.35, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    deck = model.part("deck")
    deck.inertial = Inertial.from_geometry(
        Box((DECK_WIDTH, DECK_LENGTH, 0.070)),
        mass=72.0,
        origin=Origin(xyz=(0.0, 0.0, 0.123)),
    )
    deck.visual(
        Box((DECK_WIDTH, DECK_LENGTH, DECK_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, DECK_CENTER_Z)),
        material=deck_blue,
        name="platform",
    )
    deck.visual(
        Box((0.052, 0.710, 0.022)),
        origin=Origin(xyz=(-0.140, 0.0, 0.107)),
        material=frame_gray,
        name="left_underrail",
    )
    deck.visual(
        Box((0.052, 0.710, 0.022)),
        origin=Origin(xyz=(0.140, 0.0, 0.107)),
        material=frame_gray,
        name="right_underrail",
    )
    deck.visual(
        Box((0.350, 0.060, 0.022)),
        origin=Origin(xyz=(0.0, -0.255, 0.107)),
        material=frame_gray,
        name="front_crossmember",
    )
    deck.visual(
        Box((0.350, 0.060, 0.022)),
        origin=Origin(xyz=(0.0, 0.255, 0.107)),
        material=frame_gray,
        name="rear_crossmember",
    )
    for visual_name, x_pos, y_pos in (
        ("front_left_pad", CASTER_X, -CASTER_Y),
        ("front_right_pad", -CASTER_X, -CASTER_Y),
        ("rear_left_pad", CASTER_X, CASTER_Y),
        ("rear_right_pad", -CASTER_X, CASTER_Y),
    ):
        deck.visual(
            Box((0.110, 0.110, 0.010)),
            origin=Origin(xyz=(x_pos, y_pos, 0.113)),
            material=frame_gray,
            name=visual_name,
        )
    deck.visual(
        Box((0.018, 0.018, 0.012)),
        origin=Origin(xyz=(-0.059, -0.490, 0.112)),
        material=frame_gray,
        name="left_hinge_tab",
    )
    deck.visual(
        Box((0.018, 0.018, 0.012)),
        origin=Origin(xyz=(0.059, -0.490, 0.112)),
        material=frame_gray,
        name="right_hinge_tab",
    )

    push_handle = model.part("push_handle")
    push_handle.inertial = Inertial.from_geometry(
        Box((0.440, 0.110, 0.470)),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.005, 0.235)),
    )
    push_handle.visual(
        Box((0.050, 0.044, 0.008)),
        origin=Origin(xyz=(-0.190, -0.022, 0.004)),
        material=dark_steel,
        name="left_mount_foot",
    )
    push_handle.visual(
        Box((0.050, 0.044, 0.008)),
        origin=Origin(xyz=(0.190, -0.022, 0.004)),
        material=dark_steel,
        name="right_mount_foot",
    )
    push_handle.visual(
        Cylinder(radius=0.014, length=0.040),
        origin=Origin(xyz=(-0.190, -0.022, 0.028)),
        material=dark_steel,
        name="left_post_stub",
    )
    push_handle.visual(
        Cylinder(radius=0.014, length=0.040),
        origin=Origin(xyz=(0.190, -0.022, 0.028)),
        material=dark_steel,
        name="right_post_stub",
    )
    handle_tube = wire_from_points(
        [
            (-0.190, -0.022, 0.048),
            (-0.190, 0.018, 0.315),
            (-0.175, 0.035, 0.410),
            (0.175, 0.035, 0.410),
            (0.190, 0.018, 0.315),
            (0.190, -0.022, 0.048),
        ],
        radius=0.014,
        radial_segments=18,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.040,
        corner_segments=10,
    )
    push_handle.visual(
        mesh_from_geometry(handle_tube, "platform_cart_push_handle"),
        material=dark_steel,
        name="handle_tube",
    )

    tow_bar = model.part("tow_bar")
    tow_bar.inertial = Inertial.from_geometry(
        Box((0.170, 0.050, 0.285)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, -0.142)),
    )
    tow_bar.visual(
        Box((0.020, 0.010, 0.012)),
        origin=Origin(xyz=(-0.040, 0.0, -0.008)),
        material=dark_steel,
        name="left_hinge_ear",
    )
    tow_bar.visual(
        Box((0.020, 0.010, 0.012)),
        origin=Origin(xyz=(0.040, 0.0, -0.008)),
        material=dark_steel,
        name="right_hinge_ear",
    )
    tow_bar.visual(
        Box((0.060, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=dark_steel,
        name="hinge_crossbar",
    )
    tow_bar.visual(
        Box((0.054, 0.030, 0.188)),
        origin=Origin(xyz=(0.0, 0.0, -0.117)),
        material=steel,
        name="draw_tongue",
    )
    tow_bar.visual(
        Cylinder(radius=0.013, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, -0.229)),
        material=steel,
        name="tow_end_boss",
    )

    model.articulation(
        "deck_to_push_handle",
        ArticulationType.FIXED,
        parent=deck,
        child=push_handle,
        origin=Origin(xyz=(0.0, 0.490, DECK_TOP_Z)),
    )
    model.articulation(
        "deck_to_tow_bar",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=tow_bar,
        origin=Origin(xyz=(0.0, -0.490, 0.118)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.4, lower=-1.20, upper=0.15),
    )

    _add_caster(
        model,
        name_prefix="front_left",
        position=(CASTER_X, -CASTER_Y),
        deck=deck,
        steel=steel,
        dark_steel=dark_steel,
        rubber=rubber,
    )
    _add_caster(
        model,
        name_prefix="front_right",
        position=(-CASTER_X, -CASTER_Y),
        deck=deck,
        steel=steel,
        dark_steel=dark_steel,
        rubber=rubber,
    )
    _add_caster(
        model,
        name_prefix="rear_left",
        position=(CASTER_X, CASTER_Y),
        deck=deck,
        steel=steel,
        dark_steel=dark_steel,
        rubber=rubber,
    )
    _add_caster(
        model,
        name_prefix="rear_right",
        position=(-CASTER_X, CASTER_Y),
        deck=deck,
        steel=steel,
        dark_steel=dark_steel,
        rubber=rubber,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    deck = object_model.get_part("deck")
    push_handle = object_model.get_part("push_handle")
    tow_bar = object_model.get_part("tow_bar")

    swivel_joints = [
        object_model.get_articulation("front_left_swivel"),
        object_model.get_articulation("front_right_swivel"),
        object_model.get_articulation("rear_left_swivel"),
        object_model.get_articulation("rear_right_swivel"),
    ]
    spin_joints = [
        object_model.get_articulation("front_left_wheel_spin"),
        object_model.get_articulation("front_right_wheel_spin"),
        object_model.get_articulation("rear_left_wheel_spin"),
        object_model.get_articulation("rear_right_wheel_spin"),
    ]
    tow_joint = object_model.get_articulation("deck_to_tow_bar")

    ctx.expect_contact(push_handle, deck, name="push_handle_mounted_to_deck")
    ctx.expect_contact(tow_bar, deck, name="tow_bar_clipped_to_front_hinge")
    ctx.expect_origin_gap(
        push_handle,
        deck,
        axis="y",
        min_gap=0.46,
        max_gap=0.52,
        name="push_handle_at_rear_edge",
    )
    ctx.expect_origin_gap(
        deck,
        tow_bar,
        axis="y",
        min_gap=0.46,
        max_gap=0.52,
        name="tow_bar_at_front_edge",
    )

    for name_prefix in ("front_left", "front_right", "rear_left", "rear_right"):
        caster = object_model.get_part(f"{name_prefix}_caster")
        wheel = object_model.get_part(f"{name_prefix}_wheel")
        ctx.expect_contact(caster, deck, name=f"{name_prefix}_caster_seated")
        ctx.expect_contact(wheel, caster, name=f"{name_prefix}_wheel_on_axle_bosses")

    ctx.check(
        "swivel_axes_vertical",
        all(joint.axis == (0.0, 0.0, 1.0) for joint in swivel_joints),
        details=str([joint.axis for joint in swivel_joints]),
    )
    ctx.check(
        "wheel_axes_horizontal",
        all(joint.axis == (1.0, 0.0, 0.0) for joint in spin_joints),
        details=str([joint.axis for joint in spin_joints]),
    )
    ctx.check(
        "tow_bar_hinge_axis_left_right",
        tow_joint.axis == (1.0, 0.0, 0.0),
        details=str(tow_joint.axis),
    )

    deck_aabb = ctx.part_world_aabb(deck)
    ctx.check(
        "deck_height_realistic",
        deck_aabb is not None and 0.150 <= deck_aabb[1][2] <= 0.165,
        details=str(deck_aabb),
    )

    tow_rest_pos = ctx.part_world_position(tow_bar)
    tow_rest_aabb = ctx.part_world_aabb(tow_bar)
    with ctx.pose({tow_joint: -1.10}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlaps_with_tow_bar_deployed")
        tow_deployed_pos = ctx.part_world_position(tow_bar)
        tow_deployed_aabb = ctx.part_world_aabb(tow_bar)
        ctx.check(
            "tow_bar_hinge_origin_stays_attached",
            tow_rest_pos is not None
            and tow_deployed_pos is not None
            and max(abs(a - b) for a, b in zip(tow_rest_pos, tow_deployed_pos)) <= 1e-6,
            details=f"rest={tow_rest_pos}, deployed={tow_deployed_pos}",
        )
        ctx.check(
            "tow_bar_swings_forward_for_towing",
            tow_rest_aabb is not None
            and tow_deployed_aabb is not None
            and tow_deployed_aabb[0][1] < tow_rest_aabb[0][1] - 0.10,
            details=f"rest={tow_rest_aabb}, deployed={tow_deployed_aabb}",
        )
        ctx.check(
            "tow_bar_remains_below_deck_when_deployed",
            tow_rest_aabb is not None
            and tow_deployed_aabb is not None
            and tow_deployed_aabb[1][2] < deck_aabb[1][2] - 0.010,
            details=f"rest={tow_rest_aabb}, deployed={tow_deployed_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
