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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    place_on_face_uv,
)


def _add_bolt_heads(
    part,
    centers,
    *,
    seat_z: float,
    radius: float = 0.006,
    height: float = 0.006,
    material: str = "fastener",
    prefix: str = "bolt",
) -> None:
    for index, (x_pos, y_pos) in enumerate(centers, start=1):
        part.visual(
            Cylinder(radius=radius, length=height),
            origin=Origin(xyz=(x_pos, y_pos, seat_z + (0.5 * height))),
            material=material,
            name=f"{prefix}_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_safety_turntable")

    model.material("machine_gray", rgba=(0.23, 0.24, 0.26, 1.0))
    model.material("deck_gray", rgba=(0.34, 0.35, 0.38, 1.0))
    model.material("machined_steel", rgba=(0.63, 0.65, 0.68, 1.0))
    model.material("matte_black", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("safety_yellow", rgba=(0.88, 0.75, 0.12, 1.0))
    model.material("fastener", rgba=(0.74, 0.75, 0.78, 1.0))
    model.material("signal_red", rgba=(0.80, 0.14, 0.12, 1.0))

    plinth_size = (0.58, 0.45, 0.09)
    deck_size = (0.56, 0.43, 0.012)
    deck_top = plinth_size[2] + deck_size[2]

    plinth = model.part("plinth")
    plinth.visual(
        Box(plinth_size),
        origin=Origin(xyz=(0.0, 0.0, plinth_size[2] * 0.5)),
        material="machine_gray",
        name="base_shell",
    )
    plinth.visual(
        Box(deck_size),
        origin=Origin(xyz=(0.0, 0.0, plinth_size[2] + (deck_size[2] * 0.5))),
        material="deck_gray",
        name="top_deck",
    )
    plinth.inertial = Inertial.from_geometry(
        Box(plinth_size),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, plinth_size[2] * 0.5)),
    )

    spindle_support = model.part("spindle_support")
    spindle_support.visual(
        Box((0.18, 0.14, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material="deck_gray",
        name="bearing_base",
    )
    spindle_support.visual(
        Cylinder(radius=0.055, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material="machined_steel",
        name="spindle_pedestal",
    )
    spindle_support.visual(
        Box((0.032, 0.14, 0.028)),
        origin=Origin(xyz=(-0.065, 0.0, 0.024)),
        material="deck_gray",
        name="bearing_rib_left",
    )
    spindle_support.visual(
        Box((0.032, 0.14, 0.028)),
        origin=Origin(xyz=(0.065, 0.0, 0.024)),
        material="deck_gray",
        name="bearing_rib_right",
    )
    _add_bolt_heads(
        spindle_support,
        [(-0.065, -0.045), (-0.065, 0.045), (0.065, -0.045), (0.065, 0.045)],
        seat_z=0.010,
        radius=0.0065,
        height=0.006,
        material="fastener",
        prefix="bearing_bolt",
    )
    spindle_support.inertial = Inertial.from_geometry(
        Box((0.18, 0.14, 0.040)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.045, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material="machined_steel",
        name="hub_mount",
    )
    platter.visual(
        Cylinder(radius=0.136, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material="machined_steel",
        name="main_disc",
    )
    platter.visual(
        Cylinder(radius=0.145, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material="machined_steel",
        name="outer_rim",
    )
    platter.visual(
        Cylinder(radius=0.132, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0335)),
        material="matte_black",
        name="record_mat",
    )
    platter.visual(
        Cylinder(radius=0.003, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material="machined_steel",
        name="spindle_tip",
    )
    platter.inertial = Inertial.from_geometry(
        Cylinder(radius=0.145, length=0.035),
        mass=7.2,
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
    )

    tonearm_support = model.part("tonearm_support")
    tonearm_support.visual(
        Box((0.14, 0.12, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material="deck_gray",
        name="support_base",
    )
    tonearm_support.visual(
        Box((0.12, 0.11, 0.048)),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material="machine_gray",
        name="arm_pedestal",
    )
    tonearm_support.visual(
        Box((0.09, 0.008, 0.048)),
        origin=Origin(xyz=(-0.015, -0.056, 0.034)),
        material="deck_gray",
        name="pedestal_side_plate_a",
    )
    tonearm_support.visual(
        Box((0.09, 0.008, 0.048)),
        origin=Origin(xyz=(-0.015, 0.056, 0.034)),
        material="deck_gray",
        name="pedestal_side_plate_b",
    )
    tonearm_support.visual(
        Box((0.022, 0.080, 0.038)),
        origin=Origin(xyz=(-0.058, 0.0, 0.029)),
        material="deck_gray",
        name="rear_buttress",
    )
    tonearm_support.visual(
        Box((0.014, 0.026, 0.030)),
        origin=Origin(xyz=(-0.063, -0.030, 0.025)),
        material="safety_yellow",
        name="stop_block_park",
    )
    tonearm_support.visual(
        Box((0.014, 0.026, 0.030)),
        origin=Origin(xyz=(-0.063, 0.030, 0.025)),
        material="safety_yellow",
        name="stop_block_play",
    )
    tonearm_support.visual(
        Box((0.014, 0.074, 0.012)),
        origin=Origin(xyz=(-0.063, 0.0, 0.046)),
        material="safety_yellow",
        name="stop_bridge",
    )
    _add_bolt_heads(
        tonearm_support,
        [(-0.038, -0.033), (-0.038, 0.033), (0.038, -0.033), (0.038, 0.033)],
        seat_z=0.058,
        radius=0.006,
        height=0.006,
        material="fastener",
        prefix="pedestal_bolt",
    )
    tonearm_support.inertial = Inertial.from_geometry(
        Box((0.14, 0.12, 0.070)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
    )

    tonearm_stage = model.part("tonearm_stage")
    tonearm_stage.visual(
        Cylinder(radius=0.028, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material="machined_steel",
        name="pivot_mount",
    )
    tonearm_stage.visual(
        Cylinder(radius=0.022, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material="machined_steel",
        name="pivot_barrel",
    )
    tonearm_stage.visual(
        Box((0.250, 0.018, 0.014)),
        origin=Origin(xyz=(0.125, 0.0, 0.055)),
        material="matte_black",
        name="arm_beam",
    )
    tonearm_stage.visual(
        Box((0.040, 0.030, 0.010)),
        origin=Origin(xyz=(0.255, 0.0, 0.050)),
        material="deck_gray",
        name="headshell",
    )
    tonearm_stage.visual(
        Box((0.014, 0.030, 0.018)),
        origin=Origin(xyz=(0.272, 0.0, 0.040)),
        material="safety_yellow",
        name="stylus_guard",
    )
    tonearm_stage.visual(
        Cylinder(radius=0.007, length=0.070),
        origin=Origin(xyz=(-0.035, 0.0, 0.052), rpy=(0.0, math.pi * 0.5, 0.0)),
        material="machined_steel",
        name="counterweight_stub",
    )
    tonearm_stage.visual(
        Cylinder(radius=0.014, length=0.032),
        origin=Origin(xyz=(-0.078, 0.0, 0.052), rpy=(0.0, math.pi * 0.5, 0.0)),
        material="machined_steel",
        name="counterweight",
    )
    tonearm_stage.visual(
        Box((0.016, 0.030, 0.018)),
        origin=Origin(xyz=(-0.004, -0.020, 0.018)),
        material="safety_yellow",
        name="stop_lug",
    )
    tonearm_stage.inertial = Inertial.from_geometry(
        Box((0.30, 0.05, 0.08)),
        mass=1.8,
        origin=Origin(xyz=(0.08, 0.0, 0.04)),
    )

    guard = model.part("platter_guard")
    guard.visual(
        Box((0.050, 0.090, 0.012)),
        origin=Origin(xyz=(-0.140, 0.0, 0.006)),
        material="deck_gray",
        name="left_guard_foot",
    )
    guard.visual(
        Box((0.050, 0.090, 0.012)),
        origin=Origin(xyz=(0.140, 0.0, 0.006)),
        material="deck_gray",
        name="right_guard_foot",
    )
    guard.visual(
        Box((0.020, 0.028, 0.200)),
        origin=Origin(xyz=(-0.140, 0.0, 0.112)),
        material="safety_yellow",
        name="left_stanchion",
    )
    guard.visual(
        Box((0.020, 0.028, 0.200)),
        origin=Origin(xyz=(0.140, 0.0, 0.112)),
        material="safety_yellow",
        name="right_stanchion",
    )
    guard.visual(
        Box((0.300, 0.036, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.217)),
        material="safety_yellow",
        name="top_crossbar",
    )
    guard.visual(
        Box((0.300, 0.012, 0.160)),
        origin=Origin(xyz=(0.0, 0.024, 0.137)),
        material="safety_yellow",
        name="rear_shield",
    )
    guard.visual(
        Box((0.012, 0.060, 0.135)),
        origin=Origin(xyz=(-0.144, 0.003, 0.108)),
        material="safety_yellow",
        name="left_side_shield",
    )
    guard.visual(
        Box((0.012, 0.060, 0.135)),
        origin=Origin(xyz=(0.144, 0.003, 0.108)),
        material="safety_yellow",
        name="right_side_shield",
    )
    _add_bolt_heads(
        guard,
        [(-0.140, -0.024), (-0.140, 0.024), (0.140, -0.024), (0.140, 0.024)],
        seat_z=0.012,
        radius=0.0055,
        height=0.005,
        material="fastener",
        prefix="guard_bolt",
    )

    arm_rest = model.part("arm_rest_lock")
    arm_rest.visual(
        Box((0.040, 0.050, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material="deck_gray",
        name="rest_base",
    )
    arm_rest.visual(
        Cylinder(radius=0.006, length=0.086),
        origin=Origin(xyz=(0.0, 0.0, 0.051)),
        material="machined_steel",
        name="rest_post",
    )
    arm_rest.visual(
        Box((0.014, 0.040, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.089)),
        material="safety_yellow",
        name="rest_cradle",
    )
    arm_rest.visual(
        Box((0.006, 0.020, 0.018)),
        origin=Origin(xyz=(0.009, 0.0, 0.103)),
        material="safety_yellow",
        name="rest_clip",
    )
    _add_bolt_heads(
        arm_rest,
        [(-0.010, -0.014), (-0.010, 0.014)],
        seat_z=0.008,
        radius=0.004,
        height=0.004,
        material="fastener",
        prefix="rest_bolt",
    )

    lockout = model.part("lockout_box")
    lockout.visual(
        Box((0.10, 0.08, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material="deck_gray",
        name="lockout_housing",
    )
    lockout.visual(
        Box((0.060, 0.052, 0.008)),
        origin=Origin(xyz=(0.0, 0.022, 0.059)),
        material="safety_yellow",
        name="lockout_canopy",
    )
    lockout.visual(
        Box((0.008, 0.052, 0.034)),
        origin=Origin(xyz=(-0.026, 0.022, 0.042)),
        material="safety_yellow",
        name="guard_cheek_left",
    )
    lockout.visual(
        Box((0.008, 0.052, 0.034)),
        origin=Origin(xyz=(0.026, 0.022, 0.042)),
        material="safety_yellow",
        name="guard_cheek_right",
    )
    lockout.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.0, 0.041, 0.032), rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material="signal_red",
        name="e_stop_button",
    )
    lockout.visual(
        Box((0.018, 0.012, 0.032)),
        origin=Origin(xyz=(0.028, -0.010, 0.071)),
        material="machined_steel",
        name="lockout_key",
    )

    model.articulation(
        "plinth_to_spindle_support",
        ArticulationType.FIXED,
        parent=plinth,
        child=spindle_support,
        origin=place_on_face_uv(plinth, "+z", uv=(0.43, 0.54), proud=0.0),
    )
    model.articulation(
        "spindle_support_to_platter",
        ArticulationType.CONTINUOUS,
        parent=spindle_support,
        child=platter,
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=12.0),
    )

    model.articulation(
        "plinth_to_tonearm_support",
        ArticulationType.FIXED,
        parent=plinth,
        child=tonearm_support,
        origin=place_on_face_uv(plinth, "+z", uv=(0.845, 0.233), proud=0.0),
    )
    model.articulation(
        "tonearm_support_to_stage",
        ArticulationType.REVOLUTE,
        parent=tonearm_support,
        child=tonearm_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.058), rpy=(0.0, 0.0, 1.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=0.0,
            upper=1.75,
        ),
    )

    model.articulation(
        "plinth_to_guard",
        ArticulationType.FIXED,
        parent=plinth,
        child=guard,
        origin=place_on_face_uv(plinth, "+z", uv=(0.414, 0.92), proud=0.0),
    )
    model.articulation(
        "plinth_to_arm_rest",
        ArticulationType.FIXED,
        parent=plinth,
        child=arm_rest,
        origin=place_on_face_uv(plinth, "+z", uv=(0.97, 0.44), proud=0.0),
    )
    model.articulation(
        "plinth_to_lockout",
        ArticulationType.FIXED,
        parent=plinth,
        child=lockout,
        origin=place_on_face_uv(plinth, "+z", uv=(0.16, 0.18), proud=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plinth = object_model.get_part("plinth")
    spindle_support = object_model.get_part("spindle_support")
    platter = object_model.get_part("platter")
    tonearm_support = object_model.get_part("tonearm_support")
    tonearm_stage = object_model.get_part("tonearm_stage")
    guard = object_model.get_part("platter_guard")
    arm_rest = object_model.get_part("arm_rest_lock")
    lockout = object_model.get_part("lockout_box")

    platter_joint = object_model.get_articulation("spindle_support_to_platter")
    tonearm_joint = object_model.get_articulation("tonearm_support_to_stage")

    top_deck = plinth.get_visual("top_deck")
    bearing_base = spindle_support.get_visual("bearing_base")
    spindle_pedestal = spindle_support.get_visual("spindle_pedestal")
    hub_mount = platter.get_visual("hub_mount")
    record_mat = platter.get_visual("record_mat")
    outer_rim = platter.get_visual("outer_rim")
    support_base = tonearm_support.get_visual("support_base")
    arm_pedestal = tonearm_support.get_visual("arm_pedestal")
    tonearm_mount = tonearm_stage.get_visual("pivot_mount")
    headshell = tonearm_stage.get_visual("headshell")
    guard_foot = guard.get_visual("left_guard_foot")
    rear_shield = guard.get_visual("rear_shield")
    rest_base = arm_rest.get_visual("rest_base")
    lockout_housing = lockout.get_visual("lockout_housing")

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
        "platter spindle axis vertical",
        platter_joint.axis == (0.0, 0.0, 1.0),
        f"expected platter axis (0, 0, 1), got {platter_joint.axis}",
    )
    ctx.check(
        "tonearm pivot axis vertical",
        tonearm_joint.axis == (0.0, 0.0, 1.0),
        f"expected tonearm axis (0, 0, 1), got {tonearm_joint.axis}",
    )

    ctx.expect_contact(
        spindle_support,
        plinth,
        elem_a=bearing_base,
        elem_b=top_deck,
        name="bearing support anchored to plinth deck",
    )
    ctx.expect_contact(
        platter,
        spindle_support,
        elem_a=hub_mount,
        elem_b=spindle_pedestal,
        name="platter hub seats on spindle support",
    )
    ctx.expect_contact(
        tonearm_support,
        plinth,
        elem_a=support_base,
        elem_b=top_deck,
        name="tonearm support anchored to plinth deck",
    )
    ctx.expect_contact(
        tonearm_stage,
        tonearm_support,
        elem_a=tonearm_mount,
        elem_b=arm_pedestal,
        name="tonearm stage seats on pivot pedestal",
    )
    ctx.expect_contact(
        guard,
        plinth,
        elem_a=guard_foot,
        elem_b=top_deck,
        name="guard foot lands on plinth deck",
    )
    ctx.expect_contact(
        arm_rest,
        plinth,
        elem_a=rest_base,
        elem_b=top_deck,
        name="arm rest mounted to plinth deck",
    )
    ctx.expect_contact(
        lockout,
        plinth,
        elem_a=lockout_housing,
        elem_b=top_deck,
        name="lockout housing mounted to plinth deck",
    )
    ctx.expect_gap(
        guard,
        platter,
        axis="y",
        min_gap=0.03,
        positive_elem=rear_shield,
        negative_elem=outer_rim,
        name="rear guard clears spinning platter",
    )

    with ctx.pose({tonearm_joint: 0.0}):
        ctx.expect_gap(
            tonearm_stage,
            platter,
            axis="x",
            min_gap=0.12,
            positive_elem=headshell,
            negative_elem=record_mat,
            name="parked headshell clears platter",
        )

    with ctx.pose({tonearm_joint: 1.60}):
        ctx.expect_overlap(
            tonearm_stage,
            platter,
            axes="xy",
            min_overlap=0.02,
            elem_a=headshell,
            elem_b=record_mat,
            name="operating headshell reaches record area",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
