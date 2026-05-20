from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def _box_visual(part, name, size, xyz, material):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _cylinder_y_visual(part, name, radius, length, xyz, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _cylinder_z_visual(part, name, radius, length, xyz, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_sliding_security_gate", assets=ASSETS)

    matte_charcoal = model.material("matte_charcoal", rgba=(0.17, 0.18, 0.19, 1.0))
    satin_graphite = model.material("satin_graphite", rgba=(0.28, 0.30, 0.32, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.63, 0.66, 0.69, 1.0))
    roller_polymer = model.material("roller_polymer", rgba=(0.10, 0.10, 0.11, 1.0))
    latch_dark = model.material("latch_dark", rgba=(0.14, 0.15, 0.16, 1.0))

    support = model.part("support_frame")

    _box_visual(
        support,
        "foundation_beam",
        (4.35, 0.28, 0.08),
        (-1.125, 0.0, 0.04),
        matte_charcoal,
    )
    _box_visual(
        support,
        "ground_track_base",
        (3.45, 0.10, 0.018),
        (-1.05, 0.0, 0.089),
        satin_graphite,
    )
    _box_visual(
        support,
        "track_wear_strip",
        (3.45, 0.038, 0.008),
        (-1.05, 0.0, 0.102),
        satin_metal,
    )
    _box_visual(
        support,
        "track_curb_front",
        (3.45, 0.012, 0.012),
        (-1.05, 0.043, 0.095),
        matte_charcoal,
    )
    _box_visual(
        support,
        "track_curb_back",
        (3.45, 0.012, 0.012),
        (-1.05, -0.043, 0.095),
        matte_charcoal,
    )
    _box_visual(
        support,
        "guide_post_front",
        (0.12, 0.050, 1.72),
        (-1.06, 0.088, 0.94),
        matte_charcoal,
    )
    _box_visual(
        support,
        "guide_post_back",
        (0.12, 0.050, 1.72),
        (-1.06, -0.088, 0.94),
        matte_charcoal,
    )
    _box_visual(
        support,
        "right_latch_post",
        (0.12, 0.16, 1.72),
        (0.95, 0.0, 0.94),
        matte_charcoal,
    )
    _box_visual(
        support,
        "guide_bridge",
        (0.24, 0.226, 0.04),
        (-1.07, 0.0, 1.62),
        satin_graphite,
    )
    _box_visual(
        support,
        "guide_front_arm",
        (0.035, 0.024, 0.072),
        (-1.088, 0.049, 1.565),
        satin_graphite,
    )
    _box_visual(
        support,
        "guide_back_arm",
        (0.035, 0.024, 0.072),
        (-1.088, -0.049, 1.565),
        satin_graphite,
    )
    _cylinder_z_visual(
        support,
        "guide_front_roller",
        radius=0.022,
        length=0.045,
        xyz=(-1.088, 0.053, 1.53),
        material=roller_polymer,
    )
    _cylinder_z_visual(
        support,
        "guide_back_roller",
        radius=0.022,
        length=0.045,
        xyz=(-1.088, -0.053, 1.53),
        material=roller_polymer,
    )
    _box_visual(
        support,
        "receiver_back_plate",
        (0.016, 0.060, 0.14),
        (0.89, 0.0, 0.93),
        satin_metal,
    )
    _box_visual(
        support,
        "receiver_trim_plate",
        (0.010, 0.090, 0.30),
        (0.885, 0.0, 0.93),
        satin_graphite,
    )
    _box_visual(
        support,
        "receiver_front_jaw",
        (0.05, 0.010, 0.14),
        (0.873, 0.021, 0.93),
        latch_dark,
    )
    _box_visual(
        support,
        "receiver_back_jaw",
        (0.05, 0.010, 0.14),
        (0.873, -0.021, 0.93),
        latch_dark,
    )
    _box_visual(
        support,
        "close_stop_pad",
        (0.02, 0.05, 0.06),
        (0.905, 0.0, 0.22),
        roller_polymer,
    )

    gate = model.part("gate_leaf")

    _box_visual(gate, "bottom_rail", (2.06, 0.06, 0.10), (0.0, 0.0, 0.23), satin_graphite)
    _box_visual(gate, "top_guide_rail", (2.06, 0.06, 0.08), (0.0, 0.0, 1.53), satin_graphite)
    _box_visual(gate, "trailing_stile", (0.06, 0.06, 1.34), (-1.0, 0.0, 0.90), satin_graphite)
    _box_visual(gate, "leading_stile", (0.06, 0.06, 1.34), (1.0, 0.0, 0.90), satin_graphite)
    _box_visual(gate, "mid_rail", (1.94, 0.03, 0.05), (0.0, 0.0, 0.84), satin_metal)
    _box_visual(gate, "bottom_inner_trim", (1.90, 0.026, 0.022), (0.0, 0.0, 0.31), satin_metal)
    _box_visual(gate, "top_inner_trim", (1.90, 0.026, 0.018), (0.0, 0.0, 1.45), satin_metal)
    _box_visual(gate, "leading_face_plate", (0.016, 0.048, 0.28), (0.964, 0.0, 0.93), matte_charcoal)
    _box_visual(gate, "pull_grip_front", (0.012, 0.006, 0.18), (0.944, 0.018, 0.93), satin_metal)
    _box_visual(gate, "pull_grip_back", (0.012, 0.006, 0.18), (0.944, -0.018, 0.93), satin_metal)
    _box_visual(gate, "bottom_drain_cover", (1.82, 0.018, 0.012), (0.0, 0.0, 0.275), matte_charcoal)
    _box_visual(gate, "top_shadow_strip", (1.82, 0.018, 0.012), (0.0, 0.0, 1.484), matte_charcoal)

    slat_centers = (
        -0.84,
        -0.70,
        -0.56,
        -0.42,
        -0.28,
        -0.14,
        0.0,
        0.14,
        0.28,
        0.42,
        0.56,
        0.70,
        0.84,
    )
    for index, x_pos in enumerate(slat_centers, start=1):
        _box_visual(
            gate,
            f"slat_{index:02d}",
            (0.028, 0.020, 1.15),
            (x_pos, 0.0, 0.885),
            satin_metal,
        )

    wheel_x_positions = {"rear": -0.60, "front": 0.52}
    for label, x_pos in wheel_x_positions.items():
        _box_visual(
            gate,
            f"{label}_hanger_front",
            (0.065, 0.010, 0.10),
            (x_pos, 0.022, 0.156),
            satin_graphite,
        )
        _box_visual(
            gate,
            f"{label}_hanger_back",
            (0.065, 0.010, 0.10),
            (x_pos, -0.022, 0.156),
            satin_graphite,
        )
        _cylinder_y_visual(
            gate,
            f"{label}_wheel_axle",
            radius=0.006,
            length=0.055,
            xyz=(x_pos, 0.0, 0.139),
            material=satin_metal,
        )
        _cylinder_y_visual(
            gate,
            f"{label}_wheel",
            radius=0.033,
            length=0.028,
            xyz=(x_pos, 0.0, 0.139),
            material=roller_polymer,
        )

    _box_visual(gate, "latch_tongue", (0.045, 0.028, 0.10), (1.038, 0.0, 0.93), satin_metal)
    _box_visual(gate, "close_bumper", (0.018, 0.042, 0.05), (0.99, 0.0, 0.22), roller_polymer)

    model.articulation(
        "leaf_slide",
        ArticulationType.PRISMATIC,
        parent=support,
        child=gate,
        origin=Origin(xyz=(-0.185, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1600.0,
            velocity=0.55,
            lower=0.0,
            upper=1.86,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    support = object_model.get_part("support_frame")
    gate = object_model.get_part("gate_leaf")
    slide = object_model.get_articulation("leaf_slide")

    track_wear_strip = support.get_visual("track_wear_strip")
    right_latch_post = support.get_visual("right_latch_post")
    guide_front_roller = support.get_visual("guide_front_roller")
    guide_back_roller = support.get_visual("guide_back_roller")
    receiver_back_plate = support.get_visual("receiver_back_plate")
    receiver_front_jaw = support.get_visual("receiver_front_jaw")
    receiver_back_jaw = support.get_visual("receiver_back_jaw")

    top_guide_rail = gate.get_visual("top_guide_rail")
    leading_stile = gate.get_visual("leading_stile")
    front_wheel = gate.get_visual("front_wheel")
    rear_wheel = gate.get_visual("rear_wheel")
    latch_tongue = gate.get_visual("latch_tongue")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=16, name="leaf_clearance_across_travel")

    with ctx.pose({slide: 0.0}):
        ctx.expect_gap(
            gate,
            support,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=front_wheel,
            negative_elem=track_wear_strip,
            name="front_wheel_seats_on_track_closed",
        )
        ctx.expect_gap(
            gate,
            support,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=rear_wheel,
            negative_elem=track_wear_strip,
            name="rear_wheel_seats_on_track_closed",
        )
        ctx.expect_overlap(
            gate,
            support,
            axes="x",
            min_overlap=0.02,
            elem_a=front_wheel,
            elem_b=track_wear_strip,
            name="front_wheel_stays_over_track_closed",
        )
        ctx.expect_overlap(
            gate,
            support,
            axes="x",
            min_overlap=0.02,
            elem_a=rear_wheel,
            elem_b=track_wear_strip,
            name="rear_wheel_stays_over_track_closed",
        )
        ctx.expect_gap(
            support,
            gate,
            axis="y",
            min_gap=0.0005,
            max_gap=0.002,
            positive_elem=guide_front_roller,
            negative_elem=top_guide_rail,
            name="front_guide_roller_clearance_closed",
        )
        ctx.expect_gap(
            gate,
            support,
            axis="y",
            min_gap=0.0005,
            max_gap=0.002,
            positive_elem=top_guide_rail,
            negative_elem=guide_back_roller,
            name="back_guide_roller_clearance_closed",
        )
        ctx.expect_overlap(
            gate,
            support,
            axes="z",
            min_overlap=0.035,
            elem_a=top_guide_rail,
            elem_b=guide_front_roller,
            name="front_guide_vertical_capture_closed",
        )
        ctx.expect_overlap(
            gate,
            support,
            axes="z",
            min_overlap=0.035,
            elem_a=top_guide_rail,
            elem_b=guide_back_roller,
            name="back_guide_vertical_capture_closed",
        )
        ctx.expect_gap(
            support,
            gate,
            axis="x",
            min_gap=0.004,
            max_gap=0.014,
            positive_elem=receiver_back_plate,
            negative_elem=latch_tongue,
            name="latch_tongue_seats_near_receiver_closed",
        )
        ctx.expect_overlap(
            gate,
            support,
            axes="yz",
            min_overlap=0.02,
            elem_a=latch_tongue,
            elem_b=receiver_back_plate,
            name="latch_tongue_aligned_with_receiver_closed",
        )
        ctx.expect_gap(
            support,
            gate,
            axis="y",
            min_gap=0.001,
            max_gap=0.0035,
            positive_elem=receiver_front_jaw,
            negative_elem=latch_tongue,
            name="latch_front_jaw_clearance_closed",
        )
        ctx.expect_gap(
            gate,
            support,
            axis="y",
            min_gap=0.001,
            max_gap=0.0035,
            positive_elem=latch_tongue,
            negative_elem=receiver_back_jaw,
            name="latch_back_jaw_clearance_closed",
        )

    with ctx.pose({slide: 1.86}):
        ctx.expect_gap(
            gate,
            support,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=front_wheel,
            negative_elem=track_wear_strip,
            name="front_wheel_seats_on_track_open",
        )
        ctx.expect_gap(
            gate,
            support,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=rear_wheel,
            negative_elem=track_wear_strip,
            name="rear_wheel_seats_on_track_open",
        )
        ctx.expect_overlap(
            gate,
            support,
            axes="x",
            min_overlap=0.02,
            elem_a=front_wheel,
            elem_b=track_wear_strip,
            name="front_wheel_stays_over_track_open",
        )
        ctx.expect_overlap(
            gate,
            support,
            axes="x",
            min_overlap=0.02,
            elem_a=rear_wheel,
            elem_b=track_wear_strip,
            name="rear_wheel_stays_over_track_open",
        )
        ctx.expect_gap(
            support,
            gate,
            axis="y",
            min_gap=0.0005,
            max_gap=0.002,
            positive_elem=guide_front_roller,
            negative_elem=top_guide_rail,
            name="front_guide_roller_clearance_open",
        )
        ctx.expect_gap(
            gate,
            support,
            axis="y",
            min_gap=0.0005,
            max_gap=0.002,
            positive_elem=top_guide_rail,
            negative_elem=guide_back_roller,
            name="back_guide_roller_clearance_open",
        )
        ctx.expect_gap(
            support,
            gate,
            axis="x",
            min_gap=1.75,
            positive_elem=right_latch_post,
            negative_elem=leading_stile,
            name="gate_clears_latch_post_when_open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
