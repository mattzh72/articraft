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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _box(part, name, size, center, material):
    return part.visual(
        Box(size),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def _cyl_x(part, name, radius, length, center, material):
    return part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(0.0, pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_roof_vent_tower")

    body_mat = model.material("tower_body_finish", rgba=(0.23, 0.25, 0.28, 1.0))
    flap_mat = model.material("weather_flap_finish", rgba=(0.68, 0.70, 0.73, 1.0))
    hinge_mat = model.material("hinge_hardware", rgba=(0.53, 0.54, 0.56, 1.0))

    foot_w = 0.19
    foot_d = 0.15
    foot_t = 0.006

    tower_w = 0.15
    tower_d = 0.11
    curb_t = 0.008
    curb_h = 0.028
    post_t = 0.012
    post_h = 0.072
    top_rail_t = 0.012
    top_rail_h = 0.016

    rear_rail_d = 0.016
    hinge_axis_y = tower_d / 2.0
    hinge_axis_z = foot_t + curb_h + post_h + top_rail_h + 0.007

    body = model.part("tower_body")
    _box(body, "base_flange", (foot_w, foot_d, foot_t), (0.0, 0.0, foot_t / 2.0), body_mat)

    curb_center_z = foot_t + curb_h / 2.0
    _box(
        body,
        "front_curb",
        (tower_w, curb_t, curb_h),
        (0.0, -(tower_d / 2.0 - curb_t / 2.0), curb_center_z),
        body_mat,
    )
    _box(
        body,
        "rear_curb",
        (tower_w, curb_t, curb_h),
        (0.0, tower_d / 2.0 - curb_t / 2.0, curb_center_z),
        body_mat,
    )
    _box(
        body,
        "left_curb",
        (curb_t, tower_d, curb_h),
        (-(tower_w / 2.0 - curb_t / 2.0), 0.0, curb_center_z),
        body_mat,
    )
    _box(
        body,
        "right_curb",
        (curb_t, tower_d, curb_h),
        (tower_w / 2.0 - curb_t / 2.0, 0.0, curb_center_z),
        body_mat,
    )

    post_center_z = foot_t + curb_h + post_h / 2.0
    for side_name, sx in (("left", -1.0), ("right", 1.0)):
        for end_name, sy in (("front", -1.0), ("rear", 1.0)):
            _box(
                body,
                f"{end_name}_{side_name}_post",
                (post_t, post_t, post_h),
                (
                    sx * (tower_w / 2.0 - post_t / 2.0),
                    sy * (tower_d / 2.0 - post_t / 2.0),
                    post_center_z,
                ),
                body_mat,
            )

    top_rail_center_z = foot_t + curb_h + post_h + top_rail_h / 2.0
    _box(
        body,
        "front_top_rail",
        (tower_w, top_rail_t, top_rail_h),
        (0.0, -(tower_d / 2.0 - top_rail_t / 2.0), top_rail_center_z),
        body_mat,
    )
    _box(
        body,
        "left_top_rail",
        (top_rail_t, tower_d, top_rail_h),
        (-(tower_w / 2.0 - top_rail_t / 2.0), 0.0, top_rail_center_z),
        body_mat,
    )
    _box(
        body,
        "right_top_rail",
        (top_rail_t, tower_d, top_rail_h),
        (tower_w / 2.0 - top_rail_t / 2.0, 0.0, top_rail_center_z),
        body_mat,
    )
    _box(
        body,
        "rear_hinge_rail",
        (tower_w, rear_rail_d, top_rail_h + 0.004),
        (0.0, tower_d / 2.0 - rear_rail_d / 2.0, top_rail_center_z + 0.002),
        body_mat,
    )

    for side_name, sx in (("left", -1.0), ("right", 1.0)):
        _box(
            body,
            f"{side_name}_hinge_plate",
            (0.014, 0.014, 0.020),
            (sx * 0.046, hinge_axis_y - 0.004, hinge_axis_z - 0.010),
            hinge_mat,
        )
        _cyl_x(
            body,
            f"{side_name}_barrel",
            radius=0.006,
            length=0.028,
            center=(sx * 0.046, hinge_axis_y, hinge_axis_z),
            material=hinge_mat,
        )

    _box(
        body,
        "right_stop_post",
        (0.010, 0.012, 0.028),
        (0.058, hinge_axis_y + 0.007, hinge_axis_z + 0.014),
        hinge_mat,
    )
    _box(
        body,
        "right_stop_pad",
        (0.020, 0.010, 0.008),
        (0.068, hinge_axis_y + 0.013, hinge_axis_z + 0.031),
        hinge_mat,
    )

    flap = model.part("weather_flap")
    _box(flap, "skin", (0.166, 0.100, 0.004), (0.0, -0.058, -0.001), flap_mat)
    _box(flap, "front_lip", (0.166, 0.010, 0.012), (0.0, -0.113, -0.008), flap_mat)
    _box(flap, "left_skirt", (0.006, 0.098, 0.010), (-0.086, -0.055, -0.008), flap_mat)
    _box(flap, "right_skirt", (0.006, 0.098, 0.010), (0.086, -0.055, -0.008), flap_mat)
    _box(flap, "rear_seal_rail", (0.060, 0.008, 0.008), (0.0, -0.004, 0.004), flap_mat)
    _box(flap, "hinge_leaf", (0.060, 0.012, 0.010), (0.0, 0.004, 0.001), hinge_mat)
    _cyl_x(flap, "center_barrel", radius=0.0054, length=0.052, center=(0.0, 0.0, 0.0), material=hinge_mat)
    _box(flap, "right_stop_strut", (0.008, 0.020, 0.018), (0.080, -0.018, 0.004), hinge_mat)

    model.articulation(
        "tower_to_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flap,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=0.0, upper=1.02),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("tower_body")
    flap = object_model.get_part("weather_flap")
    hinge = object_model.get_articulation("tower_to_flap")

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

    limits = hinge.motion_limits
    ctx.check(
        "hinge_axis_and_range",
        hinge.axis == (-1.0, 0.0, 0.0)
        and limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and 0.95 <= limits.upper <= 1.10,
        details="Flap should open upward from a rear hinge line through a compact but useful range.",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            flap,
            body,
            axis="z",
            positive_elem="skin",
            negative_elem="front_top_rail",
            min_gap=0.003,
            max_gap=0.005,
            name="closed_flap_stows_just_above_front_rail",
        )
        ctx.expect_gap(
            flap,
            body,
            axis="z",
            positive_elem="skin",
            negative_elem="rear_hinge_rail",
            min_gap=0.0,
            max_gap=0.0005,
            name="closed_flap_seats_on_rear_hinge_rail",
        )
        ctx.expect_overlap(
            flap,
            body,
            axes="xy",
            min_overlap=0.10,
            elem_a="skin",
            name="closed_flap_covers_tower_top",
        )
        ctx.expect_gap(
            flap,
            body,
            axis="x",
            positive_elem="right_skirt",
            negative_elem="right_top_rail",
            min_gap=0.0075,
            max_gap=0.0125,
            name="right_skirt_has_side_clearance",
        )

    with ctx.pose({hinge: 1.02}):
        ctx.expect_gap(
            flap,
            body,
            axis="z",
            positive_elem="front_lip",
            negative_elem="rear_hinge_rail",
            min_gap=0.040,
            name="front_lip_lifts_clear_when_open",
        )
        ctx.expect_gap(
            body,
            flap,
            axis="y",
            positive_elem="right_stop_pad",
            negative_elem="right_stop_strut",
            min_gap=0.0005,
            max_gap=0.003,
            name="open_stop_strut_approaches_stop_pad_without_overlap",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
