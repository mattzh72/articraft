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
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="travel_speaker")

    shell_dark = model.material("shell_dark", rgba=(0.19, 0.20, 0.22, 1.0))
    grille_fabric = model.material("grille_fabric", rgba=(0.11, 0.12, 0.13, 1.0))
    strip_black = model.material("strip_black", rgba=(0.05, 0.05, 0.06, 1.0))
    loop_rubber = model.material("loop_rubber", rgba=(0.24, 0.25, 0.27, 1.0))
    switch_accent = model.material("switch_accent", rgba=(0.86, 0.45, 0.12, 1.0))

    body_len = 0.182
    body_depth = 0.064
    body_height = 0.074
    body_half_x = body_len * 0.5
    body_half_y = body_depth * 0.5
    body_half_z = body_height * 0.5

    speaker_body = model.part("speaker_body")

    shell_geom = (
        ExtrudeGeometry(
            rounded_rect_profile(body_height, body_depth, radius=0.014, corner_segments=8),
            body_len,
            center=True,
        )
        .rotate_y(math.pi / 2.0)
    )
    speaker_body.visual(
        mesh_from_geometry(shell_geom, "speaker_body_shell"),
        material=shell_dark,
        name="body_shell",
    )

    speaker_body.visual(
        Box((0.150, 0.003, 0.054)),
        origin=Origin(xyz=(0.000, body_half_y + 0.0015, 0.000)),
        material=grille_fabric,
        name="front_grille",
    )
    speaker_body.visual(
        Box((0.150, 0.003, 0.054)),
        origin=Origin(xyz=(0.000, -(body_half_y + 0.0015), 0.000)),
        material=grille_fabric,
        name="rear_grille",
    )
    speaker_body.visual(
        Box((0.120, 0.022, 0.003)),
        origin=Origin(xyz=(-0.004, 0.000, -body_half_z - 0.0005)),
        material=strip_black,
        name="base_pad",
    )

    track_center_x = -0.006
    track_floor_size = (0.048, 0.018, 0.0025)
    track_floor_center_z = body_half_z + 0.00115
    track_floor_top_z = track_floor_center_z + (track_floor_size[2] * 0.5)
    speaker_body.visual(
        Box(track_floor_size),
        origin=Origin(xyz=(track_center_x, 0.000, track_floor_center_z)),
        material=strip_black,
        name="switch_track_floor",
    )
    speaker_body.visual(
        Box((0.110, 0.006, 0.004)),
        origin=Origin(xyz=(track_center_x, 0.010, body_half_z + 0.004)),
        material=strip_black,
        name="control_strip_front_rail",
    )
    speaker_body.visual(
        Box((0.110, 0.006, 0.004)),
        origin=Origin(xyz=(track_center_x, -0.010, body_half_z + 0.004)),
        material=strip_black,
        name="control_strip_rear_rail",
    )
    speaker_body.visual(
        Box((0.014, 0.018, 0.004)),
        origin=Origin(xyz=(-0.050, 0.000, body_half_z + 0.004)),
        material=strip_black,
        name="control_strip_left_cap",
    )
    speaker_body.visual(
        Box((0.022, 0.018, 0.004)),
        origin=Origin(xyz=(0.044, 0.000, body_half_z + 0.004)),
        material=strip_black,
        name="control_strip_right_cap",
    )
    speaker_body.visual(
        Box((0.004, 0.014, 0.004)),
        origin=Origin(xyz=(-0.026, 0.000, body_half_z + 0.004)),
        material=shell_dark,
        name="switch_slot_left_stop",
    )
    speaker_body.visual(
        Box((0.004, 0.014, 0.004)),
        origin=Origin(xyz=(0.014, 0.000, body_half_z + 0.004)),
        material=shell_dark,
        name="switch_slot_right_stop",
    )

    pivot_mount_x = body_half_x + 0.003
    pivot_z = 0.010
    pivot_span_y = 0.020
    speaker_body.visual(
        Box((0.006, 0.010, 0.018)),
        origin=Origin(xyz=(pivot_mount_x, pivot_span_y, pivot_z)),
        material=shell_dark,
        name="loop_mount_front",
    )
    speaker_body.visual(
        Cylinder(radius=0.005, length=0.010),
        origin=Origin(
            xyz=(pivot_mount_x + 0.002, pivot_span_y, pivot_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=strip_black,
        name="loop_pivot_boss_front",
    )
    speaker_body.visual(
        Box((0.006, 0.010, 0.018)),
        origin=Origin(xyz=(pivot_mount_x, -pivot_span_y, pivot_z)),
        material=shell_dark,
        name="loop_mount_rear",
    )
    speaker_body.visual(
        Cylinder(radius=0.005, length=0.010),
        origin=Origin(
            xyz=(pivot_mount_x + 0.002, -pivot_span_y, pivot_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=strip_black,
        name="loop_pivot_boss_rear",
    )

    speaker_body.inertial = Inertial.from_geometry(
        Box((body_len + 0.010, body_depth + 0.010, body_height + 0.010)),
        mass=1.0,
    )

    carry_loop = model.part("carry_loop")
    loop_geom = tube_from_spline_points(
        [
            (0.000, -pivot_span_y, 0.000),
            (0.026, -0.024, 0.011),
            (0.076, 0.000, 0.020),
            (0.026, 0.024, 0.011),
            (0.000, pivot_span_y, 0.000),
        ],
        radius=0.0046,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    loop_geom.merge(
        CylinderGeometry(radius=0.0040, height=0.008)
        .rotate_x(math.pi / 2.0)
        .translate(0.000, -pivot_span_y, 0.000)
    )
    loop_geom.merge(
        CylinderGeometry(radius=0.0040, height=0.008)
        .rotate_x(math.pi / 2.0)
        .translate(0.000, pivot_span_y, 0.000)
    )
    carry_loop.visual(
        mesh_from_geometry(loop_geom, "speaker_carry_loop"),
        material=loop_rubber,
        name="loop_band",
    )
    carry_loop.inertial = Inertial.from_geometry(
        Box((0.060, 0.060, 0.060)),
        mass=0.08,
        origin=Origin(xyz=(0.024, 0.000, 0.020)),
    )

    power_switch = model.part("power_switch")
    power_switch.visual(
        Box((0.014, 0.012, 0.004)),
        origin=Origin(xyz=(0.000, 0.000, 0.002)),
        material=switch_accent,
        name="switch_body",
    )
    power_switch.visual(
        Box((0.008, 0.004, 0.0025)),
        origin=Origin(xyz=(0.001, 0.000, 0.0052)),
        material=shell_dark,
        name="switch_ridge",
    )
    power_switch.inertial = Inertial.from_geometry(
        Box((0.016, 0.014, 0.008)),
        mass=0.02,
        origin=Origin(xyz=(0.000, 0.000, 0.003)),
    )

    model.articulation(
        "body_to_carry_loop",
        ArticulationType.REVOLUTE,
        parent=speaker_body,
        child=carry_loop,
        origin=Origin(xyz=(body_half_x + 0.014, 0.000, pivot_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(115.0),
        ),
    )

    model.articulation(
        "body_to_power_switch",
        ArticulationType.PRISMATIC,
        parent=speaker_body,
        child=power_switch,
        origin=Origin(xyz=(-0.015, 0.000, track_floor_top_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.08,
            lower=0.0,
            upper=0.018,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    speaker_body = object_model.get_part("speaker_body")
    carry_loop = object_model.get_part("carry_loop")
    power_switch = object_model.get_part("power_switch")
    loop_hinge = object_model.get_articulation("body_to_carry_loop")
    switch_slide = object_model.get_articulation("body_to_power_switch")

    ctx.expect_contact(
        power_switch,
        speaker_body,
        elem_a="switch_body",
        elem_b="switch_track_floor",
        contact_tol=0.0005,
        name="power switch rides on the guide floor",
    )
    ctx.expect_gap(
        power_switch,
        speaker_body,
        axis="x",
        positive_elem="switch_body",
        negative_elem="switch_slot_left_stop",
        min_gap=0.002,
        max_gap=0.006,
        name="power switch starts clear of the left slot stop",
    )
    ctx.expect_gap(
        carry_loop,
        speaker_body,
        axis="x",
        positive_elem="loop_band",
        negative_elem="loop_mount_front",
        min_gap=0.0,
        max_gap=0.012,
        name="carry loop stays mounted close to the end-cap pivots",
    )

    switch_rest = ctx.part_world_position(power_switch)
    with ctx.pose({switch_slide: switch_slide.motion_limits.upper}):
        switch_on = ctx.part_world_position(power_switch)
        ctx.expect_gap(
            speaker_body,
            power_switch,
            axis="x",
            positive_elem="switch_slot_right_stop",
            negative_elem="switch_body",
            min_gap=0.002,
            max_gap=0.006,
            name="power switch stops before the right end of the guide slot",
        )

    ctx.check(
        "power switch translates along the control strip",
        switch_rest is not None
        and switch_on is not None
        and switch_on[0] > switch_rest[0] + 0.015
        and abs(switch_on[1] - switch_rest[1]) < 1e-6
        and abs(switch_on[2] - switch_rest[2]) < 1e-6,
        details=f"rest={switch_rest}, on={switch_on}",
    )

    loop_rest_aabb = ctx.part_element_world_aabb(carry_loop, elem="loop_band")
    body_aabb = ctx.part_world_aabb(speaker_body)
    with ctx.pose({loop_hinge: loop_hinge.motion_limits.upper}):
        loop_open_aabb = ctx.part_element_world_aabb(carry_loop, elem="loop_band")

    ctx.check(
        "carry loop rotates upward above the speaker body",
        loop_rest_aabb is not None
        and loop_open_aabb is not None
        and body_aabb is not None
        and loop_open_aabb[1][2] > body_aabb[1][2] + 0.020
        and loop_open_aabb[1][2] > loop_rest_aabb[1][2] + 0.015,
        details=f"rest={loop_rest_aabb}, open={loop_open_aabb}, body={body_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
