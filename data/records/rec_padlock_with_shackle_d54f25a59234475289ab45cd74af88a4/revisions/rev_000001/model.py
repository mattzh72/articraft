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
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _build_body_shell(width: float, depth: float, height: float):
    shell = ExtrudeGeometry.centered(
        rounded_rect_profile(width, height, radius=0.004, corner_segments=8),
        depth,
        cap=True,
        closed=True,
    )
    shell.rotate_x(math.pi / 2.0).translate(0.0, 0.0, height / 2.0)
    return mesh_from_geometry(shell, "padlock_body_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="combination_padlock")

    body_width = 0.040
    body_depth = 0.018
    body_height = 0.038
    shackle_span = 0.024
    shackle_rise = 0.029
    shackle_radius = 0.0024

    dial_count = 4
    dial_outer_radius = 0.0040
    dial_band_radius = 0.0033
    dial_length = 0.0046
    dial_band_length = 0.0020
    dial_rim_length = (dial_length - dial_band_length) / 2.0
    shaft_radius = 0.0014
    shaft_length = 0.0018
    dial_z = 0.019
    dial_pitch = 0.0085
    dial_x0 = -0.5 * dial_pitch * (dial_count - 1)
    seat_size = 0.0036
    seat_height = 0.0012
    tip_size = 0.0032

    brass = model.material("brass_body", rgba=(0.74, 0.61, 0.22, 1.0))
    brass_dark = model.material("brass_plate", rgba=(0.56, 0.45, 0.16, 1.0))
    steel = model.material("steel", rgba=(0.82, 0.84, 0.87, 1.0))
    dial_black = model.material("dial_black", rgba=(0.14, 0.14, 0.16, 1.0))
    numeral_band = model.material("numeral_band", rgba=(0.88, 0.87, 0.80, 1.0))

    body = model.part("body")
    body.visual(
        _build_body_shell(body_width, body_depth, body_height),
        material=brass,
        name="body_shell",
    )
    body.visual(
        Box((0.034, 0.0012, 0.017)),
        origin=Origin(xyz=(0.0, body_depth / 2.0 + 0.0006, dial_z)),
        material=brass_dark,
        name="front_plate",
    )
    body.visual(
        Box((0.030, body_depth * 0.92, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=brass_dark,
        name="bottom_cap",
    )

    dial_x_positions = [dial_x0 + i * dial_pitch for i in range(dial_count)]
    for index, dial_x in enumerate(dial_x_positions, start=1):
        body.visual(
            Cylinder(radius=shaft_radius, length=shaft_length),
            origin=Origin(
                xyz=(dial_x, body_depth / 2.0 + shaft_length / 2.0, dial_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=brass_dark,
            name=f"dial_shaft_{index}",
        )
    body.visual(
        Box((seat_size, seat_size, seat_height)),
        origin=Origin(
            xyz=(shackle_span / 2.0, 0.0, body_height - seat_height / 2.0)
        ),
        material=brass_dark,
        name="pivot_seat",
    )
    body.visual(
        Box((seat_size, seat_size, seat_height)),
        origin=Origin(
            xyz=(-shackle_span / 2.0, 0.0, body_height - seat_height / 2.0)
        ),
        material=brass_dark,
        name="free_seat",
    )

    shackle = model.part("shackle")
    shackle.visual(
        Cylinder(radius=shackle_radius, length=shackle_rise),
        origin=Origin(xyz=(0.0, 0.0, shackle_rise / 2.0)),
        material=steel,
        name="pivot_leg",
    )
    shackle.visual(
        Cylinder(radius=shackle_radius, length=shackle_rise),
        origin=Origin(xyz=(-shackle_span, 0.0, shackle_rise / 2.0)),
        material=steel,
        name="free_leg",
    )
    shackle.visual(
        Cylinder(radius=shackle_radius, length=shackle_span),
        origin=Origin(
            xyz=(-shackle_span / 2.0, 0.0, shackle_rise),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="top_bar",
    )
    shackle.visual(
        Cylinder(radius=0.0030, length=seat_height),
        origin=Origin(xyz=(0.0, 0.0, seat_height / 2.0)),
        material=steel,
        name="pivot_collar",
    )
    shackle.visual(
        Box((tip_size, tip_size, seat_height)),
        origin=Origin(xyz=(0.0, 0.0, seat_height / 2.0)),
        material=steel,
        name="pivot_pad",
    )
    shackle.visual(
        Box((tip_size, tip_size, seat_height)),
        origin=Origin(xyz=(-shackle_span, 0.0, seat_height / 2.0)),
        material=steel,
        name="free_tip",
    )
    model.articulation(
        "body_to_shackle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=shackle,
        origin=Origin(
            xyz=(
                shackle_span / 2.0,
                0.0,
                body_height,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=0.0,
            upper=1.25,
        ),
    )

    for index, dial_x in enumerate(dial_x_positions, start=1):
        dial = model.part(f"dial_{index}")
        dial.visual(
            Cylinder(radius=dial_outer_radius, length=dial_rim_length),
            origin=Origin(
                xyz=(0.0, -dial_band_length / 2.0 - dial_rim_length / 2.0, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dial_black,
            name="rim_left",
        )
        dial.visual(
            Cylinder(radius=dial_band_radius, length=dial_band_length),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=numeral_band,
            name="number_band",
        )
        dial.visual(
            Cylinder(radius=dial_outer_radius, length=dial_rim_length),
            origin=Origin(
                xyz=(0.0, dial_band_length / 2.0 + dial_rim_length / 2.0, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dial_black,
            name="rim_right",
        )
        model.articulation(
            f"body_to_dial_{index}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=dial,
            origin=Origin(
                xyz=(
                    dial_x,
                    body_depth / 2.0 + shaft_length + dial_length / 2.0,
                    dial_z,
                )
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.5, velocity=8.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    shackle = object_model.get_part("shackle")
    shackle_joint = object_model.get_articulation("body_to_shackle")
    pivot_seat = body.get_visual("pivot_seat")
    free_seat = body.get_visual("free_seat")
    pivot_pad = shackle.get_visual("pivot_pad")
    free_tip = shackle.get_visual("free_tip")
    dial_parts = [object_model.get_part(f"dial_{index}") for index in range(1, 5)]
    dial_joints = [
        object_model.get_articulation(f"body_to_dial_{index}") for index in range(1, 5)
    ]

    def axis_matches(actual, expected) -> bool:
        return all(abs(a - b) < 1e-9 for a, b in zip(actual, expected))

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
    ctx.fail_if_articulation_overlaps(
        max_pose_samples=32,
        name="moving_parts_clear_through_motion",
    )

    ctx.check(
        "shackle_joint_axis_is_vertical_captive_leg",
        axis_matches(shackle_joint.axis, (0.0, 0.0, 1.0)),
        details=f"expected (0, 0, 1), got {shackle_joint.axis}",
    )
    ctx.check(
        "shackle_joint_is_bounded_revolute",
        shackle_joint.joint_type == ArticulationType.REVOLUTE
        and shackle_joint.motion_limits is not None
        and shackle_joint.motion_limits.lower == 0.0
        and shackle_joint.motion_limits.upper is not None
        and 0.9 <= shackle_joint.motion_limits.upper <= 1.4,
        details="shackle should be a realistic bounded revolute opening joint",
    )

    for index, dial_joint in enumerate(dial_joints, start=1):
        ctx.check(
            f"dial_{index}_axis_is_front_to_back",
            axis_matches(dial_joint.axis, (0.0, 1.0, 0.0)),
            details=f"expected (0, 1, 0), got {dial_joint.axis}",
        )
        ctx.check(
            f"dial_{index}_is_continuous",
            dial_joint.joint_type == ArticulationType.CONTINUOUS,
            details="combination dials should spin continuously around their shafts",
        )

    for left_index in range(3):
        ctx.expect_gap(
            dial_parts[left_index + 1],
            dial_parts[left_index],
            axis="x",
            min_gap=0.0003,
            name=f"dials_{left_index + 1}_{left_index + 2}_stay_separate",
        )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    with ctx.pose({shackle_joint: 0.0}):
        ctx.expect_contact(
            shackle,
            body,
            elem_a=pivot_pad,
            elem_b=pivot_seat,
            contact_tol=1e-6,
            name="shackle_closed_pivot_pad_contacts_seat",
        )
        ctx.expect_contact(
            shackle,
            body,
            elem_a=free_tip,
            elem_b=free_seat,
            contact_tol=1e-6,
            name="shackle_closed_free_leg_contacts_seat",
        )
        ctx.expect_overlap(
            shackle,
            body,
            axes="x",
            min_overlap=0.004,
            name="shackle_closed_spans_body",
        )

    if (
        shackle_joint.motion_limits is not None
        and shackle_joint.motion_limits.lower is not None
        and shackle_joint.motion_limits.upper is not None
    ):
        with ctx.pose({shackle_joint: shackle_joint.motion_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(
                name="shackle_open_no_overlap"
            )
            ctx.fail_if_isolated_parts(name="shackle_open_no_floating")
            ctx.expect_contact(
                shackle,
                body,
                elem_a=pivot_pad,
                elem_b=pivot_seat,
                contact_tol=1e-6,
                name="shackle_open_stays_captive_at_pivot",
            )
            ctx.expect_gap(
                body,
                shackle,
                axis="y",
                positive_elem=free_seat,
                negative_elem=free_tip,
                min_gap=0.010,
                name="shackle_open_swings_free_leg_clear_of_body",
            )

    turned_pose = {
        dial_joint: angle
        for dial_joint, angle in zip(dial_joints, (0.3, 1.4, 2.6, 3.9))
    }
    with ctx.pose(turned_pose):
        ctx.fail_if_parts_overlap_in_current_pose(name="turned_dials_no_overlap")
        ctx.fail_if_isolated_parts(name="turned_dials_no_floating")
        for index, dial_part in enumerate(dial_parts, start=1):
            ctx.expect_gap(
                dial_part,
                body,
                axis="y",
                max_gap=0.0002,
                max_penetration=0.0,
                name=f"dial_{index}_contacts_its_shaft_mount",
            )
            ctx.expect_overlap(
                dial_part,
                body,
                axes="xz",
                min_overlap=0.006,
                name=f"dial_{index}_is_centered_on_front_face",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
