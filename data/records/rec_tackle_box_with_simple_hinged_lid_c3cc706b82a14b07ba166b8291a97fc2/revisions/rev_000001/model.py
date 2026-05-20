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


def _axis_rpy(axis: str) -> tuple[float, float, float]:
    if axis == "x":
        return (0.0, pi / 2.0, 0.0)
    if axis == "y":
        return (pi / 2.0, 0.0, 0.0)
    return (0.0, 0.0, 0.0)


def _add_bolt_head(
    part,
    *,
    name: str,
    center: tuple[float, float, float],
    axis: str,
    material,
    radius: float = 0.0036,
    length: float = 0.0025,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=_axis_rpy(axis)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_safety_tackle_box")

    body_color = model.material("body_powdercoat", rgba=(0.21, 0.23, 0.25, 1.0))
    hardware_color = model.material("zinc_hardware", rgba=(0.70, 0.72, 0.74, 1.0))
    safety_color = model.material("safety_reinforcement", rgba=(0.78, 0.68, 0.16, 1.0))

    body_length = 0.56
    body_width = 0.28
    body_height = 0.18
    lid_height = 0.085
    shell_t = 0.004
    plate_t = 0.006
    lid_length = 0.568
    lid_width = 0.296
    hinge_radius = 0.008
    hinge_x = -0.282
    hinge_z = 0.184
    latch_y = 0.098
    stop_y = 0.124

    body = model.part("body")

    # Open-top body shell.
    body.visual(
        Box((body_length, body_width, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, shell_t / 2.0)),
        material=body_color,
        name="shell_bottom",
    )
    body.visual(
        Box((shell_t, body_width, body_height)),
        origin=Origin(xyz=(body_length / 2.0 - shell_t / 2.0, 0.0, body_height / 2.0)),
        material=body_color,
        name="shell_front",
    )
    body.visual(
        Box((shell_t, body_width, body_height)),
        origin=Origin(xyz=(-body_length / 2.0 + shell_t / 2.0, 0.0, body_height / 2.0)),
        material=body_color,
        name="shell_rear",
    )
    body.visual(
        Box((body_length - 2.0 * shell_t, shell_t, body_height)),
        origin=Origin(
            xyz=(0.0, body_width / 2.0 - shell_t / 2.0, body_height / 2.0)
        ),
        material=body_color,
        name="shell_left",
    )
    body.visual(
        Box((body_length - 2.0 * shell_t, shell_t, body_height)),
        origin=Origin(
            xyz=(0.0, -body_width / 2.0 + shell_t / 2.0, body_height / 2.0)
        ),
        material=body_color,
        name="shell_right",
    )

    # Internal rim reinforcement to give the lid a real load path.
    body.visual(
        Box((0.018, body_width - 0.020, 0.018)),
        origin=Origin(xyz=(body_length / 2.0 - shell_t - 0.009, 0.0, body_height - 0.009)),
        material=hardware_color,
        name="front_rim_doubler",
    )
    body.visual(
        Box((0.018, body_width - 0.020, 0.018)),
        origin=Origin(xyz=(-body_length / 2.0 + shell_t + 0.009, 0.0, body_height - 0.009)),
        material=hardware_color,
        name="rear_rim_doubler",
    )
    body.visual(
        Box((body_length - 0.036, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, body_width / 2.0 - shell_t - 0.009, body_height - 0.009)),
        material=hardware_color,
        name="left_rim_doubler",
    )
    body.visual(
        Box((body_length - 0.036, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -body_width / 2.0 + shell_t + 0.009, body_height - 0.009)),
        material=hardware_color,
        name="right_rim_doubler",
    )

    # Rear hinge reinforcement plates and body-side hinge barrels.
    for side_name, side_y in (("left", latch_y), ("right", -latch_y)):
        body.visual(
            Box((plate_t, 0.072, 0.038)),
            origin=Origin(xyz=(-body_length / 2.0 - plate_t / 2.0 + 0.002, side_y, 0.175)),
            material=hardware_color,
            name=f"hinge_plate_{side_name}",
        )
        body.visual(
            Cylinder(radius=hinge_radius, length=0.066),
            origin=Origin(
                xyz=(hinge_x, side_y, hinge_z),
                rpy=_axis_rpy("y"),
            ),
            material=hardware_color,
            name=f"hinge_barrel_{side_name}",
        )
        _add_bolt_head(
            body,
            name=f"hinge_bolt_upper_{side_name}",
            center=(-body_length / 2.0 - 0.005, side_y, 0.186),
            axis="x",
            material=hardware_color,
        )
        _add_bolt_head(
            body,
            name=f"hinge_bolt_lower_{side_name}",
            center=(-body_length / 2.0 - 0.005, side_y, 0.164),
            axis="x",
            material=hardware_color,
        )

    # Front latch reinforcement plates.
    for side_name, side_y in (("left", latch_y), ("right", -latch_y)):
        body.visual(
            Box((0.006, 0.044, 0.110)),
            origin=Origin(xyz=(body_length / 2.0 + 0.001, side_y, 0.172)),
            material=hardware_color,
            name=f"front_latch_plate_{side_name}",
        )
        _add_bolt_head(
            body,
            name=f"front_latch_bolt_upper_{side_name}",
            center=(body_length / 2.0 + 0.005, side_y, 0.196),
            axis="x",
            material=hardware_color,
        )
        _add_bolt_head(
            body,
            name=f"front_latch_bolt_lower_{side_name}",
            center=(body_length / 2.0 + 0.005, side_y, 0.148),
            axis="x",
            material=hardware_color,
        )

    # Central lockout receiver: twin tabs backed by a receiver plate.
    body.visual(
        Box((0.008, 0.032, 0.056)),
        origin=Origin(xyz=(body_length / 2.0 + 0.002, 0.0, 0.204)),
        material=safety_color,
        name="lockout_receiver_plate",
    )
    body.visual(
        Box((0.006, 0.008, 0.034)),
        origin=Origin(xyz=(body_length / 2.0 + 0.003, 0.007, 0.190)),
        material=safety_color,
        name="lower_lockout_tab_left",
    )
    body.visual(
        Box((0.006, 0.008, 0.034)),
        origin=Origin(xyz=(body_length / 2.0 + 0.003, -0.007, 0.190)),
        material=safety_color,
        name="lower_lockout_tab_right",
    )

    # External open-angle stop lugs mounted off the rear hinge plates.
    body.visual(
        Box((0.026, 0.018, 0.024)),
        origin=Origin(xyz=(-0.293, 0.112, 0.208)),
        material=safety_color,
        name="open_stop_block_left",
    )
    body.visual(
        Box((0.026, 0.018, 0.024)),
        origin=Origin(xyz=(-0.293, -0.112, 0.208)),
        material=safety_color,
        name="open_stop_block_right",
    )
    body.visual(
        Box((0.010, 0.018, 0.004)),
        origin=Origin(xyz=(-0.284, 0.112, 0.195)),
        material=safety_color,
        name="open_stop_brace_left",
    )
    body.visual(
        Box((0.010, 0.018, 0.004)),
        origin=Origin(xyz=(-0.284, -0.112, 0.195)),
        material=safety_color,
        name="open_stop_brace_right",
    )

    lid = model.part("lid")

    # Lid shell uses the hinge line as the part frame.
    lid.visual(
        Box((lid_length, lid_width, shell_t)),
        origin=Origin(xyz=(lid_length / 2.0, 0.0, lid_height - shell_t / 2.0)),
        material=body_color,
        name="lid_top",
    )
    lid.visual(
        Box((shell_t, lid_width, lid_height + 0.004)),
        origin=Origin(xyz=(lid_length - shell_t / 2.0, 0.0, (lid_height - 0.004) / 2.0)),
        material=body_color,
        name="lid_front_skirt",
    )
    lid.visual(
        Box((lid_length, shell_t, lid_height + 0.004)),
        origin=Origin(
            xyz=(lid_length / 2.0, lid_width / 2.0 - shell_t / 2.0, (lid_height - 0.004) / 2.0)
        ),
        material=body_color,
        name="lid_left_skirt",
    )
    lid.visual(
        Box((lid_length, shell_t, lid_height + 0.004)),
        origin=Origin(
            xyz=(lid_length / 2.0, -lid_width / 2.0 + shell_t / 2.0, (lid_height - 0.004) / 2.0)
        ),
        material=body_color,
        name="lid_right_skirt",
    )
    lid.visual(
        Box((0.006, 0.110, 0.086)),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=hardware_color,
        name="rear_hinge_strap",
    )
    lid.visual(
        Cylinder(radius=hinge_radius, length=0.098),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=_axis_rpy("y")),
        material=hardware_color,
        name="hinge_barrel_center",
    )

    # Seating flanges keep the lid physically supported in the closed pose.
    lid.visual(
        Box((0.025, body_width - 0.030, 0.010)),
        origin=Origin(xyz=(0.5515, 0.0, 0.001)),
        material=hardware_color,
        name="front_seat_flange",
    )
    lid.visual(
        Box((0.500, 0.0215, 0.010)),
        origin=Origin(xyz=(0.276, 0.1335, 0.001)),
        material=hardware_color,
        name="left_seat_flange",
    )
    lid.visual(
        Box((0.500, 0.0215, 0.010)),
        origin=Origin(xyz=(0.276, -0.1335, 0.001)),
        material=hardware_color,
        name="right_seat_flange",
    )
    lid.visual(
        Box((0.024, 0.020, 0.010)),
        origin=Origin(xyz=(0.040, stop_y, 0.001)),
        material=hardware_color,
        name="rear_seat_pad_left",
    )
    lid.visual(
        Box((0.024, 0.020, 0.010)),
        origin=Origin(xyz=(0.040, -stop_y, 0.001)),
        material=hardware_color,
        name="rear_seat_pad_right",
    )

    # Top ribs and front latch guards.
    lid.visual(
        Box((0.340, 0.028, 0.014)),
        origin=Origin(xyz=(0.310, 0.070, 0.090)),
        material=hardware_color,
        name="lid_rib_left",
    )
    lid.visual(
        Box((0.340, 0.028, 0.014)),
        origin=Origin(xyz=(0.310, -0.070, 0.090)),
        material=hardware_color,
        name="lid_rib_right",
    )
    lid.visual(
        Box((0.014, 0.048, 0.056)),
        origin=Origin(xyz=(0.575, latch_y, 0.038)),
        material=hardware_color,
        name="latch_guard_left",
    )
    lid.visual(
        Box((0.014, 0.048, 0.056)),
        origin=Origin(xyz=(0.575, -latch_y, 0.038)),
        material=hardware_color,
        name="latch_guard_right",
    )
    lid.visual(
        Box((0.016, 0.040, 0.056)),
        origin=Origin(xyz=(0.576, 0.0, 0.040)),
        material=safety_color,
        name="lockout_guard",
    )
    lid.visual(
        Box((0.006, 0.010, 0.040)),
        origin=Origin(xyz=(0.573, 0.0, 0.020)),
        material=safety_color,
        name="upper_lockout_tab",
    )

    # Lid-side over-travel stop ears tied back into the hinge strap.
    lid.visual(
        Box((0.022, 0.066, 0.012)),
        origin=Origin(xyz=(0.010, 0.087, 0.020)),
        material=safety_color,
        name="stop_tab_left",
    )
    lid.visual(
        Box((0.022, 0.066, 0.012)),
        origin=Origin(xyz=(0.010, -0.087, 0.020)),
        material=safety_color,
        name="stop_tab_right",
    )

    # Visible fasteners on the lid hardware.
    for side_name, side_y in (("left", latch_y), ("right", -latch_y)):
        _add_bolt_head(
            lid,
            name=f"lid_latch_bolt_upper_{side_name}",
            center=(0.582, side_y, 0.051),
            axis="x",
            material=hardware_color,
        )
        _add_bolt_head(
            lid,
            name=f"lid_latch_bolt_lower_{side_name}",
            center=(0.582, side_y, 0.025),
            axis="x",
            material=hardware_color,
        )
    _add_bolt_head(
        lid,
        name="lid_hinge_bolt_left",
        center=(-0.004, 0.028, 0.015),
        axis="x",
        material=hardware_color,
    )
    _add_bolt_head(
        lid,
        name="lid_hinge_bolt_right",
        center=(-0.004, -0.028, 0.015),
        axis="x",
        material=hardware_color,
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=1.72,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("body_to_lid")

    front_rim = body.get_visual("front_rim_doubler")
    left_rim = body.get_visual("left_rim_doubler")
    left_plate = body.get_visual("front_latch_plate_left")
    right_plate = body.get_visual("front_latch_plate_right")
    receiver = body.get_visual("lockout_receiver_plate")
    stop_block_left = body.get_visual("open_stop_block_left")
    stop_block_right = body.get_visual("open_stop_block_right")

    front_seat = lid.get_visual("front_seat_flange")
    left_seat = lid.get_visual("left_seat_flange")
    left_guard = lid.get_visual("latch_guard_left")
    right_guard = lid.get_visual("latch_guard_right")
    upper_lockout = lid.get_visual("upper_lockout_tab")
    stop_tab_left = lid.get_visual("stop_tab_left")
    stop_tab_right = lid.get_visual("stop_tab_right")

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

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_contact(
            lid,
            body,
            elem_a=front_seat,
            elem_b=front_rim,
            contact_tol=0.0015,
            name="lid front seat bears on reinforced front rim",
        )
        ctx.expect_contact(
            lid,
            body,
            elem_a=left_seat,
            elem_b=left_rim,
            contact_tol=0.0015,
            name="lid side seat bears on reinforced side rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            elem_a=left_guard,
            elem_b=left_plate,
            axes="yz",
            min_overlap=0.030,
            name="left latch guard lines up with keeper plate",
        )
        ctx.expect_gap(
            lid,
            body,
            positive_elem=left_guard,
            negative_elem=left_plate,
            axis="x",
            min_gap=0.001,
            max_gap=0.008,
            name="left latch guard stands off from keeper plate",
        )
        ctx.expect_overlap(
            lid,
            body,
            elem_a=right_guard,
            elem_b=right_plate,
            axes="yz",
            min_overlap=0.030,
            name="right latch guard lines up with keeper plate",
        )
        ctx.expect_gap(
            lid,
            body,
            positive_elem=right_guard,
            negative_elem=right_plate,
            axis="x",
            min_gap=0.001,
            max_gap=0.008,
            name="right latch guard stands off from keeper plate",
        )
        ctx.expect_overlap(
            lid,
            body,
            elem_a=upper_lockout,
            elem_b=receiver,
            axes="yz",
            min_overlap=0.010,
            name="lockout tab aligns with receiver plate",
        )
        ctx.expect_gap(
            lid,
            body,
            positive_elem=upper_lockout,
            negative_elem=receiver,
            axis="x",
            min_gap=0.001,
            max_gap=0.008,
            name="lockout tab remains hardware-constrained without penetration",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.22,
            name="closed lid covers the body footprint",
        )

    open_limit = 0.0
    if lid_hinge.motion_limits is not None and lid_hinge.motion_limits.upper is not None:
        open_limit = lid_hinge.motion_limits.upper

    with ctx.pose({lid_hinge: open_limit}):
        ctx.expect_contact(
            lid,
            body,
            elem_a=stop_tab_left,
            elem_b=stop_block_left,
            contact_tol=0.0025,
            name="left over-travel stop captures the lid at full open",
        )
        ctx.expect_contact(
            lid,
            body,
            elem_a=stop_tab_right,
            elem_b=stop_block_right,
            contact_tol=0.0025,
            name="right over-travel stop captures the lid at full open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
