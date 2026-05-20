from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rounded_box(length: float, width: float, height: float, radius: float) -> cq.Workplane:
    """Centered rounded rectangular solid, dimensioned in meters."""
    solid = cq.Workplane("XY").box(length, width, height)
    if radius > 0:
        solid = solid.edges().fillet(radius)
    return solid


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retro_clamshell_flip_phone")

    silver = model.material("matte_silver", rgba=(0.72, 0.70, 0.66, 1.0))
    dark_glass = model.material("smoked_glass", rgba=(0.015, 0.018, 0.022, 1.0))
    button_black = model.material("charcoal_rubber", rgba=(0.055, 0.055, 0.060, 1.0))
    label_gray = model.material("soft_gray_labels", rgba=(0.38, 0.38, 0.39, 1.0))

    body_length = 0.105
    body_width = 0.052
    body_thickness = 0.012
    body_start_x = 0.006
    body_center_x = body_start_x + body_length / 2.0
    lower_center_z = -0.008
    upper_center_z = 0.006
    lower_top_z = lower_center_z + body_thickness / 2.0

    body_mesh = mesh_from_cadquery(
        _rounded_box(body_length, body_width, body_thickness, 0.0026),
        "rounded_phone_half",
        tolerance=0.00035,
        angular_tolerance=0.08,
    )
    numeric_key_mesh = mesh_from_cadquery(
        _rounded_box(0.0100, 0.0068, 0.00135, 0.00032),
        "numeric_key_cap",
        tolerance=0.00018,
        angular_tolerance=0.08,
    )
    soft_key_mesh = mesh_from_cadquery(
        _rounded_box(0.0140, 0.0064, 0.00135, 0.00032),
        "soft_key_cap",
        tolerance=0.00018,
        angular_tolerance=0.08,
    )

    lower = model.part("lower_half")
    lower.visual(
        body_mesh,
        origin=Origin(xyz=(body_center_x, 0.0, lower_center_z)),
        material=silver,
        name="lower_shell",
    )
    # Outer hinge knuckles are part of the lower half.  Their y gaps leave room
    # for the upper half's central knuckle around the same hinge line.
    for name, y in (("hinge_knuckle_0", -0.019), ("hinge_knuckle_1", 0.019)):
        lower.visual(
            Cylinder(radius=0.0050, length=0.0160),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=silver,
            name=name,
        )
        lower.visual(
            Box((0.0095, 0.0160, 0.0080)),
            origin=Origin(xyz=(0.0052, y, -0.0040)),
            material=silver,
            name=f"hinge_lug_{name[-1]}",
        )

    upper = model.part("upper_half")
    upper.visual(
        body_mesh,
        origin=Origin(xyz=(body_center_x, 0.0, upper_center_z)),
        material=silver,
        name="upper_shell",
    )
    upper.visual(
        Cylinder(radius=0.0050, length=0.0190),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="center_knuckle",
    )
    upper.visual(
        Box((0.0095, 0.0190, 0.0080)),
        origin=Origin(xyz=(0.0052, 0.0, 0.0040)),
        material=silver,
        name="center_lug",
    )
    upper.visual(
        Box((0.038, 0.028, 0.0007)),
        origin=Origin(xyz=(0.056, 0.0, 0.01225)),
        material=dark_glass,
        name="external_display",
    )
    upper.visual(
        Box((0.018, 0.0026, 0.00065)),
        origin=Origin(xyz=(0.024, 0.0, 0.01225)),
        material=button_black,
        name="rear_speaker_slot",
    )
    upper.visual(
        Box((0.054, 0.036, 0.0003)),
        origin=Origin(xyz=(0.062, 0.0, -0.00015)),
        material=dark_glass,
        name="inner_display",
    )
    upper.visual(
        Box((0.022, 0.0024, 0.0003)),
        origin=Origin(xyz=(0.023, 0.0, -0.00015)),
        material=button_black,
        name="earpiece_slot",
    )

    hinge = model.articulation(
        "spine_hinge",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=upper,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.4, velocity=3.0, lower=0.0, upper=2.25),
    )
    hinge.meta["description"] = "Central clamshell spine hinge; positive rotation flips the upper half open."

    # Individual low-profile rubber keys sit on the lower interior face and
    # travel downward a short distance like a real membrane keypad.
    key_h = 0.00135
    key_rows_x = [0.061, 0.074, 0.087, 0.100]
    key_cols_y = [-0.014, 0.0, 0.014]
    for row, x in enumerate(key_rows_x):
        for col, y in enumerate(key_cols_y):
            key = model.part(f"key_{row}_{col}")
            key.visual(
                numeric_key_mesh,
                origin=Origin(xyz=(0.0, 0.0, key_h / 2.0)),
                material=button_black,
                name="cap",
            )
            model.articulation(
                f"lower_to_key_{row}_{col}",
                ArticulationType.PRISMATIC,
                parent=lower,
                child=key,
                origin=Origin(xyz=(x, y, lower_top_z)),
                axis=(0.0, 0.0, 1.0),
                motion_limits=MotionLimits(effort=0.08, velocity=0.04, lower=-0.0010, upper=0.0),
            )

    nav_specs = (
        ("nav_key", 0.040, 0.000, "nav_cap"),
        ("soft_key_0", 0.040, -0.016, "soft_cap"),
        ("soft_key_1", 0.040, 0.016, "soft_cap"),
    )
    for name, x, y, cap_name in nav_specs:
        key = model.part(name)
        key.visual(
            soft_key_mesh,
            origin=Origin(xyz=(0.0, 0.0, key_h / 2.0)),
            material=label_gray if name == "nav_key" else button_black,
            name=cap_name,
        )
        model.articulation(
            f"lower_to_{name}",
            ArticulationType.PRISMATIC,
            parent=lower,
            child=key,
            origin=Origin(xyz=(x, y, lower_top_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.08, velocity=0.04, lower=-0.0010, upper=0.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower = object_model.get_part("lower_half")
    upper = object_model.get_part("upper_half")
    hinge = object_model.get_articulation("spine_hinge")

    ctx.check(
        "central spine hinge is revolute",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={hinge.articulation_type}",
    )
    ctx.check(
        "hinge opens past right angle",
        hinge.motion_limits is not None
        and hinge.motion_limits.lower == 0.0
        and hinge.motion_limits.upper is not None
        and hinge.motion_limits.upper > 2.0,
        details=f"limits={hinge.motion_limits}",
    )

    lower_shell_bb = ctx.part_element_world_aabb(lower, elem="lower_shell")
    upper_shell_bb = ctx.part_element_world_aabb(upper, elem="upper_shell")
    if lower_shell_bb is not None and upper_shell_bb is not None:
        lower_size = tuple(lower_shell_bb[1][i] - lower_shell_bb[0][i] for i in range(3))
        upper_size = tuple(upper_shell_bb[1][i] - upper_shell_bb[0][i] for i in range(3))
        ctx.check(
            "upper and lower halves are symmetric",
            all(abs(lower_size[i] - upper_size[i]) < 0.001 for i in range(3)),
            details=f"lower={lower_size}, upper={upper_size}",
        )
    else:
        ctx.fail("upper and lower halves are symmetric", "missing shell AABBs")

    ctx.expect_gap(
        upper,
        lower,
        axis="z",
        min_gap=0.001,
        max_gap=0.0035,
        positive_elem="upper_shell",
        negative_elem="lower_shell",
        name="closed halves have a thin clamshell seam",
    )
    ctx.expect_overlap(
        upper,
        lower,
        axes="xy",
        min_overlap=0.045,
        elem_a="upper_shell",
        elem_b="lower_shell",
        name="closed halves align in plan",
    )

    display_bb = ctx.part_element_world_aabb(upper, elem="external_display")
    if upper_shell_bb is not None and display_bb is not None:
        ctx.check(
            "external display is on upper rear face",
            display_bb[0][2] > upper_shell_bb[1][2] - 0.001
            and display_bb[0][0] > upper_shell_bb[0][0] + 0.015
            and display_bb[1][0] < upper_shell_bb[1][0] - 0.020,
            details=f"display={display_bb}, upper_shell={upper_shell_bb}",
        )
    else:
        ctx.fail("external display is on upper rear face", "missing display AABB")

    rest_upper_shell = ctx.part_element_world_aabb(upper, elem="upper_shell")
    with ctx.pose({hinge: 2.0}):
        open_upper_shell = ctx.part_element_world_aabb(upper, elem="upper_shell")
        if rest_upper_shell is not None and open_upper_shell is not None:
            ctx.check(
                "upper half flips open upward",
                open_upper_shell[1][2] > rest_upper_shell[1][2] + 0.060,
                details=f"rest={rest_upper_shell}, open={open_upper_shell}",
            )
        else:
            ctx.fail("upper half flips open upward", "missing upper shell AABB")

    key = object_model.get_part("key_1_1")
    key_joint = object_model.get_articulation("lower_to_key_1_1")
    key_rest = ctx.part_world_aabb(key)
    with ctx.pose({key_joint: -0.0010}):
        key_pressed = ctx.part_world_aabb(key)
        if key_rest is not None and key_pressed is not None:
            ctx.check(
                "keypad keys depress into lower half",
                key_pressed[0][2] < key_rest[0][2] - 0.0008,
                details=f"rest={key_rest}, pressed={key_pressed}",
            )
        else:
            ctx.fail("keypad keys depress into lower half", "missing key AABB")

    return ctx.report()


object_model = build_object_model()
