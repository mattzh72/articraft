from __future__ import annotations

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_microwave")

    model.material("brushed_steel", rgba=(0.68, 0.70, 0.69, 1.0))
    model.material("dark_plastic", rgba=(0.025, 0.027, 0.030, 1.0))
    model.material("panel_black", rgba=(0.09, 0.10, 0.115, 1.0))
    model.material("cavity_black", rgba=(0.006, 0.007, 0.008, 1.0))
    model.material("smoked_glass", rgba=(0.08, 0.17, 0.22, 0.46))
    model.material("button_gray", rgba=(0.24, 0.26, 0.28, 1.0))
    model.material("button_green", rgba=(0.10, 0.34, 0.18, 1.0))
    model.material("button_red", rgba=(0.45, 0.09, 0.07, 1.0))
    model.material("display_glow", rgba=(0.20, 0.75, 0.95, 1.0))

    body = model.part("body")

    # Countertop microwave cabinet: 56 cm wide, 42 cm deep, 32 cm tall.
    # The origin is at the front lower center plane; +X is outward from the
    # front, +Y is toward the control panel, and +Z is upward.
    body.visual(
        Box((0.43, 0.56, 0.025)),
        origin=Origin(xyz=(-0.215, 0.0, 0.3075)),
        material="brushed_steel",
        name="top_shell",
    )
    body.visual(
        Box((0.43, 0.56, 0.025)),
        origin=Origin(xyz=(-0.215, 0.0, 0.0125)),
        material="brushed_steel",
        name="bottom_shell",
    )
    body.visual(
        Box((0.43, 0.015, 0.32)),
        origin=Origin(xyz=(-0.215, -0.2725, 0.16)),
        material="brushed_steel",
        name="hinge_side_shell",
    )
    body.visual(
        Box((0.43, 0.015, 0.32)),
        origin=Origin(xyz=(-0.215, 0.2725, 0.16)),
        material="brushed_steel",
        name="panel_side_shell",
    )
    body.visual(
        Box((0.025, 0.56, 0.32)),
        origin=Origin(xyz=(-0.4175, 0.0, 0.16)),
        material="brushed_steel",
        name="rear_shell",
    )

    # Front fascia around the oven aperture, opposite a raised control panel.
    body.visual(
        Box((0.025, 0.420, 0.035)),
        origin=Origin(xyz=(0.006, -0.065, 0.2925)),
        material="brushed_steel",
        name="front_upper_rail",
    )
    body.visual(
        Box((0.025, 0.420, 0.035)),
        origin=Origin(xyz=(0.006, -0.065, 0.0275)),
        material="brushed_steel",
        name="front_lower_rail",
    )
    body.visual(
        Box((0.025, 0.035, 0.285)),
        origin=Origin(xyz=(0.006, -0.2575, 0.16)),
        material="brushed_steel",
        name="front_hinge_stile",
    )
    body.visual(
        Box((0.025, 0.025, 0.285)),
        origin=Origin(xyz=(0.006, 0.1375, 0.16)),
        material="brushed_steel",
        name="front_separator_stile",
    )
    body.visual(
        Box((0.018, 0.130, 0.285)),
        origin=Origin(xyz=(0.011, 0.205, 0.16)),
        material="panel_black",
        name="control_panel_face",
    )

    # Dark oven cavity and a fixed glass turntable visible through the door.
    body.visual(
        Box((0.012, 0.380, 0.220)),
        origin=Origin(xyz=(-0.400, -0.065, 0.160)),
        material="cavity_black",
        name="cavity_back",
    )
    body.visual(
        Box((0.400, 0.420, 0.010)),
        origin=Origin(xyz=(-0.200, -0.065, 0.057)),
        material="cavity_black",
        name="cavity_floor",
    )
    body.visual(
        Box((0.400, 0.420, 0.010)),
        origin=Origin(xyz=(-0.200, -0.065, 0.263)),
        material="cavity_black",
        name="cavity_ceiling",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.032),
        origin=Origin(xyz=(-0.195, -0.065, 0.041)),
        material="dark_plastic",
        name="turntable_spindle",
    )
    body.visual(
        Cylinder(radius=0.115, length=0.008),
        origin=Origin(xyz=(-0.195, -0.065, 0.063)),
        material="smoked_glass",
        name="turntable_plate",
    )

    # Static display window on the control panel.
    body.visual(
        Box((0.004, 0.096, 0.044)),
        origin=Origin(xyz=(0.022, 0.205, 0.255)),
        material="cavity_black",
        name="display_window",
    )
    body.visual(
        Box((0.002, 0.065, 0.006)),
        origin=Origin(xyz=(0.025, 0.205, 0.257)),
        material="display_glow",
        name="display_digits",
    )

    # Small non-articulated feet under the case.
    for idx, (x, y) in enumerate(((-0.080, -0.205), (-0.330, -0.205), (-0.080, 0.205), (-0.330, 0.205))):
        body.visual(
            Box((0.060, 0.050, 0.018)),
            origin=Origin(xyz=(x, y, -0.009)),
            material="dark_plastic",
            name=f"foot_{idx}",
        )

    door = model.part("door")
    # Door part frame is the vertical hinge axis at the left edge of the door.
    door.visual(
        Box((0.032, 0.035, 0.285)),
        origin=Origin(xyz=(0.0, 0.0175, 0.0)),
        material="dark_plastic",
        name="hinge_stile",
    )
    door.visual(
        Box((0.032, 0.035, 0.285)),
        origin=Origin(xyz=(0.0, 0.3875, 0.0)),
        material="dark_plastic",
        name="latch_stile",
    )
    door.visual(
        Box((0.032, 0.405, 0.035)),
        origin=Origin(xyz=(0.0, 0.2025, 0.125)),
        material="dark_plastic",
        name="top_rail",
    )
    door.visual(
        Box((0.032, 0.405, 0.035)),
        origin=Origin(xyz=(0.0, 0.2025, -0.125)),
        material="dark_plastic",
        name="bottom_rail",
    )
    door.visual(
        Box((0.010, 0.340, 0.225)),
        origin=Origin(xyz=(0.003, 0.2025, 0.0)),
        material="smoked_glass",
        name="glass_pane",
    )
    door.visual(
        Box((0.004, 0.305, 0.010)),
        origin=Origin(xyz=(0.0095, 0.2025, 0.045)),
        material="panel_black",
        name="glass_shade_upper",
    )
    door.visual(
        Box((0.004, 0.305, 0.010)),
        origin=Origin(xyz=(0.0095, 0.2025, -0.045)),
        material="panel_black",
        name="glass_shade_lower",
    )
    for idx, z in enumerate((-0.095, 0.0, 0.095)):
        door.visual(
            Cylinder(radius=0.012, length=0.060),
            origin=Origin(xyz=(-0.002, 0.000, z)),
            material="brushed_steel",
            name=f"hinge_barrel_{idx}",
        )
    door.visual(
        Box((0.050, 0.030, 0.024)),
        origin=Origin(xyz=(0.035, 0.360, 0.075)),
        material="brushed_steel",
        name="handle_upper_mount",
    )
    door.visual(
        Box((0.050, 0.030, 0.024)),
        origin=Origin(xyz=(0.035, 0.360, -0.075)),
        material="brushed_steel",
        name="handle_lower_mount",
    )
    door.visual(
        Box((0.025, 0.028, 0.185)),
        origin=Origin(xyz=(0.065, 0.360, 0.0)),
        material="brushed_steel",
        name="handle_bar",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.033, -0.272, 0.160)),
        # Door geometry extends along local +Y from the hinge; negative Z makes
        # positive motion swing the free edge outward toward +X.
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.65),
    )

    button_specs = (
        ("button_0", 0.205, "button_gray"),
        ("button_1", 0.173, "button_gray"),
        ("button_2", 0.141, "button_gray"),
        ("button_3", 0.109, "button_gray"),
        ("button_4", 0.077, "button_green"),
        ("button_5", 0.045, "button_red"),
    )
    for name, z, material in button_specs:
        button = model.part(name)
        button.visual(
            Box((0.006, 0.082, 0.024)),
            origin=Origin(xyz=(0.003, 0.0, 0.0)),
            material=material,
            name="button_face",
        )
        button.visual(
            Box((0.002, 0.052, 0.004)),
            origin=Origin(xyz=(0.006, 0.0, 0.004)),
            material="panel_black",
            name="legend_bar",
        )
        model.articulation(
            f"panel_to_{name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(0.020, 0.205, z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=0.08, lower=0.0, upper=0.005),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    door_joint = object_model.get_articulation("body_to_door")

    ctx.expect_overlap(
        door,
        body,
        axes="yz",
        elem_a="glass_pane",
        elem_b="cavity_back",
        min_overlap=0.18,
        name="closed glazed door covers oven cavity",
    )
    ctx.expect_within(
        door,
        body,
        axes="yz",
        inner_elem="glass_pane",
        outer_elem="cavity_back",
        margin=0.006,
        name="door glass aligns with cavity aperture",
    )

    closed_handle = ctx.part_element_world_aabb(door, elem="handle_bar")
    with ctx.pose({door_joint: 1.20}):
        open_handle = ctx.part_element_world_aabb(door, elem="handle_bar")
    ctx.check(
        "side hinge swings door outward",
        closed_handle is not None
        and open_handle is not None
        and open_handle[0][0] > closed_handle[0][0] + 0.20,
        details=f"closed={closed_handle}, open={open_handle}",
    )

    for idx in range(6):
        button = object_model.get_part(f"button_{idx}")
        joint = object_model.get_articulation(f"panel_to_button_{idx}")
        ctx.allow_overlap(
            button,
            body,
            elem_a="button_face",
            elem_b="control_panel_face",
            reason="A membrane button travels a few millimeters into the simplified solid control panel pocket when pressed.",
        )
        ctx.expect_contact(
            button,
            body,
            elem_a="button_face",
            elem_b="control_panel_face",
            contact_tol=1e-5,
            name=f"button_{idx} rests on control panel",
        )
        ctx.expect_within(
            button,
            body,
            axes="yz",
            inner_elem="button_face",
            outer_elem="control_panel_face",
            margin=0.001,
            name=f"button_{idx} stays inside panel footprint",
        )
        rest_position = ctx.part_world_position(button)
        with ctx.pose({joint: 0.005}):
            pressed_position = ctx.part_world_position(button)
            ctx.expect_gap(
                button,
                body,
                axis="x",
                positive_elem="button_face",
                negative_elem="control_panel_face",
                max_penetration=0.0055,
                name=f"button_{idx} has short inward plunger travel",
            )
        ctx.check(
            f"button_{idx} translates inward",
            rest_position is not None
            and pressed_position is not None
            and pressed_position[0] < rest_position[0] - 0.0045,
            details=f"rest={rest_position}, pressed={pressed_position}",
        )

    return ctx.report()


object_model = build_object_model()
