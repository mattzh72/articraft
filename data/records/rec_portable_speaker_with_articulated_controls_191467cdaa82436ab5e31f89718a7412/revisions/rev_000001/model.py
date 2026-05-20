from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)
import cadquery as cq


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    body = cq.Workplane("XY").box(*size)
    if radius > 0.0:
        try:
            body = body.edges().fillet(radius)
        except Exception:
            # Keep the model buildable if a very small feature prevents filleting.
            pass
    return body


def _strap_frame(width_between_lugs: float = 0.254) -> cq.Workplane:
    """One connected U-shaped carry strap with pivot-eye lugs."""
    lug_x = width_between_lugs / 2.0
    lug_thickness = 0.012
    lug_outer = 0.015
    lug_hole = 0.0068
    arm_bottom = 0.009
    arm_height = 0.060
    band_x = 0.013
    band_y = 0.010
    top_z = arm_bottom + arm_height

    body = cq.Workplane("XY").box(width_between_lugs + 0.018, band_y, 0.014).translate(
        (0.0, 0.0, top_z)
    )
    for sx in (-1.0, 1.0):
        arm = (
            cq.Workplane("XY")
            .box(band_x, band_y, arm_height)
            .translate((sx * lug_x, 0.0, arm_bottom + arm_height / 2.0))
        )
        lug = (
            cq.Workplane("YZ")
            .circle(lug_outer)
            .circle(lug_hole)
            .extrude(lug_thickness)
            .translate((sx * lug_x - lug_thickness / 2.0, 0.0, 0.0))
        )
        body = body.union(arm).union(lug)

    try:
        body = body.edges().fillet(0.0016)
    except Exception:
        pass
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_battery_speaker")

    shell_mat = model.material("rubberized_teal", rgba=(0.035, 0.16, 0.18, 1.0))
    grille_mat = model.material("black_perforated_grille", rgba=(0.006, 0.007, 0.008, 1.0))
    panel_mat = model.material("matte_control_panel", rgba=(0.025, 0.027, 0.030, 1.0))
    strap_mat = model.material("soft_carry_strap", rgba=(0.95, 0.49, 0.16, 1.0))
    pivot_mat = model.material("dark_pivot_hardware", rgba=(0.018, 0.018, 0.020, 1.0))
    dial_mat = model.material("satin_dial", rgba=(0.58, 0.61, 0.62, 1.0))
    rocker_mat = model.material("power_rocker_black", rgba=(0.045, 0.047, 0.050, 1.0))
    button_mat = model.material("menu_button_grey", rgba=(0.32, 0.34, 0.35, 1.0))
    red_mark_mat = model.material("power_mark_red", rgba=(0.90, 0.08, 0.04, 1.0))

    width = 0.240
    depth = 0.090
    height = 0.130
    control_thickness = 0.006
    control_top_z = height + control_thickness - 0.0004
    control_y = -0.017
    pivot_y = 0.026
    pivot_z = height + 0.018

    shell = model.part("speaker_body")
    shell.visual(
        mesh_from_cadquery(_rounded_box((width, depth, height), 0.014), "rounded_shell"),
        origin=Origin(xyz=(0.0, 0.0, height / 2.0)),
        material=shell_mat,
        name="rounded_shell",
    )
    shell.visual(
        mesh_from_geometry(
            PerforatedPanelGeometry(
                (0.184, 0.078),
                0.004,
                hole_diameter=0.0048,
                pitch=(0.010, 0.010),
                frame=0.010,
                corner_radius=0.007,
                stagger=True,
            ),
            "front_grille",
        ),
        origin=Origin(xyz=(0.0, -depth / 2.0 - 0.0015, height * 0.51), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grille_mat,
        name="front_grille",
    )
    shell.visual(
        mesh_from_cadquery(_rounded_box((0.176, 0.036, control_thickness), 0.005), "control_plate"),
        origin=Origin(xyz=(0.0, control_y, height + control_thickness / 2.0 - 0.0004)),
        material=panel_mat,
        name="control_plate",
    )
    for sx, suffix in ((-1.0, "0"), (1.0, "1")):
        shell.visual(
            Box((0.014, 0.018, 0.028)),
            origin=Origin(xyz=(sx * 0.112, pivot_y, height + 0.013)),
            material=pivot_mat,
            name=f"pivot_mount_{suffix}",
        )
        shell.visual(
            Cylinder(radius=0.0068, length=0.022),
            origin=Origin(xyz=(sx * 0.127, pivot_y, pivot_z), rpy=(0.0, pi / 2.0, 0.0)),
            material=pivot_mat,
            name=f"pivot_pin_{suffix}",
        )

    strap = model.part("carry_strap")
    strap.visual(
        mesh_from_cadquery(_strap_frame(), "strap_frame"),
        material=strap_mat,
        name="strap_frame",
    )
    model.articulation(
        "body_to_strap",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=strap,
        origin=Origin(xyz=(0.0, pivot_y, pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.25),
    )

    dial = model.part("central_dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.036,
                0.014,
                body_style="cylindrical",
                edge_radius=0.001,
                grip=KnobGrip(style="ribbed", count=24, depth=0.0007, width=0.0014),
                indicator=KnobIndicator(style="dot", mode="raised", angle_deg=35.0),
                center=False,
            ),
            "dial_cap",
        ),
        material=dial_mat,
        name="dial_cap",
    )
    model.articulation(
        "panel_to_dial",
        ArticulationType.CONTINUOUS,
        parent=shell,
        child=dial,
        origin=Origin(xyz=(0.0, control_y, control_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=8.0),
    )

    rocker = model.part("power_rocker")
    rocker.visual(
        mesh_from_cadquery(_rounded_box((0.034, 0.020, 0.008), 0.0025), "rocker_cap"),
        origin=Origin(),
        material=rocker_mat,
        name="rocker_cap",
    )
    rocker.visual(
        Box((0.010, 0.014, 0.0012)),
        origin=Origin(xyz=(-0.008, 0.0, 0.0045)),
        material=red_mark_mat,
        name="power_mark",
    )
    model.articulation(
        "panel_to_rocker",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=rocker,
        origin=Origin(xyz=(-0.058, control_y, control_top_z + 0.004)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=3.0, lower=-0.22, upper=0.22),
    )

    for index, x in enumerate((0.055, 0.077)):
        button = model.part(f"menu_button_{index}")
        button.visual(
            Cylinder(radius=0.0067, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=button_mat,
            name="button_cap",
        )
        model.articulation(
            f"panel_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=shell,
            child=button,
            origin=Origin(xyz=(x, control_y, control_top_z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=0.25, velocity=0.08, lower=0.0, upper=0.004),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("speaker_body")
    strap = object_model.get_part("carry_strap")
    dial = object_model.get_part("central_dial")
    rocker = object_model.get_part("power_rocker")
    button_0 = object_model.get_part("menu_button_0")
    button_1 = object_model.get_part("menu_button_1")

    strap_joint = object_model.get_articulation("body_to_strap")
    dial_joint = object_model.get_articulation("panel_to_dial")
    rocker_joint = object_model.get_articulation("panel_to_rocker")
    button_joint_0 = object_model.get_articulation("panel_to_button_0")
    button_joint_1 = object_model.get_articulation("panel_to_button_1")

    for pin_name in ("pivot_pin_0", "pivot_pin_1"):
        ctx.allow_overlap(
            strap,
            body,
            elem_a="strap_frame",
            elem_b=pin_name,
            reason="The side pivot pin is intentionally captured through the carry strap eye.",
        )
        ctx.expect_contact(
            strap,
            body,
            elem_a="strap_frame",
            elem_b=pin_name,
            name=f"{pin_name} is seated in the strap eye",
        )
        ctx.expect_overlap(
            strap,
            body,
            axes="x",
            elem_a="strap_frame",
            elem_b=pin_name,
            min_overlap=0.006,
            name=f"{pin_name} passes through the strap lug thickness",
        )

    ctx.check(
        "dial is a continuous rotary control",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(dial_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={dial_joint.articulation_type}, axis={dial_joint.axis}",
    )
    ctx.check(
        "rocker and buttons have separate joints",
        rocker_joint.child == "power_rocker"
        and button_joint_0.child == "menu_button_0"
        and button_joint_1.child == "menu_button_1"
        and rocker_joint.articulation_type == ArticulationType.REVOLUTE
        and button_joint_0.articulation_type == ArticulationType.PRISMATIC
        and button_joint_1.articulation_type == ArticulationType.PRISMATIC,
        details="power rocker must remain distinct from both menu buttons",
    )
    for moving_part, label in ((dial, "dial"), (rocker, "rocker"), (button_0, "button 0"), (button_1, "button 1")):
        ctx.expect_within(
            moving_part,
            body,
            axes="xy",
            outer_elem="control_plate",
            margin=0.004,
            name=f"{label} stays on the separate top control panel",
        )

    ctx.expect_gap(
        dial,
        body,
        axis="z",
        positive_elem="dial_cap",
        negative_elem="control_plate",
        max_gap=0.001,
        max_penetration=0.0002,
        name="dial is seated on the raised control panel",
    )
    ctx.expect_gap(
        rocker,
        body,
        axis="z",
        positive_elem="rocker_cap",
        negative_elem="control_plate",
        max_gap=0.001,
        max_penetration=0.0002,
        name="rocker is seated on the raised control panel",
    )
    for button, label in ((button_0, "button 0"), (button_1, "button 1")):
        ctx.expect_gap(
            button,
            body,
            axis="z",
            positive_elem="button_cap",
            negative_elem="control_plate",
            max_gap=0.001,
            max_penetration=0.0002,
            name=f"{label} is seated on the raised control panel",
        )

    rest_strap_aabb = ctx.part_world_aabb(strap)
    with ctx.pose({strap_joint: 1.1}):
        folded_strap_aabb = ctx.part_world_aabb(strap)
    ctx.check(
        "strap folds down around the side pivots",
        rest_strap_aabb is not None
        and folded_strap_aabb is not None
        and folded_strap_aabb[1][2] < rest_strap_aabb[1][2] - 0.020,
        details=f"upright={rest_strap_aabb}, folded={folded_strap_aabb}",
    )

    for joint, button, label in (
        (button_joint_0, button_0, "button 0"),
        (button_joint_1, button_1, "button 1"),
    ):
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({joint: 0.004}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"{label} depresses independently",
            rest_pos is not None and pressed_pos is not None and pressed_pos[2] < rest_pos[2] - 0.003,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
