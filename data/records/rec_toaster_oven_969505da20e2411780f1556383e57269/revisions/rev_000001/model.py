from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="digital_toaster_oven")

    brushed = model.material("brushed_stainless", rgba=(0.62, 0.61, 0.57, 1.0))
    dark_metal = model.material("black_enamel", rgba=(0.015, 0.014, 0.013, 1.0))
    trim = model.material("matte_black_trim", rgba=(0.02, 0.018, 0.016, 1.0))
    glass = model.material("smoky_glass", rgba=(0.15, 0.22, 0.28, 0.42))
    display_glass = model.material("display_glass", rgba=(0.015, 0.035, 0.045, 1.0))
    display_green = model.material("display_green", rgba=(0.20, 1.0, 0.42, 1.0))
    button_mat = model.material("soft_silver_buttons", rgba=(0.78, 0.78, 0.74, 1.0))
    rubber = model.material("rubber_feet", rgba=(0.01, 0.01, 0.01, 1.0))
    chrome = model.material("chrome_wire", rgba=(0.82, 0.82, 0.78, 1.0))

    body = model.part("oven_body")

    # Countertop-size oven shell: width 48 cm, depth 34 cm, height 28 cm on feet.
    body.visual(Box((0.48, 0.34, 0.025)), origin=Origin(xyz=(0.0, 0.0, 0.2875)), material=brushed, name="top_shell")
    body.visual(Box((0.48, 0.34, 0.025)), origin=Origin(xyz=(0.0, 0.0, 0.0375)), material=brushed, name="bottom_shell")
    body.visual(Box((0.025, 0.34, 0.25)), origin=Origin(xyz=(-0.2275, 0.0, 0.1625)), material=brushed, name="side_shell")
    body.visual(Box((0.48, 0.025, 0.25)), origin=Origin(xyz=(0.0, 0.1575, 0.1625)), material=brushed, name="rear_shell")
    body.visual(Box((0.115, 0.34, 0.25)), origin=Origin(xyz=(0.1825, 0.0, 0.1625)), material=brushed, name="control_column")

    # Front aperture frame around the glass door.
    body.visual(Box((0.365, 0.020, 0.035)), origin=Origin(xyz=(-0.0575, -0.170, 0.2675)), material=trim, name="front_top_rail")
    body.visual(Box((0.365, 0.020, 0.035)), origin=Origin(xyz=(-0.0575, -0.170, 0.0575)), material=trim, name="front_sill")
    body.visual(Box((0.028, 0.020, 0.220)), origin=Origin(xyz=(-0.226, -0.170, 0.1625)), material=trim, name="front_side_rail")
    body.visual(Box((0.030, 0.020, 0.220)), origin=Origin(xyz=(0.110, -0.170, 0.1625)), material=trim, name="front_divider")

    # Dark, hollow-looking oven cavity with a wire rack and crumb tray.
    body.visual(Box((0.335, 0.006, 0.160)), origin=Origin(xyz=(-0.0575, 0.120, 0.160)), material=dark_metal, name="cavity_back")
    body.visual(Box((0.335, 0.250, 0.010)), origin=Origin(xyz=(-0.0575, -0.020, 0.082)), material=dark_metal, name="crumb_tray")
    body.visual(Box((0.005, 0.250, 0.155)), origin=Origin(xyz=(-0.2125, -0.020, 0.165)), material=dark_metal, name="inner_side")
    body.visual(Box((0.005, 0.250, 0.155)), origin=Origin(xyz=(0.0925, -0.020, 0.165)), material=dark_metal, name="inner_divider")
    for i, y in enumerate((-0.105, -0.055, -0.005, 0.045, 0.095)):
        body.visual(
            Cylinder(radius=0.0032, length=0.305),
            origin=Origin(xyz=(-0.0575, y, 0.150), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=chrome,
            name=f"rack_wire_{i}",
        )
    for x, y in ((-0.18, -0.10), (0.18, -0.10), (-0.18, 0.10), (0.18, 0.10)):
        body.visual(Cylinder(radius=0.018, length=0.025), origin=Origin(xyz=(x, y, 0.0125)), material=rubber, name=f"foot_{x}_{y}")

    # Small digital display and seven-segment-like green marks on the front control column.
    body.visual(Box((0.072, 0.004, 0.034)), origin=Origin(xyz=(0.181, -0.172, 0.228)), material=display_glass, name="display_lens")
    for i, x in enumerate((0.162, 0.180, 0.198)):
        body.visual(Box((0.010, 0.002, 0.003)), origin=Origin(xyz=(x, -0.175, 0.235)), material=display_green, name=f"display_segment_top_{i}")
        body.visual(Box((0.010, 0.002, 0.003)), origin=Origin(xyz=(x, -0.175, 0.221)), material=display_green, name=f"display_segment_bottom_{i}")
    body.visual(Box((0.004, 0.002, 0.004)), origin=Origin(xyz=(0.210, -0.175, 0.228)), material=display_green, name="display_dot")

    # Right-side dial mounted on a short continuous shaft.
    dial = model.part("side_dial")
    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.072,
            0.030,
            body_style="skirted",
            top_diameter=0.058,
            skirt=KnobSkirt(0.078, 0.006, flare=0.06, chamfer=0.001),
            grip=KnobGrip(style="fluted", count=22, depth=0.0014),
            indicator=KnobIndicator(style="line", mode="raised", depth=0.0008, angle_deg=0.0),
            center=False,
        ),
        "side_dial_knob",
    )
    dial.visual(dial_mesh, origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)), material=trim, name="dial_cap")
    model.articulation(
        "body_to_side_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.240, 0.035, 0.175)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.35, velocity=8.0),
    )

    # Four independent push buttons on the front control strip; each travels inward.
    for i, x in enumerate((0.151, 0.171, 0.191, 0.211)):
        button = model.part(f"button_{i}")
        button.visual(Box((0.015, 0.012, 0.018)), origin=Origin(xyz=(0.0, -0.006, 0.0)), material=button_mat, name="button_cap")
        button.visual(Box((0.010, 0.003, 0.012)), origin=Origin(xyz=(0.0, -0.0015, 0.0)), material=trim, name="button_shadow")
        model.articulation(
            f"body_to_button_{i}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, -0.170, 0.170)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.2, velocity=0.12, lower=0.0, upper=0.006),
        )

    # Downward-opening glass door hinged along the lower front edge.
    door = model.part("glass_door")
    door.visual(Box((0.355, 0.012, 0.030)), origin=Origin(xyz=(0.0, 0.0, 0.015)), material=trim, name="bottom_frame")
    door.visual(Box((0.355, 0.012, 0.030)), origin=Origin(xyz=(0.0, 0.0, 0.195)), material=trim, name="top_frame")
    door.visual(Box((0.028, 0.012, 0.210)), origin=Origin(xyz=(-0.1635, 0.0, 0.105)), material=trim, name="side_frame_0")
    door.visual(Box((0.028, 0.012, 0.210)), origin=Origin(xyz=(0.1635, 0.0, 0.105)), material=trim, name="side_frame_1")
    door.visual(Box((0.292, 0.004, 0.145)), origin=Origin(xyz=(0.0, -0.002, 0.110)), material=glass, name="glass_pane")
    door.visual(
        Cylinder(radius=0.006, length=0.350),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="hinge_barrel",
    )
    door.visual(
        Cylinder(radius=0.009, length=0.260),
        origin=Origin(xyz=(0.0, -0.055, 0.185), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed,
        name="door_handle",
    )
    door.visual(Box((0.018, 0.050, 0.014)), origin=Origin(xyz=(-0.095, -0.030, 0.185)), material=brushed, name="handle_post_0")
    door.visual(Box((0.018, 0.050, 0.014)), origin=Origin(xyz=(0.095, -0.030, 0.185)), material=brushed, name="handle_post_1")
    model.articulation(
        "body_to_glass_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-0.0575, -0.186, 0.072)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.8, lower=0.0, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("oven_body")
    door = object_model.get_part("glass_door")
    dial = object_model.get_part("side_dial")
    hinge = object_model.get_articulation("body_to_glass_door")
    dial_joint = object_model.get_articulation("body_to_side_dial")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(body, door, axis="y", min_gap=-0.001, max_gap=0.010, name="closed door sits just in front of body")
        ctx.expect_overlap(door, body, axes="xz", min_overlap=0.15, name="closed door covers oven aperture")

    rest_aabb = ctx.part_element_world_aabb(door, elem="top_frame")
    rest_top = None
    if rest_aabb is not None:
        rest_top = tuple((rest_aabb[0][axis] + rest_aabb[1][axis]) / 2.0 for axis in range(3))
    with ctx.pose({hinge: 1.25}):
        opened_aabb = ctx.part_element_world_aabb(door, elem="top_frame")
        opened_top = None
        if opened_aabb is not None:
            opened_top = tuple((opened_aabb[0][axis] + opened_aabb[1][axis]) / 2.0 for axis in range(3))
    ctx.check(
        "door opens downward and forward",
        rest_top is not None and opened_top is not None and opened_top[1] < rest_top[1] - 0.08 and opened_top[2] < rest_top[2] - 0.08,
        details=f"rest_top={rest_top}, opened_top={opened_top}",
    )

    ctx.expect_contact(dial, body, elem_a="dial_cap", elem_b="control_column", contact_tol=0.002, name="side dial mounted to side panel")
    with ctx.pose({dial_joint: math.tau}):
        ctx.expect_contact(dial, body, elem_a="dial_cap", elem_b="control_column", contact_tol=0.002, name="dial remains on short shaft after full turn")

    for i in range(4):
        button = object_model.get_part(f"button_{i}")
        joint = object_model.get_articulation(f"body_to_button_{i}")
        ctx.expect_contact(button, body, elem_a="button_cap", elem_b="control_column", contact_tol=0.003, name=f"button {i} rests on control panel")
        rest_button = ctx.part_world_position(button)
        with ctx.pose({joint: 0.006}):
            pressed_button = ctx.part_world_position(button)
        ctx.check(
            f"button {i} depresses inward",
            rest_button is not None and pressed_button is not None and pressed_button[1] > rest_button[1] + 0.004,
            details=f"rest={rest_button}, pressed={pressed_button}",
        )

    return ctx.report()


object_model = build_object_model()
