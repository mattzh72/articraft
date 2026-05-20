from __future__ import annotations

import math

import cadquery as cq
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
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """CadQuery rounded rectangular solid, authored in meters."""
    sx, sy, sz = size
    body = cq.Workplane("XY").box(sx, sy, sz)
    if radius > 0:
        body = body.edges().fillet(radius)
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="square_clamshell_waffle_iron")

    shell_mat = model.material("brushed_red_shell", rgba=(0.50, 0.035, 0.025, 1.0))
    dark_mat = model.material("matte_black_insulator", rgba=(0.015, 0.014, 0.012, 1.0))
    metal_mat = model.material("dark_cast_aluminum", rgba=(0.45, 0.43, 0.39, 1.0))
    seam_mat = model.material("black_rubber_seam", rgba=(0.02, 0.018, 0.016, 1.0))
    panel_mat = model.material("charcoal_control_panel", rgba=(0.055, 0.060, 0.065, 1.0))
    tick_mat = model.material("cream_temperature_marks", rgba=(0.92, 0.82, 0.62, 1.0))
    green_mat = model.material("green_push_button", rgba=(0.05, 0.55, 0.14, 1.0))
    amber_mat = model.material("amber_push_button", rgba=(0.95, 0.47, 0.06, 1.0))
    pointer_mat = model.material("white_pointer", rgba=(0.95, 0.94, 0.86, 1.0))

    lower_body = model.part("lower_body")
    lower_body.visual(
        mesh_from_cadquery(_rounded_box((0.34, 0.34, 0.055), 0.018), "lower_shell"),
        origin=Origin(xyz=(0.18, 0.0, 0.0425)),
        material=shell_mat,
        name="lower_shell",
    )
    lower_body.visual(
        Box((0.275, 0.275, 0.008)),
        origin=Origin(xyz=(0.18, 0.0, 0.073)),
        material=metal_mat,
        name="lower_plate",
    )

    rib_count = 5
    rib_pitch = 0.045
    lower_x_names = ("lower_waffle_x_0", "lower_waffle_x_1", "lower_waffle_x_2", "lower_waffle_x_3", "lower_waffle_x_4")
    lower_y_names = ("lower_waffle_y_0", "lower_waffle_y_1", "lower_waffle_y_2", "lower_waffle_y_3", "lower_waffle_y_4")
    upper_x_names = ("upper_waffle_x_0", "upper_waffle_x_1", "upper_waffle_x_2", "upper_waffle_x_3", "upper_waffle_x_4")
    upper_y_names = ("upper_waffle_y_0", "upper_waffle_y_1", "upper_waffle_y_2", "upper_waffle_y_3", "upper_waffle_y_4")
    for i in range(rib_count):
        offset = (i - (rib_count - 1) / 2.0) * rib_pitch
        lower_body.visual(
            Box((0.260, 0.006, 0.004)),
            origin=Origin(xyz=(0.18, offset, 0.078)),
            material=metal_mat,
            name=lower_x_names[i],
        )
        lower_body.visual(
            Box((0.006, 0.260, 0.004)),
            origin=Origin(xyz=(0.18 + offset, 0.0, 0.078)),
            material=metal_mat,
            name=lower_y_names[i],
        )

    # A dark gasket line around the square cooking plate makes the clamshell seam legible.
    lower_body.visual(
        Box((0.285, 0.010, 0.006)),
        origin=Origin(xyz=(0.18, 0.142, 0.074)),
        material=seam_mat,
        name="rear_gasket",
    )
    lower_body.visual(
        Box((0.285, 0.010, 0.006)),
        origin=Origin(xyz=(0.18, -0.142, 0.074)),
        material=seam_mat,
        name="front_gasket",
    )
    lower_body.visual(
        Box((0.010, 0.285, 0.006)),
        origin=Origin(xyz=(0.040, 0.0, 0.074)),
        material=seam_mat,
        name="rear_side_gasket",
    )
    lower_body.visual(
        Box((0.010, 0.285, 0.006)),
        origin=Origin(xyz=(0.320, 0.0, 0.074)),
        material=seam_mat,
        name="front_side_gasket",
    )

    # Rubber feet keep the appliance visibly at countertop scale and supported.
    for idx, (x, y) in enumerate(((0.055, -0.120), (0.305, -0.120), (0.055, 0.120), (0.305, 0.120))):
        lower_body.visual(
            Cylinder(radius=0.018, length=0.016),
            origin=Origin(xyz=(x, y, 0.008)),
            material=dark_mat,
            name=f"foot_{idx}",
        )

    # Side-mounted control panel on the user's right side (+Y).
    lower_body.visual(
        mesh_from_cadquery(_rounded_box((0.205, 0.022, 0.060), 0.006), "side_panel"),
        origin=Origin(xyz=(0.225, 0.171, 0.045)),
        material=panel_mat,
        name="side_panel",
    )

    dial_center = (0.165, 0.182, 0.050)
    for idx, (dx, dz, sx, sz) in enumerate(
        (
            (-0.030, 0.000, 0.006, 0.016),
            (-0.020, 0.014, 0.005, 0.010),
            (0.000, 0.020, 0.005, 0.010),
            (0.020, 0.014, 0.005, 0.010),
            (0.030, 0.000, 0.006, 0.016),
        )
    ):
        lower_body.visual(
            Box((sx, 0.002, sz)),
            origin=Origin(xyz=(dial_center[0] + dx, dial_center[1], dial_center[2] + dz)),
            material=tick_mat,
            name=f"temp_tick_{idx}",
        )

    # Hinge support knuckles at the back of the appliance, with simple leaves tied into the lower shell.
    for idx, y in enumerate((-0.128, 0.128)):
        lower_body.visual(
            Box((0.014, 0.052, 0.016)),
            origin=Origin(xyz=(0.005, y, 0.062)),
            material=dark_mat,
            name=f"hinge_bridge_{idx}",
        )
        lower_body.visual(
            Box((0.048, 0.052, 0.030)),
            origin=Origin(xyz=(-0.016, y, 0.083)),
            material=dark_mat,
            name=f"hinge_leaf_{idx}",
        )
        lower_body.visual(
            Cylinder(radius=0.012, length=0.052),
            origin=Origin(xyz=(-0.020, y, 0.100), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_mat,
            name=f"hinge_knuckle_{idx}",
        )

    upper_plate = model.part("upper_plate")
    upper_plate.visual(
        mesh_from_cadquery(_rounded_box((0.34, 0.34, 0.050), 0.018), "upper_shell"),
        origin=Origin(xyz=(0.18, 0.0, 0.010)),
        material=shell_mat,
        name="upper_shell",
    )
    upper_plate.visual(
        Box((0.270, 0.270, 0.006)),
        origin=Origin(xyz=(0.18, 0.0, -0.008)),
        material=metal_mat,
        name="upper_plate_face",
    )
    for i in range(rib_count):
        offset = (i - (rib_count - 1) / 2.0) * rib_pitch
        upper_plate.visual(
            Box((0.255, 0.006, 0.006)),
            origin=Origin(xyz=(0.18, offset, -0.014)),
            material=metal_mat,
            name=upper_x_names[i],
        )
        upper_plate.visual(
            Box((0.006, 0.255, 0.006)),
            origin=Origin(xyz=(0.18 + offset, 0.0, -0.014)),
            material=metal_mat,
            name=upper_y_names[i],
        )

    upper_plate.visual(
        Cylinder(radius=0.012, length=0.150),
        origin=Origin(xyz=(-0.020, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_mat,
        name="hinge_barrel",
    )
    upper_plate.visual(
        Cylinder(radius=0.004, length=0.310),
        origin=Origin(xyz=(-0.020, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="hinge_pin",
    )
    upper_plate.visual(
        Box((0.050, 0.150, 0.014)),
        origin=Origin(xyz=(-0.004, 0.0, -0.008)),
        material=dark_mat,
        name="hinge_leaf",
    )

    # Short insulated front handle with two visible standoffs on the lifting plate.
    for idx, y in enumerate((-0.080, 0.080)):
        upper_plate.visual(
            Box((0.045, 0.026, 0.032)),
            origin=Origin(xyz=(0.348, y, 0.011)),
            material=dark_mat,
            name=f"handle_standoff_{idx}",
        )
    upper_plate.visual(
        Cylinder(radius=0.017, length=0.245),
        origin=Origin(xyz=(0.385, 0.0, 0.012), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_mat,
        name="front_handle",
    )

    model.articulation(
        "body_to_upper_plate",
        ArticulationType.REVOLUTE,
        parent=lower_body,
        child=upper_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.6, lower=0.0, upper=1.25),
    )

    temp_dial = model.part("temp_dial")
    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.052,
            0.024,
            body_style="skirted",
            top_diameter=0.043,
            edge_radius=0.0015,
            grip=KnobGrip(style="fluted", count=18, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
            center=False,
        ),
        "temperature_dial",
    )
    temp_dial.visual(
        dial_mesh,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_mat,
        name="dial_cap",
    )
    temp_dial.visual(
        Box((0.004, 0.002, 0.018)),
        origin=Origin(xyz=(0.0, 0.0248, 0.008)),
        material=pointer_mat,
        name="dial_pointer",
    )
    model.articulation(
        "body_to_temp_dial",
        ArticulationType.CONTINUOUS,
        parent=lower_body,
        child=temp_dial,
        origin=Origin(xyz=dial_center),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=6.0),
    )

    button_positions = ((0.235, 0.182, 0.050), (0.290, 0.182, 0.050))
    button_mats = (green_mat, amber_mat)
    for idx, (pos, mat) in enumerate(zip(button_positions, button_mats)):
        button = model.part(f"button_{idx}")
        button.visual(
            Cylinder(radius=0.016, length=0.014),
            origin=Origin(xyz=(0.0, 0.007, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=mat,
            name="button_cap",
        )
        button.visual(
            Cylinder(radius=0.009, length=0.006),
            origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_mat,
            name="button_stem",
        )
        model.articulation(
            f"body_to_button_{idx}",
            ArticulationType.PRISMATIC,
            parent=lower_body,
            child=button,
            origin=Origin(xyz=pos),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=0.06, lower=0.0, upper=0.008),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower = object_model.get_part("lower_body")
    upper = object_model.get_part("upper_plate")
    dial = object_model.get_part("temp_dial")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")

    hinge = object_model.get_articulation("body_to_upper_plate")
    dial_joint = object_model.get_articulation("body_to_temp_dial")
    button_joint_0 = object_model.get_articulation("body_to_button_0")
    button_joint_1 = object_model.get_articulation("body_to_button_1")

    for knuckle_name in ("hinge_knuckle_0", "hinge_knuckle_1"):
        ctx.allow_overlap(
            lower,
            upper,
            elem_a=knuckle_name,
            elem_b="hinge_pin",
            reason="The metal hinge pin is intentionally captured inside the simplified solid hinge knuckle.",
        )
        ctx.expect_within(
            upper,
            lower,
            axes="xz",
            inner_elem="hinge_pin",
            outer_elem=knuckle_name,
            margin=0.001,
            name=f"{knuckle_name} captures hinge pin radially",
        )
        ctx.expect_overlap(
            upper,
            lower,
            axes="y",
            elem_a="hinge_pin",
            elem_b=knuckle_name,
            min_overlap=0.040,
            name=f"{knuckle_name} has retained hinge pin length",
        )

    ctx.expect_overlap(
        upper,
        lower,
        axes="xy",
        elem_a="upper_plate_face",
        elem_b="lower_plate",
        min_overlap=0.24,
        name="square cooking plates align when closed",
    )
    ctx.expect_gap(
        upper,
        lower,
        axis="z",
        positive_elem="upper_waffle_x_0",
        negative_elem="lower_waffle_x_0",
        min_gap=0.001,
        max_gap=0.006,
        name="opposing waffle grids have a narrow cooking gap",
    )
    ctx.expect_gap(
        dial,
        lower,
        axis="y",
        positive_elem="dial_cap",
        negative_elem="side_panel",
        max_gap=0.003,
        max_penetration=0.0005,
        name="temperature dial is seated on side panel",
    )
    for idx, button in enumerate((button_0, button_1)):
        ctx.expect_gap(
            button,
            lower,
            axis="y",
            positive_elem="button_cap",
            negative_elem="side_panel",
            max_gap=0.003,
            max_penetration=0.0005,
            name=f"button_{idx} cap starts proud of side panel",
        )

    closed_handle_aabb = ctx.part_element_world_aabb(upper, elem="front_handle")
    with ctx.pose({hinge: 1.15}):
        open_handle_aabb = ctx.part_element_world_aabb(upper, elem="front_handle")
    ctx.check(
        "top plate hinge lifts front handle upward",
        closed_handle_aabb is not None
        and open_handle_aabb is not None
        and open_handle_aabb[0][2] > closed_handle_aabb[0][2] + 0.18,
        details=f"closed={closed_handle_aabb}, open={open_handle_aabb}",
    )

    pointer_rest = ctx.part_element_world_aabb(dial, elem="dial_pointer")
    with ctx.pose({dial_joint: math.pi / 2.0}):
        pointer_rotated = ctx.part_element_world_aabb(dial, elem="dial_pointer")
    ctx.check(
        "temperature dial rotates its pointer about side normal",
        pointer_rest is not None
        and pointer_rotated is not None
        and (pointer_rest[1][2] - pointer_rest[0][2]) > 0.012
        and (pointer_rotated[1][0] - pointer_rotated[0][0]) > 0.012,
        details=f"rest={pointer_rest}, rotated={pointer_rotated}",
    )

    for idx, (button, joint) in enumerate(((button_0, button_joint_0), (button_1, button_joint_1))):
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({joint: 0.008}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button_{idx} moves inward along press axis",
            rest_pos is not None and pressed_pos is not None and pressed_pos[1] < rest_pos[1] - 0.006,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
