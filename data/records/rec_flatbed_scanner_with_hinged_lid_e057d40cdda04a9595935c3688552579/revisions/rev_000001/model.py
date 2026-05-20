from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LoftGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_flatbed_scanner")

    warm_white = Material("warm_white_plastic", rgba=(0.86, 0.85, 0.80, 1.0))
    edge_gray = Material("shadow_gray_plastic", rgba=(0.18, 0.19, 0.20, 1.0))
    dark_gray = Material("dark_gray_plastic", rgba=(0.05, 0.055, 0.060, 1.0))
    soft_gray = Material("soft_gray_rubber", rgba=(0.35, 0.36, 0.34, 1.0))
    glass_blue = Material("slightly_blue_glass", rgba=(0.45, 0.75, 0.92, 0.48))
    lcd_green = Material("muted_green_lcd", rgba=(0.22, 0.75, 0.45, 1.0))
    button_gray = Material("button_gray", rgba=(0.62, 0.64, 0.64, 1.0))
    blue_button = Material("scan_blue_button", rgba=(0.18, 0.38, 0.78, 1.0))
    green_button = Material("copy_green_button", rgba=(0.12, 0.56, 0.25, 1.0))
    red_button = Material("pdf_red_button", rgba=(0.74, 0.18, 0.14, 1.0))
    white_mark = Material("white_control_marking", rgba=(0.92, 0.94, 0.93, 1.0))

    def rr_loop(width: float, height: float, radius: float, z: float):
        return [
            (x, y, z)
            for x, y in rounded_rect_profile(
                width, height, radius, corner_segments=10
            )
        ]

    body_bottom = (0.600, 0.410)
    body_top = (0.580, 0.390)
    body_h = 0.062

    base = model.part("base")
    body_shell = LoftGeometry(
        [
            rr_loop(body_bottom[0], body_bottom[1], 0.038, 0.000),
            rr_loop(body_top[0], body_top[1], 0.032, body_h),
        ],
        cap=True,
        closed=True,
    )
    base.visual(
        mesh_from_geometry(body_shell, "tapered_body_shell"),
        material=warm_white,
        name="body_shell",
    )

    glass_bezel = BezelGeometry(
        opening_size=(0.392, 0.272),
        outer_size=(0.448, 0.328),
        depth=0.008,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.010,
        outer_corner_radius=0.018,
    )
    base.visual(
        mesh_from_geometry(glass_bezel, "glass_bezel"),
        origin=Origin(xyz=(-0.055, 0.0, body_h + 0.004)),
        material=edge_gray,
        name="glass_bezel",
    )
    base.visual(
        Box((0.386, 0.266, 0.003)),
        origin=Origin(xyz=(-0.055, 0.0, body_h + 0.0015)),
        material=glass_blue,
        name="glass_pane",
    )
    base.visual(
        Box((0.014, 0.266, 0.0012)),
        origin=Origin(xyz=(-0.240, 0.0, body_h + 0.0036)),
        material=Material("white_reference_strip", rgba=(0.94, 0.96, 0.95, 1.0)),
        name="scan_reference_strip",
    )

    control_panel_geom = ExtrudeGeometry.from_z0(
        rounded_rect_profile(0.128, 0.322, 0.018, corner_segments=8),
        0.009,
    )
    base.visual(
        mesh_from_geometry(control_panel_geom, "control_panel"),
        origin=Origin(xyz=(0.224, 0.0, body_h)),
        material=edge_gray,
        name="control_panel",
    )
    base.visual(
        Box((0.054, 0.030, 0.0015)),
        origin=Origin(xyz=(0.200, -0.105, body_h + 0.00975)),
        material=dark_gray,
        name="lcd_window",
    )
    base.visual(
        Box((0.040, 0.018, 0.0018)),
        origin=Origin(xyz=(0.200, -0.105, body_h + 0.0114)),
        material=lcd_green,
        name="lcd_screen",
    )
    base.visual(
        Box((0.003, 0.316, 0.0012)),
        origin=Origin(xyz=(0.158, 0.0, body_h + 0.0006)),
        material=edge_gray,
        name="front_panel_seam",
    )
    base.visual(
        Box((0.012, 0.350, 0.003)),
        origin=Origin(xyz=(0.292, 0.0, 0.030)),
        material=edge_gray,
        name="front_shadow_line",
    )
    base.visual(
        Box((0.040, 0.360, 0.008)),
        origin=Origin(xyz=(-0.272, 0.0, body_h + 0.004)),
        material=edge_gray,
        name="rear_hinge_shelf",
    )
    for idx, y in enumerate((-0.122, 0.122)):
        base.visual(
            Box((0.026, 0.048, 0.009)),
            origin=Origin(xyz=(-0.266, y, body_h + 0.0045)),
            material=edge_gray,
            name=f"hinge_block_{idx}",
        )
    for idx, (x, y) in enumerate(
        ((-0.230, -0.152), (-0.230, 0.152), (0.230, -0.152), (0.230, 0.152))
    ):
        base.visual(
            Box((0.055, 0.030, 0.006)),
            origin=Origin(xyz=(x, y, -0.003)),
            material=soft_gray,
            name=f"rubber_foot_{idx}",
        )

    lid = model.part("lid")
    lid_len = 0.430
    lid_w = 0.340
    lid_t = 0.026
    lid_shell = ExtrudeGeometry.from_z0(
        rounded_rect_profile(lid_len, lid_w, 0.026, corner_segments=10),
        lid_t,
    )
    lid.visual(
        mesh_from_geometry(lid_shell, "lid_shell"),
        origin=Origin(xyz=(lid_len / 2.0, 0.0, 0.0)),
        material=warm_white,
        name="lid_shell",
    )
    lid.visual(
        Box((0.370, 0.255, 0.004)),
        origin=Origin(xyz=(lid_len / 2.0, 0.0, -0.001)),
        material=Material("matte_white_backing", rgba=(0.78, 0.78, 0.72, 1.0)),
        name="underside_pad",
    )
    lid.visual(
        Box((0.350, 0.260, 0.002)),
        origin=Origin(xyz=(lid_len / 2.0, 0.0, lid_t + 0.001)),
        material=Material("subtle_lid_inset", rgba=(0.80, 0.79, 0.74, 1.0)),
        name="top_inset_panel",
    )
    lid.visual(
        Box((0.060, 0.120, 0.003)),
        origin=Origin(xyz=(lid_len - 0.012, 0.0, 0.006)),
        material=soft_gray,
        name="front_grip_pad",
    )
    for idx, y in enumerate((-0.118, 0.0, 0.118)):
        lid.visual(
            Cylinder(radius=0.006, length=0.064),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=edge_gray,
            name=f"hinge_barrel_{idx}",
        )

    hinge_z = body_h + 0.015
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(-0.266, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.6, lower=0.0, upper=1.32),
    )

    def add_round_button(
        part_name: str,
        joint_name: str,
        x: float,
        y: float,
        radius: float,
        material: Material,
    ):
        button = model.part(part_name)
        button.visual(
            Cylinder(radius=radius, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=material,
            name="button_cap",
        )
        button.visual(
            Box((radius * 1.15, 0.0020, 0.0008)),
            origin=Origin(xyz=(0.0, 0.0, 0.0064)),
            material=white_mark,
            name="button_mark",
        )
        model.articulation(
            joint_name,
            ArticulationType.PRISMATIC,
            parent=base,
            child=button,
            origin=Origin(xyz=(x, y, body_h + 0.009)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=1.5, velocity=0.04, lower=0.0, upper=0.003
            ),
        )
        return button

    def add_rect_button(
        part_name: str,
        joint_name: str,
        x: float,
        y: float,
        material: Material,
    ):
        button = model.part(part_name)
        cap = ExtrudeGeometry.from_z0(
            rounded_rect_profile(0.036, 0.022, 0.005, corner_segments=6),
            0.006,
        )
        button.visual(
            mesh_from_geometry(cap, f"{part_name}_cap"),
            material=material,
            name="button_cap",
        )
        button.visual(
            Box((0.018, 0.003, 0.0008)),
            origin=Origin(xyz=(0.0, 0.0, 0.0064)),
            material=white_mark,
            name="button_mark",
        )
        model.articulation(
            joint_name,
            ArticulationType.PRISMATIC,
            parent=base,
            child=button,
            origin=Origin(xyz=(x, y, body_h + 0.009)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=1.5, velocity=0.04, lower=0.0, upper=0.003
            ),
        )
        return button

    add_round_button(
        "power_button", "power_press", 0.210, 0.126, 0.0105, button_gray
    )
    add_rect_button("scan_button", "scan_press", 0.236, 0.068, blue_button)
    add_rect_button("copy_button", "copy_press", 0.236, 0.020, green_button)
    add_rect_button("pdf_button", "pdf_press", 0.236, -0.028, red_button)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("lid_hinge")
    power = object_model.get_part("power_button")
    power_press = object_model.get_articulation("power_press")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="control_panel",
            min_gap=0.001,
            max_gap=0.018,
            name="closed lid sits just above scanner deck",
        )
        ctx.expect_overlap(
            lid,
            base,
            axes="xy",
            elem_a="lid_shell",
            elem_b="glass_pane",
            min_overlap=0.25,
            name="closed lid covers the glass bed",
        )
        ctx.expect_gap(
            power,
            base,
            axis="z",
            positive_elem="button_cap",
            negative_elem="control_panel",
            min_gap=0.0,
            max_gap=0.0005,
            name="power button rests on control panel",
        )

    closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({hinge: 1.1}):
        open_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    ctx.check(
        "lid opens upward on rear hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.25,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    rest_power = ctx.part_world_position(power)
    with ctx.pose({power_press: 0.003}):
        pressed_power = ctx.part_world_position(power)
    ctx.check(
        "power button depresses into the panel",
        rest_power is not None
        and pressed_power is not None
        and pressed_power[2] < rest_power[2] - 0.002,
        details=f"rest={rest_power}, pressed={pressed_power}",
    )

    return ctx.report()


object_model = build_object_model()
