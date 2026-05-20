from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _add_grill_ribs(
    part,
    *,
    prefix: str,
    x_positions: tuple[float, ...],
    z_center: float,
    rib_size: tuple[float, float, float],
    material,
) -> None:
    for index, x_pos in enumerate(x_positions):
        part.visual(
            Box(rib_size),
            origin=Origin(xyz=(x_pos, 0.0, z_center)),
            material=material,
            name=f"{prefix}_{index}",
        )


def _loft_station(
    *,
    x_pos: float,
    width_y: float,
    height_z: float,
    radius: float,
    z_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (y_pos, z_center + z_pos, x_pos)
        for y_pos, z_pos in rounded_rect_profile(width_y, height_z, radius, corner_segments=8)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cafe_panini_press")

    stainless = model.material("stainless", rgba=(0.72, 0.74, 0.76, 1.0))
    dark_body = model.material("dark_body", rgba=(0.18, 0.19, 0.20, 1.0))
    grill_iron = model.material("grill_iron", rgba=(0.16, 0.16, 0.17, 1.0))
    handle_metal = model.material("handle_metal", rgba=(0.61, 0.63, 0.66, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    program_green = model.material("program_green", rgba=(0.20, 0.56, 0.28, 1.0))
    program_amber = model.material("program_amber", rgba=(0.78, 0.46, 0.10, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.43, 0.40, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=dark_body,
        name="lower_body",
    )
    base.visual(
        Box((0.15, 0.40, 0.060)),
        origin=Origin(xyz=(-0.14, 0.0, 0.062)),
        material=stainless,
        name="rear_housing",
    )
    base.visual(
        Box((0.33, 0.32, 0.010)),
        origin=Origin(xyz=(0.02, 0.0, 0.073)),
        material=stainless,
        name="top_deck",
    )
    base.visual(
        Box((0.32, 0.30, 0.016)),
        origin=Origin(xyz=(0.015, 0.0, 0.080)),
        material=grill_iron,
        name="lower_plate",
    )
    base.visual(
        Box((0.080, 0.060, 0.010)),
        origin=Origin(xyz=(0.148, 0.126, 0.083)),
        material=dark_body,
        name="control_console",
    )
    _add_grill_ribs(
        base,
        prefix="lower_rib",
        x_positions=(-0.115, -0.070, -0.025, 0.020, 0.065, 0.110),
        z_center=0.0895,
        rib_size=(0.014, 0.275, 0.004),
        material=grill_iron,
    )
    base.visual(
        Box((0.034, 0.300, 0.040)),
        origin=Origin(xyz=(-0.198, 0.0, 0.096)),
        material=stainless,
        name="hinge_bridge",
    )
    for index, y_pos in enumerate((-0.158, 0.158)):
        base.visual(
            Box((0.040, 0.022, 0.090)),
            origin=Origin(xyz=(-0.186, y_pos, 0.126)),
            material=stainless,
            name=f"hinge_cheek_{index}",
        )
    for index, (x_pos, y_pos) in enumerate(
        ((-0.150, -0.145), (-0.150, 0.145), (0.160, -0.145), (0.160, 0.145))
    ):
        base.visual(
            Cylinder(radius=0.017, length=0.008),
            origin=Origin(xyz=(x_pos, y_pos, 0.004)),
            material=rubber,
            name=f"foot_{index}",
        )

    lid = model.part("lid")
    lid_cap_mesh = mesh_from_geometry(
        LoftGeometry(
            [
                _loft_station(
                    x_pos=0.030,
                    width_y=0.280,
                    height_z=0.034,
                    radius=0.016,
                    z_center=0.006,
                ),
                _loft_station(
                    x_pos=0.175,
                    width_y=0.340,
                    height_z=0.050,
                    radius=0.022,
                    z_center=0.015,
                ),
                _loft_station(
                    x_pos=0.320,
                    width_y=0.310,
                    height_z=0.060,
                    radius=0.020,
                    z_center=0.004,
                ),
            ],
            cap=True,
            closed=True,
        ),
        "panini_press_lid_cap",
    )
    lid.visual(
        lid_cap_mesh,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, math.pi / 2.0)),
        material=stainless,
        name="lid_cap",
    )
    lid.visual(
        Box((0.028, 0.38, 0.090)),
        origin=Origin(xyz=(0.326, 0.0, -0.008)),
        material=stainless,
        name="front_shell",
    )
    lid.visual(
        Box((0.024, 0.280, 0.060)),
        origin=Origin(xyz=(0.020, 0.0, -0.020)),
        material=stainless,
        name="rear_shell",
    )
    for index, y_pos in enumerate((-0.178, 0.178)):
        lid.visual(
            Box((0.320, 0.022, 0.090)),
            origin=Origin(xyz=(0.168, y_pos, -0.008)),
            material=stainless,
            name=f"side_shell_{index}",
        )
    lid.visual(
        Box((0.31, 0.30, 0.018)),
        origin=Origin(xyz=(0.170, 0.0, -0.057)),
        material=grill_iron,
        name="upper_plate",
    )
    _add_grill_ribs(
        lid,
        prefix="upper_rib",
        x_positions=(0.040, 0.085, 0.130, 0.175, 0.220, 0.265),
        z_center=-0.064,
        rib_size=(0.014, 0.275, 0.006),
        material=grill_iron,
    )
    for index, y_pos in enumerate((-0.132, 0.132)):
        lid.visual(
            Box((0.030, 0.018, 0.060)),
            origin=Origin(xyz=(0.010, y_pos, -0.014)),
            material=stainless,
            name=f"hinge_arm_{index}",
        )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(xyz=(0.009, -0.105, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_metal,
        name="mount_0",
    )
    handle.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(xyz=(0.009, 0.105, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_metal,
        name="mount_1",
    )
    handle_loop = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.014, -0.105, 0.000),
                (0.026, -0.105, 0.018),
                (0.041, -0.060, 0.034),
                (0.046, 0.000, 0.040),
                (0.041, 0.060, 0.034),
                (0.026, 0.105, 0.018),
                (0.014, 0.105, 0.000),
            ],
            radius=0.0075,
            samples_per_segment=18,
            radial_segments=20,
            cap_ends=True,
        ),
        "panini_press_handle",
    )
    handle.visual(
        handle_loop,
        material=handle_metal,
        name="handle_loop",
    )

    button_specs = (
        ("button_0", "program_green", (0.148, 0.111, 0.088)),
        ("button_1", "program_amber", (0.148, 0.141, 0.088)),
    )
    for button_name, material_name, joint_xyz in button_specs:
        button = model.part(button_name)
        button.visual(
            Box((0.020, 0.020, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=material_name,
            name="cap",
        )
        button.visual(
            Box((0.012, 0.012, 0.002)),
            origin=Origin(xyz=(0.0, 0.0, 0.0055)),
            material=stainless,
            name="top_lens",
        )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(-0.186, 0.0, 0.160)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=70.0,
            velocity=1.6,
            lower=0.0,
            upper=1.25,
        ),
    )
    model.articulation(
        "lid_to_handle",
        ArticulationType.FIXED,
        parent=lid,
        child=handle,
        origin=Origin(xyz=(0.340, 0.0, 0.000)),
    )
    for button_name, _, joint_xyz in button_specs:
        button = model.get_part(button_name)
        model.articulation(
            f"base_to_{button_name}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=button,
            origin=Origin(xyz=joint_xyz),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.06,
                lower=0.0,
                upper=0.0025,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("handle")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")
    lid_hinge = object_model.get_articulation("base_to_lid")
    button_0_slide = object_model.get_articulation("base_to_button_0")
    button_1_slide = object_model.get_articulation("base_to_button_1")

    ctx.expect_gap(
        lid,
        base,
        axis="z",
        positive_elem="upper_plate",
        negative_elem="lower_plate",
        min_gap=0.001,
        max_gap=0.010,
        name="upper plate sits just above lower plate when closed",
    )
    ctx.expect_overlap(
        lid,
        base,
        axes="xy",
        elem_a="upper_plate",
        elem_b="lower_plate",
        min_overlap=0.24,
        name="upper and lower grill plates align in the closed pose",
    )
    ctx.expect_contact(
        handle,
        lid,
        elem_a="mount_0",
        elem_b="front_shell",
        name="handle mounts contact the lid shell",
    )
    ctx.expect_contact(
        button_0,
        base,
        elem_a="cap",
        elem_b="control_console",
        name="button 0 sits on the lower control console",
    )
    ctx.expect_contact(
        button_1,
        base,
        elem_a="cap",
        elem_b="control_console",
        name="button 1 sits on the lower control console",
    )

    closed_shell = ctx.part_element_world_aabb(lid, elem="front_shell")
    button_0_rest = ctx.part_world_position(button_0)
    button_1_rest = ctx.part_world_position(button_1)
    with ctx.pose({lid_hinge: 1.10}):
        open_shell = ctx.part_element_world_aabb(lid, elem="front_shell")
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            positive_elem="upper_plate",
            negative_elem="lower_plate",
            min_gap=0.045,
            name="opened lid lifts well clear of the lower plate",
        )
    with ctx.pose({button_0_slide: 0.0025}):
        button_0_pressed = ctx.part_world_position(button_0)
        button_1_when_button_0_pressed = ctx.part_world_position(button_1)
    with ctx.pose({button_1_slide: 0.0025}):
        button_1_pressed = ctx.part_world_position(button_1)
        button_0_when_button_1_pressed = ctx.part_world_position(button_0)

    ctx.check(
        "lid opens upward about the rear hinge",
        closed_shell is not None
        and open_shell is not None
        and open_shell[0][2] > closed_shell[0][2] + 0.10
        and open_shell[1][2] > closed_shell[1][2] + 0.20,
        details=f"closed={closed_shell}, open={open_shell}",
    )
    ctx.check(
        "button 0 presses downward without moving button 1",
        button_0_rest is not None
        and button_1_rest is not None
        and button_0_pressed is not None
        and button_1_when_button_0_pressed is not None
        and button_0_pressed[2] < button_0_rest[2] - 0.0015
        and abs(button_1_when_button_0_pressed[2] - button_1_rest[2]) < 1e-6,
        details=(
            f"rest_0={button_0_rest}, pressed_0={button_0_pressed}, "
            f"rest_1={button_1_rest}, unaffected_1={button_1_when_button_0_pressed}"
        ),
    )
    ctx.check(
        "button 1 presses downward without moving button 0",
        button_0_rest is not None
        and button_1_rest is not None
        and button_1_pressed is not None
        and button_0_when_button_1_pressed is not None
        and button_1_pressed[2] < button_1_rest[2] - 0.0015
        and abs(button_0_when_button_1_pressed[2] - button_0_rest[2]) < 1e-6,
        details=(
            f"rest_1={button_1_rest}, pressed_1={button_1_pressed}, "
            f"rest_0={button_0_rest}, unaffected_0={button_0_when_button_1_pressed}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
