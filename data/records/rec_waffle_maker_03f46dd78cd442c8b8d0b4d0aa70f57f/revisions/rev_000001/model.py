from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
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


def _lofted_rounded_slab(
    lower_size: tuple[float, float],
    upper_size: tuple[float, float],
    height: float,
    *,
    lower_radius: float,
    upper_radius: float,
):
    lower = [(x, y, 0.0) for x, y in rounded_rect_profile(*lower_size, lower_radius, corner_segments=8)]
    upper = [(x, y, height) for x, y in rounded_rect_profile(*upper_size, upper_radius, corner_segments=8)]
    return LoftGeometry([lower, upper], cap=True, closed=True)


def _rounded_prism(size: tuple[float, float, float], radius: float):
    return ExtrudeGeometry(
        rounded_rect_profile(size[0], size[1], radius, corner_segments=8),
        size[2],
        center=True,
        closed=True,
    )


def _add_plate_grid(
    part,
    *,
    center_x: float,
    center_z: float,
    material: Material,
    prefix: str,
    downward: bool = False,
) -> None:
    """Add a connected waffle-grid ridge set over a rectangular plate."""
    rib_z = center_z - 0.004 if downward else center_z + 0.0035
    rib_height = 0.003
    for index, y in enumerate((-0.066, -0.033, 0.000, 0.033, 0.066)):
        if prefix == "lower_grid" and index == 0:
            rib_name = "lower_grid_rib_y_0"
        elif prefix == "upper_grid" and index == 0:
            rib_name = "upper_grid_rib_y_0"
        else:
            rib_name = f"{prefix}_rib_y_{index}"
        part.visual(
            Box((0.190, 0.004, rib_height)),
            origin=Origin(xyz=(center_x, y, rib_z)),
            material=material,
            name=rib_name,
        )
    for index, x in enumerate((-0.078, -0.039, 0.000, 0.039, 0.078)):
        part.visual(
            Box((0.004, 0.168, rib_height)),
            origin=Origin(xyz=(center_x + x, 0.0, rib_z)),
            material=material,
            name=f"{prefix}_rib_x_{index}",
        )


def _add_plate_break_frame(
    part,
    *,
    center_x: float,
    center_z: float,
    shell_material: Material,
    seam_material: Material,
    prefix: str,
) -> None:
    plate_x = 0.215
    plate_y = 0.190
    rim = 0.010
    outer_y = plate_y + 2.0 * rim
    part.visual(
        Box((rim, outer_y, 0.007)),
        origin=Origin(xyz=(center_x - plate_x / 2.0 - rim / 2.0, 0.0, center_z)),
        material=shell_material,
        name=f"{prefix}_front_rim",
    )
    part.visual(
        Box((rim, outer_y, 0.007)),
        origin=Origin(xyz=(center_x + plate_x / 2.0 + rim / 2.0, 0.0, center_z)),
        material=shell_material,
        name=f"{prefix}_rear_rim",
    )
    part.visual(
        Box((plate_x, rim, 0.007)),
        origin=Origin(xyz=(center_x, -plate_y / 2.0 - rim / 2.0, center_z)),
        material=shell_material,
        name=f"{prefix}_side_rim_0",
    )
    part.visual(
        Box((plate_x, rim, 0.007)),
        origin=Origin(xyz=(center_x, plate_y / 2.0 + rim / 2.0, center_z)),
        material=shell_material,
        name=f"{prefix}_side_rim_1",
    )

    # Thin dark break lines make the shell split read as a real inserted plate.
    seam_z = center_z + 0.003
    part.visual(
        Box((0.003, plate_y + 0.008, 0.002)),
        origin=Origin(xyz=(center_x - plate_x / 2.0 - 0.0015, 0.0, seam_z)),
        material=seam_material,
        name=f"{prefix}_front_break",
    )
    part.visual(
        Box((0.003, plate_y + 0.008, 0.002)),
        origin=Origin(xyz=(center_x + plate_x / 2.0 + 0.0015, 0.0, seam_z)),
        material=seam_material,
        name=f"{prefix}_rear_break",
    )
    part.visual(
        Box((plate_x + 0.006, 0.003, 0.002)),
        origin=Origin(xyz=(center_x, -plate_y / 2.0 - 0.0015, seam_z)),
        material=seam_material,
        name=f"{prefix}_side_break_0",
    )
    part.visual(
        Box((plate_x + 0.006, 0.003, 0.002)),
        origin=Origin(xyz=(center_x, plate_y / 2.0 + 0.0015, seam_z)),
        material=seam_material,
        name=f"{prefix}_side_break_1",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="family_waffle_iron")

    warm_plastic = model.material("warm_black_plastic", rgba=(0.075, 0.070, 0.065, 1.0))
    graphite = model.material("graphite_shell", rgba=(0.18, 0.18, 0.17, 1.0))
    seam_black = model.material("black_shadow_seam", rgba=(0.015, 0.014, 0.013, 1.0))
    plate_dark = model.material("seasoned_cast_plate", rgba=(0.055, 0.055, 0.052, 1.0))
    plate_ridge = model.material("plate_highlights", rgba=(0.105, 0.105, 0.097, 1.0))
    handle_black = model.material("cool_touch_handle", rgba=(0.025, 0.025, 0.024, 1.0))
    hinge_metal = model.material("brushed_hinge_pin", rgba=(0.62, 0.60, 0.55, 1.0))
    rubber = model.material("matte_rubber_feet", rgba=(0.020, 0.020, 0.020, 1.0))
    button_blue = model.material("blue_program_button", rgba=(0.08, 0.35, 0.62, 1.0))
    button_amber = model.material("amber_program_button", rgba=(0.95, 0.55, 0.13, 1.0))
    button_icon = model.material("button_icon_white", rgba=(0.95, 0.94, 0.88, 1.0))

    hinge_x = 0.145
    hinge_z = 0.088

    base = model.part("base")
    base.visual(
        mesh_from_geometry(
            _lofted_rounded_slab(
                (0.345, 0.300),
                (0.325, 0.284),
                0.050,
                lower_radius=0.040,
                upper_radius=0.030,
            ),
            "base_broad_shell",
        ),
        origin=Origin(xyz=(-0.015, 0.0, 0.012)),
        material=graphite,
        name="base_shell",
    )
    for index, (x, y) in enumerate(((-0.140, -0.112), (-0.140, 0.112), (0.095, -0.112), (0.095, 0.112))):
        base.visual(
            Box((0.052, 0.030, 0.010)),
            origin=Origin(xyz=(x, y, 0.007)),
            material=rubber,
            name=f"foot_{index}",
        )

    base.visual(
        Box((0.215, 0.190, 0.004)),
        origin=Origin(xyz=(-0.020, 0.0, 0.064)),
        material=plate_dark,
        name="lower_cooking_plate",
    )
    _add_plate_break_frame(
        base,
        center_x=-0.020,
        center_z=0.0655,
        shell_material=warm_plastic,
        seam_material=seam_black,
        prefix="lower_plate",
    )
    _add_plate_grid(
        base,
        center_x=-0.020,
        center_z=0.064,
        material=plate_ridge,
        prefix="lower_grid",
    )

    # Rear hinge ears are split into two outer barrels so the lid's center
    # knuckle can rotate between them without intersecting in the closed pose.
    for index, y in enumerate((-0.092, 0.092)):
        base.visual(
            Box((0.024, 0.074, 0.034)),
            origin=Origin(xyz=(hinge_x, y, 0.072)),
            material=warm_plastic,
            name=f"hinge_stand_{index}",
        )
        base.visual(
            Cylinder(radius=0.010, length=0.070),
            origin=Origin(xyz=(hinge_x, y, hinge_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hinge_metal,
            name=f"hinge_barrel_{index}",
        )

    button_socket_positions = [(-0.172, 0.089), (-0.172, 0.124)]
    for index, (x, y) in enumerate(button_socket_positions):
        socket_name = "button_socket_0" if index == 0 else "button_socket_1"
        base.visual(
            Box((0.034, 0.030, 0.004)),
            origin=Origin(xyz=(x, y, 0.064)),
            material=seam_black,
            name=socket_name,
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(
            _lofted_rounded_slab(
                (0.304, 0.272),
                (0.296, 0.264),
                0.042,
                lower_radius=0.030,
                upper_radius=0.026,
            ),
            "flat_upper_lid",
        ),
        # The local rear edge of this rounded rectangle lies on the hinge line.
        origin=Origin(xyz=(-0.172, 0.0, -0.002)),
        material=graphite,
        name="lid_shell",
    )
    lid.visual(
        Box((0.215, 0.190, 0.006)),
        origin=Origin(xyz=(-0.165, 0.0, -0.010)),
        material=plate_dark,
        name="upper_cooking_plate",
    )
    _add_plate_break_frame(
        lid,
        center_x=-0.165,
        center_z=-0.005,
        shell_material=warm_plastic,
        seam_material=seam_black,
        prefix="upper_plate",
    )
    _add_plate_grid(
        lid,
        center_x=-0.165,
        center_z=-0.010,
        material=plate_ridge,
        prefix="upper_grid",
        downward=True,
    )
    lid.visual(
        Box((0.038, 0.096, 0.014)),
        origin=Origin(xyz=(-0.018, 0.0, 0.001)),
        material=warm_plastic,
        name="hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=0.010, length=0.114),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_metal,
        name="hinge_barrel",
    )

    # Centered cool-touch front grip, connected to the lid by two stout saddles.
    for index, y in enumerate((-0.055, 0.055)):
        lid.visual(
            Box((0.040, 0.020, 0.026)),
            origin=Origin(xyz=(-0.294, y, 0.014)),
            material=handle_black,
            name=f"grip_saddle_{index}",
        )
    lid.visual(
        Cylinder(radius=0.014, length=0.146),
        origin=Origin(xyz=(-0.323, 0.0, 0.018), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_black,
        name="front_grip",
    )

    lid_hinge = model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        # The closed lid extends along local -X from the rear hinge line.
        # +Y therefore raises the front grip for positive opening motion.
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.6, lower=0.0, upper=1.18),
    )
    lid_hinge.meta["qc_samples"] = [0.0, 0.75, 1.18]

    button_mesh = mesh_from_geometry(_rounded_prism((0.028, 0.024, 0.010), 0.006), "program_button_cap")
    for index, ((x, y), material) in enumerate(zip(button_socket_positions, (button_blue, button_amber))):
        button = model.part(f"program_button_{index}")
        button.visual(
            button_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.005)),
            material=material,
            name="button_cap",
        )
        button.visual(
            Box((0.014, 0.0025, 0.0007)),
            origin=Origin(xyz=(0.0, 0.0, 0.0102)),
            material=button_icon,
            name="button_icon",
        )
        joint = model.articulation(
            f"base_to_program_button_{index}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=button,
            origin=Origin(xyz=(x, y, 0.066)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=3.5, velocity=0.04, lower=0.0, upper=0.004),
        )
        joint.meta["qc_samples"] = [0.0, 0.002, 0.004]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("base_to_lid")
    button_0 = object_model.get_part("program_button_0")
    button_1 = object_model.get_part("program_button_1")
    button_joint_0 = object_model.get_articulation("base_to_program_button_0")
    button_joint_1 = object_model.get_articulation("base_to_program_button_1")

    for index, button in enumerate((button_0, button_1)):
        ctx.allow_overlap(
            button,
            base,
            elem_a="button_cap",
            elem_b=f"button_socket_{index}",
            reason="Each raised program button intentionally travels down into its own shallow socket when pressed.",
        )
        ctx.expect_contact(
            button,
            base,
            elem_a="button_cap",
            elem_b=f"button_socket_{index}",
            name=f"program button {index} rests on its discrete socket",
        )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_overlap(
            lid,
            base,
            axes="xy",
            elem_a="upper_cooking_plate",
            elem_b="lower_cooking_plate",
            min_overlap=0.18,
            name="upper and lower waffle plates align when closed",
        )
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            positive_elem="upper_grid_rib_y_0",
            negative_elem="lower_grid_rib_y_0",
            min_gap=0.002,
            max_gap=0.008,
            name="closed waffle grids have a real air gap",
        )

    rest_grip = ctx.part_element_world_aabb(lid, elem="front_grip")
    with ctx.pose({hinge: 1.05}):
        open_grip = ctx.part_element_world_aabb(lid, elem="front_grip")
    ctx.check(
        "lid hinge lifts the front grip upward",
        rest_grip is not None
        and open_grip is not None
        and open_grip[0][2] > rest_grip[0][2] + 0.12,
        details=f"rest_grip={rest_grip}, open_grip={open_grip}",
    )

    ctx.expect_gap(
        button_1,
        button_0,
        axis="y",
        min_gap=0.006,
        name="two program buttons remain visibly separate",
    )

    rest_button_0 = ctx.part_world_position(button_0)
    rest_button_1 = ctx.part_world_position(button_1)
    with ctx.pose({button_joint_0: 0.004}):
        pressed_button_0 = ctx.part_world_position(button_0)
        unpressed_button_1 = ctx.part_world_position(button_1)
        ctx.expect_gap(
            button_0,
            base,
            axis="z",
            positive_elem="button_cap",
            negative_elem="button_socket_0",
            max_penetration=0.0045,
            name="pressed button 0 remains captured in its socket",
        )
    with ctx.pose({button_joint_1: 0.004}):
        pressed_button_1 = ctx.part_world_position(button_1)

    ctx.check(
        "program button 0 depresses independently",
        rest_button_0 is not None
        and pressed_button_0 is not None
        and rest_button_1 is not None
        and unpressed_button_1 is not None
        and pressed_button_0[2] < rest_button_0[2] - 0.003
        and abs(unpressed_button_1[2] - rest_button_1[2]) < 1e-6,
        details=f"rest0={rest_button_0}, pressed0={pressed_button_0}, rest1={rest_button_1}, other={unpressed_button_1}",
    )
    ctx.check(
        "program button 1 depresses independently",
        rest_button_1 is not None
        and pressed_button_1 is not None
        and pressed_button_1[2] < rest_button_1[2] - 0.003,
        details=f"rest1={rest_button_1}, pressed1={pressed_button_1}",
    )
    return ctx.report()


object_model = build_object_model()
