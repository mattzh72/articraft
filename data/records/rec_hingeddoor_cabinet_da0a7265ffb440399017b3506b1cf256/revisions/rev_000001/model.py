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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sideboard_cabinet")

    walnut = model.material("walnut", rgba=(0.40, 0.27, 0.17, 1.0))
    walnut_dark = model.material("walnut_dark", rgba=(0.28, 0.18, 0.11, 1.0))
    interior_oak = model.material("interior_oak", rgba=(0.62, 0.48, 0.32, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.62, 0.63, 0.64, 1.0))
    brass = model.material("brass", rgba=(0.73, 0.59, 0.29, 1.0))

    width = 1.68
    depth = 0.46
    height = 0.66
    side_thickness = 0.028
    divider_thickness = 0.024
    top_thickness = 0.024
    bottom_thickness = 0.024
    back_thickness = 0.009
    shelf_thickness = 0.018
    plinth_height = 0.08

    door_thickness = 0.019
    door_height = 0.542
    opening_height = height - plinth_height - top_thickness - bottom_thickness
    door_center_z = plinth_height + bottom_thickness + opening_height * 0.5
    cabinet_front_y = depth * 0.5
    door_panel_center_y = cabinet_front_y + door_thickness * 0.5
    hinge_axis_y = cabinet_front_y + 0.024
    panel_local_y = door_panel_center_y - hinge_axis_y

    hinge_pin_radius = 0.004
    hinge_sleeve_radius = 0.005
    hinge_pin_length = door_height - 0.04
    hinge_leaf_width = 0.024
    hinge_leaf_depth = 0.036
    hinge_leaf_height = 0.10
    hinge_leaf_z_offset = 0.172

    handle_pin_radius = 0.0025
    handle_pin_length = 0.022
    handle_pin_local_y = panel_local_y + door_thickness * 0.5 + 0.0015

    divider_left_x = -0.2745
    divider_right_x = 0.2745
    side_inner_left = -width * 0.5 + side_thickness
    side_inner_right = width * 0.5 - side_thickness
    divider_left_faces = (
        divider_left_x - divider_thickness * 0.5,
        divider_left_x + divider_thickness * 0.5,
    )
    divider_right_faces = (
        divider_right_x - divider_thickness * 0.5,
        divider_right_x + divider_thickness * 0.5,
    )

    carcass = model.part("carcass")
    carcass.visual(
        Box((width, depth, top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, height - top_thickness * 0.5)),
        material=walnut,
        name="top_panel",
    )
    carcass.visual(
        Box((width, depth, bottom_thickness)),
        origin=Origin(xyz=(0.0, 0.0, plinth_height + bottom_thickness * 0.5)),
        material=walnut_dark,
        name="bottom_panel",
    )
    carcass.visual(
        Box((side_thickness, depth, height - plinth_height)),
        origin=Origin(
            xyz=(-width * 0.5 + side_thickness * 0.5, 0.0, plinth_height + (height - plinth_height) * 0.5)
        ),
        material=walnut,
        name="left_side",
    )
    carcass.visual(
        Box((side_thickness, depth, height - plinth_height)),
        origin=Origin(
            xyz=(width * 0.5 - side_thickness * 0.5, 0.0, plinth_height + (height - plinth_height) * 0.5)
        ),
        material=walnut,
        name="right_side",
    )
    carcass.visual(
        Box((width - 2.0 * side_thickness, back_thickness, opening_height)),
        origin=Origin(
            xyz=(
                0.0,
                -depth * 0.5 + back_thickness * 0.5,
                plinth_height + bottom_thickness + opening_height * 0.5,
            )
        ),
        material=interior_oak,
        name="back_panel",
    )
    carcass.visual(
        Box((divider_thickness, depth - back_thickness, opening_height)),
        origin=Origin(
            xyz=(
                divider_left_x,
                back_thickness * 0.5,
                plinth_height + bottom_thickness + opening_height * 0.5,
            )
        ),
        material=walnut_dark,
        name="left_divider",
    )
    carcass.visual(
        Box((divider_thickness, depth - back_thickness, opening_height)),
        origin=Origin(
            xyz=(
                divider_right_x,
                back_thickness * 0.5,
                plinth_height + bottom_thickness + opening_height * 0.5,
            )
        ),
        material=walnut_dark,
        name="right_divider",
    )
    carcass.visual(
        Box((width - 0.10, depth - 0.12, plinth_height)),
        origin=Origin(xyz=(0.0, -0.025, plinth_height * 0.5)),
        material=walnut_dark,
        name="recessed_plinth",
    )

    shelf_depth = depth - back_thickness - 0.06
    shelf_center_y = (back_thickness - 0.06) * 0.5
    shelf_center_z = plinth_height + bottom_thickness + 0.255
    for index, (x0, x1) in enumerate(
        (
            (side_inner_left, divider_left_faces[0]),
            (divider_left_faces[1], divider_right_faces[0]),
            (divider_right_faces[1], side_inner_right),
        ),
        start=1,
    ):
        carcass.visual(
            Box((x1 - x0, shelf_depth, shelf_thickness)),
            origin=Origin(xyz=((x0 + x1) * 0.5, shelf_center_y, shelf_center_z)),
            material=interior_oak,
            name=f"shelf_{index}",
        )

    hinge_defs = (
        ("left", side_inner_left + 0.0015, side_inner_left + 0.011),
        ("center", divider_right_faces[0] - 0.001, divider_right_faces[0] - 0.012),
        ("right", side_inner_right - 0.0015, side_inner_right - 0.011),
    )
    for prefix, hinge_x, leaf_x in hinge_defs:
        carcass.visual(
            Cylinder(radius=hinge_pin_radius, length=hinge_pin_length),
            origin=Origin(xyz=(hinge_x, hinge_axis_y, door_center_z)),
            material=hinge_metal,
            name=f"{prefix}_hinge_pin",
        )
        carcass.visual(
            Box((hinge_leaf_width, hinge_leaf_depth, hinge_leaf_height)),
            origin=Origin(xyz=(leaf_x, cabinet_front_y + 0.012, door_center_z + hinge_leaf_z_offset)),
            material=hinge_metal,
            name=f"{prefix}_upper_leaf",
        )
        carcass.visual(
            Box((hinge_leaf_width, hinge_leaf_depth, hinge_leaf_height)),
            origin=Origin(xyz=(leaf_x, cabinet_front_y + 0.012, door_center_z - hinge_leaf_z_offset)),
            material=hinge_metal,
            name=f"{prefix}_lower_leaf",
        )
    carcass.inertial = Inertial.from_geometry(
        Box((width, depth, height)),
        mass=58.0,
        origin=Origin(xyz=(0.0, 0.0, height * 0.5)),
    )

    def add_door(
        *,
        door_name: str,
        handle_name: str,
        hinge_joint_name: str,
        handle_joint_name: str,
        hinge_x: float,
        panel_width: float,
        panel_center_x: float,
        hinge_stile_x: float,
        handle_x: float,
        axis_z: float,
    ) -> None:
        door = model.part(door_name)
        door.visual(
            Cylinder(radius=hinge_sleeve_radius, length=hinge_pin_length),
            material=hinge_metal,
            name="hinge_sleeve",
        )
        door.visual(
            Box((0.006, 0.016, door_height)),
            origin=Origin(xyz=(hinge_stile_x, -0.008, 0.0)),
            material=walnut_dark,
            name="hinge_stile",
        )
        door.visual(
            Box((panel_width, door_thickness, door_height)),
            origin=Origin(xyz=(panel_center_x, panel_local_y, 0.0)),
            material=walnut,
            name="panel",
        )
        door.visual(
            Cylinder(radius=handle_pin_radius, length=handle_pin_length),
            origin=Origin(xyz=(handle_x, handle_pin_local_y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brass,
            name="handle_pin",
        )
        door.inertial = Inertial.from_geometry(
            Box((abs(panel_center_x) * 2.0 + 0.04, 0.04, door_height)),
            mass=8.0,
            origin=Origin(xyz=(panel_center_x * 0.5, -0.008, 0.0)),
        )

        handle = model.part(handle_name)
        handle.visual(
            Cylinder(radius=0.0045, length=handle_pin_length),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brass,
            name="pivot_sleeve",
        )
        handle.visual(
            Box((handle_pin_length * 0.90, 0.006, 0.012)),
            origin=Origin(xyz=(0.0, 0.004, -0.004)),
            material=brass,
            name="neck",
        )
        handle.visual(
            Box((handle_pin_length * 0.84, 0.009, 0.048)),
            origin=Origin(xyz=(0.0, 0.010, -0.032)),
            material=brass,
            name="pull_tab",
        )
        handle.visual(
            Box((handle_pin_length, 0.010, 0.012)),
            origin=Origin(xyz=(0.0, 0.013, -0.058)),
            material=brass,
            name="pull_lip",
        )
        handle.inertial = Inertial.from_geometry(
            Box((handle_pin_length + 0.004, 0.018, 0.074)),
            mass=0.12,
            origin=Origin(xyz=(0.0, 0.010, -0.034)),
        )

        model.articulation(
            hinge_joint_name,
            ArticulationType.REVOLUTE,
            parent=carcass,
            child=door,
            origin=Origin(xyz=(hinge_x, hinge_axis_y, door_center_z)),
            axis=(0.0, 0.0, axis_z),
            motion_limits=MotionLimits(effort=10.0, velocity=1.1, lower=0.0, upper=math.radians(105.0)),
        )
        model.articulation(
            handle_joint_name,
            ArticulationType.REVOLUTE,
            parent=door,
            child=handle,
            origin=Origin(xyz=(handle_x, handle_pin_local_y, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.3, velocity=1.2, lower=-0.35, upper=0.30),
        )

    add_door(
        door_name="door_left",
        handle_name="handle_left",
        hinge_joint_name="left_door_hinge",
        handle_joint_name="left_handle_pivot",
        hinge_x=side_inner_left + 0.0015,
        panel_width=0.545,
        panel_center_x=0.2725,
        hinge_stile_x=-0.003,
        handle_x=0.503,
        axis_z=1.0,
    )
    add_door(
        door_name="door_center",
        handle_name="handle_center",
        hinge_joint_name="center_door_hinge",
        handle_joint_name="center_handle_pivot",
        hinge_x=divider_right_faces[0] - 0.001,
        panel_width=0.525,
        panel_center_x=-0.2625,
        hinge_stile_x=0.003,
        handle_x=-0.220,
        axis_z=-1.0,
    )
    add_door(
        door_name="door_right",
        handle_name="handle_right",
        hinge_joint_name="right_door_hinge",
        handle_joint_name="right_handle_pivot",
        hinge_x=side_inner_right - 0.0015,
        panel_width=0.545,
        panel_center_x=-0.2725,
        hinge_stile_x=0.003,
        handle_x=-0.503,
        axis_z=-1.0,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    carcass = object_model.get_part("carcass")
    door_left = object_model.get_part("door_left")
    door_center = object_model.get_part("door_center")
    door_right = object_model.get_part("door_right")
    handle_left = object_model.get_part("handle_left")
    handle_center = object_model.get_part("handle_center")
    handle_right = object_model.get_part("handle_right")

    left_door_hinge = object_model.get_articulation("left_door_hinge")
    center_door_hinge = object_model.get_articulation("center_door_hinge")
    right_door_hinge = object_model.get_articulation("right_door_hinge")
    left_handle_pivot = object_model.get_articulation("left_handle_pivot")
    center_handle_pivot = object_model.get_articulation("center_handle_pivot")
    right_handle_pivot = object_model.get_articulation("right_handle_pivot")

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()

    ctx.allow_overlap(carcass, door_left, elem_a="left_hinge_pin", elem_b="hinge_sleeve", reason="Pinned hinge barrel intentionally nests through the left door sleeve.")
    ctx.allow_overlap(carcass, door_center, elem_a="center_hinge_pin", elem_b="hinge_sleeve", reason="Pinned hinge barrel intentionally nests through the center door sleeve.")
    ctx.allow_overlap(carcass, door_right, elem_a="right_hinge_pin", elem_b="hinge_sleeve", reason="Pinned hinge barrel intentionally nests through the right door sleeve.")
    ctx.allow_overlap(carcass, door_left, elem_a="left_upper_leaf", elem_b="hinge_sleeve", reason="Left hinge leaf wraps into the sleeve barrel at the hinge knuckle.")
    ctx.allow_overlap(carcass, door_left, elem_a="left_lower_leaf", elem_b="hinge_sleeve", reason="Left hinge leaf wraps into the sleeve barrel at the hinge knuckle.")
    ctx.allow_overlap(carcass, door_center, elem_a="center_upper_leaf", elem_b="hinge_sleeve", reason="Center hinge leaf wraps into the sleeve barrel at the hinge knuckle.")
    ctx.allow_overlap(carcass, door_center, elem_a="center_lower_leaf", elem_b="hinge_sleeve", reason="Center hinge leaf wraps into the sleeve barrel at the hinge knuckle.")
    ctx.allow_overlap(carcass, door_right, elem_a="right_upper_leaf", elem_b="hinge_sleeve", reason="Right hinge leaf wraps into the sleeve barrel at the hinge knuckle.")
    ctx.allow_overlap(carcass, door_right, elem_a="right_lower_leaf", elem_b="hinge_sleeve", reason="Right hinge leaf wraps into the sleeve barrel at the hinge knuckle.")
    ctx.allow_overlap(carcass, door_left, elem_a="left_upper_leaf", elem_b="panel", reason="Left hinge leaf is mortised into the back edge of the outer door.")
    ctx.allow_overlap(carcass, door_left, elem_a="left_lower_leaf", elem_b="panel", reason="Left hinge leaf is mortised into the back edge of the outer door.")
    ctx.allow_overlap(carcass, door_center, elem_a="center_upper_leaf", elem_b="hinge_stile", reason="Center hinge leaf is recessed into the stile side of the middle door.")
    ctx.allow_overlap(carcass, door_center, elem_a="center_lower_leaf", elem_b="hinge_stile", reason="Center hinge leaf is recessed into the stile side of the middle door.")
    ctx.allow_overlap(carcass, door_center, elem_a="center_upper_leaf", elem_b="panel", reason="Center hinge leaf is mortised into the back edge of the middle door near its hinge stile.")
    ctx.allow_overlap(carcass, door_center, elem_a="center_lower_leaf", elem_b="panel", reason="Center hinge leaf is mortised into the back edge of the middle door near its hinge stile.")
    ctx.allow_overlap(carcass, door_right, elem_a="right_upper_leaf", elem_b="panel", reason="Right hinge leaf is mortised into the back edge of the outer door.")
    ctx.allow_overlap(carcass, door_right, elem_a="right_lower_leaf", elem_b="panel", reason="Right hinge leaf is mortised into the back edge of the outer door.")
    ctx.allow_overlap(door_left, handle_left, elem_a="handle_pin", elem_b="pivot_sleeve", reason="Handle rotates around a short pin captured by its sleeve.")
    ctx.allow_overlap(door_center, handle_center, elem_a="handle_pin", elem_b="pivot_sleeve", reason="Handle rotates around a short pin captured by its sleeve.")
    ctx.allow_overlap(door_right, handle_right, elem_a="handle_pin", elem_b="pivot_sleeve", reason="Handle rotates around a short pin captured by its sleeve.")

    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(door_left, carcass, elem_a="hinge_sleeve", elem_b="left_hinge_pin", name="left door clipped to hinge stile")
    ctx.expect_contact(door_center, carcass, elem_a="hinge_sleeve", elem_b="center_hinge_pin", name="center door clipped to hinge stile")
    ctx.expect_contact(door_right, carcass, elem_a="hinge_sleeve", elem_b="right_hinge_pin", name="right door clipped to hinge stile")
    ctx.expect_contact(handle_left, door_left, elem_a="pivot_sleeve", elem_b="handle_pin", name="left handle mounted to door")
    ctx.expect_contact(handle_center, door_center, elem_a="pivot_sleeve", elem_b="handle_pin", name="center handle mounted to door")
    ctx.expect_contact(handle_right, door_right, elem_a="pivot_sleeve", elem_b="handle_pin", name="right handle mounted to door")

    ctx.expect_gap(door_center, door_left, axis="x", min_gap=0.001, max_gap=0.005, positive_elem="panel", negative_elem="panel", name="left and center doors meet with a small gap")
    ctx.expect_gap(door_right, door_center, axis="x", min_gap=0.001, max_gap=0.005, positive_elem="panel", negative_elem="panel", name="center and right doors meet with a small gap")
    ctx.expect_overlap(door_left, door_center, axes="yz", min_overlap=0.015, elem_a="panel", elem_b="panel", name="left and center door faces align across the front")
    ctx.expect_overlap(door_center, door_right, axes="yz", min_overlap=0.015, elem_a="panel", elem_b="panel", name="center and right door faces align across the front")

    left_closed_panel = ctx.part_element_world_aabb(door_left, elem="panel")
    center_closed_panel = ctx.part_element_world_aabb(door_center, elem="panel")
    right_closed_panel = ctx.part_element_world_aabb(door_right, elem="panel")
    left_closed_handle = ctx.part_element_world_aabb(handle_left, elem="pull_tab")
    assert left_closed_panel is not None
    assert center_closed_panel is not None
    assert right_closed_panel is not None
    assert left_closed_handle is not None

    with ctx.pose({left_door_hinge: math.radians(82.0)}):
        left_open_panel = ctx.part_element_world_aabb(door_left, elem="panel")
        assert left_open_panel is not None
        assert left_open_panel[1][1] > left_closed_panel[1][1] + 0.18
        ctx.expect_contact(door_left, carcass, elem_a="hinge_sleeve", elem_b="left_hinge_pin", name="left door stays supported while open")
    with ctx.pose({center_door_hinge: math.radians(82.0)}):
        center_open_panel = ctx.part_element_world_aabb(door_center, elem="panel")
        assert center_open_panel is not None
        assert center_open_panel[1][1] > center_closed_panel[1][1] + 0.18
        ctx.expect_contact(door_center, carcass, elem_a="hinge_sleeve", elem_b="center_hinge_pin", name="center door stays supported while open")
    with ctx.pose({right_door_hinge: math.radians(82.0)}):
        right_open_panel = ctx.part_element_world_aabb(door_right, elem="panel")
        assert right_open_panel is not None
        assert right_open_panel[1][1] > right_closed_panel[1][1] + 0.18
        ctx.expect_contact(door_right, carcass, elem_a="hinge_sleeve", elem_b="right_hinge_pin", name="right door stays supported while open")

    with ctx.pose({left_handle_pivot: 0.25}):
        left_open_handle = ctx.part_element_world_aabb(handle_left, elem="pull_tab")
        assert left_open_handle is not None
        assert left_open_handle[1][1] > left_closed_handle[1][1] + 0.008
        ctx.expect_contact(handle_left, door_left, elem_a="pivot_sleeve", elem_b="handle_pin", name="left handle stays on its pivot while rotated")
    with ctx.pose({center_handle_pivot: 0.25}):
        ctx.expect_contact(handle_center, door_center, elem_a="pivot_sleeve", elem_b="handle_pin", name="center handle stays on its pivot while rotated")
    with ctx.pose({right_handle_pivot: 0.25}):
        ctx.expect_contact(handle_right, door_right, elem_a="pivot_sleeve", elem_b="handle_pin", name="right handle stays on its pivot while rotated")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
