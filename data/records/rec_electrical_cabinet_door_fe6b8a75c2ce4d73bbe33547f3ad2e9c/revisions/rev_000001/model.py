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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _add_mesh_grid(
    part,
    *,
    x0: float,
    x1: float,
    z0: float,
    z1: float,
    y: float,
    bar_width: float,
    bar_depth: float,
    vertical_count: int,
    horizontal_count: int,
    material,
) -> None:
    inner_width = x1 - x0
    inner_height = z1 - z0
    z_center = (z0 + z1) * 0.5
    x_center = (x0 + x1) * 0.5

    for idx in range(vertical_count):
        frac = (idx + 1) / (vertical_count + 1)
        x = x0 + inner_width * frac
        part.visual(
            Box((bar_width, bar_depth, inner_height + bar_width * 2.0)),
            origin=Origin(xyz=(x, y, z_center)),
            material=material,
            name=f"mesh_vertical_{idx + 1}",
        )

    for idx in range(horizontal_count):
        frac = (idx + 1) / (horizontal_count + 1)
        z = z0 + inner_height * frac
        part.visual(
            Box((inner_width + bar_width * 2.0, bar_depth, bar_width)),
            origin=Origin(xyz=(x_center, y, z)),
            material=material,
            name=f"mesh_horizontal_{idx + 1}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="server_rack_front_door_enclosure")

    cabinet_width = 0.60
    cabinet_depth = 1.00
    cabinet_height = 2.00
    post_width = 0.045
    post_depth = 0.080
    rail_height = 0.045
    plinth_height = 0.080
    rack_rail_height = 1.80

    door_width = 0.540
    door_height = 1.920
    door_thickness = 0.032
    door_stile = 0.060
    door_rail = 0.060
    door_bezel = 0.030
    bezel_overlap = 0.004
    mesh_bar = 0.004
    mesh_depth = 0.004
    hinge_radius = 0.012
    hinge_length = 0.180
    hinge_offset_z = 0.690

    powder_black = model.material("powder_black", rgba=(0.10, 0.11, 0.12, 1.0))
    textured_charcoal = model.material("textured_charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.27, 0.29, 0.31, 1.0))
    rack_gray = model.material("rack_gray", rgba=(0.39, 0.41, 0.43, 1.0))
    latch_steel = model.material("latch_steel", rgba=(0.66, 0.68, 0.70, 1.0))

    frame = model.part("cabinet_frame")
    frame.inertial = Inertial.from_geometry(
        Box((cabinet_width, cabinet_depth, cabinet_height)),
        mass=110.0,
        origin=Origin(xyz=(0.0, 0.0, cabinet_height * 0.5)),
    )

    front_y = -cabinet_depth * 0.5 + post_depth * 0.5
    rear_y = cabinet_depth * 0.5 - post_depth * 0.5
    left_x = -cabinet_width * 0.5 + post_width * 0.5
    right_x = cabinet_width * 0.5 - post_width * 0.5

    for name, x, y in (
        ("front_left_post", left_x, front_y),
        ("front_right_post", right_x, front_y),
        ("rear_left_post", left_x, rear_y),
        ("rear_right_post", right_x, rear_y),
    ):
        frame.visual(
            Box((post_width, post_depth, cabinet_height)),
            origin=Origin(xyz=(x, y, cabinet_height * 0.5)),
            material=powder_black,
            name=name,
        )

    front_span = cabinet_width - 2.0 * post_width
    side_span = cabinet_depth - 2.0 * post_depth
    for name, y, z in (
        ("front_bottom_rail", front_y, rail_height * 0.5),
        ("front_top_rail", front_y, cabinet_height - rail_height * 0.5),
        ("rear_bottom_rail", rear_y, rail_height * 0.5),
        ("rear_top_rail", rear_y, cabinet_height - rail_height * 0.5),
    ):
        frame.visual(
            Box((front_span, post_depth, rail_height)),
            origin=Origin(xyz=(0.0, y, z)),
            material=powder_black,
            name=name,
        )

    for name, x, z in (
        ("left_bottom_side_rail", left_x, rail_height * 0.5),
        ("left_top_side_rail", left_x, cabinet_height - rail_height * 0.5),
        ("right_bottom_side_rail", right_x, rail_height * 0.5),
        ("right_top_side_rail", right_x, cabinet_height - rail_height * 0.5),
    ):
        frame.visual(
            Box((post_width, side_span, rail_height)),
            origin=Origin(xyz=(x, 0.0, z)),
            material=powder_black,
            name=name,
        )

    frame.visual(
        Box((cabinet_width - 0.050, cabinet_depth - 0.050, plinth_height)),
        origin=Origin(xyz=(0.0, 0.0, plinth_height * 0.5)),
        material=textured_charcoal,
        name="base_plinth",
    )
    frame.visual(
        Box((cabinet_width - 0.090, cabinet_depth - 0.090, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, cabinet_height - 0.010)),
        material=textured_charcoal,
        name="top_cover",
    )

    front_rail_x = cabinet_width * 0.5 - 0.095
    rear_rail_y = cabinet_depth * 0.5 - 0.115
    front_rail_y = -cabinet_depth * 0.5 + 0.115
    for name, x, y in (
        ("front_left_rack_rail", -front_rail_x, front_rail_y),
        ("front_right_rack_rail", front_rail_x, front_rail_y),
        ("rear_left_rack_rail", -front_rail_x, rear_rail_y),
        ("rear_right_rack_rail", front_rail_x, rear_rail_y),
    ):
        frame.visual(
            Box((0.020, 0.030, rack_rail_height)),
            origin=Origin(xyz=(x, y, plinth_height + rack_rail_height * 0.5)),
            material=rack_gray,
            name=name,
        )

    hinge_bracket_x = -door_width * 0.5 - 0.014
    hinge_bracket_y = -cabinet_depth * 0.5 + 0.010
    for name, z in (("upper_hinge_bracket", 1.000 + hinge_offset_z), ("lower_hinge_bracket", 1.000 - hinge_offset_z)):
        frame.visual(
            Box((0.024, 0.028, 0.220)),
            origin=Origin(xyz=(hinge_bracket_x, hinge_bracket_y, z)),
            material=dark_steel,
            name=name,
        )

    strike_x = cabinet_width * 0.5 - 0.020
    strike_y = -cabinet_depth * 0.5 + 0.020
    for name, z in (("upper_strike", 1.720), ("center_strike", 1.000), ("lower_strike", 0.280)):
        frame.visual(
            Box((0.018, 0.030, 0.060)),
            origin=Origin(xyz=(strike_x, strike_y, z)),
            material=dark_steel,
            name=name,
        )

    door = model.part("front_door")
    door.inertial = Inertial.from_geometry(
        Box((door_width, door_thickness, door_height)),
        mass=18.0,
        origin=Origin(xyz=(door_width * 0.5, 0.0, 0.0)),
    )

    door.visual(
        Box((door_stile, door_thickness, door_height)),
        origin=Origin(xyz=(door_stile * 0.5, 0.0, 0.0)),
        material=powder_black,
        name="door_left_stile",
    )
    door.visual(
        Box((door_stile, door_thickness, door_height)),
        origin=Origin(xyz=(door_width - door_stile * 0.5, 0.0, 0.0)),
        material=powder_black,
        name="door_right_stile",
    )
    door.visual(
        Box((door_width - 2.0 * door_stile, door_thickness, door_rail)),
        origin=Origin(xyz=(door_width * 0.5, 0.0, door_height * 0.5 - door_rail * 0.5)),
        material=powder_black,
        name="door_top_rail",
    )
    door.visual(
        Box((door_width - 2.0 * door_stile, door_thickness, door_rail)),
        origin=Origin(xyz=(door_width * 0.5, 0.0, -door_height * 0.5 + door_rail * 0.5)),
        material=powder_black,
        name="door_bottom_rail",
    )

    bezel_depth = door_thickness * 0.70
    bezel_y = 0.006
    door.visual(
        Box((door_bezel, bezel_depth, door_height - 0.120)),
        origin=Origin(
            xyz=(door_stile + door_bezel * 0.5 - bezel_overlap, bezel_y, 0.0),
        ),
        material=textured_charcoal,
        name="bezel_left",
    )
    door.visual(
        Box((door_bezel, bezel_depth, door_height - 0.120)),
        origin=Origin(
            xyz=(door_width - door_stile - door_bezel * 0.5 + bezel_overlap, bezel_y, 0.0),
        ),
        material=textured_charcoal,
        name="bezel_right",
    )
    door.visual(
        Box((door_width - 2.0 * (door_stile + door_bezel) + 2.0 * bezel_overlap, bezel_depth, door_bezel)),
        origin=Origin(
            xyz=(door_width * 0.5, bezel_y, door_height * 0.5 - door_rail - door_bezel * 0.5 + bezel_overlap),
        ),
        material=textured_charcoal,
        name="bezel_top",
    )
    door.visual(
        Box((door_width - 2.0 * (door_stile + door_bezel) + 2.0 * bezel_overlap, bezel_depth, door_bezel)),
        origin=Origin(
            xyz=(door_width * 0.5, bezel_y, -door_height * 0.5 + door_rail + door_bezel * 0.5 - bezel_overlap),
        ),
        material=textured_charcoal,
        name="bezel_bottom",
    )

    mesh_x0 = door_stile + door_bezel + 0.006
    mesh_x1 = door_width - door_stile - door_bezel - 0.006
    mesh_z0 = -door_height * 0.5 + door_rail + door_bezel + 0.006
    mesh_z1 = door_height * 0.5 - door_rail - door_bezel - 0.006
    _add_mesh_grid(
        door,
        x0=mesh_x0,
        x1=mesh_x1,
        z0=mesh_z0,
        z1=mesh_z1,
        y=0.006,
        bar_width=mesh_bar,
        bar_depth=mesh_depth,
        vertical_count=8,
        horizontal_count=18,
        material=rack_gray,
    )

    for name, z in (("upper_knuckle", hinge_offset_z), ("lower_knuckle", -hinge_offset_z)):
        door.visual(
            Cylinder(radius=hinge_radius, length=hinge_length),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=dark_steel,
            name=name,
        )
    for name, z in (("upper_hinge_leaf", hinge_offset_z), ("lower_hinge_leaf", -hinge_offset_z)):
        door.visual(
            Box((0.012, 0.012, 0.220)),
            origin=Origin(xyz=(-0.006, 0.010, z)),
            material=dark_steel,
            name=name,
        )

    door.visual(
        Box((0.080, 0.022, 0.160)),
        origin=Origin(xyz=(door_width - 0.060, 0.008, 0.0)),
        material=textured_charcoal,
        name="latch_gearbox",
    )
    door.visual(
        Cylinder(radius=0.006, length=door_height - 0.280),
        origin=Origin(xyz=(door_width - 0.035, 0.010, 0.0)),
        material=latch_steel,
        name="latch_rod",
    )
    for idx, z in enumerate((0.720, 0.0, -0.720), start=1):
        door.visual(
            Box((0.050, 0.018, 0.040)),
            origin=Origin(xyz=(door_width - 0.038, 0.010, z)),
            material=textured_charcoal,
            name=f"rod_guide_{idx}",
        )
        door.visual(
            Box((0.020, 0.010, 0.060)),
            origin=Origin(xyz=(door_width - 0.016, 0.010, z)),
            material=latch_steel,
            name=f"latch_cam_{idx}",
        )

    handle = model.part("turn_handle")
    handle.inertial = Inertial.from_geometry(
        Box((0.060, 0.020, 0.180)),
        mass=0.9,
        origin=Origin(xyz=(0.0, -0.006, 0.0)),
    )
    handle.visual(
        Cylinder(radius=0.022, length=0.012),
        origin=Origin(rpy=(pi * 0.5, 0.0, 0.0)),
        material=dark_steel,
        name="handle_boss",
    )
    handle.visual(
        Box((0.030, 0.010, 0.180)),
        origin=Origin(xyz=(0.0, -0.006, 0.0)),
        material=latch_steel,
        name="handle_paddle",
    )
    handle.visual(
        Box((0.050, 0.014, 0.032)),
        origin=Origin(xyz=(0.0, -0.009, -0.040)),
        material=latch_steel,
        name="handle_grip",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=door,
        origin=Origin(xyz=(-door_width * 0.5, -cabinet_depth * 0.5 - 0.020, 1.000)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.5, lower=0.0, upper=2.20),
    )
    model.articulation(
        "handle_turn",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle,
        origin=Origin(xyz=(door_width - 0.055, -door_thickness * 0.5 - 0.006, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.57),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    frame = object_model.get_part("cabinet_frame")
    door = object_model.get_part("front_door")
    handle = object_model.get_part("turn_handle")
    front_right_post = frame.get_visual("front_right_post")
    door_right_stile = door.get_visual("door_right_stile")
    handle_boss = handle.get_visual("handle_boss")
    door_hinge = object_model.get_articulation("door_hinge")
    handle_turn = object_model.get_articulation("handle_turn")

    ctx.check(
        "expected parts exist",
        frame is not None and door is not None and handle is not None,
        details="cabinet frame, front door, and turn handle should all be present",
    )

    with ctx.pose({door_hinge: 0.0, handle_turn: 0.0}):
        ctx.expect_overlap(
            door,
            frame,
            axes="xz",
            min_overlap=0.50,
            name="closed door covers rack opening",
        )
        ctx.expect_gap(
            frame,
            door,
            axis="y",
            positive_elem=front_right_post,
            negative_elem=door_right_stile,
            max_gap=0.020,
            max_penetration=0.0,
            name="closed door sits flush at the latch side",
        )
        ctx.expect_contact(
            handle,
            door,
            elem_a=handle_boss,
            elem_b=door_right_stile,
            name="turn handle mounts against the right stile",
        )

    closed_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.20, handle_turn: 0.0}):
        open_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door swings outward from the cabinet front",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.25,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    with ctx.pose({door_hinge: 0.0, handle_turn: 0.0}):
        rest_handle_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({door_hinge: 0.0, handle_turn: 1.35}):
        turned_handle_aabb = ctx.part_world_aabb(handle)
    rest_dx = None if rest_handle_aabb is None else rest_handle_aabb[1][0] - rest_handle_aabb[0][0]
    rest_dz = None if rest_handle_aabb is None else rest_handle_aabb[1][2] - rest_handle_aabb[0][2]
    turned_dx = None if turned_handle_aabb is None else turned_handle_aabb[1][0] - turned_handle_aabb[0][0]
    turned_dz = None if turned_handle_aabb is None else turned_handle_aabb[1][2] - turned_handle_aabb[0][2]
    ctx.check(
        "turn handle rotates from vertical toward horizontal",
        rest_dx is not None
        and rest_dz is not None
        and turned_dx is not None
        and turned_dz is not None
        and rest_dz > rest_dx
        and turned_dx > turned_dz,
        details=(
            f"rest_spans=({rest_dx}, {rest_dz}), "
            f"turned_spans=({turned_dx}, {turned_dz})"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
