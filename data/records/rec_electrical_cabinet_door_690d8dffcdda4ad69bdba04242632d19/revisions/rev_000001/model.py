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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hv_busbar_chamber")

    cabinet_paint = model.material("cabinet_paint", rgba=(0.76, 0.78, 0.81, 1.0))
    cabinet_shadow = model.material("cabinet_shadow", rgba=(0.54, 0.57, 0.61, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.17, 0.18, 0.19, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.61, 0.63, 0.67, 1.0))

    body_width = 0.84
    body_depth = 0.46
    body_height = 2.00
    wall_thickness = 0.004

    front_frame_depth = 0.022
    front_frame_y = 0.219
    opening_height = 1.88
    opening_bottom = 0.06
    stile_width = 0.05

    door_width = 0.76
    door_height = 1.88
    door_thickness = 0.026
    hinge_radius = 0.028
    door_skin_inset = 0.034

    door_hinge_x = -door_width / 2.0
    door_hinge_y = body_depth / 2.0 + 0.030
    door_hinge_z = opening_bottom

    body = model.part("body")
    body.visual(
        Box((body_width, wall_thickness, body_height)),
        origin=Origin(xyz=(0.0, -body_depth / 2.0 + wall_thickness / 2.0, body_height / 2.0)),
        material=cabinet_paint,
        name="back_panel",
    )
    body.visual(
        Box((wall_thickness, body_depth, body_height)),
        origin=Origin(xyz=(-body_width / 2.0 + wall_thickness / 2.0, 0.0, body_height / 2.0)),
        material=cabinet_paint,
        name="left_wall",
    )
    body.visual(
        Box((wall_thickness, body_depth, body_height)),
        origin=Origin(xyz=(body_width / 2.0 - wall_thickness / 2.0, 0.0, body_height / 2.0)),
        material=cabinet_paint,
        name="right_wall",
    )
    body.visual(
        Box((body_width, body_depth, wall_thickness)),
        origin=Origin(xyz=(0.0, 0.0, wall_thickness / 2.0)),
        material=cabinet_shadow,
        name="floor_panel",
    )
    body.visual(
        Box((body_width, body_depth, wall_thickness)),
        origin=Origin(xyz=(0.0, 0.0, body_height - wall_thickness / 2.0)),
        material=cabinet_paint,
        name="roof_panel",
    )
    body.visual(
        Box((0.012, front_frame_depth, 1.90)),
        origin=Origin(xyz=(-0.364, front_frame_y, 1.00)),
        material=cabinet_shadow,
        name="left_jamb",
    )
    body.visual(
        Box((stile_width, front_frame_depth, opening_height)),
        origin=Origin(xyz=(0.395, front_frame_y, opening_bottom + opening_height / 2.0)),
        material=cabinet_shadow,
        name="right_jamb",
    )
    body.visual(
        Box((0.74, front_frame_depth, 0.05)),
        origin=Origin(xyz=(0.0, front_frame_y, body_height - 0.025)),
        material=cabinet_shadow,
        name="header",
    )
    body.visual(
        Box((0.74, front_frame_depth, 0.05)),
        origin=Origin(xyz=(0.0, front_frame_y, 0.025)),
        material=cabinet_shadow,
        name="sill",
    )

    for strap_name, strap_z, strap_len, knuckle_name, knuckle_z, knuckle_len in (
        ("lower_hinge_strap", 0.60, 0.44, "lower_body_knuckle", 0.60, 0.44),
        ("upper_hinge_strap", 1.34, 0.44, "upper_body_knuckle", 1.34, 0.44),
    ):
        body.visual(
            Box((0.044, 0.030, strap_len)),
            origin=Origin(xyz=(door_hinge_x - 0.022, door_hinge_y - 0.015, strap_z)),
            material=hinge_metal,
            name=strap_name,
        )
        body.visual(
            Cylinder(radius=hinge_radius, length=knuckle_len),
            origin=Origin(xyz=(door_hinge_x, door_hinge_y, knuckle_z)),
            material=hinge_metal,
            name=knuckle_name,
        )

    body.visual(
        Box((0.020, 0.018, 0.060)),
        origin=Origin(xyz=(0.378, 0.239, 1.74)),
        material=dark_hardware,
        name="upper_keeper",
    )
    body.visual(
        Box((0.020, 0.018, 0.060)),
        origin=Origin(xyz=(0.378, 0.239, 1.00)),
        material=dark_hardware,
        name="center_keeper",
    )
    body.visual(
        Box((0.020, 0.018, 0.060)),
        origin=Origin(xyz=(0.378, 0.239, 0.24)),
        material=dark_hardware,
        name="lower_keeper",
    )

    door = model.part("door")
    door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(door_skin_inset + door_width / 2.0, 0.0, door_height / 2.0)),
        material=cabinet_paint,
        name="door_skin",
    )
    door.visual(
        Box((0.700, 0.010, 1.720)),
        origin=Origin(xyz=(door_skin_inset + door_width / 2.0, -0.008, door_height / 2.0)),
        material=cabinet_shadow,
        name="door_inner_pan",
    )
    door.visual(
        Box((0.028, 0.010, 1.54)),
        origin=Origin(xyz=(0.730, 0.015, door_height / 2.0)),
        material=dark_hardware,
        name="latch_cover",
    )
    door.visual(
        Box((0.085, 0.008, 0.34)),
        origin=Origin(xyz=(0.714, 0.017, 0.96)),
        material=dark_hardware,
        name="latch_backplate",
    )
    door.visual(
        Box((0.012, 0.010, 0.96)),
        origin=Origin(xyz=(0.758, 0.008, 1.26)),
        material=hinge_metal,
        name="upper_latch_rod",
    )
    door.visual(
        Box((0.012, 0.010, 0.74)),
        origin=Origin(xyz=(0.758, 0.008, 0.61)),
        material=hinge_metal,
        name="lower_latch_rod",
    )

    for leaf_name, leaf_z, leaf_len, knuckle_name, knuckle_z, knuckle_len in (
        ("lower_leaf", 0.18, 0.28, "lower_door_knuckle", 0.18, 0.28),
        ("middle_leaf", 0.91, 0.30, "middle_door_knuckle", 0.91, 0.30),
        ("upper_leaf", 1.66, 0.32, "upper_door_knuckle", 1.66, 0.32),
    ):
        door.visual(
            Box((0.040, 0.050, leaf_len)),
            origin=Origin(xyz=(0.020, 0.0, leaf_z)),
            material=hinge_metal,
            name=leaf_name,
        )
        door.visual(
            Cylinder(radius=hinge_radius, length=knuckle_len),
            origin=Origin(xyz=(0.0, 0.0, knuckle_z)),
            material=hinge_metal,
            name=knuckle_name,
        )

    door.visual(
        Cylinder(radius=0.010, length=0.026),
        origin=Origin(xyz=(0.769, 0.0, 1.74), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_metal,
        name="upper_draw_bolt",
    )
    door.visual(
        Cylinder(radius=0.010, length=0.026),
        origin=Origin(xyz=(0.769, 0.0, 1.00), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_metal,
        name="center_draw_bolt",
    )
    door.visual(
        Cylinder(radius=0.010, length=0.026),
        origin=Origin(xyz=(0.769, 0.0, 0.24), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_metal,
        name="lower_draw_bolt",
    )

    latch_handle = model.part("latch_handle")
    latch_handle.visual(
        Cylinder(radius=0.011, length=0.020),
        origin=Origin(xyz=(0.0, 0.013, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hinge_metal,
        name="spindle_hub",
    )
    latch_handle.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(xyz=(0.0, 0.014, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_hardware,
        name="hub_shell",
    )
    latch_handle.visual(
        Box((0.034, 0.016, 0.24)),
        origin=Origin(xyz=(0.0, 0.022, -0.12)),
        material=dark_hardware,
        name="lever_bar",
    )
    latch_handle.visual(
        Box((0.070, 0.018, 0.040)),
        origin=Origin(xyz=(0.0, 0.024, -0.22)),
        material=dark_hardware,
        name="hand_grip",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(door_hinge_x, door_hinge_y, door_hinge_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=50.0, velocity=1.4, lower=0.0, upper=2.10),
    )
    model.articulation(
        "door_to_latch_handle",
        ArticulationType.REVOLUTE,
        parent=door,
        child=latch_handle,
        origin=Origin(xyz=(0.714, 0.016, 0.96)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0, lower=-1.57, upper=1.57),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    latch_handle = object_model.get_part("latch_handle")
    door_hinge = object_model.get_articulation("body_to_door")
    handle_joint = object_model.get_articulation("door_to_latch_handle")

    def extent(aabb, axis: int) -> float | None:
        if aabb is None:
            return None
        return aabb[1][axis] - aabb[0][axis]

    with ctx.pose({door_hinge: 0.0, handle_joint: 0.0}):
        ctx.expect_gap(
            door,
            body,
            axis="y",
            positive_elem="door_skin",
            negative_elem="right_jamb",
            min_gap=0.003,
            max_gap=0.020,
            name="door skin sits just proud of the right frame stile",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="xz",
            elem_a="door_skin",
            min_overlap=0.70,
            name="inspection door covers the cubicle opening",
        )
        ctx.expect_gap(
            door,
            body,
            axis="y",
            positive_elem="upper_draw_bolt",
            negative_elem="upper_keeper",
            min_gap=0.001,
            max_gap=0.006,
            name="upper draw bolt sits close to its keeper",
        )
        ctx.expect_gap(
            door,
            body,
            axis="y",
            positive_elem="center_draw_bolt",
            negative_elem="center_keeper",
            min_gap=0.001,
            max_gap=0.006,
            name="center draw bolt sits close to its keeper",
        )
        ctx.expect_gap(
            door,
            body,
            axis="y",
            positive_elem="lower_draw_bolt",
            negative_elem="lower_keeper",
            min_gap=0.001,
            max_gap=0.006,
            name="lower draw bolt sits close to its keeper",
        )

    with ctx.pose({door_hinge: 0.0}):
        closed_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.35}):
        open_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door swings outward on the left-side hinge line",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][1] > closed_door_aabb[1][1] + 0.45,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    with ctx.pose({handle_joint: 0.0}):
        rest_handle_aabb = ctx.part_element_world_aabb(latch_handle, elem="lever_bar")
    with ctx.pose({handle_joint: 1.35}):
        turned_handle_aabb = ctx.part_element_world_aabb(latch_handle, elem="lever_bar")
    rest_x = extent(rest_handle_aabb, 0)
    rest_z = extent(rest_handle_aabb, 2)
    turned_x = extent(turned_handle_aabb, 0)
    turned_z = extent(turned_handle_aabb, 2)
    ctx.check(
        "turning handle rotates from vertical parked position to release position",
        rest_x is not None
        and rest_z is not None
        and turned_x is not None
        and turned_z is not None
        and rest_z > rest_x * 3.5
        and turned_x > turned_z * 1.6,
        details=(
            f"rest_x={rest_x}, rest_z={rest_z}, "
            f"turned_x={turned_x}, turned_z={turned_z}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
