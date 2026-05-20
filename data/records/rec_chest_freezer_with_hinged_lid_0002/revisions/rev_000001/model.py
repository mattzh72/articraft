from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import os
from math import pi

_ORIG_GETCWD = os.getcwd
_SAFE_CWD = "/tmp"


def _patched_getcwd() -> str:
    try:
        return _ORIG_GETCWD()
    except FileNotFoundError:
        return _SAFE_CWD


os.getcwd = _patched_getcwd
try:
    _ORIG_GETCWD()
except FileNotFoundError:
    try:
        os.chdir(_SAFE_CWD)
    except FileNotFoundError:
        pass

from sdk import ArticulatedObject, ArticulationType, Box, Cylinder, MotionLimits, Origin, TestContext, TestReport


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chest_freezer")

    white = model.material("appliance_white", rgba=(0.95, 0.96, 0.97, 1.0))
    liner_white = model.material("liner_white", rgba=(0.92, 0.93, 0.94, 1.0))
    hinge_gray = model.material("hinge_gray", rgba=(0.58, 0.60, 0.63, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.75, 0.77, 0.79, 1.0))
    foot_black = model.material("foot_black", rgba=(0.18, 0.18, 0.19, 1.0))

    cabinet_width = 1.08
    cabinet_depth = 0.70
    cabinet_body_height = 0.82
    feet_height = 0.03
    cabinet_top_z = feet_height + cabinet_body_height

    wall_thickness = 0.055
    floor_thickness = 0.070
    inner_width = cabinet_width - (2.0 * wall_thickness)
    inner_depth = cabinet_depth - (2.0 * wall_thickness)

    lid_width = 1.10
    lid_depth = 0.74
    lid_panel_thickness = 0.030
    lid_insert_thickness = 0.028
    lid_center_y = 0.392

    hinge_axis_y = -(cabinet_depth / 2.0) - 0.008
    hinge_axis_z = cabinet_top_z + 0.012
    hinge_x_positions = (-0.36, 0.36)

    barrel_radius = 0.008
    outer_knuckle_length = 0.016
    center_knuckle_length = 0.020
    knuckle_offset = 0.018

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((wall_thickness, cabinet_depth, cabinet_body_height)),
        origin=Origin(
            xyz=(
                -(cabinet_width / 2.0) + (wall_thickness / 2.0),
                0.0,
                feet_height + (cabinet_body_height / 2.0),
            )
        ),
        material=white,
        name="left_wall",
    )
    cabinet.visual(
        Box((wall_thickness, cabinet_depth, cabinet_body_height)),
        origin=Origin(
            xyz=(
                (cabinet_width / 2.0) - (wall_thickness / 2.0),
                0.0,
                feet_height + (cabinet_body_height / 2.0),
            )
        ),
        material=white,
        name="right_wall",
    )
    cabinet.visual(
        Box((inner_width, wall_thickness, cabinet_body_height)),
        origin=Origin(
            xyz=(
                0.0,
                (cabinet_depth / 2.0) - (wall_thickness / 2.0),
                feet_height + (cabinet_body_height / 2.0),
            )
        ),
        material=white,
        name="front_wall",
    )
    cabinet.visual(
        Box((inner_width, wall_thickness, cabinet_body_height)),
        origin=Origin(
            xyz=(
                0.0,
                -(cabinet_depth / 2.0) + (wall_thickness / 2.0),
                feet_height + (cabinet_body_height / 2.0),
            )
        ),
        material=white,
        name="rear_wall",
    )
    cabinet.visual(
        Box((inner_width, inner_depth, floor_thickness)),
        origin=Origin(xyz=(0.0, 0.0, feet_height + (floor_thickness / 2.0))),
        material=liner_white,
        name="liner_floor",
    )
    cabinet.visual(
        Box((0.24, 0.18, 0.17)),
        origin=Origin(
            xyz=(
                (inner_width / 2.0) - 0.12,
                -(inner_depth / 2.0) + 0.09,
                feet_height + floor_thickness + 0.085,
            )
        ),
        material=liner_white,
        name="compressor_hump",
    )
    cabinet.visual(
        Box((0.28, 0.018, 0.040)),
        origin=Origin(
            xyz=(
                0.0,
                (cabinet_depth / 2.0) + 0.009,
                cabinet_top_z - 0.090,
            )
        ),
        material=trim_gray,
        name="front_handle_recess",
    )
    for foot_name, foot_x, foot_y in (
        ("front_left_foot", -0.44, 0.26),
        ("front_right_foot", 0.44, 0.26),
        ("rear_left_foot", -0.44, -0.26),
        ("rear_right_foot", 0.44, -0.26),
    ):
        cabinet.visual(
            Box((0.06, 0.06, feet_height)),
            origin=Origin(xyz=(foot_x, foot_y, feet_height / 2.0)),
            material=foot_black,
            name=foot_name,
        )

    for side_name, hinge_x in (("left", hinge_x_positions[0]), ("right", hinge_x_positions[1])):
        for position_name, x_center in (
            ("outer_knuckle_a", hinge_x - knuckle_offset),
            ("outer_knuckle_b", hinge_x + knuckle_offset),
        ):
            cabinet.visual(
                Box((outer_knuckle_length, 0.030, 0.036)),
                origin=Origin(xyz=(x_center, hinge_axis_y, cabinet_top_z - 0.008)),
                material=hinge_gray,
                name=f"{side_name}_{position_name}_leaf",
            )
            cabinet.visual(
                Cylinder(radius=barrel_radius, length=outer_knuckle_length),
                origin=Origin(
                    xyz=(x_center, hinge_axis_y, hinge_axis_z),
                    rpy=(0.0, pi / 2.0, 0.0),
                ),
                material=hinge_gray,
                name=f"{side_name}_{position_name}",
            )

    lid = model.part("lid")
    lid.visual(
        Box((lid_width, lid_depth, lid_panel_thickness)),
        origin=Origin(xyz=(0.0, lid_center_y, 0.003)),
        material=white,
        name="top_panel",
    )
    lid.visual(
        Box((inner_width - 0.012, inner_depth - 0.014, lid_insert_thickness)),
        origin=Origin(xyz=(0.0, (inner_depth / 2.0) + 0.061, -0.026)),
        material=liner_white,
        name="lid_plug",
    )
    lid.visual(
        Box((0.34, 0.038, 0.050)),
        origin=Origin(xyz=(0.0, lid_center_y + (lid_depth / 2.0) + 0.010, -0.010)),
        material=trim_gray,
        name="front_handle",
    )

    for side_name, hinge_x in (("left", hinge_x_positions[0]), ("right", hinge_x_positions[1])):
        lid.visual(
            Box((0.024, 0.034, 0.022)),
            origin=Origin(xyz=(hinge_x, 0.012, -0.003)),
            material=hinge_gray,
            name=f"{side_name}_hinge_strap",
        )
        lid.visual(
            Cylinder(radius=barrel_radius, length=center_knuckle_length),
            origin=Origin(
                xyz=(hinge_x, 0.0, 0.0),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=hinge_gray,
            name=f"{side_name}_center_knuckle",
        )

    model.articulation(
        "rear_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.5,
            lower=0.0,
            upper=1.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, seed=0)

    cabinet = object_model.get_part("cabinet")
    lid = object_model.get_part("lid")
    rear_hinge = object_model.get_articulation("rear_hinge")

    def dims_from_aabb(aabb):
        if aabb is None:
            return None
        low, high = aabb
        return (high[0] - low[0], high[1] - low[1], high[2] - low[2])

    def center_from_aabb(aabb):
        if aabb is None:
            return None
        low, high = aabb
        return (
            (low[0] + high[0]) / 2.0,
            (low[1] + high[1]) / 2.0,
            (low[2] + high[2]) / 2.0,
        )

    def require_elem_center(part_name: str, elem_name: str):
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        ctx.check(
            f"{part_name}_{elem_name}_present",
            aabb is not None,
            f"Expected visual {elem_name!r} on part {part_name!r}.",
        )
        return center_from_aabb(aabb)

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    cabinet_aabb = ctx.part_world_aabb(cabinet)
    lid_aabb = ctx.part_world_aabb(lid)
    cabinet_dims = dims_from_aabb(cabinet_aabb)
    lid_dims = dims_from_aabb(lid_aabb)

    ctx.check("cabinet_exists", cabinet_aabb is not None, "Cabinet geometry should resolve.")
    ctx.check("lid_exists", lid_aabb is not None, "Lid geometry should resolve.")
    ctx.check(
        "rear_hinge_axis_is_widthwise",
        rear_hinge.axis == (1.0, 0.0, 0.0),
        f"Expected lid hinge axis along freezer width; got {rear_hinge.axis!r}.",
    )

    if cabinet_dims is not None:
        ctx.check(
            "cabinet_realistic_width",
            1.0 <= cabinet_dims[0] <= 1.15,
            f"Cabinet width {cabinet_dims[0]:.3f} m is not chest-freezer sized.",
        )
        ctx.check(
            "cabinet_realistic_depth",
            0.65 <= cabinet_dims[1] <= 0.76,
            f"Cabinet depth {cabinet_dims[1]:.3f} m is not chest-freezer sized.",
        )
        ctx.check(
            "cabinet_realistic_height",
            0.82 <= cabinet_dims[2] <= 0.88,
            f"Cabinet height {cabinet_dims[2]:.3f} m is not realistic.",
        )

    if lid_dims is not None and cabinet_dims is not None:
        ctx.check(
            "lid_overhangs_cabinet_width",
            lid_dims[0] > cabinet_dims[0],
            "Lid should slightly overhang the cabinet width.",
        )
        ctx.check(
            "lid_overhangs_cabinet_depth",
            lid_dims[1] > cabinet_dims[1],
            "Lid should slightly overhang the cabinet depth.",
        )

    left_hinge = require_elem_center("lid", "left_center_knuckle")
    right_hinge = require_elem_center("lid", "right_center_knuckle")
    handle_center = require_elem_center("lid", "front_handle")

    if left_hinge is not None and right_hinge is not None:
        ctx.check(
            "hinges_are_spaced_left_right",
            left_hinge[0] < -0.25 and right_hinge[0] > 0.25,
            f"Hinge centers should sit near opposite rear corners, got {left_hinge!r} and {right_hinge!r}.",
        )
        ctx.check(
            "hinges_share_rear_axis",
            abs(left_hinge[1] - right_hinge[1]) <= 0.002 and abs(left_hinge[2] - right_hinge[2]) <= 0.002,
            f"Both hinge knuckles should lie on one rear hinge axis, got {left_hinge!r} and {right_hinge!r}.",
        )

    if handle_center is not None:
        ctx.check(
            "handle_is_front_mounted",
            handle_center[1] > 0.30,
            f"Front handle should sit on the front edge of the lid, got center {handle_center!r}.",
        )

    with ctx.pose({rear_hinge: 0.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="rear_hinge_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="rear_hinge_lower_no_floating")
        ctx.expect_contact(lid, cabinet, name="lid_closed_contacts_cabinet")
        ctx.expect_overlap(lid, cabinet, axes="xy", min_overlap=0.68, name="lid_closed_covers_cabinet")
        ctx.expect_contact(
            lid,
            cabinet,
            elem_a="left_center_knuckle",
            elem_b="left_outer_knuckle_a",
            name="left_hinge_contact_closed_a",
        )
        ctx.expect_contact(
            lid,
            cabinet,
            elem_a="left_center_knuckle",
            elem_b="left_outer_knuckle_b",
            name="left_hinge_contact_closed_b",
        )
        ctx.expect_contact(
            lid,
            cabinet,
            elem_a="right_center_knuckle",
            elem_b="right_outer_knuckle_a",
            name="right_hinge_contact_closed_a",
        )
        ctx.expect_contact(
            lid,
            cabinet,
            elem_a="right_center_knuckle",
            elem_b="right_outer_knuckle_b",
            name="right_hinge_contact_closed_b",
        )

    limits = rear_hinge.motion_limits
    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({rear_hinge: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="rear_hinge_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="rear_hinge_upper_no_floating")
            ctx.expect_contact(
                lid,
                cabinet,
                elem_a="left_center_knuckle",
                elem_b="left_outer_knuckle_a",
                name="left_hinge_contact_open",
            )
            ctx.expect_contact(
                lid,
                cabinet,
                elem_a="right_center_knuckle",
                elem_b="right_outer_knuckle_a",
                name="right_hinge_contact_open",
            )

        with ctx.pose({rear_hinge: limits.lower}):
            closed_handle_center = center_from_aabb(ctx.part_element_world_aabb("lid", elem="front_handle"))
        with ctx.pose({rear_hinge: limits.upper}):
            open_handle_center = center_from_aabb(ctx.part_element_world_aabb("lid", elem="front_handle"))

        if closed_handle_center is not None and open_handle_center is not None:
            ctx.check(
                "lid_opens_upward",
                open_handle_center[2] > closed_handle_center[2] + 0.28,
                f"Handle should rise substantially when the lid opens, got closed {closed_handle_center!r} and open {open_handle_center!r}.",
            )
            ctx.check(
                "lid_swings_from_rear_edge",
                open_handle_center[1] < closed_handle_center[1] - 0.18,
                f"Front edge should swing rearward as the lid rotates up, got closed {closed_handle_center!r} and open {open_handle_center!r}.",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
