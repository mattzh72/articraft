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
    mesh_from_geometry,
    wire_from_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedalboard_case")

    shell = model.material("shell", rgba=(0.16, 0.17, 0.19, 1.0))
    trim = model.material("trim", rgba=(0.08, 0.08, 0.09, 1.0))
    aluminum = model.material("aluminum", rgba=(0.70, 0.72, 0.75, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.10, 1.0))

    base_depth = 0.40
    base_width = 0.68
    base_height = 0.085
    base_rear_wall_height = 0.073
    shell_t = 0.012
    floor_t = 0.008

    platform_depth = 0.30
    platform_width = 0.60
    platform_t = 0.012
    platform_pitch = 0.15
    platform_center_z = 0.0462

    lid_outer_depth = 0.426
    lid_outer_width = 0.706
    lid_height = 0.065
    lid_wall_t = 0.010
    lid_overlap = 0.010
    hinge_axis_backset = 0.006

    hinge_radius = 0.006
    hinge_seg = 0.18
    hinge_gap = 0.01

    handle_pivot_half = 0.290
    handle_axis_x = hinge_axis_backset + lid_outer_depth + 0.006
    handle_axis_z = 0.042
    handle_bracket_x = handle_axis_x - 0.006
    handle_bracket_y = handle_pivot_half + 0.012

    base = model.part("base")
    base.visual(
        Box((base_depth, base_width, floor_t)),
        origin=Origin(xyz=(0.0, 0.0, floor_t / 2)),
        material=shell,
        name="base_floor",
    )
    base.visual(
        Box((shell_t, base_width, base_height)),
        origin=Origin(xyz=(base_depth / 2 - shell_t / 2, 0.0, base_height / 2)),
        material=shell,
        name="base_front_wall",
    )
    base.visual(
        Box((shell_t, base_width, base_rear_wall_height)),
        origin=Origin(
            xyz=(-base_depth / 2 + shell_t / 2, 0.0, base_rear_wall_height / 2)
        ),
        material=shell,
        name="base_rear_wall",
    )
    side_span = base_depth - 2 * shell_t
    base.visual(
        Box((side_span, shell_t, base_height)),
        origin=Origin(xyz=(0.0, base_width / 2 - shell_t / 2, base_height / 2)),
        material=shell,
        name="base_right_wall",
    )
    base.visual(
        Box((side_span, shell_t, base_height)),
        origin=Origin(xyz=(0.0, -base_width / 2 + shell_t / 2, base_height / 2)),
        material=shell,
        name="base_left_wall",
    )

    base.visual(
        Box((0.030, 0.60, 0.020)),
        origin=Origin(xyz=(0.135, 0.0, 0.010)),
        material=trim,
        name="platform_front_riser",
    )
    base.visual(
        Box((0.030, 0.60, 0.060)),
        origin=Origin(xyz=(-0.135, 0.0, 0.030)),
        material=trim,
        name="platform_rear_riser",
    )
    base.visual(
        Box((platform_depth, platform_width, platform_t)),
        origin=Origin(
            xyz=(0.0, 0.0, platform_center_z),
            rpy=(0.0, platform_pitch, 0.0),
        ),
        material=aluminum,
        name="platform_board",
    )

    base_hinge_y = base_width / 2 - hinge_seg / 2
    for idx, y_center in enumerate((-base_hinge_y, base_hinge_y), start=1):
        base.visual(
            Cylinder(radius=hinge_radius, length=hinge_seg),
            origin=Origin(
                xyz=(-base_depth / 2 - hinge_axis_backset, y_center, base_height),
                rpy=(pi / 2, 0.0, 0.0),
            ),
            material=trim,
            name=f"base_hinge_knuckle_{idx}",
        )
        base.visual(
            Box((0.012, hinge_seg, 0.024)),
            origin=Origin(
                xyz=(-base_depth / 2 - hinge_axis_backset / 2, y_center, 0.079),
            ),
            material=trim,
            name=f"base_hinge_strap_{idx}",
        )

    lid = model.part("lid")
    lid.visual(
        Box((lid_outer_depth, lid_outer_width, lid_wall_t)),
        origin=Origin(
            xyz=(
                hinge_axis_backset + lid_outer_depth / 2,
                0.0,
                lid_height - lid_wall_t / 2,
            )
        ),
        material=shell,
        name="lid_top",
    )
    lid_wall_len = lid_height + lid_overlap
    wall_center_z = (lid_height - lid_overlap) / 2
    lid.visual(
        Box((lid_wall_t, lid_outer_width, lid_wall_len)),
        origin=Origin(
            xyz=(hinge_axis_backset + lid_wall_t / 2, 0.0, wall_center_z),
        ),
        material=shell,
        name="lid_rear_wall",
    )
    lid.visual(
        Box((lid_wall_t, lid_outer_width, lid_wall_len)),
        origin=Origin(
            xyz=(hinge_axis_backset + lid_outer_depth - lid_wall_t / 2, 0.0, wall_center_z),
        ),
        material=shell,
        name="lid_front_wall",
    )
    lid_side_span = lid_outer_depth - 2 * lid_wall_t
    lid.visual(
        Box((lid_side_span, lid_wall_t, lid_wall_len)),
        origin=Origin(
            xyz=(
                hinge_axis_backset + lid_outer_depth / 2,
                lid_outer_width / 2 - lid_wall_t / 2,
                wall_center_z,
            ),
        ),
        material=shell,
        name="lid_right_wall",
    )
    lid.visual(
        Box((lid_side_span, lid_wall_t, lid_wall_len)),
        origin=Origin(
            xyz=(
                hinge_axis_backset + lid_outer_depth / 2,
                -lid_outer_width / 2 + lid_wall_t / 2,
                wall_center_z,
            ),
        ),
        material=shell,
        name="lid_left_wall",
    )

    lid.visual(
        Box((0.012, 0.024, 0.028)),
        origin=Origin(xyz=(handle_bracket_x, handle_bracket_y, handle_axis_z)),
        material=trim,
        name="handle_right_bracket",
    )
    lid.visual(
        Box((0.012, 0.024, 0.028)),
        origin=Origin(xyz=(handle_bracket_x, -handle_bracket_y, handle_axis_z)),
        material=trim,
        name="handle_left_bracket",
    )

    lid_center_seg = lid_outer_width - 2 * hinge_seg - 2 * hinge_gap
    lid.visual(
        Cylinder(radius=hinge_radius, length=lid_center_seg),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(pi / 2, 0.0, 0.0),
        ),
        material=trim,
        name="lid_hinge_knuckle",
    )
    lid.visual(
        Box((0.012, lid_center_seg, 0.010)),
        origin=Origin(xyz=(hinge_axis_backset / 2, 0.0, 0.0)),
        material=trim,
        name="lid_hinge_strap",
    )

    handle = model.part("handle")
    handle_path = wire_from_points(
        [
            (0.0, -handle_pivot_half, 0.0),
            (0.020, -handle_pivot_half, -0.020),
            (0.032, -0.230, -0.058),
            (0.032, 0.230, -0.058),
            (0.020, handle_pivot_half, -0.020),
            (0.0, handle_pivot_half, 0.0),
        ],
        radius=0.007,
        radial_segments=18,
        closed_path=False,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.040,
        corner_segments=10,
    )
    handle.visual(
        mesh_from_geometry(handle_path, "carry_handle_frame"),
        material=trim,
        name="handle_frame",
    )
    handle.visual(
        Cylinder(radius=0.012, length=0.18),
        origin=Origin(
            xyz=(0.032, 0.0, -0.058),
            rpy=(pi / 2, 0.0, 0.0),
        ),
        material=rubber,
        name="handle_grip",
    )

    lid_hinge = model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(-base_depth / 2 - hinge_axis_backset, 0.0, base_height)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=0.0,
            upper=1.55,
        ),
    )
    model.articulation(
        "lid_to_handle",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=handle,
        origin=Origin(xyz=(handle_axis_x, 0.0, handle_axis_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=3.0,
            lower=0.0,
            upper=1.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("handle")
    lid_hinge = object_model.get_articulation("base_to_lid")
    handle_hinge = object_model.get_articulation("lid_to_handle")

    ctx.check(
        "parts exist",
        all(part is not None for part in (base, lid, handle)),
        details=f"base={base}, lid={lid}, handle={handle}",
    )
    ctx.check(
        "hinges use the width axis",
        tuple(round(v, 3) for v in lid_hinge.axis) == (0.0, -1.0, 0.0)
        and tuple(round(v, 3) for v in handle_hinge.axis) == (0.0, -1.0, 0.0),
        details=f"lid_axis={lid_hinge.axis}, handle_axis={handle_hinge.axis}",
    )

    with ctx.pose({lid_hinge: 0.0, handle_hinge: 0.0}):
        ctx.expect_overlap(
            lid,
            base,
            axes="xy",
            min_overlap=0.34,
            name="lid covers the base footprint when closed",
        )
        ctx.expect_contact(
            handle,
            lid,
            name="handle parks on its side brackets",
        )

        base_aabb = ctx.part_world_aabb(base)
        platform_aabb = ctx.part_element_world_aabb(base, elem="platform_board")
        ctx.check(
            "platform sits inside the tray volume",
            base_aabb is not None
            and platform_aabb is not None
            and platform_aabb[0][0] > base_aabb[0][0] + 0.03
            and platform_aabb[1][0] < base_aabb[1][0] - 0.03
            and platform_aabb[0][1] > base_aabb[0][1] + 0.02
            and platform_aabb[1][1] < base_aabb[1][1] - 0.02
            and platform_aabb[0][2] > base_aabb[0][2] + 0.01,
            details=f"base_aabb={base_aabb}, platform_aabb={platform_aabb}",
        )

        closed_lid_aabb = ctx.part_world_aabb(lid)
        closed_handle_aabb = ctx.part_world_aabb(handle)

    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        open_lid_aabb = ctx.part_world_aabb(lid)

    ctx.check(
        "lid opens upward from the rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.12,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    with ctx.pose({handle_hinge: handle_hinge.motion_limits.upper}):
        raised_handle_aabb = ctx.part_world_aabb(handle)

    ctx.check(
        "handle folds up around the side pivots",
        closed_handle_aabb is not None
        and raised_handle_aabb is not None
        and raised_handle_aabb[1][2] > closed_handle_aabb[1][2] + 0.04,
        details=f"closed={closed_handle_aabb}, raised={raised_handle_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
