from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_telescoping_drawer_slide")

    zinc = model.material("zinc_plated_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.32, 0.34, 0.37, 1.0))
    black = model.material("black_cap", rgba=(0.14, 0.14, 0.15, 1.0))

    length = 0.30
    outer_width = 0.0142
    outer_height = 0.040
    wall_thickness = 0.0016
    floor_thickness = 0.0018
    lip_width = 0.0018
    lip_thickness = 0.0014

    rear_setback = 0.002
    plate_thickness = 0.004
    inner_length = length - rear_setback - plate_thickness
    inner_width = outer_width - (2.0 * wall_thickness) - 0.0016
    inner_height = 0.030
    end_plate_width = 0.017
    end_plate_height = 0.038
    travel = 0.12

    outer = model.part("outer_channel")
    outer.visual(
        Box((length, outer_width, floor_thickness)),
        origin=Origin(xyz=(length / 2.0, 0.0, floor_thickness / 2.0)),
        material=zinc,
        name="outer_floor",
    )

    wall_height = outer_height - floor_thickness
    wall_z = floor_thickness + (wall_height / 2.0)
    wall_y = (outer_width / 2.0) - (wall_thickness / 2.0)
    outer.visual(
        Box((length, wall_thickness, wall_height)),
        origin=Origin(xyz=(length / 2.0, wall_y, wall_z)),
        material=zinc,
        name="outer_wall_left",
    )
    outer.visual(
        Box((length, wall_thickness, wall_height)),
        origin=Origin(xyz=(length / 2.0, -wall_y, wall_z)),
        material=zinc,
        name="outer_wall_right",
    )

    inner_clear_half = (outer_width / 2.0) - wall_thickness
    lip_y = inner_clear_half - (lip_width / 2.0)
    lip_z = outer_height - (lip_thickness / 2.0)
    outer.visual(
        Box((length, lip_width, lip_thickness)),
        origin=Origin(xyz=(length / 2.0, lip_y, lip_z)),
        material=zinc,
        name="outer_lip_left",
    )
    outer.visual(
        Box((length, lip_width, lip_thickness)),
        origin=Origin(xyz=(length / 2.0, -lip_y, lip_z)),
        material=zinc,
        name="outer_lip_right",
    )

    # A short rear bridge makes the fixed section read as a real captured channel
    # rather than four unrelated strips.
    outer.visual(
        Box((0.018, inner_clear_half * 2.0, lip_thickness)),
        origin=Origin(
            xyz=(0.009, 0.0, lip_z),
        ),
        material=dark_steel,
        name="rear_capture_bridge",
    )

    inner = model.part("inner_slide")
    inner.visual(
        Box((inner_length, inner_width, inner_height)),
        origin=Origin(xyz=(inner_length / 2.0, 0.0, inner_height / 2.0)),
        material=dark_steel,
        name="inner_rail",
    )
    inner.visual(
        Box((plate_thickness, end_plate_width, end_plate_height)),
        origin=Origin(
            xyz=(inner_length + (plate_thickness / 2.0), 0.0, end_plate_height / 2.0)
        ),
        material=black,
        name="end_plate",
    )

    model.articulation(
        "outer_to_inner",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=inner,
        origin=Origin(xyz=(rear_setback, 0.0, floor_thickness)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.30,
            lower=0.0,
            upper=travel,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_channel")
    inner = object_model.get_part("inner_slide")
    slide = object_model.get_articulation("outer_to_inner")
    limits = slide.motion_limits
    upper = 0.12 if limits is None or limits.upper is None else limits.upper

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    with ctx.pose({slide: 0.0}):
        ctx.expect_contact(
            inner,
            outer,
            elem_a="inner_rail",
            elem_b="outer_floor",
            contact_tol=1e-6,
            name="inner rail bears on outer floor when closed",
        )
        ctx.expect_within(
            inner,
            outer,
            axes="yz",
            inner_elem="inner_rail",
            margin=0.0,
            name="inner rail stays captured within outer channel section",
        )
        outer_aabb = ctx.part_world_aabb(outer)
        plate_aabb = ctx.part_element_world_aabb(inner, elem="end_plate")
        flush_ok = (
            outer_aabb is not None
            and plate_aabb is not None
            and abs(plate_aabb[1][0] - outer_aabb[1][0]) <= 0.001
        )
        ctx.check(
            "end plate closes the front of the slide",
            flush_ok,
            details=(
                f"expected end plate front near outer front; "
                f"plate_max_x={None if plate_aabb is None else plate_aabb[1][0]}, "
                f"outer_max_x={None if outer_aabb is None else outer_aabb[1][0]}"
            ),
        )

    with ctx.pose({slide: upper}):
        ctx.expect_contact(
            inner,
            outer,
            elem_a="inner_rail",
            elem_b="outer_floor",
            contact_tol=1e-6,
            name="inner rail remains supported on outer floor when extended",
        )
        ctx.expect_within(
            inner,
            outer,
            axes="yz",
            inner_elem="inner_rail",
            margin=0.0,
            name="inner rail remains laterally captured when extended",
        )
        open_pos = ctx.part_world_position(inner)

    with ctx.pose({slide: 0.0}):
        closed_pos = ctx.part_world_position(inner)

    motion_ok = (
        open_pos is not None
        and closed_pos is not None
        and open_pos[0] > closed_pos[0] + 0.10
        and abs(open_pos[1] - closed_pos[1]) <= 1e-6
        and abs(open_pos[2] - closed_pos[2]) <= 1e-6
    )
    ctx.check(
        "prismatic slide advances along the rail axis",
        motion_ok,
        details=(
            f"expected x-only extension; closed={closed_pos}, open={open_pos}, "
            f"travel_target={upper}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
