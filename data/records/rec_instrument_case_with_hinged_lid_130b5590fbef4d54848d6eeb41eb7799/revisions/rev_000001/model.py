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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _shell_profiles(length: float, width: float, wall: float, corner_radius: float):
    outer = rounded_rect_profile(length, width, corner_radius, corner_segments=10)
    inner = rounded_rect_profile(
        length - 2.0 * wall,
        width - 2.0 * wall,
        max(corner_radius - wall, wall * 0.35),
        corner_segments=10,
    )
    return outer, inner


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="trumpet_case")

    exterior_black = model.material("exterior_black", rgba=(0.14, 0.13, 0.12, 1.0))
    trim_aluminum = model.material("trim_aluminum", rgba=(0.72, 0.73, 0.75, 1.0))
    hardware_dark = model.material("hardware_dark", rgba=(0.22, 0.22, 0.24, 1.0))
    plush_red = model.material("plush_red", rgba=(0.50, 0.10, 0.12, 1.0))
    handle_black = model.material("handle_black", rgba=(0.08, 0.08, 0.09, 1.0))

    case_length = 0.58
    case_width = 0.24
    base_height = 0.055
    lid_height = 0.052
    wall = 0.008
    corner_radius = 0.026
    seam_overlap = 0.0005

    base_outer, base_inner = _shell_profiles(case_length, case_width, wall, corner_radius)
    base_ring = ExtrudeWithHolesGeometry(
        base_outer,
        [base_inner],
        height=base_height - wall,
        center=True,
    ).translate(0.0, 0.0, wall + (base_height - wall) * 0.5)
    base_floor = ExtrudeGeometry(base_outer, wall + seam_overlap, center=True).translate(
        0.0,
        0.0,
        (wall + seam_overlap) * 0.5,
    )

    lid_back_margin = 0.008
    lid_z0 = 0.001
    lid_outer = rounded_rect_profile(case_length - 0.002, case_width, corner_radius, corner_segments=10)
    lid_inner = rounded_rect_profile(
        case_length - 0.002 - 2.0 * wall,
        case_width - 2.0 * wall,
        max(corner_radius - wall, wall * 0.35),
        corner_segments=10,
    )
    lid_length = case_length - 0.002
    lid_ring = ExtrudeWithHolesGeometry(
        lid_outer,
        [lid_inner],
        height=lid_height - wall,
        center=True,
    ).translate(
        lid_back_margin + lid_length * 0.5,
        0.0,
        lid_z0 + (lid_height - wall) * 0.5,
    )
    lid_top = ExtrudeGeometry(lid_outer, wall + seam_overlap, center=True).translate(
        lid_back_margin + lid_length * 0.5,
        0.0,
        lid_z0 + lid_height - (wall + seam_overlap) * 0.5,
    )

    base = model.part("base_shell")
    base.visual(_mesh("trumpet_case_base_ring", base_ring), material=exterior_black, name="base_shell_ring")
    base.visual(_mesh("trumpet_case_base_floor", base_floor), material=exterior_black, name="base_floor")
    base.visual(
        Box((case_length - 0.062, case_width - 0.060, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, wall + 0.006)),
        material=plush_red,
        name="base_liner_pad",
    )
    base.visual(
        Box((case_length - 0.100, 0.060, 0.018)),
        origin=Origin(xyz=(0.010, 0.0, wall + 0.015)),
        material=plush_red,
        name="instrument_support_pad",
    )
    base.visual(
        Box((0.010, 0.060, 0.012)),
        origin=Origin(xyz=(-case_length * 0.24, 0.0, wall + 0.016)),
        material=plush_red,
        name="mouthpiece_block",
    )
    base.visual(
        Box((case_length - 0.010, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, case_width * 0.5 - wall * 0.5, base_height - 0.003)),
        material=trim_aluminum,
        name="base_right_trim",
    )
    base.visual(
        Box((case_length - 0.010, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, -case_width * 0.5 + wall * 0.5, base_height - 0.003)),
        material=trim_aluminum,
        name="base_left_trim",
    )
    base.visual(
        Box((0.010, case_width - 0.030, 0.006)),
        origin=Origin(xyz=(case_length * 0.5 - wall * 0.5, 0.0, base_height - 0.003)),
        material=trim_aluminum,
        name="base_front_trim",
    )
    base.visual(
        Box((0.010, case_width - 0.030, 0.006)),
        origin=Origin(xyz=(-case_length * 0.5 + wall * 0.5, 0.0, base_height - 0.003)),
        material=trim_aluminum,
        name="base_rear_trim",
    )

    hinge_radius = 0.005
    hinge_barrel_x = -case_length * 0.5 - 0.004
    hinge_barrel_z = base_height - 0.001
    for name, y in (
        ("rear_hinge_left_barrel", -0.068),
        ("rear_hinge_right_barrel", 0.068),
    ):
        base.visual(
            Cylinder(radius=hinge_radius, length=0.055),
            origin=Origin(xyz=(hinge_barrel_x, y, hinge_barrel_z), rpy=(pi * 0.5, 0.0, 0.0)),
            material=hardware_dark,
            name=name,
        )
    base.visual(
        Box((0.007, 0.190, 0.010)),
        origin=Origin(xyz=(-case_length * 0.5 + 0.0055, 0.0, base_height - 0.004)),
        material=hardware_dark,
        name="rear_hinge_leaf",
    )

    handle_pivot_x = case_length * 0.5 + 0.011
    handle_pivot_z = 0.036
    bracket_plate_x = case_length * 0.5 + 0.003
    bracket_ear_x = case_length * 0.5 + 0.009
    for side, y0 in (("left", -0.055), ("right", 0.055)):
        base.visual(
            Box((0.008, 0.022, 0.024)),
            origin=Origin(xyz=(bracket_plate_x, y0, handle_pivot_z)),
            material=hardware_dark,
            name=f"{side}_handle_bracket_plate",
        )
        for tag, y_off in (("inboard", -0.006), ("outboard", 0.006)):
            base.visual(
                Box((0.012, 0.004, 0.018)),
                origin=Origin(xyz=(bracket_ear_x, y0 + y_off, handle_pivot_z)),
                material=hardware_dark,
                name=f"{side}_handle_bracket_{tag}",
            )

    latch_x = case_length * 0.5 + 0.004
    latch_z = base_height - 0.013
    for side, y in (("left", -0.085), ("right", 0.085)):
        base.visual(
            Box((0.012, 0.022, 0.020)),
            origin=Origin(xyz=(latch_x, y, latch_z)),
            material=trim_aluminum,
            name=f"{side}_base_latch_body",
        )
        base.visual(
            Box((0.006, 0.014, 0.010)),
            origin=Origin(xyz=(latch_x + 0.006, y, latch_z + 0.010)),
            material=hardware_dark,
            name=f"{side}_base_latch_catch",
        )

    base.inertial = Inertial.from_geometry(
        Box((case_length, case_width, base_height)),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.0, base_height * 0.5)),
    )

    lid = model.part("lid_shell")
    lid.visual(_mesh("trumpet_case_lid_ring", lid_ring), material=exterior_black, name="lid_shell_ring")
    lid.visual(_mesh("trumpet_case_lid_top", lid_top), material=exterior_black, name="lid_top_panel")
    lid.visual(
        Box((lid_length - 0.060, case_width - 0.060, 0.010)),
        origin=Origin(
            xyz=(
                lid_back_margin + lid_length * 0.5,
                0.0,
                lid_z0 + lid_height - wall - 0.005,
            )
        ),
        material=plush_red,
        name="lid_inner_pad",
    )
    lid.visual(
        Box((0.010, case_width - 0.030, 0.006)),
        origin=Origin(
            xyz=(
                lid_back_margin + lid_length - wall * 0.5,
                0.0,
                lid_z0 + 0.003,
            )
        ),
        material=trim_aluminum,
        name="lid_front_trim",
    )
    lid.visual(
        Box((lid_length - 0.020, 0.010, 0.006)),
        origin=Origin(
            xyz=(
                lid_back_margin + lid_length * 0.5,
                case_width * 0.5 - wall * 0.5,
                lid_z0 + 0.003,
            )
        ),
        material=trim_aluminum,
        name="lid_right_trim",
    )
    lid.visual(
        Box((lid_length - 0.020, 0.010, 0.006)),
        origin=Origin(
            xyz=(
                lid_back_margin + lid_length * 0.5,
                -case_width * 0.5 + wall * 0.5,
                lid_z0 + 0.003,
            )
        ),
        material=trim_aluminum,
        name="lid_left_trim",
    )
    lid.visual(
        Cylinder(radius=hinge_radius, length=0.074),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(pi * 0.5, 0.0, 0.0),
        ),
        material=hardware_dark,
        name="lid_center_hinge_barrel",
    )
    lid.visual(
        Box((0.018, 0.064, 0.010)),
        origin=Origin(
            xyz=(0.009, 0.0, lid_z0 + 0.004),
        ),
        material=hardware_dark,
        name="lid_hinge_leaf",
    )
    for side, y in (("left", -0.085), ("right", 0.085)):
        lid.visual(
            Box((0.012, 0.018, 0.014)),
            origin=Origin(
                xyz=(
                    lid_back_margin + lid_length - 0.006,
                    y,
                    lid_z0 + 0.009,
                )
            ),
            material=trim_aluminum,
            name=f"{side}_lid_latch_keeper",
        )
    lid.inertial = Inertial.from_geometry(
        Box((case_length, case_width, lid_height)),
        mass=1.2,
        origin=Origin(xyz=(lid_back_margin + case_length * 0.5, 0.0, lid_z0 + lid_height * 0.5)),
    )

    handle = model.part("carry_handle")
    handle_frame = tube_from_spline_points(
        [
            (0.0, -0.055, 0.0),
            (0.014, -0.055, -0.010),
            (0.028, -0.040, -0.046),
            (0.032, 0.0, -0.068),
            (0.028, 0.040, -0.046),
            (0.014, 0.055, -0.010),
            (0.0, 0.055, 0.0),
        ],
        radius=0.006,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
    )
    handle.visual(_mesh("trumpet_case_handle_frame", handle_frame), material=handle_black, name="handle_frame")
    handle.visual(
        Cylinder(radius=0.0075, length=0.070),
        origin=Origin(xyz=(0.031, 0.0, -0.066), rpy=(pi * 0.5, 0.0, 0.0)),
        material=handle_black,
        name="handle_grip",
    )
    for side, y in (("left", -0.055), ("right", 0.055)):
        handle.visual(
            Cylinder(radius=0.0045, length=0.008),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
            material=hardware_dark,
            name=f"{side}_handle_pivot",
        )
    handle.inertial = Inertial.from_geometry(
        Box((0.045, 0.130, 0.080)),
        mass=0.22,
        origin=Origin(xyz=(0.018, 0.0, -0.034)),
    )

    model.articulation(
        "rear_lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(hinge_barrel_x, 0.0, hinge_barrel_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=0.0,
            upper=1.65,
        ),
    )
    model.articulation(
        "front_handle_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=handle,
        origin=Origin(xyz=(handle_pivot_x, 0.0, handle_pivot_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=3.5,
            lower=0.0,
            upper=1.15,
        ),
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

    base = object_model.get_part("base_shell")
    lid = object_model.get_part("lid_shell")
    handle = object_model.get_part("carry_handle")
    lid_hinge = object_model.get_articulation("rear_lid_hinge")
    handle_joint = object_model.get_articulation("front_handle_pivot")

    base_front_trim = base.get_visual("base_front_trim")
    lid_front_trim = lid.get_visual("lid_front_trim")
    left_base_latch = base.get_visual("left_base_latch_body")
    right_base_latch = base.get_visual("right_base_latch_body")
    left_lid_keeper = lid.get_visual("left_lid_latch_keeper")
    right_lid_keeper = lid.get_visual("right_lid_latch_keeper")
    handle_grip = handle.get_visual("handle_grip")

    ctx.check(
        "lid hinge uses rear horizontal axis",
        tuple(lid_hinge.axis) == (0.0, -1.0, 0.0),
        details=f"axis={lid_hinge.axis}",
    )
    ctx.check(
        "handle uses front horizontal pivot axis",
        tuple(handle_joint.axis) == (0.0, -1.0, 0.0),
        details=f"axis={handle_joint.axis}",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_overlap(
            lid,
            base,
            axes="xy",
            min_overlap=0.20,
            name="closed lid covers the base footprint",
        )
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            positive_elem=lid_front_trim,
            negative_elem=base_front_trim,
            max_gap=0.0015,
            max_penetration=1e-6,
            name="front seam closes cleanly",
        )
        ctx.expect_overlap(
            lid,
            base,
            axes="xy",
            elem_a=left_lid_keeper,
            elem_b=left_base_latch,
            min_overlap=0.003,
            name="left latch aligns across the front seam",
        )
        ctx.expect_overlap(
            lid,
            base,
            axes="xy",
            elem_a=right_lid_keeper,
            elem_b=right_base_latch,
            min_overlap=0.003,
            name="right latch aligns across the front seam",
        )

    closed_lid_front = ctx.part_element_world_aabb(lid, elem="lid_front_trim")
    with ctx.pose({lid_hinge: 1.20}):
        open_lid_front = ctx.part_element_world_aabb(lid, elem="lid_front_trim")
    ctx.check(
        "lid swings upward when opened",
        closed_lid_front is not None
        and open_lid_front is not None
        and open_lid_front[0][2] > closed_lid_front[0][2] + 0.14,
        details=f"closed={closed_lid_front}, open={open_lid_front}",
    )

    closed_handle_grip = ctx.part_element_world_aabb(handle, elem="handle_grip")
    with ctx.pose({handle_joint: 0.95}):
        raised_handle_grip = ctx.part_element_world_aabb(handle, elem="handle_grip")
    ctx.check(
        "handle lifts away from the front face",
        closed_handle_grip is not None
        and raised_handle_grip is not None
        and raised_handle_grip[0][2] > closed_handle_grip[0][2] + 0.040,
        details=f"closed={closed_handle_grip}, raised={raised_handle_grip}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
