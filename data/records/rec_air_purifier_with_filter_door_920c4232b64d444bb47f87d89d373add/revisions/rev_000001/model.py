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


def _x_barrel(part, *, x: float, y: float, z: float, radius: float, length: float, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="window_sill_air_purifier")

    shell_white = model.material("shell_white", rgba=(0.93, 0.94, 0.95, 1.0))
    trim_grey = model.material("trim_grey", rgba=(0.56, 0.58, 0.61, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.20, 1.0))
    filter_dark = model.material("filter_dark", rgba=(0.24, 0.26, 0.28, 1.0))
    filter_mesh = model.material("filter_mesh", rgba=(0.14, 0.16, 0.17, 0.78))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    width = 0.620
    depth = 0.172
    base_thickness = 0.010
    wall = 0.008
    body_top = 0.210
    front_frame_depth = 0.012
    intake_bottom = 0.034
    intake_top = 0.186
    intake_height = intake_top - intake_bottom
    stile_width = 0.026
    hinge_axis_y = depth * 0.5 - 0.010
    lid_thickness = 0.014
    hinge_axis_z = body_top + lid_thickness

    housing = model.part("housing")
    housing.visual(
        Box((width, depth, base_thickness)),
        origin=Origin(xyz=(0.0, 0.0, base_thickness * 0.5)),
        material=shell_white,
        name="bottom_pan",
    )
    housing.visual(
        Box((wall, depth, body_top - base_thickness)),
        origin=Origin(
            xyz=(-width * 0.5 + wall * 0.5, 0.0, base_thickness + (body_top - base_thickness) * 0.5)
        ),
        material=shell_white,
        name="left_wall",
    )
    housing.visual(
        Box((wall, depth, body_top - base_thickness)),
        origin=Origin(
            xyz=(width * 0.5 - wall * 0.5, 0.0, base_thickness + (body_top - base_thickness) * 0.5)
        ),
        material=shell_white,
        name="right_wall",
    )
    housing.visual(
        Box((width - 2.0 * wall, wall, body_top - base_thickness)),
        origin=Origin(
            xyz=(0.0, depth * 0.5 - wall * 0.5, base_thickness + (body_top - base_thickness) * 0.5)
        ),
        material=shell_white,
        name="rear_wall",
    )
    housing.visual(
        Box((width - 2.0 * wall, front_frame_depth, intake_bottom - base_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -depth * 0.5 + front_frame_depth * 0.5,
                base_thickness + (intake_bottom - base_thickness) * 0.5,
            )
        ),
        material=shell_white,
        name="front_lower_rail",
    )
    housing.visual(
        Box((width - 2.0 * wall, front_frame_depth, body_top - intake_top)),
        origin=Origin(
            xyz=(0.0, -depth * 0.5 + front_frame_depth * 0.5, intake_top + (body_top - intake_top) * 0.5)
        ),
        material=shell_white,
        name="front_upper_rail",
    )
    housing.visual(
        Box((stile_width, front_frame_depth, intake_height)),
        origin=Origin(
            xyz=(
                -width * 0.5 + wall + stile_width * 0.5,
                -depth * 0.5 + front_frame_depth * 0.5,
                intake_bottom + intake_height * 0.5,
            )
        ),
        material=shell_white,
        name="front_left_stile",
    )
    housing.visual(
        Box((stile_width, front_frame_depth, intake_height)),
        origin=Origin(
            xyz=(
                width * 0.5 - wall - stile_width * 0.5,
                -depth * 0.5 + front_frame_depth * 0.5,
                intake_bottom + intake_height * 0.5,
            )
        ),
        material=shell_white,
        name="front_right_stile",
    )

    slat_width = width - 2.0 * (wall + stile_width)
    for index, z in enumerate((0.050, 0.074, 0.098, 0.122, 0.146, 0.170)):
        housing.visual(
            Box((slat_width, front_frame_depth, 0.006)),
            origin=Origin(xyz=(0.0, -depth * 0.5 + front_frame_depth * 0.5, z)),
            material=trim_grey,
            name=f"grille_slat_{index}",
        )

    housing.visual(
        Box((width - 2.0 * wall, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, depth * 0.5 - 0.010, body_top - 0.009)),
        material=shell_white,
        name="rear_hinge_bridge",
    )
    for side, x in (("left", -0.200), ("right", 0.200)):
        housing.visual(
            Box((0.056, 0.008, 0.012)),
            origin=Origin(xyz=(x, hinge_axis_y + 0.008, hinge_axis_z - 0.008)),
            material=shell_white,
            name=f"{side}_hinge_pedestal",
        )
        housing.visual(
            Box((0.016, 0.018, 0.012)),
            origin=Origin(xyz=(x, hinge_axis_y + 0.006, body_top - 0.008)),
            material=trim_grey,
            name=f"{side}_hinge_upright",
        )

    _x_barrel(
        housing,
        x=-0.219,
        y=hinge_axis_y,
        z=hinge_axis_z,
        radius=0.008,
        length=0.012,
        material=trim_grey,
        name="left_hinge_barrel_outer",
    )
    _x_barrel(
        housing,
        x=-0.181,
        y=hinge_axis_y,
        z=hinge_axis_z,
        radius=0.008,
        length=0.012,
        material=trim_grey,
        name="left_hinge_barrel_inner",
    )
    _x_barrel(
        housing,
        x=0.181,
        y=hinge_axis_y,
        z=hinge_axis_z,
        radius=0.008,
        length=0.012,
        material=trim_grey,
        name="right_hinge_barrel_inner",
    )
    _x_barrel(
        housing,
        x=0.219,
        y=hinge_axis_y,
        z=hinge_axis_z,
        radius=0.008,
        length=0.012,
        material=trim_grey,
        name="right_hinge_barrel_outer",
    )

    guide_height = 0.164
    guide_center_z = 0.112
    guide_depth = 0.022
    guide_center_y = -0.060
    housing.visual(
        Box((0.010, guide_depth, guide_height)),
        origin=Origin(xyz=(-0.297, guide_center_y, guide_center_z)),
        material=charcoal,
        name="left_guide",
    )
    housing.visual(
        Box((0.010, guide_depth, guide_height)),
        origin=Origin(xyz=(0.297, guide_center_y, guide_center_z)),
        material=charcoal,
        name="right_guide",
    )

    for side, x in (("left", -0.235), ("right", 0.235)):
        housing.visual(
            Box((0.110, 0.026, 0.008)),
            origin=Origin(xyz=(x, 0.0, 0.004)),
            material=rubber,
            name=f"{side}_foot",
        )

    housing.inertial = Inertial.from_geometry(
        Box((width, depth, hinge_axis_z + 0.015)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, (hinge_axis_z + 0.015) * 0.5)),
    )

    lid = model.part("top_lid")
    lid.visual(
        Box((0.616, 0.150, lid_thickness)),
        origin=Origin(xyz=(0.0, -0.091, -lid_thickness * 0.5)),
        material=shell_white,
        name="lid_panel",
    )
    lid.visual(
        Box((0.606, 0.010, 0.022)),
        origin=Origin(xyz=(0.0, -0.168, -0.018)),
        material=shell_white,
        name="front_lip",
    )
    lid.visual(
        Box((0.540, 0.028, 0.006)),
        origin=Origin(xyz=(0.0, -0.056, -0.011)),
        material=trim_grey,
        name="underside_stiffener",
    )
    lid.visual(
        Box((0.024, 0.030, 0.010)),
        origin=Origin(xyz=(-0.200, -0.016, -0.006)),
        material=trim_grey,
        name="left_hinge_leaf",
    )
    lid.visual(
        Box((0.024, 0.030, 0.010)),
        origin=Origin(xyz=(0.200, -0.016, -0.006)),
        material=trim_grey,
        name="right_hinge_leaf",
    )
    _x_barrel(
        lid,
        x=-0.200,
        y=0.0,
        z=0.0,
        radius=0.007,
        length=0.018,
        material=charcoal,
        name="left_lid_barrel",
    )
    _x_barrel(
        lid,
        x=0.200,
        y=0.0,
        z=0.0,
        radius=0.007,
        length=0.018,
        material=charcoal,
        name="right_lid_barrel",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.616, 0.160, 0.032)),
        mass=1.1,
        origin=Origin(xyz=(0.0, -0.078, -0.011)),
    )

    pre_filter = model.part("pre_filter_panel")
    pre_filter.visual(
        Box((0.016, 0.006, 0.152)),
        origin=Origin(xyz=(-0.262, 0.0, 0.076)),
        material=filter_dark,
        name="left_frame",
    )
    pre_filter.visual(
        Box((0.016, 0.006, 0.152)),
        origin=Origin(xyz=(0.262, 0.0, 0.076)),
        material=filter_dark,
        name="right_frame",
    )
    pre_filter.visual(
        Box((0.540, 0.006, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=filter_dark,
        name="bottom_frame",
    )
    pre_filter.visual(
        Box((0.540, 0.006, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.144)),
        material=filter_dark,
        name="top_frame",
    )
    pre_filter.visual(
        Box((0.508, 0.002, 0.120)),
        origin=Origin(xyz=(0.0, 0.0, 0.076)),
        material=filter_mesh,
        name="filter_media",
    )
    pre_filter.visual(
        Box((0.022, 0.012, 0.120)),
        origin=Origin(xyz=(-0.281, 0.009, 0.076)),
        material=charcoal,
        name="left_shoe",
    )
    pre_filter.visual(
        Box((0.022, 0.012, 0.120)),
        origin=Origin(xyz=(0.281, 0.009, 0.076)),
        material=charcoal,
        name="right_shoe",
    )
    pre_filter.visual(
        Box((0.090, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, 0.006, 0.157)),
        material=trim_grey,
        name="pull_handle",
    )
    pre_filter.inertial = Inertial.from_geometry(
        Box((0.562, 0.020, 0.175)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.007, 0.087)),
    )

    model.articulation(
        "housing_to_top_lid",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=9.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(82.0),
        ),
    )
    model.articulation(
        "housing_to_pre_filter_panel",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=pre_filter,
        origin=Origin(xyz=(0.0, -0.067, 0.034)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.12,
            lower=0.0,
            upper=0.095,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    lid = object_model.get_part("top_lid")
    pre_filter = object_model.get_part("pre_filter_panel")
    lid_hinge = object_model.get_articulation("housing_to_top_lid")
    filter_slide = object_model.get_articulation("housing_to_pre_filter_panel")

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

    ctx.expect_overlap(
        lid,
        housing,
        axes="xy",
        min_overlap=0.150,
        name="closed lid covers the purifier housing",
    )
    ctx.expect_within(
        pre_filter,
        housing,
        axes="xy",
        margin=0.0,
        name="closed pre-filter sits within the housing footprint",
    )
    ctx.expect_gap(
        pre_filter,
        housing,
        axis="x",
        positive_elem="left_shoe",
        negative_elem="left_guide",
        max_gap=0.004,
        max_penetration=1e-6,
        name="left filter shoe tracks the left guide rail",
    )
    ctx.expect_gap(
        housing,
        pre_filter,
        axis="x",
        positive_elem="right_guide",
        negative_elem="right_shoe",
        max_gap=0.004,
        max_penetration=1e-6,
        name="right filter shoe tracks the right guide rail",
    )

    closed_lid_front = ctx.part_element_world_aabb(lid, elem="front_lip")
    with ctx.pose({lid_hinge: math.radians(82.0)}):
        open_lid_front = ctx.part_element_world_aabb(lid, elem="front_lip")
    ctx.check(
        "lid front edge rises when opened",
        closed_lid_front is not None
        and open_lid_front is not None
        and open_lid_front[0][2] > closed_lid_front[0][2] + 0.10,
        details=f"closed={closed_lid_front}, open={open_lid_front}",
    )

    closed_filter_handle = ctx.part_element_world_aabb(pre_filter, elem="pull_handle")
    with ctx.pose({filter_slide: 0.095}):
        extended_filter_handle = ctx.part_element_world_aabb(pre_filter, elem="pull_handle")
        ctx.expect_within(
            pre_filter,
            housing,
            axes="xy",
            margin=0.0,
            name="extended pre-filter stays aligned in the front opening",
        )
        ctx.expect_overlap(
            pre_filter,
            housing,
            axes="z",
            min_overlap=0.060,
            name="extended pre-filter retains insertion on the guide rails",
        )
        ctx.expect_gap(
            pre_filter,
            housing,
            axis="x",
            positive_elem="left_shoe",
            negative_elem="left_guide",
            max_gap=0.004,
            max_penetration=1e-6,
            name="left guide still captures the filter when extended",
        )
        ctx.expect_gap(
            housing,
            pre_filter,
            axis="x",
            positive_elem="right_guide",
            negative_elem="right_shoe",
            max_gap=0.004,
            max_penetration=1e-6,
            name="right guide still captures the filter when extended",
        )
    ctx.check(
        "pre-filter lifts upward for removal",
        closed_filter_handle is not None
        and extended_filter_handle is not None
        and extended_filter_handle[0][2] > closed_filter_handle[0][2] + 0.08,
        details=f"closed={closed_filter_handle}, extended={extended_filter_handle}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
