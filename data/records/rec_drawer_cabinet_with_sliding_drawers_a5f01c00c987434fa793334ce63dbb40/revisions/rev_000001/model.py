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


def _build_drawer(
    model: ArticulatedObject,
    *,
    name: str,
    front_material,
    box_material,
    runner_material,
    handle_material,
    front_width: float,
    front_height: float,
    front_thickness: float,
    box_width: float,
    box_depth: float,
    box_height: float,
    box_floor_z: float,
    runner_thickness: float,
    runner_height: float,
    runner_length: float,
    runner_y0: float,
) -> None:
    drawer = model.part(name)

    side_stile_width = 0.090
    pocket_width = front_width - 2.0 * side_stile_width
    pocket_height = 0.070
    pocket_center_z = front_height * 0.58
    pocket_bottom_z = pocket_center_z - pocket_height / 2.0
    top_rail_height = front_height - (pocket_bottom_z + pocket_height)
    bottom_rail_height = pocket_bottom_z
    pocket_back_thickness = 0.012

    drawer.visual(
        Box((side_stile_width, front_thickness, front_height)),
        origin=Origin(
            xyz=(
                -front_width / 2.0 + side_stile_width / 2.0,
                front_thickness / 2.0,
                front_height / 2.0,
            )
        ),
        material=front_material,
        name="front_left_stile",
    )
    drawer.visual(
        Box((side_stile_width, front_thickness, front_height)),
        origin=Origin(
            xyz=(
                front_width / 2.0 - side_stile_width / 2.0,
                front_thickness / 2.0,
                front_height / 2.0,
            )
        ),
        material=front_material,
        name="front_right_stile",
    )
    drawer.visual(
        Box((pocket_width, front_thickness, top_rail_height)),
        origin=Origin(
            xyz=(
                0.0,
                front_thickness / 2.0,
                front_height - top_rail_height / 2.0,
            )
        ),
        material=front_material,
        name="front_top_rail",
    )
    drawer.visual(
        Box((pocket_width, front_thickness, bottom_rail_height)),
        origin=Origin(
            xyz=(0.0, front_thickness / 2.0, bottom_rail_height / 2.0)
        ),
        material=front_material,
        name="front_bottom_rail",
    )
    drawer.visual(
        Box((pocket_width, pocket_back_thickness, pocket_height)),
        origin=Origin(
            xyz=(
                0.0,
                front_thickness - pocket_back_thickness / 2.0,
                pocket_center_z,
            )
        ),
        material=front_material,
        name="handle_recess_back",
    )

    handle_radius = 0.007
    handle_length = pocket_width - 0.140
    handle_mount_width = 0.020
    handle_mount_height = 0.024
    handle_mount_depth = 0.012
    handle_mount_offset = handle_length / 2.0 - 0.050

    drawer.visual(
        Box((handle_mount_width, handle_mount_depth, handle_mount_height)),
        origin=Origin(xyz=(-handle_mount_offset, 0.010, pocket_center_z)),
        material=handle_material,
        name="handle_mount_left",
    )
    drawer.visual(
        Box((handle_mount_width, handle_mount_depth, handle_mount_height)),
        origin=Origin(xyz=(handle_mount_offset, 0.010, pocket_center_z)),
        material=handle_material,
        name="handle_mount_right",
    )
    drawer.visual(
        Cylinder(radius=handle_radius, length=handle_length),
        origin=Origin(
            xyz=(0.0, 0.010, pocket_center_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=handle_material,
        name="handle_bar",
    )

    drawer_side_thickness = 0.012
    drawer_bottom_thickness = 0.012
    inner_front_thickness = 0.012
    box_internal_depth = box_depth - front_thickness

    drawer.visual(
        Box((box_width, box_internal_depth, drawer_bottom_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                front_thickness + box_internal_depth / 2.0,
                box_floor_z + drawer_bottom_thickness / 2.0,
            )
        ),
        material=box_material,
        name="drawer_box_bottom",
    )
    drawer.visual(
        Box((drawer_side_thickness, box_internal_depth, box_height)),
        origin=Origin(
            xyz=(
                -box_width / 2.0 + drawer_side_thickness / 2.0,
                front_thickness + box_internal_depth / 2.0,
                box_floor_z + box_height / 2.0,
            )
        ),
        material=box_material,
        name="drawer_box_left_side",
    )
    drawer.visual(
        Box((drawer_side_thickness, box_internal_depth, box_height)),
        origin=Origin(
            xyz=(
                box_width / 2.0 - drawer_side_thickness / 2.0,
                front_thickness + box_internal_depth / 2.0,
                box_floor_z + box_height / 2.0,
            )
        ),
        material=box_material,
        name="drawer_box_right_side",
    )
    drawer.visual(
        Box((box_width - 2.0 * drawer_side_thickness, drawer_side_thickness, box_height)),
        origin=Origin(
            xyz=(
                0.0,
                front_thickness + inner_front_thickness / 2.0,
                box_floor_z + box_height / 2.0,
            )
        ),
        material=box_material,
        name="drawer_box_front_wall",
    )
    drawer.visual(
        Box((box_width - 2.0 * drawer_side_thickness, drawer_side_thickness, box_height)),
        origin=Origin(
            xyz=(
                0.0,
                front_thickness + box_internal_depth - drawer_side_thickness / 2.0,
                box_floor_z + box_height / 2.0,
            )
        ),
        material=box_material,
        name="drawer_box_back_wall",
    )

    runner_center_z = box_floor_z + 0.044
    drawer.visual(
        Box((runner_thickness, runner_length, runner_height)),
        origin=Origin(
            xyz=(
                -box_width / 2.0 - runner_thickness / 2.0,
                runner_y0 + runner_length / 2.0,
                runner_center_z,
            )
        ),
        material=runner_material,
        name="left_runner",
    )
    drawer.visual(
        Box((runner_thickness, runner_length, runner_height)),
        origin=Origin(
            xyz=(
                box_width / 2.0 + runner_thickness / 2.0,
                runner_y0 + runner_length / 2.0,
                runner_center_z,
            )
        ),
        material=runner_material,
        name="right_runner",
    )

    drawer.inertial = Inertial.from_geometry(
        Box((front_width, box_depth, front_height)),
        mass=8.5,
        origin=Origin(xyz=(0.0, box_depth / 2.0, front_height / 2.0)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="kitchen_base_cabinet")

    body_wood = model.material("body_wood", rgba=(0.63, 0.47, 0.29, 1.0))
    drawer_front_wood = model.material("drawer_front_wood", rgba=(0.56, 0.39, 0.23, 1.0))
    drawer_box_wood = model.material("drawer_box_wood", rgba=(0.78, 0.68, 0.49, 1.0))
    runner_wood = model.material("runner_wood", rgba=(0.50, 0.35, 0.20, 1.0))
    handle_metal = model.material("handle_metal", rgba=(0.30, 0.30, 0.32, 1.0))

    cabinet_width = 0.92
    cabinet_depth = 0.58
    cabinet_height = 0.88
    panel_thickness = 0.018
    back_thickness = 0.008
    toe_kick_height = 0.11
    toe_kick_recess = 0.055
    inner_width = cabinet_width - 2.0 * panel_thickness

    drawer_front_thickness = 0.020
    drawer_bottom_reveal = 0.012
    drawer_mid_gap = 0.010
    drawer_top_reveal = 0.020
    drawer_front_height = (
        cabinet_height
        - toe_kick_height
        - drawer_bottom_reveal
        - drawer_mid_gap
        - drawer_top_reveal
    ) / 2.0
    lower_drawer_z = toe_kick_height + drawer_bottom_reveal
    upper_drawer_z = lower_drawer_z + drawer_front_height + drawer_mid_gap

    drawer_front_width = inner_width - 0.008
    drawer_box_width = 0.840
    drawer_box_depth = 0.500
    drawer_box_height = 0.200
    drawer_box_floor_z = 0.048
    runner_thickness = 0.010
    runner_height = 0.020
    runner_length = 0.482
    runner_y0 = 0.055
    drawer_travel = 0.320

    body = model.part("cabinet_body")

    body.visual(
        Box((panel_thickness, cabinet_depth, cabinet_height - toe_kick_height)),
        origin=Origin(
            xyz=(
                -cabinet_width / 2.0 + panel_thickness / 2.0,
                cabinet_depth / 2.0,
                toe_kick_height + (cabinet_height - toe_kick_height) / 2.0,
            )
        ),
        material=body_wood,
        name="left_side_upper",
    )
    body.visual(
        Box((panel_thickness, cabinet_depth - toe_kick_recess, toe_kick_height)),
        origin=Origin(
            xyz=(
                -cabinet_width / 2.0 + panel_thickness / 2.0,
                toe_kick_recess + (cabinet_depth - toe_kick_recess) / 2.0,
                toe_kick_height / 2.0,
            )
        ),
        material=body_wood,
        name="left_side_leg",
    )
    body.visual(
        Box((panel_thickness, cabinet_depth, cabinet_height - toe_kick_height)),
        origin=Origin(
            xyz=(
                cabinet_width / 2.0 - panel_thickness / 2.0,
                cabinet_depth / 2.0,
                toe_kick_height + (cabinet_height - toe_kick_height) / 2.0,
            )
        ),
        material=body_wood,
        name="right_side_upper",
    )
    body.visual(
        Box((panel_thickness, cabinet_depth - toe_kick_recess, toe_kick_height)),
        origin=Origin(
            xyz=(
                cabinet_width / 2.0 - panel_thickness / 2.0,
                toe_kick_recess + (cabinet_depth - toe_kick_recess) / 2.0,
                toe_kick_height / 2.0,
            )
        ),
        material=body_wood,
        name="right_side_leg",
    )
    body.visual(
        Box((inner_width, cabinet_depth - toe_kick_recess - back_thickness, panel_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                toe_kick_recess
                + (cabinet_depth - toe_kick_recess - back_thickness) / 2.0,
                toe_kick_height + panel_thickness / 2.0,
            )
        ),
        material=body_wood,
        name="bottom_deck",
    )
    body.visual(
        Box((inner_width, cabinet_depth - back_thickness, panel_thickness)),
        origin=Origin(
            xyz=(0.0, (cabinet_depth - back_thickness) / 2.0, cabinet_height - panel_thickness / 2.0)
        ),
        material=body_wood,
        name="top_panel",
    )
    body.visual(
        Box((inner_width, back_thickness, cabinet_height)),
        origin=Origin(
            xyz=(0.0, cabinet_depth - back_thickness / 2.0, cabinet_height / 2.0)
        ),
        material=body_wood,
        name="back_panel",
    )
    body.visual(
        Box((inner_width, cabinet_depth - 0.080 - back_thickness, panel_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                0.080 + (cabinet_depth - 0.080 - back_thickness) / 2.0,
                lower_drawer_z + drawer_front_height + panel_thickness / 2.0,
            )
        ),
        material=body_wood,
        name="mid_shelf",
    )

    guide_thickness = 0.012
    guide_height = 0.024
    guide_length = 0.490
    guide_y0 = 0.055
    left_guide_x = -inner_width / 2.0 + guide_thickness / 2.0
    right_guide_x = inner_width / 2.0 - guide_thickness / 2.0
    lower_guide_z = lower_drawer_z + drawer_box_floor_z + 0.044
    upper_guide_z = upper_drawer_z + drawer_box_floor_z + 0.044

    body.visual(
        Box((guide_thickness, guide_length, guide_height)),
        origin=Origin(
            xyz=(left_guide_x, guide_y0 + guide_length / 2.0, lower_guide_z)
        ),
        material=runner_wood,
        name="lower_left_guide",
    )
    body.visual(
        Box((guide_thickness, guide_length, guide_height)),
        origin=Origin(
            xyz=(right_guide_x, guide_y0 + guide_length / 2.0, lower_guide_z)
        ),
        material=runner_wood,
        name="lower_right_guide",
    )
    body.visual(
        Box((guide_thickness, guide_length, guide_height)),
        origin=Origin(
            xyz=(left_guide_x, guide_y0 + guide_length / 2.0, upper_guide_z)
        ),
        material=runner_wood,
        name="upper_left_guide",
    )
    body.visual(
        Box((guide_thickness, guide_length, guide_height)),
        origin=Origin(
            xyz=(right_guide_x, guide_y0 + guide_length / 2.0, upper_guide_z)
        ),
        material=runner_wood,
        name="upper_right_guide",
    )

    body.inertial = Inertial.from_geometry(
        Box((cabinet_width, cabinet_depth, cabinet_height)),
        mass=42.0,
        origin=Origin(xyz=(0.0, cabinet_depth / 2.0, cabinet_height / 2.0)),
    )

    _build_drawer(
        model,
        name="lower_drawer",
        front_material=drawer_front_wood,
        box_material=drawer_box_wood,
        runner_material=runner_wood,
        handle_material=handle_metal,
        front_width=drawer_front_width,
        front_height=drawer_front_height,
        front_thickness=drawer_front_thickness,
        box_width=drawer_box_width,
        box_depth=drawer_box_depth,
        box_height=drawer_box_height,
        box_floor_z=drawer_box_floor_z,
        runner_thickness=runner_thickness,
        runner_height=runner_height,
        runner_length=runner_length,
        runner_y0=runner_y0,
    )
    _build_drawer(
        model,
        name="upper_drawer",
        front_material=drawer_front_wood,
        box_material=drawer_box_wood,
        runner_material=runner_wood,
        handle_material=handle_metal,
        front_width=drawer_front_width,
        front_height=drawer_front_height,
        front_thickness=drawer_front_thickness,
        box_width=drawer_box_width,
        box_depth=drawer_box_depth,
        box_height=drawer_box_height,
        box_floor_z=drawer_box_floor_z,
        runner_thickness=runner_thickness,
        runner_height=runner_height,
        runner_length=runner_length,
        runner_y0=runner_y0,
    )

    kick_plate = model.part("kick_plate")
    kick_width = inner_width - 0.024
    kick_thickness = 0.016
    kick_height = toe_kick_height
    kick_plate.visual(
        Box((kick_width, kick_thickness, kick_height)),
        origin=Origin(xyz=(0.0, kick_thickness / 2.0, kick_height / 2.0)),
        material=body_wood,
        name="kick_panel",
    )
    kick_plate.visual(
        Box((kick_width - 0.120, 0.012, 0.020)),
        origin=Origin(xyz=(0.0, 0.010, kick_height - 0.018)),
        material=runner_wood,
        name="kick_back_batten",
    )
    kick_plate.inertial = Inertial.from_geometry(
        Box((kick_width, kick_thickness, kick_height)),
        mass=2.0,
        origin=Origin(xyz=(0.0, kick_thickness / 2.0, kick_height / 2.0)),
    )

    model.articulation(
        "body_to_lower_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child="lower_drawer",
        origin=Origin(xyz=(0.0, 0.0, lower_drawer_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.55,
            lower=0.0,
            upper=drawer_travel,
        ),
    )
    model.articulation(
        "body_to_upper_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child="upper_drawer",
        origin=Origin(xyz=(0.0, 0.0, upper_drawer_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.55,
            lower=0.0,
            upper=drawer_travel,
        ),
    )
    model.articulation(
        "body_to_kick_plate",
        ArticulationType.REVOLUTE,
        parent=body,
        child=kick_plate,
        origin=Origin(xyz=(0.0, toe_kick_recess, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(72.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("cabinet_body")
    upper_drawer = object_model.get_part("upper_drawer")
    lower_drawer = object_model.get_part("lower_drawer")
    kick_plate = object_model.get_part("kick_plate")
    upper_slide = object_model.get_articulation("body_to_upper_drawer")
    lower_slide = object_model.get_articulation("body_to_lower_drawer")
    kick_hinge = object_model.get_articulation("body_to_kick_plate")

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

    ctx.expect_origin_gap(
        upper_drawer,
        lower_drawer,
        axis="z",
        min_gap=0.37,
        max_gap=0.38,
        name="drawers are stacked with realistic vertical spacing",
    )

    ctx.expect_contact(
        upper_drawer,
        body,
        elem_a="left_runner",
        elem_b="upper_left_guide",
        name="upper drawer left runner bears on the cabinet guide",
    )
    ctx.expect_contact(
        lower_drawer,
        body,
        elem_a="left_runner",
        elem_b="lower_left_guide",
        name="lower drawer left runner bears on the cabinet guide",
    )
    ctx.expect_contact(
        kick_plate,
        body,
        elem_a="kick_panel",
        elem_b="bottom_deck",
        name="kick plate closes against the cabinet floor deck",
    )

    ctx.expect_overlap(
        upper_drawer,
        body,
        axes="y",
        elem_a="left_runner",
        elem_b="upper_left_guide",
        min_overlap=0.45,
        name="upper drawer runner is fully seated on its guide when closed",
    )
    ctx.expect_overlap(
        lower_drawer,
        body,
        axes="y",
        elem_a="left_runner",
        elem_b="lower_left_guide",
        min_overlap=0.45,
        name="lower drawer runner is fully seated on its guide when closed",
    )

    upper_closed = ctx.part_world_position(upper_drawer)
    with ctx.pose({upper_slide: upper_slide.motion_limits.upper}):
        upper_open = ctx.part_world_position(upper_drawer)
        ctx.expect_overlap(
            upper_drawer,
            body,
            axes="y",
            elem_a="left_runner",
            elem_b="upper_left_guide",
            min_overlap=0.16,
            name="upper drawer runner retains insertion at full extension",
        )
    ctx.check(
        "upper drawer extends forward on its slide",
        upper_closed is not None
        and upper_open is not None
        and upper_open[1] < upper_closed[1] - 0.25,
        details=f"closed={upper_closed}, open={upper_open}",
    )

    lower_closed = ctx.part_world_position(lower_drawer)
    with ctx.pose({lower_slide: lower_slide.motion_limits.upper}):
        lower_open = ctx.part_world_position(lower_drawer)
        ctx.expect_overlap(
            lower_drawer,
            body,
            axes="y",
            elem_a="left_runner",
            elem_b="lower_left_guide",
            min_overlap=0.16,
            name="lower drawer runner retains insertion at full extension",
        )
    ctx.check(
        "lower drawer extends forward on its slide",
        lower_closed is not None
        and lower_open is not None
        and lower_open[1] < lower_closed[1] - 0.25,
        details=f"closed={lower_closed}, open={lower_open}",
    )

    kick_closed = ctx.part_world_aabb(kick_plate)
    with ctx.pose({kick_hinge: kick_hinge.motion_limits.upper}):
        kick_open = ctx.part_world_aabb(kick_plate)
    ctx.check(
        "kick plate swings down and outward from the toe recess",
        kick_closed is not None
        and kick_open is not None
        and kick_open[0][1] < kick_closed[0][1] - 0.07
        and kick_open[1][2] < kick_closed[1][2] - 0.03,
        details=f"closed={kick_closed}, open={kick_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
