from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _build_turned_knob_mesh():
    profile = [
        (0.0, 0.000),
        (0.010, 0.000),
        (0.012, 0.002),
        (0.009, 0.006),
        (0.007, 0.011),
        (0.010, 0.017),
        (0.016, 0.023),
        (0.011, 0.029),
        (0.0, 0.032),
    ]
    knob_geometry = LatheGeometry(profile, segments=40)
    knob_geometry.rotate_x(-math.pi / 2.0)
    return mesh_from_geometry(knob_geometry, "turned_wood_drawer_knob")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dresser_chest")

    body_wood = model.material("body_wood", rgba=(0.60, 0.42, 0.24, 1.0))
    drawer_wood = model.material("drawer_wood", rgba=(0.67, 0.49, 0.29, 1.0))
    inner_wood = model.material("inner_wood", rgba=(0.71, 0.54, 0.34, 1.0))
    shadow_wood = model.material("shadow_wood", rgba=(0.47, 0.31, 0.18, 1.0))

    carcass_width = 1.18
    carcass_depth = 0.50
    total_height = 0.98
    side_thickness = 0.02
    back_thickness = 0.01
    plinth_height = 0.08
    top_thickness = 0.03
    shelf_thickness = 0.018
    deck_thickness = 0.02
    top_width = 1.22
    top_depth = 0.53
    shelf_depth = 0.476
    shelf_center_y = -0.002

    body = model.part("body")

    side_height = total_height - top_thickness - plinth_height
    side_center_z = plinth_height + side_height * 0.5
    inner_width = carcass_width - 2.0 * side_thickness
    bottom_deck_top = plinth_height + deck_thickness
    opening_height = (side_height - deck_thickness - 3.0 * shelf_thickness) / 4.0

    body.visual(
        Box((carcass_width, carcass_depth, plinth_height)),
        origin=Origin(xyz=(0.0, 0.0, plinth_height * 0.5)),
        material=shadow_wood,
        name="plinth_base",
    )
    body.visual(
        Box((side_thickness, carcass_depth, side_height)),
        origin=Origin(
            xyz=(-carcass_width * 0.5 + side_thickness * 0.5, 0.0, side_center_z)
        ),
        material=body_wood,
        name="left_side",
    )
    body.visual(
        Box((side_thickness, carcass_depth, side_height)),
        origin=Origin(
            xyz=(carcass_width * 0.5 - side_thickness * 0.5, 0.0, side_center_z)
        ),
        material=body_wood,
        name="right_side",
    )
    body.visual(
        Box((inner_width, back_thickness, side_height)),
        origin=Origin(
            xyz=(0.0, -carcass_depth * 0.5 + back_thickness * 0.5, side_center_z)
        ),
        material=inner_wood,
        name="back_panel",
    )
    body.visual(
        Box((top_width, top_depth, top_thickness)),
        origin=Origin(xyz=(0.0, 0.005, total_height - top_thickness * 0.5)),
        material=body_wood,
        name="top_cap",
    )
    body.visual(
        Box((inner_width, shelf_depth, deck_thickness)),
        origin=Origin(xyz=(0.0, shelf_center_y, plinth_height + deck_thickness * 0.5)),
        material=inner_wood,
        name="bottom_deck",
    )

    drawer_centers_z: list[float] = []
    runner_names: list[tuple[str, str]] = []

    for index in range(3):
        shelf_bottom = bottom_deck_top + (index + 1) * opening_height + index * shelf_thickness
        body.visual(
            Box((inner_width, shelf_depth, shelf_thickness)),
            origin=Origin(xyz=(0.0, shelf_center_y, shelf_bottom + shelf_thickness * 0.5)),
            material=inner_wood,
            name=f"divider_shelf_{index + 1}",
        )

    knob_mesh = _build_turned_knob_mesh()

    drawer_front_width = inner_width - 0.008
    drawer_front_height = opening_height - 0.008
    drawer_front_thickness = 0.02
    drawer_box_width = 1.04
    drawer_side_thickness = 0.012
    drawer_back_thickness = 0.012
    drawer_box_height = 0.16
    drawer_back_height = 0.145
    drawer_bottom_thickness = 0.008
    drawer_total_depth = 0.42
    drawer_side_depth = 0.40
    drawer_front_center_y = drawer_total_depth * 0.5 - drawer_front_thickness * 0.5
    drawer_side_center_y = (drawer_front_thickness - drawer_back_thickness) * 0.5
    drawer_back_center_y = (
        drawer_side_center_y - drawer_side_depth * 0.5 + drawer_back_thickness * 0.5
    )
    drawer_bottom_center_y = drawer_side_center_y
    knob_center_y = drawer_total_depth * 0.5 - 0.002
    knob_offset_x = 0.29

    cleat_width = 0.016
    cleat_depth = 0.34
    cleat_height = 0.014
    cleat_center_z = -0.079
    cleat_center_x = drawer_box_width * 0.5 + cleat_width * 0.5

    runner_width = 0.042
    runner_depth = 0.41
    runner_height = 0.014
    runner_center_y = 0.02
    runner_center_z_offset = -0.093
    drawer_closed_center_y = 0.04
    drawer_open_travel = 0.28

    for index in range(4):
        opening_bottom = bottom_deck_top + index * (opening_height + shelf_thickness)
        drawer_center_z = opening_bottom + opening_height * 0.5
        drawer_centers_z.append(drawer_center_z)

        left_runner_name = f"runner_{index + 1}_left"
        right_runner_name = f"runner_{index + 1}_right"
        runner_names.append((left_runner_name, right_runner_name))

        body.visual(
            Box((runner_width, runner_depth, runner_height)),
            origin=Origin(
                xyz=(
                    -inner_width * 0.5 + runner_width * 0.5,
                    runner_center_y,
                    drawer_center_z + runner_center_z_offset,
                )
            ),
            material=inner_wood,
            name=left_runner_name,
        )
        body.visual(
            Box((runner_width, runner_depth, runner_height)),
            origin=Origin(
                xyz=(
                    inner_width * 0.5 - runner_width * 0.5,
                    runner_center_y,
                    drawer_center_z + runner_center_z_offset,
                )
            ),
            material=inner_wood,
            name=right_runner_name,
        )

        drawer = model.part(f"drawer_{index + 1}")
        drawer.visual(
            Box((drawer_front_width, drawer_front_thickness, drawer_front_height)),
            origin=Origin(xyz=(0.0, drawer_front_center_y, 0.0)),
            material=drawer_wood,
            name="front_panel",
        )
        drawer.visual(
            Box((drawer_side_thickness, drawer_side_depth, drawer_box_height)),
            origin=Origin(
                xyz=(
                    -drawer_box_width * 0.5 + drawer_side_thickness * 0.5,
                    drawer_side_center_y,
                    0.0,
                )
            ),
            material=inner_wood,
            name="left_side",
        )
        drawer.visual(
            Box((drawer_side_thickness, drawer_side_depth, drawer_box_height)),
            origin=Origin(
                xyz=(
                    drawer_box_width * 0.5 - drawer_side_thickness * 0.5,
                    drawer_side_center_y,
                    0.0,
                )
            ),
            material=inner_wood,
            name="right_side",
        )
        drawer.visual(
            Box(
                (
                    drawer_box_width - 2.0 * drawer_side_thickness,
                    drawer_back_thickness,
                    drawer_back_height,
                )
            ),
            origin=Origin(xyz=(0.0, drawer_back_center_y, -0.007)),
            material=inner_wood,
            name="back_panel",
        )
        drawer.visual(
            Box(
                (
                    drawer_box_width - 2.0 * drawer_side_thickness,
                    drawer_side_depth,
                    drawer_bottom_thickness,
                )
            ),
            origin=Origin(xyz=(0.0, drawer_bottom_center_y, -0.076)),
            material=inner_wood,
            name="bottom_panel",
        )
        drawer.visual(
            Box((cleat_width, cleat_depth, cleat_height)),
            origin=Origin(xyz=(-cleat_center_x, drawer_side_center_y, cleat_center_z)),
            material=shadow_wood,
            name="left_cleat",
        )
        drawer.visual(
            Box((cleat_width, cleat_depth, cleat_height)),
            origin=Origin(xyz=(cleat_center_x, drawer_side_center_y, cleat_center_z)),
            material=shadow_wood,
            name="right_cleat",
        )
        drawer.visual(
            knob_mesh,
            origin=Origin(xyz=(-knob_offset_x, knob_center_y, 0.0)),
            material=drawer_wood,
            name="left_knob",
        )
        drawer.visual(
            knob_mesh,
            origin=Origin(xyz=(knob_offset_x, knob_center_y, 0.0)),
            material=drawer_wood,
            name="right_knob",
        )
        drawer.inertial = Inertial.from_geometry(
            Box((drawer_front_width, drawer_total_depth, drawer_front_height)),
            mass=6.0,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
        )

        model.articulation(
            f"body_to_drawer_{index + 1}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=drawer,
            origin=Origin(xyz=(0.0, drawer_closed_center_y, drawer_center_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=120.0,
                velocity=0.35,
                lower=0.0,
                upper=drawer_open_travel,
            ),
        )

    body.inertial = Inertial.from_geometry(
        Box((top_width, top_depth, total_height)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, total_height * 0.5)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drawers = [object_model.get_part(f"drawer_{index}") for index in range(1, 5)]
    joints = [object_model.get_articulation(f"body_to_drawer_{index}") for index in range(1, 5)]

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

    for index, (drawer, joint) in enumerate(zip(drawers, joints), start=1):
        limits = joint.motion_limits
        ctx.check(
            f"drawer_{index}_uses_prismatic_slide",
            joint.articulation_type == ArticulationType.PRISMATIC
            and joint.axis == (0.0, 1.0, 0.0)
            and limits is not None
            and limits.lower == 0.0
            and limits.upper == 0.28,
            "Each drawer should slide forward on a +Y prismatic joint with 0.28 m travel.",
        )
        ctx.expect_origin_distance(
            drawer,
            body,
            axes="x",
            max_dist=1e-6,
            name=f"drawer_{index}_is_centered_in_body",
        )
        ctx.expect_contact(
            drawer,
            body,
            elem_a="left_cleat",
            elem_b=f"runner_{index}_left",
            name=f"drawer_{index}_left_runner_contact_closed",
        )
        ctx.expect_contact(
            drawer,
            body,
            elem_a="right_cleat",
            elem_b=f"runner_{index}_right",
            name=f"drawer_{index}_right_runner_contact_closed",
        )

        rest_position = ctx.part_world_position(drawer)
        with ctx.pose({joint: 0.26}):
            open_position = ctx.part_world_position(drawer)
            ctx.check(
                f"drawer_{index}_slides_forward_when_opened",
                rest_position is not None
                and open_position is not None
                and open_position[1] > rest_position[1] + 0.24,
                "Opening the drawer should translate it forward along +Y.",
            )
            ctx.expect_contact(
                drawer,
                body,
                elem_a="left_cleat",
                elem_b=f"runner_{index}_left",
                name=f"drawer_{index}_left_runner_contact_open",
            )
            ctx.expect_contact(
                drawer,
                body,
                elem_a="right_cleat",
                elem_b=f"runner_{index}_right",
                name=f"drawer_{index}_right_runner_contact_open",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
