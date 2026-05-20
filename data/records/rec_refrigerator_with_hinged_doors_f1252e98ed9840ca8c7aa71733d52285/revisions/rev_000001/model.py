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
    model = ArticulatedObject(name="back_bar_beverage_refrigerator")

    width = 0.90
    depth = 0.52
    height = 0.86
    wall = 0.03
    back_thickness = 0.025
    top_thickness = 0.03
    floor_thickness = 0.05
    plinth_height = 0.07
    trim_height = 0.10
    mullion_width = 0.024
    door_thickness = 0.032
    door_bottom_gap = 0.004
    door_top_gap = 0.004
    door_height = height - trim_height - plinth_height - door_bottom_gap - door_top_gap
    door_width = 0.448
    shelf_depth = depth - back_thickness - 0.05
    shelf_thickness = 0.006

    stainless = model.material("stainless", rgba=(0.71, 0.73, 0.75, 1.0))
    body_black = model.material("body_black", rgba=(0.12, 0.13, 0.14, 1.0))
    trim_black = model.material("trim_black", rgba=(0.08, 0.08, 0.09, 1.0))
    interior_dark = model.material("interior_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.50, 0.60, 0.66, 0.32))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(-width / 2 + wall / 2, -depth / 2, height / 2)),
        material=body_black,
        name="left_side",
    )
    cabinet.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(width / 2 - wall / 2, -depth / 2, height / 2)),
        material=body_black,
        name="right_side",
    )
    cabinet.visual(
        Box((width - 2 * wall, back_thickness, height)),
        origin=Origin(
            xyz=(0.0, -depth + back_thickness / 2, height / 2),
        ),
        material=body_black,
        name="back_panel",
    )
    cabinet.visual(
        Box((width, depth, top_thickness)),
        origin=Origin(xyz=(0.0, -depth / 2, height - top_thickness / 2)),
        material=stainless,
        name="top_panel",
    )
    cabinet.visual(
        Box((width, depth, floor_thickness)),
        origin=Origin(xyz=(0.0, -depth / 2, floor_thickness / 2)),
        material=interior_dark,
        name="floor_panel",
    )
    cabinet.visual(
        Box((width, wall, plinth_height)),
        origin=Origin(xyz=(0.0, -wall / 2, plinth_height / 2)),
        material=trim_black,
        name="plinth_front",
    )
    cabinet.visual(
        Box((width, wall, trim_height)),
        origin=Origin(xyz=(0.0, -wall / 2, height - trim_height / 2)),
        material=trim_black,
        name="upper_trim",
    )
    cabinet.visual(
        Box((mullion_width, wall, height - trim_height - plinth_height)),
        origin=Origin(
            xyz=(
                0.0,
                -wall / 2,
                plinth_height + (height - trim_height - plinth_height) / 2,
            )
        ),
        material=trim_black,
        name="center_mullion",
    )
    cabinet.visual(
        Box((width - 2 * wall, shelf_depth, shelf_thickness)),
        origin=Origin(xyz=(0.0, -0.05 - shelf_depth / 2, 0.30)),
        material=stainless,
        name="lower_shelf",
    )
    cabinet.visual(
        Box((width - 2 * wall, shelf_depth, shelf_thickness)),
        origin=Origin(xyz=(0.0, -0.05 - shelf_depth / 2, 0.52)),
        material=stainless,
        name="upper_shelf",
    )

    def add_door(part_name: str, swing_sign: float) -> None:
        door = model.part(part_name)

        outer_stile_width = 0.05
        inner_stile_width = 0.045
        top_rail_height = 0.055
        bottom_rail_height = 0.075
        glass_width = door_width - outer_stile_width - inner_stile_width
        glass_height = door_height - top_rail_height - bottom_rail_height
        handle_width = 0.018
        handle_depth = 0.022
        handle_height = 0.34

        door.visual(
            Box((outer_stile_width, door_thickness, door_height)),
            origin=Origin(
                xyz=(
                    swing_sign * outer_stile_width / 2,
                    door_thickness / 2,
                    door_height / 2,
                )
            ),
            material=trim_black,
            name="outer_stile",
        )
        door.visual(
            Box((inner_stile_width, door_thickness, door_height)),
            origin=Origin(
                xyz=(
                    swing_sign * (door_width - inner_stile_width / 2),
                    door_thickness / 2,
                    door_height / 2,
                )
            ),
            material=trim_black,
            name="inner_stile",
        )
        door.visual(
            Box((door_width, door_thickness, top_rail_height)),
            origin=Origin(
                xyz=(
                    swing_sign * door_width / 2,
                    door_thickness / 2,
                    door_height - top_rail_height / 2,
                )
            ),
            material=trim_black,
            name="top_rail",
        )
        door.visual(
            Box((door_width, door_thickness, bottom_rail_height)),
            origin=Origin(
                xyz=(
                    swing_sign * door_width / 2,
                    door_thickness / 2,
                    bottom_rail_height / 2,
                )
            ),
            material=trim_black,
            name="bottom_rail",
        )
        door.visual(
            Box((glass_width, door_thickness * 0.70, glass_height)),
            origin=Origin(
                xyz=(
                    swing_sign * (outer_stile_width + glass_width / 2),
                    door_thickness / 2,
                    bottom_rail_height + glass_height / 2,
                )
            ),
            material=smoked_glass,
            name="glass_panel",
        )
        door.visual(
            Box((handle_width, handle_depth, handle_height)),
            origin=Origin(
                xyz=(
                    swing_sign * (door_width - inner_stile_width / 2),
                    door_thickness + handle_depth / 2,
                    door_height * 0.52,
                )
            ),
            material=stainless,
            name="handle",
        )

    add_door("left_door", swing_sign=1.0)
    add_door("right_door", swing_sign=-1.0)

    thermostat_knob = model.part("thermostat_knob")
    knob_radius = 0.018
    knob_body_length = 0.022
    knob_face_length = 0.010
    thermostat_knob.visual(
        Cylinder(radius=knob_radius, length=knob_body_length),
        origin=Origin(
            xyz=(0.0, knob_body_length / 2, 0.0),
            rpy=(pi / 2, 0.0, 0.0),
        ),
        material=trim_black,
        name="knob_body",
    )
    thermostat_knob.visual(
        Cylinder(radius=0.013, length=knob_face_length),
        origin=Origin(
            xyz=(0.0, knob_body_length + knob_face_length / 2, 0.0),
            rpy=(pi / 2, 0.0, 0.0),
        ),
        material=body_black,
        name="knob_face",
    )
    thermostat_knob.visual(
        Box((0.004, 0.004, 0.018)),
        origin=Origin(
            xyz=(0.0, knob_body_length + knob_face_length + 0.002, 0.010),
        ),
        material=model.material("indicator_red", rgba=(0.88, 0.18, 0.16, 1.0)),
        name="indicator",
    )

    model.articulation(
        "left_door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child="left_door",
        origin=Origin(xyz=(-width / 2, 0.0, plinth_height + door_bottom_gap)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "right_door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child="right_door",
        origin=Origin(xyz=(width / 2, 0.0, plinth_height + door_bottom_gap)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=-1.45,
            upper=0.0,
        ),
    )
    model.articulation(
        "thermostat_rotation",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=thermostat_knob,
        origin=Origin(xyz=(0.28, 0.0, height - trim_height / 2)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=-2.4,
            upper=2.4,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    thermostat_knob = object_model.get_part("thermostat_knob")
    left_hinge = object_model.get_articulation("left_door_hinge")
    right_hinge = object_model.get_articulation("right_door_hinge")
    thermostat_rotation = object_model.get_articulation("thermostat_rotation")

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

    ctx.check(
        "left door hinge axis is vertical",
        tuple(left_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"expected (0, 0, 1), got {left_hinge.axis}",
    )
    ctx.check(
        "right door hinge axis is vertical",
        tuple(right_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"expected (0, 0, 1), got {right_hinge.axis}",
    )
    ctx.check(
        "thermostat axis is front to back",
        tuple(thermostat_rotation.axis) == (0.0, 1.0, 0.0),
        details=f"expected (0, 1, 0), got {thermostat_rotation.axis}",
    )

    with ctx.pose(
        {
            left_hinge: 0.0,
            right_hinge: 0.0,
            thermostat_rotation: 0.0,
        }
    ):
        ctx.expect_contact(
            left_door,
            cabinet,
            name="left door is mounted to cabinet in closed pose",
        )
        ctx.expect_contact(
            right_door,
            cabinet,
            name="right door is mounted to cabinet in closed pose",
        )
        ctx.expect_contact(
            thermostat_knob,
            cabinet,
            name="thermostat knob is mounted to trim",
        )
        ctx.expect_gap(
            right_door,
            left_door,
            axis="x",
            min_gap=0.002,
            max_gap=0.008,
            name="closed doors leave only a slim center seam",
        )
        ctx.expect_overlap(
            thermostat_knob,
            cabinet,
            axes="xz",
            min_overlap=0.02,
            name="thermostat knob sits within the upper trim footprint",
        )

    with ctx.pose({left_hinge: 1.1}):
        left_aabb = ctx.part_world_aabb(left_door)
        ctx.check(
            "left door swings outward",
            left_aabb is not None and left_aabb[1][1] > 0.30,
            details=f"left door AABB in open pose: {left_aabb}",
        )

    with ctx.pose({right_hinge: -1.1}):
        right_aabb = ctx.part_world_aabb(right_door)
        ctx.check(
            "right door swings outward",
            right_aabb is not None and right_aabb[1][1] > 0.30,
            details=f"right door AABB in open pose: {right_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
