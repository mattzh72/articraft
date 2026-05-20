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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_upright_refrigerator")

    width = 1.40
    depth = 0.86
    height = 2.08
    wall_thickness = 0.05
    rear_panel_thickness = 0.02
    plinth_height = 0.11
    fascia_height = 0.17
    fascia_depth = 0.09

    door_thickness = 0.065
    door_front_gap = 0.0
    door_top_bottom_clearance = 0.004
    center_split_gap = 0.012
    side_reveal = 0.004
    door_width = (width - (2.0 * side_reveal) - center_split_gap) / 2.0
    door_height = (
        height
        - plinth_height
        - fascia_height
        - (2.0 * door_top_bottom_clearance)
    )
    door_center_z = (
        plinth_height + door_top_bottom_clearance + (door_height / 2.0)
    )
    front_y = depth / 2.0
    left_hinge_x = -((width / 2.0) - side_reveal)
    right_hinge_x = (width / 2.0) - side_reveal

    cabinet_white = model.material("cabinet_white", color=(0.93, 0.95, 0.97))
    cabinet_shadow = model.material("cabinet_shadow", color=(0.70, 0.73, 0.77))
    dark_trim = model.material("dark_trim", color=(0.13, 0.15, 0.17))
    handle_metal = model.material("handle_metal", color=(0.77, 0.79, 0.81))
    knob_black = model.material("knob_black", color=(0.10, 0.11, 0.12))

    cabinet = model.part(
        "cabinet",
        inertial=Inertial.from_geometry(
            Box((width, depth, height)),
            mass=145.0,
            origin=Origin(xyz=(0.0, 0.0, height / 2.0)),
        ),
    )
    cabinet.visual(
        Box((width, depth, plinth_height)),
        origin=Origin(xyz=(0.0, 0.0, plinth_height / 2.0)),
        material=cabinet_shadow,
        name="plinth",
    )
    cabinet.visual(
        Box((wall_thickness, depth, height)),
        origin=Origin(
            xyz=(-(width / 2.0) + (wall_thickness / 2.0), 0.0, height / 2.0)
        ),
        material=cabinet_white,
        name="left_side",
    )
    cabinet.visual(
        Box((wall_thickness, depth, height)),
        origin=Origin(
            xyz=((width / 2.0) - (wall_thickness / 2.0), 0.0, height / 2.0)
        ),
        material=cabinet_white,
        name="right_side",
    )
    cabinet.visual(
        Box((width - (2.0 * wall_thickness), depth - rear_panel_thickness, wall_thickness)),
        origin=Origin(
            xyz=(0.0, rear_panel_thickness / 2.0, plinth_height + (wall_thickness / 2.0))
        ),
        material=cabinet_white,
        name="bottom_deck",
    )
    cabinet.visual(
        Box((width - (2.0 * wall_thickness), depth - rear_panel_thickness, wall_thickness)),
        origin=Origin(
            xyz=(0.0, rear_panel_thickness / 2.0, height - (wall_thickness / 2.0))
        ),
        material=cabinet_white,
        name="top_deck",
    )
    cabinet.visual(
        Box((width - (2.0 * wall_thickness), rear_panel_thickness, height - plinth_height)),
        origin=Origin(
            xyz=(
                0.0,
                -(depth / 2.0) + (rear_panel_thickness / 2.0),
                plinth_height + ((height - plinth_height) / 2.0),
            )
        ),
        material=cabinet_white,
        name="rear_panel",
    )
    cabinet.visual(
        Box((width - (2.0 * wall_thickness), fascia_depth, fascia_height)),
        origin=Origin(
            xyz=(0.0, front_y - (fascia_depth / 2.0), height - (fascia_height / 2.0))
        ),
        material=cabinet_white,
        name="top_fascia",
    )
    cabinet.visual(
        Box((0.035, 0.05, door_height)),
        origin=Origin(xyz=(0.0, front_y - 0.025, door_center_z)),
        material=cabinet_shadow,
        name="center_mullion",
    )

    def add_door(
        *,
        part_name: str,
        slab_sign: float,
        hinge_x: float,
        articulation_name: str,
        lower_limit: float,
        upper_limit: float,
    ) -> None:
        door = model.part(
            part_name,
            inertial=Inertial.from_geometry(
                Box((door_width, door_thickness + 0.05, door_height)),
                mass=24.0,
                origin=Origin(
                    xyz=(slab_sign * (door_width / 2.0), (door_thickness + 0.05) / 2.0, 0.0)
                ),
            ),
        )

        handle_x = slab_sign * (door_width - 0.11)
        trim_x = slab_sign * (door_width - 0.009)
        return_x = -slab_sign * 0.010
        handle_y = door_thickness + 0.030
        standoff_y = 0.080
        hinge_barrel_y = (door_thickness / 2.0) - 0.010

        door.visual(
            Box((door_width, door_thickness, door_height)),
            origin=Origin(xyz=(slab_sign * (door_width / 2.0), door_thickness / 2.0, 0.0)),
            material=cabinet_white,
            name="door_shell",
        )
        door.visual(
            Box((0.020, door_thickness - 0.010, door_height - 0.030)),
            origin=Origin(xyz=(return_x, door_thickness / 2.0, 0.0)),
            material=cabinet_shadow,
            name="hinge_return",
        )
        door.visual(
            Box((0.018, door_thickness + 0.004, door_height - 0.030)),
            origin=Origin(xyz=(trim_x, (door_thickness / 2.0) + 0.002, 0.0)),
            material=dark_trim,
            name="center_gasket",
        )
        door.visual(
            Box((door_width - 0.08, 0.010, door_height - 0.14)),
            origin=Origin(
                xyz=(
                    slab_sign * (door_width / 2.0),
                    door_thickness - 0.005,
                    0.0,
                )
            ),
            material=cabinet_shadow,
            name="front_relief",
        )
        door.visual(
            Cylinder(radius=0.014, length=1.08),
            origin=Origin(xyz=(handle_x, handle_y, 0.0)),
            material=handle_metal,
            name="pull_bar",
        )
        for index, z_offset in enumerate((-0.37, 0.37), start=1):
            door.visual(
                Box((0.026, 0.040, 0.026)),
                origin=Origin(xyz=(handle_x, standoff_y, z_offset)),
                material=handle_metal,
                name=f"pull_standoff_{index}",
            )
        for index, z_offset in enumerate((-0.68, 0.0, 0.68), start=1):
            door.visual(
                Cylinder(radius=0.012, length=0.12),
                origin=Origin(xyz=(0.0, hinge_barrel_y, z_offset)),
                material=handle_metal,
                name=f"hinge_barrel_{index}",
            )

        model.articulation(
            articulation_name,
            ArticulationType.REVOLUTE,
            parent=cabinet,
            child=door,
            origin=Origin(xyz=(hinge_x, front_y + door_front_gap, door_center_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=45.0,
                velocity=1.4,
                lower=lower_limit,
                upper=upper_limit,
            ),
        )

    add_door(
        part_name="left_door",
        slab_sign=1.0,
        hinge_x=left_hinge_x,
        articulation_name="left_door_hinge",
        lower_limit=0.0,
        upper_limit=2.15,
    )
    add_door(
        part_name="right_door",
        slab_sign=-1.0,
        hinge_x=right_hinge_x,
        articulation_name="right_door_hinge",
        lower_limit=-2.15,
        upper_limit=0.0,
    )

    knob = model.part(
        "control_knob",
        inertial=Inertial.from_geometry(
            Cylinder(radius=0.030, length=0.028),
            mass=0.25,
            origin=Origin(xyz=(0.0, 0.014, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        ),
    )
    knob.visual(
        Cylinder(radius=0.030, length=0.028),
        origin=Origin(xyz=(0.0, 0.014, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="knob_body",
    )
    knob.visual(
        Cylinder(radius=0.036, length=0.008),
        origin=Origin(xyz=(0.0, 0.004, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="knob_bezel",
    )
    knob.visual(
        Box((0.006, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, 0.030, 0.017)),
        material=cabinet_white,
        name="knob_indicator",
    )
    model.articulation(
        "control_knob_joint",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=knob,
        origin=Origin(xyz=(0.0, front_y, height - (fascia_height / 2.0))),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    control_knob = object_model.get_part("control_knob")
    left_hinge = object_model.get_articulation("left_door_hinge")
    right_hinge = object_model.get_articulation("right_door_hinge")
    knob_joint = object_model.get_articulation("control_knob_joint")

    height = 2.08

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
    ctx.fail_if_articulation_origin_far_from_geometry(
        tol=0.035,
        name="articulation origins stay on mounted hardware",
    )

    ctx.check(
        "all refrigerator parts are present",
        all(
            part is not None
            for part in (cabinet, left_door, right_door, control_knob)
        ),
        details="Cabinet, both doors, and the control knob must all be authored.",
    )
    ctx.check(
        "door hinge axes are vertical",
        tuple(left_hinge.axis) == (0.0, 0.0, 1.0)
        and tuple(right_hinge.axis) == (0.0, 0.0, 1.0),
        details="Both full-height doors should rotate on vertical side hinges.",
    )
    ctx.check(
        "control knob rotates on fascia normal",
        tuple(knob_joint.axis) == (0.0, 1.0, 0.0),
        details="The knob should spin about a front-to-back axis in the upper fascia.",
    )
    ctx.check(
        "door opening directions mirror across the center split",
        left_hinge.motion_limits is not None
        and right_hinge.motion_limits is not None
        and left_hinge.motion_limits.lower == 0.0
        and left_hinge.motion_limits.upper is not None
        and left_hinge.motion_limits.upper > 1.5
        and right_hinge.motion_limits.upper == 0.0
        and right_hinge.motion_limits.lower is not None
        and right_hinge.motion_limits.lower < -1.5,
        details="Left and right doors should open away from the center on opposite sides.",
    )

    knob_position = ctx.part_world_position(control_knob)
    ctx.check(
        "control knob sits high in the fascia",
        knob_position is not None
        and abs(knob_position[0]) < 0.12
        and knob_position[1] > 0.42
        and knob_position[2] > (height - 0.20),
        details="The control knob should be centered high on the refrigerator face.",
    )

    with ctx.pose({left_hinge: 0.0, right_hinge: 0.0}):
        ctx.expect_contact(
            left_door,
            cabinet,
            name="left door is physically seated against cabinet frame",
        )
        ctx.expect_contact(
            right_door,
            cabinet,
            name="right door is physically seated against cabinet frame",
        )
        ctx.expect_contact(
            control_knob,
            cabinet,
            name="control knob bezel contacts the fascia",
        )
        ctx.expect_gap(
            left_door,
            cabinet,
            axis="y",
            max_gap=0.006,
            max_penetration=0.0,
            name="left door sits just proud of cabinet face",
        )
        ctx.expect_gap(
            right_door,
            cabinet,
            axis="y",
            max_gap=0.006,
            max_penetration=0.0,
            name="right door sits just proud of cabinet face",
        )
        ctx.expect_gap(
            control_knob,
            cabinet,
            axis="y",
            max_gap=0.004,
            max_penetration=0.0,
            name="control knob mounts directly on fascia",
        )
        ctx.expect_gap(
            right_door,
            left_door,
            axis="x",
            min_gap=0.010,
            max_gap=0.014,
            name="center split reads as a narrow door seam",
        )

    with ctx.pose({left_hinge: 1.20, right_hinge: -1.20}):
        ctx.fail_if_parts_overlap_in_current_pose(
            name="open doors and knob remain collision free in service pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
