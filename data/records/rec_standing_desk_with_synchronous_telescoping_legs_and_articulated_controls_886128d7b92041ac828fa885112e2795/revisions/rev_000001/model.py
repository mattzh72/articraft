from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


COLUMN_POSITIONS = {
    "corner": (-0.72, -0.25),
    "long": (0.72, -0.25),
    "return": (-0.72, 0.92),
}

BUTTON_X = (-0.075, -0.025, 0.025, 0.075)


def _l_desktop_shape() -> cq.Workplane:
    """Continuous L-shaped corner worksurface, authored in meters."""
    outline = [
        (-0.95, -0.45),
        (0.95, -0.45),
        (0.95, 0.35),
        (-0.25, 0.35),
        (-0.25, 1.15),
        (-0.95, 1.15),
    ]
    return (
        cq.Workplane("XY")
        .polyline(outline)
        .close()
        .extrude(0.05)
        .edges("|Z")
        .fillet(0.025)
        .translate((0.0, 0.0, 0.745))
    )


def _handset_housing_shape() -> cq.Workplane:
    """Small under-desk keypad shell with four real button pockets."""
    body = (
        cq.Workplane("XY")
        .box(0.24, 0.045, 0.10)
        .edges("|Y")
        .fillet(0.006)
    )

    # The front is local -Y.  Oversize rectangular pockets give each button a
    # visible clearance gap and enough inward travel without intersecting the
    # handset shell.
    for x in BUTTON_X:
        pocket = cq.Workplane("XY").box(0.043, 0.060, 0.028).translate((x, -0.018, -0.010))
        body = body.cut(pocket)

    return body


def _add_square_sleeve(
    frame,
    key: str,
    x: float,
    y: float,
    material: Material,
) -> None:
    """Four wall visuals make a visibly hollow outer lifting column."""
    zc = 0.345
    height = 0.63
    outer = 0.100
    wall = 0.012
    half = outer / 2.0 - wall / 2.0

    frame.visual(
        Box((outer, wall, height)),
        origin=Origin(xyz=(x, y - half, zc)),
        material=material,
        name=f"{key}_front_wall",
    )
    frame.visual(
        Box((outer, wall, height)),
        origin=Origin(xyz=(x, y + half, zc)),
        material=material,
        name=f"{key}_back_wall",
    )
    frame.visual(
        Box((wall, outer, height)),
        origin=Origin(xyz=(x - half, y, zc)),
        material=material,
        name=f"{key}_side_wall_0",
    )
    frame.visual(
        Box((wall, outer, height)),
        origin=Origin(xyz=(x + half, y, zc)),
        material=material,
        name=f"{key}_side_wall_1",
    )
    frame.visual(
        Box((0.18, 0.18, 0.030)),
        origin=Origin(xyz=(x, y, 0.015)),
        material=material,
        name=f"{key}_foot",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="l_shaped_standing_desk")

    wood = model.material("warm_maple", rgba=(0.72, 0.48, 0.25, 1.0))
    black = model.material("black_powdercoat", rgba=(0.015, 0.014, 0.013, 1.0))
    dark = model.material("controller_black", rgba=(0.035, 0.035, 0.038, 1.0))
    rubber = model.material("matte_button_gray", rgba=(0.22, 0.23, 0.24, 1.0))
    blue = model.material("accent_button_blue", rgba=(0.04, 0.18, 0.45, 1.0))
    steel = model.material("brushed_steel", rgba=(0.66, 0.68, 0.67, 1.0))
    screen = model.material("dark_display_glass", rgba=(0.02, 0.05, 0.06, 1.0))

    frame = model.part("frame")

    for key, (x, y) in COLUMN_POSITIONS.items():
        _add_square_sleeve(frame, key, x, y, black)

    # One continuous-looking rigid corner frame ties the three outer columns
    # together at floor level, with the rails set outside the lift cavities.
    frame.visual(
        Box((1.44, 0.045, 0.060)),
        origin=Origin(xyz=(0.0, -0.315, 0.120)),
        material=black,
        name="long_floor_rail",
    )
    frame.visual(
        Box((0.045, 1.17, 0.060)),
        origin=Origin(xyz=(-0.785, 0.335, 0.120)),
        material=black,
        name="return_floor_rail",
    )
    frame.visual(
        Box((0.085, 0.045, 0.060)),
        origin=Origin(xyz=(-0.7625, -0.315, 0.120)),
        material=black,
        name="corner_frame_node_x",
    )
    frame.visual(
        Box((0.045, 0.130, 0.060)),
        origin=Origin(xyz=(-0.785, -0.2925, 0.120)),
        material=black,
        name="corner_frame_node_y",
    )

    lift_stages = model.part("lift_stages")
    lift_stages.visual(
        mesh_from_cadquery(_l_desktop_shape(), "corner_desktop", tolerance=0.001),
        material=wood,
        name="desktop",
    )

    for key, (x, y) in COLUMN_POSITIONS.items():
        lift_stages.visual(
            Box((0.076, 0.076, 0.700)),
            origin=Origin(xyz=(x, y, 0.390)),
            material=steel,
            name=f"{key}_inner_stage",
        )

    # Upper L-shaped steel rails are part of the synchronized moving lift
    # assembly and visibly gather the three inner stages under the top.
    lift_stages.visual(
        Box((1.44, 0.050, 0.045)),
        origin=Origin(xyz=(0.0, -0.25, 0.712)),
        material=black,
        name="long_top_rail",
    )
    lift_stages.visual(
        Box((0.050, 1.17, 0.045)),
        origin=Origin(xyz=(-0.72, 0.335, 0.712)),
        material=black,
        name="return_top_rail",
    )
    lift_stages.visual(
        Box((0.16, 0.16, 0.022)),
        origin=Origin(xyz=(0.72, -0.25, 0.737)),
        material=black,
        name="long_mount_plate",
    )
    lift_stages.visual(
        Box((0.16, 0.16, 0.022)),
        origin=Origin(xyz=(-0.72, -0.25, 0.737)),
        material=black,
        name="corner_mount_plate",
    )
    lift_stages.visual(
        Box((0.16, 0.16, 0.022)),
        origin=Origin(xyz=(-0.72, 0.92, 0.737)),
        material=black,
        name="return_mount_plate",
    )

    handset = model.part("handset")
    handset.visual(
        mesh_from_cadquery(_handset_housing_shape(), "handset_housing", tolerance=0.0008),
        material=dark,
        name="housing",
    )
    handset.visual(
        Box((0.18, 0.033, 0.026)),
        origin=Origin(xyz=(0.0, 0.036, 0.061)),
        material=dark,
        name="mount_tab",
    )
    handset.visual(
        Box((0.13, 0.003, 0.020)),
        origin=Origin(xyz=(0.0, -0.023, 0.030)),
        material=screen,
        name="display_window",
    )

    buttons = []
    for i, x in enumerate(BUTTON_X):
        button = model.part(f"button_{i}")
        buttons.append(button)
        button.visual(
            Box((0.045, 0.012, 0.030)),
            origin=Origin(xyz=(0.0, -0.006, 0.0)),
            material=blue if i in (0, 3) else rubber,
            name="button_cap",
        )

    model.articulation(
        "frame_to_lift_stages",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=lift_stages,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.055, lower=0.0, upper=0.50),
    )
    model.articulation(
        "lift_stages_to_handset",
        ArticulationType.FIXED,
        parent=lift_stages,
        child=handset,
        # Mounted under the front edge of the long run.
        origin=Origin(xyz=(0.35, -0.49, 0.675)),
    )

    for i, x in enumerate(BUTTON_X):
        model.articulation(
            f"button_{i}_slide",
            ArticulationType.PRISMATIC,
            parent=handset,
            child=buttons[i],
            origin=Origin(xyz=(x, -0.0225, -0.010)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.5, velocity=0.04, lower=0.0, upper=0.006),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    lift_stages = object_model.get_part("lift_stages")
    handset = object_model.get_part("handset")
    lift = object_model.get_articulation("frame_to_lift_stages")

    for key in COLUMN_POSITIONS:
        inner = f"{key}_inner_stage"
        ctx.expect_gap(
            lift_stages,
            frame,
            axis="y",
            positive_elem=inner,
            negative_elem=f"{key}_front_wall",
            min_gap=-0.001,
            max_gap=0.001,
            name=f"{key} stage guided by front sleeve wall",
        )
        ctx.expect_gap(
            frame,
            lift_stages,
            axis="y",
            positive_elem=f"{key}_back_wall",
            negative_elem=inner,
            min_gap=-0.001,
            max_gap=0.001,
            name=f"{key} stage guided by back sleeve wall",
        )
        ctx.expect_overlap(
            lift_stages,
            frame,
            axes="z",
            elem_a=inner,
            elem_b=f"{key}_front_wall",
            min_overlap=0.55,
            name=f"{key} stage deeply inserted when lowered",
        )

    ctx.expect_contact(
        handset,
        lift_stages,
        elem_a="mount_tab",
        elem_b="desktop",
        contact_tol=0.002,
        name="handset mount tab touches underside of long run",
    )

    lowered_pos = ctx.part_world_position(lift_stages)
    with ctx.pose({lift: 0.50}):
        raised_pos = ctx.part_world_position(lift_stages)
        for key in COLUMN_POSITIONS:
            ctx.expect_overlap(
                lift_stages,
                frame,
                axes="z",
                elem_a=f"{key}_inner_stage",
                elem_b=f"{key}_front_wall",
                min_overlap=0.10,
                name=f"{key} stage retains insertion at standing height",
            )
    ctx.check(
        "desktop lift travel is upward",
        lowered_pos is not None and raised_pos is not None and raised_pos[2] > lowered_pos[2] + 0.45,
        details=f"lowered={lowered_pos}, raised={raised_pos}",
    )

    for i in range(4):
        joint = object_model.get_articulation(f"button_{i}_slide")
        button = object_model.get_part(f"button_{i}")
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({joint: 0.006}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button_{i} presses inward independently",
            rest_pos is not None and pressed_pos is not None and pressed_pos[1] > rest_pos[1] + 0.005,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
