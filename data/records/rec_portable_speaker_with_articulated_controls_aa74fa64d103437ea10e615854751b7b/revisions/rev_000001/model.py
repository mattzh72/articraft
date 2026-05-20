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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="personal_speaker")

    width = 0.182
    depth = 0.068
    height = 0.084
    wall = 0.004
    front_bezel_thickness = 0.003
    rear_shell_thickness = 0.0032
    corner_radius = 0.020

    inner_width = width - 2.0 * wall
    inner_depth = depth - front_bezel_thickness - rear_shell_thickness
    shell_span_depth = inner_depth + 0.0006
    grille_width = width - 0.026
    grille_height = height - 0.020

    button_cap_radius = 0.006
    button_hole_width = 0.016
    button_hole_depth = 0.016
    button_center_y = 0.006
    left_button_x = -0.018
    right_button_x = 0.018

    hole_y_min = button_center_y - button_hole_depth * 0.5
    hole_y_max = button_center_y + button_hole_depth * 0.5
    hole_left_min = left_button_x - button_hole_width * 0.5
    hole_left_max = left_button_x + button_hole_width * 0.5
    hole_right_min = right_button_x - button_hole_width * 0.5
    hole_right_max = right_button_x + button_hole_width * 0.5

    shell_black = model.material("shell_black", rgba=(0.15, 0.16, 0.17, 1.0))
    grille_dark = model.material("grille_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    button_gray = model.material("button_gray", rgba=(0.38, 0.40, 0.43, 1.0))
    stand_gray = model.material("stand_gray", rgba=(0.28, 0.29, 0.31, 1.0))
    foot_gray = model.material("foot_gray", rgba=(0.09, 0.09, 0.10, 1.0))

    body = model.part("body")

    front_bezel = ExtrudeWithHolesGeometry(
        rounded_rect_profile(width, height, corner_radius, corner_segments=8),
        [
            rounded_rect_profile(
                width - 0.022,
                height - 0.018,
                corner_radius - 0.007,
                corner_segments=8,
            )
        ],
        front_bezel_thickness,
        center=True,
        cap=True,
        closed=True,
    )
    body.visual(
        mesh_from_geometry(front_bezel, "speaker_front_bezel"),
        origin=Origin(
            xyz=(0.0, -depth * 0.5 + front_bezel_thickness * 0.5, height * 0.5),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=shell_black,
        name="front_bezel",
    )

    front_grille = ExtrudeGeometry(
        rounded_rect_profile(
            grille_width,
            grille_height,
            corner_radius - 0.010,
            corner_segments=8,
        ),
        0.0016,
        cap=True,
        center=True,
        closed=True,
    )
    body.visual(
        mesh_from_geometry(front_grille, "speaker_front_grille"),
        origin=Origin(
            xyz=(0.0, -depth * 0.5 + front_bezel_thickness + 0.0005, height * 0.5),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=grille_dark,
        name="front_grille",
    )

    for index, x_pos in enumerate((-0.050, -0.030, -0.010, 0.010, 0.030, 0.050)):
        body.visual(
            Box((0.0036, 0.0018, grille_height - 0.010)),
            origin=Origin(
                xyz=(
                    x_pos,
                    -depth * 0.5 + front_bezel_thickness + 0.0009,
                    height * 0.5,
                )
            ),
            material=shell_black,
            name=f"grille_rib_{index}",
        )

    for index, x_pos in enumerate((-0.082, 0.082)):
        body.visual(
            Box((0.010, 0.0045, 0.050)),
            origin=Origin(
                xyz=(
                    x_pos,
                    -depth * 0.5 + front_bezel_thickness + 0.0020,
                    height * 0.5,
                )
            ),
            material=shell_black,
            name=f"grille_support_tab_{index}",
        )

    rear_shell = ExtrudeGeometry(
        rounded_rect_profile(width, height, corner_radius, corner_segments=8),
        rear_shell_thickness,
        cap=True,
        center=True,
        closed=True,
    )
    body.visual(
        mesh_from_geometry(rear_shell, "speaker_rear_shell"),
        origin=Origin(
            xyz=(0.0, depth * 0.5 - rear_shell_thickness * 0.5, height * 0.5),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=shell_black,
        name="rear_shell",
    )

    body.visual(
        Box((wall, shell_span_depth, height)),
        origin=Origin(xyz=(-width * 0.5 + wall * 0.5, 0.0, height * 0.5)),
        material=shell_black,
        name="left_wall",
    )
    body.visual(
        Box((wall, shell_span_depth, height)),
        origin=Origin(xyz=(width * 0.5 - wall * 0.5, 0.0, height * 0.5)),
        material=shell_black,
        name="right_wall",
    )
    body.visual(
        Box((inner_width, shell_span_depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall * 0.5)),
        material=shell_black,
        name="bottom_wall",
    )

    span_y_min = -shell_span_depth * 0.5
    span_y_max = shell_span_depth * 0.5
    top_front_depth = hole_y_min - span_y_min
    top_rear_depth = span_y_max - hole_y_max
    top_middle_depth = hole_y_max - hole_y_min
    left_top_width = hole_left_min - (-inner_width * 0.5)
    center_top_width = hole_right_min - hole_left_max
    right_top_width = inner_width * 0.5 - hole_right_max

    body.visual(
        Box((inner_width, top_front_depth, wall)),
        origin=Origin(
            xyz=(
                0.0,
                (span_y_min + hole_y_min) * 0.5,
                height - wall * 0.5,
            )
        ),
        material=shell_black,
        name="top_front_bridge",
    )
    body.visual(
        Box((inner_width, top_rear_depth, wall)),
        origin=Origin(
            xyz=(
                0.0,
                (hole_y_max + span_y_max) * 0.5,
                height - wall * 0.5,
            )
        ),
        material=shell_black,
        name="top_rear_bridge",
    )
    body.visual(
        Box((left_top_width, top_middle_depth, wall)),
        origin=Origin(
            xyz=(
                (-inner_width * 0.5 + hole_left_min) * 0.5,
                button_center_y,
                height - wall * 0.5,
            )
        ),
        material=shell_black,
        name="top_left_bridge",
    )
    body.visual(
        Box((center_top_width, top_middle_depth, wall)),
        origin=Origin(
            xyz=((hole_left_max + hole_right_min) * 0.5, button_center_y, height - wall * 0.5)
        ),
        material=shell_black,
        name="top_center_bridge",
    )
    body.visual(
        Box((right_top_width, top_middle_depth, wall)),
        origin=Origin(
            xyz=(
                (hole_right_max + inner_width * 0.5) * 0.5,
                button_center_y,
                height - wall * 0.5,
            )
        ),
        material=shell_black,
        name="top_right_bridge",
    )

    for index, foot_x in enumerate((-0.046, 0.046)):
        body.visual(
            Box((0.022, 0.008, 0.0024)),
            origin=Origin(xyz=(foot_x, depth * 0.5 - 0.014, 0.0012)),
            material=foot_gray,
            name=f"base_foot_{index}",
        )

    body.inertial = Inertial.from_geometry(
        Box((width, depth, height)),
        mass=0.78,
        origin=Origin(xyz=(0.0, 0.0, height * 0.5)),
    )

    kickstand = model.part("kickstand")
    kickstand.visual(
        Box((0.094, 0.003, 0.048)),
        origin=Origin(xyz=(0.0, 0.0015, 0.024)),
        material=stand_gray,
        name="stand_panel",
    )
    kickstand.visual(
        Box((0.018, 0.006, 0.036)),
        origin=Origin(xyz=(0.0, 0.0030, 0.024)),
        material=stand_gray,
        name="stand_rib",
    )
    kickstand.visual(
        Cylinder(radius=0.0032, length=0.070),
        origin=Origin(xyz=(0.0, 0.0032, 0.0032), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=stand_gray,
        name="stand_hinge_barrel",
    )
    kickstand.visual(
        Box((0.094, 0.006, 0.008)),
        origin=Origin(xyz=(0.0, 0.003, 0.044)),
        material=foot_gray,
        name="stand_tip_pad",
    )
    kickstand.inertial = Inertial.from_geometry(
        Box((0.094, 0.008, 0.052)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.003, 0.026)),
    )

    model.articulation(
        "body_to_kickstand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=kickstand,
        origin=Origin(xyz=(0.0, depth * 0.5, 0.010)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.2,
            lower=0.0,
            upper=1.10,
        ),
    )

    def add_button(name: str, x_pos: float) -> None:
        button = model.part(name)
        button.visual(
            Cylinder(radius=button_cap_radius, length=0.0028),
            origin=Origin(xyz=(0.0, 0.0, 0.0014)),
            material=button_gray,
            name="button_cap",
        )
        button.visual(
            Cylinder(radius=0.0022, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, -0.005)),
            material=button_gray,
            name="button_plunger",
        )
        button.visual(
            Cylinder(radius=0.0090, length=0.0012),
            origin=Origin(xyz=(0.0, 0.0, -(wall + 0.0006))),
            material=button_gray,
            name="button_retainer",
        )
        button.inertial = Inertial.from_geometry(
            Box((0.014, 0.014, 0.014)),
            mass=0.01,
            origin=Origin(xyz=(0.0, 0.0, -0.001)),
        )
        model.articulation(
            f"body_to_{name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, button_center_y, height)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.05,
                lower=0.0,
                upper=0.0022,
            ),
        )

    add_button("left_button", left_button_x)
    add_button("right_button", right_button_x)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    kickstand = object_model.get_part("kickstand")
    left_button = object_model.get_part("left_button")
    right_button = object_model.get_part("right_button")

    kickstand_joint = object_model.get_articulation("body_to_kickstand")
    left_button_joint = object_model.get_articulation("body_to_left_button")
    right_button_joint = object_model.get_articulation("body_to_right_button")

    left_button_cap = left_button.get_visual("button_cap")
    right_button_cap = right_button.get_visual("button_cap")

    with ctx.pose({kickstand_joint: 0.0}):
        ctx.expect_gap(
            kickstand,
            body,
            axis="y",
            min_gap=0.0,
            max_gap=0.001,
            name="kickstand sits on the rear face when closed",
        )
        ctx.expect_overlap(
            kickstand,
            body,
            axes="x",
            min_overlap=0.090,
            name="kickstand spans the speaker width when folded",
        )
        ctx.expect_gap(
            left_button,
            body,
            axis="z",
            positive_elem=left_button_cap,
            min_gap=0.0,
            max_gap=0.001,
            name="left button cap sits proud of the top surface",
        )
        ctx.expect_gap(
            right_button,
            body,
            axis="z",
            positive_elem=right_button_cap,
            min_gap=0.0,
            max_gap=0.001,
            name="right button cap sits proud of the top surface",
        )

    closed_stand_aabb = ctx.part_world_aabb(kickstand)
    with ctx.pose({kickstand_joint: kickstand_joint.motion_limits.upper}):
        open_stand_aabb = ctx.part_world_aabb(kickstand)

    ctx.check(
        "kickstand swings rearward when opened",
        closed_stand_aabb is not None
        and open_stand_aabb is not None
        and open_stand_aabb[1][1] > closed_stand_aabb[1][1] + 0.018,
        details=f"closed={closed_stand_aabb}, open={open_stand_aabb}",
    )

    left_rest = ctx.part_world_position(left_button)
    right_rest = ctx.part_world_position(right_button)
    with ctx.pose(
        {
            left_button_joint: left_button_joint.motion_limits.upper,
            right_button_joint: right_button_joint.motion_limits.upper,
        }
    ):
        left_pressed = ctx.part_world_position(left_button)
        right_pressed = ctx.part_world_position(right_button)

    ctx.check(
        "left button plunges downward into the enclosure",
        left_rest is not None
        and left_pressed is not None
        and left_pressed[2] < left_rest[2] - 0.0018,
        details=f"rest={left_rest}, pressed={left_pressed}",
    )
    ctx.check(
        "right button plunges downward into the enclosure",
        right_rest is not None
        and right_pressed is not None
        and right_pressed[2] < right_rest[2] - 0.0018,
        details=f"rest={right_rest}, pressed={right_pressed}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
