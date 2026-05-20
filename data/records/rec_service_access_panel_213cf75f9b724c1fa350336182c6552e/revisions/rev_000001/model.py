from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
    model = ArticulatedObject(name="equipment_access_panel")

    housing_paint = model.material("housing_paint", rgba=(0.63, 0.66, 0.69, 1.0))
    frame_paint = model.material("frame_paint", rgba=(0.44, 0.47, 0.50, 1.0))
    door_paint = model.material("door_paint", rgba=(0.76, 0.78, 0.80, 1.0))
    dark_cavity = model.material("dark_cavity", rgba=(0.16, 0.17, 0.18, 1.0))
    hardware = model.material("hardware", rgba=(0.23, 0.24, 0.26, 1.0))

    body_width = 0.80
    body_depth = 0.22
    body_height = 1.00
    wall = 0.02
    front_y = body_depth * 0.5
    shell_depth = body_depth - wall

    opening_width = 0.56
    opening_height = 0.74
    outer_frame_width = 0.66
    outer_frame_height = 0.84
    frame_depth = 0.028
    opening_bottom = 0.13
    opening_center_z = opening_bottom + opening_height * 0.5
    frame_center_z = opening_bottom + opening_height * 0.5

    door_width = 0.60
    door_height = 0.78
    door_thickness = 0.018
    hinge_radius = 0.008
    hinge_axis_x = -door_width * 0.5 - hinge_radius
    hinge_axis_y = front_y + 0.013

    housing = model.part("housing")
    housing.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=65.0,
        origin=Origin(xyz=(0.0, 0.0, body_height * 0.5)),
    )

    housing.visual(
        Box((body_width, wall, body_height)),
        origin=Origin(xyz=(0.0, -front_y + wall * 0.5, body_height * 0.5)),
        material=housing_paint,
        name="back_panel",
    )
    housing.visual(
        Box((wall, shell_depth, body_height)),
        origin=Origin(xyz=(-body_width * 0.5 + wall * 0.5, 0.0, body_height * 0.5)),
        material=housing_paint,
        name="left_wall",
    )
    housing.visual(
        Box((wall, shell_depth, body_height)),
        origin=Origin(xyz=(body_width * 0.5 - wall * 0.5, 0.0, body_height * 0.5)),
        material=housing_paint,
        name="right_wall",
    )
    housing.visual(
        Box((body_width - 2.0 * wall, shell_depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, body_height - wall * 0.5)),
        material=housing_paint,
        name="top_wall",
    )
    housing.visual(
        Box((body_width - 2.0 * wall, shell_depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall * 0.5)),
        material=housing_paint,
        name="bottom_wall",
    )

    housing.visual(
        Box((outer_frame_width, frame_depth, outer_frame_height)),
        origin=Origin(xyz=(0.0, front_y - frame_depth * 0.5, frame_center_z)),
        material=dark_cavity,
        name="opening_recess",
    )

    side_frame_width = (outer_frame_width - opening_width) * 0.5
    top_frame_height = (outer_frame_height - opening_height) * 0.5
    frame_y = front_y - frame_depth * 0.5

    housing.visual(
        Box((side_frame_width, frame_depth, outer_frame_height)),
        origin=Origin(
            xyz=(-opening_width * 0.5 - side_frame_width * 0.5, frame_y, frame_center_z)
        ),
        material=frame_paint,
        name="left_frame",
    )
    housing.visual(
        Box((side_frame_width, frame_depth, outer_frame_height)),
        origin=Origin(
            xyz=(opening_width * 0.5 + side_frame_width * 0.5, frame_y, frame_center_z)
        ),
        material=frame_paint,
        name="right_frame",
    )
    housing.visual(
        Box((opening_width, frame_depth, top_frame_height)),
        origin=Origin(
            xyz=(0.0, frame_y, opening_bottom + opening_height + top_frame_height * 0.5)
        ),
        material=frame_paint,
        name="top_frame",
    )
    housing.visual(
        Box((opening_width, frame_depth, top_frame_height)),
        origin=Origin(xyz=(0.0, frame_y, opening_bottom - top_frame_height * 0.5)),
        material=frame_paint,
        name="bottom_frame",
    )
    housing.visual(
        Box(((body_width - outer_frame_width) * 0.5, frame_depth, body_height)),
        origin=Origin(
            xyz=(
                -body_width * 0.5 + (body_width - outer_frame_width) * 0.25,
                frame_y,
                body_height * 0.5,
            )
        ),
        material=housing_paint,
        name="left_face_skin",
    )
    housing.visual(
        Box(((body_width - outer_frame_width) * 0.5, frame_depth, body_height)),
        origin=Origin(
            xyz=(
                body_width * 0.5 - (body_width - outer_frame_width) * 0.25,
                frame_y,
                body_height * 0.5,
            )
        ),
        material=housing_paint,
        name="right_face_skin",
    )
    housing.visual(
        Box((outer_frame_width, frame_depth, body_height - (opening_bottom + opening_height + top_frame_height))),
        origin=Origin(
            xyz=(
                0.0,
                frame_y,
                opening_bottom + opening_height + top_frame_height + (body_height - (opening_bottom + opening_height + top_frame_height)) * 0.5,
            )
        ),
        material=housing_paint,
        name="top_face_skin",
    )
    housing.visual(
        Box((outer_frame_width, frame_depth, opening_bottom - top_frame_height)),
        origin=Origin(xyz=(0.0, frame_y, (opening_bottom - top_frame_height) * 0.5)),
        material=housing_paint,
        name="bottom_face_skin",
    )

    housing.visual(
        Box((opening_width - 0.10, shell_depth - 0.02, 0.07)),
        origin=Origin(xyz=(0.0, -0.01, opening_bottom + 0.05)),
        material=dark_cavity,
        name="lower_cavity_shelf",
    )
    housing.visual(
        Box((0.08, wall, 0.18)),
        origin=Origin(
            xyz=(opening_width * 0.5 - 0.04, front_y - frame_depth - wall * 0.5, frame_center_z)
        ),
        material=hardware,
        name="latch_strike",
    )
    housing.visual(
        Box((0.016, 0.017, outer_frame_height - 0.02)),
        origin=Origin(
            xyz=(
                -outer_frame_width * 0.5 + 0.008,
                0.1065,
                frame_center_z,
            )
        ),
        material=hardware,
        name="hinge_leaf",
    )

    door = model.part("door")
    door.inertial = Inertial.from_geometry(
        Box((door_width, door_thickness, door_height)),
        mass=11.0,
        origin=Origin(xyz=(hinge_radius + door_width * 0.5, -0.004, 0.0)),
    )
    door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(hinge_radius + door_width * 0.5, -0.004, 0.0)),
        material=door_paint,
        name="door_panel",
    )
    door.visual(
        Cylinder(radius=hinge_radius, length=door_height),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=hardware,
        name="hinge_barrel",
    )
    door.visual(
        Box((0.034, 0.012, 0.16)),
        origin=Origin(xyz=(hinge_radius + door_width - 0.038, 0.010, 0.0)),
        material=hardware,
        name="latch_pull",
    )

    model.articulation(
        "hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, opening_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=0.0, upper=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("hinge")

    ctx.expect_gap(
        door,
        housing,
        axis="y",
        positive_elem="door_panel",
        negative_elem="right_frame",
        max_gap=0.001,
        max_penetration=0.0,
        name="door seats flush against the frame",
    )
    ctx.expect_overlap(
        door,
        housing,
        axes="xz",
        elem_a="door_panel",
        elem_b="opening_recess",
        min_overlap=0.58,
        name="door remains a large panel covering the framed opening",
    )

    closed_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({hinge: 1.45}):
        opened_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "door swings outward from the equipment face",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[1][1] > closed_aabb[1][1] + 0.20,
        details=f"closed_aabb={closed_aabb}, opened_aabb={opened_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
