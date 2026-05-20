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
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hinged_service_access_panel")

    cabinet_width = 0.62
    cabinet_depth = 0.42
    cabinet_height = 0.88
    side_wall = 0.024
    back_wall = 0.018
    face_thickness = 0.012

    opening_width = 0.18
    opening_height = 0.62
    bezel_outer_width = 0.226
    bezel_outer_height = 0.664
    bezel_depth = 0.006

    door_width = 0.214
    door_height = 0.652
    door_thickness = 0.012
    hinge_barrel_radius = 0.0065
    hinge_axis_y = cabinet_depth / 2.0 + 0.0125

    dark_body = model.material("body_dark", rgba=(0.22, 0.24, 0.27, 1.0))
    frame_gray = model.material("frame_gray", rgba=(0.46, 0.49, 0.52, 1.0))
    door_gray = model.material("door_gray", rgba=(0.74, 0.76, 0.78, 1.0))
    hinge_steel = model.material("hinge_steel", rgba=(0.67, 0.69, 0.71, 1.0))
    handle_dark = model.material("handle_dark", rgba=(0.16, 0.17, 0.19, 1.0))

    front_face_y = cabinet_depth / 2.0 - face_thickness / 2.0
    bezel_y = cabinet_depth / 2.0 + bezel_depth / 2.0
    body = model.part("body")

    # Equipment housing shell behind the service opening.
    body.visual(
        Box((cabinet_width, back_wall, cabinet_height)),
        origin=Origin(xyz=(0.0, -cabinet_depth / 2.0 + back_wall / 2.0, 0.0)),
        material=dark_body,
        name="back_panel",
    )
    shell_depth = cabinet_depth - back_wall
    shell_center_y = back_wall / 2.0
    body.visual(
        Box((side_wall, shell_depth, cabinet_height)),
        origin=Origin(
            xyz=(-cabinet_width / 2.0 + side_wall / 2.0, shell_center_y, 0.0)
        ),
        material=dark_body,
        name="left_wall",
    )
    body.visual(
        Box((side_wall, shell_depth, cabinet_height)),
        origin=Origin(
            xyz=(cabinet_width / 2.0 - side_wall / 2.0, shell_center_y, 0.0)
        ),
        material=dark_body,
        name="right_wall",
    )
    body.visual(
        Box((cabinet_width - 2.0 * side_wall, shell_depth, side_wall)),
        origin=Origin(
            xyz=(0.0, shell_center_y, cabinet_height / 2.0 - side_wall / 2.0)
        ),
        material=dark_body,
        name="top_wall",
    )
    body.visual(
        Box((cabinet_width - 2.0 * side_wall, shell_depth, side_wall)),
        origin=Origin(
            xyz=(0.0, shell_center_y, -cabinet_height / 2.0 + side_wall / 2.0)
        ),
        material=dark_body,
        name="bottom_wall",
    )

    # Equipment front face with a cutout for the service opening.
    side_face_width = (cabinet_width - opening_width) / 2.0
    top_face_height = (cabinet_height - opening_height) / 2.0
    body.visual(
        Box((side_face_width, face_thickness, cabinet_height)),
        origin=Origin(
            xyz=(-(opening_width + side_face_width) / 2.0, front_face_y, 0.0)
        ),
        material=dark_body,
        name="front_face_left",
    )
    body.visual(
        Box((side_face_width, face_thickness, cabinet_height)),
        origin=Origin(
            xyz=((opening_width + side_face_width) / 2.0, front_face_y, 0.0)
        ),
        material=dark_body,
        name="front_face_right",
    )
    body.visual(
        Box((opening_width, face_thickness, top_face_height)),
        origin=Origin(
            xyz=(0.0, front_face_y, (opening_height + top_face_height) / 2.0)
        ),
        material=dark_body,
        name="front_face_top",
    )
    body.visual(
        Box((opening_width, face_thickness, top_face_height)),
        origin=Origin(
            xyz=(0.0, front_face_y, -(opening_height + top_face_height) / 2.0)
        ),
        material=dark_body,
        name="front_face_bottom",
    )

    # Raised rectangular frame around the opening.
    bezel_side_width = (bezel_outer_width - opening_width) / 2.0
    bezel_cap_height = (bezel_outer_height - opening_height) / 2.0
    body.visual(
        Box((bezel_side_width, bezel_depth, bezel_outer_height)),
        origin=Origin(
            xyz=(-(opening_width + bezel_side_width) / 2.0, bezel_y, 0.0)
        ),
        material=frame_gray,
        name="bezel_left",
    )
    body.visual(
        Box((bezel_side_width, bezel_depth, bezel_outer_height)),
        origin=Origin(
            xyz=((opening_width + bezel_side_width) / 2.0, bezel_y, 0.0)
        ),
        material=frame_gray,
        name="bezel_right",
    )
    body.visual(
        Box((opening_width, bezel_depth, bezel_cap_height)),
        origin=Origin(
            xyz=(0.0, bezel_y, (opening_height + bezel_cap_height) / 2.0)
        ),
        material=frame_gray,
        name="bezel_top",
    )
    body.visual(
        Box((opening_width, bezel_depth, bezel_cap_height)),
        origin=Origin(
            xyz=(0.0, bezel_y, -(opening_height + bezel_cap_height) / 2.0)
        ),
        material=frame_gray,
        name="bezel_bottom",
    )

    # Fixed hinge leaf and strike on the equipment face.
    body.visual(
        Box((0.010, 0.008, 0.56)),
        origin=Origin(
            xyz=(-door_width / 2.0 - 0.0105, cabinet_depth / 2.0 + 0.004, 0.0)
        ),
        material=hinge_steel,
        name="body_hinge_leaf",
    )
    body.visual(
        Cylinder(radius=hinge_barrel_radius, length=0.12),
        origin=Origin(
            xyz=(-door_width / 2.0, hinge_axis_y, 0.20)
        ),
        material=hinge_steel,
        name="body_hinge_upper",
    )
    body.visual(
        Cylinder(radius=hinge_barrel_radius, length=0.12),
        origin=Origin(
            xyz=(-door_width / 2.0, hinge_axis_y, -0.20)
        ),
        material=hinge_steel,
        name="body_hinge_lower",
    )
    body.visual(
        Box((0.012, 0.008, 0.075)),
        origin=Origin(
            xyz=(door_width / 2.0 + 0.006, bezel_y + 0.001, 0.0)
        ),
        material=hinge_steel,
        name="latch_strike",
    )
    body.inertial = Inertial.from_geometry(
        Box((cabinet_width, cabinet_depth, cabinet_height)),
        mass=28.0,
        origin=Origin(),
    )

    door = model.part("door")
    door.visual(
        Box((door_width - 0.008, door_thickness, door_height)),
        origin=Origin(xyz=((door_width - 0.008) / 2.0 + 0.008, 0.0, 0.0)),
        material=door_gray,
        name="door_skin",
    )
    door.visual(
        Box((door_width - 0.052, 0.004, door_height - 0.080)),
        origin=Origin(xyz=(0.118, 0.004, 0.0)),
        material=frame_gray,
        name="door_stiffener",
    )
    door.visual(
        Box((0.014, 0.004, 0.56)),
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        material=hinge_steel,
        name="door_hinge_leaf",
    )
    door.visual(
        Cylinder(radius=hinge_barrel_radius, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=hinge_steel,
        name="door_hinge_barrel",
    )
    door.visual(
        Box((0.016, 0.018, 0.14)),
        origin=Origin(xyz=(door_width - 0.016, 0.012, 0.0)),
        material=handle_dark,
        name="latch_handle",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_width, door_thickness, door_height)),
        mass=4.0,
        origin=Origin(xyz=(door_width / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-door_width / 2.0, hinge_axis_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=0.0,
            upper=2.1,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("body_to_door")

    ctx.check(
        "door hinge uses a vertical side axis",
        hinge.axis == (0.0, 0.0, 1.0),
        details=f"axis={hinge.axis}",
    )
    ctx.check(
        "door is tall and slender",
        (0.652 / 0.214) > 2.5,
        details="Expected the service door to be much taller than it is wide.",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            door,
            body,
            axis="y",
            positive_elem="door_skin",
            negative_elem="bezel_top",
            min_gap=0.0005,
            max_gap=0.004,
            name="closed door stands just proud of the frame",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="z",
            elem_a="door_skin",
            elem_b="bezel_left",
            min_overlap=0.50,
            name="closed door spans most of the service opening height",
        )

    closed_latch = ctx.part_element_world_aabb(door, elem="latch_handle")
    with ctx.pose({hinge: 1.3}):
        opened_latch = ctx.part_element_world_aabb(door, elem="latch_handle")
        ctx.expect_origin_gap(
            door,
            body,
            axis="y",
            min_gap=0.07,
            name="opened door swings outward from the equipment face",
        )

    ctx.check(
        "door free edge moves outward when opened",
        closed_latch is not None
        and opened_latch is not None
        and (opened_latch[0][1] + opened_latch[1][1]) / 2.0
        > (closed_latch[0][1] + closed_latch[1][1]) / 2.0 + 0.08,
        details=f"closed_latch={closed_latch}, opened_latch={opened_latch}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
