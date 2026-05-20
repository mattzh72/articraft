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
    model = ArticulatedObject(name="rackmount_chassis")

    body_width = 0.445
    rack_width = 0.483
    chassis_height = 0.0889
    chassis_depth = 0.460
    wall = 0.0025
    front_flange_t = 0.004
    rear_frame_t = 0.0035
    door_width = body_width - 0.010
    door_height = chassis_height - 0.006
    door_thickness = 0.018
    cover_width = 0.255
    cover_height = 0.056
    cover_thickness = 0.010

    dark_paint = model.material("dark_paint", rgba=(0.16, 0.17, 0.19, 1.0))
    darker_trim = model.material("darker_trim", rgba=(0.10, 0.11, 0.12, 1.0))
    steel = model.material("steel", rgba=(0.56, 0.58, 0.61, 1.0))
    smoked_acrylic = model.material("smoked_acrylic", rgba=(0.20, 0.30, 0.34, 0.38))
    satin_black = model.material("satin_black", rgba=(0.08, 0.08, 0.09, 1.0))

    chassis = model.part("chassis")
    chassis.visual(
        Box((body_width, chassis_depth, wall)),
        origin=Origin(xyz=(0.0, chassis_depth * 0.5, wall * 0.5)),
        material=dark_paint,
        name="bottom_pan",
    )
    chassis.visual(
        Box((body_width, chassis_depth, wall)),
        origin=Origin(xyz=(0.0, chassis_depth * 0.5, chassis_height - wall * 0.5)),
        material=dark_paint,
        name="top_cover",
    )
    chassis.visual(
        Box((wall, chassis_depth, chassis_height)),
        origin=Origin(xyz=(-body_width * 0.5 + wall * 0.5, chassis_depth * 0.5, chassis_height * 0.5)),
        material=dark_paint,
        name="left_wall",
    )
    chassis.visual(
        Box((wall, chassis_depth, chassis_height)),
        origin=Origin(xyz=(body_width * 0.5 - wall * 0.5, chassis_depth * 0.5, chassis_height * 0.5)),
        material=dark_paint,
        name="right_wall",
    )
    chassis.visual(
        Box((body_width, 0.012, chassis_height)),
        origin=Origin(xyz=(0.0, 0.012 * 0.5, chassis_height * 0.5)),
        material=darker_trim,
        name="front_panel",
    )

    rack_ear_width = (rack_width - body_width) * 0.5
    chassis.visual(
        Box((rack_ear_width, front_flange_t, chassis_height)),
        origin=Origin(
            xyz=(
                -body_width * 0.5 - rack_ear_width * 0.5,
                front_flange_t * 0.5,
                chassis_height * 0.5,
            )
        ),
        material=steel,
        name="left_rack_ear",
    )
    chassis.visual(
        Box((rack_ear_width, front_flange_t, chassis_height)),
        origin=Origin(
            xyz=(
                body_width * 0.5 + rack_ear_width * 0.5,
                front_flange_t * 0.5,
                chassis_height * 0.5,
            )
        ),
        material=steel,
        name="right_rack_ear",
    )

    opening_width = 0.240
    opening_height = 0.050
    opening_center_z = 0.043
    side_fill_width = (body_width - opening_width) * 0.5
    top_fill_height = chassis_height - (opening_center_z + opening_height * 0.5)
    bottom_fill_height = opening_center_z - opening_height * 0.5

    chassis.visual(
        Box((body_width, rear_frame_t, top_fill_height)),
        origin=Origin(
            xyz=(
                0.0,
                chassis_depth - rear_frame_t * 0.5,
                opening_center_z + opening_height * 0.5 + top_fill_height * 0.5,
            )
        ),
        material=dark_paint,
        name="rear_top_frame",
    )
    chassis.visual(
        Box((body_width, rear_frame_t, bottom_fill_height)),
        origin=Origin(
            xyz=(
                0.0,
                chassis_depth - rear_frame_t * 0.5,
                bottom_fill_height * 0.5,
            )
        ),
        material=dark_paint,
        name="rear_bottom_frame",
    )
    chassis.visual(
        Box((side_fill_width, rear_frame_t, opening_height)),
        origin=Origin(
            xyz=(
                -opening_width * 0.5 - side_fill_width * 0.5,
                chassis_depth - rear_frame_t * 0.5,
                opening_center_z,
            )
        ),
        material=dark_paint,
        name="rear_left_frame",
    )
    chassis.visual(
        Box((side_fill_width, rear_frame_t, opening_height)),
        origin=Origin(
            xyz=(
                opening_width * 0.5 + side_fill_width * 0.5,
                chassis_depth - rear_frame_t * 0.5,
                opening_center_z,
            )
        ),
        material=dark_paint,
        name="rear_right_frame",
    )
    chassis.visual(
        Box((0.170, 0.090, 0.060)),
        origin=Origin(xyz=(0.0, chassis_depth - 0.045, opening_center_z)),
        material=satin_black,
        name="fan_shadow_box",
    )
    chassis.inertial = Inertial.from_geometry(
        Box((rack_width, chassis_depth, chassis_height)),
        mass=8.2,
        origin=Origin(xyz=(0.0, chassis_depth * 0.5, chassis_height * 0.5)),
    )

    front_bezel_door = model.part("front_bezel_door")
    bezel_frame = darker_trim
    front_bezel_door.visual(
        Box((0.016, door_thickness, door_height)),
        origin=Origin(xyz=(0.008, 0.0, door_height * 0.5)),
        material=bezel_frame,
        name="left_stile",
    )
    front_bezel_door.visual(
        Box((0.016, door_thickness, door_height)),
        origin=Origin(xyz=(door_width - 0.008, 0.0, door_height * 0.5)),
        material=bezel_frame,
        name="right_stile",
    )
    front_bezel_door.visual(
        Box((door_width, door_thickness, 0.016)),
        origin=Origin(xyz=(door_width * 0.5, 0.0, 0.008)),
        material=bezel_frame,
        name="bottom_rail",
    )
    front_bezel_door.visual(
        Box((door_width, door_thickness, 0.016)),
        origin=Origin(xyz=(door_width * 0.5, 0.0, door_height - 0.008)),
        material=bezel_frame,
        name="top_rail",
    )
    front_bezel_door.visual(
        Box((door_width - 0.020, door_thickness * 0.62, door_height - 0.020)),
        origin=Origin(xyz=(door_width * 0.5, 0.0, door_height * 0.5)),
        material=smoked_acrylic,
        name="door_window",
    )
    front_bezel_door.visual(
        Box((0.010, 0.006, 0.042)),
        origin=Origin(xyz=(door_width - 0.018, -door_thickness * 0.5 - 0.003, door_height * 0.5)),
        material=steel,
        name="door_handle",
    )
    front_bezel_door.visual(
        Cylinder(radius=0.004, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=steel,
        name="upper_hinge_knuckle",
    )
    front_bezel_door.visual(
        Cylinder(radius=0.004, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, door_height - 0.016)),
        material=steel,
        name="lower_hinge_knuckle",
    )
    front_bezel_door.inertial = Inertial.from_geometry(
        Box((door_width, door_thickness, door_height)),
        mass=1.2,
        origin=Origin(xyz=(door_width * 0.5, 0.0, door_height * 0.5)),
    )

    cover_top_z = 0.071
    rear_exhaust_cover = model.part("rear_exhaust_cover")
    rear_exhaust_cover.visual(
        Box((cover_width, cover_thickness, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=steel,
        name="cover_top_rail",
    )
    rear_exhaust_cover.visual(
        Box((cover_width, cover_thickness, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -(cover_height - 0.005))),
        material=steel,
        name="cover_bottom_rail",
    )
    rear_exhaust_cover.visual(
        Box((0.012, cover_thickness, cover_height)),
        origin=Origin(xyz=(-cover_width * 0.5 + 0.006, 0.0, -cover_height * 0.5)),
        material=steel,
        name="cover_left_stile",
    )
    rear_exhaust_cover.visual(
        Box((0.012, cover_thickness, cover_height)),
        origin=Origin(xyz=(cover_width * 0.5 - 0.006, 0.0, -cover_height * 0.5)),
        material=steel,
        name="cover_right_stile",
    )
    for index, z_pos in enumerate((-0.016, -0.026, -0.036, -0.046), start=1):
        rear_exhaust_cover.visual(
            Box((cover_width - 0.024, cover_thickness * 0.55, 0.004)),
            origin=Origin(xyz=(0.0, 0.0, z_pos)),
            material=darker_trim,
            name=f"cover_slat_{index}",
        )
    rear_exhaust_cover.visual(
        Cylinder(radius=0.004, length=0.050),
        origin=Origin(xyz=(-0.075, 0.0, 0.0), rpy=(0.0, pi * 0.5, 0.0)),
        material=steel,
        name="left_cover_knuckle",
    )
    rear_exhaust_cover.visual(
        Cylinder(radius=0.004, length=0.050),
        origin=Origin(xyz=(0.075, 0.0, 0.0), rpy=(0.0, pi * 0.5, 0.0)),
        material=steel,
        name="right_cover_knuckle",
    )
    rear_exhaust_cover.inertial = Inertial.from_geometry(
        Box((cover_width, cover_thickness, cover_height)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, -cover_height * 0.5)),
    )

    model.articulation(
        "bezel_door_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=front_bezel_door,
        origin=Origin(xyz=(-door_width * 0.5, -door_thickness * 0.5, 0.003)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.5,
            lower=0.0,
            upper=2.2,
        ),
    )
    model.articulation(
        "rear_exhaust_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=rear_exhaust_cover,
        origin=Origin(xyz=(0.0, chassis_depth + cover_thickness * 0.5, cover_top_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.2,
            lower=0.0,
            upper=1.35,
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

    chassis = object_model.get_part("chassis")
    front_bezel_door = object_model.get_part("front_bezel_door")
    rear_exhaust_cover = object_model.get_part("rear_exhaust_cover")
    bezel_door_hinge = object_model.get_articulation("bezel_door_hinge")
    rear_exhaust_hinge = object_model.get_articulation("rear_exhaust_hinge")

    ctx.check(
        "front bezel hinge axis is vertical",
        bezel_door_hinge.axis == (0.0, 0.0, -1.0),
        details=f"axis={bezel_door_hinge.axis}",
    )
    ctx.check(
        "rear exhaust hinge axis is horizontal",
        rear_exhaust_hinge.axis == (1.0, 0.0, 0.0),
        details=f"axis={rear_exhaust_hinge.axis}",
    )

    ctx.expect_gap(
        chassis,
        front_bezel_door,
        axis="y",
        max_gap=0.002,
        max_penetration=0.0,
        name="front bezel door sits flush ahead of the chassis",
    )
    ctx.expect_overlap(
        chassis,
        front_bezel_door,
        axes="xz",
        min_overlap=0.080,
        name="front bezel door covers the chassis face",
    )
    ctx.expect_gap(
        rear_exhaust_cover,
        chassis,
        axis="y",
        max_gap=0.002,
        max_penetration=0.0,
        name="rear exhaust cover sits flush on the rear face",
    )
    ctx.expect_overlap(
        rear_exhaust_cover,
        chassis,
        axes="xz",
        min_overlap=0.045,
        name="rear exhaust cover spans the rear opening",
    )

    closed_door_aabb = ctx.part_world_aabb(front_bezel_door)
    with ctx.pose({bezel_door_hinge: 1.2}):
        opened_door_aabb = ctx.part_world_aabb(front_bezel_door)
    ctx.check(
        "front bezel door swings outward from its left edge",
        closed_door_aabb is not None
        and opened_door_aabb is not None
        and opened_door_aabb[0][1] < closed_door_aabb[0][1] - 0.12
        and opened_door_aabb[1][0] < closed_door_aabb[1][0] - 0.08,
        details=f"closed={closed_door_aabb}, opened={opened_door_aabb}",
    )

    closed_cover_aabb = ctx.part_world_aabb(rear_exhaust_cover)
    with ctx.pose({rear_exhaust_hinge: 1.0}):
        opened_cover_aabb = ctx.part_world_aabb(rear_exhaust_cover)
    ctx.check(
        "rear exhaust cover lifts upward from its top edge",
        closed_cover_aabb is not None
        and opened_cover_aabb is not None
        and opened_cover_aabb[1][1] > closed_cover_aabb[1][1] + 0.02
        and opened_cover_aabb[0][2] > closed_cover_aabb[0][2] + 0.015,
        details=f"closed={closed_cover_aabb}, opened={opened_cover_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
