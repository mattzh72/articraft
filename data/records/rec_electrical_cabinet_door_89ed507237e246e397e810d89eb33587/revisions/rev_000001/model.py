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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="motor_control_center_section")

    enclosure_gray = model.material("enclosure_gray", rgba=(0.78, 0.79, 0.77, 1.0))
    door_gray = model.material("door_gray", rgba=(0.82, 0.83, 0.81, 1.0))
    bucket_gray = model.material("bucket_gray", rgba=(0.58, 0.60, 0.62, 1.0))
    dark_charcoal = model.material("dark_charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    rail_metal = model.material("rail_metal", rgba=(0.70, 0.72, 0.74, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.52, 0.54, 0.56, 1.0))
    caution_yellow = model.material("caution_yellow", rgba=(0.92, 0.75, 0.12, 1.0))

    width = 0.65
    depth = 0.50
    height = 2.10
    steel_t = 0.024
    front_frame_depth = 0.050
    front_x = depth / 2.0
    opening_width = 0.552
    opening_bottom = 0.203
    door_height = 1.774
    hinge_radius = 0.009
    hinge_axis_x = front_x - 0.002
    hinge_axis_y = -(opening_width / 2.0) + hinge_radius
    door_panel_y0 = hinge_radius
    door_panel_width = 0.531
    door_panel_x = 0.015
    bucket_bottom = 0.780
    bucket_joint_x = front_x - front_frame_depth
    bucket_width = 0.480
    bucket_height = 0.390
    bucket_depth = 0.300
    guide_center_z = 0.170

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((steel_t, width, height)),
        origin=Origin(xyz=(-front_x + steel_t / 2.0, 0.0, height / 2.0)),
        material=enclosure_gray,
        name="rear_panel",
    )
    cabinet.visual(
        Box((depth - steel_t, steel_t, height)),
        origin=Origin(xyz=(steel_t / 2.0, -(width / 2.0) + steel_t / 2.0, height / 2.0)),
        material=enclosure_gray,
        name="left_wall",
    )
    cabinet.visual(
        Box((depth - steel_t, steel_t, height)),
        origin=Origin(xyz=(steel_t / 2.0, (width / 2.0) - steel_t / 2.0, height / 2.0)),
        material=enclosure_gray,
        name="right_wall",
    )
    cabinet.visual(
        Box((depth - steel_t, width - (2.0 * steel_t), steel_t)),
        origin=Origin(xyz=(steel_t / 2.0, 0.0, steel_t / 2.0)),
        material=enclosure_gray,
        name="floor_pan",
    )
    cabinet.visual(
        Box((depth - steel_t, width - (2.0 * steel_t), steel_t)),
        origin=Origin(xyz=(steel_t / 2.0, 0.0, height - (steel_t / 2.0))),
        material=enclosure_gray,
        name="roof_pan",
    )

    jamb_width = (width - opening_width) / 2.0
    cabinet.visual(
        Box((front_frame_depth, jamb_width, door_height)),
        origin=Origin(
            xyz=(
                front_x - (front_frame_depth / 2.0),
                -((opening_width / 2.0) + (jamb_width / 2.0)),
                opening_bottom + (door_height / 2.0),
            )
        ),
        material=enclosure_gray,
        name="front_left_jamb",
    )
    cabinet.visual(
        Box((front_frame_depth, jamb_width, door_height)),
        origin=Origin(
            xyz=(
                front_x - (front_frame_depth / 2.0),
                (opening_width / 2.0) + (jamb_width / 2.0),
                opening_bottom + (door_height / 2.0),
            )
        ),
        material=enclosure_gray,
        name="front_right_jamb",
    )
    cabinet.visual(
        Box((front_frame_depth, opening_width, opening_bottom)),
        origin=Origin(
            xyz=(
                front_x - (front_frame_depth / 2.0),
                0.0,
                opening_bottom / 2.0,
            )
        ),
        material=enclosure_gray,
        name="front_bottom_rail",
    )
    cabinet.visual(
        Box((front_frame_depth, opening_width, height - (opening_bottom + door_height))),
        origin=Origin(
            xyz=(
                front_x - (front_frame_depth / 2.0),
                0.0,
                opening_bottom + door_height + ((height - (opening_bottom + door_height)) / 2.0),
            )
        ),
        material=enclosure_gray,
        name="front_top_rail",
    )
    cabinet.visual(
        Box((0.080, width, 0.085)),
        origin=Origin(xyz=(-front_x + 0.040, 0.0, 0.0425)),
        material=dark_charcoal,
        name="base_plinth",
    )
    cabinet.visual(
        Box((0.320, 0.026, 0.028)),
        origin=Origin(
            xyz=(
                0.060,
                -0.275,
                bucket_bottom + guide_center_z,
            )
        ),
        material=rail_metal,
        name="cabinet_left_guide",
    )
    cabinet.visual(
        Box((0.320, 0.026, 0.028)),
        origin=Origin(
            xyz=(
                0.060,
                0.275,
                bucket_bottom + guide_center_z,
            )
        ),
        material=rail_metal,
        name="cabinet_right_guide",
    )
    for index, (z_center, length) in enumerate(((0.475, 0.430), (1.302, 0.430))):
        cabinet.visual(
            Cylinder(radius=hinge_radius, length=length),
            origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, opening_bottom + z_center)),
            material=hinge_metal,
            name=f"cabinet_hinge_knuckle_{index}",
        )
    cabinet.inertial = Inertial.from_geometry(
        Box((depth, width, height)),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, height / 2.0)),
    )

    door = model.part("compartment_door")
    door.visual(
        Box((0.022, door_panel_width, door_height)),
        origin=Origin(
            xyz=(
                door_panel_x,
                door_panel_y0 + (door_panel_width / 2.0),
                door_height / 2.0,
            )
        ),
        material=door_gray,
        name="door_skin",
    )
    door.visual(
        Box((0.032, 0.030, door_height - 0.120)),
        origin=Origin(
            xyz=(
                0.004,
                door_panel_y0 + door_panel_width - 0.015,
                door_height / 2.0,
            )
        ),
        material=door_gray,
        name="door_latch_stile",
    )
    for index, z_center in enumerate((0.120, 0.890, 1.654)):
        door.visual(
            Box((0.018, 0.012, 0.140)),
            origin=Origin(
                xyz=(
                    0.006,
                    0.006,
                    z_center,
                )
            ),
            material=door_gray,
            name=f"door_hinge_leaf_{index}",
        )
    door.visual(
        Box((0.032, door_panel_width, 0.030)),
        origin=Origin(
            xyz=(
                0.004,
                door_panel_y0 + (door_panel_width / 2.0),
                0.015,
            )
        ),
        material=door_gray,
        name="door_bottom_return",
    )
    door.visual(
        Box((0.032, door_panel_width, 0.030)),
        origin=Origin(
            xyz=(
                0.004,
                door_panel_y0 + (door_panel_width / 2.0),
                door_height - 0.015,
            )
        ),
        material=door_gray,
        name="door_top_return",
    )
    door.visual(
        Box((0.008, 0.080, 0.280)),
        origin=Origin(
            xyz=(
                0.030,
                door_panel_y0 + door_panel_width - 0.070,
                0.960,
            )
        ),
        material=dark_charcoal,
        name="door_handle_backplate",
    )
    door.visual(
        Cylinder(radius=0.014, length=0.180),
        origin=Origin(
            xyz=(
                0.036,
                door_panel_y0 + door_panel_width - 0.070,
                0.960,
            )
        ),
        material=dark_charcoal,
        name="door_handle_grip",
    )
    door.visual(
        Box((0.012, 0.100, 0.030)),
        origin=Origin(
            xyz=(
                0.052,
                door_panel_y0 + door_panel_width - 0.070,
                0.960,
            )
        ),
        material=dark_charcoal,
        name="door_handle_lever",
    )
    for index, z_center in enumerate((0.120, 0.890, 1.654)):
        door.visual(
            Cylinder(radius=hinge_radius, length=0.180),
            origin=Origin(xyz=(0.0, 0.0, z_center)),
            material=hinge_metal,
            name=f"door_hinge_knuckle_{index}",
        )
    door.inertial = Inertial.from_geometry(
        Box((0.060, door_panel_y0 + door_panel_width, door_height)),
        mass=22.0,
        origin=Origin(
            xyz=(
                0.020,
                (door_panel_y0 + door_panel_width) / 2.0,
                door_height / 2.0,
            )
        ),
    )

    bucket = model.part("bucket_assembly")
    bucket.visual(
        Box((bucket_depth, bucket_width, 0.020)),
        origin=Origin(xyz=(-(bucket_depth / 2.0), 0.0, 0.010)),
        material=bucket_gray,
        name="bucket_floor",
    )
    bucket.visual(
        Box((bucket_depth - 0.040, bucket_width, 0.016)),
        origin=Origin(xyz=(-0.150, 0.0, bucket_height - 0.008)),
        material=bucket_gray,
        name="bucket_roof",
    )
    bucket.visual(
        Box((bucket_depth, 0.020, bucket_height - 0.040)),
        origin=Origin(
            xyz=(
                -(bucket_depth / 2.0),
                -(bucket_width / 2.0) + 0.010,
                0.020 + ((bucket_height - 0.040) / 2.0),
            )
        ),
        material=bucket_gray,
        name="bucket_left_wall",
    )
    bucket.visual(
        Box((bucket_depth, 0.020, bucket_height - 0.040)),
        origin=Origin(
            xyz=(
                -(bucket_depth / 2.0),
                (bucket_width / 2.0) - 0.010,
                0.020 + ((bucket_height - 0.040) / 2.0),
            )
        ),
        material=bucket_gray,
        name="bucket_right_wall",
    )
    bucket.visual(
        Box((0.020, bucket_width, bucket_height - 0.040)),
        origin=Origin(
            xyz=(
                -bucket_depth + 0.010,
                0.0,
                0.020 + ((bucket_height - 0.040) / 2.0),
            )
        ),
        material=bucket_gray,
        name="bucket_rear_wall",
    )
    bucket.visual(
        Box((0.018, 0.520, 0.400)),
        origin=Origin(xyz=(-0.020, 0.0, 0.200)),
        material=dark_charcoal,
        name="bucket_faceplate",
    )
    bucket.visual(
        Box((0.012, 0.180, 0.180)),
        origin=Origin(xyz=(-0.005, 0.0, 0.220)),
        material=caution_yellow,
        name="bucket_operator_escutcheon",
    )
    bucket.visual(
        Cylinder(radius=0.024, length=0.028),
        origin=Origin(xyz=(0.006, 0.0, 0.220), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_charcoal,
        name="bucket_operator_hub",
    )
    bucket.visual(
        Box((0.016, 0.090, 0.032)),
        origin=Origin(xyz=(0.024, 0.0, 0.220)),
        material=dark_charcoal,
        name="bucket_operator_handle",
    )
    bucket.visual(
        Box((0.300, 0.020, 0.024)),
        origin=Origin(xyz=(-0.150, -0.252, guide_center_z)),
        material=rail_metal,
        name="bucket_left_runner",
    )
    bucket.visual(
        Box((0.300, 0.020, 0.024)),
        origin=Origin(xyz=(-0.150, 0.252, guide_center_z)),
        material=rail_metal,
        name="bucket_right_runner",
    )
    bucket.visual(
        Box((0.110, 0.180, 0.170)),
        origin=Origin(xyz=(-0.180, 0.0, 0.105)),
        material=dark_charcoal,
        name="bucket_contactor_block",
    )
    bucket.visual(
        Box((0.080, 0.130, 0.090)),
        origin=Origin(xyz=(-0.105, 0.0, 0.337)),
        material=bucket_gray,
        name="bucket_terminal_block",
    )
    bucket.inertial = Inertial.from_geometry(
        Box((bucket_depth + 0.050, 0.520, 0.400)),
        mass=26.0,
        origin=Origin(xyz=(-0.120, 0.0, 0.200)),
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, opening_bottom)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(135.0),
        ),
    )
    model.articulation(
        "bucket_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=bucket,
        origin=Origin(xyz=(bucket_joint_x, 0.0, bucket_bottom)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.30,
            lower=0.0,
            upper=0.200,
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

    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("compartment_door")
    bucket = object_model.get_part("bucket_assembly")
    door_hinge = object_model.get_articulation("door_hinge")
    bucket_slide = object_model.get_articulation("bucket_slide")

    ctx.check("cabinet exists", cabinet is not None)
    ctx.check("door exists", door is not None)
    ctx.check("bucket exists", bucket is not None)

    with ctx.pose({door_hinge: 0.0, bucket_slide: 0.0}):
        ctx.expect_gap(
            door,
            cabinet,
            axis="x",
            positive_elem="door_skin",
            negative_elem="front_bottom_rail",
            min_gap=0.001,
            max_gap=0.006,
            name="door sits just proud of the front frame",
        )
        ctx.expect_gap(
            door,
            bucket,
            axis="x",
            positive_elem="door_skin",
            negative_elem="bucket_faceplate",
            min_gap=0.040,
            max_gap=0.090,
            name="retracted bucket stays behind the closed door",
        )
        ctx.expect_contact(
            bucket,
            cabinet,
            elem_a="bucket_left_runner",
            elem_b="cabinet_left_guide",
            name="left bucket runner seats on the left guide rail",
        )
        ctx.expect_contact(
            bucket,
            cabinet,
            elem_a="bucket_right_runner",
            elem_b="cabinet_right_guide",
            name="right bucket runner seats on the right guide rail",
        )

    closed_door_aabb = None
    open_door_aabb = None
    with ctx.pose({door_hinge: 0.0}):
        closed_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: math.radians(100.0)}):
        open_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door swings outward from the left hinge line",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][0] > closed_door_aabb[1][0] + 0.18
        and open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.12,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    retracted_bucket_aabb = None
    extended_bucket_aabb = None
    with ctx.pose({bucket_slide: 0.0}):
        retracted_bucket_aabb = ctx.part_world_aabb(bucket)
    with ctx.pose({bucket_slide: 0.200}):
        extended_bucket_aabb = ctx.part_world_aabb(bucket)
    ctx.check(
        "bucket slides outward on its guide rails",
        retracted_bucket_aabb is not None
        and extended_bucket_aabb is not None
        and extended_bucket_aabb[1][0] > retracted_bucket_aabb[1][0] + 0.180,
        details=f"retracted={retracted_bucket_aabb}, extended={extended_bucket_aabb}",
    )

    with ctx.pose({door_hinge: math.radians(100.0), bucket_slide: 0.200}):
        ctx.expect_overlap(
            bucket,
            cabinet,
            axes="x",
            elem_a="bucket_left_runner",
            elem_b="cabinet_left_guide",
            min_overlap=0.090,
            name="extended bucket retains insertion in the left guide rail",
        )
        ctx.expect_overlap(
            bucket,
            cabinet,
            axes="x",
            elem_a="bucket_right_runner",
            elem_b="cabinet_right_guide",
            min_overlap=0.090,
            name="extended bucket retains insertion in the right guide rail",
        )
        ctx.expect_gap(
            bucket,
            cabinet,
            axis="x",
            positive_elem="bucket_faceplate",
            negative_elem="front_bottom_rail",
            min_gap=0.080,
            name="extended bucket projects clear of the door opening",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
