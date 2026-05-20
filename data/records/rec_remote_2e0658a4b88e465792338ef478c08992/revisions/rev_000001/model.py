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
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _rounded_rect_loop(
    width: float, length: float, radius: float, z: float
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z)
        for x, y in rounded_rect_profile(
            width,
            length,
            radius,
            corner_segments=8,
        )
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="set_top_box_remote")

    body_plastic = model.material("body_plastic", rgba=(0.14, 0.15, 0.16, 1.0))
    soft_black = model.material("soft_black", rgba=(0.08, 0.08, 0.09, 1.0))
    graphite = model.material("graphite", rgba=(0.22, 0.23, 0.25, 1.0))
    button_grey = model.material("button_grey", rgba=(0.34, 0.35, 0.37, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.16, 0.20, 0.24, 0.75))
    accent_red = model.material("accent_red", rgba=(0.72, 0.15, 0.12, 1.0))

    body = model.part("body")

    body_shell = section_loft(
        [
            _rounded_rect_loop(0.050, 0.185, 0.0090, -0.0102),
            _rounded_rect_loop(0.049, 0.183, 0.0086, -0.0018),
            _rounded_rect_loop(0.0465, 0.179, 0.0072, 0.0098),
        ]
    )
    body.visual(
        mesh_from_geometry(body_shell, "remote_body_shell"),
        material=body_plastic,
        name="body_shell",
    )

    body.visual(
        Box((0.020, 0.014, 0.0012)),
        origin=Origin(xyz=(0.0, 0.084, 0.0101)),
        material=smoked_glass,
        name="ir_window",
    )
    body.visual(
        Cylinder(radius=0.0062, length=0.0018),
        origin=Origin(xyz=(0.0, 0.063, 0.0100)),
        material=button_grey,
        name="nav_pad",
    )
    body.visual(
        Cylinder(radius=0.0058, length=0.0018),
        origin=Origin(xyz=(0.0, 0.063, 0.0105)),
        material=graphite,
        name="ok_button",
    )
    body.visual(
        Cylinder(radius=0.0048, length=0.0018),
        origin=Origin(xyz=(0.0, 0.084, 0.0102)),
        material=accent_red,
        name="power_button",
    )
    body.visual(
        Box((0.034, 0.060, 0.0011)),
        origin=Origin(xyz=(0.0, -0.056, 0.0098)),
        material=soft_black,
        name="keypad_zone",
    )
    body.visual(
        Box((0.0022, 0.156, 0.0018)),
        origin=Origin(xyz=(-0.0198, -0.013, 0.0101)),
        material=graphite,
        name="front_left_rail",
    )
    body.visual(
        Box((0.0022, 0.156, 0.0018)),
        origin=Origin(xyz=(0.0198, -0.013, 0.0101)),
        material=graphite,
        name="front_right_rail",
    )
    body.visual(
        Box((0.040, 0.0036, 0.0016)),
        origin=Origin(xyz=(0.0, 0.066, 0.0100)),
        material=graphite,
        name="front_upper_stop",
    )
    body.visual(
        Box((0.040, 0.0036, 0.0016)),
        origin=Origin(xyz=(0.0, -0.091, 0.0100)),
        material=graphite,
        name="front_lower_stop",
    )
    body.visual(
        Box((0.036, 0.092, 0.0010)),
        origin=Origin(xyz=(0.0, -0.010, -0.0096)),
        material=soft_black,
        name="battery_bay_zone",
    )
    body.visual(
        Box((0.0022, 0.132, 0.0018)),
        origin=Origin(xyz=(-0.0185, -0.018, -0.0102)),
        material=graphite,
        name="rear_left_rail",
    )
    body.visual(
        Box((0.0022, 0.132, 0.0018)),
        origin=Origin(xyz=(0.0185, -0.018, -0.0102)),
        material=graphite,
        name="rear_right_rail",
    )
    body.visual(
        Box((0.035, 0.0032, 0.0015)),
        origin=Origin(xyz=(0.0, 0.049, -0.0100)),
        material=graphite,
        name="rear_upper_stop",
    )
    body.visual(
        Box((0.024, 0.0070, 0.0016)),
        origin=Origin(xyz=(0.0, -0.081, -0.0102)),
        material=graphite,
        name="rear_release_step",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.050, 0.185, 0.022)),
        mass=0.22,
        origin=Origin(),
    )

    sliding_cover = model.part("sliding_cover")
    sliding_cover.visual(
        Box((0.0430, 0.072, 0.0026)),
        origin=Origin(xyz=(0.0, -0.056, 0.0129)),
        material=graphite,
        name="cover_panel",
    )
    sliding_cover.visual(
        Box((0.0018, 0.072, 0.0022)),
        origin=Origin(xyz=(-0.0223, -0.056, 0.0115)),
        material=graphite,
        name="cover_left_skirt",
    )
    sliding_cover.visual(
        Box((0.0018, 0.072, 0.0022)),
        origin=Origin(xyz=(0.0223, -0.056, 0.0115)),
        material=graphite,
        name="cover_right_skirt",
    )
    sliding_cover.visual(
        Box((0.0022, 0.084, 0.0008)),
        origin=Origin(xyz=(-0.0198, -0.056, 0.0114)),
        material=graphite,
        name="cover_left_runner",
    )
    sliding_cover.visual(
        Box((0.0022, 0.084, 0.0008)),
        origin=Origin(xyz=(0.0198, -0.056, 0.0114)),
        material=graphite,
        name="cover_right_runner",
    )
    sliding_cover.visual(
        Box((0.020, 0.006, 0.0020)),
        origin=Origin(xyz=(0.0, -0.089, 0.0140)),
        material=soft_black,
        name="cover_thumb_ridge",
    )
    sliding_cover.inertial = Inertial.from_geometry(
        Box((0.046, 0.078, 0.006)),
        mass=0.025,
        origin=Origin(xyz=(0.0, -0.056, 0.0122)),
    )

    battery_door = model.part("battery_door")
    battery_door.visual(
        Box((0.040, 0.106, 0.0022)),
        origin=Origin(xyz=(0.0, -0.010, -0.0126)),
        material=graphite,
        name="battery_door_panel",
    )
    battery_door.visual(
        Box((0.0018, 0.104, 0.0022)),
        origin=Origin(xyz=(-0.0208, -0.010, -0.0115)),
        material=graphite,
        name="battery_left_skirt",
    )
    battery_door.visual(
        Box((0.0018, 0.104, 0.0022)),
        origin=Origin(xyz=(0.0208, -0.010, -0.0115)),
        material=graphite,
        name="battery_right_skirt",
    )
    battery_door.visual(
        Box((0.0022, 0.114, 0.0010)),
        origin=Origin(xyz=(-0.0185, -0.010, -0.0116)),
        material=graphite,
        name="battery_left_runner",
    )
    battery_door.visual(
        Box((0.0022, 0.114, 0.0010)),
        origin=Origin(xyz=(0.0185, -0.010, -0.0116)),
        material=graphite,
        name="battery_right_runner",
    )
    battery_door.visual(
        Box((0.018, 0.008, 0.0018)),
        origin=Origin(xyz=(0.0, -0.061, -0.0140)),
        material=soft_black,
        name="battery_door_grip",
    )
    battery_door.inertial = Inertial.from_geometry(
        Box((0.043, 0.108, 0.006)),
        mass=0.018,
        origin=Origin(xyz=(0.0, -0.010, -0.0122)),
    )

    model.articulation(
        "body_to_cover",
        ArticulationType.PRISMATIC,
        parent=body,
        child=sliding_cover,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.18,
            lower=0.0,
            upper=0.068,
        ),
    )
    model.articulation(
        "body_to_battery_door",
        ArticulationType.PRISMATIC,
        parent=body,
        child=battery_door,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.10,
            lower=0.0,
            upper=0.034,
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

    body = object_model.get_part("body")
    sliding_cover = object_model.get_part("sliding_cover")
    battery_door = object_model.get_part("battery_door")
    cover_slide = object_model.get_articulation("body_to_cover")
    battery_slide = object_model.get_articulation("body_to_battery_door")

    ctx.check("body part exists", body is not None)
    ctx.check("sliding cover part exists", sliding_cover is not None)
    ctx.check("battery door part exists", battery_door is not None)

    ctx.expect_overlap(
        sliding_cover,
        body,
        axes="xy",
        elem_a="cover_panel",
        elem_b="keypad_zone",
        min_overlap=0.030,
        name="closed cover sits over numeric keypad",
    )
    ctx.expect_overlap(
        battery_door,
        body,
        axes="xy",
        elem_a="battery_door_panel",
        elem_b="battery_bay_zone",
        min_overlap=0.030,
        name="closed battery door covers rear battery bay",
    )
    ctx.expect_contact(
        sliding_cover,
        body,
        elem_a="cover_left_runner",
        elem_b="front_left_rail",
        name="cover runner rides on front guide rail at rest",
    )
    ctx.expect_contact(
        battery_door,
        body,
        elem_a="battery_left_runner",
        elem_b="rear_left_rail",
        name="battery door runner rides on rear guide rail at rest",
    )

    cover_rest = ctx.part_world_position(sliding_cover)
    with ctx.pose({cover_slide: 0.068}):
        cover_open = ctx.part_world_position(sliding_cover)
        ctx.expect_gap(
            sliding_cover,
            body,
            axis="y",
            positive_elem="cover_panel",
            negative_elem="keypad_zone",
            min_gap=0.001,
            name="open cover clears numeric keypad area",
        )
        ctx.expect_overlap(
            sliding_cover,
            body,
            axes="y",
            elem_a="cover_left_runner",
            elem_b="front_left_rail",
            min_overlap=0.070,
            name="open cover remains engaged on front guide rail",
        )
        ctx.expect_contact(
            sliding_cover,
            body,
            elem_a="cover_left_runner",
            elem_b="front_left_rail",
            name="open cover still rides on front guide rail",
        )

    ctx.check(
        "cover slides toward remote head",
        cover_rest is not None
        and cover_open is not None
        and cover_open[1] > cover_rest[1] + 0.05,
        details=f"rest={cover_rest}, open={cover_open}",
    )

    battery_rest = ctx.part_world_position(battery_door)
    with ctx.pose({battery_slide: 0.034}):
        battery_open = ctx.part_world_position(battery_door)
        ctx.expect_overlap(
            battery_door,
            body,
            axes="y",
            elem_a="battery_left_runner",
            elem_b="rear_left_rail",
            min_overlap=0.085,
            name="battery door stays captured by rear rail",
        )
        ctx.expect_contact(
            battery_door,
            body,
            elem_a="battery_left_runner",
            elem_b="rear_left_rail",
            name="battery door remains in rail contact when slid open",
        )

    ctx.check(
        "battery door slides toward remote tail",
        battery_rest is not None
        and battery_open is not None
        and battery_open[1] < battery_rest[1] - 0.02,
        details=f"rest={battery_rest}, open={battery_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
