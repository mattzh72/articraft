from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _add_rect_frame(
    part,
    *,
    size_x: float,
    size_y: float,
    thickness: float,
    bar: float,
    material,
    prefix: str,
) -> None:
    half_x = size_x / 2.0
    half_y = size_y / 2.0
    part.visual(
        Box((size_x, bar, thickness)),
        origin=Origin(xyz=(0.0, half_y - bar / 2.0, 0.0)),
        material=material,
        name=f"{prefix}_rear_frame",
    )
    part.visual(
        Box((size_x, bar, thickness)),
        origin=Origin(xyz=(0.0, -(half_y - bar / 2.0), 0.0)),
        material=material,
        name=f"{prefix}_front_frame",
    )
    part.visual(
        Box((bar, size_y - 2.0 * bar, thickness)),
        origin=Origin(xyz=(half_x - bar / 2.0, 0.0, 0.0)),
        material=material,
        name=f"{prefix}_right_frame",
    )
    part.visual(
        Box((bar, size_y - 2.0 * bar, thickness)),
        origin=Origin(xyz=(-(half_x - bar / 2.0), 0.0, 0.0)),
        material=material,
        name=f"{prefix}_left_frame",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="display_freezer")

    body_white = model.material("body_white", rgba=(0.93, 0.95, 0.97, 1.0))
    liner_white = model.material("liner_white", rgba=(0.98, 0.99, 1.0, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.34, 0.37, 0.40, 1.0))
    dark_seal = model.material("dark_seal", rgba=(0.12, 0.12, 0.13, 1.0))
    glass = model.material("tinted_glass", rgba=(0.72, 0.86, 0.95, 0.30))

    cabinet_length = 1.82
    cabinet_depth = 0.92
    body_top_z = 0.78
    total_height = 0.86
    plinth_height = 0.10
    outer_wall_thickness = 0.045
    inner_wall_thickness = 0.025
    top_frame_height = total_height - body_top_z

    lid_length = 0.80
    lid_depth = 0.74
    lid_frame_bar = 0.05
    lid_thickness = 0.024
    lid_glass_thickness = 0.006
    lower_lid_z = total_height + lid_thickness / 2.0
    upper_lid_z = lower_lid_z + lid_thickness + 0.003
    closed_lid_x = 0.415
    slide_travel = 0.30

    upper_support_depth = 0.016
    upper_support_height = upper_lid_z - lid_thickness / 2.0 - total_height

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((1.50, 0.58, plinth_height)),
        origin=Origin(xyz=(0.0, 0.0, plinth_height / 2.0)),
        material=trim_gray,
        name="plinth",
    )
    cabinet.visual(
        Box((1.73, 0.83, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, plinth_height + 0.01)),
        material=body_white,
        name="base_pan",
    )
    cabinet.visual(
        Box((cabinet_length, outer_wall_thickness, body_top_z - plinth_height)),
        origin=Origin(
            xyz=(0.0, cabinet_depth / 2.0 - outer_wall_thickness / 2.0, plinth_height + (body_top_z - plinth_height) / 2.0)
        ),
        material=body_white,
        name="outer_back_wall",
    )
    cabinet.visual(
        Box((cabinet_length, outer_wall_thickness, body_top_z - plinth_height)),
        origin=Origin(
            xyz=(0.0, -(cabinet_depth / 2.0 - outer_wall_thickness / 2.0), plinth_height + (body_top_z - plinth_height) / 2.0)
        ),
        material=body_white,
        name="outer_front_wall",
    )
    cabinet.visual(
        Box((outer_wall_thickness, cabinet_depth - 2.0 * outer_wall_thickness, body_top_z - plinth_height)),
        origin=Origin(
            xyz=(cabinet_length / 2.0 - outer_wall_thickness / 2.0, 0.0, plinth_height + (body_top_z - plinth_height) / 2.0)
        ),
        material=body_white,
        name="outer_right_wall",
    )
    cabinet.visual(
        Box((outer_wall_thickness, cabinet_depth - 2.0 * outer_wall_thickness, body_top_z - plinth_height)),
        origin=Origin(
            xyz=(-(cabinet_length / 2.0 - outer_wall_thickness / 2.0), 0.0, plinth_height + (body_top_z - plinth_height) / 2.0)
        ),
        material=body_white,
        name="outer_left_wall",
    )
    cabinet.visual(
        Box((1.63, 0.68, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, plinth_height + 0.015)),
        material=liner_white,
        name="liner_floor",
    )
    cabinet.visual(
        Box((1.58, inner_wall_thickness, body_top_z - plinth_height + 0.02)),
        origin=Origin(
            xyz=(0.0, 0.34, plinth_height + (body_top_z - plinth_height + 0.02) / 2.0)
        ),
        material=liner_white,
        name="liner_back_wall",
    )
    cabinet.visual(
        Box((1.58, inner_wall_thickness, body_top_z - plinth_height + 0.02)),
        origin=Origin(
            xyz=(0.0, -0.34, plinth_height + (body_top_z - plinth_height + 0.02) / 2.0)
        ),
        material=liner_white,
        name="liner_front_wall",
    )
    cabinet.visual(
        Box((inner_wall_thickness, 0.63, body_top_z - plinth_height + 0.02)),
        origin=Origin(
            xyz=(0.79, 0.0, plinth_height + (body_top_z - plinth_height + 0.02) / 2.0)
        ),
        material=liner_white,
        name="liner_right_wall",
    )
    cabinet.visual(
        Box((inner_wall_thickness, 0.63, body_top_z - plinth_height + 0.02)),
        origin=Origin(
            xyz=(-0.79, 0.0, plinth_height + (body_top_z - plinth_height + 0.02) / 2.0)
        ),
        material=liner_white,
        name="liner_left_wall",
    )
    cabinet.visual(
        Box((cabinet_length, 0.08, top_frame_height)),
        origin=Origin(xyz=(0.0, 0.42, body_top_z + top_frame_height / 2.0)),
        material=trim_gray,
        name="top_back_rail",
    )
    cabinet.visual(
        Box((cabinet_length, 0.08, top_frame_height)),
        origin=Origin(xyz=(0.0, -0.42, body_top_z + top_frame_height / 2.0)),
        material=trim_gray,
        name="top_front_rail",
    )
    cabinet.visual(
        Box((0.08, 0.76, top_frame_height)),
        origin=Origin(xyz=(0.87, 0.0, body_top_z + top_frame_height / 2.0)),
        material=trim_gray,
        name="top_right_end_rail",
    )
    cabinet.visual(
        Box((0.08, 0.76, top_frame_height)),
        origin=Origin(xyz=(-0.87, 0.0, body_top_z + top_frame_height / 2.0)),
        material=trim_gray,
        name="top_left_end_rail",
    )
    cabinet.visual(
        Box((1.58, 0.055, 0.02)),
        origin=Origin(xyz=(0.0, 0.3675, 0.79)),
        material=body_white,
        name="back_rim_bridge",
    )
    cabinet.visual(
        Box((1.58, 0.055, 0.02)),
        origin=Origin(xyz=(0.0, -0.3675, 0.79)),
        material=body_white,
        name="front_rim_bridge",
    )
    cabinet.visual(
        Box((0.055, 0.63, 0.02)),
        origin=Origin(xyz=(0.8175, 0.0, 0.79)),
        material=body_white,
        name="right_rim_bridge",
    )
    cabinet.visual(
        Box((0.055, 0.63, 0.02)),
        origin=Origin(xyz=(-0.8175, 0.0, 0.79)),
        material=body_white,
        name="left_rim_bridge",
    )
    cabinet.visual(
        Box((0.80, upper_support_depth, upper_support_height)),
        origin=Origin(
            xyz=(0.40, 0.388, total_height + upper_support_height / 2.0)
        ),
        material=dark_seal,
        name="upper_back_guide",
    )
    cabinet.visual(
        Box((0.80, upper_support_depth, upper_support_height)),
        origin=Origin(
            xyz=(0.40, -0.388, total_height + upper_support_height / 2.0)
        ),
        material=dark_seal,
        name="upper_front_guide",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((cabinet_length, cabinet_depth, total_height)),
        mass=92.0,
        origin=Origin(xyz=(0.0, 0.0, total_height / 2.0)),
    )

    lower_lid = model.part("lower_lid")
    _add_rect_frame(
        lower_lid,
        size_x=lid_length,
        size_y=lid_depth,
        thickness=lid_thickness,
        bar=lid_frame_bar,
        material=trim_gray,
        prefix="lower_lid",
    )
    lower_lid.visual(
        Box((lid_length - 2.0 * lid_frame_bar, lid_depth - 2.0 * lid_frame_bar, lid_glass_thickness)),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=glass,
        name="lower_lid_glass",
    )
    lower_lid.visual(
        Box((lid_length, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, -lid_depth / 2.0 + 0.020, 0.010)),
        material=dark_seal,
        name="lower_pull_rail",
    )
    lower_lid.visual(
        Box((0.76, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, 0.425, -0.007)),
        material=dark_seal,
        name="lower_back_runner",
    )
    lower_lid.visual(
        Box((0.76, 0.045, 0.010)),
        origin=Origin(xyz=(0.0, 0.3925, -0.007)),
        material=dark_seal,
        name="lower_back_runner_flange",
    )
    lower_lid.visual(
        Box((0.76, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, -0.425, -0.007)),
        material=dark_seal,
        name="lower_front_runner",
    )
    lower_lid.visual(
        Box((0.76, 0.045, 0.010)),
        origin=Origin(xyz=(0.0, -0.3925, -0.007)),
        material=dark_seal,
        name="lower_front_runner_flange",
    )
    lower_lid.inertial = Inertial.from_geometry(
        Box((lid_length, lid_depth, lid_thickness)),
        mass=9.0,
        origin=Origin(),
    )

    upper_lid = model.part("upper_lid")
    _add_rect_frame(
        upper_lid,
        size_x=lid_length,
        size_y=lid_depth,
        thickness=lid_thickness,
        bar=lid_frame_bar,
        material=trim_gray,
        prefix="upper_lid",
    )
    upper_lid.visual(
        Box((0.70, 0.365, lid_glass_thickness)),
        origin=Origin(xyz=(0.0, -0.1375, -0.002)),
        material=glass,
        name="upper_front_glass",
    )
    upper_lid.visual(
        Box((0.175, 0.150, lid_glass_thickness)),
        origin=Origin(xyz=(-0.2625, 0.14, -0.002)),
        material=glass,
        name="upper_left_glass_strip",
    )
    upper_lid.visual(
        Box((0.175, 0.150, lid_glass_thickness)),
        origin=Origin(xyz=(0.2625, 0.14, -0.002)),
        material=glass,
        name="upper_right_glass_strip",
    )
    upper_lid.visual(
        Box((0.70, 0.085, lid_glass_thickness)),
        origin=Origin(xyz=(0.0, 0.2775, -0.002)),
        material=glass,
        name="upper_rear_glass_strip",
    )
    upper_lid.visual(
        Box((0.35, 0.020, lid_thickness)),
        origin=Origin(xyz=(0.0, 0.055, 0.0)),
        material=trim_gray,
        name="hatch_front_surround",
    )
    upper_lid.visual(
        Box((0.020, 0.190, lid_thickness)),
        origin=Origin(xyz=(-0.165, 0.14, 0.0)),
        material=trim_gray,
        name="hatch_left_surround",
    )
    upper_lid.visual(
        Box((0.020, 0.190, lid_thickness)),
        origin=Origin(xyz=(0.165, 0.14, 0.0)),
        material=trim_gray,
        name="hatch_right_surround",
    )
    upper_lid.visual(
        Box((0.35, 0.020, lid_thickness)),
        origin=Origin(xyz=(0.0, 0.225, 0.0)),
        material=trim_gray,
        name="hatch_rear_surround",
    )
    upper_lid.visual(
        Box((lid_length, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, lid_depth / 2.0 - 0.020, 0.010)),
        material=dark_seal,
        name="upper_pull_rail",
    )
    upper_lid.visual(
        Box((0.76, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.390, -0.007)),
        material=dark_seal,
        name="upper_back_runner",
    )
    upper_lid.visual(
        Box((0.76, 0.015, 0.010)),
        origin=Origin(xyz=(0.0, 0.3775, -0.007)),
        material=dark_seal,
        name="upper_back_runner_flange",
    )
    upper_lid.visual(
        Box((0.76, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -0.390, -0.007)),
        material=dark_seal,
        name="upper_front_runner",
    )
    upper_lid.visual(
        Box((0.76, 0.015, 0.010)),
        origin=Origin(xyz=(0.0, -0.3775, -0.007)),
        material=dark_seal,
        name="upper_front_runner_flange",
    )
    upper_lid.inertial = Inertial.from_geometry(
        Box((lid_length, lid_depth, lid_thickness)),
        mass=9.5,
        origin=Origin(),
    )

    service_hatch = model.part("service_hatch")
    service_hatch.visual(
        Box((0.30, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, -0.015, 0.0)),
        material=trim_gray,
        name="hatch_rear_frame",
    )
    service_hatch.visual(
        Box((0.30, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, -0.130, 0.0)),
        material=trim_gray,
        name="hatch_front_frame",
    )
    service_hatch.visual(
        Box((0.030, 0.085, 0.018)),
        origin=Origin(xyz=(-0.135, -0.0725, 0.0)),
        material=trim_gray,
        name="hatch_left_frame",
    )
    service_hatch.visual(
        Box((0.030, 0.085, 0.018)),
        origin=Origin(xyz=(0.135, -0.0725, 0.0)),
        material=trim_gray,
        name="hatch_right_frame",
    )
    service_hatch.visual(
        Box((0.24, 0.085, lid_glass_thickness)),
        origin=Origin(xyz=(0.0, -0.0725, -0.002)),
        material=glass,
        name="hatch_glass",
    )
    service_hatch.visual(
        Box((0.30, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -0.120, 0.008)),
        material=dark_seal,
        name="hatch_front_bar",
    )
    service_hatch.inertial = Inertial.from_geometry(
        Box((0.30, 0.145, 0.018)),
        mass=1.2,
        origin=Origin(xyz=(0.0, -0.0725, 0.0)),
    )

    model.articulation(
        "cabinet_to_lower_lid",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lower_lid,
        origin=Origin(xyz=(-closed_lid_x, 0.0, lower_lid_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.35,
            lower=0.0,
            upper=slide_travel,
        ),
    )
    model.articulation(
        "cabinet_to_upper_lid",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=upper_lid,
        origin=Origin(xyz=(closed_lid_x, 0.0, upper_lid_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.35,
            lower=0.0,
            upper=slide_travel,
        ),
    )
    model.articulation(
        "upper_lid_to_service_hatch",
        ArticulationType.REVOLUTE,
        parent=upper_lid,
        child=service_hatch,
        origin=Origin(xyz=(0.0, 0.215, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=0.0,
            upper=1.15,
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
    lower_lid = object_model.get_part("lower_lid")
    upper_lid = object_model.get_part("upper_lid")
    service_hatch = object_model.get_part("service_hatch")

    lower_slide = object_model.get_articulation("cabinet_to_lower_lid")
    upper_slide = object_model.get_articulation("cabinet_to_upper_lid")
    hatch_hinge = object_model.get_articulation("upper_lid_to_service_hatch")

    ctx.expect_overlap(
        lower_lid,
        cabinet,
        axes="x",
        elem_b="top_front_rail",
        min_overlap=0.70,
        name="lower lid stays supported by the freezer frame at rest",
    )
    ctx.expect_overlap(
        upper_lid,
        cabinet,
        axes="x",
        elem_b="upper_front_guide",
        min_overlap=0.70,
        name="upper lid sits on the raised guide track at rest",
    )
    ctx.expect_within(
        service_hatch,
        upper_lid,
        axes="xy",
        margin=0.06,
        name="service hatch nests within the upper lid footprint",
    )

    lower_rest = ctx.part_world_position(lower_lid)
    upper_rest = ctx.part_world_position(upper_lid)

    with ctx.pose({lower_slide: 0.26}):
        lower_open = ctx.part_world_position(lower_lid)
        ctx.expect_overlap(
            lower_lid,
            cabinet,
            axes="x",
            elem_b="top_front_rail",
            min_overlap=0.50,
            name="lower lid retains engagement on the frame when opened",
        )
    with ctx.pose({upper_slide: 0.26}):
        upper_open = ctx.part_world_position(upper_lid)
        ctx.expect_overlap(
            upper_lid,
            cabinet,
            axes="x",
            elem_b="upper_front_guide",
            min_overlap=0.50,
            name="upper lid retains guide overlap when opened",
        )

    ctx.check(
        "lower lid slides toward the center bay",
        lower_rest is not None
        and lower_open is not None
        and lower_open[0] > lower_rest[0] + 0.20,
        details=f"rest={lower_rest}, open={lower_open}",
    )
    ctx.check(
        "upper lid slides toward the center bay",
        upper_rest is not None
        and upper_open is not None
        and upper_open[0] < upper_rest[0] - 0.20,
        details=f"rest={upper_rest}, open={upper_open}",
    )

    hatch_closed_aabb = ctx.part_element_world_aabb(service_hatch, elem="hatch_front_bar")
    upper_front_aabb = ctx.part_element_world_aabb(upper_lid, elem="upper_lid_front_frame")
    with ctx.pose({hatch_hinge: 0.90}):
        hatch_open_aabb = ctx.part_element_world_aabb(service_hatch, elem="hatch_front_bar")

    hatch_closed_top = hatch_closed_aabb[1][2] if hatch_closed_aabb is not None else None
    hatch_open_top = hatch_open_aabb[1][2] if hatch_open_aabb is not None else None
    upper_front_top = upper_front_aabb[1][2] if upper_front_aabb is not None else None

    ctx.check(
        "service hatch sits nearly flush when closed",
        hatch_closed_top is not None
        and upper_front_top is not None
        and abs(hatch_closed_top - upper_front_top) <= 0.010,
        details=f"hatch_top={hatch_closed_top}, lid_top={upper_front_top}",
    )
    ctx.check(
        "service hatch lifts upward on its hinge",
        hatch_closed_top is not None
        and hatch_open_top is not None
        and hatch_open_top > hatch_closed_top + 0.06,
        details=f"closed_top={hatch_closed_top}, open_top={hatch_open_top}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
