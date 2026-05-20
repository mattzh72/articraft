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


def _add_vertical_knuckles(
    part,
    *,
    prefix: str,
    x: float,
    y: float,
    z_centers: tuple[float, ...],
    radius: float,
    length: float,
    material,
) -> None:
    for index, z_center in enumerate(z_centers):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=(x, y, z_center)),
            material=material,
            name=f"{prefix}_{index}",
        )


def _add_hinge_mount_tabs(
    part,
    *,
    prefix: str,
    x: float,
    y: float,
    z_centers: tuple[float, ...],
    size: tuple[float, float, float],
    material,
) -> None:
    for index, z_center in enumerate(z_centers):
        part.visual(
            Box(size),
            origin=Origin(xyz=(x, y, z_center)),
            material=material,
            name=f"{prefix}_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="control_cabinet")

    cabinet_body = model.material("cabinet_body", rgba=(0.79, 0.80, 0.82, 1.0))
    cabinet_frame = model.material("cabinet_frame", rgba=(0.66, 0.67, 0.69, 1.0))
    panel_blue = model.material("panel_blue", rgba=(0.34, 0.48, 0.63, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.16, 0.17, 0.18, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.56, 0.58, 0.60, 1.0))

    cabinet_width = 0.55
    cabinet_depth = 0.38
    cabinet_height = 1.70
    wall = 0.020
    outer_frame = 0.042
    front_frame_depth = 0.028
    return_depth = 0.034

    outer_opening_width = cabinet_width - (2.0 * outer_frame)
    outer_opening_height = cabinet_height - (2.0 * outer_frame)
    outer_door_width = outer_opening_width - 0.004
    outer_door_height = outer_opening_height - 0.006
    outer_door_thickness = 0.026
    outer_knuckle_radius = 0.0105
    outer_hinge_x = ((-cabinet_width * 0.5) + outer_frame) - outer_knuckle_radius
    outer_hinge_y = (cabinet_depth * 0.5) + 0.014
    outer_panel_y = ((cabinet_depth * 0.5) - outer_hinge_y) + (outer_door_thickness * 0.5)
    outer_panel_x = outer_knuckle_radius + (outer_door_width * 0.5)

    inner_return_jamb = 0.040
    inner_return_rail = 0.070
    inner_opening_width = outer_opening_width - (2.0 * inner_return_jamb)
    inner_opening_height = outer_opening_height - (2.0 * inner_return_rail)
    inner_door_width = inner_opening_width - 0.004
    inner_door_height = inner_opening_height - 0.006
    inner_door_thickness = 0.018
    inner_knuckle_radius = 0.0085
    return_frame_y = (cabinet_depth * 0.5) - front_frame_depth - (return_depth * 0.5)
    inner_hinge_x = (-0.5 * inner_opening_width) - inner_knuckle_radius
    inner_hinge_y = return_frame_y + 0.027
    inner_panel_y = -0.014
    inner_panel_x = inner_knuckle_radius + (inner_door_width * 0.5)

    carcass = model.part("carcass")
    carcass.visual(
        Box((wall, cabinet_depth, cabinet_height)),
        origin=Origin(xyz=((-cabinet_width * 0.5) + (wall * 0.5), 0.0, cabinet_height * 0.5)),
        material=cabinet_body,
        name="left_side",
    )
    carcass.visual(
        Box((wall, cabinet_depth, cabinet_height)),
        origin=Origin(xyz=((cabinet_width * 0.5) - (wall * 0.5), 0.0, cabinet_height * 0.5)),
        material=cabinet_body,
        name="right_side",
    )
    carcass.visual(
        Box((cabinet_width, cabinet_depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall * 0.5)),
        material=cabinet_body,
        name="bottom_pan",
    )
    carcass.visual(
        Box((cabinet_width, cabinet_depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, cabinet_height - (wall * 0.5))),
        material=cabinet_body,
        name="top_pan",
    )
    carcass.visual(
        Box((cabinet_width - (2.0 * wall), wall, cabinet_height - (2.0 * wall))),
        origin=Origin(
            xyz=(0.0, (-cabinet_depth * 0.5) + (wall * 0.5), cabinet_height * 0.5),
        ),
        material=cabinet_body,
        name="rear_panel",
    )
    carcass.visual(
        Box((cabinet_width * 0.88, cabinet_depth * 0.88, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=cabinet_frame,
        name="plinth_base",
    )
    carcass.visual(
        Box((outer_frame, front_frame_depth, cabinet_height)),
        origin=Origin(
            xyz=(
                (-cabinet_width * 0.5) + (outer_frame * 0.5),
                (cabinet_depth * 0.5) - (front_frame_depth * 0.5),
                cabinet_height * 0.5,
            )
        ),
        material=cabinet_frame,
        name="front_left_stile",
    )
    carcass.visual(
        Box((outer_frame, front_frame_depth, cabinet_height)),
        origin=Origin(
            xyz=(
                (cabinet_width * 0.5) - (outer_frame * 0.5),
                (cabinet_depth * 0.5) - (front_frame_depth * 0.5),
                cabinet_height * 0.5,
            )
        ),
        material=cabinet_frame,
        name="front_right_stile",
    )
    carcass.visual(
        Box((outer_opening_width, front_frame_depth, outer_frame)),
        origin=Origin(
            xyz=(
                0.0,
                (cabinet_depth * 0.5) - (front_frame_depth * 0.5),
                cabinet_height - (outer_frame * 0.5),
            )
        ),
        material=cabinet_frame,
        name="front_top_rail",
    )
    carcass.visual(
        Box((outer_opening_width, front_frame_depth, outer_frame)),
        origin=Origin(
            xyz=(0.0, (cabinet_depth * 0.5) - (front_frame_depth * 0.5), outer_frame * 0.5),
        ),
        material=cabinet_frame,
        name="front_bottom_rail",
    )
    carcass.visual(
        Box((inner_return_jamb, return_depth, inner_opening_height)),
        origin=Origin(
            xyz=(
                -0.5 * outer_opening_width + (inner_return_jamb * 0.5),
                return_frame_y,
                cabinet_height * 0.5,
            )
        ),
        material=cabinet_frame,
        name="inner_left_jamb",
    )
    carcass.visual(
        Box((inner_return_jamb, return_depth, inner_opening_height)),
        origin=Origin(
            xyz=(
                0.5 * outer_opening_width - (inner_return_jamb * 0.5),
                return_frame_y,
                cabinet_height * 0.5,
            )
        ),
        material=cabinet_frame,
        name="inner_right_jamb",
    )
    carcass.visual(
        Box((outer_opening_width, return_depth, inner_return_rail)),
        origin=Origin(
            xyz=(
                0.0,
                return_frame_y,
                cabinet_height - outer_frame - (inner_return_rail * 0.5),
            )
        ),
        material=cabinet_frame,
        name="inner_top_rail",
    )
    carcass.visual(
        Box((outer_opening_width, return_depth, inner_return_rail)),
        origin=Origin(
            xyz=(0.0, return_frame_y, outer_frame + (inner_return_rail * 0.5)),
        ),
        material=cabinet_frame,
        name="inner_bottom_rail",
    )
    carcass.visual(
        Box((cabinet_width - (2.0 * wall) + 0.004, 0.010, cabinet_height - 0.18)),
        origin=Origin(xyz=(0.0, -0.045, cabinet_height * 0.5)),
        material=panel_blue,
        name="equipment_backplate",
    )
    _add_vertical_knuckles(
        carcass,
        prefix="outer_frame_knuckle",
        x=outer_hinge_x,
        y=outer_hinge_y,
        z_centers=(cabinet_height * 0.5 - 0.28, cabinet_height * 0.5 + 0.28),
        radius=outer_knuckle_radius,
        length=0.22,
        material=hinge_metal,
    )
    _add_hinge_mount_tabs(
        carcass,
        prefix="outer_frame_hinge_leaf",
        x=outer_hinge_x,
        y=(outer_hinge_y + ((cabinet_depth * 0.5) - (front_frame_depth * 0.5))) * 0.5,
        z_centers=(cabinet_height * 0.5 - 0.28, cabinet_height * 0.5 + 0.28),
        size=(0.021, 0.028, 0.22),
        material=hinge_metal,
    )
    _add_vertical_knuckles(
        carcass,
        prefix="inner_frame_knuckle",
        x=inner_hinge_x,
        y=inner_hinge_y,
        z_centers=(cabinet_height * 0.5 - 0.245, cabinet_height * 0.5 + 0.245),
        radius=inner_knuckle_radius,
        length=0.19,
        material=hinge_metal,
    )
    _add_hinge_mount_tabs(
        carcass,
        prefix="inner_frame_hinge_leaf",
        x=inner_hinge_x,
        y=(inner_hinge_y + (return_frame_y + (return_depth * 0.5))) * 0.5,
        z_centers=(cabinet_height * 0.5 - 0.245, cabinet_height * 0.5 + 0.245),
        size=(0.018, 0.020, 0.19),
        material=hinge_metal,
    )
    carcass.inertial = Inertial.from_geometry(
        Box((cabinet_width, cabinet_depth, cabinet_height)),
        mass=92.0,
        origin=Origin(xyz=(0.0, 0.0, cabinet_height * 0.5)),
    )

    outer_door = model.part("outer_door")
    outer_door.visual(
        Box((outer_door_width, outer_door_thickness, outer_door_height)),
        origin=Origin(xyz=(outer_panel_x, outer_panel_y, 0.0)),
        material=cabinet_body,
        name="outer_door_panel",
    )
    outer_door.visual(
        Box((outer_door_width - 0.070, 0.012, outer_door_height - 0.120)),
        origin=Origin(xyz=(outer_panel_x, 0.002, 0.0)),
        material=cabinet_frame,
        name="outer_inner_stiffener",
    )
    outer_door.visual(
        Box((0.014, 0.016, 0.070)),
        origin=Origin(xyz=(outer_knuckle_radius + outer_door_width - 0.052, 0.020, -0.085)),
        material=dark_hardware,
        name="outer_handle_mount_lower",
    )
    outer_door.visual(
        Box((0.014, 0.016, 0.070)),
        origin=Origin(xyz=(outer_knuckle_radius + outer_door_width - 0.052, 0.020, 0.085)),
        material=dark_hardware,
        name="outer_handle_mount_upper",
    )
    outer_door.visual(
        Box((0.016, 0.012, 0.290)),
        origin=Origin(xyz=(outer_knuckle_radius + outer_door_width - 0.052, 0.034, 0.0)),
        material=dark_hardware,
        name="outer_pull_handle",
    )
    _add_vertical_knuckles(
        outer_door,
        prefix="outer_door_knuckle",
        x=0.0,
        y=0.0,
        z_centers=(-0.56, 0.0, 0.56),
        radius=outer_knuckle_radius,
        length=0.22,
        material=hinge_metal,
    )
    outer_door.inertial = Inertial.from_geometry(
        Box((outer_door_width, 0.040, outer_door_height)),
        mass=14.0,
        origin=Origin(xyz=(outer_panel_x, outer_panel_y, 0.0)),
    )

    inner_door = model.part("inner_door")
    inner_door.visual(
        Box((inner_door_width, inner_door_thickness, inner_door_height)),
        origin=Origin(xyz=(inner_panel_x, inner_panel_y, 0.0)),
        material=panel_blue,
        name="inner_door_panel",
    )
    inner_door.visual(
        Box((inner_door_width - 0.050, 0.010, inner_door_height - 0.100)),
        origin=Origin(xyz=(inner_panel_x, -0.019, 0.0)),
        material=cabinet_frame,
        name="inner_panel_stiffener",
    )
    inner_door.visual(
        Box((0.014, 0.006, 0.050)),
        origin=Origin(xyz=(inner_knuckle_radius + inner_door_width - 0.048, -0.002, -0.060)),
        material=dark_hardware,
        name="inner_handle_mount_lower",
    )
    inner_door.visual(
        Box((0.014, 0.006, 0.050)),
        origin=Origin(xyz=(inner_knuckle_radius + inner_door_width - 0.048, -0.002, 0.060)),
        material=dark_hardware,
        name="inner_handle_mount_upper",
    )
    inner_door.visual(
        Box((0.014, 0.004, 0.120)),
        origin=Origin(xyz=(inner_knuckle_radius + inner_door_width - 0.048, 0.002, 0.0)),
        material=dark_hardware,
        name="inner_pull_handle",
    )
    _add_vertical_knuckles(
        inner_door,
        prefix="inner_door_knuckle",
        x=0.0,
        y=0.0,
        z_centers=(-0.49, 0.0, 0.49),
        radius=inner_knuckle_radius,
        length=0.19,
        material=hinge_metal,
    )
    _add_hinge_mount_tabs(
        inner_door,
        prefix="inner_door_hinge_leaf",
        x=0.010,
        y=-0.004,
        z_centers=(-0.49, 0.0, 0.49),
        size=(0.020, 0.010, 0.19),
        material=hinge_metal,
    )
    inner_door.inertial = Inertial.from_geometry(
        Box((inner_door_width, 0.032, inner_door_height)),
        mass=8.0,
        origin=Origin(xyz=(inner_panel_x, inner_panel_y, 0.0)),
    )

    model.articulation(
        "outer_door_hinge",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=outer_door,
        origin=Origin(xyz=(outer_hinge_x, outer_hinge_y, cabinet_height * 0.5)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.6,
            lower=0.0,
            upper=2.35,
        ),
    )
    model.articulation(
        "inner_door_hinge",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=inner_door,
        origin=Origin(xyz=(inner_hinge_x, inner_hinge_y, cabinet_height * 0.5)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.8,
            lower=0.0,
            upper=2.10,
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

    carcass = object_model.get_part("carcass")
    outer_door = object_model.get_part("outer_door")
    inner_door = object_model.get_part("inner_door")
    outer_hinge = object_model.get_articulation("outer_door_hinge")
    inner_hinge = object_model.get_articulation("inner_door_hinge")

    ctx.expect_gap(
        outer_door,
        inner_door,
        axis="y",
        min_gap=0.020,
        positive_elem="outer_door_panel",
        negative_elem="inner_door_panel",
        name="outer door closes ahead of the inner sub-door",
    )
    ctx.expect_within(
        inner_door,
        outer_door,
        axes="xz",
        inner_elem="inner_door_panel",
        outer_elem="outer_door_panel",
        margin=0.0,
        name="inner sub-door stays nested inside the outer door footprint",
    )
    ctx.expect_overlap(
        outer_door,
        carcass,
        axes="z",
        elem_a="outer_door_panel",
        elem_b="front_left_stile",
        min_overlap=1.20,
        name="outer door spans the cabinet opening height",
    )
    ctx.expect_overlap(
        inner_door,
        carcass,
        axes="z",
        elem_a="inner_door_panel",
        elem_b="inner_left_jamb",
        min_overlap=1.10,
        name="inner sub-door spans the inner equipment frame height",
    )

    outer_closed = ctx.part_element_world_aabb(outer_door, elem="outer_door_panel")
    with ctx.pose({outer_hinge: 1.10}):
        outer_open = ctx.part_element_world_aabb(outer_door, elem="outer_door_panel")
    ctx.check(
        "outer door swings outward from the left hinge line",
        outer_closed is not None
        and outer_open is not None
        and outer_open[1][1] > outer_closed[1][1] + 0.18,
        details=f"closed={outer_closed}, open={outer_open}",
    )

    with ctx.pose({outer_hinge: 1.20, inner_hinge: 0.0}):
        inner_closed = ctx.part_element_world_aabb(inner_door, elem="inner_door_panel")
    with ctx.pose({outer_hinge: 1.20, inner_hinge: 0.95}):
        inner_open = ctx.part_element_world_aabb(inner_door, elem="inner_door_panel")
    ctx.check(
        "inner sub-door swings outward on its own hinge set",
        inner_closed is not None
        and inner_open is not None
        and inner_open[1][1] > inner_closed[1][1] + 0.12,
        details=f"closed={inner_closed}, open={inner_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
