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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_bar(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _build_lid(
    model: ArticulatedObject,
    *,
    name: str,
    panel_length: float,
    panel_width: float,
    glass_length: float,
    glass_width: float,
    rail_width: float,
    shoe_width: float,
    handle_x: float,
    frame_color,
    glass_color,
) -> object:
    lid = model.part(name)

    side_y = panel_width * 0.5 - rail_width * 0.5
    end_x = panel_length * 0.5 - rail_width * 0.5

    lid.visual(
        Box((panel_length, rail_width, 0.012)),
        origin=Origin(xyz=(0.0, side_y, 0.010)),
        material=frame_color,
        name="right_side_rail",
    )
    lid.visual(
        Box((panel_length, rail_width, 0.012)),
        origin=Origin(xyz=(0.0, -side_y, 0.010)),
        material=frame_color,
        name="left_side_rail",
    )
    lid.visual(
        Box((rail_width, panel_width - 2.0 * rail_width, 0.012)),
        origin=Origin(xyz=(end_x, 0.0, 0.010)),
        material=frame_color,
        name="front_cross_rail",
    )
    lid.visual(
        Box((rail_width, panel_width - 2.0 * rail_width, 0.012)),
        origin=Origin(xyz=(-end_x, 0.0, 0.010)),
        material=frame_color,
        name="rear_cross_rail",
    )
    lid.visual(
        Box((glass_length, glass_width, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=glass_color,
        name="glass_panel",
    )
    lid.visual(
        Box((0.160, 0.050, 0.006)),
        origin=Origin(xyz=(handle_x, 0.0, 0.019)),
        material=frame_color,
        name="pull_grip",
    )

    shoe_x = panel_length * 0.5 - 0.090
    shoe_y = panel_width * 0.5 - shoe_width * 0.5
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            lid.visual(
                Box((0.040, shoe_width, 0.004)),
                origin=Origin(xyz=(x_sign * shoe_x, y_sign * shoe_y, 0.002)),
                material=frame_color,
                name=f"shoe_{'p' if x_sign > 0 else 'n'}x_{'p' if y_sign > 0 else 'n'}y",
            )

    return lid


def _build_caster_swivel(
    model: ArticulatedObject,
    *,
    name: str,
    metal,
) -> object:
    caster = model.part(name)
    caster.visual(
        Box((0.090, 0.060, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=metal,
        name="top_plate",
    )
    caster.visual(
        Cylinder(radius=0.016, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.013)),
        material=metal,
        name="swivel_stem",
    )
    caster.visual(
        Box((0.022, 0.022, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.022)),
        material=metal,
        name="stem_neck",
    )
    caster.visual(
        Box((0.040, 0.032, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.028)),
        material=metal,
        name="fork_crown",
    )
    caster.visual(
        Box((0.006, 0.024, 0.046)),
        origin=Origin(xyz=(0.017, 0.0, -0.055)),
        material=metal,
        name="right_fork_plate",
    )
    caster.visual(
        Box((0.006, 0.024, 0.046)),
        origin=Origin(xyz=(-0.017, 0.0, -0.055)),
        material=metal,
        name="left_fork_plate",
    )
    return caster


def _build_caster_wheel(
    model: ArticulatedObject,
    *,
    name: str,
    rubber,
    hub_metal,
) -> object:
    wheel = model.part(name)
    wheel.visual(
        Cylinder(radius=0.034, length=0.022),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="tire",
    )
    wheel.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_metal,
        name="hub",
    )
    wheel.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.011, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_metal,
        name="right_hub_cap",
    )
    wheel.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(-0.011, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_metal,
        name="left_hub_cap",
    )
    return wheel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="island_display_freezer")

    cabinet_white = model.material("cabinet_white", rgba=(0.94, 0.95, 0.96, 1.0))
    liner_white = model.material("liner_white", rgba=(0.98, 0.98, 0.99, 1.0))
    aluminum = model.material("aluminum", rgba=(0.69, 0.72, 0.75, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.25, 0.27, 0.30, 1.0))
    glass = model.material("glass", rgba=(0.70, 0.86, 0.93, 0.28))
    basket_wire = model.material("basket_wire", rgba=(0.72, 0.74, 0.76, 1.0))
    caster_rubber = model.material("caster_rubber", rgba=(0.12, 0.12, 0.13, 1.0))
    caster_metal = model.material("caster_metal", rgba=(0.55, 0.57, 0.60, 1.0))

    body_length = 1.72
    body_width = 0.92
    body_bottom = 0.09
    plinth_height = 0.10
    plinth_length = 1.56
    plinth_width = 0.78
    shell_top = 0.79
    rim_top = 0.84
    outer_wall_height = shell_top - (body_bottom + plinth_height)
    side_wall_thickness = 0.055
    end_wall_thickness = 0.045
    open_length = 1.56
    open_width = 0.66
    liner_thickness = 0.006
    cavity_bottom = 0.24
    liner_height = 0.80 - cavity_bottom

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((plinth_length, plinth_width, plinth_height)),
        origin=Origin(xyz=(0.0, 0.0, body_bottom + plinth_height * 0.5)),
        material=dark_trim,
        name="base_plinth",
    )
    cabinet.visual(
        Box((body_length - 0.04, 0.070, 0.018)),
        origin=Origin(
            xyz=(0.0, body_width * 0.5 - 0.080, body_bottom + plinth_height + 0.009)
        ),
        material=dark_trim,
        name="right_kick_trim",
    )
    cabinet.visual(
        Box((body_length - 0.04, 0.070, 0.018)),
        origin=Origin(
            xyz=(0.0, -body_width * 0.5 + 0.080, body_bottom + plinth_height + 0.009)
        ),
        material=dark_trim,
        name="left_kick_trim",
    )

    shell_center_z = body_bottom + plinth_height + outer_wall_height * 0.5
    cabinet.visual(
        Box((body_length, side_wall_thickness, outer_wall_height)),
        origin=Origin(
            xyz=(0.0, body_width * 0.5 - side_wall_thickness * 0.5, shell_center_z)
        ),
        material=cabinet_white,
        name="right_outer_wall",
    )
    cabinet.visual(
        Box((body_length, side_wall_thickness, outer_wall_height)),
        origin=Origin(
            xyz=(0.0, -body_width * 0.5 + side_wall_thickness * 0.5, shell_center_z)
        ),
        material=cabinet_white,
        name="left_outer_wall",
    )
    cabinet.visual(
        Box((end_wall_thickness, body_width - 2.0 * side_wall_thickness, outer_wall_height)),
        origin=Origin(
            xyz=(body_length * 0.5 - end_wall_thickness * 0.5, 0.0, shell_center_z)
        ),
        material=cabinet_white,
        name="front_outer_end_wall",
    )
    cabinet.visual(
        Box((end_wall_thickness, body_width - 2.0 * side_wall_thickness, outer_wall_height)),
        origin=Origin(
            xyz=(-body_length * 0.5 + end_wall_thickness * 0.5, 0.0, shell_center_z)
        ),
        material=cabinet_white,
        name="rear_outer_end_wall",
    )

    rim_height = rim_top - shell_top
    cabinet.visual(
        Box((body_length, 0.120, rim_height)),
        origin=Origin(xyz=(0.0, body_width * 0.5 - 0.060, shell_top + rim_height * 0.5)),
        material=aluminum,
        name="right_top_rim",
    )
    cabinet.visual(
        Box((body_length, 0.120, rim_height)),
        origin=Origin(xyz=(0.0, -body_width * 0.5 + 0.060, shell_top + rim_height * 0.5)),
        material=aluminum,
        name="left_top_rim",
    )
    cabinet.visual(
        Box((0.120, open_width + 0.080, rim_height)),
        origin=Origin(
            xyz=(body_length * 0.5 - 0.060, 0.0, shell_top + rim_height * 0.5)
        ),
        material=aluminum,
        name="front_top_rim",
    )
    cabinet.visual(
        Box((0.120, open_width + 0.080, rim_height)),
        origin=Origin(
            xyz=(-body_length * 0.5 + 0.060, 0.0, shell_top + rim_height * 0.5)
        ),
        material=aluminum,
        name="rear_top_rim",
    )

    liner_center_z = cavity_bottom + liner_height * 0.5
    cabinet.visual(
        Box((open_length, liner_thickness, liner_height)),
        origin=Origin(xyz=(0.0, open_width * 0.5 - liner_thickness * 0.5, liner_center_z)),
        material=liner_white,
        name="right_inner_liner",
    )
    cabinet.visual(
        Box((open_length, liner_thickness, liner_height)),
        origin=Origin(xyz=(0.0, -open_width * 0.5 + liner_thickness * 0.5, liner_center_z)),
        material=liner_white,
        name="left_inner_liner",
    )
    cabinet.visual(
        Box((liner_thickness, open_width, liner_height)),
        origin=Origin(xyz=(open_length * 0.5 - liner_thickness * 0.5, 0.0, liner_center_z)),
        material=liner_white,
        name="front_inner_liner",
    )
    cabinet.visual(
        Box((liner_thickness, open_width, liner_height)),
        origin=Origin(xyz=(-open_length * 0.5 + liner_thickness * 0.5, 0.0, liner_center_z)),
        material=liner_white,
        name="rear_inner_liner",
    )
    cabinet.visual(
        Box((open_length, open_width, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, cavity_bottom - 0.014)),
        material=liner_white,
        name="liner_floor",
    )
    cabinet.visual(
        Box((open_length, 0.022, 0.016)),
        origin=Origin(xyz=(0.0, 0.319, 0.792)),
        material=aluminum,
        name="right_basket_ledge",
    )
    cabinet.visual(
        Box((open_length, 0.022, 0.016)),
        origin=Origin(xyz=(0.0, -0.319, 0.792)),
        material=aluminum,
        name="left_basket_ledge",
    )
    cabinet.visual(
        Box((body_length - 0.12, 0.022, 0.004)),
        origin=Origin(xyz=(0.0, 0.378, 0.842)),
        material=dark_trim,
        name="right_lower_track",
    )
    cabinet.visual(
        Box((body_length - 0.12, 0.022, 0.004)),
        origin=Origin(xyz=(0.0, -0.378, 0.842)),
        material=dark_trim,
        name="left_lower_track",
    )
    cabinet.visual(
        Box((body_length - 0.10, 0.020, 0.020)),
        origin=Origin(xyz=(0.0, 0.410, 0.850)),
        material=aluminum,
        name="right_upper_track",
    )
    cabinet.visual(
        Box((body_length - 0.10, 0.020, 0.020)),
        origin=Origin(xyz=(0.0, -0.410, 0.850)),
        material=aluminum,
        name="left_upper_track",
    )

    basket_insert = model.part("basket_insert")
    basket_top_z = 0.805
    basket_bottom_z = 0.315
    basket_half_width = 0.290
    basket_end_x = 0.735
    inner_divider_xs = (-0.245, 0.245)

    basket_insert.visual(
        Box((1.470, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.319, basket_top_z)),
        material=basket_wire,
        name="right_hanger_rail",
    )
    basket_insert.visual(
        Box((1.470, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -0.319, basket_top_z)),
        material=basket_wire,
        name="left_hanger_rail",
    )
    for frame_x, frame_name in (
        (-basket_end_x, "rear_end_frame"),
        (*inner_divider_xs[:1], "rear_divider_frame"),
        (*inner_divider_xs[1:], "front_divider_frame"),
        (basket_end_x, "front_end_frame"),
    ):
        _add_bar(
            basket_insert,
            (frame_x, -basket_half_width, basket_top_z),
            (frame_x, basket_half_width, basket_top_z),
            radius=0.0032,
            material=basket_wire,
            name=f"{frame_name}_top",
        )
        _add_bar(
            basket_insert,
            (frame_x, -basket_half_width, basket_bottom_z),
            (frame_x, basket_half_width, basket_bottom_z),
            radius=0.0030,
            material=basket_wire,
            name=f"{frame_name}_bottom",
        )
        _add_bar(
            basket_insert,
            (frame_x, -basket_half_width, basket_bottom_z),
            (frame_x, -basket_half_width, basket_top_z),
            radius=0.0030,
            material=basket_wire,
            name=f"{frame_name}_left_upright",
        )
        _add_bar(
            basket_insert,
            (frame_x, basket_half_width, basket_bottom_z),
            (frame_x, basket_half_width, basket_top_z),
            radius=0.0030,
            material=basket_wire,
            name=f"{frame_name}_right_upright",
        )
        _add_bar(
            basket_insert,
            (frame_x, basket_half_width, basket_top_z),
            (frame_x, 0.319, basket_top_z),
            radius=0.0030,
            material=basket_wire,
            name=f"{frame_name}_right_hook",
        )
        _add_bar(
            basket_insert,
            (frame_x, -basket_half_width, basket_top_z),
            (frame_x, -0.319, basket_top_z),
            radius=0.0030,
            material=basket_wire,
            name=f"{frame_name}_left_hook",
        )
        for wire_y in (-0.180, -0.060, 0.060, 0.180):
            _add_bar(
                basket_insert,
                (frame_x, wire_y, basket_bottom_z),
                (frame_x, wire_y, basket_top_z),
                radius=0.0019,
                material=basket_wire,
                name=f"{frame_name}_wire_{wire_y:+.2f}",
            )

    for bottom_y, suffix in ((-0.205, "left_bottom_runner"), (0.205, "right_bottom_runner")):
        _add_bar(
            basket_insert,
            (-basket_end_x, bottom_y, basket_bottom_z),
            (basket_end_x, bottom_y, basket_bottom_z),
            radius=0.0026,
            material=basket_wire,
            name=suffix,
        )

    for top_y, suffix in ((-0.180, "left_mid_runner"), (0.180, "right_mid_runner")):
        _add_bar(
            basket_insert,
            (-basket_end_x, top_y, 0.560),
            (basket_end_x, top_y, 0.560),
            radius=0.0022,
            material=basket_wire,
            name=suffix,
        )

    left_lid = _build_lid(
        model,
        name="left_lid",
        panel_length=0.88,
        panel_width=0.78,
        glass_length=0.832,
        glass_width=0.732,
        rail_width=0.024,
        shoe_width=0.014,
        handle_x=-0.360,
        frame_color=aluminum,
        glass_color=glass,
    )
    right_lid = _build_lid(
        model,
        name="right_lid",
        panel_length=0.88,
        panel_width=0.844,
        glass_length=0.832,
        glass_width=0.796,
        rail_width=0.024,
        shoe_width=0.014,
        handle_x=0.360,
        frame_color=aluminum,
        glass_color=glass,
    )

    model.articulation(
        "cabinet_to_basket_insert",
        ArticulationType.FIXED,
        parent=cabinet,
        child=basket_insert,
        origin=Origin(),
    )
    model.articulation(
        "cabinet_to_left_lid",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=left_lid,
        origin=Origin(xyz=(-0.340, 0.0, 0.844)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.25, lower=0.0, upper=0.44),
    )
    model.articulation(
        "cabinet_to_right_lid",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=right_lid,
        origin=Origin(xyz=(0.340, 0.0, 0.860)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.25, lower=0.0, upper=0.44),
    )

    caster_locations = {
        "front_left": (0.620, 0.300),
        "front_right": (0.620, -0.300),
        "rear_left": (-0.620, 0.300),
        "rear_right": (-0.620, -0.300),
    }
    for prefix, (cx, cy) in caster_locations.items():
        swivel = _build_caster_swivel(
            model,
            name=f"{prefix}_caster_swivel",
            metal=caster_metal,
        )
        wheel = _build_caster_wheel(
            model,
            name=f"{prefix}_wheel",
            rubber=caster_rubber,
            hub_metal=caster_metal,
        )
        model.articulation(
            f"cabinet_to_{prefix}_caster_swivel",
            ArticulationType.CONTINUOUS,
            parent=cabinet,
            child=swivel,
            origin=Origin(xyz=(cx, cy, body_bottom)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=12.0, velocity=4.0),
        )
        model.articulation(
            f"{prefix}_caster_swivel_to_{prefix}_wheel",
            ArticulationType.CONTINUOUS,
            parent=swivel,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.066)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=10.0, velocity=20.0),
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
    basket_insert = object_model.get_part("basket_insert")
    left_lid = object_model.get_part("left_lid")
    right_lid = object_model.get_part("right_lid")

    left_lid_slide = object_model.get_articulation("cabinet_to_left_lid")
    right_lid_slide = object_model.get_articulation("cabinet_to_right_lid")

    front_left_swivel = object_model.get_articulation("cabinet_to_front_left_caster_swivel")
    front_left_spin = object_model.get_articulation(
        "front_left_caster_swivel_to_front_left_wheel"
    )
    rear_right_swivel = object_model.get_articulation("cabinet_to_rear_right_caster_swivel")
    rear_right_spin = object_model.get_articulation("rear_right_caster_swivel_to_rear_right_wheel")

    front_left_caster = object_model.get_part("front_left_caster_swivel")
    front_left_wheel = object_model.get_part("front_left_wheel")
    rear_right_caster = object_model.get_part("rear_right_caster_swivel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")

    ctx.expect_contact(
        basket_insert,
        cabinet,
        contact_tol=0.001,
        name="basket insert hangs from the freezer ledges",
    )
    ctx.expect_contact(
        left_lid,
        cabinet,
        contact_tol=0.001,
        name="left lid is supported on the lower rim track",
    )
    ctx.expect_contact(
        right_lid,
        cabinet,
        contact_tol=0.001,
        name="right lid is supported on the upper rim track",
    )
    ctx.expect_contact(
        front_left_caster,
        cabinet,
        contact_tol=0.001,
        name="front-left caster swivel mounts to the cabinet base",
    )
    ctx.expect_contact(
        front_left_wheel,
        front_left_caster,
        contact_tol=0.001,
        name="front-left wheel sits between the caster fork plates",
    )
    ctx.expect_contact(
        rear_right_caster,
        cabinet,
        contact_tol=0.001,
        name="rear-right caster swivel mounts to the cabinet base",
    )
    ctx.expect_contact(
        rear_right_wheel,
        rear_right_caster,
        contact_tol=0.001,
        name="rear-right wheel sits between the caster fork plates",
    )

    left_rest = ctx.part_world_position(left_lid)
    right_rest = ctx.part_world_position(right_lid)
    with ctx.pose({left_lid_slide: 0.40, right_lid_slide: 0.40}):
        left_open = ctx.part_world_position(left_lid)
        right_open = ctx.part_world_position(right_lid)

    ctx.check(
        "left lid slides toward +X when opened",
        left_rest is not None
        and left_open is not None
        and left_open[0] > left_rest[0] + 0.30,
        details=f"rest={left_rest}, open={left_open}",
    )
    ctx.check(
        "right lid slides toward -X when opened",
        right_rest is not None
        and right_open is not None
        and right_open[0] < right_rest[0] - 0.30,
        details=f"rest={right_rest}, open={right_open}",
    )

    ctx.check(
        "caster swivel joints are vertical",
        front_left_swivel.axis == (0.0, 0.0, 1.0) and rear_right_swivel.axis == (0.0, 0.0, 1.0),
        details=f"front_left={front_left_swivel.axis}, rear_right={rear_right_swivel.axis}",
    )
    ctx.check(
        "caster wheel joints spin on horizontal axles",
        front_left_spin.axis == (1.0, 0.0, 0.0) and rear_right_spin.axis == (1.0, 0.0, 0.0),
        details=f"front_left={front_left_spin.axis}, rear_right={rear_right_spin.axis}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
