from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _hollow_stile_mesh(
    *,
    outer_width: float,
    outer_depth: float,
    wall: float,
    length: float,
    name: str,
):
    corner_radius = min(outer_width, outer_depth) * 0.22
    inner_width = outer_width - 2.0 * wall
    inner_depth = outer_depth - 2.0 * wall
    inner_radius = max(corner_radius - wall * 0.8, 0.001)
    geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(
            outer_width,
            outer_depth,
            corner_radius,
            corner_segments=8,
        ),
        [
            rounded_rect_profile(
                inner_width,
                inner_depth,
                inner_radius,
                corner_segments=8,
            )
        ],
        length,
        center=True,
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(geom, name)


def _stile_x_positions(overall_width: float, stile_width: float) -> tuple[float, float]:
    half_spacing = 0.5 * (overall_width - stile_width)
    return (-half_spacing, half_spacing)


def _add_section(
    part,
    *,
    prefix: str,
    length: float,
    bottom_z: float,
    overall_width: float,
    stile_width: float,
    stile_depth: float,
    wall: float,
    rung_count: int,
    rung_pitch: float,
    rung_start: float,
    aluminium,
    rung_material,
    rail_material,
    shoe_material,
    rail_depth: float | None = None,
    rail_length: float | None = None,
    rail_z: float | None = None,
    rail_width: float | None = None,
    shoe_depth: float | None = None,
    shoe_width: float | None = None,
    shoe_height: float | None = None,
    shoe_z_positions: tuple[float, ...] = (),
    add_feet: bool = False,
    foot_material=None,
) -> None:
    stile_mesh = _hollow_stile_mesh(
        outer_width=stile_width,
        outer_depth=stile_depth,
        wall=wall,
        length=length,
        name=f"{prefix}_stile_mesh",
    )
    stile_center_z = bottom_z + 0.5 * length
    stile_xs = _stile_x_positions(overall_width, stile_width)

    for side_name, x in zip(("left", "right"), stile_xs):
        part.visual(
            stile_mesh,
            origin=Origin(xyz=(x, 0.0, stile_center_z)),
            material=aluminium,
            name=f"{prefix}_{side_name}_stile",
        )

    rung_span = overall_width - 1.28 * stile_width
    rung_depth = max(stile_depth - 0.004, 0.018)
    rung_height = 0.024
    rung_y = 0.002
    for rung_index in range(rung_count):
        rung_z = bottom_z + rung_start + rung_index * rung_pitch
        part.visual(
            Box((rung_span, rung_depth, rung_height)),
            origin=Origin(xyz=(0.0, rung_y, rung_z)),
            material=rung_material,
            name=f"{prefix}_rung_{rung_index + 1}",
        )

    if rail_depth and rail_length and rail_z and rail_width:
        rail_y = 0.5 * stile_depth + 0.5 * rail_depth - 0.001
        for side_name, x in zip(("left", "right"), stile_xs):
            rail_x = math.copysign(abs(x) - 0.17 * stile_width, x)
            part.visual(
                Box((rail_width, rail_depth, rail_length)),
                origin=Origin(xyz=(rail_x, rail_y, rail_z)),
                material=rail_material,
                name=f"{prefix}_{side_name}_guide_rail",
            )

    if shoe_depth and shoe_width and shoe_height and shoe_z_positions:
        shoe_y = -(0.5 * stile_depth + 0.5 * shoe_depth - 0.001)
        for side_name, x in zip(("left", "right"), stile_xs):
            shoe_x = math.copysign(abs(x) - 0.16 * stile_width, x)
            for shoe_index, shoe_z in enumerate(shoe_z_positions):
                part.visual(
                    Box((shoe_width, shoe_depth, shoe_height)),
                    origin=Origin(xyz=(shoe_x, shoe_y, shoe_z)),
                    material=shoe_material,
                    name=f"{prefix}_{side_name}_guide_shoe_{shoe_index + 1}",
                )

    if add_feet and foot_material is not None:
        for side_name, x in zip(("left", "right"), stile_xs):
            part.visual(
                Box((0.090, 0.058, 0.030)),
                origin=Origin(xyz=(x, 0.010, bottom_z + 0.015)),
                material=foot_material,
                name=f"{prefix}_{side_name}_foot",
            )
            part.visual(
                Box((0.075, 0.018, 0.060)),
                origin=Origin(xyz=(x, 0.022, bottom_z + 0.090)),
                material=foot_material,
                name=f"{prefix}_{side_name}_heel_pad",
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_section_extension_ladder")

    aluminium = model.material("aluminium", rgba=(0.80, 0.82, 0.84, 1.0))
    rung_material = model.material("rung_aluminium", rgba=(0.72, 0.74, 0.77, 1.0))
    shoe_material = model.material("guide_polymer", rgba=(0.20, 0.22, 0.24, 1.0))
    rail_material = model.material("rail_aluminium", rgba=(0.66, 0.68, 0.71, 1.0))
    foot_material = model.material("rubber", rgba=(0.11, 0.11, 0.12, 1.0))

    base = model.part("base_section")
    _add_section(
        base,
        prefix="base",
        length=3.60,
        bottom_z=0.0,
        overall_width=0.500,
        stile_width=0.073,
        stile_depth=0.034,
        wall=0.0045,
        rung_count=12,
        rung_pitch=0.280,
        rung_start=0.380,
        aluminium=aluminium,
        rung_material=rung_material,
        rail_material=rail_material,
        shoe_material=shoe_material,
        rail_depth=0.010,
        rail_length=2.25,
        rail_z=1.72,
        rail_width=0.018,
        add_feet=True,
        foot_material=foot_material,
    )
    base.visual(
        Box((0.500, 0.020, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 3.585)),
        material=rail_material,
        name="base_top_cap",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.540, 0.120, 3.66)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.020, 1.83)),
    )

    middle = model.part("middle_section")
    _add_section(
        middle,
        prefix="middle",
        length=3.50,
        bottom_z=-0.42,
        overall_width=0.452,
        stile_width=0.064,
        stile_depth=0.030,
        wall=0.0040,
        rung_count=12,
        rung_pitch=0.280,
        rung_start=0.360,
        aluminium=aluminium,
        rung_material=rung_material,
        rail_material=rail_material,
        shoe_material=shoe_material,
        rail_depth=0.009,
        rail_length=2.16,
        rail_z=1.58,
        rail_width=0.016,
        shoe_depth=0.009,
        shoe_width=0.022,
        shoe_height=0.180,
        shoe_z_positions=(0.06, 2.42),
    )
    middle.visual(
        Box((0.452, 0.018, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 3.067)),
        material=rail_material,
        name="middle_top_cap",
    )
    middle.inertial = Inertial.from_geometry(
        Box((0.490, 0.100, 3.56)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, 1.33)),
    )

    fly = model.part("fly_section")
    _add_section(
        fly,
        prefix="fly",
        length=3.38,
        bottom_z=-0.38,
        overall_width=0.406,
        stile_width=0.056,
        stile_depth=0.026,
        wall=0.0038,
        rung_count=11,
        rung_pitch=0.280,
        rung_start=0.360,
        aluminium=aluminium,
        rung_material=rung_material,
        rail_material=rail_material,
        shoe_material=shoe_material,
        shoe_depth=0.008,
        shoe_width=0.020,
        shoe_height=0.170,
        shoe_z_positions=(0.08, 2.30),
    )
    fly.visual(
        Box((0.406, 0.016, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 2.988)),
        material=rail_material,
        name="fly_head_cap",
    )
    for side_name, x in zip(("left", "right"), _stile_x_positions(0.406, 0.056)):
        fly.visual(
            Box((0.074, 0.034, 0.120)),
            origin=Origin(xyz=(x, 0.012, 2.920)),
            material=foot_material,
            name=f"fly_{side_name}_wall_bumper",
        )
    fly.inertial = Inertial.from_geometry(
        Box((0.440, 0.090, 3.42)),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.0, 1.31)),
    )

    model.articulation(
        "base_to_middle",
        ArticulationType.PRISMATIC,
        parent=base,
        child=middle,
        origin=Origin(xyz=(0.0, 0.049, 0.72)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.45,
            lower=0.0,
            upper=1.65,
        ),
    )
    model.articulation(
        "middle_to_fly",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=fly,
        origin=Origin(xyz=(0.0, 0.043, 0.66)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=150.0,
            velocity=0.45,
            lower=0.0,
            upper=1.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_section")
    middle = object_model.get_part("middle_section")
    fly = object_model.get_part("fly_section")
    base_to_middle = object_model.get_articulation("base_to_middle")
    middle_to_fly = object_model.get_articulation("middle_to_fly")
    base_left_rail = base.get_visual("base_left_guide_rail")
    middle_left_shoe = middle.get_visual("middle_left_guide_shoe_1")
    middle_left_rail = middle.get_visual("middle_left_guide_rail")
    fly_left_shoe = fly.get_visual("fly_left_guide_shoe_1")

    ctx.check("base section exists", base is not None)
    ctx.check("middle section exists", middle is not None)
    ctx.check("fly section exists", fly is not None)

    ctx.expect_contact(
        base,
        middle,
        elem_a=base_left_rail,
        elem_b=middle_left_shoe,
        contact_tol=0.0005,
        name="middle guide shoe bears on base rail",
    )
    ctx.expect_contact(
        middle,
        fly,
        elem_a=middle_left_rail,
        elem_b=fly_left_shoe,
        contact_tol=0.0005,
        name="fly guide shoe bears on middle rail",
    )
    ctx.expect_overlap(
        base,
        middle,
        axes="x",
        min_overlap=0.400,
        name="middle remains centered between base stiles",
    )
    ctx.expect_overlap(
        middle,
        fly,
        axes="x",
        min_overlap=0.350,
        name="fly remains centered between middle stiles",
    )
    ctx.expect_overlap(
        base,
        middle,
        axes="z",
        min_overlap=2.0,
        name="middle retains substantial insertion in base at rest",
    )
    ctx.expect_overlap(
        middle,
        fly,
        axes="z",
        min_overlap=2.2,
        name="fly retains substantial insertion in middle at rest",
    )

    rest_middle = ctx.part_world_position(middle)
    rest_fly = ctx.part_world_position(fly)
    with ctx.pose({base_to_middle: 1.65, middle_to_fly: 1.55}):
        ctx.expect_gap(
            fly,
            base,
            axis="y",
            min_gap=0.030,
            max_gap=0.140,
            name="nested sections stack progressively forward when extended",
        )
        ctx.expect_contact(
            base,
            middle,
            elem_a=base_left_rail,
            elem_b=middle_left_shoe,
            contact_tol=0.0005,
            name="middle stays guided on base rail when extended",
        )
        ctx.expect_contact(
            middle,
            fly,
            elem_a=middle_left_rail,
            elem_b=fly_left_shoe,
            contact_tol=0.0005,
            name="fly stays guided on middle rail when extended",
        )
        ctx.expect_overlap(
            base,
            middle,
            axes="z",
            min_overlap=1.40,
            name="middle keeps retained insertion at full base extension",
        )
        ctx.expect_overlap(
            middle,
            fly,
            axes="z",
            min_overlap=1.10,
            name="fly keeps retained insertion at full ladder extension",
        )
        extended_middle = ctx.part_world_position(middle)
        extended_fly = ctx.part_world_position(fly)

    ctx.check(
        "middle section extends upward",
        rest_middle is not None
        and extended_middle is not None
        and extended_middle[2] > rest_middle[2] + 1.50,
        details=f"rest={rest_middle}, extended={extended_middle}",
    )
    ctx.check(
        "fly section extends upward",
        rest_fly is not None
        and extended_fly is not None
        and extended_fly[2] > rest_fly[2] + 3.00,
        details=f"rest={rest_fly}, extended={extended_fly}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
