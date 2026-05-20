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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


SECTION_NAMES = (
    "base_section",
    "section_2",
    "section_3",
    "section_4",
    "section_5",
)
SLIDE_NAMES = (
    "base_to_section_2",
    "section_2_to_section_3",
    "section_3_to_section_4",
    "section_4_to_section_5",
)
SECTION_HEIGHT = 0.92
COLLAPSED_OFFSET = 0.07
EXTENSION_TRAVEL = 0.45
CENTER_SPACING = 0.39
RUNG_ZS = (0.18, 0.44)
COLLAR_HEIGHT = 0.05
STILE_WIDTHS = (0.060, 0.054, 0.048, 0.042, 0.036)
STILE_DEPTHS = (0.034, 0.029, 0.024, 0.019, 0.014)
STILE_WALL = 0.0025
NEST_Y_STEPS = tuple(
    (STILE_DEPTHS[index] + STILE_DEPTHS[index + 1]) * 0.5
    for index in range(len(STILE_DEPTHS) - 1)
)


def _tube_shell_profile(width: float, depth: float, wall: float):
    outer_radius = min(width, depth) * 0.18
    inner_width = width - 2.0 * wall
    inner_depth = depth - 2.0 * wall
    inner_radius = max(min(inner_width, inner_depth) * 0.16, 0.001)
    outer = rounded_rect_profile(width, depth, outer_radius, corner_segments=8)
    inner = rounded_rect_profile(
        inner_width,
        inner_depth,
        inner_radius,
        corner_segments=8,
    )
    return outer, inner


def _rect_tube_geometry(width: float, depth: float, height: float, wall: float):
    outer, inner = _tube_shell_profile(width, depth, wall)
    return ExtrudeWithHolesGeometry(
        outer,
        [inner],
        height,
        cap=True,
        center=False,
        closed=True,
    )


def _rect_sleeve_geometry(
    outer_width: float,
    outer_depth: float,
    inner_width: float,
    inner_depth: float,
    height: float,
):
    outer_radius = min(outer_width, outer_depth) * 0.18
    inner_radius = max(min(inner_width, inner_depth) * 0.16, 0.001)
    return ExtrudeWithHolesGeometry(
        rounded_rect_profile(
            outer_width,
            outer_depth,
            outer_radius,
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
        height,
        cap=True,
        center=False,
        closed=True,
    )


def _add_stile_shell(
    part,
    *,
    x_center: float,
    width: float,
    depth: float,
    wall: float,
    z0: float,
    height: float,
    material,
    visual_name: str,
    mesh_name: str,
) -> None:
    shell = _rect_tube_geometry(width, depth, height, wall)
    shell.translate(x_center, 0.0, z0)
    part.visual(
        mesh_from_geometry(shell, mesh_name),
        material=material,
        name=visual_name,
    )


def _add_wrap_sleeve(
    part,
    *,
    x_center: float,
    outer_width: float,
    outer_depth: float,
    inner_width: float,
    inner_depth: float,
    z0: float,
    height: float,
    material,
    visual_name: str,
    mesh_name: str,
) -> None:
    sleeve = _rect_sleeve_geometry(
        outer_width,
        outer_depth,
        inner_width,
        inner_depth,
        height,
    )
    sleeve.translate(x_center, 0.0, z0)
    part.visual(
        mesh_from_geometry(sleeve, mesh_name),
        material=material,
        name=visual_name,
    )


def _add_ladder_section(
    part,
    *,
    stage_index: int,
    stile_width: float,
    stile_depth: float,
    material_al,
    material_dark,
    material_catch,
    with_feet: bool = False,
    with_top_caps: bool = False,
) -> None:
    left_x = -CENTER_SPACING * 0.5
    right_x = CENTER_SPACING * 0.5
    rung_length = CENTER_SPACING - stile_width + 0.012

    _add_stile_shell(
        part,
        x_center=left_x,
        width=stile_width,
        depth=stile_depth,
        wall=STILE_WALL,
        z0=0.0,
        height=SECTION_HEIGHT,
        material=material_al,
        visual_name="left_stile",
        mesh_name=f"ladder_stage_{stage_index}_left_stile",
    )
    _add_stile_shell(
        part,
        x_center=right_x,
        width=stile_width,
        depth=stile_depth,
        wall=STILE_WALL,
        z0=0.0,
        height=SECTION_HEIGHT,
        material=material_al,
        visual_name="right_stile",
        mesh_name=f"ladder_stage_{stage_index}_right_stile",
    )

    collar_width = stile_width + 0.010
    collar_depth = stile_depth + 0.008
    _add_wrap_sleeve(
        part,
        x_center=left_x,
        outer_width=collar_width,
        outer_depth=collar_depth,
        inner_width=stile_width - 0.001,
        inner_depth=stile_depth - 0.001,
        z0=SECTION_HEIGHT - COLLAR_HEIGHT,
        height=COLLAR_HEIGHT,
        material=material_al,
        visual_name="left_collar",
        mesh_name=f"ladder_stage_{stage_index}_left_collar",
    )
    _add_wrap_sleeve(
        part,
        x_center=right_x,
        outer_width=collar_width,
        outer_depth=collar_depth,
        inner_width=stile_width - 0.001,
        inner_depth=stile_depth - 0.001,
        z0=SECTION_HEIGHT - COLLAR_HEIGHT,
        height=COLLAR_HEIGHT,
        material=material_al,
        visual_name="right_collar",
        mesh_name=f"ladder_stage_{stage_index}_right_collar",
    )

    for rung_index, rung_z in enumerate(RUNG_ZS):
        part.visual(
            Box((rung_length, 0.018, 0.032)),
            origin=Origin(xyz=(0.0, 0.0, rung_z)),
            material=material_al,
            name=f"rung_{rung_index + 1}",
        )
        part.visual(
            Box((rung_length * 0.90, 0.010, 0.004)),
            origin=Origin(xyz=(0.0, 0.0, rung_z + 0.018)),
            material=material_dark,
            name=f"rung_tread_{rung_index + 1}",
        )

    latch_z = SECTION_HEIGHT - COLLAR_HEIGHT - 0.035
    for side_name, x_center, side_sign in (
        ("left", left_x, -1.0),
        ("right", right_x, 1.0),
    ):
        part.visual(
            Box((0.012, stile_depth * 0.82, 0.042)),
            origin=Origin(
                xyz=(x_center + side_sign * (stile_width * 0.5 + 0.006), 0.0, latch_z),
            ),
            material=material_dark,
            name=f"{side_name}_latch_housing",
        )
        part.visual(
            Box((0.006, stile_depth * 0.52, 0.014)),
            origin=Origin(
                xyz=(x_center + side_sign * (stile_width * 0.5 + 0.014), 0.0, latch_z + 0.004),
            ),
            material=material_catch,
            name=f"{side_name}_catch_button",
        )

    if with_feet:
        foot_width = stile_width + 0.020
        foot_depth = stile_depth + 0.024
        for side_name, x_center in (("left", left_x), ("right", right_x)):
            part.visual(
                Box((foot_width, foot_depth, 0.034)),
                origin=Origin(xyz=(x_center, 0.002, 0.017)),
                material=material_dark,
                name=f"{side_name}_foot",
            )

    if with_top_caps:
        cap_width = stile_width - 0.006
        cap_depth = max(stile_depth - 0.004, 0.008)
        for side_name, x_center in (("left", left_x), ("right", right_x)):
            part.visual(
                Box((cap_width, cap_depth, 0.014)),
                origin=Origin(xyz=(x_center, 0.0, SECTION_HEIGHT - 0.007)),
                material=material_dark,
                name=f"{side_name}_top_cap",
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_compact_ladder")

    brushed_aluminium = model.material(
        "brushed_aluminium",
        rgba=(0.78, 0.80, 0.82, 1.0),
    )
    dark_plastic = model.material(
        "dark_plastic",
        rgba=(0.18, 0.19, 0.20, 1.0),
    )
    catch_red = model.material(
        "catch_red",
        rgba=(0.82, 0.17, 0.14, 1.0),
    )

    sections = []
    masses = (4.8, 4.0, 3.3, 2.7, 2.1)
    for index, name in enumerate(SECTION_NAMES):
        section = model.part(name)
        _add_ladder_section(
            section,
            stage_index=index,
            stile_width=STILE_WIDTHS[index],
            stile_depth=STILE_DEPTHS[index],
            material_al=brushed_aluminium,
            material_dark=dark_plastic,
            material_catch=catch_red,
            with_feet=index == 0,
            with_top_caps=index == len(SECTION_NAMES) - 1,
        )
        section.inertial = Inertial.from_geometry(
            Box((CENTER_SPACING + STILE_WIDTHS[index] + 0.030, 0.080, SECTION_HEIGHT)),
            mass=masses[index],
            origin=Origin(xyz=(0.0, 0.0, SECTION_HEIGHT * 0.5)),
        )
        sections.append(section)

    for index, articulation_name in enumerate(SLIDE_NAMES):
        model.articulation(
            articulation_name,
            ArticulationType.PRISMATIC,
            parent=sections[index],
            child=sections[index + 1],
            origin=Origin(xyz=(0.0, NEST_Y_STEPS[index], COLLAPSED_OFFSET)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=120.0,
                velocity=0.35,
                lower=0.0,
                upper=EXTENSION_TRAVEL,
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

    sections = [object_model.get_part(name) for name in SECTION_NAMES]
    slides = [object_model.get_articulation(name) for name in SLIDE_NAMES]

    for index, slide in enumerate(slides):
        parent = sections[index]
        child = sections[index + 1]
        ctx.expect_origin_distance(
            child,
            parent,
            axes="x",
            max_dist=0.002,
            name=f"{child.name} remains centered laterally on {parent.name}",
        )
        ctx.expect_origin_gap(
            child,
            parent,
            axis="y",
            min_gap=NEST_Y_STEPS[index] - 0.001,
            max_gap=NEST_Y_STEPS[index] + 0.001,
            name=f"{child.name} keeps the stepped nesting offset on {parent.name}",
        )
        ctx.expect_overlap(
            child,
            parent,
            axes="x",
            min_overlap=0.34,
            name=f"{child.name} keeps its stile span aligned with {parent.name}",
        )
        ctx.expect_overlap(
            child,
            parent,
            axes="z",
            min_overlap=0.42,
            name=f"{child.name} remains telescopically inserted into {parent.name}",
        )
        ctx.expect_contact(
            child,
            parent,
            contact_tol=1e-5,
            name=f"{child.name} stays physically guided by {parent.name} when nested",
        )

        rest_pos = ctx.part_world_position(child)
        with ctx.pose({slide: EXTENSION_TRAVEL}):
            ctx.expect_origin_distance(
                child,
                parent,
                axes="x",
                max_dist=0.002,
                name=f"{child.name} stays centered laterally on {parent.name} when extended",
            )
            ctx.expect_origin_gap(
                child,
                parent,
                axis="y",
                min_gap=NEST_Y_STEPS[index] - 0.001,
                max_gap=NEST_Y_STEPS[index] + 0.001,
                name=f"{child.name} keeps the stepped nesting offset on {parent.name} when extended",
            )
            ctx.expect_overlap(
                child,
                parent,
                axes="x",
                min_overlap=0.34,
                name=f"{child.name} keeps aligned stile spacing on {parent.name} when extended",
            )
            ctx.expect_overlap(
                child,
                parent,
                axes="z",
                min_overlap=0.18,
                name=f"{child.name} keeps retained insertion in {parent.name} at full extension",
            )
            extended_pos = ctx.part_world_position(child)

        ctx.check(
            f"{slide.name} extends upward",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[2] > rest_pos[2] + EXTENSION_TRAVEL - 0.02,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    top_section = sections[-1]
    closed_aabb = ctx.part_world_aabb(top_section)
    with ctx.pose({slide: EXTENSION_TRAVEL for slide in slides}):
        extended_aabb = ctx.part_world_aabb(top_section)

    ctx.check(
        "ladder reaches a substantially taller fully extended height",
        closed_aabb is not None
        and extended_aabb is not None
        and extended_aabb[1][2] > closed_aabb[1][2] + 1.60,
        details=f"closed={closed_aabb}, extended={extended_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
