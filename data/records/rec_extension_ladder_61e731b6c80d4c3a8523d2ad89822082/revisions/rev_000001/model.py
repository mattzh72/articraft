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
)


BASE_LENGTH = 3.20
FLY_LENGTH = 3.00
BASE_RAIL_X = 0.19
FLY_RAIL_X = 0.17
BASE_RAIL_SIZE = (0.060, 0.026)
FLY_RAIL_SIZE = (0.050, 0.020)
RAIL_WALL = 0.003
FLY_Y_OFFSET = 0.032
FLY_CLOSED_BOTTOM = 1.00
FLY_TRAVEL = 1.60


def _rect_profile(width: float, depth: float) -> list[tuple[float, float]]:
    half_w = width * 0.5
    half_d = depth * 0.5
    return [
        (-half_w, -half_d),
        (half_w, -half_d),
        (half_w, half_d),
        (-half_w, half_d),
    ]


def _rhs_mesh(width: float, depth: float, wall: float, length: float, name: str):
    outer = _rect_profile(width, depth)
    inner = _rect_profile(width - 2.0 * wall, depth - 2.0 * wall)
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(outer, [inner], height=length, center=True),
        name,
    )


def _add_rungs(
    part,
    *,
    prefix: str,
    count: int,
    length: float,
    depth: float,
    height: float,
    y_center: float,
    z_start: float,
    spacing: float,
    material,
) -> None:
    for index in range(count):
        z_center = z_start + index * spacing + height * 0.5
        part.visual(
            Box((length, depth, height)),
            origin=Origin(xyz=(0.0, y_center, z_center)),
            material=material,
            name=f"{prefix}_rung_{index + 1}",
        )


def _add_guide_bracket(
    part,
    *,
    side_sign: float,
    z_center: float,
    height: float,
    material,
) -> None:
    rail_x = side_sign * BASE_RAIL_X
    fly_x = side_sign * FLY_RAIL_X
    rail_w, rail_d = BASE_RAIL_SIZE
    fly_w, _ = FLY_RAIL_SIZE

    pad_w = 0.014
    pad_d = 0.014
    bridge_d = 0.016
    rib_w = 0.016
    rib_d = 0.024
    nose_w = 0.012
    nose_d = 0.006

    rail_outer_face = rail_x + side_sign * (rail_w * 0.5)
    fly_outer_face = fly_x + side_sign * (fly_w * 0.5)

    pad_x = rail_outer_face + side_sign * (pad_w * 0.5)
    pad_y = rail_d * 0.5 + pad_d * 0.5

    rib_x = fly_outer_face + side_sign * (rib_w * 0.5)
    rib_y = FLY_Y_OFFSET

    bridge_x = 0.5 * (pad_x + rib_x)
    bridge_w = abs(rib_x - pad_x) + 0.5 * (pad_w + rib_w)

    part.visual(
        Box((pad_w, pad_d, height)),
        origin=Origin(xyz=(pad_x, pad_y, z_center)),
        material=material,
    )
    part.visual(
        Box((bridge_w, bridge_d, height)),
        origin=Origin(xyz=(bridge_x, 0.020, z_center)),
        material=material,
    )
    part.visual(
        Box((rib_w, rib_d, height)),
        origin=Origin(xyz=(rib_x, rib_y, z_center)),
        material=material,
    )
    part.visual(
        Box((nose_w, nose_d, height * 0.75)),
        origin=Origin(xyz=(rib_x, 0.047, z_center)),
        material=material,
    )


def _add_base_foot_mount(part, *, rail_x: float, material, prefix: str) -> None:
    cheek_w = 0.010
    cheek_d = 0.028
    cheek_h = 0.030
    cheek_offset = 0.030
    cheek_z = 0.003
    for side, suffix in ((-1.0, "outer"), (1.0, "inner")):
        part.visual(
            Box((cheek_w, cheek_d, cheek_h)),
            origin=Origin(xyz=(rail_x + side * cheek_offset, 0.0, cheek_z)),
            material=material,
            name=f"{prefix}_{suffix}_cheek",
        )


def _build_swivel_foot(part, *, prefix: str, metal_material, rubber_material) -> None:
    part.visual(
        Cylinder(radius=0.010, length=0.050),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=metal_material,
        name=f"{prefix}_barrel",
    )
    part.visual(
        Box((0.038, 0.022, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=metal_material,
        name=f"{prefix}_yoke",
    )
    part.visual(
        Box((0.035, 0.020, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, -0.027)),
        material=metal_material,
        name=f"{prefix}_stem",
    )
    part.visual(
        Box((0.125, 0.055, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
        material=rubber_material,
        name=f"{prefix}_pad",
    )
    part.visual(
        Box((0.028, 0.022, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, -0.041)),
        material=rubber_material,
        name=f"{prefix}_pad_boss",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="twin_section_extension_ladder")

    aluminium = model.material("aluminium", rgba=(0.83, 0.85, 0.87, 1.0))
    cast_aluminium = model.material("cast_aluminium", rgba=(0.73, 0.75, 0.78, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.34, 0.36, 0.39, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    base_rail_mesh = _rhs_mesh(BASE_RAIL_SIZE[0], BASE_RAIL_SIZE[1], RAIL_WALL, BASE_LENGTH, "base_rail_rhs")
    fly_rail_mesh = _rhs_mesh(FLY_RAIL_SIZE[0], FLY_RAIL_SIZE[1], RAIL_WALL, FLY_LENGTH, "fly_rail_rhs")

    base_section = model.part("base_section")
    base_section.visual(
        base_rail_mesh,
        origin=Origin(xyz=(-BASE_RAIL_X, 0.0, BASE_LENGTH * 0.5)),
        material=aluminium,
        name="base_left_rail",
    )
    base_section.visual(
        base_rail_mesh,
        origin=Origin(xyz=(BASE_RAIL_X, 0.0, BASE_LENGTH * 0.5)),
        material=aluminium,
        name="base_right_rail",
    )
    _add_rungs(
        base_section,
        prefix="base",
        count=11,
        length=0.360,
        depth=0.030,
        height=0.030,
        y_center=0.0,
        z_start=0.320,
        spacing=0.275,
        material=aluminium,
    )
    for guide_z in (1.98, 2.53):
        _add_guide_bracket(base_section, side_sign=-1.0, z_center=guide_z, height=0.110, material=cast_aluminium)
        _add_guide_bracket(base_section, side_sign=1.0, z_center=guide_z, height=0.110, material=cast_aluminium)
    _add_base_foot_mount(base_section, rail_x=-BASE_RAIL_X, material=cast_aluminium, prefix="left_foot_mount")
    _add_base_foot_mount(base_section, rail_x=BASE_RAIL_X, material=cast_aluminium, prefix="right_foot_mount")
    base_section.inertial = Inertial.from_geometry(
        Box((0.48, 0.10, 3.25)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, 1.62)),
    )

    fly_section = model.part("fly_section")
    fly_section.visual(
        fly_rail_mesh,
        origin=Origin(xyz=(-FLY_RAIL_X, FLY_Y_OFFSET, FLY_LENGTH * 0.5)),
        material=aluminium,
        name="fly_left_rail",
    )
    fly_section.visual(
        fly_rail_mesh,
        origin=Origin(xyz=(FLY_RAIL_X, FLY_Y_OFFSET, FLY_LENGTH * 0.5)),
        material=aluminium,
        name="fly_right_rail",
    )
    _add_rungs(
        fly_section,
        prefix="fly",
        count=10,
        length=0.310,
        depth=0.024,
        height=0.028,
        y_center=FLY_Y_OFFSET,
        z_start=0.280,
        spacing=0.275,
        material=aluminium,
    )
    fly_section.visual(
        Box((0.300, 0.020, 0.016)),
        origin=Origin(xyz=(0.0, FLY_Y_OFFSET, 0.110)),
        material=dark_steel,
        name="fly_bottom_brace",
    )
    fly_section.inertial = Inertial.from_geometry(
        Box((0.42, 0.09, 3.02)),
        mass=10.0,
        origin=Origin(xyz=(0.0, FLY_Y_OFFSET, FLY_LENGTH * 0.5)),
    )

    left_foot = model.part("left_foot")
    _build_swivel_foot(left_foot, prefix="left_foot", metal_material=dark_steel, rubber_material=rubber)
    left_foot.inertial = Inertial.from_geometry(
        Box((0.13, 0.06, 0.07)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
    )

    right_foot = model.part("right_foot")
    _build_swivel_foot(right_foot, prefix="right_foot", metal_material=dark_steel, rubber_material=rubber)
    right_foot.inertial = Inertial.from_geometry(
        Box((0.13, 0.06, 0.07)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
    )

    model.articulation(
        "base_to_fly",
        ArticulationType.PRISMATIC,
        parent=base_section,
        child=fly_section,
        origin=Origin(xyz=(0.0, 0.0, FLY_CLOSED_BOTTOM)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.45,
            lower=0.0,
            upper=FLY_TRAVEL,
        ),
    )
    model.articulation(
        "base_to_left_foot",
        ArticulationType.REVOLUTE,
        parent=base_section,
        child=left_foot,
        origin=Origin(xyz=(-BASE_RAIL_X, 0.0, 0.003)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.5,
            lower=-0.65,
            upper=0.65,
        ),
    )
    model.articulation(
        "base_to_right_foot",
        ArticulationType.REVOLUTE,
        parent=base_section,
        child=right_foot,
        origin=Origin(xyz=(BASE_RAIL_X, 0.0, 0.003)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.5,
            lower=-0.65,
            upper=0.65,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_section = object_model.get_part("base_section")
    fly_section = object_model.get_part("fly_section")
    left_foot = object_model.get_part("left_foot")
    right_foot = object_model.get_part("right_foot")
    fly_slide = object_model.get_articulation("base_to_fly")
    left_swivel = object_model.get_articulation("base_to_left_foot")
    right_swivel = object_model.get_articulation("base_to_right_foot")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "fly section is vertical prismatic stage",
        fly_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(fly_slide.axis) == (0.0, 0.0, 1.0),
        details=f"type={fly_slide.articulation_type}, axis={fly_slide.axis}",
    )
    ctx.check(
        "swivel feet hinge about x axis",
        tuple(left_swivel.axis) == (1.0, 0.0, 0.0) and tuple(right_swivel.axis) == (1.0, 0.0, 0.0),
        details=f"left_axis={left_swivel.axis}, right_axis={right_swivel.axis}",
    )

    ctx.expect_contact(left_foot, base_section, name="left swivel foot is mounted to base rail")
    ctx.expect_contact(right_foot, base_section, name="right swivel foot is mounted to base rail")

    with ctx.pose({fly_slide: 0.0}):
        ctx.expect_gap(
            fly_section,
            base_section,
            axis="y",
            min_gap=0.006,
            max_gap=0.014,
            positive_elem="fly_left_rail",
            negative_elem="base_left_rail",
            name="left fly rail clears the base rail face",
        )
        ctx.expect_gap(
            fly_section,
            base_section,
            axis="y",
            min_gap=0.006,
            max_gap=0.014,
            positive_elem="fly_right_rail",
            negative_elem="base_right_rail",
            name="right fly rail clears the base rail face",
        )
        ctx.expect_overlap(
            fly_section,
            base_section,
            axes="z",
            min_overlap=2.10,
            elem_a="fly_left_rail",
            elem_b="base_left_rail",
            name="collapsed fly section stays deeply nested in left guide span",
        )
        ctx.expect_overlap(
            fly_section,
            base_section,
            axes="z",
            min_overlap=2.10,
            elem_a="fly_right_rail",
            elem_b="base_right_rail",
            name="collapsed fly section stays deeply nested in right guide span",
        )

    fly_rest_position = ctx.part_world_position(fly_section)
    fly_extended_position = None
    with ctx.pose({fly_slide: FLY_TRAVEL}):
        ctx.expect_overlap(
            fly_section,
            base_section,
            axes="z",
            min_overlap=0.55,
            elem_a="fly_left_rail",
            elem_b="base_left_rail",
            name="extended fly section retains left rail insertion",
        )
        ctx.expect_overlap(
            fly_section,
            base_section,
            axes="z",
            min_overlap=0.55,
            elem_a="fly_right_rail",
            elem_b="base_right_rail",
            name="extended fly section retains right rail insertion",
        )
        fly_extended_position = ctx.part_world_position(fly_section)

    ctx.check(
        "fly section extends upward",
        fly_rest_position is not None
        and fly_extended_position is not None
        and fly_extended_position[2] > fly_rest_position[2] + 1.55,
        details=f"rest={fly_rest_position}, extended={fly_extended_position}",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        minimum, maximum = aabb
        return tuple((minimum[i] + maximum[i]) * 0.5 for i in range(3))

    left_pad_rest = None
    left_pad_high = None
    with ctx.pose({left_swivel: 0.0}):
        left_pad_rest = _aabb_center(ctx.part_element_world_aabb(left_foot, elem="left_foot_pad"))
    with ctx.pose({left_swivel: 0.45}):
        left_pad_high = _aabb_center(ctx.part_element_world_aabb(left_foot, elem="left_foot_pad"))

    ctx.check(
        "left swivel foot changes pitch",
        left_pad_rest is not None
        and left_pad_high is not None
        and left_pad_high[1] > left_pad_rest[1] + 0.018
        and left_pad_high[2] > left_pad_rest[2] + 0.003,
        details=f"rest={left_pad_rest}, high={left_pad_high}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
