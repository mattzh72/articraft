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


def _x_rod(part, length: float, radius: float, xyz, *, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _y_rod(part, length: float, radius: float, xyz, *, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _z_rod(part, length: float, radius: float, xyz, *, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fold_out_drying_rack")

    powder_white = model.material("powder_white", rgba=(0.92, 0.92, 0.90, 1.0))
    soft_gray = model.material("soft_gray", rgba=(0.66, 0.68, 0.70, 1.0))
    bracket_gray = model.material("bracket_gray", rgba=(0.44, 0.46, 0.48, 1.0))

    rack_width = 0.72
    rack_height = 0.86
    tube_radius = 0.010
    rail_radius = 0.0048
    embed = 0.006

    central = model.part("central_frame")
    central.inertial = Inertial.from_geometry(
        Box((rack_width, 0.08, rack_height)),
        mass=4.8,
        origin=Origin(),
    )

    side_x = rack_width * 0.5 - tube_radius
    top_z = rack_height * 0.5 - tube_radius
    upright_len = rack_height - 2.0 * tube_radius + 2.0 * embed
    cross_len = rack_width - 2.0 * tube_radius + 2.0 * embed

    _z_rod(central, upright_len, tube_radius, (-side_x, 0.0, 0.0), material=powder_white, name="left_upright")
    _z_rod(central, upright_len, tube_radius, (side_x, 0.0, 0.0), material=powder_white, name="right_upright")
    _x_rod(central, cross_len, tube_radius, (0.0, 0.0, top_z), material=powder_white, name="top_rail")
    _x_rod(central, cross_len, tube_radius, (0.0, 0.0, -top_z), material=powder_white, name="bottom_rail")

    rail_low = -rack_height * 0.28
    rail_high = rack_height * 0.28
    for index, z in enumerate(
        [
            rail_low,
            rail_low * 0.55,
            0.0,
            rail_high * 0.55,
            rail_high,
        ],
        start=1,
    ):
        _x_rod(
            central,
            cross_len,
            rail_radius,
            (0.0, 0.0, z),
            material=soft_gray,
            name=f"central_drying_rail_{index}",
        )

    bracket_y = -0.020
    bracket_z = top_z - 0.055
    for side_name, x in (("left", -side_x + 0.018), ("right", side_x - 0.018)):
        central.visual(
            Box((0.045, 0.022, 0.120)),
            origin=Origin(xyz=(x, bracket_y, bracket_z)),
            material=bracket_gray,
            name=f"{side_name}_top_bracket",
        )
        _y_rod(
            central,
            0.030,
            0.005,
            (x, -0.024, bracket_z + 0.030),
            material=bracket_gray,
            name=f"{side_name}_hanger_pin",
        )

    hinge_radius = 0.007
    hinge_offset_z = rack_height * 0.26
    side_hinge_x = rack_width * 0.5 + hinge_radius + 0.001
    hinge_bridge_x = side_x + (side_hinge_x - side_x) * 0.5
    wing_hinge_len = 2.0 * (hinge_offset_z - 0.045)
    for side_name, x_sign in (("left", -1.0), ("right", 1.0)):
        x = side_hinge_x * x_sign
        _x_rod(
            central,
            0.024,
            0.0045,
            (hinge_bridge_x * x_sign, 0.0, hinge_offset_z),
            material=bracket_gray,
            name=f"{side_name}_upper_hinge_bridge",
        )
        _z_rod(
            central,
            0.090,
            hinge_radius,
            (x, 0.0, hinge_offset_z),
            material=bracket_gray,
            name=f"{side_name}_upper_hinge_knuckle",
        )
        _x_rod(
            central,
            0.024,
            0.0045,
            (hinge_bridge_x * x_sign, 0.0, -hinge_offset_z),
            material=bracket_gray,
            name=f"{side_name}_lower_hinge_bridge",
        )
        _z_rod(
            central,
            0.090,
            hinge_radius,
            (x, 0.0, -hinge_offset_z),
            material=bracket_gray,
            name=f"{side_name}_lower_hinge_knuckle",
        )

    bottom_hinge_z = -rack_height * 0.5 - hinge_radius - 0.001
    for side_name, x in (("left", -0.220), ("right", 0.220)):
        _z_rod(
            central,
            0.028,
            0.0045,
            (x, 0.0, bottom_hinge_z + 0.009),
            material=bracket_gray,
            name=f"{side_name}_support_hinge_bridge",
        )
        _x_rod(
            central,
            0.160,
            hinge_radius,
            (x, 0.0, bottom_hinge_z),
            material=bracket_gray,
            name=f"{side_name}_support_hinge_knuckle",
        )

    wing_span = 0.27
    wing_height = 0.76
    wing_inner_y = 0.020
    wing_side_y = wing_span - tube_radius
    wing_top_z = wing_height * 0.5 - tube_radius
    wing_upright_len = wing_height - 2.0 * tube_radius + 2.0 * embed
    wing_cross_len = wing_side_y - wing_inner_y + 2.0 * embed
    wing_mid_y = (wing_inner_y + wing_side_y) * 0.5

    left_wing = model.part("left_wing_frame")
    left_wing.inertial = Inertial.from_geometry(
        Box((0.08, wing_span, wing_height)),
        mass=1.7,
        origin=Origin(xyz=(0.0, wing_span * 0.5, 0.0)),
    )
    right_wing = model.part("right_wing_frame")
    right_wing.inertial = Inertial.from_geometry(
        Box((0.08, wing_span, wing_height)),
        mass=1.7,
        origin=Origin(xyz=(0.0, wing_span * 0.5, 0.0)),
    )

    for wing_name, wing in (("left", left_wing), ("right", right_wing)):
        _z_rod(
            wing,
            wing_hinge_len,
            hinge_radius,
            (0.0, 0.0, 0.0),
            material=bracket_gray,
            name=f"{wing_name}_hinge_knuckle",
        )
        _y_rod(wing, 0.040, 0.0055, (0.0, wing_inner_y * 0.5, 0.0), material=bracket_gray, name=f"{wing_name}_hinge_bridge")
        _z_rod(wing, wing_upright_len, tube_radius, (0.0, wing_inner_y, 0.0), material=powder_white, name="inner_upright")
        _z_rod(
            wing,
            wing_upright_len,
            tube_radius,
            (0.0, wing_side_y, 0.0),
            material=powder_white,
            name="outer_upright",
        )
        _y_rod(wing, wing_cross_len, tube_radius, (0.0, wing_mid_y, wing_top_z), material=powder_white, name="top_rail")
        _y_rod(wing, wing_cross_len, tube_radius, (0.0, wing_mid_y, -wing_top_z), material=powder_white, name="bottom_rail")
        for index, z in enumerate((-0.22, -0.07, 0.08, 0.23), start=1):
            _y_rod(
                wing,
                wing_cross_len,
                rail_radius,
                (0.0, wing_mid_y, z),
                material=soft_gray,
                name=f"drying_rail_{index}",
            )

    support_width = 0.68
    support_depth = 0.28
    support_top_y = 0.014
    support_side_x = support_width * 0.5 - tube_radius
    support_front_y = support_depth - tube_radius
    support_cross_x = support_width - 2.0 * tube_radius + 2.0 * embed
    support_side_len = support_depth - 2.0 * tube_radius + 2.0 * embed

    lower_support = model.part("lower_support_frame")
    lower_support.inertial = Inertial.from_geometry(
        Box((support_width, support_depth, 0.08)),
        mass=2.1,
        origin=Origin(xyz=(0.0, support_depth * 0.5, 0.0)),
    )
    _x_rod(
        lower_support,
        0.180,
        hinge_radius,
        (0.0, 0.0, 0.0),
        material=bracket_gray,
        name="support_hinge_knuckle",
    )
    for x in (-0.070, 0.070):
        _y_rod(
            lower_support,
            0.028,
            0.0055,
            (x, 0.014, 0.0),
            material=bracket_gray,
            name=f"hinge_bridge_{'left' if x < 0.0 else 'right'}",
        )

    _x_rod(
        lower_support,
        support_cross_x,
        tube_radius,
        (0.0, support_top_y, 0.0),
        material=powder_white,
        name="rear_top_rail",
    )
    _x_rod(
        lower_support,
        support_cross_x,
        tube_radius,
        (0.0, support_front_y, 0.0),
        material=powder_white,
        name="front_rail",
    )
    _y_rod(
        lower_support,
        support_side_len,
        tube_radius,
        (-support_side_x, support_depth * 0.5, 0.0),
        material=powder_white,
        name="left_side_rail",
    )
    _y_rod(
        lower_support,
        support_side_len,
        tube_radius,
        (support_side_x, support_depth * 0.5, 0.0),
        material=powder_white,
        name="right_side_rail",
    )
    for index, y in enumerate((0.070, 0.135, 0.200), start=1):
        _x_rod(
            lower_support,
            support_cross_x,
            rail_radius,
            (0.0, y, 0.0),
            material=soft_gray,
            name=f"support_drying_rail_{index}",
        )
    for side_name, x in (("left", -support_side_x + 0.030), ("right", support_side_x - 0.030)):
        lower_support.visual(
            Box((0.060, 0.016, 0.040)),
            origin=Origin(xyz=(x, 0.035, -0.014)),
            material=bracket_gray,
            name=f"{side_name}_folding_pad",
        )

    model.articulation(
        "left_wing_hinge",
        ArticulationType.REVOLUTE,
        parent=central,
        child=left_wing,
        origin=Origin(xyz=(-side_hinge_x, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.8, lower=0.0, upper=1.55),
    )
    model.articulation(
        "right_wing_hinge",
        ArticulationType.REVOLUTE,
        parent=central,
        child=right_wing,
        origin=Origin(xyz=(side_hinge_x, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.8, lower=0.0, upper=1.55),
    )
    model.articulation(
        "lower_support_hinge",
        ArticulationType.REVOLUTE,
        parent=central,
        child=lower_support,
        origin=Origin(xyz=(0.0, 0.0, bottom_hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.6, lower=0.0, upper=1.45),
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
    central = object_model.get_part("central_frame")
    left_wing = object_model.get_part("left_wing_frame")
    right_wing = object_model.get_part("right_wing_frame")
    lower_support = object_model.get_part("lower_support_frame")
    left_hinge = object_model.get_articulation("left_wing_hinge")
    right_hinge = object_model.get_articulation("right_wing_hinge")
    support_hinge = object_model.get_articulation("lower_support_hinge")
    central_aabb = ctx.part_world_aabb(central)
    ctx.check(
        "central frame has full wall-rack scale",
        central_aabb is not None
        and (central_aabb[1][0] - central_aabb[0][0]) > 0.68
        and (central_aabb[1][2] - central_aabb[0][2]) > 0.80,
        details=f"aabb={central_aabb}",
    )

    left_outer_rest = ctx.part_element_world_aabb(left_wing, elem="outer_upright")
    right_outer_rest = ctx.part_element_world_aabb(right_wing, elem="outer_upright")
    ctx.check(
        "wings start spread wide from the center frame",
        left_outer_rest is not None
        and right_outer_rest is not None
        and left_outer_rest[1][0] < -0.33
        and right_outer_rest[0][0] > 0.33,
        details=f"left={left_outer_rest}, right={right_outer_rest}",
    )

    with ctx.pose({left_hinge: 1.2, right_hinge: 1.2}):
        left_outer_folded = ctx.part_element_world_aabb(left_wing, elem="outer_upright")
        right_outer_folded = ctx.part_element_world_aabb(right_wing, elem="outer_upright")
        ctx.check(
            "wings fold inward toward the center span",
            left_outer_rest is not None
            and right_outer_rest is not None
            and left_outer_folded is not None
            and right_outer_folded is not None
            and left_outer_folded[1][0] > left_outer_rest[1][0] + 0.16
            and right_outer_folded[0][0] < right_outer_rest[0][0] - 0.16,
            details=(
                f"left_rest={left_outer_rest}, left_folded={left_outer_folded}, "
                f"right_rest={right_outer_rest}, right_folded={right_outer_folded}"
            ),
        )

    front_rail_rest = ctx.part_element_world_aabb(lower_support, elem="front_rail")
    with ctx.pose({support_hinge: 1.2}):
        front_rail_folded = ctx.part_element_world_aabb(lower_support, elem="front_rail")
        ctx.check(
            "lower support frame folds upward toward the wall",
            front_rail_rest is not None
            and front_rail_folded is not None
            and front_rail_folded[0][2] > front_rail_rest[0][2] + 0.20
            and front_rail_folded[1][1] < front_rail_rest[1][1] - 0.12,
            details=f"rest={front_rail_rest}, folded={front_rail_folded}",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
