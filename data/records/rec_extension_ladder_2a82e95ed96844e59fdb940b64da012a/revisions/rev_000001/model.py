from __future__ import annotations

from math import pi

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


def _add_ladder_section(
    part,
    *,
    rail_half_width: float,
    rail_length: float,
    rail_start_z: float,
    rail_size: tuple[float, float],
    rung_radius: float,
    rung_positions: list[float],
    rail_material,
    groove_material,
    rung_material,
) -> None:
    rail_x, rail_y = rail_size
    rail_center_z = rail_start_z + rail_length / 2.0
    rail_names = ("rail_0", "rail_1")
    groove_names = ("rail_groove_0", "rail_groove_1")

    for index, side in enumerate((-1.0, 1.0)):
        x = side * rail_half_width
        part.visual(
            Box((rail_x, rail_y, rail_length)),
            origin=Origin(xyz=(x, 0.0, rail_center_z)),
            material=rail_material,
            name=rail_names[index],
        )
        # A dark recessed line makes the solid rail read as an aluminium channel.
        part.visual(
            Box((rail_x * 0.46, 0.006, rail_length * 0.96)),
            origin=Origin(xyz=(x, -rail_y / 2.0 - 0.003, rail_center_z)),
            material=groove_material,
            name=groove_names[index],
        )

    rung_length = 2.0 * rail_half_width + rail_x * 1.35
    for index, z in enumerate(rung_positions):
        part.visual(
            Cylinder(radius=rung_radius, length=rung_length),
            origin=Origin(xyz=(0.0, -0.002, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=rung_material,
            name=f"rung_{index}",
        )


def _add_channel_guides(
    part,
    *,
    guide_prefix: str,
    parent_rail_half_width: float,
    child_rail_half_width: float,
    child_y: float,
    guide_zs: tuple[float, float],
    rail_x: float,
    rail_y: float,
    material,
    wear_material,
) -> None:
    """Add open C-channel guide collars on a lower ladder section.

    The collars are open toward the ladder centerline so the moving section's
    rungs can pass while the outside cheek and front/rear lips visibly capture
    each fly rail.
    """

    plate_t = 0.012
    guide_height = 0.14
    y_clear = rail_y + 0.028
    x_clear = rail_x + 0.026
    critical_outer_cheek_names = {
        ("middle_guide", 1, 1): "middle_guide_1_1_outer_cheek",
        ("top_guide", 1, 1): "top_guide_1_1_outer_cheek",
    }

    for z_index, z in enumerate(guide_zs):
        for side_index, side in enumerate((-1.0, 1.0)):
            child_x = side * child_rail_half_width
            parent_x = side * parent_rail_half_width
            outside_x = child_x + side * (x_clear / 2.0 + plate_t / 2.0)
            lip_center_x = child_x + side * (x_clear * 0.23)
            # Rear lips carry the nylon wear liner, which lightly bears on the
            # sliding rail face so the fly section is visibly guided rather than
            # floating through an oversized opening.
            rear_y = child_y + rail_y / 2.0 + 0.006 + plate_t / 2.0
            front_y = child_y - y_clear / 2.0 - plate_t / 2.0

            name_root = f"{guide_prefix}_{side_index}_{z_index}"
            outer_cheek_name = critical_outer_cheek_names.get(
                (guide_prefix, side_index, z_index),
                f"{name_root}_outer_cheek",
            )

            part.visual(
                Box((plate_t, y_clear + 2.0 * plate_t, guide_height)),
                origin=Origin(xyz=(outside_x, child_y, z)),
                material=material,
                name=outer_cheek_name,
            )
            part.visual(
                Box((x_clear * 0.72, plate_t, guide_height)),
                origin=Origin(xyz=(lip_center_x, rear_y, z)),
                material=material,
                name=f"{name_root}_rear_lip",
            )
            part.visual(
                Box((x_clear * 0.72, plate_t, guide_height)),
                origin=Origin(xyz=(lip_center_x, front_y, z)),
                material=material,
                name=f"{name_root}_front_lip",
            )
            part.visual(
                Box((x_clear * 0.45, 0.006, guide_height * 0.78)),
                origin=Origin(xyz=(child_x, child_y + rail_y / 2.0 + 0.003, z)),
                material=wear_material,
                name=f"{name_root}_nylon_liner",
            )
            # A welded strap ties the guide back into the fixed lower rail.
            strap_center_x = (parent_x + child_x) / 2.0
            strap_span_x = abs(parent_x - child_x) + rail_x * 0.75
            part.visual(
                Box((strap_span_x, 0.018, guide_height * 0.92)),
                origin=Origin(xyz=(strap_center_x, rear_y + 0.006, z)),
                material=material,
                name=f"{name_root}_mount_strap",
            )
            part.visual(
                Box((rail_x * 0.80, 0.020, guide_height * 0.70)),
                origin=Origin(xyz=(parent_x, -rail_y / 2.0 - 0.010, z)),
                material=material,
                name=f"{name_root}_rail_standoff",
            )


def _add_base_foot_mounts(part, *, rail_half_width: float, material) -> None:
    mount_names = ("foot_mount_0", "foot_mount_1")
    for index, side in enumerate((-1.0, 1.0)):
        part.visual(
            Box((0.055, 0.072, 0.078)),
            origin=Origin(xyz=(side * rail_half_width, 0.0, 0.096)),
            material=material,
            name=mount_names[index],
        )


def _add_swivel_foot(part, *, rubber_material, steel_material) -> None:
    part.visual(
        Box((0.185, 0.140, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, -0.092)),
        material=rubber_material,
        name="rubber_pad",
    )
    part.visual(
        Box((0.172, 0.118, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.069)),
        material=steel_material,
        name="shoe_plate",
    )
    for index, x in enumerate((-0.042, 0.042)):
        part.visual(
            Box((0.014, 0.060, 0.096)),
            origin=Origin(xyz=(x, 0.0, -0.045)),
            material=steel_material,
            name=f"fork_cheek_{index}",
        )
    part.visual(
        Cylinder(radius=0.013, length=0.108),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=steel_material,
        name="hinge_pin",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_section_extension_ladder")

    aluminium = model.material("brushed_aluminium", rgba=(0.78, 0.80, 0.78, 1.0))
    darker_aluminium = model.material("shadowed_aluminium", rgba=(0.36, 0.38, 0.38, 1.0))
    nylon = model.material("black_nylon_guide_liners", rgba=(0.025, 0.025, 0.023, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    steel = model.material("galvanized_steel", rgba=(0.55, 0.56, 0.54, 1.0))

    base = model.part("base")
    middle = model.part("middle_fly")
    top = model.part("top_fly")
    foot_0 = model.part("foot_0")
    foot_1 = model.part("foot_1")

    _add_ladder_section(
        base,
        rail_half_width=0.305,
        rail_length=4.22,
        rail_start_z=0.135,
        rail_size=(0.056, 0.070),
        rung_radius=0.021,
        rung_positions=[0.45 + 0.34 * i for i in range(11)],
        rail_material=aluminium,
        groove_material=darker_aluminium,
        rung_material=aluminium,
    )
    _add_base_foot_mounts(base, rail_half_width=0.305, material=steel)
    _add_channel_guides(
        base,
        guide_prefix="middle_guide",
        parent_rail_half_width=0.305,
        child_rail_half_width=0.255,
        child_y=-0.095,
        guide_zs=(1.41, 3.55),
        rail_x=0.050,
        rail_y=0.058,
        material=steel,
        wear_material=nylon,
    )

    _add_ladder_section(
        middle,
        rail_half_width=0.255,
        rail_length=4.02,
        rail_start_z=0.0,
        rail_size=(0.050, 0.058),
        rung_radius=0.019,
        rung_positions=[0.35 + 0.34 * i for i in range(11)],
        rail_material=aluminium,
        groove_material=darker_aluminium,
        rung_material=aluminium,
    )
    _add_channel_guides(
        middle,
        guide_prefix="top_guide",
        parent_rail_half_width=0.255,
        child_rail_half_width=0.215,
        child_y=-0.095,
        guide_zs=(1.25, 3.35),
        rail_x=0.046,
        rail_y=0.052,
        material=steel,
        wear_material=nylon,
    )

    _add_ladder_section(
        top,
        rail_half_width=0.215,
        rail_length=3.78,
        rail_start_z=0.0,
        rail_size=(0.046, 0.052),
        rung_radius=0.018,
        rung_positions=[0.35 + 0.34 * i for i in range(10)],
        rail_material=aluminium,
        groove_material=darker_aluminium,
        rung_material=aluminium,
    )
    # A slightly proud cap suggests the rounded top end stop of an extension ladder.
    top.visual(
        Box((0.52, 0.058, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 3.82)),
        material=aluminium,
        name="top_cap",
    )

    _add_swivel_foot(foot_0, rubber_material=rubber, steel_material=steel)
    _add_swivel_foot(foot_1, rubber_material=rubber, steel_material=steel)

    model.articulation(
        "base_to_middle_fly",
        ArticulationType.PRISMATIC,
        parent=base,
        child=middle,
        origin=Origin(xyz=(0.0, -0.095, 0.55)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.45, lower=0.0, upper=2.20),
    )
    model.articulation(
        "middle_to_top_fly",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=top,
        origin=Origin(xyz=(0.0, -0.095, 0.55)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=700.0, velocity=0.40, lower=0.0, upper=2.00),
    )
    model.articulation(
        "base_to_foot_0",
        ArticulationType.REVOLUTE,
        parent=base,
        child=foot_0,
        origin=Origin(xyz=(-0.305, 0.0, 0.115)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.5, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "base_to_foot_1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=foot_1,
        origin=Origin(xyz=(0.305, 0.0, 0.115)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.5, lower=-0.55, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    middle = object_model.get_part("middle_fly")
    top = object_model.get_part("top_fly")
    foot_0 = object_model.get_part("foot_0")
    foot_1 = object_model.get_part("foot_1")
    middle_slide = object_model.get_articulation("base_to_middle_fly")
    top_slide = object_model.get_articulation("middle_to_top_fly")

    ctx.allow_overlap(
        base,
        foot_0,
        elem_a="foot_mount_0",
        elem_b="hinge_pin",
        reason="The swivel-foot hinge pin is intentionally captured through the base rail end lug.",
    )
    ctx.allow_overlap(
        base,
        foot_1,
        elem_a="foot_mount_1",
        elem_b="hinge_pin",
        reason="The swivel-foot hinge pin is intentionally captured through the base rail end lug.",
    )

    ctx.expect_overlap(
        base,
        foot_0,
        axes="xyz",
        elem_a="foot_mount_0",
        elem_b="hinge_pin",
        min_overlap=0.010,
        name="foot 0 hinge pin is captured in the base lug",
    )
    ctx.expect_overlap(
        base,
        foot_1,
        axes="xyz",
        elem_a="foot_mount_1",
        elem_b="hinge_pin",
        min_overlap=0.010,
        name="foot 1 hinge pin is captured in the base lug",
    )

    ctx.expect_overlap(
        middle,
        base,
        axes="z",
        elem_a="rail_1",
        elem_b="middle_guide_1_1_outer_cheek",
        min_overlap=0.10,
        name="middle fly rail passes through the upper base guide",
    )
    ctx.expect_gap(
        base,
        middle,
        axis="x",
        positive_elem="middle_guide_1_1_outer_cheek",
        negative_elem="rail_1",
        min_gap=0.006,
        max_gap=0.030,
        name="middle guide has side clearance around the fly rail",
    )
    ctx.expect_overlap(
        top,
        middle,
        axes="z",
        elem_a="rail_1",
        elem_b="top_guide_1_1_outer_cheek",
        min_overlap=0.10,
        name="top fly rail passes through the upper middle guide",
    )

    rest_middle_z = ctx.part_world_position(middle)[2]
    rest_top_z = ctx.part_world_position(top)[2]
    with ctx.pose({middle_slide: 2.20}):
        extended_middle_z = ctx.part_world_position(middle)[2]
        ctx.expect_overlap(
            middle,
            base,
            axes="z",
            elem_a="rail_1",
            elem_b="middle_guide_1_1_outer_cheek",
            min_overlap=0.10,
            name="extended middle fly remains retained in the base guide",
        )

    with ctx.pose({middle_slide: 2.20, top_slide: 2.00}):
        extended_top_z = ctx.part_world_position(top)[2]
        ctx.expect_overlap(
            top,
            middle,
            axes="z",
            elem_a="rail_1",
            elem_b="top_guide_1_1_outer_cheek",
            min_overlap=0.10,
            name="extended top fly remains retained in the middle guide",
        )

    ctx.check(
        "middle fly extends upward on its prismatic joint",
        extended_middle_z > rest_middle_z + 2.0,
        details=f"rest_z={rest_middle_z}, extended_z={extended_middle_z}",
    )
    ctx.check(
        "top fly extends after the middle fly",
        extended_top_z > rest_top_z + 3.8,
        details=f"rest_z={rest_top_z}, extended_z={extended_top_z}",
    )

    return ctx.report()


object_model = build_object_model()
