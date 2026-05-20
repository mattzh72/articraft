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


def _add_ladder_section(
    part,
    *,
    outer_width: float,
    rail_width: float,
    rail_depth: float,
    length: float,
    rung_count: int,
    rung_depth: float,
    rung_height: float,
    stile_material,
    rung_material,
    foot_material,
    barrel_y: float,
    barrel_radius: float,
    barrel_length: float,
    include_guides: bool = False,
    fly_outer_width: float = 0.0,
    fly_rail_width: float = 0.0,
    fly_y: float = 0.0,
) -> float:
    x_stile = (outer_width - rail_width) * 0.5
    rung_span = outer_width - 2.0 * rail_width + 0.014
    top_cap_height = 0.060
    top_cap_width = outer_width - 0.030

    part.visual(
        Box((rail_width, rail_depth, length)),
        origin=Origin(xyz=(-x_stile, 0.0, -length * 0.5)),
        material=stile_material,
        name="left_stile",
    )
    part.visual(
        Box((rail_width, rail_depth, length)),
        origin=Origin(xyz=(x_stile, 0.0, -length * 0.5)),
        material=stile_material,
        name="right_stile",
    )
    part.visual(
        Box((top_cap_width, rail_depth, top_cap_height)),
        origin=Origin(xyz=(0.0, 0.0, -top_cap_height * 0.5)),
        material=stile_material,
        name="top_cap",
    )

    top_margin = 0.30
    bottom_margin = 0.28
    for index in range(rung_count):
        t = index / max(1, rung_count - 1)
        z = -(top_margin + (length - top_margin - bottom_margin) * t)
        part.visual(
            Box((rung_span, rung_depth, rung_height)),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=rung_material,
            name=f"rung_{index + 1}",
        )

    if barrel_radius > 0.0 and barrel_length > 0.0:
        leaf_depth = abs(barrel_y) + rail_depth * 0.05
        leaf_center_y = barrel_y * 0.7
        for x_stile_sign, side_name in ((-x_stile, "left"), (x_stile, "right")):
            part.visual(
                Box((rail_width * 0.92, leaf_depth, 0.092)),
                origin=Origin(xyz=(x_stile_sign, leaf_center_y, -0.056)),
                material=stile_material,
                name=f"{side_name}_hinge_leaf",
            )
            part.visual(
                Cylinder(radius=barrel_radius, length=barrel_length),
                origin=Origin(
                    xyz=(x_stile_sign, barrel_y, 0.0),
                    rpy=(0.0, math.pi * 0.5, 0.0),
                ),
                material=stile_material,
                name=f"{side_name}_knuckle",
            )

    foot_width = 0.100
    foot_depth = 0.044
    foot_height = 0.032
    foot_z = -length - 0.006
    part.visual(
        Box((foot_width, foot_depth, foot_height)),
        origin=Origin(xyz=(-x_stile, 0.0, foot_z)),
        material=foot_material,
        name="left_foot",
    )
    part.visual(
        Box((foot_width, foot_depth, foot_height)),
        origin=Origin(xyz=(x_stile, 0.0, foot_z)),
        material=foot_material,
        name="right_foot",
    )

    if include_guides:
        fly_x = (fly_outer_width - fly_rail_width) * 0.5
        guide_centers_z = (-0.50, -1.18)
        outer_strip_w = 0.012
        inner_strip_w = 0.012
        strap_w = fly_rail_width + 0.050
        strip_depth = 0.076
        strip_height = 0.120
        strap_depth = 0.018
        strap_y = fly_y + rail_depth * 0.95
        outer_x_offset = fly_rail_width * 0.5 + outer_strip_w * 0.5 + 0.002
        inner_x_offset = fly_rail_width * 0.5 + inner_strip_w * 0.5 + 0.002
        for z_center in guide_centers_z:
            for sign, label in ((-1.0, "left"), (1.0, "right")):
                guide_x = sign * fly_x
                part.visual(
                    Box((outer_strip_w, strip_depth, strip_height)),
                    origin=Origin(
                        xyz=(guide_x - sign * outer_x_offset, fly_y, z_center),
                    ),
                    material=stile_material,
                    name=f"{label}_guide_outer_{int((z_center < -0.7))}",
                )
                part.visual(
                    Box((inner_strip_w, strip_depth, strip_height)),
                    origin=Origin(
                        xyz=(guide_x + sign * inner_x_offset, fly_y, z_center),
                    ),
                    material=stile_material,
                    name=f"{label}_guide_inner_{int((z_center < -0.7))}",
                )
                part.visual(
                    Box((strap_w, strap_depth, strip_height)),
                    origin=Origin(xyz=(guide_x, strap_y, z_center)),
                    material=stile_material,
                    name=f"{label}_guide_bridge_{int((z_center < -0.7))}",
                )

    return x_stile


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="convertible_ladder")

    aluminum = model.material("aluminum", rgba=(0.80, 0.82, 0.84, 1.0))
    rung_aluminum = model.material("rung_aluminum", rgba=(0.73, 0.75, 0.77, 1.0))
    hinge_steel = model.material("hinge_steel", rgba=(0.48, 0.50, 0.54, 1.0))
    rubber = model.material("rubber", rgba=(0.12, 0.12, 0.12, 1.0))
    lock_red = model.material("lock_red", rgba=(0.72, 0.13, 0.10, 1.0))

    section_angle = math.radians(31.0)
    front_length = 2.05
    rear_length = 2.10
    fly_length = 1.90
    fly_y = 0.040
    front_x_stile = (0.46 - 0.054) * 0.5
    rear_x_stile = (0.48 - 0.056) * 0.5
    pivot_x = (front_x_stile + rear_x_stile) * 0.5
    pivot_knuckle_radius = 0.022
    pivot_center_y = 0.013 + 0.024 * math.cos(section_angle)
    pivot_center_z = 0.024 * math.sin(section_angle)
    pivot_block_depth = 2.0 * (pivot_center_y - pivot_knuckle_radius)
    pivot_bridge_height = 0.012
    pivot_bridge_z = 0.015

    hinge_core = model.part("hinge_core")
    hinge_plate_span = 0.425
    hinge_plate_x = 0.205
    hinge_core.visual(
        Box((hinge_plate_span, 0.026, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=hinge_steel,
        name="lock_crossbar",
    )
    for sign, side_name in ((-1.0, "left"), (1.0, "right")):
        hinge_core.visual(
            Box((0.016, 0.006, 0.078)),
            origin=Origin(xyz=(sign * hinge_plate_x, 0.0, 0.048)),
            material=hinge_steel,
            name=f"{side_name}_lock_plate",
        )
        hinge_core.visual(
            Box((0.090, pivot_block_depth, pivot_bridge_height)),
            origin=Origin(xyz=(sign * pivot_x, 0.0, pivot_bridge_z)),
            material=hinge_steel,
            name=f"{side_name}_pivot_spacer",
        )
        hinge_core.visual(
            Box((0.032, 0.020, 0.026)),
            origin=Origin(xyz=(sign * hinge_plate_x, 0.0, 0.086)),
            material=lock_red,
            name=f"{side_name}_lock_release",
        )
    hinge_core.visual(
        Box((0.120, 0.026, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=hinge_steel,
        name="center_lock_block",
    )
    hinge_core.inertial = Inertial.from_geometry(
        Box((0.44, 0.04, 0.16)),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
    )

    front_section = model.part("front_section")
    _add_ladder_section(
        front_section,
        outer_width=0.46,
        rail_width=0.054,
        rail_depth=0.028,
        length=front_length,
        rung_count=6,
        rung_depth=0.032,
        rung_height=0.030,
        stile_material=aluminum,
        rung_material=rung_aluminum,
        foot_material=rubber,
        barrel_y=-0.024,
        barrel_radius=0.022,
        barrel_length=0.066,
    )
    front_section.inertial = Inertial.from_geometry(
        Box((0.46, 0.10, 2.12)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, -1.05)),
    )

    rear_section = model.part("rear_section")
    _add_ladder_section(
        rear_section,
        outer_width=0.48,
        rail_width=0.056,
        rail_depth=0.028,
        length=rear_length,
        rung_count=6,
        rung_depth=0.032,
        rung_height=0.030,
        stile_material=aluminum,
        rung_material=rung_aluminum,
        foot_material=rubber,
        barrel_y=0.024,
        barrel_radius=0.022,
        barrel_length=0.066,
        include_guides=True,
        fly_outer_width=0.36,
        fly_rail_width=0.042,
        fly_y=fly_y,
    )
    rear_section.inertial = Inertial.from_geometry(
        Box((0.48, 0.16, 2.16)),
        mass=8.8,
        origin=Origin(xyz=(0.0, 0.03, -1.07)),
    )

    fly_section = model.part("fly_section")
    fly_x_stile = _add_ladder_section(
        fly_section,
        outer_width=0.36,
        rail_width=0.042,
        rail_depth=0.024,
        length=fly_length,
        rung_count=5,
        rung_depth=0.028,
        rung_height=0.028,
        stile_material=aluminum,
        rung_material=rung_aluminum,
        foot_material=rubber,
        barrel_y=0.0,
        barrel_radius=0.0,
        barrel_length=0.0,
    )
    for sign, side_name in ((-1.0, "left"), (1.0, "right")):
        fly_section.visual(
            Box((0.022, 0.040, 0.110)),
            origin=Origin(xyz=(sign * fly_x_stile, -0.004, -0.52)),
            material=hinge_steel,
            name=f"{side_name}_guide_shoe_upper",
        )
        fly_section.visual(
            Box((0.022, 0.040, 0.110)),
            origin=Origin(xyz=(sign * fly_x_stile, -0.004, -1.12)),
            material=hinge_steel,
            name=f"{side_name}_guide_shoe_lower",
        )
    fly_section.visual(
        Box((0.31, 0.024, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=hinge_steel,
        name="top_stop_bar",
    )
    fly_section.inertial = Inertial.from_geometry(
        Box((0.36, 0.12, 1.98)),
        mass=6.2,
        origin=Origin(xyz=(0.0, 0.0, -0.78)),
    )

    model.articulation(
        "hinge_to_front",
        ArticulationType.FIXED,
        parent=hinge_core,
        child=front_section,
        origin=Origin(xyz=(0.0, -0.013, 0.0), rpy=(-section_angle, 0.0, 0.0)),
    )
    model.articulation(
        "front_to_rear_hinge",
        ArticulationType.REVOLUTE,
        parent=hinge_core,
        child=rear_section,
        origin=Origin(xyz=(0.0, 0.013, 0.0), rpy=(section_angle, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=55.0,
            velocity=1.2,
            lower=-2.0 * section_angle,
            upper=0.30,
        ),
    )
    model.articulation(
        "rear_to_fly_slide",
        ArticulationType.PRISMATIC,
        parent=rear_section,
        child=fly_section,
        origin=Origin(xyz=(0.0, fly_y, -0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.35,
            lower=0.0,
            upper=0.78,
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

    hinge_core = object_model.get_part("hinge_core")
    front_section = object_model.get_part("front_section")
    rear_section = object_model.get_part("rear_section")
    fly_section = object_model.get_part("fly_section")
    rear_hinge = object_model.get_articulation("front_to_rear_hinge")
    fly_slide = object_model.get_articulation("rear_to_fly_slide")

    hinge_limits = rear_hinge.motion_limits
    slide_limits = fly_slide.motion_limits
    ctx.check(
        "hinge range supports straight and A-frame lock positions",
        hinge_limits is not None
        and hinge_limits.lower is not None
        and hinge_limits.upper is not None
        and hinge_limits.lower <= -1.0
        and hinge_limits.upper >= 0.25,
        details=f"limits={hinge_limits}",
    )
    ctx.check(
        "fly section has meaningful extension travel",
        slide_limits is not None
        and slide_limits.upper is not None
        and slide_limits.upper >= 0.70,
        details=f"limits={slide_limits}",
    )

    with ctx.pose({rear_hinge: 0.0, fly_slide: 0.0}):
        ctx.expect_contact(
            front_section,
            hinge_core,
            elem_a="left_knuckle",
            elem_b="left_pivot_spacer",
            name="front section knuckle bears on the hinge core",
        )
        ctx.expect_contact(
            rear_section,
            hinge_core,
            elem_a="left_knuckle",
            elem_b="left_pivot_spacer",
            name="rear section knuckle bears on the hinge core",
        )
        ctx.expect_gap(
            rear_section,
            front_section,
            axis="y",
            positive_elem="left_foot",
            negative_elem="left_foot",
            min_gap=0.75,
            name="rear foot stands behind front foot in the A-frame pose",
        )
        ctx.expect_origin_distance(
            fly_section,
            rear_section,
            axes="x",
            max_dist=0.01,
            name="collapsed fly section stays centered between the rear stiles",
        )
        ctx.expect_overlap(
            fly_section,
            rear_section,
            axes="z",
            min_overlap=1.20,
            name="collapsed fly remains deeply retained in the rear guides",
        )

    rest_rear_aabb = ctx.part_world_aabb(rear_section)
    with ctx.pose({rear_hinge: 0.22}):
        opened_rear_aabb = ctx.part_world_aabb(rear_section)
    with ctx.pose({rear_hinge: -0.55}):
        narrowed_rear_aabb = ctx.part_world_aabb(rear_section)

    ctx.check(
        "rear hinge opens outward with positive rotation",
        rest_rear_aabb is not None
        and opened_rear_aabb is not None
        and opened_rear_aabb[1][1] > rest_rear_aabb[1][1] + 0.18,
        details=f"rest={rest_rear_aabb}, opened={opened_rear_aabb}",
    )
    ctx.check(
        "rear hinge can narrow toward a straighter ladder",
        rest_rear_aabb is not None
        and narrowed_rear_aabb is not None
        and narrowed_rear_aabb[1][1] < rest_rear_aabb[1][1] - 0.30,
        details=f"rest={rest_rear_aabb}, narrowed={narrowed_rear_aabb}",
    )

    rest_fly_aabb = ctx.part_world_aabb(fly_section)
    with ctx.pose({fly_slide: 0.78}):
        extended_fly_aabb = ctx.part_world_aabb(fly_section)
        ctx.expect_origin_distance(
            fly_section,
            rear_section,
            axes="x",
            max_dist=0.01,
            name="extended fly stays laterally aligned with the rear guides",
        )
        ctx.expect_overlap(
            fly_section,
            rear_section,
            axes="z",
            min_overlap=0.62,
            name="extended fly keeps retained insertion in the rear section",
        )

    ctx.check(
        "fly section extends upward when the prismatic slide opens",
        rest_fly_aabb is not None
        and extended_fly_aabb is not None
        and extended_fly_aabb[1][2] > rest_fly_aabb[1][2] + 0.42,
        details=f"rest={rest_fly_aabb}, extended={extended_fly_aabb}",
    )

    ctx.check(
        "hinge core remains part of the single ladder assembly",
        hinge_core is not None and front_section is not None and rear_section is not None,
        details="Resolved all major parts successfully.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
