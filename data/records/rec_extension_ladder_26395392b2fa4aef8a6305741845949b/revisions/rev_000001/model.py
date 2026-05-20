from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rectangular_tube_mesh(width: float, depth: float, wall: float, length: float, name: str):
    """Open-ended rectangular aluminium tube extruded along local Z."""
    outer = rounded_rect_profile(width, depth, radius=0.006, corner_segments=5)
    inner = rounded_rect_profile(
        width - 2.0 * wall,
        depth - 2.0 * wall,
        radius=0.0025,
        corner_segments=5,
    )
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(outer, [inner], length, center=True),
        name,
    )


def _rounded_pad_mesh(name: str):
    profile = rounded_rect_profile(0.19, 0.115, radius=0.028, corner_segments=8)
    return mesh_from_geometry(ExtrudeGeometry(profile, 0.035, center=True), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="twin_section_extension_ladder")

    aluminium = model.material("brushed_aluminium", rgba=(0.76, 0.78, 0.76, 1.0))
    cast = model.material("duller_cast_aluminium", rgba=(0.48, 0.50, 0.49, 1.0))
    dark_pin = model.material("dark_galvanized_pin", rgba=(0.15, 0.16, 0.16, 1.0))
    nylon = model.material("pale_nylon_wear_pad", rgba=(0.86, 0.84, 0.76, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))

    base_rail_mesh = _rectangular_tube_mesh(0.055, 0.038, 0.006, 3.20, "base_rail_tube")
    fly_rail_mesh = _rectangular_tube_mesh(0.052, 0.034, 0.0055, 3.15, "fly_rail_tube")
    pad_mesh = _rounded_pad_mesh("rubber_swivel_pad")

    base = model.part("base_section")
    fly = model.part("fly_section")

    # Fixed base section: two hollow rectangular stiles tied together by rungs.
    base_xs = (-0.285, 0.285)
    base_y = 0.035
    base_bottom = 0.22
    base_length = 3.20
    base_center_z = base_bottom + base_length / 2.0
    for x, rail_name in zip(base_xs, ("base_rail_0", "base_rail_1")):
        base.visual(
            base_rail_mesh,
            origin=Origin(xyz=(x, base_y, base_center_z)),
            material=aluminium,
            name=rail_name,
        )

    for i, z in enumerate((0.54, 0.86, 1.18, 1.50, 1.82, 2.14, 2.46, 2.78, 3.10)):
        base.visual(
            Cylinder(radius=0.018, length=0.62),
            origin=Origin(xyz=(0.0, base_y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=aluminium,
            name=f"base_rung_{i}",
        )
        # Flattened anti-slip crown on each round rung.
        base.visual(
            Box((0.50, 0.008, 0.010)),
            origin=Origin(xyz=(0.0, base_y - 0.014, z + 0.013)),
            material=cast,
            name=f"base_rung_grip_{i}",
        )

    # Cast guide collars mounted to the base rails.  Their rectangular holes
    # clear the fly stiles while the mounting lugs overlap the base stiles.
    fly_xs = (-0.215, 0.215)
    fly_y = -0.055
    # Four cast blocks form each real rectangular guide window instead of a
    # solid proxy; the fly stile clears the central opening.
    base.visual(Box((0.020, 0.092, 0.110)), origin=Origin(xyz=(-0.259, fly_y, 1.08)), material=cast, name="guide_0_lower_x_neg")
    base.visual(Box((0.008, 0.026, 0.092)), origin=Origin(xyz=(-0.245, fly_y, 1.08)), material=nylon, name="guide_wear_pad_0_lower")
    base.visual(Box((0.020, 0.092, 0.110)), origin=Origin(xyz=(-0.171, fly_y, 1.08)), material=cast, name="guide_0_lower_x_pos")
    base.visual(Box((0.068, 0.020, 0.110)), origin=Origin(xyz=(-0.215, fly_y - 0.036, 1.08)), material=cast, name="guide_0_lower_y_neg")
    base.visual(Box((0.068, 0.020, 0.110)), origin=Origin(xyz=(-0.215, fly_y + 0.036, 1.08)), material=cast, name="guide_0_lower_y_pos")
    base.visual(Box((0.095, 0.050, 0.112)), origin=Origin(xyz=(-0.250, -0.001, 1.08)), material=cast, name="guide_mount_0_lower")
    base.visual(Box((0.020, 0.092, 0.110)), origin=Origin(xyz=(-0.259, fly_y, 2.72)), material=cast, name="guide_0_upper_x_neg")
    base.visual(Box((0.008, 0.026, 0.092)), origin=Origin(xyz=(-0.245, fly_y, 2.72)), material=nylon, name="guide_wear_pad_0_upper")
    base.visual(Box((0.020, 0.092, 0.110)), origin=Origin(xyz=(-0.171, fly_y, 2.72)), material=cast, name="guide_0_upper_x_pos")
    base.visual(Box((0.068, 0.020, 0.110)), origin=Origin(xyz=(-0.215, fly_y - 0.036, 2.72)), material=cast, name="guide_0_upper_y_neg")
    base.visual(Box((0.068, 0.020, 0.110)), origin=Origin(xyz=(-0.215, fly_y + 0.036, 2.72)), material=cast, name="guide_0_upper_y_pos")
    base.visual(Box((0.095, 0.050, 0.112)), origin=Origin(xyz=(-0.250, -0.001, 2.72)), material=cast, name="guide_mount_0_upper")

    base.visual(Box((0.020, 0.092, 0.110)), origin=Origin(xyz=(0.171, fly_y, 1.08)), material=cast, name="guide_1_lower_x_neg")
    base.visual(Box((0.020, 0.092, 0.110)), origin=Origin(xyz=(0.259, fly_y, 1.08)), material=cast, name="guide_1_lower_x_pos")
    base.visual(Box((0.008, 0.026, 0.092)), origin=Origin(xyz=(0.245, fly_y, 1.08)), material=nylon, name="guide_wear_pad_1_lower")
    base.visual(Box((0.068, 0.020, 0.110)), origin=Origin(xyz=(0.215, fly_y - 0.036, 1.08)), material=cast, name="guide_1_lower_y_neg")
    base.visual(Box((0.068, 0.020, 0.110)), origin=Origin(xyz=(0.215, fly_y + 0.036, 1.08)), material=cast, name="guide_1_lower_y_pos")
    base.visual(Box((0.095, 0.050, 0.112)), origin=Origin(xyz=(0.250, -0.001, 1.08)), material=cast, name="guide_mount_1_lower")
    base.visual(Box((0.020, 0.092, 0.110)), origin=Origin(xyz=(0.171, fly_y, 2.72)), material=cast, name="guide_1_upper_x_neg")
    base.visual(Box((0.020, 0.092, 0.110)), origin=Origin(xyz=(0.259, fly_y, 2.72)), material=cast, name="guide_1_upper_x_pos")
    base.visual(Box((0.008, 0.026, 0.092)), origin=Origin(xyz=(0.245, fly_y, 2.72)), material=nylon, name="guide_wear_pad_1_upper")
    base.visual(Box((0.068, 0.020, 0.110)), origin=Origin(xyz=(0.215, fly_y - 0.036, 2.72)), material=cast, name="guide_1_upper_y_neg")
    base.visual(Box((0.068, 0.020, 0.110)), origin=Origin(xyz=(0.215, fly_y + 0.036, 2.72)), material=cast, name="guide_1_upper_y_pos")
    base.visual(Box((0.095, 0.050, 0.112)), origin=Origin(xyz=(0.250, -0.001, 2.72)), material=cast, name="guide_mount_1_upper")

    # Hinge-barrel halves and cheek plates for the two adjustable base feet.
    foot_pivots = []
    for i, x in enumerate(base_xs):
        pivot = (x, base_y, 0.140)
        foot_pivots.append(pivot)
        for side_index, dx in enumerate((-0.033, 0.033)):
            base.visual(
                Cylinder(radius=0.013, length=0.022),
                origin=Origin(xyz=(x + dx, base_y, pivot[2]), rpy=(0.0, pi / 2.0, 0.0)),
                material=dark_pin,
                name=f"foot_hinge_barrel_{i}_{side_index}",
            )
            base.visual(
                Box((0.023, 0.012, 0.084)),
                origin=Origin(xyz=(x + dx, base_y, 0.181)),
                material=cast,
                name=f"foot_hinge_cheek_{i}_{side_index}",
            )

    # Sliding fly section: a narrower front section already visibly extending
    # beyond the base section, with enough lower length retained in the guides.
    fly_length = 3.15
    fly.visual(
        fly_rail_mesh,
        origin=Origin(xyz=(fly_xs[0], fly_y, fly_length / 2.0)),
        material=aluminium,
        name="fly_rail_0",
    )
    fly.visual(
        fly_rail_mesh,
        origin=Origin(xyz=(fly_xs[1], fly_y, fly_length / 2.0)),
        material=aluminium,
        name="fly_rail_1",
    )

    for i, z in enumerate((0.34, 0.66, 0.98, 1.30, 1.62, 1.94, 2.26, 2.58, 2.90)):
        fly.visual(
            Cylinder(radius=0.017, length=0.48),
            origin=Origin(xyz=(0.0, fly_y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=aluminium,
            name=f"fly_rung_{i}",
        )
        fly.visual(
            Box((0.38, 0.007, 0.009)),
            origin=Origin(xyz=(0.0, fly_y - 0.013, z + 0.012)),
            material=cast,
            name=f"fly_rung_grip_{i}",
        )

    fly_joint = model.articulation(
        "base_to_fly",
        ArticulationType.PRISMATIC,
        parent=base,
        child=fly,
        origin=Origin(xyz=(0.0, 0.0, 0.95)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.20, lower=0.0, upper=0.70),
    )
    fly_joint.meta["description"] = "single fly section slides upward through the cast guide brackets"

    # Two independently swiveling rubber shoes at the bottoms of the base stiles.
    for i, pivot in enumerate(foot_pivots):
        foot = model.part(f"foot_{i}")
        foot.visual(
            Cylinder(radius=0.012, length=0.044),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_pin,
            name="center_barrel",
        )
        foot.visual(
            Box((0.024, 0.030, 0.120)),
            origin=Origin(xyz=(0.0, 0.0, -0.055)),
            material=cast,
            name="swivel_tongue",
        )
        foot.visual(
            Box((0.170, 0.096, 0.008)),
            origin=Origin(xyz=(0.0, 0.012, -0.113)),
            material=dark_pin,
            name="shoe_plate",
        )
        foot.visual(
            pad_mesh,
            origin=Origin(xyz=(0.0, 0.015, -0.134)),
            material=rubber,
            name="rubber_pad",
        )
        model.articulation(
            f"base_to_foot_{i}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=foot,
            origin=Origin(xyz=pivot),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=30.0, velocity=1.5, lower=-0.55, upper=0.55),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_section")
    fly = object_model.get_part("fly_section")
    fly_slide = object_model.get_articulation("base_to_fly")
    foot_0 = object_model.get_part("foot_0")
    foot_1 = object_model.get_part("foot_1")
    foot_swivel = object_model.get_articulation("base_to_foot_0")

    ctx.expect_overlap(
        fly,
        base,
        axes="yz",
        elem_a="fly_rail_1",
        elem_b="guide_1_lower_x_pos",
        min_overlap=0.02,
        name="fly rail reaches lower guide bracket",
    )
    ctx.expect_contact(
        fly,
        base,
        elem_a="fly_rail_1",
        elem_b="guide_wear_pad_1_lower",
        contact_tol=0.001,
        name="fly rail bears on lower guide wear pad",
    )
    ctx.expect_gap(
        base,
        fly,
        axis="x",
        positive_elem="guide_1_lower_x_pos",
        negative_elem="fly_rail_1",
        min_gap=0.004,
        max_gap=0.014,
        name="fly rail clears positive lower guide cheek",
    )
    ctx.expect_gap(
        fly,
        base,
        axis="x",
        positive_elem="fly_rail_1",
        negative_elem="guide_1_lower_x_neg",
        min_gap=0.004,
        max_gap=0.014,
        name="fly rail clears negative lower guide cheek",
    )
    ctx.expect_overlap(
        fly,
        base,
        axes="z",
        elem_a="fly_rail_1",
        elem_b="guide_1_upper_x_pos",
        min_overlap=0.08,
        name="fly rail passes through upper guide at rest",
    )

    rest_position = ctx.part_world_position(fly)
    with ctx.pose({fly_slide: fly_slide.motion_limits.upper}):
        ctx.expect_gap(
            base,
            fly,
            axis="x",
            positive_elem="guide_1_upper_x_pos",
            negative_elem="fly_rail_1",
            min_gap=0.004,
            max_gap=0.014,
            name="extended fly rail clears positive upper guide cheek",
        )
        ctx.expect_gap(
            fly,
            base,
            axis="x",
            positive_elem="fly_rail_1",
            negative_elem="guide_1_upper_x_neg",
            min_gap=0.004,
            max_gap=0.014,
            name="extended fly rail clears negative upper guide cheek",
        )
        ctx.expect_overlap(
            fly,
            base,
            axes="z",
            elem_a="fly_rail_1",
            elem_b="guide_1_upper_x_pos",
            min_overlap=0.08,
            name="extended fly rail remains inserted through guide",
        )
        extended_position = ctx.part_world_position(fly)

    ctx.check(
        "prismatic fly section moves upward",
        rest_position is not None
        and extended_position is not None
        and extended_position[2] > rest_position[2] + 0.60,
        details=f"rest={rest_position}, extended={extended_position}",
    )

    for foot, rail_elem in ((foot_0, "base_rail_0"), (foot_1, "base_rail_1")):
        ctx.expect_gap(
            base,
            foot,
            axis="z",
            positive_elem=rail_elem,
            negative_elem="rubber_pad",
            min_gap=0.15,
            max_gap=0.24,
            name=f"{foot.name} rubber pad sits below stile",
        )

    pad_aabb_rest = ctx.part_element_world_aabb(foot_0, elem="rubber_pad")
    with ctx.pose({foot_swivel: 0.45}):
        pad_aabb_swiveled = ctx.part_element_world_aabb(foot_0, elem="rubber_pad")

    rest_y = None if pad_aabb_rest is None else (pad_aabb_rest[0][1] + pad_aabb_rest[1][1]) / 2.0
    swivel_y = None if pad_aabb_swiveled is None else (
        pad_aabb_swiveled[0][1] + pad_aabb_swiveled[1][1]
    ) / 2.0
    ctx.check(
        "rubber foot swivels about hinge barrel",
        rest_y is not None and swivel_y is not None and abs(swivel_y - rest_y) > 0.035,
        details=f"rest_y={rest_y}, swivel_y={swivel_y}",
    )

    return ctx.report()


object_model = build_object_model()
