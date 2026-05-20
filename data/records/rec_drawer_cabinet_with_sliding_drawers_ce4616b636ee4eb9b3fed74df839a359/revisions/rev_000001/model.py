from __future__ import annotations

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mobile_parts_bin_organizer_cart")

    steel = model.material("powder_coated_steel", rgba=(0.08, 0.09, 0.09, 1.0))
    galvanized = model.material("galvanized_rails", rgba=(0.58, 0.60, 0.58, 1.0))
    drawer_plastic = model.material("smoke_translucent_plastic", rgba=(0.35, 0.55, 0.70, 0.46))
    drawer_front = model.material("blue_tinted_drawer_front", rgba=(0.18, 0.39, 0.65, 0.62))
    label_mat = model.material("paper_labels", rgba=(0.92, 0.90, 0.78, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    zinc = model.material("zinc_hardware", rgba=(0.66, 0.67, 0.64, 1.0))

    frame = model.part("frame")

    tube = 0.025
    width = 0.62
    half_w = width / 2.0
    front_x = 0.0
    back_x = -0.34
    mid_x = (front_x + back_x) / 2.0
    depth = front_x - back_x
    bottom_z = 0.135
    top_z = 0.845
    post_h = top_z - bottom_z
    grid_x = -0.006

    # Four-post welded rectangular cart frame.
    for ix, x in enumerate((front_x, back_x)):
        for iy, y in enumerate((-half_w, half_w)):
            frame.visual(
                Box((tube, tube, post_h)),
                origin=Origin(xyz=(x, y, (bottom_z + top_z) / 2.0)),
                material=steel,
                name=f"post_{ix}_{iy}",
            )

    for z_name, z in (("bottom", bottom_z), ("top", top_z)):
        for iy, y in enumerate((-half_w, half_w)):
            frame.visual(
                Box((depth + tube, tube, 0.030)),
                origin=Origin(xyz=(mid_x, y, z)),
                material=steel,
                name=f"{z_name}_side_rail_{iy}",
            )
        for ix, x in enumerate((front_x, back_x)):
            frame.visual(
                Box((tube, width + tube, 0.030)),
                origin=Origin(xyz=(x, 0.0, z)),
                material=steel,
                name=f"{z_name}_cross_rail_{ix}",
            )

    # Front and rear lattices define the sixteen drawer openings.
    col_centers = [-0.225, -0.075, 0.075, 0.225]
    row_centers = [0.305, 0.435, 0.565, 0.695]
    y_boundaries = [-0.300, -0.150, 0.0, 0.150, 0.300]
    z_boundaries = [0.240, 0.370, 0.500, 0.630, 0.760]

    for face_name, x in (("front", grid_x), ("rear", back_x)):
        for i, y in enumerate(y_boundaries):
            frame.visual(
                Box((0.014, 0.014, z_boundaries[-1] - z_boundaries[0] + 0.020)),
                origin=Origin(xyz=(x, y, (z_boundaries[0] + z_boundaries[-1]) / 2.0)),
                material=steel,
                name=f"{face_name}_divider_{i}",
            )
        for i, z in enumerate(z_boundaries):
            frame.visual(
                Box((0.014, width, 0.014)),
                origin=Origin(xyz=(x, 0.0, z)),
                material=steel,
                name=f"{face_name}_shelf_bar_{i}",
            )

    # Individual fixed steel guide rails for every drawer, one pair per slot.
    rail_len = 0.360
    rail_center_x = -0.160
    for r, zc in enumerate(row_centers):
        for c, yc in enumerate(col_centers):
            for side_name, side in (("lower_y", -1), ("upper_y", 1)):
                frame.visual(
                    Box((rail_len, 0.006, 0.010)),
                    origin=Origin(xyz=(rail_center_x, yc + side * 0.0725, zc - 0.044)),
                    material=galvanized,
                    name=f"guide_{r}_{c}_{side_name}",
                )

    # Caster mounting plates welded to the underside of the lower rectangle.
    caster_mounts = [
        (-0.030, -0.285),
        (-0.030, 0.285),
        (-0.310, -0.285),
        (-0.310, 0.285),
    ]
    for i, (x, y) in enumerate(caster_mounts):
        frame.visual(
            Box((0.080, 0.080, 0.008)),
            origin=Origin(xyz=(x, y, 0.116)),
            material=steel,
            name=f"caster_plate_{i}",
        )

    # Sixteen hollow plastic drawers.  Their part frame sits at the center of the
    # closed front face; positive prismatic travel pulls the drawer out of the cart.
    drawer_w = 0.116
    drawer_h = 0.102
    bin_depth = 0.305
    wall = 0.006
    pull_limit = 0.200

    for r, zc in enumerate(row_centers):
        for c, yc in enumerate(col_centers):
            drawer = model.part(f"drawer_{r}_{c}")

            drawer.visual(
                Box((0.014, 0.135, 0.112)),
                origin=Origin(xyz=(0.0, 0.0, 0.0)),
                material=drawer_front,
                name="front_panel",
            )
            drawer.visual(
                Box((0.004, 0.075, 0.026)),
                origin=Origin(xyz=(0.008, 0.0, 0.022)),
                material=label_mat,
                name="label_card",
            )
            drawer.visual(
                Box((0.012, 0.082, 0.012)),
                origin=Origin(xyz=(0.012, 0.0, -0.026)),
                material=zinc,
                name="pull_lip",
            )
            drawer.visual(
                Box((0.300, drawer_w, wall)),
                origin=Origin(xyz=(-0.155, 0.0, -drawer_h / 2.0 + wall / 2.0)),
                material=drawer_plastic,
                name="bottom_pan",
            )
            drawer.visual(
                Box((bin_depth - wall, wall, 0.090)),
                origin=Origin(xyz=(-0.155, -drawer_w / 2.0 + wall / 2.0, -0.005)),
                material=drawer_plastic,
                name="lower_side_wall",
            )
            drawer.visual(
                Box((bin_depth - wall, wall, 0.090)),
                origin=Origin(xyz=(-0.155, drawer_w / 2.0 - wall / 2.0, -0.005)),
                material=drawer_plastic,
                name="upper_side_wall",
            )
            drawer.visual(
                Box((0.008, drawer_w, 0.090)),
                origin=Origin(xyz=(-bin_depth + 0.004, 0.0, -0.005)),
                material=drawer_plastic,
                name="back_wall",
            )
            drawer.visual(
                Box((0.260, 0.006, 0.008)),
                origin=Origin(xyz=(-0.135, -0.065, -0.035)),
                material=zinc,
                name="lower_runner",
            )
            drawer.visual(
                Box((0.260, 0.006, 0.008)),
                origin=Origin(xyz=(-0.135, 0.065, -0.035)),
                material=zinc,
                name="upper_runner",
            )

            model.articulation(
                f"drawer_{r}_{c}_slide",
                ArticulationType.PRISMATIC,
                parent=frame,
                child=drawer,
                origin=Origin(xyz=(0.025, yc, zc)),
                axis=(1.0, 0.0, 0.0),
                motion_limits=MotionLimits(lower=0.0, upper=pull_limit, effort=35.0, velocity=0.25),
            )

    # Four swivel casters, each with a separate freely rolling wheel.
    for i, (x, y) in enumerate(caster_mounts):
        caster = model.part(f"caster_{i}")
        caster.visual(
            Cylinder(radius=0.024, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=zinc,
            name="swivel_bearing",
        )
        caster.visual(
            Cylinder(radius=0.008, length=0.046),
            origin=Origin(xyz=(0.0, 0.0, -0.023)),
            material=zinc,
            name="stem",
        )
        caster.visual(
            Box((0.042, 0.060, 0.012)),
            origin=Origin(xyz=(0.0, 0.0, -0.033)),
            material=zinc,
            name="fork_bridge",
        )
        for side_name, sy in (("lower", -1.0), ("upper", 1.0)):
            caster.visual(
                Box((0.014, 0.006, 0.052)),
                origin=Origin(xyz=(0.0, sy * 0.027, -0.065)),
                material=zinc,
                name=f"{side_name}_fork_plate",
            )
            caster.visual(
                Cylinder(radius=0.005, length=0.014),
                origin=Origin(xyz=(0.0, sy * 0.019, -0.086), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=zinc,
                name=f"{side_name}_axle_stub",
            )

        model.articulation(
            f"caster_{i}_swivel",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=caster,
            origin=Origin(xyz=(x, y, 0.108)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=4.0, velocity=8.0),
        )

        wheel = model.part(f"wheel_{i}")
        wheel.visual(
            Cylinder(radius=0.033, length=0.024),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name="rubber_tire",
        )
        wheel.visual(
            Cylinder(radius=0.017, length=0.030),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=zinc,
            name="metal_hub",
        )
        model.articulation(
            f"wheel_{i}_spin",
            ArticulationType.CONTINUOUS,
            parent=caster,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.086)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    drawers = [p for p in object_model.parts if p.name.startswith("drawer_")]
    drawer_slides = [j for j in object_model.articulations if j.name.endswith("_slide")]
    ctx.check("sixteen individual drawers", len(drawers) == 16, details=f"drawers={len(drawers)}")
    ctx.check(
        "sixteen drawer slides",
        len(drawer_slides) == 16
        and all(j.articulation_type == ArticulationType.PRISMATIC for j in drawer_slides),
        details=f"slides={len(drawer_slides)}",
    )

    sample_drawer = object_model.get_part("drawer_1_2")
    sample_slide = object_model.get_articulation("drawer_1_2_slide")
    frame = object_model.get_part("frame")
    limits = sample_slide.motion_limits
    travel = limits.upper if limits and limits.upper is not None else 0.0

    ctx.expect_gap(
        sample_drawer,
        frame,
        axis="z",
        positive_elem="lower_runner",
        negative_elem="guide_1_2_lower_y",
        min_gap=0.0,
        max_gap=0.001,
        name="drawer runner sits on its guide rail",
    )
    ctx.expect_overlap(
        sample_drawer,
        frame,
        axes="x",
        elem_a="lower_runner",
        elem_b="guide_1_2_lower_y",
        min_overlap=0.15,
        name="closed drawer runner is fully engaged",
    )

    rest_pos = ctx.part_world_position(sample_drawer)
    with ctx.pose({sample_slide: travel}):
        ctx.expect_overlap(
            sample_drawer,
            frame,
            axes="x",
            elem_a="lower_runner",
            elem_b="guide_1_2_lower_y",
            min_overlap=0.040,
            name="pulled drawer retains rail engagement",
        )
        extended_pos = ctx.part_world_position(sample_drawer)

    ctx.check(
        "drawer pulls outward along the cart front",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.15,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
