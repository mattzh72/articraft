from __future__ import annotations

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
import math


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="laboratory_workbench")

    resin_top = model.material("dark_phenolic_top", rgba=(0.06, 0.065, 0.07, 1.0))
    powder_coat = model.material("light_gray_powder_coat", rgba=(0.72, 0.75, 0.75, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    drawer_blue = model.material("drawer_blue_enamel", rgba=(0.18, 0.31, 0.48, 1.0))
    shelf_mat = model.material("laminated_shelf_gray", rgba=(0.50, 0.52, 0.52, 1.0))
    black = model.material("black_keyway", rgba=(0.01, 0.01, 0.012, 1.0))

    bench = model.part("bench")

    # Thick chemical-resistant work surface.
    bench.visual(
        Box((1.60, 0.75, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.860)),
        material=resin_top,
        name="thick_top",
    )

    # Fixed side pedestal carcass below the right-hand side of the top.
    bench.visual(
        Box((0.036, 0.655, 0.765)),
        origin=Origin(xyz=(0.340, 0.000, 0.442)),
        material=powder_coat,
        name="pedestal_side_0",
    )
    bench.visual(
        Box((0.036, 0.655, 0.765)),
        origin=Origin(xyz=(0.780, 0.000, 0.442)),
        material=powder_coat,
        name="pedestal_side_1",
    )
    bench.visual(
        Box((0.480, 0.036, 0.765)),
        origin=Origin(xyz=(0.560, 0.308, 0.442)),
        material=powder_coat,
        name="pedestal_back",
    )
    bench.visual(
        Box((0.480, 0.655, 0.050)),
        origin=Origin(xyz=(0.560, 0.000, 0.083)),
        material=powder_coat,
        name="pedestal_plinth",
    )
    bench.visual(
        Box((0.480, 0.655, 0.040)),
        origin=Origin(xyz=(0.560, 0.000, 0.802)),
        material=powder_coat,
        name="pedestal_top_plate",
    )

    # Front rails divide the pedestal into three drawer openings.
    bench.visual(
        Box((0.480, 0.026, 0.032)),
        origin=Origin(xyz=(0.560, -0.323, 0.104)),
        material=powder_coat,
        name="pedestal_front_rail_0",
    )
    bench.visual(
        Box((0.480, 0.026, 0.032)),
        origin=Origin(xyz=(0.560, -0.323, 0.325)),
        material=powder_coat,
        name="pedestal_front_rail_1",
    )
    bench.visual(
        Box((0.480, 0.026, 0.032)),
        origin=Origin(xyz=(0.560, -0.323, 0.560)),
        material=powder_coat,
        name="pedestal_front_rail_2",
    )
    bench.visual(
        Box((0.480, 0.026, 0.032)),
        origin=Origin(xyz=(0.560, -0.323, 0.785)),
        material=powder_coat,
        name="pedestal_front_rail_3",
    )

    # Steel drawer guide rails fixed to the cabinet sides.  The moving drawer
    # slides ride on these ledges and remain engaged at full extension.
    drawer_centers_z = (0.210, 0.445, 0.680)
    for drawer_idx, zc in enumerate(drawer_centers_z):
        for side_idx, x in enumerate((0.374, 0.746)):
            bench.visual(
                Box((0.036, 0.500, 0.026)),
                origin=Origin(xyz=(x, -0.060, zc - 0.096)),
                material=steel,
                name=f"drawer_guide_{drawer_idx}_{side_idx}",
            )

    # Open side steel-tube legs and rear stretcher for the kneehole area.
    for leg_idx, (x, y) in enumerate(((-0.720, -0.315), (-0.720, 0.315), (-0.100, 0.315))):
        bench.visual(
            Box((0.055, 0.055, 0.765)),
            origin=Origin(xyz=(x, y, 0.442)),
            material=steel,
            name=f"tube_leg_{leg_idx}",
        )
    bench.visual(
        Box((0.690, 0.042, 0.065)),
        origin=Origin(xyz=(-0.410, 0.315, 0.710)),
        material=steel,
        name="rear_stretcher",
    )
    bench.visual(
        Box((0.665, 0.040, 0.045)),
        origin=Origin(xyz=(-0.410, -0.315, 0.135)),
        material=steel,
        name="front_low_stretcher",
    )

    # Fixed runners under the benchtop for the pull-out keyboard shelf.
    bench.visual(
        Box((0.038, 0.430, 0.027)),
        origin=Origin(xyz=(-0.650, -0.160, 0.8065)),
        material=steel,
        name="keyboard_runner_0",
    )
    bench.visual(
        Box((0.038, 0.430, 0.027)),
        origin=Origin(xyz=(0.010, -0.160, 0.8065)),
        material=steel,
        name="keyboard_runner_1",
    )

    drawer_center_x = 0.560
    drawer_center_y = -0.080
    for drawer_idx, zc in enumerate(drawer_centers_z):
        drawer = model.part(f"drawer_{drawer_idx}")
        drawer.visual(
            Box((0.340, 0.500, 0.150)),
            origin=Origin(xyz=(0.000, 0.000, 0.000)),
            material=powder_coat,
            name="drawer_box",
        )
        drawer.visual(
            Box((0.390, 0.035, 0.190)),
            origin=Origin(xyz=(0.000, -0.285, 0.000)),
            material=drawer_blue,
            name="drawer_front",
        )
        for tab_idx, x in enumerate((-0.120, 0.120)):
            drawer.visual(
                Box((0.040, 0.030, 0.050)),
                origin=Origin(xyz=(x, -0.258, 0.030)),
                material=powder_coat,
                name=f"front_mount_tab_{tab_idx}",
            )
        drawer.visual(
            Box((0.026, 0.480, 0.026)),
            origin=Origin(xyz=(-0.180, 0.000, -0.070)),
            material=steel,
            name="side_slide_0",
        )
        drawer.visual(
            Box((0.026, 0.480, 0.026)),
            origin=Origin(xyz=(0.180, 0.000, -0.070)),
            material=steel,
            name="side_slide_1",
        )

        # Pull handle and lock cylinder on each drawer face.
        for post_idx, x in enumerate((-0.120, 0.120)):
            drawer.visual(
                Box((0.026, 0.036, 0.026)),
                origin=Origin(xyz=(x, -0.312, -0.052)),
                material=steel,
                name=f"handle_post_{post_idx}",
            )
        drawer.visual(
            Box((0.285, 0.026, 0.026)),
            origin=Origin(xyz=(0.000, -0.335, -0.052)),
            material=steel,
            name="handle_bar",
        )
        drawer.visual(
            Cylinder(radius=0.018, length=0.014),
            origin=Origin(xyz=(0.000, -0.309, 0.052), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="lock_cylinder",
        )
        drawer.visual(
            Box((0.007, 0.004, 0.022)),
            origin=Origin(xyz=(0.000, -0.318, 0.052)),
            material=black,
            name="key_slot",
        )

        model.articulation(
            f"bench_to_drawer_{drawer_idx}",
            ArticulationType.PRISMATIC,
            parent=bench,
            child=drawer,
            origin=Origin(xyz=(drawer_center_x, drawer_center_y, zc)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=70.0, velocity=0.35, lower=0.0, upper=0.380),
        )

    shelf = model.part("keyboard_shelf")
    shelf.visual(
        Box((0.650, 0.310, 0.025)),
        origin=Origin(xyz=(0.000, 0.000, 0.000)),
        material=shelf_mat,
        name="shelf_panel",
    )
    shelf.visual(
        Box((0.036, 0.420, 0.025)),
        origin=Origin(xyz=(-0.330, -0.005, 0.025)),
        material=steel,
        name="shelf_slide_0",
    )
    shelf.visual(
        Box((0.036, 0.420, 0.025)),
        origin=Origin(xyz=(0.330, -0.005, 0.025)),
        material=steel,
        name="shelf_slide_1",
    )
    shelf.visual(
        Box((0.690, 0.035, 0.052)),
        origin=Origin(xyz=(0.000, -0.172, 0.014)),
        material=shelf_mat,
        name="front_lip",
    )

    model.articulation(
        "bench_to_keyboard_shelf",
        ArticulationType.PRISMATIC,
        parent=bench,
        child=shelf,
        origin=Origin(xyz=(-0.320, -0.160, 0.7555)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.30, lower=0.0, upper=0.300),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bench = object_model.get_part("bench")

    for drawer_idx in range(3):
        drawer = object_model.get_part(f"drawer_{drawer_idx}")
        joint = object_model.get_articulation(f"bench_to_drawer_{drawer_idx}")
        ctx.check(
            f"drawer {drawer_idx} uses a prismatic slide",
            joint.articulation_type == ArticulationType.PRISMATIC
            and joint.motion_limits is not None
            and abs((joint.motion_limits.upper or 0.0) - 0.380) < 1e-6,
            details=f"type={joint.articulation_type}, limits={joint.motion_limits}",
        )
        ctx.expect_contact(
            drawer,
            bench,
            elem_a="side_slide_0",
            elem_b=f"drawer_guide_{drawer_idx}_0",
            name=f"drawer {drawer_idx} left slide rides on guide",
        )
        ctx.expect_contact(
            drawer,
            bench,
            elem_a="side_slide_1",
            elem_b=f"drawer_guide_{drawer_idx}_1",
            name=f"drawer {drawer_idx} right slide rides on guide",
        )
        ctx.expect_gap(
            bench,
            drawer,
            axis="y",
            positive_elem="pedestal_front_rail_1",
            negative_elem="drawer_front",
            min_gap=0.004,
            max_gap=0.030,
            name=f"drawer {drawer_idx} front is just proud of pedestal",
        )
        rest_pos = ctx.part_world_position(drawer)
        with ctx.pose({joint: 0.380}):
            extended_pos = ctx.part_world_position(drawer)
            ctx.expect_overlap(
                drawer,
                bench,
                axes="y",
                elem_a="side_slide_0",
                elem_b=f"drawer_guide_{drawer_idx}_0",
                min_overlap=0.080,
                name=f"drawer {drawer_idx} remains retained on extension",
            )
        ctx.check(
            f"drawer {drawer_idx} extends toward the user",
            rest_pos is not None and extended_pos is not None and extended_pos[1] < rest_pos[1] - 0.300,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    shelf = object_model.get_part("keyboard_shelf")
    shelf_joint = object_model.get_articulation("bench_to_keyboard_shelf")
    ctx.check(
        "keyboard shelf uses a prismatic under-surface slide",
        shelf_joint.articulation_type == ArticulationType.PRISMATIC
        and shelf_joint.motion_limits is not None
        and abs((shelf_joint.motion_limits.upper or 0.0) - 0.300) < 1e-6,
        details=f"type={shelf_joint.articulation_type}, limits={shelf_joint.motion_limits}",
    )
    ctx.expect_contact(
        shelf,
        bench,
        elem_a="shelf_slide_0",
        elem_b="keyboard_runner_0",
        name="keyboard shelf slide rides on left runner",
    )
    ctx.expect_contact(
        shelf,
        bench,
        elem_a="shelf_slide_1",
        elem_b="keyboard_runner_1",
        name="keyboard shelf slide rides on right runner",
    )
    shelf_rest = ctx.part_world_position(shelf)
    with ctx.pose({shelf_joint: 0.300}):
        shelf_extended = ctx.part_world_position(shelf)
        ctx.expect_overlap(
            shelf,
            bench,
            axes="y",
            elem_a="shelf_slide_0",
            elem_b="keyboard_runner_0",
            min_overlap=0.050,
            name="keyboard shelf remains captured by runners",
        )
    ctx.check(
        "keyboard shelf pulls out from below the top",
        shelf_rest is not None and shelf_extended is not None and shelf_extended[1] < shelf_rest[1] - 0.240,
        details=f"rest={shelf_rest}, extended={shelf_extended}",
    )

    return ctx.report()


object_model = build_object_model()
