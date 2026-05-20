from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="frameless_flush_sunroof_cassette")

    aluminum = model.material("satin_aluminium", rgba=(0.70, 0.72, 0.70, 1.0))
    dark_body = model.material("dark_roof_panel", rgba=(0.025, 0.030, 0.035, 1.0))
    rail_finish = model.material("anodized_track", rgba=(0.12, 0.13, 0.14, 1.0))
    rubber = model.material("black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    shoe_plastic = model.material("black_slide_plastic", rgba=(0.015, 0.015, 0.014, 1.0))
    glass_mat = model.material("smoked_glass", rgba=(0.20, 0.32, 0.40, 0.48))

    cassette = model.part("cassette")

    # A shallow dark cassette tray ties the static assembly together below the
    # glass pocket, with raised side walls supporting the upper seal frame.
    cassette.visual(
        Box((1.12, 1.60, 0.008)),
        origin=Origin(xyz=(0.0, 0.10, -0.024)),
        material=dark_body,
        name="lower_tray",
    )
    for x in (-0.535, 0.535):
        cassette.visual(
            Box((0.030, 1.60, 0.060)),
            origin=Origin(xyz=(x, 0.10, 0.010)),
            material=dark_body,
            name=f"side_wall_{0 if x < 0 else 1}",
        )

    # The thin, satin aluminium perimeter seal frame surrounds the flush glass
    # without adding a framed border to the glass itself.
    cassette.visual(
        Box((1.10, 0.050, 0.008)),
        origin=Origin(xyz=(0.0, -0.675, 0.038)),
        material=aluminum,
        name="front_seal_frame",
    )
    cassette.visual(
        Box((1.10, 0.050, 0.008)),
        origin=Origin(xyz=(0.0, 0.075, 0.038)),
        material=aluminum,
        name="rear_seal_frame",
    )
    for x in (-0.500, 0.500):
        cassette.visual(
            Box((0.050, 0.800, 0.008)),
            origin=Origin(xyz=(x, -0.300, 0.038)),
            material=aluminum,
            name=f"side_seal_frame_{0 if x < 0 else 1}",
        )

    # Black elastomer lips immediately inside the aluminium frame make the
    # opening read as a flush weather seal around the frameless glass.
    cassette.visual(
        Box((0.94, 0.018, 0.007)),
        origin=Origin(xyz=(0.0, -0.641, 0.039)),
        material=rubber,
        name="front_gasket",
    )
    cassette.visual(
        Box((0.94, 0.018, 0.007)),
        origin=Origin(xyz=(0.0, 0.041, 0.039)),
        material=rubber,
        name="rear_gasket",
    )
    for x, gasket_name in ((-0.466, "side_gasket_0"), (0.466, "side_gasket_1")):
        cassette.visual(
            Box((0.018, 0.655, 0.007)),
            origin=Origin(xyz=(x, -0.300, 0.039)),
            material=rubber,
            name=gasket_name,
        )

    # The opaque rear roof skin sits above the slide pocket.  Its front lip
    # connects down to the seal frame while leaving a low, clear cavity for the
    # glass panel to disappear beneath it.
    cassette.visual(
        Box((1.12, 0.780, 0.010)),
        origin=Origin(xyz=(0.0, 0.495, 0.049)),
        material=dark_body,
        name="rear_body_panel",
    )
    cassette.visual(
        Box((1.12, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, 0.104, 0.035)),
        material=dark_body,
        name="rear_panel_lip",
    )

    # Twin prismatic rail channels run continuously from the front of the
    # opening into the rear pocket.  Each rail is a low U-channel with a base
    # contact surface for the slide shoes and narrow guide lips.
    for rail_index, rail_x in enumerate((-0.340, 0.340)):
        cassette.visual(
            Box((0.150, 1.590, 0.015)),
            origin=Origin(xyz=(rail_x, 0.105, -0.0125)),
            material=rail_finish,
            name=f"rail_base_{rail_index}",
        )
        for side_index, lip_x in enumerate((rail_x - 0.070, rail_x + 0.070)):
            cassette.visual(
                Box((0.015, 1.590, 0.030)),
                origin=Origin(xyz=(lip_x, 0.105, 0.003)),
                material=rail_finish,
                name=f"rail_lip_{rail_index}_{side_index}",
            )

    # Front and rear crossmembers visibly bolt the rails into the cassette tray.
    cassette.visual(
        Box((0.95, 0.030, 0.020)),
        origin=Origin(xyz=(0.0, -0.675, -0.010)),
        material=rail_finish,
        name="front_crossmember",
    )
    cassette.visual(
        Box((0.95, 0.030, 0.020)),
        origin=Origin(xyz=(0.0, 0.875, -0.010)),
        material=rail_finish,
        name="rear_crossmember",
    )

    glass_panel = model.part("glass_panel")
    glass_lite = (
        cq.Workplane("XY")
        .box(0.840, 0.600, 0.010)
        .edges("|Z")
        .fillet(0.040)
    )
    glass_panel.visual(
        mesh_from_cadquery(glass_lite, "rounded_glass_lite", tolerance=0.0008),
        origin=Origin(),
        material=glass_mat,
        name="glass_lite",
    )

    # Four shoes are bonded to the glass through small downstand brackets; the
    # glass remains visibly frameless while the black hardware rides in the two
    # fixed rail channels.
    shoe_positions = [
        (-0.340, -0.240),
        (0.340, -0.240),
        (-0.340, 0.240),
        (0.340, 0.240),
    ]
    for shoe_index, (x, y) in enumerate(shoe_positions):
        glass_panel.visual(
            Box((0.050, 0.045, 0.018)),
            origin=Origin(xyz=(x, y, -0.014)),
            material=shoe_plastic,
            name=f"shoe_stem_{shoe_index}",
        )
        glass_panel.visual(
            Box((0.105, 0.075, 0.018)),
            origin=Origin(xyz=(x, y, -0.032)),
            material=shoe_plastic,
            name=f"slide_shoe_{shoe_index}",
        )

    model.articulation(
        "cassette_to_glass",
        ArticulationType.PRISMATIC,
        parent=cassette,
        child=glass_panel,
        origin=Origin(xyz=(0.0, -0.300, 0.036)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.780),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cassette = object_model.get_part("cassette")
    glass_panel = object_model.get_part("glass_panel")
    slide = object_model.get_articulation("cassette_to_glass")

    # Closed: the glass sits within the aluminium/rubber opening, flush with the
    # perimeter seal frame, while the four corner shoes are captured between the
    # guide lips and rest on the two track bases.
    ctx.expect_gap(
        cassette,
        glass_panel,
        axis="x",
        positive_elem="side_gasket_1",
        negative_elem="glass_lite",
        min_gap=0.020,
        max_gap=0.050,
        name="closed glass clears right seal",
    )
    ctx.expect_gap(
        glass_panel,
        cassette,
        axis="x",
        positive_elem="glass_lite",
        negative_elem="side_gasket_0",
        min_gap=0.020,
        max_gap=0.050,
        name="closed glass clears left seal",
    )
    ctx.expect_gap(
        glass_panel,
        cassette,
        axis="y",
        positive_elem="glass_lite",
        negative_elem="front_gasket",
        min_gap=0.020,
        max_gap=0.050,
        name="closed glass clears front seal",
    )
    ctx.expect_gap(
        cassette,
        glass_panel,
        axis="y",
        positive_elem="rear_gasket",
        negative_elem="glass_lite",
        min_gap=0.020,
        max_gap=0.050,
        name="closed glass clears rear seal",
    )
    glass_box = ctx.part_element_world_aabb(glass_panel, elem="glass_lite")
    frame_box = ctx.part_element_world_aabb(cassette, elem="rear_seal_frame")
    ctx.check(
        "glass top is flush with seal frame",
        glass_box is not None
        and frame_box is not None
        and abs(glass_box[1][2] - frame_box[1][2]) <= 0.002,
        details=f"glass_aabb={glass_box}, rear_frame_aabb={frame_box}",
    )
    for shoe_index, rail_index in enumerate((0, 1, 0, 1)):
        ctx.expect_within(
            glass_panel,
            cassette,
            axes="x",
            inner_elem=f"slide_shoe_{shoe_index}",
            outer_elem=f"rail_base_{rail_index}",
            margin=0.0,
            name=f"slide shoe {shoe_index} centered on its rail",
        )
        ctx.expect_gap(
            glass_panel,
            cassette,
            axis="z",
            positive_elem=f"slide_shoe_{shoe_index}",
            negative_elem=f"rail_base_{rail_index}",
            min_gap=-0.0005,
            max_gap=0.001,
            name=f"slide shoe {shoe_index} bears on rail base",
        )

    rest_pos = ctx.part_world_position(glass_panel)
    with ctx.pose({slide: 0.780}):
        ctx.expect_within(
            glass_panel,
            cassette,
            axes="xy",
            inner_elem="glass_lite",
            outer_elem="rear_body_panel",
            margin=0.0,
            name="opened glass fully under rear body panel footprint",
        )
        ctx.expect_gap(
            cassette,
            glass_panel,
            axis="z",
            positive_elem="rear_body_panel",
            negative_elem="glass_lite",
            min_gap=0.002,
            max_gap=0.006,
            name="opened glass clears below rear body panel",
        )
        extended_pos = ctx.part_world_position(glass_panel)
    ctx.check(
        "glass retracts rearward along rails",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[1] > rest_pos[1] + 0.70,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
