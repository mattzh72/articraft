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
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelGeometry,
    WheelHub,
    WheelRim,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="platform_cart")

    steel = model.material("powder_blue_steel", rgba=(0.08, 0.22, 0.34, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.04, 0.05, 0.055, 1.0))
    zinc = model.material("zinc_plated_metal", rgba=(0.70, 0.72, 0.68, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    warning = model.material("red_release_tabs", rgba=(0.75, 0.06, 0.03, 1.0))

    deck_length = 0.96
    deck_width = 0.58
    deck_thickness = 0.060
    deck_center_z = 0.320
    deck_top_z = deck_center_z + deck_thickness / 2.0
    deck_bottom_z = deck_center_z - deck_thickness / 2.0

    deck = model.part("main_deck")
    deck.visual(
        Box((deck_length, deck_width, deck_thickness)),
        origin=Origin(xyz=(0.0, 0.0, deck_center_z)),
        material=steel,
        name="deck_panel",
    )
    deck.visual(
        Box((deck_length, 0.040, 0.045)),
        origin=Origin(xyz=(0.0, deck_width / 2.0 - 0.020, deck_top_z + 0.0125)),
        material=steel,
        name="side_lip_0",
    )
    deck.visual(
        Box((deck_length, 0.040, 0.045)),
        origin=Origin(xyz=(0.0, -deck_width / 2.0 + 0.020, deck_top_z + 0.0125)),
        material=steel,
        name="side_lip_1",
    )
    deck.visual(
        Box((0.045, deck_width, 0.045)),
        origin=Origin(xyz=(deck_length / 2.0 - 0.0225, 0.0, deck_top_z + 0.0125)),
        material=steel,
        name="front_lip",
    )
    deck.visual(
        Box((0.045, deck_width, 0.045)),
        origin=Origin(xyz=(-deck_length / 2.0 + 0.0225, 0.0, deck_top_z + 0.0125)),
        material=steel,
        name="rear_lip",
    )
    for i, y in enumerate((-0.145, 0.0, 0.145)):
        deck.visual(
            Box((0.72, 0.040, 0.008)),
            origin=Origin(xyz=(0.015, y, deck_top_z + 0.004)),
            material=rubber,
            name=f"deck_grip_{i}",
        )

    # Underside T-rails retain the sliding front platform.  Their webs touch the
    # underside of the deck and the lower bulbs are captured by the extension clips.
    deck.visual(
        Box((0.610, 0.012, 0.045)),
        origin=Origin(xyz=(0.185, -0.145, deck_bottom_z - 0.0225)),
        material=zinc,
        name="guide_web_0",
    )
    deck.visual(
        Box((0.610, 0.030, 0.018)),
        origin=Origin(xyz=(0.185, -0.145, deck_bottom_z - 0.054)),
        material=zinc,
        name="guide_rail_0",
    )
    deck.visual(
        Box((0.610, 0.012, 0.045)),
        origin=Origin(xyz=(0.185, 0.145, deck_bottom_z - 0.0225)),
        material=zinc,
        name="guide_web_1",
    )
    deck.visual(
        Box((0.610, 0.030, 0.018)),
        origin=Origin(xyz=(0.185, 0.145, deck_bottom_z - 0.054)),
        material=zinc,
        name="guide_rail_1",
    )

    # Rear hinge cheeks: two forked bracket pairs mounted to the deck lip.
    for side, y_sign in enumerate((-1.0, 1.0)):
        for j, y_offset in enumerate((0.205, 0.265)):
            deck.visual(
                Box((0.052, 0.012, 0.085)),
                origin=Origin(
                    xyz=(
                        -deck_length / 2.0 - 0.024,
                        y_sign * y_offset,
                        deck_top_z + 0.0425,
                    )
                ),
                material=zinc,
                name=f"rear_hinge_cheek_{side}_{j}",
            )
        deck.visual(
            Cylinder(radius=0.018, length=0.085),
            origin=Origin(
                xyz=(-deck_length / 2.0 - 0.024, y_sign * 0.235, deck_top_z + 0.0425),
                rpy=(-pi / 2.0, 0.0, 0.0),
            ),
            material=dark_steel,
            name=f"hinge_pin_shadow_{side}",
        )

    caster_origins = (
        (0.350, -0.250),
        (0.350, 0.250),
        (-0.350, -0.250),
        (-0.350, 0.250),
    )

    for i, (x, y) in enumerate(caster_origins):
        deck.visual(
            Box((0.120, 0.090, 0.012)),
            origin=Origin(xyz=(x, y, deck_bottom_z - 0.006)),
            material=zinc,
            name=f"caster_mount_{i}",
        )

        fork = model.part(f"swivel_fork_{i}")
        fork.visual(
            Cylinder(radius=0.017, length=0.080),
            origin=Origin(xyz=(0.0, 0.0, -0.058)),
            material=zinc,
            name="swivel_stem",
        )
        fork.visual(
            Cylinder(radius=0.038, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, -0.018)),
            material=zinc,
            name="top_bearing_cap",
        )
        fork.visual(
            Cylinder(radius=0.018, length=0.070),
            origin=Origin(xyz=(0.0, -0.028, -0.060), rpy=(0.20, 0.0, 0.0)),
            material=zinc,
            name="offset_neck",
        )
        fork.visual(
            Box((0.135, 0.058, 0.020)),
            origin=Origin(xyz=(0.0, -0.035, -0.090)),
            material=zinc,
            name="fork_bridge",
        )
        for sx in (-1.0, 1.0):
            fork.visual(
                Box((0.012, 0.052, 0.140)),
                origin=Origin(xyz=(sx * 0.040, -0.035, -0.170)),
                material=zinc,
                name=f"fork_cheek_{0 if sx < 0 else 1}",
            )
        fork.visual(
            Cylinder(radius=0.010, length=0.100),
            origin=Origin(xyz=(0.0, -0.035, -0.185), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_steel,
            name="wheel_axle",
        )

        model.articulation(
            f"deck_to_caster_{i}",
            ArticulationType.CONTINUOUS,
            parent=deck,
            child=fork,
            origin=Origin(xyz=(x, y, deck_bottom_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=6.0),
        )

        wheel = model.part(f"wheel_{i}")
        wheel.visual(
            mesh_from_geometry(
                TireGeometry(
                    0.075,
                    0.052,
                    inner_radius=0.055,
                    tread=TireTread(style="block", depth=0.004, count=16, land_ratio=0.58),
                    grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.002),),
                    sidewall=TireSidewall(style="rounded", bulge=0.04),
                    shoulder=TireShoulder(width=0.006, radius=0.003),
                ),
                f"caster_tire_{i}",
            ),
            material=rubber,
            name="tire",
        )
        wheel.visual(
            mesh_from_geometry(
                WheelGeometry(
                    0.054,
                    0.046,
                    rim=WheelRim(
                        inner_radius=0.033,
                        flange_height=0.004,
                        flange_thickness=0.002,
                        bead_seat_depth=0.002,
                    ),
                    hub=WheelHub(radius=0.018, width=0.038, cap_style="domed"),
                    bore=WheelBore(style="round", diameter=0.020),
                ),
                f"caster_rim_{i}",
            ),
            material=zinc,
            name="rim",
        )
        model.articulation(
            f"caster_to_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(0.0, -0.035, -0.185)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=6.0, velocity=20.0),
        )

    handle = model.part("rear_handle")
    handle.visual(
        Cylinder(radius=0.018, length=0.738),
        origin=Origin(xyz=(0.0, -0.235, 0.391)),
        material=dark_steel,
        name="handle_post_0",
    )
    handle.visual(
        Cylinder(radius=0.022, length=0.038),
        origin=Origin(xyz=(0.0, -0.235, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="hinge_sleeve_0",
    )
    handle.visual(
        Box((0.030, 0.030, 0.012)),
        origin=Origin(xyz=(0.0, -0.235, 0.026)),
        material=zinc,
        name="hinge_weld_0",
    )
    handle.visual(
        Cylinder(radius=0.018, length=0.738),
        origin=Origin(xyz=(0.0, 0.235, 0.391)),
        material=dark_steel,
        name="handle_post_1",
    )
    handle.visual(
        Cylinder(radius=0.022, length=0.038),
        origin=Origin(xyz=(0.0, 0.235, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="hinge_sleeve_1",
    )
    handle.visual(
        Box((0.030, 0.030, 0.012)),
        origin=Origin(xyz=(0.0, 0.235, 0.026)),
        material=zinc,
        name="hinge_weld_1",
    )
    handle.visual(
        Cylinder(radius=0.018, length=0.470),
        origin=Origin(xyz=(0.0, 0.0, 0.760), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="top_crossbar",
    )
    handle.visual(
        Cylinder(radius=0.024, length=0.260),
        origin=Origin(xyz=(0.0, 0.0, 0.760), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="rubber_grip",
    )
    handle.visual(
        Cylinder(radius=0.014, length=0.470),
        origin=Origin(xyz=(0.0, 0.0, 0.055), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="lower_crossbar",
    )
    model.articulation(
        "rear_handle_hinge",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=handle,
        origin=Origin(xyz=(-deck_length / 2.0 - 0.024, 0.0, deck_top_z + 0.0425)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.5, lower=0.0, upper=pi / 2.0),
    )

    extension = model.part("front_extension")
    extension.visual(
        Box((0.500, 0.380, 0.035)),
        origin=Origin(xyz=(-0.220, 0.0, -0.045)),
        material=steel,
        name="extension_panel",
    )
    extension.visual(
        Box((0.470, 0.350, 0.006)),
        origin=Origin(xyz=(-0.220, 0.0, -0.0245)),
        material=rubber,
        name="extension_grip",
    )
    extension.visual(
        Box((0.034, 0.420, 0.060)),
        origin=Origin(xyz=(0.040, 0.0, -0.025)),
        material=steel,
        name="front_nose",
    )
    extension.visual(
        Box((0.470, 0.068, 0.011)),
        origin=Origin(xyz=(-0.210, -0.145, -0.032)),
        material=zinc,
        name="rail_clip_bridge_0",
    )
    extension.visual(
        Box((0.470, 0.010, 0.034)),
        origin=Origin(xyz=(-0.210, -0.115, -0.017)),
        material=zinc,
        name="rail_clip_lip_0_0",
    )
    extension.visual(
        Box((0.470, 0.010, 0.034)),
        origin=Origin(xyz=(-0.210, -0.175, -0.017)),
        material=zinc,
        name="rail_clip_lip_0_1",
    )
    extension.visual(
        Box((0.470, 0.068, 0.011)),
        origin=Origin(xyz=(-0.210, 0.145, -0.032)),
        material=zinc,
        name="rail_clip_bridge_1",
    )
    extension.visual(
        Box((0.470, 0.010, 0.034)),
        origin=Origin(xyz=(-0.210, 0.115, -0.017)),
        material=zinc,
        name="rail_clip_lip_1_0",
    )
    extension.visual(
        Box((0.470, 0.010, 0.034)),
        origin=Origin(xyz=(-0.210, 0.175, -0.017)),
        material=zinc,
        name="rail_clip_lip_1_1",
    )
    for i, y in enumerate((-0.140, 0.140)):
        extension.visual(
            Box((0.060, 0.030, 0.030)),
            origin=Origin(xyz=(0.060, y, -0.003)),
            material=warning,
            name=f"spring_tab_{i}",
        )
    model.articulation(
        "front_extension_slide",
        ArticulationType.PRISMATIC,
        parent=deck,
        child=extension,
        origin=Origin(xyz=(deck_length / 2.0 + 0.010, 0.0, deck_bottom_z - 0.037)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.35, lower=0.0, upper=0.280),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("main_deck")
    handle = object_model.get_part("rear_handle")
    extension = object_model.get_part("front_extension")
    handle_hinge = object_model.get_articulation("rear_handle_hinge")
    extension_slide = object_model.get_articulation("front_extension_slide")

    for i in range(2):
        ctx.allow_overlap(
            deck,
            handle,
            elem_a=f"hinge_pin_shadow_{i}",
            elem_b=f"hinge_sleeve_{i}",
            reason="The rear handle hinge sleeve is intentionally captured around the fixed hinge pin.",
        )
        ctx.expect_overlap(
            handle,
            deck,
            axes="y",
            elem_a=f"hinge_sleeve_{i}",
            elem_b=f"hinge_pin_shadow_{i}",
            min_overlap=0.020,
            name=f"handle hinge sleeve {i} surrounds pin",
        )

    for i in range(4):
        fork_name = f"swivel_fork_{i}"
        wheel_name = f"wheel_{i}"
        ctx.allow_overlap(
            fork_name,
            wheel_name,
            elem_a="wheel_axle",
            elem_b="rim",
            reason="The caster wheel hub is intentionally captured on the axle through its center bore.",
        )
        ctx.expect_overlap(
            fork_name,
            wheel_name,
            axes="x",
            elem_a="wheel_axle",
            elem_b="rim",
            min_overlap=0.040,
            name=f"caster wheel {i} hub stays on axle",
        )

    for i in range(2):
        ctx.expect_overlap(
            extension,
            deck,
            axes="x",
            elem_a=f"rail_clip_bridge_{i}",
            elem_b=f"guide_rail_{i}",
            min_overlap=0.20,
            name=f"collapsed extension clip {i} grips rail",
        )

    rest_pos = ctx.part_world_position(extension)
    with ctx.pose({extension_slide: 0.280}):
        extended_pos = ctx.part_world_position(extension)
        for i in range(2):
            ctx.expect_overlap(
                extension,
                deck,
                axes="x",
                elem_a=f"rail_clip_bridge_{i}",
                elem_b=f"guide_rail_{i}",
                min_overlap=0.10,
                name=f"extended extension clip {i} remains captured",
            )
        ctx.expect_gap(
            deck,
            extension,
            axis="z",
            positive_elem="guide_rail_1",
            negative_elem="rail_clip_bridge_1",
            min_gap=0.0,
            max_gap=0.004,
            name="extension clip bridge rides just below the rail",
        )
        ctx.expect_gap(
            deck,
            extension,
            axis="y",
            positive_elem="guide_rail_1",
            negative_elem="rail_clip_lip_1_0",
            min_gap=0.006,
            max_gap=0.018,
            name="extension clip lip brackets side of rail",
        )
        ctx.expect_overlap(
            extension,
            deck,
            axes="z",
            elem_a="rail_clip_lip_1_0",
            elem_b="guide_rail_1",
            min_overlap=0.012,
            name="extension clip lip rises beside rail",
        )

    ctx.check(
        "front extension translates outward",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.25,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    with ctx.pose({handle_hinge: pi / 2.0}):
        ctx.expect_overlap(
            handle,
            deck,
            axes="x",
            min_overlap=0.55,
            elem_a="handle_post_0",
            elem_b="deck_panel",
            name="folded handle post lies over deck length",
        )
        ctx.expect_gap(
            handle,
            deck,
            axis="z",
            positive_elem="top_crossbar",
            negative_elem="deck_panel",
            min_gap=0.015,
            max_gap=0.090,
            name="folded handle clears top deck",
        )

    return ctx.report()


object_model = build_object_model()
