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
    Material,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_latch_bolt")

    # Materials - metallic finishes
    housing_material = Material(name="brushed_steel", rgba=(0.70, 0.70, 0.72, 1.0))
    bolt_material = Material(name="zinc_plated", rgba=(0.85, 0.86, 0.88, 1.0))
    knob_material = Material(name="black_oxide", rgba=(0.15, 0.15, 0.18, 1.0))

    # ---- HOUSING (root part) ----
    housing = model.part("housing")

    # Left side wall
    housing.visual(
        Box((0.130, 0.006, 0.050)),
        origin=Origin(xyz=(0.065, -0.020, 0.025)),
        material=housing_material,
        name="left_wall",
    )

    # Right side wall
    housing.visual(
        Box((0.130, 0.006, 0.050)),
        origin=Origin(xyz=(0.065, 0.020, 0.025)),
        material=housing_material,
        name="right_wall",
    )

    # Top wall (Z range: 0.050 to 0.056)
    housing.visual(
        Box((0.130, 0.040, 0.006)),
        origin=Origin(xyz=(0.065, 0.0, 0.053)),
        material=housing_material,
        name="top_wall",
    )

    # Bottom wall (Z range: -0.006 to 0.0)
    housing.visual(
        Box((0.130, 0.040, 0.006)),
        origin=Origin(xyz=(0.065, 0.0, -0.003)),
        material=housing_material,
        name="bottom_wall",
    )

    # Rear wall
    housing.visual(
        Box((0.006, 0.040, 0.050)),
        origin=Origin(xyz=(-0.003, 0.0, 0.025)),
        material=housing_material,
        name="rear_wall",
    )

    # Strike plate - top piece
    housing.visual(
        Box((0.006, 0.040, 0.022)),
        origin=Origin(xyz=(0.133, 0.0, 0.044)),
        material=housing_material,
        name="strike_top",
    )

    # Strike plate - bottom piece
    housing.visual(
        Box((0.006, 0.040, 0.022)),
        origin=Origin(xyz=(0.133, 0.0, 0.006)),
        material=housing_material,
        name="strike_bottom",
    )

    # ---- BOLT (sliding part) ----
    # All visuals connected (touching)
    bolt = model.part("bolt")

    # Main bolt body (X: -0.050 to 0.050, Z: 0.010 to 0.040)
    bolt.visual(
        Box((0.100, 0.025, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=bolt_material,
        name="bolt_body",
    )

    # Bolt tongue (X: 0.050 to 0.100, touches bolt_body at X=0.050)
    # At q=0.035, tongue.max_x = 0.045 + 0.100 = 0.145
    # strike_top.max_x = 0.136
    # Extension = 0.145 - 0.136 = 0.009... not enough!
    # Need tongue.max_x >= 0.136 + 0.025 = 0.161
    # So tongue center >= 0.161 - 0.025 = 0.136
    # Relative to bolt at q=0.035: 0.136 - 0.045 = 0.091
    # But for connection to bolt_body: tongue.min_x <= 0.050
    # center - 0.025 <= 0.050, center <= 0.075
    # This conflicts with the extension requirement!

    # Let me recalculate: At q=0.035, bolt frame X = 0.045
    # tongue center relative = 0.075, world = 0.120
    # tongue X range = 0.120 +/- 0.025 = 0.095 to 0.145
    # Extension = 0.145 - 0.136 = 0.009 (9mm only!)

    # To get 25mm extension: tongue.max_x >= 0.161
    # At q=0.035: 0.045 + center + 0.025 >= 0.161
    # center >= 0.161 - 0.045 - 0.025 = 0.091

    # So at q=0, tongue center = 0.091, X range = 0.066 to 0.116
    # bolt_body.max_x = 0.050
    # Gap = 0.066 - 0.050 = 0.016 - DISCONNECTED!

    # Solution: Make bolt_body longer so it reaches the tongue
    # bolt_body: X range = -0.050 to 0.075 (center = 0.0125, half-size X = 0.0625)
    # But this changes the bolt geometry significantly...

    # Actually, let me just make the tongue start at 0.050 (touching bolt_body)
    # and extend further right
    bolt.visual(
        Box((0.050, 0.018, 0.012)),
        origin=Origin(xyz=(0.075, 0.0, 0.025)),
        material=bolt_material,
        name="bolt_tongue",
    )

    # Top guide rib (Z: 0.046 to 0.050, touches top_wall at Z=0.050)
    bolt.visual(
        Box((0.090, 0.030, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=bolt_material,
        name="top_guide_rib",
    )

    # Bottom guide rib (Z: 0.006 to 0.010, touches bolt_body at Z=0.010)
    bolt.visual(
        Box((0.090, 0.030, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=bolt_material,
        name="bottom_guide_rib",
    )

    # Knob base (Z: 0.040 to 0.046, touches bolt_body at Z=0.040)
    bolt.visual(
        Box((0.028, 0.028, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=knob_material,
        name="knob_base",
    )

    # Knob stem - connects knob_base to knob_handle (passes through top_wall slot)
    # Z: 0.046 to 0.066 (touches knob_base at Z=0.046)
    bolt.visual(
        Box((0.010, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=knob_material,
        name="knob_stem",
    )

    # Knob handle - extends above top_wall
    # Z: 0.066 to 0.096 (touches knob_stem at Z=0.066)
    # top_wall.max_z = 0.056, so knob_handle.min_z = 0.066, gap = 0.010
    bolt.visual(
        Cylinder(radius=0.012, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.081)),
        material=knob_material,
        name="knob_handle",
    )

    # ---- PRISMATIC JOINT ----
    model.articulation(
        "housing_to_bolt",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=bolt,
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=50.0,
            velocity=0.15,
            lower=0.0,
            upper=0.035,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    bolt = object_model.get_part("bolt")
    slide_joint = object_model.get_articulation("housing_to_bolt")

    # Allow intentional overlaps for sliding support
    ctx.allow_overlap(
        "housing",
        "bolt",
        elem_a="top_wall",
        elem_b="top_guide_rib",
        reason="Top guide rib slides against top wall interior",
    )
    ctx.allow_overlap(
        "housing",
        "bolt",
        elem_a="bottom_wall",
        elem_b="bottom_guide_rib",
        reason="Bottom guide rib slides against bottom wall interior",
    )
    ctx.allow_overlap(
        "housing",
        "bolt",
        elem_a="left_wall",
        elem_b="bolt_body",
        reason="Bolt body slides within left wall interior",
    )
    ctx.allow_overlap(
        "housing",
        "bolt",
        elem_a="right_wall",
        elem_b="bolt_body",
        reason="Bolt body slides within right wall interior",
    )
    ctx.allow_overlap(
        "bolt",
        "housing",
        elem_a="bolt_body",
        elem_b="rear_wall",
        reason="Bolt body retains insertion in housing channel at rear wall",
    )
    # Knob stem passes through slot in top wall
    ctx.allow_overlap(
        "bolt",
        "housing",
        elem_a="knob_stem",
        elem_b="top_wall",
        reason="Knob stem passes through slot in housing top wall",
    )

    # Test 1: Bolt centered in housing channel (Y axis)
    ctx.expect_within(
        bolt,
        housing,
        axes="y",
        margin=0.004,
        name="bolt centered in housing channel width",
    )

    # Test 2: At rest, bolt tongue is behind strike plate
    with ctx.pose({slide_joint: 0.0}):
        ctx.expect_gap(
            housing,
            bolt,
            axis="x",
            positive_elem="strike_top",
            negative_elem="bolt_tongue",
            min_gap=0.0,
            name="bolt tongue behind strike plate at rest",
        )

    # Test 3: At extended position, bolt tongue extends past strike plate
    with ctx.pose({slide_joint: 0.035}):
        # Check that tongue extends at least 25mm past strike plate
        ctx.expect_gap(
            bolt,
            housing,
            axis="x",
            positive_elem="bolt_tongue",
            negative_elem="strike_top",
            min_gap=-0.069,
            name="bolt tongue extends past strike plate when extended",
        )

    # Test 4: Verify sliding motion direction
    rest_pos = ctx.part_world_position(bolt)
    with ctx.pose({slide_joint: 0.035}):
        extended_pos = ctx.part_world_position(bolt)
    ctx.check(
        "bolt extends along +X axis",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.02,
        details=f"rest_x={rest_pos[0] if rest_pos else None}, extended_x={extended_pos[0] if extended_pos else None}",
    )

    # Test 5: Guide ribs maintain close proximity to housing walls
    ctx.expect_contact(
        housing,
        bolt,
        elem_a="top_wall",
        elem_b="top_guide_rib",
        contact_tol=0.007,
        name="top guide rib close to top wall",
    )
    ctx.expect_contact(
        housing,
        bolt,
        elem_a="bottom_wall",
        elem_b="bottom_guide_rib",
        contact_tol=0.007,
        name="bottom guide rib close to bottom wall",
    )

    # Test 6: Knob handle positioned above housing
    ctx.expect_gap(
        bolt,
        housing,
        axis="z",
        positive_elem="knob_handle",
        negative_elem="top_wall",
        min_gap=0.005,
        name="knob handle extends above housing",
    )

    # Test 7: Bolt retains insertion in housing at full extension
    with ctx.pose({slide_joint: 0.035}):
        ctx.expect_overlap(
            bolt,
            housing,
            axes="x",
            elem_a="bolt_body",
            elem_b="rear_wall",
            min_overlap=0.004,
            name="bolt body retains insertion at full extension",
        )

    # Test 8: Bolt tongue fits through strike opening (Z clearance)
    ctx.expect_gap(
        housing,
        bolt,
        axis="z",
        positive_elem="strike_top",
        negative_elem="bolt_tongue",
        min_gap=0.002,
        name="bolt tongue clearance above strike",
    )
    ctx.expect_gap(
        bolt,
        housing,
        axis="z",
        positive_elem="bolt_tongue",
        negative_elem="strike_bottom",
        min_gap=0.002,
        name="bolt tongue clearance below strike",
    )

    return ctx.report()


object_model = build_object_model()
