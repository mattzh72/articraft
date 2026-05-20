from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    WheelBore,
    WheelGeometry,
    WheelHub,
    WheelRim,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="home_care_overbed_table")

    powder_coat = model.material("warm_white_powder_coat", rgba=(0.82, 0.84, 0.80, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    laminate = model.material("maple_laminate", rgba=(0.78, 0.64, 0.43, 1.0))
    dark_edge = model.material("dark_vinyl_edge", rgba=(0.10, 0.11, 0.11, 1.0))
    rubber = model.material("black_rubber", rgba=(0.012, 0.012, 0.010, 1.0))
    knob_mat = model.material("charcoal_knob", rgba=(0.045, 0.050, 0.055, 1.0))

    post_x = -0.34
    post_y = -0.18
    sleeve_top_z = 0.60

    base = model.part("base")
    # Offset low base: a long under-bed rail with a wider patient-side crossbar
    # and a shorter rear stabilizer under the side-mounted pedestal.
    base.visual(
        Box((1.06, 0.070, 0.035)),
        origin=Origin(xyz=(0.16, 0.0, 0.095)),
        material=powder_coat,
        name="underbed_rail",
    )
    base.visual(
        Box((0.17, 0.52, 0.035)),
        origin=Origin(xyz=(0.56, 0.0, 0.095)),
        material=powder_coat,
        name="patient_crossbar",
    )
    base.visual(
        Box((0.25, 0.40, 0.035)),
        origin=Origin(xyz=(post_x, post_y + 0.04, 0.095)),
        material=powder_coat,
        name="post_side_foot",
    )
    base.visual(
        Box((0.20, 0.20, 0.025)),
        origin=Origin(xyz=(post_x, post_y, 0.125)),
        material=powder_coat,
        name="pedestal_plate",
    )
    for i, y in enumerate((post_y - 0.10, post_y + 0.18)):
        base.visual(
            Box((0.075, 0.070, 0.035)),
            origin=Origin(xyz=(post_x, y, 0.060)),
            material=rubber,
            name=f"rear_glide_{i}",
        )

    # The lower post is a visibly hollow square sleeve so the sliding mast can
    # telescope inside it without being modeled as a solid-on-solid collision.
    sleeve_len = sleeve_top_z - 0.11
    sleeve_center_z = 0.11 + sleeve_len / 2.0
    base.visual(
        Box((0.008, 0.070, sleeve_len)),
        origin=Origin(xyz=(post_x - 0.031, post_y, sleeve_center_z)),
        material=powder_coat,
        name="sleeve_wall_x0",
    )
    base.visual(
        Box((0.008, 0.070, sleeve_len)),
        origin=Origin(xyz=(post_x + 0.031, post_y, sleeve_center_z)),
        material=powder_coat,
        name="sleeve_wall_x1",
    )
    base.visual(
        Box((0.070, 0.008, sleeve_len)),
        origin=Origin(xyz=(post_x, post_y - 0.031, sleeve_center_z)),
        material=powder_coat,
        name="sleeve_wall_y0",
    )
    base.visual(
        Box((0.070, 0.008, sleeve_len)),
        origin=Origin(xyz=(post_x, post_y + 0.031, sleeve_center_z)),
        material=powder_coat,
        name="sleeve_wall_y1",
    )

    # Clamp collar, boss, and exposed threaded hub for the rotating hand knob.
    collar_z = sleeve_top_z - 0.010
    base.visual(
        Box((0.094, 0.014, 0.076)),
        origin=Origin(xyz=(post_x, post_y - 0.042, collar_z)),
        material=brushed_steel,
        name="collar_front_band",
    )
    base.visual(
        Box((0.094, 0.014, 0.076)),
        origin=Origin(xyz=(post_x, post_y + 0.042, collar_z)),
        material=brushed_steel,
        name="collar_rear_band",
    )
    base.visual(
        Box((0.014, 0.094, 0.076)),
        origin=Origin(xyz=(post_x - 0.042, post_y, collar_z)),
        material=brushed_steel,
        name="collar_side_band_0",
    )
    base.visual(
        Box((0.014, 0.094, 0.076)),
        origin=Origin(xyz=(post_x + 0.042, post_y, collar_z)),
        material=brushed_steel,
        name="collar_side_band_1",
    )
    base.visual(
        Box((0.036, 0.014, 0.050)),
        origin=Origin(xyz=(post_x, post_y - 0.053, collar_z)),
        material=brushed_steel,
        name="collar_boss",
    )
    base.visual(
        Cylinder(radius=0.012, length=0.050),
        origin=Origin(xyz=(post_x, post_y - 0.080, collar_z), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="threaded_hub",
    )

    # Fork cheeks for the two small front casters.  They are tucked under the
    # patient-side crossbar but leave clearance for the rotating wheels.
    caster_x = 0.56
    caster_z = 0.038
    for idx, y in enumerate((-0.18, 0.18)):
        base.visual(
            Box((0.026, 0.006, 0.058)),
            origin=Origin(xyz=(caster_x, y - 0.020, 0.053)),
            material=brushed_steel,
            name=f"caster_fork_{idx}_0",
        )
        base.visual(
            Box((0.026, 0.006, 0.058)),
            origin=Origin(xyz=(caster_x, y + 0.020, 0.053)),
            material=brushed_steel,
            name=f"caster_fork_{idx}_1",
        )
        base.visual(
            Cylinder(radius=0.005, length=0.020),
            origin=Origin(xyz=(caster_x, y - 0.032, caster_z), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name=f"caster_axle_cap_{idx}_0",
        )
        base.visual(
            Cylinder(radius=0.005, length=0.020),
            origin=Origin(xyz=(caster_x, y + 0.032, caster_z), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name=f"caster_axle_cap_{idx}_1",
        )
        base.visual(
            Cylinder(radius=0.0036, length=0.050),
            origin=Origin(xyz=(caster_x, y, caster_z), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name=f"caster_axle_shaft_{idx}",
        )

    upper_column = model.part("upper_column")
    upper_column.visual(
        Box((0.044, 0.044, 0.550)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=brushed_steel,
        name="inner_mast",
    )
    upper_column.visual(
        Box((0.005, 0.030, 0.105)),
        origin=Origin(xyz=(-0.0245, 0.0, -0.145)),
        material=dark_edge,
        name="guide_pad_0",
    )
    upper_column.visual(
        Box((0.005, 0.030, 0.105)),
        origin=Origin(xyz=(0.0245, 0.0, -0.145)),
        material=dark_edge,
        name="guide_pad_1",
    )
    upper_column.visual(
        Box((0.32, 0.046, 0.040)),
        origin=Origin(xyz=(0.16, 0.0, 0.200)),
        material=powder_coat,
        name="cantilever_arm",
    )
    upper_column.visual(
        Box((0.075, 0.31, 0.032)),
        origin=Origin(xyz=(0.285, 0.155, 0.204)),
        material=powder_coat,
        name="tray_cross_arm",
    )
    upper_column.visual(
        Box((0.035, 0.035, 0.20)),
        origin=Origin(xyz=(0.025, 0.0, 0.105), rpy=(0.0, pi / 7.0, 0.0)),
        material=powder_coat,
        name="sloped_gusset",
    )

    model.articulation(
        "base_to_column",
        ArticulationType.PRISMATIC,
        parent=base,
        child=upper_column,
        origin=Origin(xyz=(post_x, post_y, sleeve_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.18, lower=0.0, upper=0.28),
    )

    main_top = model.part("main_top")
    main_top.visual(
        Box((0.760, 0.420, 0.035)),
        origin=Origin(xyz=(0.380, 0.0, 0.0175)),
        material=laminate,
        name="top_panel",
    )
    main_top.visual(
        Box((0.790, 0.018, 0.046)),
        origin=Origin(xyz=(0.380, -0.219, 0.023)),
        material=dark_edge,
        name="near_edge_band",
    )
    main_top.visual(
        Box((0.790, 0.018, 0.046)),
        origin=Origin(xyz=(0.380, 0.219, 0.023)),
        material=dark_edge,
        name="wing_edge_band",
    )
    main_top.visual(
        Box((0.018, 0.456, 0.046)),
        origin=Origin(xyz=(-0.009, 0.0, 0.023)),
        material=dark_edge,
        name="post_edge_band",
    )
    main_top.visual(
        Box((0.018, 0.456, 0.046)),
        origin=Origin(xyz=(0.769, 0.0, 0.023)),
        material=dark_edge,
        name="patient_edge_band",
    )
    main_top.visual(
        Box((0.230, 0.048, 0.016)),
        origin=Origin(xyz=(0.480, 0.235, -0.007)),
        material=brushed_steel,
        name="wing_support_bracket",
    )
    main_top.visual(
        Cylinder(radius=0.006, length=0.066),
        origin=Origin(xyz=(0.480, 0.246, 0.030)),
        material=brushed_steel,
        name="wing_hinge_pin",
    )
    main_top.visual(
        Box((0.160, 0.140, 0.012)),
        origin=Origin(xyz=(0.055, 0.0, -0.006)),
        material=brushed_steel,
        name="mounting_plate",
    )

    model.articulation(
        "column_to_top",
        ArticulationType.FIXED,
        parent=upper_column,
        child=main_top,
        origin=Origin(xyz=(0.300, 0.180, 0.232)),
    )

    wing = model.part("reading_wing")
    wing.visual(
        Box((0.340, 0.240, 0.026)),
        origin=Origin(xyz=(0.0, 0.137, 0.013)),
        material=laminate,
        name="wing_panel",
    )
    wing.visual(
        Box((0.360, 0.018, 0.040)),
        origin=Origin(xyz=(0.0, 0.263, 0.020)),
        material=dark_edge,
        name="book_stop_lip",
    )
    wing.visual(
        Box((0.026, 0.260, 0.036)),
        origin=Origin(xyz=(-0.183, 0.137, 0.018)),
        material=dark_edge,
        name="wing_end_band_0",
    )
    wing.visual(
        Box((0.026, 0.260, 0.036)),
        origin=Origin(xyz=(0.183, 0.137, 0.018)),
        material=dark_edge,
        name="wing_end_band_1",
    )
    wing.visual(
        Cylinder(radius=0.010, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=brushed_steel,
        name="hinge_knuckle",
    )
    wing.visual(
        Box((0.220, 0.030, 0.012)),
        origin=Origin(xyz=(0.0, 0.018, 0.006)),
        material=brushed_steel,
        name="hinge_leaf",
    )

    model.articulation(
        "top_to_wing",
        ArticulationType.REVOLUTE,
        parent=main_top,
        child=wing,
        origin=Origin(xyz=(0.480, 0.246, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=0.0, upper=1.57),
    )

    knob = model.part("collar_knob")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.060,
            0.032,
            body_style="lobed",
            base_diameter=0.036,
            top_diameter=0.056,
            crown_radius=0.002,
            grip=KnobGrip(style="ribbed", count=10, depth=0.0012),
            bore=KnobBore(style="round", diameter=0.012),
        ),
        "collar_knob_body",
    )
    knob.visual(
        knob_mesh,
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=knob_mat,
        name="knob_body",
    )
    model.articulation(
        "hub_to_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=knob,
        origin=Origin(xyz=(post_x, post_y - 0.121, collar_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=8.0),
    )

    caster_tire_mesh = mesh_from_geometry(
        TireGeometry(0.035, 0.024, inner_radius=0.024),
        "caster_tire",
    )
    for idx, y in enumerate((-0.18, 0.18)):
        caster = model.part(f"caster_{idx}")
        caster.visual(
            caster_tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, pi / 2.0)),
            material=rubber,
            name="tire",
        )
        caster.visual(
            Cylinder(radius=0.0248, length=0.026),
            origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name="wheel_rim",
        )
        caster.visual(
            Cylinder(radius=0.010, length=0.030),
            origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name="hub_cap",
        )
        model.articulation(
            f"base_to_caster_{idx}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=caster,
            origin=Origin(xyz=(caster_x, y, caster_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    column = object_model.get_part("upper_column")
    top = object_model.get_part("main_top")
    wing = object_model.get_part("reading_wing")
    caster_0 = object_model.get_part("caster_0")
    caster_1 = object_model.get_part("caster_1")

    height_slide = object_model.get_articulation("base_to_column")
    wing_hinge = object_model.get_articulation("top_to_wing")

    ctx.allow_overlap(
        top,
        wing,
        elem_a="wing_hinge_pin",
        elem_b="hinge_knuckle",
        reason="The reading-wing hinge pin is intentionally captured inside the rotating knuckle.",
    )
    ctx.expect_within(
        top,
        wing,
        axes="xy",
        inner_elem="wing_hinge_pin",
        outer_elem="hinge_knuckle",
        margin=0.001,
        name="reading wing hinge pin is centered in its knuckle",
    )
    ctx.expect_overlap(
        top,
        wing,
        axes="z",
        elem_a="wing_hinge_pin",
        elem_b="hinge_knuckle",
        min_overlap=0.055,
        name="reading wing hinge pin spans the short knuckle",
    )

    for idx, caster in enumerate((caster_0, caster_1)):
        ctx.allow_overlap(
            base,
            caster,
            elem_a=f"caster_axle_shaft_{idx}",
            elem_b="wheel_rim",
            reason="The caster axle shaft is intentionally nested through the wheel bore.",
        )
        ctx.allow_overlap(
            base,
            caster,
            elem_a=f"caster_axle_shaft_{idx}",
            elem_b="hub_cap",
            reason="The caster axle continues through the central rotating hub cap.",
        )
        ctx.expect_within(
            base,
            caster,
            axes="xz",
            inner_elem=f"caster_axle_shaft_{idx}",
            outer_elem="wheel_rim",
            margin=0.002,
            name=f"caster {idx} axle is centered in the wheel bore",
        )
        ctx.expect_overlap(
            base,
            caster,
            axes="y",
            elem_a=f"caster_axle_shaft_{idx}",
            elem_b="wheel_rim",
            min_overlap=0.020,
            name=f"caster {idx} axle crosses the wheel hub",
        )
        ctx.expect_overlap(
            base,
            caster,
            axes="y",
            elem_a=f"caster_axle_shaft_{idx}",
            elem_b="hub_cap",
            min_overlap=0.020,
            name=f"caster {idx} axle is retained by the hub cap",
        )

    ctx.expect_gap(
        wing,
        top,
        axis="y",
        min_gap=0.0,
        max_gap=0.060,
        positive_elem="wing_panel",
        negative_elem="top_panel",
        name="reading wing sits just beside the main tray edge",
    )
    ctx.expect_overlap(
        wing,
        top,
        axes="x",
        min_overlap=0.24,
        elem_a="wing_panel",
        elem_b="top_panel",
        name="reading wing is mounted along the tray side span",
    )
    ctx.expect_gap(
        top,
        column,
        axis="z",
        max_gap=0.006,
        max_penetration=0.0,
        positive_elem="mounting_plate",
        negative_elem="tray_cross_arm",
        name="tray mounting plate seats on the cantilever arm",
    )

    rest_top_aabb = ctx.part_element_world_aabb(top, elem="top_panel")
    rest_column_pos = ctx.part_world_position(column)
    with ctx.pose({height_slide: 0.28}):
        raised_top_aabb = ctx.part_element_world_aabb(top, elem="top_panel")
        raised_column_pos = ctx.part_world_position(column)
    ctx.check(
        "prismatic post raises the tray by the height travel",
        rest_top_aabb is not None
        and raised_top_aabb is not None
        and rest_column_pos is not None
        and raised_column_pos is not None
        and raised_top_aabb[0][2] > rest_top_aabb[0][2] + 0.25
        and raised_column_pos[2] > rest_column_pos[2] + 0.25,
        details=f"rest_top={rest_top_aabb}, raised_top={raised_top_aabb}, rest_column={rest_column_pos}, raised_column={raised_column_pos}",
    )

    rest_wing_panel = ctx.part_element_world_aabb(wing, elem="wing_panel")
    with ctx.pose({wing_hinge: 1.20}):
        swung_wing_panel = ctx.part_element_world_aabb(wing, elem="wing_panel")
    ctx.check(
        "reading wing swings on its short side hinge",
        rest_wing_panel is not None
        and swung_wing_panel is not None
        and swung_wing_panel[0][0] < rest_wing_panel[0][0] - 0.10,
        details=f"rest_wing={rest_wing_panel}, swung_wing={swung_wing_panel}",
    )

    for caster in (caster_0, caster_1):
        pos = ctx.part_world_position(caster)
        ctx.check(
            f"{caster.name} is a low front caster under the patient side",
            pos is not None and pos[0] > 0.45 and 0.02 < pos[2] < 0.06,
            details=f"caster_position={pos}",
        )

    ctx.check(
        "knob and caster wheel joints are continuous rotations",
        all(
            object_model.get_articulation(name).articulation_type == ArticulationType.CONTINUOUS
            for name in ("hub_to_knob", "base_to_caster_0", "base_to_caster_1")
        ),
        details="collar knob and the two wheel axles should be continuous rotational joints",
    )

    return ctx.report()


object_model = build_object_model()
