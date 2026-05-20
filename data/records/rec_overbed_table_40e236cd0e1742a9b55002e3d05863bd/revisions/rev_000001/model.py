from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_overbed_table")

    metal = Material("satin_black_metal", rgba=(0.05, 0.055, 0.06, 1.0))
    dark_metal = Material("dark_steel", rgba=(0.16, 0.17, 0.18, 1.0))
    rubber = Material("matte_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    grey = Material("brushed_grey", rgba=(0.55, 0.57, 0.58, 1.0))
    wood = Material("warm_laminate", rgba=(0.72, 0.54, 0.34, 1.0))
    lip_mat = Material("soft_front_lip", rgba=(0.08, 0.08, 0.075, 1.0))

    # Root floor frame: a long, narrow, offset under-bed runner.  The column is
    # close to one end of the base, while the tray support above remains centered
    # on the column.
    base = model.part("base")
    base.visual(
        Box((0.18, 0.72, 0.030)),
        origin=Origin(xyz=(0.0, -0.15, 0.085)),
        material=metal,
        name="offset_runner",
    )
    base.visual(
        Box((0.24, 0.18, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=dark_metal,
        name="column_foot",
    )

    # Hollow square outer sleeve, made from four connected walls so the
    # telescoping inner mast has real clearance instead of occupying a solid tube.
    sleeve_z = 0.31
    sleeve_len = 0.42
    for x, name in ((-0.028, "sleeve_side_0"), (0.028, "sleeve_side_1")):
        base.visual(
            Box((0.006, 0.056, sleeve_len)),
            origin=Origin(xyz=(x, 0.0, sleeve_z)),
            material=metal,
            name=name,
        )
    for y, name in ((-0.028, "sleeve_face_0"), (0.028, "sleeve_face_1")):
        base.visual(
            Box((0.056, 0.006, sleeve_len)),
            origin=Origin(xyz=(0.0, y, sleeve_z)),
            material=metal,
            name=name,
        )
    for x, name in ((-0.034, "collar_side_0"), (0.034, "collar_side_1")):
        base.visual(
            Box((0.008, 0.076, 0.012)),
            origin=Origin(xyz=(x, 0.0, 0.523)),
            material=grey,
            name=name,
        )
    for y, name in ((-0.034, "collar_face_0"), (0.034, "collar_face_1")):
        base.visual(
            Box((0.076, 0.008, 0.012)),
            origin=Origin(xyz=(0.0, y, 0.523)),
            material=grey,
            name=name,
        )

    # Low-friction guide pads just inside the sleeve make the telescoping fit
    # visibly supported while leaving the mast free to slide vertically.
    for x, name in ((-0.020, "guide_pad_0"), (0.020, "guide_pad_1")):
        base.visual(
            Box((0.010, 0.016, 0.035)),
            origin=Origin(xyz=(x, 0.0, 0.480)),
            material=lip_mat,
            name=name,
        )

    # Four compact caster yokes are part of the fixed floor frame; each wheel is
    # a separate continuous child on its axle.
    caster_positions = (
        (-0.065, 0.15),
        (0.065, 0.15),
        (-0.065, -0.45),
        (0.065, -0.45),
    )
    for i, (x, y) in enumerate(caster_positions):
        base.visual(
            Cylinder(radius=0.008, length=0.046),
            origin=Origin(xyz=(x, y, 0.093)),
            material=grey,
            name=f"caster_stem_{i}",
        )
        for side, sx in (("inboard", x - 0.019), ("outboard", x + 0.019)):
            base.visual(
                Box((0.004, 0.048, 0.054)),
                origin=Origin(xyz=(sx, y, 0.053)),
                material=grey,
                name=f"caster_fork_{i}_{side}",
            )

    inner_column = model.part("inner_column")
    inner_column.visual(
        Box((0.030, 0.030, 0.620)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=grey,
        name="inner_mast",
    )
    inner_column.visual(
        Box((0.060, 0.044, 0.085)),
        origin=Origin(xyz=(0.0, 0.0, 0.345)),
        material=dark_metal,
        name="head_stem",
    )
    inner_column.visual(
        Box((0.180, 0.100, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.370)),
        material=dark_metal,
        name="support_head",
    )
    inner_column.visual(
        Cylinder(radius=0.014, length=0.205),
        origin=Origin(xyz=(0.0, 0.0, 0.400), rpy=(0.0, pi / 2.0, 0.0)),
        material=grey,
        name="tilt_hinge_barrel",
    )

    column_slide = model.articulation(
        "base_to_inner_column",
        ArticulationType.PRISMATIC,
        parent=base,
        child=inner_column,
        origin=Origin(xyz=(0.0, 0.0, 0.500)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.20, lower=0.0, upper=0.20),
    )

    main_top = model.part("main_top")
    main_top.visual(
        Box((0.620, 0.380, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=wood,
        name="tray_panel",
    )
    main_top.visual(
        Box((0.620, 0.026, 0.058)),
        origin=Origin(xyz=(0.0, -0.203, 0.061)),
        material=lip_mat,
        name="front_lip",
    )
    main_top.visual(
        Box((0.185, 0.070, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=dark_metal,
        name="tilt_leaf",
    )
    for j, y in enumerate((-0.105, 0.105)):
        main_top.visual(
            Box((0.060, 0.026, 0.010)),
            origin=Origin(xyz=(0.322, y, 0.014)),
            material=dark_metal,
            name=f"wing_bracket_{j}",
        )

    top_tilt = model.articulation(
        "column_to_main_top",
        ArticulationType.REVOLUTE,
        parent=inner_column,
        child=main_top,
        origin=Origin(xyz=(0.0, 0.0, 0.400)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.0, lower=-0.35, upper=0.75),
    )

    reading_wing = model.part("reading_wing")
    reading_wing.visual(
        Box((0.240, 0.280, 0.022)),
        origin=Origin(xyz=(0.120, 0.0, 0.0)),
        material=wood,
        name="wing_panel",
    )
    reading_wing.visual(
        Box((0.220, 0.018, 0.036)),
        origin=Origin(xyz=(0.120, -0.149, 0.019)),
        material=lip_mat,
        name="wing_lip",
    )
    reading_wing.visual(
        Box((0.030, 0.230, 0.008)),
        origin=Origin(xyz=(0.022, 0.0, -0.015)),
        material=dark_metal,
        name="hinge_leaf",
    )

    wing_hinge = model.articulation(
        "main_top_to_reading_wing",
        ArticulationType.REVOLUTE,
        parent=main_top,
        child=reading_wing,
        origin=Origin(xyz=(0.345, 0.0, 0.032)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.2, lower=0.0, upper=1.45),
    )

    for i, (x, y) in enumerate(caster_positions):
        caster = model.part(f"caster_{i}")
        caster.visual(
            Cylinder(radius=0.032, length=0.024),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=rubber,
            name="tire",
        )
        caster.visual(
            Cylinder(radius=0.014, length=0.034),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=grey,
            name="hub",
        )
        model.articulation(
            f"base_to_caster_{i}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=caster,
            origin=Origin(xyz=(x, y, 0.032)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=12.0),
        )

    # Keep local variables alive only through their model entries.
    _ = (column_slide, top_tilt, wing_hinge)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    inner_column = object_model.get_part("inner_column")
    main_top = object_model.get_part("main_top")
    reading_wing = object_model.get_part("reading_wing")

    column_slide = object_model.get_articulation("base_to_inner_column")
    top_tilt = object_model.get_articulation("column_to_main_top")
    wing_hinge = object_model.get_articulation("main_top_to_reading_wing")

    def _center_z(part_name: str, elem_name: str) -> float | None:
        box = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if box is None:
            return None
        lower, upper = box
        return 0.5 * (lower[2] + upper[2])

    # The tray hinge/head is centered on the telescoping column, not hidden in a
    # cantilever arm.
    ctx.expect_origin_distance(
        main_top,
        inner_column,
        axes="xy",
        max_dist=0.002,
        name="main top hinge centered over column",
    )
    ctx.expect_contact(
        main_top,
        inner_column,
        elem_a="tilt_leaf",
        elem_b="tilt_hinge_barrel",
        name="tilt leaf bears on centered hinge barrel",
    )

    # The inner mast is guided by pads and stays retained in the hollow sleeve at
    # both travel limits.
    ctx.expect_contact(
        inner_column,
        base,
        elem_a="inner_mast",
        elem_b="guide_pad_0",
        name="inner mast contacts sleeve guide pad",
    )
    ctx.expect_overlap(
        inner_column,
        base,
        axes="z",
        elem_a="inner_mast",
        elem_b="sleeve_side_0",
        min_overlap=0.25,
        name="collapsed column remains deeply inserted",
    )

    rest_column_pos = ctx.part_world_position(inner_column)
    with ctx.pose({column_slide: 0.20}):
        raised_column_pos = ctx.part_world_position(inner_column)
        ctx.expect_contact(
            inner_column,
            base,
            elem_a="inner_mast",
            elem_b="guide_pad_0",
            name="raised mast still guided by sleeve pad",
        )
        ctx.expect_overlap(
            inner_column,
            base,
            axes="z",
            elem_a="inner_mast",
            elem_b="sleeve_side_0",
            min_overlap=0.09,
            name="raised column remains inserted in sleeve",
        )
    ctx.check(
        "column slide raises workstation height",
        rest_column_pos is not None
        and raised_column_pos is not None
        and raised_column_pos[2] > rest_column_pos[2] + 0.18,
        details=f"rest={rest_column_pos}, raised={raised_column_pos}",
    )

    # Front lip is a distinct raised retainer on the user side of the main tray.
    lip_z = _center_z("main_top", "front_lip")
    tray_z = _center_z("main_top", "tray_panel")
    ctx.check(
        "front retaining lip rises above tray panel",
        lip_z is not None and tray_z is not None and lip_z > tray_z + 0.025,
        details=f"front_lip_z={lip_z}, tray_panel_z={tray_z}",
    )

    rest_lip_z = lip_z
    with ctx.pose({top_tilt: 0.75}):
        tilted_lip_z = _center_z("main_top", "front_lip")
    ctx.check(
        "main top tilts upward at front lip",
        rest_lip_z is not None and tilted_lip_z is not None and tilted_lip_z > rest_lip_z + 0.10,
        details=f"rest={rest_lip_z}, tilted={tilted_lip_z}",
    )

    # The secondary reading wing is smaller, beside the tray, and supported by a
    # short bracket/hinge at the tray edge.
    ctx.expect_gap(
        reading_wing,
        main_top,
        axis="x",
        positive_elem="wing_panel",
        negative_elem="tray_panel",
        min_gap=0.025,
        max_gap=0.050,
        name="reading wing sits beside main tray",
    )
    ctx.expect_contact(
        reading_wing,
        main_top,
        elem_a="hinge_leaf",
        elem_b="wing_bracket_0",
        name="reading wing hinge leaf meets edge bracket",
    )
    rest_wing_z = _center_z("reading_wing", "wing_panel")
    with ctx.pose({wing_hinge: 1.45}):
        raised_wing_z = _center_z("reading_wing", "wing_panel")
    ctx.check(
        "reading wing rotates upward on side hinge",
        rest_wing_z is not None and raised_wing_z is not None and raised_wing_z > rest_wing_z + 0.10,
        details=f"rest={rest_wing_z}, raised={raised_wing_z}",
    )

    for i in range(4):
        caster = object_model.get_part(f"caster_{i}")
        caster_joint = object_model.get_articulation(f"base_to_caster_{i}")
        ctx.check(
            f"caster_{i} has continuous axle rotation",
            caster_joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(caster_joint.axis) == (1.0, 0.0, 0.0),
            details=f"type={caster_joint.articulation_type}, axis={caster_joint.axis}",
        )
        ctx.expect_contact(
            caster,
            base,
            elem_a="hub",
            elem_b=f"caster_fork_{i}_inboard",
            name=f"caster_{i} axle hub seated in fork",
        )
        tire_box = ctx.part_element_world_aabb(caster, elem="tire")
        ctx.check(
            f"caster_{i} tire reaches floor",
            tire_box is not None and abs(tire_box[0][2]) < 0.002,
            details=f"tire_aabb={tire_box}",
        )

    return ctx.report()


object_model = build_object_model()
