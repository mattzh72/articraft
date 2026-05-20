from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _xz_extrusion(profile: list[tuple[float, float]], thickness: float) -> object:
    """Extrude an X/Z side profile through local Y."""
    geom = ExtrudeGeometry(profile, thickness, center=True, closed=True)
    geom.rotate_x(math.pi / 2.0)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_magazine_utility_knife")

    safety_orange = model.material("safety_orange", rgba=(0.95, 0.30, 0.05, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    blackened_steel = model.material("blackened_steel", rgba=(0.05, 0.055, 0.060, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.86, 0.86, 0.82, 1.0))

    handle = model.part("handle")
    side_profile = [
        (-0.080, 0.006),
        (-0.074, 0.030),
        (-0.040, 0.034),
        (0.058, 0.034),
        (0.084, 0.024),
        (0.089, 0.015),
        (0.076, 0.006),
        (-0.054, 0.002),
    ]
    side_plate_mesh = mesh_from_geometry(_xz_extrusion(side_profile, 0.0022), "handle_side_plate")
    handle.visual(
        side_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0100, 0.0)),
        material=safety_orange,
        name="side_plate_0",
    )
    handle.visual(
        side_plate_mesh,
        origin=Origin(xyz=(0.0, -0.0100, 0.0)),
        material=safety_orange,
        name="side_plate_1",
    )
    handle.visual(
        Box((0.145, 0.020, 0.0045)),
        origin=Origin(xyz=(0.006, 0.0, 0.0042)),
        material=blackened_steel,
        name="bottom_spine",
    )
    for y, name in ((0.0076, "top_rail_0"), (-0.0076, "top_rail_1")):
        handle.visual(
            Box((0.103, 0.0052, 0.0034)),
            origin=Origin(xyz=(0.010, y, 0.0320)),
            material=blackened_steel,
            name=name,
        )
    handle.visual(
        Box((0.027, 0.020, 0.006)),
        origin=Origin(xyz=(0.076, 0.0, 0.0302)),
        material=blackened_steel,
        name="nose_upper_guide",
    )
    handle.visual(
        Box((0.027, 0.020, 0.005)),
        origin=Origin(xyz=(0.076, 0.0, 0.0060)),
        material=blackened_steel,
        name="nose_lower_guide",
    )
    handle.visual(
        Box((0.012, 0.020, 0.0045)),
        origin=Origin(xyz=(-0.072, 0.0, 0.0305)),
        material=blackened_steel,
        name="tail_top_lip",
    )
    handle.visual(
        Box((0.012, 0.020, 0.0045)),
        origin=Origin(xyz=(-0.072, 0.0, 0.0042)),
        material=blackened_steel,
        name="tail_bottom_lip",
    )
    for y, name in ((0.0114, "grip_insert_0"), (-0.0114, "grip_insert_1")):
        handle.visual(
            Box((0.078, 0.0012, 0.012)),
            origin=Origin(xyz=(-0.014, y, 0.015)),
            material=dark_rubber,
            name=name,
        )
    for i, x in enumerate((-0.047, -0.008, 0.031, 0.066)):
        handle.visual(
            Cylinder(radius=0.0032, length=0.0026),
            origin=Origin(xyz=(x, 0.0119, 0.024), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=blackened_steel,
            name=f"screw_head_{i}",
        )
    handle.visual(
        Cylinder(radius=0.0030, length=0.0058),
        origin=Origin(xyz=(-0.082, 0.0068, 0.0060), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blackened_steel,
        name="tail_hinge_knuckle_0",
    )
    handle.visual(
        Cylinder(radius=0.0030, length=0.0058),
        origin=Origin(xyz=(-0.082, -0.0068, 0.0060), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blackened_steel,
        name="tail_hinge_knuckle_1",
    )

    blade_carriage = model.part("blade_carriage")
    blade_carriage.visual(
        Box((0.070, 0.0070, 0.0060)),
        origin=Origin(xyz=(0.005, 0.0, 0.0150)),
        material=blackened_steel,
        name="slide_rail",
    )
    blade_carriage.visual(
        Box((0.048, 0.0060, 0.0020)),
        origin=Origin(xyz=(0.000, 0.0, 0.00745)),
        material=blackened_steel,
        name="lower_guide_shoe",
    )
    blade_carriage.visual(
        Box((0.045, 0.0040, 0.0042)),
        origin=Origin(xyz=(0.000, 0.0, 0.01040)),
        material=blackened_steel,
        name="guide_web",
    )
    blade_carriage.visual(
        Box((0.018, 0.0060, 0.0100)),
        origin=Origin(xyz=(0.044, 0.0, 0.0170)),
        material=brushed_steel,
        name="blade_clamp",
    )
    blade_profile = [
        (0.045, 0.011),
        (0.081, 0.011),
        (0.099, 0.016),
        (0.082, 0.023),
        (0.047, 0.023),
    ]
    blade_mesh = mesh_from_geometry(_xz_extrusion(blade_profile, 0.0016), "trapezoid_blade")
    blade_carriage.visual(blade_mesh, material=blade_steel, name="blade")
    blade_carriage.visual(
        Box((0.024, 0.0004, 0.0010)),
        origin=Origin(xyz=(0.066, 0.0010, 0.0212), rpy=(0.0, -0.22, 0.0)),
        material=blackened_steel,
        name="blade_score_line",
    )
    blade_carriage.visual(
        Box((0.008, 0.0035, 0.0180)),
        origin=Origin(xyz=(0.000, 0.0, 0.0270)),
        material=blackened_steel,
        name="thumb_stem",
    )
    blade_carriage.visual(
        Box((0.022, 0.0140, 0.0050)),
        origin=Origin(xyz=(0.000, 0.0, 0.0375)),
        material=dark_rubber,
        name="thumb_slider",
    )
    for i, x in enumerate((-0.006, 0.0, 0.006)):
        blade_carriage.visual(
            Box((0.0020, 0.0110, 0.0010)),
            origin=Origin(xyz=(x, 0.0, 0.0405)),
            material=blackened_steel,
            name=f"thumb_rib_{i}",
        )

    spare_tray = model.part("spare_tray")
    spare_tray.visual(
        Box((0.0040, 0.0180, 0.0320)),
        origin=Origin(xyz=(-0.0020, 0.0, 0.0185)),
        material=safety_orange,
        name="tray_end_cap",
    )
    spare_tray.visual(
        Cylinder(radius=0.0028, length=0.0075),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blackened_steel,
        name="tray_knuckle",
    )
    for y, name in ((0.0067, "tray_side_lip_0"), (-0.0067, "tray_side_lip_1")):
        spare_tray.visual(
            Box((0.0080, 0.0012, 0.0180)),
            origin=Origin(xyz=(0.0030, y, 0.0170)),
            material=blackened_steel,
            name=name,
        )
    spare_tray.visual(
        Box((0.0012, 0.0120, 0.0210)),
        origin=Origin(xyz=(0.0005, 0.0, 0.0185)),
        material=blade_steel,
        name="spare_blade_stack",
    )
    for i, z in enumerate((0.0130, 0.0185, 0.0240)):
        spare_tray.visual(
            Box((0.0016, 0.0110, 0.0008)),
            origin=Origin(xyz=(0.0016, 0.0, z), rpy=(0.0, 0.25, 0.0)),
            material=brushed_steel,
            name=f"spare_blade_edge_{i}",
        )

    model.articulation(
        "handle_to_blade_carriage",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=blade_carriage,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.35, lower=0.0, upper=0.038),
    )
    model.articulation(
        "handle_to_spare_tray",
        ArticulationType.REVOLUTE,
        parent=handle,
        child=spare_tray,
        origin=Origin(xyz=(-0.082, 0.0, 0.0060)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.65),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    handle = object_model.get_part("handle")
    blade_carriage = object_model.get_part("blade_carriage")
    spare_tray = object_model.get_part("spare_tray")
    blade_slide = object_model.get_articulation("handle_to_blade_carriage")
    tray_hinge = object_model.get_articulation("handle_to_spare_tray")

    ctx.check(
        "blade carriage has prismatic handle travel",
        blade_slide.articulation_type == ArticulationType.PRISMATIC
        and blade_slide.motion_limits is not None
        and blade_slide.motion_limits.upper >= 0.035,
        details=f"type={blade_slide.articulation_type}, limits={blade_slide.motion_limits}",
    )
    ctx.check(
        "spare tray has fold out hinge travel",
        tray_hinge.articulation_type == ArticulationType.REVOLUTE
        and tray_hinge.motion_limits is not None
        and tray_hinge.motion_limits.upper >= 1.5,
        details=f"type={tray_hinge.articulation_type}, limits={tray_hinge.motion_limits}",
    )

    ctx.expect_contact(
        blade_carriage,
        handle,
        elem_a="lower_guide_shoe",
        elem_b="bottom_spine",
        contact_tol=1e-6,
        name="blade carriage rides on lower guide",
    )
    ctx.expect_within(
        blade_carriage,
        handle,
        axes="y",
        inner_elem="slide_rail",
        margin=0.001,
        name="slide rail stays between handle side plates",
    )
    ctx.expect_overlap(
        blade_carriage,
        handle,
        axes="x",
        elem_a="slide_rail",
        elem_b="bottom_spine",
        min_overlap=0.045,
        name="carriage remains engaged in handle at rest",
    )

    rest_pos = ctx.part_world_position(blade_carriage)
    with ctx.pose({blade_slide: blade_slide.motion_limits.upper}):
        ctx.expect_overlap(
            blade_carriage,
            handle,
            axes="x",
            elem_a="slide_rail",
            elem_b="bottom_spine",
            min_overlap=0.045,
            name="extended carriage remains retained in handle",
        )
        extended_pos = ctx.part_world_position(blade_carriage)
    ctx.check(
        "blade carriage extends toward knife nose",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.030,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    ctx.expect_gap(
        handle,
        spare_tray,
        axis="y",
        positive_elem="tail_hinge_knuckle_0",
        negative_elem="tray_knuckle",
        min_gap=0.0,
        max_gap=0.0005,
        name="upper hinge knuckles are closely spaced",
    )
    ctx.expect_gap(
        spare_tray,
        handle,
        axis="y",
        positive_elem="tray_knuckle",
        negative_elem="tail_hinge_knuckle_1",
        min_gap=0.0,
        max_gap=0.0005,
        name="lower hinge knuckles are closely spaced",
    )
    closed_cap = ctx.part_element_world_aabb(spare_tray, elem="tray_end_cap")
    with ctx.pose({tray_hinge: tray_hinge.motion_limits.upper}):
        open_cap = ctx.part_element_world_aabb(spare_tray, elem="tray_end_cap")
    ctx.check(
        "spare tray folds rearward from tail",
        closed_cap is not None
        and open_cap is not None
        and open_cap[0][0] < closed_cap[0][0] - 0.020
        and open_cap[1][2] < closed_cap[1][2] - 0.015,
        details=f"closed={closed_cap}, open={open_cap}",
    )

    return ctx.report()


object_model = build_object_model()
