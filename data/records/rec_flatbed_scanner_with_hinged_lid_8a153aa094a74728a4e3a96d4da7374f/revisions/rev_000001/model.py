from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_flatbed_scanner")

    body_plastic = model.material("body_plastic", rgba=(0.14, 0.15, 0.16, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.08, 0.09, 0.10, 1.0))
    lid_finish = model.material("lid_finish", rgba=(0.78, 0.80, 0.82, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.46, 0.48, 0.50, 1.0))
    rail_metal = model.material("rail_metal", rgba=(0.76, 0.78, 0.80, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.55, 0.77, 0.92, 0.35))
    optics_strip = model.material("optics_strip", rgba=(0.14, 0.38, 0.58, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.36, 0.26, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=body_plastic,
        name="bottom_plate",
    )
    body.visual(
        Box((0.012, 0.26, 0.036)),
        origin=Origin(xyz=(-0.174, 0.0, 0.022)),
        material=body_plastic,
        name="left_wall",
    )
    body.visual(
        Box((0.012, 0.26, 0.036)),
        origin=Origin(xyz=(0.174, 0.0, 0.022)),
        material=body_plastic,
        name="right_wall",
    )
    body.visual(
        Box((0.336, 0.012, 0.036)),
        origin=Origin(xyz=(0.0, -0.124, 0.022)),
        material=body_plastic,
        name="front_wall",
    )
    body.visual(
        Box((0.336, 0.012, 0.036)),
        origin=Origin(xyz=(0.0, 0.124, 0.022)),
        material=body_plastic,
        name="rear_wall",
    )
    body.visual(
        Box((0.016, 0.208, 0.004)),
        origin=Origin(xyz=(-0.172, 0.0, 0.042)),
        material=body_plastic,
        name="top_left_rail",
    )
    body.visual(
        Box((0.016, 0.208, 0.004)),
        origin=Origin(xyz=(0.172, 0.0, 0.042)),
        material=body_plastic,
        name="top_right_rail",
    )
    body.visual(
        Box((0.328, 0.020, 0.004)),
        origin=Origin(xyz=(0.0, -0.104, 0.042)),
        material=body_plastic,
        name="top_front_rail",
    )
    body.visual(
        Box((0.328, 0.020, 0.004)),
        origin=Origin(xyz=(0.0, 0.104, 0.042)),
        material=body_plastic,
        name="top_rear_rail",
    )
    body.visual(
        Box((0.020, 0.188, 0.002)),
        origin=Origin(xyz=(-0.157, 0.0, 0.039)),
        material=trim_dark,
        name="left_glass_ledge",
    )
    body.visual(
        Box((0.020, 0.188, 0.002)),
        origin=Origin(xyz=(0.157, 0.0, 0.039)),
        material=trim_dark,
        name="right_glass_ledge",
    )
    body.visual(
        Box((0.294, 0.020, 0.002)),
        origin=Origin(xyz=(0.0, -0.104, 0.039)),
        material=trim_dark,
        name="front_glass_ledge",
    )
    body.visual(
        Box((0.294, 0.020, 0.002)),
        origin=Origin(xyz=(0.0, 0.104, 0.039)),
        material=trim_dark,
        name="rear_glass_ledge",
    )
    body.visual(
        Box((0.012, 0.024, 0.016)),
        origin=Origin(xyz=(-0.146, -0.100, 0.008)),
        material=trim_dark,
        name="left_front_rail_post",
    )
    body.visual(
        Box((0.012, 0.024, 0.016)),
        origin=Origin(xyz=(-0.146, 0.100, 0.008)),
        material=trim_dark,
        name="left_rear_rail_post",
    )
    body.visual(
        Box((0.012, 0.024, 0.016)),
        origin=Origin(xyz=(0.146, -0.100, 0.008)),
        material=trim_dark,
        name="right_front_rail_post",
    )
    body.visual(
        Box((0.012, 0.024, 0.016)),
        origin=Origin(xyz=(0.146, 0.100, 0.008)),
        material=trim_dark,
        name="right_rear_rail_post",
    )
    body.visual(
        Cylinder(radius=0.003, length=0.176),
        origin=Origin(xyz=(-0.146, 0.0, 0.019), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rail_metal,
        name="left_guide_rod",
    )
    body.visual(
        Cylinder(radius=0.003, length=0.176),
        origin=Origin(xyz=(0.146, 0.0, 0.019), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rail_metal,
        name="right_guide_rod",
    )
    body.visual(
        Box((0.022, 0.014, 0.008)),
        origin=Origin(xyz=(-0.146, 0.130, 0.040)),
        material=hinge_metal,
        name="left_hinge_bracket",
    )
    body.visual(
        Box((0.022, 0.014, 0.008)),
        origin=Origin(xyz=(0.146, 0.130, 0.040)),
        material=hinge_metal,
        name="right_hinge_bracket",
    )
    body.visual(
        Cylinder(radius=0.0045, length=0.018),
        origin=Origin(xyz=(-0.146, 0.137, 0.046), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_metal,
        name="left_body_hinge_knuckle",
    )
    body.visual(
        Cylinder(radius=0.0045, length=0.018),
        origin=Origin(xyz=(0.146, 0.137, 0.046), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_metal,
        name="right_body_hinge_knuckle",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.36, 0.26, 0.048)),
        mass=1.65,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
    )

    glass = model.part("platen_glass")
    glass.visual(
        Box((0.294, 0.188, 0.0025)),
        material=glass_tint,
        name="glass_pane",
    )
    glass.inertial = Inertial.from_geometry(
        Box((0.294, 0.188, 0.0025)),
        mass=0.20,
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.354, 0.252, 0.010)),
        origin=Origin(xyz=(0.0, -0.133, 0.003)),
        material=lid_finish,
        name="lid_panel",
    )
    lid.visual(
        Box((0.020, 0.010, 0.008)),
        origin=Origin(xyz=(-0.122, -0.005, 0.002)),
        material=hinge_metal,
        name="left_lid_hinge_tab",
    )
    lid.visual(
        Box((0.020, 0.010, 0.008)),
        origin=Origin(xyz=(0.122, -0.005, 0.002)),
        material=hinge_metal,
        name="right_lid_hinge_tab",
    )
    lid.visual(
        Cylinder(radius=0.0045, length=0.018),
        origin=Origin(xyz=(-0.122, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_metal,
        name="left_lid_hinge_knuckle",
    )
    lid.visual(
        Cylinder(radius=0.0045, length=0.018),
        origin=Origin(xyz=(0.122, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_metal,
        name="right_lid_hinge_knuckle",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.354, 0.252, 0.010)),
        mass=0.55,
        origin=Origin(xyz=(0.0, -0.133, 0.003)),
    )

    scan_head = model.part("scan_head")
    scan_head.visual(
        Box((0.292, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=trim_dark,
        name="carriage_bar",
    )
    scan_head.visual(
        Box((0.018, 0.024, 0.018)),
        origin=Origin(xyz=(-0.146, 0.0, 0.0)),
        material=trim_dark,
        name="left_carriage_block",
    )
    scan_head.visual(
        Box((0.018, 0.024, 0.018)),
        origin=Origin(xyz=(0.146, 0.0, 0.0)),
        material=trim_dark,
        name="right_carriage_block",
    )
    scan_head.visual(
        Box((0.252, 0.004, 0.0015)),
        origin=Origin(xyz=(0.0, 0.0, 0.00775)),
        material=optics_strip,
        name="ccd_window",
    )
    scan_head.inertial = Inertial.from_geometry(
        Box((0.310, 0.024, 0.018)),
        mass=0.12,
    )

    model.articulation(
        "body_to_glass",
        ArticulationType.FIXED,
        parent=body,
        child=glass,
        origin=Origin(xyz=(0.0, 0.0, 0.04125)),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.137, 0.046)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=0.0,
            upper=1.25,
        ),
    )
    model.articulation(
        "body_to_scan_head",
        ArticulationType.PRISMATIC,
        parent=body,
        child=scan_head,
        origin=Origin(xyz=(0.0, -0.074, 0.031)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.22,
            lower=0.0,
            upper=0.148,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    expected_parts = {"body", "platen_glass", "lid", "scan_head"}
    expected_joints = {"body_to_glass", "body_to_lid", "body_to_scan_head"}
    ctx.check(
        "expected scanner parts exist",
        expected_parts.issubset({part.name for part in object_model.parts}),
        details=f"parts={[part.name for part in object_model.parts]}",
    )
    ctx.check(
        "expected scanner joints exist",
        expected_joints.issubset({joint.name for joint in object_model.articulations}),
        details=f"joints={[joint.name for joint in object_model.articulations]}",
    )

    body = object_model.get_part("body")
    glass = object_model.get_part("platen_glass")
    lid = object_model.get_part("lid")
    scan_head = object_model.get_part("scan_head")
    lid_hinge = object_model.get_articulation("body_to_lid")
    scan_slide = object_model.get_articulation("body_to_scan_head")

    with ctx.pose({lid_hinge: 0.0, scan_slide: 0.0}):
        ctx.expect_contact(
            glass,
            body,
            elem_a="glass_pane",
            elem_b="left_glass_ledge",
            name="glass seats on the left support ledge",
        )
        ctx.expect_gap(
            lid,
            glass,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="glass_pane",
            min_gap=0.001,
            max_gap=0.004,
            name="closed lid stays just above the platen glass",
        )
        ctx.expect_overlap(
            lid,
            glass,
            axes="xy",
            elem_a="lid_panel",
            elem_b="glass_pane",
            min_overlap=0.18,
            name="closed lid fully covers the scan window",
        )
        ctx.expect_gap(
            glass,
            scan_head,
            axis="z",
            positive_elem="glass_pane",
            negative_elem="carriage_bar",
            min_gap=0.0015,
            max_gap=0.008,
            name="scan head parks just below the glass",
        )
        ctx.expect_within(
            scan_head,
            body,
            axes="x",
            inner_elem="carriage_bar",
            margin=0.0,
            name="scan head remains between the side walls",
        )

    lid_open = 1.10
    closed_lid_panel = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({lid_hinge: lid_open}):
        opened_lid_panel = ctx.part_element_world_aabb(lid, elem="lid_panel")
        ctx.check(
            "lid opens upward around the rear hinges",
            closed_lid_panel is not None
            and opened_lid_panel is not None
            and opened_lid_panel[1][2] > closed_lid_panel[1][2] + 0.10,
            details=f"closed={closed_lid_panel}, opened={opened_lid_panel}",
        )

    slide_upper = scan_slide.motion_limits.upper if scan_slide.motion_limits is not None else None
    rest_head_pos = ctx.part_world_position(scan_head)
    if slide_upper is not None:
        with ctx.pose({scan_slide: slide_upper}):
            extended_head_pos = ctx.part_world_position(scan_head)
            ctx.expect_gap(
                glass,
                scan_head,
                axis="z",
                positive_elem="glass_pane",
                negative_elem="carriage_bar",
                min_gap=0.0015,
                max_gap=0.008,
                name="scan head stays below the glass at full travel",
            )
            ctx.expect_within(
                scan_head,
                body,
                axes="x",
                inner_elem="carriage_bar",
                margin=0.0,
                name="scan head remains centered between the rails at full travel",
            )
            ctx.check(
                "scan head travels rearward along the scan axis",
                rest_head_pos is not None
                and extended_head_pos is not None
                and extended_head_pos[1] > rest_head_pos[1] + 0.12,
                details=f"rest={rest_head_pos}, extended={extended_head_pos}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
