from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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
    model = ArticulatedObject(name="a3_flatbed_scanner")

    body_dark = model.material("body_dark", rgba=(0.19, 0.20, 0.22, 1.0))
    lid_dark = model.material("lid_dark", rgba=(0.23, 0.24, 0.26, 1.0))
    bezel_black = model.material("bezel_black", rgba=(0.08, 0.09, 0.10, 1.0))
    inner_white = model.material("inner_white", rgba=(0.91, 0.92, 0.89, 1.0))
    glass_blue = model.material("glass_blue", rgba=(0.52, 0.68, 0.78, 0.32))
    carriage_dark = model.material("carriage_dark", rgba=(0.14, 0.15, 0.16, 1.0))
    rail_metal = model.material("rail_metal", rgba=(0.65, 0.68, 0.70, 1.0))
    button_grey = model.material("button_grey", rgba=(0.34, 0.36, 0.38, 1.0))

    body_width = 0.590
    body_depth = 0.420
    wall_thickness = 0.012
    body_top_z = 0.064
    wall_height = body_top_z - 0.004
    platen_width = 0.460
    platen_depth = 0.330
    platen_thickness = 0.004

    lid_width = 0.588
    lid_depth = 0.405
    lid_thickness = 0.020
    hinge_y = 0.215
    hinge_z = 0.070

    body = model.part("body")

    def add_box(
        part,
        name: str,
        size: tuple[float, float, float],
        xyz: tuple[float, float, float],
        material,
        *,
        rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        part.visual(
            Box(size),
            origin=Origin(xyz=xyz, rpy=rpy),
            material=material,
            name=name,
        )

    add_box(
        body,
        "bottom_plate",
        (body_width, body_depth, 0.004),
        (0.0, 0.0, 0.002),
        body_dark,
    )
    add_box(
        body,
        "left_wall",
        (wall_thickness, body_depth, wall_height),
        (-(body_width / 2.0) + (wall_thickness / 2.0), 0.0, 0.004 + wall_height / 2.0),
        body_dark,
    )
    add_box(
        body,
        "right_wall",
        (wall_thickness, body_depth, wall_height),
        ((body_width / 2.0) - (wall_thickness / 2.0), 0.0, 0.004 + wall_height / 2.0),
        body_dark,
    )
    add_box(
        body,
        "front_wall",
        (body_width - (2.0 * wall_thickness), wall_thickness, wall_height),
        (0.0, -(body_depth / 2.0) + (wall_thickness / 2.0), 0.004 + wall_height / 2.0),
        body_dark,
    )
    add_box(
        body,
        "rear_wall",
        (body_width - (2.0 * wall_thickness), wall_thickness, 0.048),
        (0.0, (body_depth / 2.0) - (wall_thickness / 2.0), 0.028),
        body_dark,
    )

    bezel_side_depth = platen_depth
    bezel_side_width = (body_width - platen_width) / 2.0
    bezel_front_depth = (body_depth - platen_depth) / 2.0
    bezel_z = body_top_z - 0.003
    bezel_thickness = 0.006

    add_box(
        body,
        "left_bezel",
        (bezel_side_width, bezel_side_depth, bezel_thickness),
        (
            -(platen_width / 2.0) - (bezel_side_width / 2.0),
            0.0,
            bezel_z,
        ),
        bezel_black,
    )
    add_box(
        body,
        "right_bezel",
        (bezel_side_width, bezel_side_depth, bezel_thickness),
        (
            (platen_width / 2.0) + (bezel_side_width / 2.0),
            0.0,
            bezel_z,
        ),
        bezel_black,
    )
    add_box(
        body,
        "front_bezel",
        (body_width, bezel_front_depth, bezel_thickness),
        (
            0.0,
            -(platen_depth / 2.0) - (bezel_front_depth / 2.0),
            bezel_z,
        ),
        bezel_black,
    )
    add_box(
        body,
        "rear_bezel",
        (body_width, bezel_front_depth, bezel_thickness),
        (
            0.0,
            (platen_depth / 2.0) + (bezel_front_depth / 2.0),
            bezel_z,
        ),
        bezel_black,
    )
    add_box(
        body,
        "platen_glass",
        (platen_width, platen_depth, platen_thickness),
        (0.0, 0.0, body_top_z - (platen_thickness / 2.0)),
        glass_blue,
    )

    add_box(
        body,
        "left_guide_rail",
        (0.012, 0.312, 0.010),
        (-0.200, 0.0, 0.033),
        rail_metal,
    )
    add_box(
        body,
        "right_guide_rail",
        (0.012, 0.312, 0.010),
        (0.200, 0.0, 0.033),
        rail_metal,
    )
    add_box(
        body,
        "left_rail_front_standoff",
        (0.014, 0.024, 0.030),
        (-0.200, -0.125, 0.019),
        body_dark,
    )
    add_box(
        body,
        "left_rail_rear_standoff",
        (0.014, 0.024, 0.030),
        (-0.200, 0.125, 0.019),
        body_dark,
    )
    add_box(
        body,
        "right_rail_front_standoff",
        (0.014, 0.024, 0.030),
        (0.200, -0.125, 0.019),
        body_dark,
    )
    add_box(
        body,
        "right_rail_rear_standoff",
        (0.014, 0.024, 0.030),
        (0.200, 0.125, 0.019),
        body_dark,
    )
    add_box(
        body,
        "drive_belt_cover",
        (0.500, 0.040, 0.010),
        (0.0, 0.190, 0.034),
        body_dark,
    )
    add_box(
        body,
        "control_strip",
        (0.130, 0.010, 0.022),
        (0.196, -(body_depth / 2.0) + 0.012, 0.046),
        bezel_black,
    )
    add_box(
        body,
        "power_button",
        (0.020, 0.006, 0.012),
        (0.236, -(body_depth / 2.0) + 0.019, 0.046),
        button_grey,
    )
    add_box(
        body,
        "scan_button",
        (0.024, 0.006, 0.010),
        (0.196, -(body_depth / 2.0) + 0.019, 0.044),
        button_grey,
    )

    hinge_ear_size = (0.008, 0.018, 0.024)
    hinge_ear_y = 0.220
    hinge_ear_z = 0.068
    left_outer_x = -0.250
    left_inner_x = -0.220
    right_inner_x = 0.220
    right_outer_x = 0.250
    add_box(body, "left_hinge_outer_ear", hinge_ear_size, (left_outer_x, hinge_ear_y, hinge_ear_z), body_dark)
    add_box(body, "left_hinge_inner_ear", hinge_ear_size, (left_inner_x, hinge_ear_y, hinge_ear_z), body_dark)
    add_box(body, "right_hinge_inner_ear", hinge_ear_size, (right_inner_x, hinge_ear_y, hinge_ear_z), body_dark)
    add_box(body, "right_hinge_outer_ear", hinge_ear_size, (right_outer_x, hinge_ear_y, hinge_ear_z), body_dark)
    add_box(
        body,
        "left_hinge_pedestal",
        (0.050, 0.024, 0.018),
        (-0.235, 0.214, 0.051),
        body_dark,
    )
    add_box(
        body,
        "right_hinge_pedestal",
        (0.050, 0.024, 0.018),
        (0.235, 0.214, 0.051),
        body_dark,
    )

    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, 0.085)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0425)),
    )

    lid = model.part("lid")
    add_box(
        lid,
        "lid_top_shell",
        (lid_width, lid_depth, lid_thickness),
        (0.0, -(lid_depth / 2.0) - 0.005, 0.004),
        lid_dark,
    )
    add_box(
        lid,
        "lid_inner_panel",
        (0.548, 0.372, 0.006),
        (0.0, -0.193, -0.003),
        inner_white,
    )
    add_box(
        lid,
        "left_knuckle_block",
        (0.024, 0.014, 0.012),
        (-0.235, -0.006, 0.001),
        lid_dark,
    )
    add_box(
        lid,
        "right_knuckle_block",
        (0.024, 0.014, 0.012),
        (0.235, -0.006, 0.001),
        lid_dark,
    )
    lid.visual(
        Cylinder(radius=0.006, length=0.020),
        origin=Origin(xyz=(-0.235, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lid_dark,
        name="left_hinge_knuckle",
    )
    lid.visual(
        Cylinder(radius=0.006, length=0.020),
        origin=Origin(xyz=(0.235, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lid_dark,
        name="right_hinge_knuckle",
    )
    lid.inertial = Inertial.from_geometry(
        Box((lid_width, lid_depth, 0.030)),
        mass=2.6,
        origin=Origin(xyz=(0.0, -(lid_depth / 2.0), 0.010)),
    )

    scan_head = model.part("scan_head")
    add_box(
        scan_head,
        "carriage_bar",
        (0.468, 0.028, 0.018),
        (0.0, 0.0, 0.0),
        carriage_dark,
    )
    add_box(
        scan_head,
        "optics_housing",
        (0.148, 0.034, 0.020),
        (0.0, 0.0, -0.010),
        carriage_dark,
    )
    add_box(
        scan_head,
        "left_guide_shoe",
        (0.022, 0.032, 0.014),
        (-0.195, 0.0, -0.002),
        rail_metal,
    )
    add_box(
        scan_head,
        "right_guide_shoe",
        (0.022, 0.032, 0.014),
        (0.195, 0.0, -0.002),
        rail_metal,
    )
    scan_head.inertial = Inertial.from_geometry(
        Box((0.468, 0.036, 0.040)),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )

    model.articulation(
        "body_to_scan_head",
        ArticulationType.PRISMATIC,
        parent=body,
        child=scan_head,
        origin=Origin(xyz=(0.0, -0.130, 0.046)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.22,
            lower=0.0,
            upper=0.260,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    scan_head = object_model.get_part("scan_head")
    lid_hinge = object_model.get_articulation("body_to_lid")
    scan_slide = object_model.get_articulation("body_to_scan_head")

    platen_glass = body.get_visual("platen_glass")
    lid_panel = lid.get_visual("lid_inner_panel")
    carriage_bar = scan_head.get_visual("carriage_bar")

    ctx.check(
        "lid hinge uses rear x-axis rotation",
        abs(lid_hinge.axis[0] + 1.0) < 1e-6
        and abs(lid_hinge.axis[1]) < 1e-6
        and abs(lid_hinge.axis[2]) < 1e-6,
        details=f"axis={lid_hinge.axis}",
    )
    ctx.check(
        "scan head uses depth-axis slide",
        abs(scan_slide.axis[0]) < 1e-6
        and abs(scan_slide.axis[1] - 1.0) < 1e-6
        and abs(scan_slide.axis[2]) < 1e-6,
        details=f"axis={scan_slide.axis}",
    )

    with ctx.pose({lid_hinge: 0.0, scan_slide: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem=lid_panel,
            negative_elem=platen_glass,
            min_gap=0.0,
            max_gap=0.0025,
            name="closed lid panel rests just above the platen glass",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a=lid_panel,
            elem_b=platen_glass,
            min_overlap=0.30,
            name="closed lid covers the A3 platen area",
        )
        ctx.expect_gap(
            body,
            scan_head,
            axis="z",
            positive_elem=platen_glass,
            negative_elem=carriage_bar,
            min_gap=0.004,
            max_gap=0.012,
            name="scan head rides immediately below the platen glass",
        )
        ctx.expect_within(
            scan_head,
            body,
            axes="x",
            inner_elem=carriage_bar,
            outer_elem=platen_glass,
            margin=0.012,
            name="scan head remains laterally under the platen",
        )
        ctx.expect_overlap(
            scan_head,
            body,
            axes="y",
            elem_a=carriage_bar,
            elem_b=platen_glass,
            min_overlap=0.020,
            name="home scan head remains under the platen opening",
        )

    closed_lid_aabb = None
    open_lid_aabb = None
    with ctx.pose({lid_hinge: 0.0}):
        closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_inner_panel")
    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_inner_panel")

    ctx.check(
        "lid opens upward from the rear hinge line",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.16,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    home_pos = None
    travel_pos = None
    travel_upper = scan_slide.motion_limits.upper
    with ctx.pose({scan_slide: 0.0}):
        home_pos = ctx.part_world_position(scan_head)
    with ctx.pose({scan_slide: travel_upper}):
        travel_pos = ctx.part_world_position(scan_head)
        ctx.expect_overlap(
            scan_head,
            body,
            axes="y",
            elem_a=carriage_bar,
            elem_b=platen_glass,
            min_overlap=0.020,
            name="fully traveled scan head still remains under the platen opening",
        )
        ctx.expect_within(
            scan_head,
            body,
            axes="x",
            inner_elem=carriage_bar,
            outer_elem=platen_glass,
            margin=0.012,
            name="fully traveled scan head stays laterally aligned with the platen",
        )

    ctx.check(
        "scan head traverses toward the rear of the scanner",
        home_pos is not None and travel_pos is not None and travel_pos[1] > home_pos[1] + 0.20,
        details=f"home={home_pos}, travel={travel_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
