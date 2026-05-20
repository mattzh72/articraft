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
    mesh_from_geometry,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[index] + upper[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="prosumer_espresso_machine")

    stainless = model.material("stainless", rgba=(0.77, 0.79, 0.80, 1.0))
    brushed = model.material("brushed", rgba=(0.68, 0.70, 0.72, 1.0))
    dark_panel = model.material("dark_panel", rgba=(0.17, 0.18, 0.19, 1.0))
    black = model.material("black", rgba=(0.09, 0.09, 0.10, 1.0))
    glass = model.material("glass", rgba=(0.70, 0.82, 0.88, 0.35))

    portafilter_handle = _save_mesh(
        "portafilter_handle",
        tube_from_spline_points(
            [
                (0.0, 0.020, -0.008),
                (0.0, 0.045, -0.012),
                (0.0, 0.090, -0.017),
                (0.0, 0.150, -0.020),
            ],
            radius=0.010,
            samples_per_segment=16,
            radial_segments=18,
            cap_ends=True,
        ),
    )
    wand_tube = _save_mesh(
        "steam_wand_tube",
        tube_from_spline_points(
            [
                (0.0, 0.024, -0.004),
                (0.0, 0.038, -0.040),
                (0.002, 0.046, -0.110),
                (0.004, 0.050, -0.205),
            ],
            radius=0.0042,
            samples_per_segment=18,
            radial_segments=16,
            cap_ends=True,
        ),
    )

    body = model.part("body")
    body.visual(
        Box((0.340, 0.420, 0.026)),
        origin=Origin(xyz=(0.000, 0.000, 0.013)),
        material=stainless,
        name="base",
    )
    body.visual(
        Box((0.340, 0.240, 0.310)),
        origin=Origin(xyz=(0.000, -0.080, 0.181)),
        material=stainless,
        name="rear_shell",
    )
    body.visual(
        Box((0.340, 0.160, 0.140)),
        origin=Origin(xyz=(0.000, 0.120, 0.290)),
        material=stainless,
        name="upper_shell",
    )
    body.visual(
        Box((0.020, 0.160, 0.200)),
        origin=Origin(xyz=(-0.160, 0.120, 0.126)),
        material=stainless,
        name="left_leg",
    )
    body.visual(
        Box((0.020, 0.160, 0.200)),
        origin=Origin(xyz=(0.160, 0.120, 0.126)),
        material=stainless,
        name="right_leg",
    )
    body.visual(
        Box((0.340, 0.060, 0.060)),
        origin=Origin(xyz=(0.000, 0.090, 0.050)),
        material=stainless,
        name="front_bridge",
    )
    body.visual(
        Box((0.300, 0.018, 0.100)),
        origin=Origin(xyz=(0.000, 0.209, 0.120)),
        material=dark_panel,
        name="front_lower",
    )
    body.visual(
        Box((0.084, 0.018, 0.195)),
        origin=Origin(xyz=(-0.108, 0.209, 0.2325)),
        material=dark_panel,
        name="front_left",
    )
    body.visual(
        Box((0.084, 0.018, 0.195)),
        origin=Origin(xyz=(0.108, 0.209, 0.2325)),
        material=dark_panel,
        name="front_right",
    )
    body.visual(
        Box((0.300, 0.018, 0.072)),
        origin=Origin(xyz=(0.000, 0.209, 0.334)),
        material=dark_panel,
        name="front_upper",
    )
    body.visual(
        Box((0.248, 0.162, 0.012)),
        origin=Origin(xyz=(0.000, 0.123, 0.084)),
        material=dark_panel,
        name="tray_top",
    )
    body.visual(
        Box((0.248, 0.020, 0.046)),
        origin=Origin(xyz=(0.000, 0.195, 0.065)),
        material=dark_panel,
        name="tray_front",
    )
    body.visual(
        Box((0.020, 0.162, 0.034)),
        origin=Origin(xyz=(-0.114, 0.123, 0.071)),
        material=dark_panel,
        name="tray_left",
    )
    body.visual(
        Box((0.020, 0.162, 0.034)),
        origin=Origin(xyz=(0.114, 0.123, 0.071)),
        material=dark_panel,
        name="tray_right",
    )
    body.visual(
        Box((0.302, 0.320, 0.012)),
        origin=Origin(xyz=(0.000, -0.005, 0.356)),
        material=stainless,
        name="top_skin",
    )
    body.visual(
        Box((0.278, 0.010, 0.018)),
        origin=Origin(xyz=(0.000, -0.162, 0.369)),
        material=brushed,
        name="rear_rail",
    )
    body.visual(
        Box((0.010, 0.328, 0.018)),
        origin=Origin(xyz=(-0.136, -0.002, 0.369)),
        material=brushed,
        name="left_rail",
    )
    body.visual(
        Box((0.010, 0.328, 0.018)),
        origin=Origin(xyz=(0.136, -0.002, 0.369)),
        material=brushed,
        name="right_rail",
    )
    for index, slat_y in enumerate((-0.060, 0.000, 0.060)):
        body.visual(
            Box((0.212, 0.010, 0.004)),
            origin=Origin(xyz=(0.000, slat_y, 0.362)),
            material=brushed,
            name=f"top_slat_{index}",
        )
    body.visual(
        Box((0.100, 0.086, 0.006)),
        origin=Origin(xyz=(0.000, -0.081, 0.363)),
        material=brushed,
        name="fill_bezel",
    )
    body.visual(
        Cylinder(radius=0.007, length=0.022),
        origin=Origin(xyz=(-0.028, -0.124, 0.362), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed,
        name="hinge_barrel_0",
    )
    body.visual(
        Cylinder(radius=0.007, length=0.022),
        origin=Origin(xyz=(0.028, -0.124, 0.362), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed,
        name="hinge_barrel_1",
    )
    body.visual(
        Box((0.112, 0.070, 0.046)),
        origin=Origin(xyz=(0.000, 0.174, 0.268)),
        material=brushed,
        name="group_block",
    )
    body.visual(
        Cylinder(radius=0.029, length=0.060),
        origin=Origin(xyz=(0.000, 0.217, 0.278), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="group_body",
    )
    body.visual(
        Cylinder(radius=0.030, length=0.018),
        origin=Origin(xyz=(0.000, 0.218, 0.246)),
        material=brushed,
        name="group_face",
    )
    body.visual(
        Box((0.048, 0.056, 0.080)),
        origin=Origin(xyz=(0.126, 0.190, 0.286)),
        material=brushed,
        name="wand_block",
    )
    body.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(xyz=(0.074, 0.209, 0.258), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="knob_bezel",
    )
    body.visual(
        Cylinder(radius=0.032, length=0.014),
        origin=Origin(xyz=(-0.092, 0.208, 0.266), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="gauge_bezel",
    )
    body.visual(
        Cylinder(radius=0.028, length=0.010),
        origin=Origin(xyz=(-0.092, 0.214, 0.266), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="gauge_glass",
    )
    for foot_index, (foot_x, foot_y) in enumerate(
        ((-0.125, -0.155), (0.125, -0.155), (-0.125, 0.155), (0.125, 0.155))
    ):
        body.visual(
            Cylinder(radius=0.018, length=0.010),
            origin=Origin(xyz=(foot_x, foot_y, 0.005)),
            material=black,
            name=f"foot_{foot_index}",
        )
    body.inertial = Inertial.from_geometry(
        Box((0.340, 0.420, 0.372)),
        mass=15.0,
        origin=Origin(xyz=(0.000, 0.000, 0.186)),
    )

    portafilter = model.part("portafilter")
    portafilter.visual(
        Cylinder(radius=0.031, length=0.030),
        origin=Origin(xyz=(0.000, 0.000, -0.015)),
        material=brushed,
        name="basket",
    )
    portafilter.visual(
        Cylinder(radius=0.039, length=0.006),
        origin=Origin(xyz=(0.000, 0.000, -0.003)),
        material=brushed,
        name="rim",
    )
    portafilter.visual(
        Box((0.016, 0.024, 0.014)),
        origin=Origin(xyz=(0.000, 0.022, -0.008)),
        material=black,
        name="neck",
    )
    portafilter.visual(
        portafilter_handle,
        material=black,
        name="handle",
    )
    portafilter.inertial = Inertial.from_geometry(
        Box((0.090, 0.165, 0.050)),
        mass=0.9,
        origin=Origin(xyz=(0.000, 0.070, -0.018)),
    )

    wand = model.part("wand")
    wand.visual(
        Cylinder(radius=0.006, length=0.024),
        origin=Origin(xyz=(0.000, 0.012, -0.005), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="stub",
    )
    wand.visual(
        wand_tube,
        material=brushed,
        name="tube",
    )
    wand.visual(
        Cylinder(radius=0.0048, length=0.018),
        origin=Origin(xyz=(0.004, 0.050, -0.214)),
        material=brushed,
        name="tip",
    )
    wand.inertial = Inertial.from_geometry(
        Box((0.030, 0.050, 0.230)),
        mass=0.2,
        origin=Origin(xyz=(0.002, 0.020, -0.110)),
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=(0.000, 0.009, 0.000), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="shaft",
    )
    knob.visual(
        Cylinder(radius=0.022, length=0.028),
        origin=Origin(xyz=(0.000, 0.032, 0.000), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="dial",
    )
    knob.visual(
        Box((0.004, 0.006, 0.011)),
        origin=Origin(xyz=(0.000, 0.049, 0.011)),
        material=brushed,
        name="grip",
    )
    knob.inertial = Inertial.from_geometry(
        Box((0.050, 0.055, 0.050)),
        mass=0.08,
        origin=Origin(xyz=(0.000, 0.030, 0.000)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.082, 0.070, 0.006)),
        origin=Origin(xyz=(0.000, 0.042, 0.003)),
        material=brushed,
        name="lid_panel",
    )
    lid.visual(
        Box((0.030, 0.008, 0.010)),
        origin=Origin(xyz=(0.000, 0.073, 0.005)),
        material=black,
        name="pull",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.082, 0.070, 0.016)),
        mass=0.12,
        origin=Origin(xyz=(0.000, 0.034, 0.004)),
    )

    model.articulation(
        "body_to_portafilter",
        ArticulationType.REVOLUTE,
        parent=body,
        child=portafilter,
        origin=Origin(xyz=(0.000, 0.252, 0.237)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=-0.95,
            upper=0.12,
        ),
    )
    model.articulation(
        "body_to_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=wand,
        origin=Origin(xyz=(0.126, 0.218, 0.307)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=-1.20,
            upper=0.90,
        ),
    )
    model.articulation(
        "body_to_knob",
        ArticulationType.REVOLUTE,
        parent=body,
        child=knob,
        origin=Origin(xyz=(0.074, 0.218, 0.258)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=3.0,
            lower=0.0,
            upper=4.20,
        ),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.000, -0.124, 0.3625)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=1.5,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    portafilter = object_model.get_part("portafilter")
    wand = object_model.get_part("wand")
    knob = object_model.get_part("knob")
    lid = object_model.get_part("lid")

    portafilter_joint = object_model.get_articulation("body_to_portafilter")
    wand_joint = object_model.get_articulation("body_to_wand")
    knob_joint = object_model.get_articulation("body_to_knob")
    lid_joint = object_model.get_articulation("body_to_lid")

    ctx.expect_gap(
        body,
        portafilter,
        axis="z",
        positive_elem="group_face",
        negative_elem="rim",
        max_gap=0.008,
        max_penetration=0.0,
        name="portafilter seats just below the group head",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_panel",
        elem_b="fill_bezel",
        min_overlap=0.060,
        name="fill lid covers the fill opening zone",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="top_skin",
        max_gap=0.0015,
        max_penetration=0.0,
        name="fill lid sits nearly flush on the top deck",
    )

    rest_handle = _aabb_center(ctx.part_element_world_aabb(portafilter, elem="handle"))
    with ctx.pose({portafilter_joint: -0.80}):
        turned_handle = _aabb_center(ctx.part_element_world_aabb(portafilter, elem="handle"))
    ctx.check(
        "portafilter rotates sideways to unlock",
        rest_handle is not None
        and turned_handle is not None
        and turned_handle[0] > rest_handle[0] + 0.040,
        details=f"rest_handle={rest_handle}, turned_handle={turned_handle}",
    )

    rest_tip = _aabb_center(ctx.part_element_world_aabb(wand, elem="tip"))
    with ctx.pose({wand_joint: -1.00}):
        swung_tip = _aabb_center(ctx.part_element_world_aabb(wand, elem="tip"))
    ctx.check(
        "steam wand swings on a vertical pivot",
        rest_tip is not None
        and swung_tip is not None
        and swung_tip[0] > rest_tip[0] + 0.040,
        details=f"rest_tip={rest_tip}, swung_tip={swung_tip}",
    )

    rest_grip = _aabb_center(ctx.part_element_world_aabb(knob, elem="grip"))
    with ctx.pose({knob_joint: math.pi / 2.0}):
        turned_grip = _aabb_center(ctx.part_element_world_aabb(knob, elem="grip"))
    ctx.check(
        "steam knob turns around its front-facing shaft",
        rest_grip is not None
        and turned_grip is not None
        and turned_grip[0] > rest_grip[0] + 0.008
        and turned_grip[2] < rest_grip[2] - 0.006,
        details=f"rest_grip={rest_grip}, turned_grip={turned_grip}",
    )

    rest_lid = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({lid_joint: 1.00}):
        open_lid = ctx.part_element_world_aabb(lid, elem="lid_panel")
    ctx.check(
        "fill lid opens upward from the rear hinge",
        rest_lid is not None
        and open_lid is not None
        and open_lid[1][2] > rest_lid[1][2] + 0.030,
        details=f"rest_lid={rest_lid}, open_lid={open_lid}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
