from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
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
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="keg_hand_truck")

    red = model.material("red_powder_coat", rgba=(0.72, 0.04, 0.02, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.62, 0.58, 1.0))
    black = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    dark = model.material("dark_plastic", rgba=(0.04, 0.045, 0.05, 1.0))
    plate_mat = model.material("scuffed_toe_plate", rgba=(0.42, 0.42, 0.38, 1.0))

    frame = model.part("frame")

    rail_radius = 0.018
    rail_z = (0.14, 0.42, 0.73, 1.05, 1.34)
    rail_y = (-0.13, 0.02, 0.08, 0.02, -0.09)
    for x, name in ((-0.23, "rail_0"), (0.23, "rail_1")):
        rail = tube_from_spline_points(
            [(x, y, z) for y, z in zip(rail_y, rail_z)],
            radius=rail_radius,
            samples_per_segment=16,
            radial_segments=18,
            cap_ends=True,
        )
        frame.visual(mesh_from_geometry(rail, name), material=red, name=name)

    def crossbar(name: str, z: float, end_y: float, bow: float, radius: float = 0.016) -> None:
        bar = tube_from_spline_points(
            [(-0.245, end_y, z), (0.0, end_y + bow, z + 0.012), (0.245, end_y, z)],
            radius=radius,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        )
        frame.visual(mesh_from_geometry(bar, name), material=red, name=name)

    crossbar("lower_cradle", 0.42, 0.02, 0.075)
    crossbar("middle_cradle", 0.73, 0.08, 0.090)
    crossbar("upper_cradle", 1.05, 0.02, 0.060)
    crossbar("top_handle", 1.34, -0.09, -0.010, radius=0.019)

    # Rubber pads on the bowed cradle bars make the back read as a keg-contacting frame.
    for y, z, name in ((0.105, 0.432, "lower_pad"), (0.168, 0.742, "middle_pad")):
        frame.visual(
            Box((0.16, 0.020, 0.050)),
            origin=Origin(xyz=(0.0, y, z), rpy=(0.0, 0.0, 0.0)),
            material=dark,
            name=name,
        )

    # Toe plate and welded side cheeks tying the load platform into the axle/rails.
    frame.visual(
        Box((0.48, 0.30, 0.026)),
        origin=Origin(xyz=(0.0, 0.10, 0.030)),
        material=plate_mat,
        name="toe_plate",
    )
    frame.visual(
        Box((0.50, 0.030, 0.055)),
        origin=Origin(xyz=(0.0, 0.235, 0.055)),
        material=plate_mat,
        name="toe_lip",
    )
    for x, name in ((-0.23, "cheek_0"), (0.23, "cheek_1")):
        frame.visual(
            Box((0.060, 0.145, 0.150)),
            origin=Origin(xyz=(x, -0.055, 0.085)),
            material=red,
            name=name,
        )

    # The visible axle passes through the wheel bores; the wheels themselves are child links.
    frame.visual(
        Cylinder(radius=0.0165, length=0.74),
        origin=Origin(xyz=(0.0, -0.13, 0.14), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="axle",
    )

    # Top hinge brackets for the retaining hook.
    for x, name in ((-0.090, "hinge_lug_0"), (0.090, "hinge_lug_1")):
        frame.visual(
            Box((0.032, 0.060, 0.240)),
            origin=Origin(xyz=(x, 0.015, 1.130)),
            material=red,
            name=name,
        )
    for x, name in ((-0.090, "hinge_web_0"), (0.090, "hinge_web_1")):
        frame.visual(
            Box((0.034, 0.090, 0.080)),
            origin=Origin(xyz=(x, 0.055, 1.040)),
            material=red,
            name=name,
        )
    frame.visual(
        Cylinder(radius=0.018, length=0.22),
        origin=Origin(xyz=(0.0, 0.015, 1.205), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hinge_pin",
    )

    wheel_rim = mesh_from_geometry(
        WheelGeometry(
            0.102,
            0.060,
            rim=WheelRim(inner_radius=0.070, flange_height=0.006, flange_thickness=0.004),
            hub=WheelHub(
                radius=0.034,
                width=0.050,
                cap_style="flat",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.045, hole_diameter=0.005),
            ),
            face=WheelFace(dish_depth=0.005, front_inset=0.003, rear_inset=0.003),
            spokes=WheelSpokes(style="straight", count=5, thickness=0.006, window_radius=0.012),
            bore=WheelBore(style="round", diameter=0.032),
        ),
        "transport_wheel_rim",
    )
    wheel_tire = mesh_from_geometry(
        TireGeometry(
            0.145,
            0.074,
            inner_radius=0.105,
            tread=TireTread(style="block", depth=0.007, count=22, land_ratio=0.55),
            grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.003),),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.008, radius=0.004),
        ),
        "transport_wheel_tire",
    )

    for x, name in ((-0.36, "wheel_0"), (0.36, "wheel_1")):
        wheel = model.part(name)
        wheel.visual(wheel_tire, material=black, name="tire")
        wheel.visual(wheel_rim, material=steel, name="rim")
        model.articulation(
            f"frame_to_{name}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(x, -0.13, 0.14)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=6.0, velocity=18.0),
        )

    hook = model.part("retaining_hook")
    hook.visual(
        Cylinder(radius=0.021, length=0.130),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hinge_sleeve",
    )
    hook_tube = tube_from_spline_points(
        [
            (0.0, 0.030, -0.018),
            (0.0, 0.095, -0.035),
            (0.0, 0.195, -0.125),
            (0.0, 0.165, -0.205),
        ],
        radius=0.014,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    hook.visual(mesh_from_geometry(hook_tube, "retaining_hook_tube"), material=steel, name="hook_tube")
    hook.visual(
        Box((0.050, 0.022, 0.032)),
        origin=Origin(xyz=(0.0, 0.031, -0.006)),
        material=steel,
        name="weld_tab",
    )
    hook.visual(
        Box((0.105, 0.030, 0.035)),
        origin=Origin(xyz=(0.0, 0.158, -0.205)),
        material=dark,
        name="rubber_tip",
    )

    model.articulation(
        "frame_to_retaining_hook",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=hook,
        origin=Origin(xyz=(0.0, 0.015, 1.205)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=2.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    hook = object_model.get_part("retaining_hook")
    hook_joint = object_model.get_articulation("frame_to_retaining_hook")

    ctx.allow_overlap(
        frame,
        hook,
        elem_a="hinge_pin",
        elem_b="hinge_sleeve",
        reason="The hook sleeve is intentionally captured around the hinge pin.",
    )
    ctx.expect_within(
        hook,
        frame,
        axes="x",
        inner_elem="hinge_sleeve",
        outer_elem="hinge_pin",
        margin=0.060,
        name="hook sleeve stays centered on hinge pin",
    )
    ctx.expect_contact(
        hook,
        hook,
        elem_a="weld_tab",
        elem_b="hinge_sleeve",
        contact_tol=0.004,
        name="hook weld tab touches hinge sleeve",
    )
    ctx.expect_contact(
        hook,
        hook,
        elem_a="hook_tube",
        elem_b="weld_tab",
        contact_tol=0.004,
        name="hook tube is welded to tab",
    )

    for wheel in (wheel_0, wheel_1):
        ctx.allow_overlap(
            frame,
            wheel,
            elem_a="axle",
            elem_b="rim",
            reason="The axle is intentionally represented as captured through the solid hub/rim proxy.",
        )
        ctx.expect_within(
            frame,
            wheel,
            axes="yz",
            inner_elem="axle",
            outer_elem="rim",
            margin=0.030,
            name=f"{wheel.name} bore aligns with axle",
        )
        ctx.expect_overlap(
            wheel,
            frame,
            axes="x",
            elem_a="rim",
            elem_b="axle",
            min_overlap=0.015,
            name=f"{wheel.name} remains on the axle",
        )

    rest_tip = ctx.part_element_world_aabb(hook, elem="rubber_tip")
    with ctx.pose({hook_joint: 2.20}):
        raised_tip = ctx.part_element_world_aabb(hook, elem="rubber_tip")
    ctx.check(
        "retaining hook folds upward from load",
        rest_tip is not None
        and raised_tip is not None
        and raised_tip[0][2] > rest_tip[0][2] + 0.16
        and raised_tip[0][1] < rest_tip[0][1] - 0.02,
        details=f"rest_tip={rest_tip}, raised_tip={raised_tip}",
    )

    return ctx.report()


object_model = build_object_model()
