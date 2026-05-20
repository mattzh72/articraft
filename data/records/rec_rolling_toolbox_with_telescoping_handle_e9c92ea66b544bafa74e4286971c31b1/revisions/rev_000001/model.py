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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="contractor_rolling_toolbox")

    dark_plastic = model.material("dark_plastic", color=(0.055, 0.060, 0.058, 1.0))
    black = model.material("black_rubber", color=(0.010, 0.010, 0.010, 1.0))
    yellow = model.material("jobsite_yellow", color=(0.96, 0.69, 0.07, 1.0))
    metal = model.material("brushed_steel", color=(0.55, 0.56, 0.54, 1.0))
    latch_red = model.material("red_latch", color=(0.78, 0.06, 0.035, 1.0))

    body = model.part("body")

    # Tall hollow base tub: separate walls leave an actual open cavity and a
    # front drawer aperture rather than a solid block.
    body.visual(
        Box((0.72, 0.36, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0675)),
        material=dark_plastic,
        name="bottom_pan",
    )
    body.visual(
        Box((0.72, 0.035, 0.49)),
        origin=Origin(xyz=(0.0, 0.1825, 0.325)),
        material=dark_plastic,
        name="side_wall_0",
    )
    body.visual(
        Box((0.72, 0.035, 0.49)),
        origin=Origin(xyz=(0.0, -0.1825, 0.325)),
        material=dark_plastic,
        name="side_wall_1",
    )
    body.visual(
        Box((0.035, 0.36, 0.49)),
        origin=Origin(xyz=(-0.3525, 0.0, 0.325)),
        material=dark_plastic,
        name="rear_wall",
    )
    body.visual(
        Box((0.035, 0.36, 0.345)),
        origin=Origin(xyz=(0.3525, 0.0, 0.2525)),
        material=dark_plastic,
        name="front_lower_wall",
    )
    body.visual(
        Box((0.035, 0.36, 0.016)),
        origin=Origin(xyz=(0.3525, 0.0, 0.562)),
        material=dark_plastic,
        name="front_top_rail",
    )
    body.visual(
        Box((0.035, 0.04, 0.13)),
        origin=Origin(xyz=(0.3525, 0.17, 0.495)),
        material=dark_plastic,
        name="front_stile_0",
    )
    body.visual(
        Box((0.035, 0.04, 0.13)),
        origin=Origin(xyz=(0.3525, -0.17, 0.495)),
        material=dark_plastic,
        name="front_stile_1",
    )
    body.visual(
        Box((0.64, 0.032, 0.028)),
        origin=Origin(xyz=(0.02, 0.0, 0.575)),
        material=yellow,
        name="top_rear_rim",
    )
    body.visual(
        Box((0.50, 0.033, 0.020)),
        origin=Origin(xyz=(0.06, 0.1485, 0.425)),
        material=metal,
        name="drawer_runner_0",
    )
    body.visual(
        Box((0.50, 0.033, 0.020)),
        origin=Origin(xyz=(0.06, -0.1485, 0.425)),
        material=metal,
        name="drawer_runner_1",
    )

    # Rear telescoping handle guide channels, with brackets tying them into the shell.
    for i, y in enumerate((0.14, -0.14)):
        suffix = str(i)
        body.visual(
            Box((0.014, 0.052, 0.52)),
            origin=Origin(xyz=(-0.438, y, 0.39)),
            material=metal,
            name=("guide_web_0", "guide_web_1")[i],
        )
        body.visual(
            Box((0.045, 0.006, 0.52)),
            origin=Origin(xyz=(-0.455, y + 0.021, 0.39)),
            material=metal,
            name=f"guide_flange_{suffix}_0",
        )
        body.visual(
            Box((0.045, 0.006, 0.52)),
            origin=Origin(xyz=(-0.455, y - 0.021, 0.39)),
            material=metal,
            name=f"guide_flange_{suffix}_1",
        )
        body.visual(
            Box((0.075, 0.075, 0.035)),
            origin=Origin(xyz=(-0.402, y, 0.22)),
            material=dark_plastic,
            name=f"guide_mount_{suffix}_0",
        )
        body.visual(
            Box((0.075, 0.075, 0.035)),
            origin=Origin(xyz=(-0.402, y, 0.56)),
            material=dark_plastic,
            name=f"guide_mount_{suffix}_1",
        )

    # Rear hinge barrel and cross axle.  Cylinders are rotated from local Z into Y.
    body.visual(
        Cylinder(radius=0.014, length=0.34),
        origin=Origin(xyz=(-0.411, 0.0, 0.610), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="rear_hinge_pin",
    )
    body.visual(
        Box((0.045, 0.31, 0.040)),
        origin=Origin(xyz=(-0.375, 0.0, 0.590)),
        material=metal,
        name="rear_hinge_leaf",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.48),
        origin=Origin(xyz=(-0.30, 0.0, 0.105), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="wheel_axle",
    )

    # Side latch fixed hardware on the shell.
    for i, y in enumerate((0.211, -0.211)):
        body.visual(
            Box((0.060, 0.026, 0.060)),
            origin=Origin(xyz=(0.18, y, 0.515)),
            material=metal,
            name=f"latch_pivot_plate_{i}",
        )

    # Articulated lid, modeled as a hollow cap with downturned lips.
    lid = model.part("lid")
    lid.visual(
        Box((0.78, 0.405, 0.035)),
        origin=Origin(xyz=(0.415, 0.0, 0.052)),
        material=yellow,
        name="lid_top_panel",
    )
    lid.visual(
        Box((0.040, 0.405, 0.080)),
        origin=Origin(xyz=(0.785, 0.0, 0.020)),
        material=yellow,
        name="front_lip",
    )
    lid.visual(
        Box((0.76, 0.035, 0.075)),
        origin=Origin(xyz=(0.415, 0.202, 0.0175)),
        material=yellow,
        name="side_lip_0",
    )
    lid.visual(
        Box((0.76, 0.035, 0.075)),
        origin=Origin(xyz=(0.415, -0.202, 0.0175)),
        material=yellow,
        name="side_lip_1",
    )
    lid.visual(
        Box((0.43, 0.25, 0.015)),
        origin=Origin(xyz=(0.43, 0.0, 0.077)),
        material=dark_plastic,
        name="top_recess_pad",
    )
    lid.visual(
        Box((0.080, 0.360, 0.035)),
        origin=Origin(xyz=(0.070, 0.0, 0.0175)),
        material=yellow,
        name="rear_lid_strip",
    )
    lid.visual(
        Box((0.025, 0.310, 0.040)),
        origin=Origin(xyz=(0.071, 0.0, -0.020)),
        material=metal,
        name="lid_hinge_leaf",
    )
    for i, y in enumerate((0.219, -0.219)):
        lid.visual(
            Box((0.052, 0.014, 0.050)),
            origin=Origin(xyz=(0.60, y, 0.000)),
            material=metal,
            name=("side_catch_0", "side_catch_1")[i],
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.411, 0.0, 0.610)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=1.35),
    )

    # Telescoping rear pull handle.
    rear_handle = model.part("rear_handle")
    for i, y in enumerate((0.14, -0.14)):
        rear_handle.visual(
            Box((0.022, 0.036, 0.670)),
            origin=Origin(xyz=(0.0, y, -0.200)),
            material=metal,
            name=("handle_rod_0", "handle_rod_1")[i],
        )
    rear_handle.visual(
        Box((0.045, 0.34, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=metal,
        name="handle_crossbar",
    )
    rear_handle.visual(
        Cylinder(radius=0.026, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, 0.172), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="rubber_grip",
    )
    model.articulation(
        "body_to_rear_handle",
        ArticulationType.PRISMATIC,
        parent=body,
        child=rear_handle,
        origin=Origin(xyz=(-0.470, 0.0, 0.640)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.25, lower=0.0, upper=0.35),
    )

    # Front accessory drawer below the lid.
    front_drawer = model.part("front_drawer")
    front_drawer.visual(
        Box((0.040, 0.270, 0.120)),
        origin=Origin(xyz=(0.005, 0.0, 0.0)),
        material=yellow,
        name="drawer_front",
    )
    front_drawer.visual(
        Box((0.460, 0.264, 0.070)),
        origin=Origin(xyz=(-0.235, 0.0, -0.006)),
        material=dark_plastic,
        name="drawer_tray",
    )
    front_drawer.visual(
        Box((0.450, 0.018, 0.018)),
        origin=Origin(xyz=(-0.240, 0.123, -0.050)),
        material=metal,
        name="drawer_slide_0",
    )
    front_drawer.visual(
        Box((0.450, 0.018, 0.018)),
        origin=Origin(xyz=(-0.240, -0.123, -0.050)),
        material=metal,
        name="drawer_slide_1",
    )
    front_drawer.visual(
        Box((0.014, 0.160, 0.026)),
        origin=Origin(xyz=(0.032, 0.0, 0.000)),
        material=dark_plastic,
        name="drawer_pull",
    )
    model.articulation(
        "body_to_front_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=front_drawer,
        origin=Origin(xyz=(0.390, 0.0, 0.490)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.30, lower=0.0, upper=0.28),
    )

    # Side latch levers pivot on the shell and bridge across the lid/body seam.
    for i, y in enumerate((0.232, -0.232)):
        latch = model.part(f"side_latch_{i}")
        latch.visual(
            Cylinder(radius=0.020, length=0.016),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name="pivot_cap",
        )
        latch.visual(
            Box((0.045, 0.012, 0.160)),
            origin=Origin(xyz=(0.0, 0.0, 0.065)),
            material=latch_red,
            name="latch_bar",
        )
        latch.visual(
            Box((0.060, 0.012, 0.028)),
            origin=Origin(xyz=(0.010, 0.0, 0.140)),
            material=metal,
            name="hook_nose",
        )
        model.articulation(
            f"body_to_side_latch_{i}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=latch,
            origin=Origin(xyz=(0.18, y, 0.515)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=0.0, upper=1.15),
        )

    # Two rubber transport wheels on the common rear axle.
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.105,
            0.060,
            inner_radius=0.072,
            tread=TireTread(style="block", depth=0.006, count=18, land_ratio=0.55),
            sidewall=TireSidewall(style="square", bulge=0.02),
            shoulder=TireShoulder(width=0.006, radius=0.002),
        ),
        "utility_tire",
    )
    rim_mesh = mesh_from_geometry(
        WheelGeometry(
            0.074,
            0.052,
            rim=WheelRim(inner_radius=0.040, flange_height=0.006, flange_thickness=0.004),
            hub=WheelHub(
                radius=0.026,
                width=0.040,
                cap_style="flat",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.034, hole_diameter=0.004),
            ),
            face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.004, window_radius=0.010),
            bore=WheelBore(style="round", diameter=0.028),
        ),
        "utility_wheel_rim",
    )
    for i, y in enumerate((0.245, -0.245)):
        wheel = model.part(f"wheel_{i}")
        wheel.visual(tire_mesh, material=black, name="tire")
        wheel.visual(rim_mesh, material=metal, name="rim")
        wheel.visual(
            Cylinder(radius=0.016, length=0.058),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal,
            name="axle_bushing",
        )
        model.articulation(
            f"body_to_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(-0.300, y, 0.105), rpy=(0.0, 0.0, math.pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=25.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    rear_handle = object_model.get_part("rear_handle")
    front_drawer = object_model.get_part("front_drawer")
    latch_0 = object_model.get_part("side_latch_0")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")

    lid_joint = object_model.get_articulation("body_to_lid")
    handle_joint = object_model.get_articulation("body_to_rear_handle")
    drawer_joint = object_model.get_articulation("body_to_front_drawer")
    latch_joint = object_model.get_articulation("body_to_side_latch_0")
    wheel_joint = object_model.get_articulation("body_to_wheel_0")

    for wheel in (wheel_0, wheel_1):
        ctx.allow_overlap(
            wheel,
            body,
            elem_a="axle_bushing",
            elem_b="wheel_axle",
            reason="The wheel bushing is intentionally captured around the fixed axle.",
        )
        ctx.expect_overlap(
            wheel,
            body,
            axes="yz",
            elem_a="axle_bushing",
            elem_b="wheel_axle",
            min_overlap=0.020,
            name=f"{wheel.name} bushing surrounds axle",
        )

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="front_lip",
        negative_elem="front_top_rail",
        min_gap=0.015,
        max_gap=0.025,
        name="closed lid sits above body seam",
    )
    ctx.expect_gap(
        front_drawer,
        body,
        axis="x",
        positive_elem="drawer_front",
        negative_elem="front_lower_wall",
        min_gap=0.003,
        max_gap=0.012,
        name="drawer front is proud of outer shell",
    )
    ctx.expect_overlap(
        latch_0,
        lid,
        axes="z",
        elem_a="latch_bar",
        elem_b="side_catch_0",
        min_overlap=0.030,
        name="side latch bridges up to lid catch",
    )
    ctx.expect_overlap(
        rear_handle,
        body,
        axes="z",
        elem_a="handle_rod_0",
        elem_b="guide_web_0",
        min_overlap=0.40,
        name="collapsed handle remains in guide rail",
    )
    ctx.expect_overlap(
        front_drawer,
        body,
        axes="x",
        elem_a="drawer_slide_0",
        elem_b="drawer_runner_0",
        min_overlap=0.25,
        name="closed drawer retained on straight runner",
    )

    rest_handle = ctx.part_world_position(rear_handle)
    with ctx.pose({handle_joint: 0.35}):
        extended_handle = ctx.part_world_position(rear_handle)
        ctx.expect_overlap(
            rear_handle,
            body,
            axes="z",
            elem_a="handle_rod_0",
            elem_b="guide_web_0",
            min_overlap=0.12,
            name="extended handle still retained in guide rail",
        )
    ctx.check(
        "handle slides upward",
        rest_handle is not None
        and extended_handle is not None
        and extended_handle[2] > rest_handle[2] + 0.30,
        details=f"rest={rest_handle}, extended={extended_handle}",
    )

    rest_drawer = ctx.part_world_position(front_drawer)
    with ctx.pose({drawer_joint: 0.28}):
        extended_drawer = ctx.part_world_position(front_drawer)
        ctx.expect_overlap(
            front_drawer,
            body,
            axes="x",
            elem_a="drawer_slide_0",
            elem_b="drawer_runner_0",
            min_overlap=0.035,
            name="extended drawer keeps runner engagement",
        )
    ctx.check(
        "drawer slides forward",
        rest_drawer is not None
        and extended_drawer is not None
        and extended_drawer[0] > rest_drawer[0] + 0.25,
        details=f"rest={rest_drawer}, extended={extended_drawer}",
    )

    rest_lid = ctx.part_world_aabb(lid)
    with ctx.pose({latch_joint: 1.0, lid_joint: 1.1}):
        open_lid = ctx.part_world_aabb(lid)
    ctx.check(
        "lid rotates upward on rear hinge",
        rest_lid is not None
        and open_lid is not None
        and open_lid[1][2] > rest_lid[1][2] + 0.18,
        details=f"closed={rest_lid}, open={open_lid}",
    )

    rest_wheel = ctx.part_world_aabb(wheel_0)
    with ctx.pose({wheel_joint: math.pi / 2.0}):
        spun_wheel = ctx.part_world_aabb(wheel_0)
    ctx.check(
        "wheel has continuous spin articulation",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS and spun_wheel is not None and rest_wheel is not None,
        details=f"joint={wheel_joint.articulation_type}, rest={rest_wheel}, spun={spun_wheel}",
    )

    return ctx.report()


object_model = build_object_model()
