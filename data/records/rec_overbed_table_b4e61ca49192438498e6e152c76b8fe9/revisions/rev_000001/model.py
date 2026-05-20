from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireSidewall,
    TireShoulder,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bariatric_overbed_table")

    powder_white = model.material("warm_white_powdercoat", color=(0.88, 0.90, 0.88, 1.0))
    brushed_steel = model.material("brushed_stainless_steel", color=(0.62, 0.65, 0.66, 1.0))
    dark_rubber = model.material("black_rubber", color=(0.015, 0.014, 0.013, 1.0))
    gray_plastic = model.material("gray_caster_plastic", color=(0.19, 0.20, 0.21, 1.0))
    tray_laminate = model.material("speckled_laminate", color=(0.86, 0.80, 0.68, 1.0))
    tray_edge = model.material("molded_tray_edge", color=(0.72, 0.66, 0.55, 1.0))
    safety_red = model.material("red_lock_pedal", color=(0.82, 0.08, 0.05, 1.0))
    dark_handle = model.material("dark_release_paddle", color=(0.08, 0.09, 0.09, 1.0))

    # Shared rounded rubber tire mesh for the four caster wheels.
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.055,
            0.052,
            inner_radius=0.035,
            sidewall=TireSidewall(style="rounded", bulge=0.05),
            shoulder=TireShoulder(width=0.004, radius=0.003),
        ),
        "caster_rubber_tire",
    )

    base = model.part("base")

    # Wide H-base frame, sized for a patient-room bariatric overbed table.
    base.visual(Box((0.10, 1.10, 0.065)), origin=Origin(xyz=(-0.50, 0.0, 0.165)), material=powder_white, name="rail_0")
    base.visual(Box((0.10, 1.10, 0.065)), origin=Origin(xyz=(0.50, 0.0, 0.165)), material=powder_white, name="rail_1")
    base.visual(Box((1.08, 0.12, 0.065)), origin=Origin(xyz=(0.0, 0.0, 0.165)), material=powder_white, name="cross_rail")
    base.visual(Box((0.18, 0.18, 0.030)), origin=Origin(xyz=(-0.28, 0.0, 0.205)), material=powder_white, name="left_post_foot")
    base.visual(Box((0.18, 0.18, 0.030)), origin=Origin(xyz=(0.28, 0.0, 0.205)), material=powder_white, name="right_post_foot")

    # Four hollow outer lift columns.  Each sleeve is represented as four
    # visible walls around a clear bore so the inner post can actually slide
    # through it without needing overlap allowances.
    sleeve_outer = 0.085
    wall = 0.012
    sleeve_h = 0.560
    sleeve_z = 0.420
    sleeve_offsets = (-0.28, 0.28)
    for label, x in (("left", sleeve_offsets[0]), ("right", sleeve_offsets[1])):
        side = sleeve_outer / 2.0 - wall / 2.0
        base.visual(
            Box((sleeve_outer, wall, sleeve_h)),
            origin=Origin(xyz=(x, side, sleeve_z)),
            material=brushed_steel,
            name=f"{label}_outer_front_wall",
        )
        base.visual(
            Box((sleeve_outer, wall, sleeve_h)),
            origin=Origin(xyz=(x, -side, sleeve_z)),
            material=brushed_steel,
            name=f"{label}_outer_rear_wall",
        )
        base.visual(
            Box((wall, sleeve_outer, sleeve_h)),
            origin=Origin(xyz=(x - side, 0.0, sleeve_z)),
            material=brushed_steel,
            name=f"{label}_outer_side_0",
        )
        base.visual(
            Box((wall, sleeve_outer, sleeve_h)),
            origin=Origin(xyz=(x + side, 0.0, sleeve_z)),
            material=brushed_steel,
            name=f"{label}_outer_side_1",
        )
        base.visual(
            Box((0.105, wall, 0.018)),
            origin=Origin(xyz=(x, side, 0.704)),
            material=powder_white,
            name=f"{label}_collar_front",
        )
        base.visual(
            Box((0.105, wall, 0.018)),
            origin=Origin(xyz=(x, -side, 0.704)),
            material=powder_white,
            name=f"{label}_collar_rear",
        )
        base.visual(
            Box((wall, 0.105, 0.018)),
            origin=Origin(xyz=(x - side, 0.0, 0.704)),
            material=powder_white,
            name=f"{label}_collar_side_0",
        )
        base.visual(
            Box((wall, 0.105, 0.018)),
            origin=Origin(xyz=(x + side, 0.0, 0.704)),
            material=powder_white,
            name=f"{label}_collar_side_1",
        )

    # Individually visible caster forks and mounting hardware under each base
    # corner.  The fork side plates are fixed to the base; the wheel itself is a
    # separate continuous-spin child part.
    caster_positions = [
        (-0.50, -0.45),
        (0.50, -0.45),
        (-0.50, 0.45),
        (0.50, 0.45),
    ]
    for i, (x, y) in enumerate(caster_positions):
        base.visual(
            Box((0.14, 0.11, 0.014)),
            origin=Origin(xyz=(x, y, 0.125)),
            material=powder_white,
            name=f"caster_mount_{i}",
        )
        base.visual(
            Cylinder(radius=0.016, length=0.042),
            origin=Origin(xyz=(x, y, 0.154)),
            material=brushed_steel,
            name=f"caster_stem_{i}",
        )
        base.visual(
            Box((0.008, 0.092, 0.104)),
            origin=Origin(xyz=(x - 0.036, y, 0.092)),
            material=powder_white,
            name=f"caster_fork_{i}_0",
        )
        base.visual(
            Box((0.008, 0.092, 0.104)),
            origin=Origin(xyz=(x + 0.036, y, 0.092)),
            material=powder_white,
            name=f"caster_fork_{i}_1",
        )
        y_sign = 1.0 if y > 0.0 else -1.0
        base.visual(
            Box((0.090, 0.018, 0.012)),
            origin=Origin(xyz=(x, y + y_sign * 0.055, 0.124)),
            material=powder_white,
            name=f"lock_hinge_bracket_{i}",
        )

    left_inner = model.part("left_inner_post")
    left_inner.visual(
        Box((0.048, 0.048, 0.660)),
        origin=Origin(xyz=(0.0, 0.0, -0.100)),
        material=brushed_steel,
        name="left_inner_tube",
    )
    left_inner.visual(
        Box((0.064, 0.064, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.221)),
        material=brushed_steel,
        name="left_top_plate",
    )
    left_inner.visual(
        Box((0.0065, 0.030, 0.050)),
        origin=Origin(xyz=(0.02725, 0.0, -0.250)),
        material=brushed_steel,
        name="left_slide_guide",
    )

    right_inner = model.part("right_inner_post")
    right_inner.visual(
        Box((0.048, 0.048, 0.660)),
        origin=Origin(xyz=(0.0, 0.0, -0.100)),
        material=brushed_steel,
        name="right_inner_tube",
    )
    right_inner.visual(
        Box((0.064, 0.064, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.221)),
        material=brushed_steel,
        name="right_top_plate",
    )
    right_inner.visual(
        Box((0.0065, 0.030, 0.050)),
        origin=Origin(xyz=(0.02725, 0.0, -0.250)),
        material=brushed_steel,
        name="right_slide_guide",
    )

    model.articulation(
        "left_post_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=left_inner,
        origin=Origin(xyz=(-0.28, 0.0, 0.680)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=600.0, velocity=0.18, lower=0.0, upper=0.220),
    )
    model.articulation(
        "right_post_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=right_inner,
        origin=Origin(xyz=(0.28, 0.0, 0.680)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=600.0, velocity=0.18, lower=0.0, upper=0.220),
        mimic=Mimic(joint="left_post_slide"),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.700, 0.075, 0.040)),
        origin=Origin(xyz=(0.280, 0.0, 0.020)),
        material=powder_white,
        name="top_crossbar",
    )
    carriage.visual(
        Box((0.100, 0.100, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=powder_white,
        name="left_socket",
    )
    carriage.visual(
        Box((0.100, 0.100, 0.020)),
        origin=Origin(xyz=(0.560, 0.0, 0.010)),
        material=powder_white,
        name="right_socket",
    )
    for x in (0.0, 0.560):
        carriage.visual(
            Box((0.065, 0.420, 0.025)),
            origin=Origin(xyz=(x, -0.145, 0.037)),
            material=powder_white,
            name=f"tilt_arm_{0 if x == 0.0 else 1}",
        )
    carriage.visual(
        Cylinder(radius=0.014, length=1.120),
        origin=Origin(xyz=(0.280, -0.340, 0.055), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_steel,
        name="tray_hinge_rod",
    )

    model.articulation(
        "left_post_to_carriage",
        ArticulationType.FIXED,
        parent=left_inner,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.230)),
    )

    tray = model.part("tray")
    tray.visual(
        Box((1.220, 0.540, 0.030)),
        origin=Origin(xyz=(0.0, 0.340, 0.015)),
        material=tray_laminate,
        name="tray_panel",
    )
    tray.visual(
        Box((1.260, 0.035, 0.060)),
        origin=Origin(xyz=(0.0, 0.610, 0.040)),
        material=tray_edge,
        name="front_lip",
    )
    tray.visual(
        Box((1.030, 0.026, 0.048)),
        origin=Origin(xyz=(0.0, 0.070, 0.036)),
        material=tray_edge,
        name="rear_lip",
    )
    tray.visual(
        Box((0.035, 0.565, 0.060)),
        origin=Origin(xyz=(-0.625, 0.340, 0.040)),
        material=tray_edge,
        name="side_lip_0",
    )
    tray.visual(
        Box((0.035, 0.565, 0.060)),
        origin=Origin(xyz=(0.625, 0.340, 0.040)),
        material=tray_edge,
        name="side_lip_1",
    )
    for i, x in enumerate((-0.410, 0.0, 0.410)):
        tray.visual(
            Box((0.180, 0.075, 0.012)),
            origin=Origin(xyz=(x, 0.042, 0.018)),
            material=brushed_steel,
            name=f"hinge_leaf_{i}",
        )
    for i, x in enumerate((-0.140, 0.140)):
        tray.visual(
            Box((0.026, 0.036, 0.027)),
            origin=Origin(xyz=(x, 0.500, -0.0135)),
            material=brushed_steel,
            name=f"paddle_bracket_{i}",
        )

    model.articulation(
        "tray_tilt",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=tray,
        origin=Origin(xyz=(0.280, -0.340, 0.055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.6, lower=-0.12, upper=0.52),
    )

    release_paddle = model.part("release_paddle")
    release_paddle.visual(
        Cylinder(radius=0.008, length=0.320),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_steel,
        name="paddle_pivot",
    )
    release_paddle.visual(
        Box((0.310, 0.070, 0.012)),
        origin=Origin(xyz=(0.0, 0.055, -0.004)),
        material=dark_handle,
        name="paddle_plate",
    )
    release_paddle.visual(
        Box((0.025, 0.090, 0.012)),
        origin=Origin(xyz=(-0.120, 0.030, -0.001)),
        material=brushed_steel,
        name="paddle_arm_0",
    )
    release_paddle.visual(
        Box((0.025, 0.090, 0.012)),
        origin=Origin(xyz=(0.120, 0.030, -0.001)),
        material=brushed_steel,
        name="paddle_arm_1",
    )
    model.articulation(
        "release_paddle_pull",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=release_paddle,
        origin=Origin(xyz=(0.0, 0.500, -0.035)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=0.45),
    )

    # Four wheel parts and four independent locking tabs.
    for i, (x, y) in enumerate(caster_positions):
        wheel = model.part(f"caster_wheel_{i}")
        wheel.visual(
            Cylinder(radius=0.040, length=0.046),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=gray_plastic,
            name="rim",
        )
        wheel.visual(
            Cylinder(radius=0.014, length=0.064),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=gray_plastic,
            name="axle_bushing",
        )
        wheel.visual(tire_mesh, origin=Origin(), material=dark_rubber, name="tire")
        model.articulation(
            f"caster_spin_{i}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=wheel,
            origin=Origin(xyz=(x, y, 0.058)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=12.0),
        )

        y_sign = 1.0 if y > 0.0 else -1.0
        lock_tab = model.part(f"lock_tab_{i}")
        lock_tab.visual(
            Cylinder(radius=0.006, length=0.090),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=brushed_steel,
            name="tab_pivot",
        )
        lock_tab.visual(
            Box((0.075, 0.052, 0.012)),
            origin=Origin(xyz=(0.0, y_sign * 0.026, -0.010)),
            material=safety_red,
            name="lock_pedal",
        )
        model.articulation(
            f"lock_tab_pivot_{i}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=lock_tab,
            origin=Origin(xyz=(x, y + y_sign * 0.055, 0.112)),
            axis=(-y_sign, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=0.0, upper=0.65),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    left_inner = object_model.get_part("left_inner_post")
    right_inner = object_model.get_part("right_inner_post")
    carriage = object_model.get_part("carriage")
    tray = object_model.get_part("tray")
    release_paddle = object_model.get_part("release_paddle")
    left_slide = object_model.get_articulation("left_post_slide")
    tray_tilt = object_model.get_articulation("tray_tilt")
    paddle_pull = object_model.get_articulation("release_paddle_pull")

    ctx.expect_contact(
        left_inner,
        carriage,
        elem_a="left_inner_tube",
        elem_b="left_socket",
        name="left lift post bears on shared carriage",
    )
    ctx.expect_contact(
        right_inner,
        carriage,
        elem_a="right_inner_tube",
        elem_b="right_socket",
        name="right lift post bears on shared carriage",
    )

    left_rest = ctx.part_world_position(left_inner)
    right_rest = ctx.part_world_position(right_inner)
    with ctx.pose({left_slide: 0.220}):
        left_high = ctx.part_world_position(left_inner)
        right_high = ctx.part_world_position(right_inner)
        ctx.expect_contact(
            right_inner,
            carriage,
            elem_a="right_inner_tube",
            elem_b="right_socket",
            name="matched post remains under carriage when raised",
        )
    ctx.check(
        "twin posts rise prismatically together",
        left_rest is not None
        and right_rest is not None
        and left_high is not None
        and right_high is not None
        and left_high[2] > left_rest[2] + 0.20
        and right_high[2] > right_rest[2] + 0.20
        and abs((left_high[2] - left_rest[2]) - (right_high[2] - right_rest[2])) < 1e-6,
        details=f"left_rest={left_rest}, left_high={left_high}, right_rest={right_rest}, right_high={right_high}",
    )

    tray_front_rest = ctx.part_element_world_aabb(tray, elem="front_lip")
    with ctx.pose({tray_tilt: 0.50}):
        tray_front_tilted = ctx.part_element_world_aabb(tray, elem="front_lip")
    ctx.check(
        "tray tilts upward about transverse hinge",
        tray_front_rest is not None
        and tray_front_tilted is not None
        and tray_front_tilted[1][2] > tray_front_rest[1][2] + 0.18,
        details=f"rest={tray_front_rest}, tilted={tray_front_tilted}",
    )

    paddle_rest = ctx.part_world_aabb(release_paddle)
    with ctx.pose({paddle_pull: 0.45}):
        paddle_pulled = ctx.part_world_aabb(release_paddle)
    ctx.check(
        "release paddle rotates down below work surface",
        paddle_rest is not None and paddle_pulled is not None and paddle_pulled[0][2] < paddle_rest[0][2] - 0.015,
        details=f"rest={paddle_rest}, pulled={paddle_pulled}",
    )

    for i in range(4):
        spin = object_model.get_articulation(f"caster_spin_{i}")
        tab_joint = object_model.get_articulation(f"lock_tab_pivot_{i}")
        tab = object_model.get_part(f"lock_tab_{i}")
        ctx.check(
            f"caster {i} spins continuously",
            "continuous" in str(spin.articulation_type).lower(),
            details=str(spin.articulation_type),
        )
        tab_rest = ctx.part_world_aabb(tab)
        with ctx.pose({tab_joint: 0.65}):
            tab_rotated = ctx.part_world_aabb(tab)
        ctx.check(
            f"lock tab {i} rotates on local pivot",
            tab_rest is not None and tab_rotated is not None and tab_rotated[0][2] < tab_rest[0][2] - 0.010,
            details=f"rest={tab_rest}, rotated={tab_rotated}",
        )

    return ctx.report()


object_model = build_object_model()
