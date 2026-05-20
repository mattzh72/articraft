from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_tool_cart")

    red = Material("powder_coated_red", rgba=(0.78, 0.05, 0.035, 1.0))
    dark_red = Material("shadowed_red", rgba=(0.46, 0.025, 0.02, 1.0))
    black = Material("matte_black", rgba=(0.01, 0.01, 0.01, 1.0))
    rubber = Material("black_rubber", rgba=(0.006, 0.006, 0.005, 1.0))
    steel = Material("brushed_steel", rgba=(0.62, 0.62, 0.58, 1.0))
    zinc = Material("zinc_hardware", rgba=(0.78, 0.77, 0.70, 1.0))
    drawer_dark = Material("drawer_shadow", rgba=(0.13, 0.13, 0.13, 1.0))
    label = Material("small_white_labels", rgba=(0.92, 0.90, 0.82, 1.0))

    body = model.part("cart_body")

    # Welded square-tube frame and two lipped trays.
    for i, (x, y) in enumerate(
        ((-0.37, -0.21), (0.37, -0.21), (-0.37, 0.21), (0.37, 0.21))
    ):
        body.visual(
            Box((0.045, 0.045, 0.68)),
            origin=Origin(xyz=(x, y, 0.50)),
            material=black,
            name=f"corner_post_{i}",
        )
        body.visual(
            Box((0.105, 0.095, 0.012)),
            origin=Origin(xyz=(x, y, 0.164)),
            material=zinc,
            name=f"caster_mount_{i}",
        )

    for y, name in ((-0.23, "front"), (0.23, "rear")):
        body.visual(
            Box((0.82, 0.035, 0.045)),
            origin=Origin(xyz=(0.0, y, 0.815)),
            material=black,
            name=f"{name}_top_tube",
        )
        body.visual(
            Box((0.78, 0.035, 0.040)),
            origin=Origin(xyz=(0.0, y, 0.275)),
            material=black,
            name=f"{name}_lower_tube",
        )
    for x, name in ((-0.39, "side_0"), (0.39, "side_1")):
        body.visual(
            Box((0.035, 0.50, 0.045)),
            origin=Origin(xyz=(x, 0.0, 0.815)),
            material=black,
            name=f"{name}_top_tube",
        )
        body.visual(
            Box((0.035, 0.46, 0.040)),
            origin=Origin(xyz=(x, 0.0, 0.275)),
            material=black,
            name=f"{name}_lower_tube",
        )

    body.visual(
        Box((0.82, 0.48, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.820)),
        material=red,
        name="top_tray_floor",
    )
    body.visual(
        Box((0.83, 0.035, 0.085)),
        origin=Origin(xyz=(0.0, -0.242, 0.872)),
        material=red,
        name="top_tray_front_lip",
    )
    body.visual(
        Box((0.83, 0.035, 0.085)),
        origin=Origin(xyz=(0.0, 0.242, 0.872)),
        material=red,
        name="top_tray_rear_lip",
    )
    body.visual(
        Box((0.035, 0.49, 0.085)),
        origin=Origin(xyz=(-0.405, 0.0, 0.872)),
        material=red,
        name="top_tray_side_0",
    )
    body.visual(
        Box((0.035, 0.49, 0.085)),
        origin=Origin(xyz=(0.405, 0.0, 0.872)),
        material=red,
        name="top_tray_side_1",
    )
    body.visual(
        Box((0.70, 0.36, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.837)),
        material=black,
        name="top_rubber_mat",
    )

    body.visual(
        Box((0.76, 0.43, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
        material=red,
        name="lower_tray_floor",
    )
    body.visual(
        Box((0.77, 0.032, 0.072)),
        origin=Origin(xyz=(0.0, -0.216, 0.292)),
        material=red,
        name="lower_tray_front_lip",
    )
    body.visual(
        Box((0.77, 0.032, 0.072)),
        origin=Origin(xyz=(0.0, 0.216, 0.292)),
        material=red,
        name="lower_tray_rear_lip",
    )
    body.visual(
        Box((0.032, 0.43, 0.072)),
        origin=Origin(xyz=(-0.384, 0.0, 0.292)),
        material=red,
        name="lower_tray_side_0",
    )
    body.visual(
        Box((0.032, 0.43, 0.072)),
        origin=Origin(xyz=(0.384, 0.0, 0.292)),
        material=red,
        name="lower_tray_side_1",
    )

    # Sheet-metal drawer chest: open front with side rails and rear panel.
    body.visual(
        Box((0.025, 0.39, 0.315)),
        origin=Origin(xyz=(-0.342, -0.025, 0.585)),
        material=red,
        name="cabinet_side_0",
    )
    body.visual(
        Box((0.025, 0.39, 0.315)),
        origin=Origin(xyz=(0.342, -0.025, 0.585)),
        material=red,
        name="cabinet_side_1",
    )
    body.visual(
        Box((0.690, 0.39, 0.025)),
        origin=Origin(xyz=(0.0, -0.025, 0.740)),
        material=red,
        name="cabinet_top",
    )
    body.visual(
        Box((0.690, 0.39, 0.025)),
        origin=Origin(xyz=(0.0, -0.025, 0.430)),
        material=red,
        name="cabinet_bottom",
    )
    body.visual(
        Box((0.690, 0.025, 0.315)),
        origin=Origin(xyz=(0.0, 0.170, 0.585)),
        material=red,
        name="cabinet_rear",
    )
    for z, name in ((0.735, "front_rail_top"), (0.435, "front_rail_bottom")):
        body.visual(
            Box((0.690, 0.025, 0.030)),
            origin=Origin(xyz=(0.0, -0.220, z)),
            material=dark_red,
            name=name,
        )
    for x, name in ((-0.342, "front_stile_0"), (0.342, "front_stile_1")):
        body.visual(
            Box((0.030, 0.025, 0.315)),
            origin=Origin(xyz=(x, -0.220, 0.585)),
            material=dark_red,
            name=name,
        )
    for z, name in ((0.635, "drawer_divider_0"), (0.545, "drawer_divider_1")):
        body.visual(
            Box((0.675, 0.025, 0.016)),
            origin=Origin(xyz=(0.0, -0.220, z)),
            material=dark_red,
            name=name,
        )

    for drawer_i, zc in enumerate((0.680, 0.590, 0.505)):
        for side, x in enumerate((-0.323, 0.323)):
            body.visual(
                Box((0.020, 0.315, 0.018)),
                origin=Origin(xyz=(x, -0.035, zc - 0.020)),
                material=steel,
                name=f"slide_rail_{drawer_i}_{side}",
            )

    # Rear push handle with visible welded brackets.
    for x, name in ((-0.275, "handle_upright_0"), (0.275, "handle_upright_1")):
        body.visual(
            Cylinder(radius=0.017, length=0.23),
            origin=Origin(xyz=(x, 0.285, 0.900), rpy=(0.0, 0.0, 0.0)),
            material=black,
            name=name,
        )
        body.visual(
            Box((0.065, 0.048, 0.035)),
            origin=Origin(xyz=(x, 0.266, 0.795)),
            material=black,
            name=f"handle_bracket_{name[-1]}",
        )
        body.visual(
            Box((0.055, 0.050, 0.050)),
            origin=Origin(xyz=(x, 0.285, 1.006)),
            material=black,
            name=f"handle_elbow_{name[-1]}",
        )
    body.visual(
        Cylinder(radius=0.020, length=0.60),
        origin=Origin(xyz=(0.0, 0.285, 1.015), rpy=(0.0, pi / 2, 0.0)),
        material=black,
        name="push_grip",
    )

    # Small workshop details: bolted plates, side hooks, label plates.
    for i, (x, y) in enumerate(
        ((-0.37, -0.21), (0.37, -0.21), (-0.37, 0.21), (0.37, 0.21))
    ):
        for bx, by in ((-0.028, -0.023), (0.028, -0.023), (-0.028, 0.023), (0.028, 0.023)):
            body.visual(
                Cylinder(radius=0.006, length=0.004),
                origin=Origin(xyz=(x + bx, y + by, 0.171)),
                material=steel,
                name=f"mount_bolt_{i}_{bx:+.0e}_{by:+.0e}",
            )
    for i, z in enumerate((0.50, 0.56, 0.62)):
        body.visual(
            Box((0.010, 0.030, 0.030)),
            origin=Origin(xyz=(-0.354, -0.075 + i * 0.075, z)),
            material=black,
            name=f"side_hook_base_{i}",
        )
        body.visual(
            Cylinder(radius=0.010, length=0.045),
            origin=Origin(xyz=(-0.366, -0.075 + i * 0.075, z), rpy=(pi / 2, 0.0, 0.0)),
            material=black,
            name=f"side_hook_{i}",
        )

    def add_drawer(part_name: str, zc: float, front_height: float, pan_wall: float) -> None:
        drawer = model.part(part_name)
        drawer.visual(
            Box((0.620, 0.018, front_height)),
            origin=Origin(xyz=(0.0, -0.185, 0.0)),
            material=red,
            name="front_panel",
        )
        drawer.visual(
            Box((0.580, 0.312, 0.012)),
            origin=Origin(xyz=(0.0, -0.020, -front_height / 2 + 0.006)),
            material=drawer_dark,
            name="bottom_pan",
        )
        for side, x in enumerate((-0.297, 0.297)):
            drawer.visual(
                Box((0.014, 0.312, pan_wall)),
                origin=Origin(xyz=(x, -0.020, -front_height / 2 + pan_wall / 2)),
                material=drawer_dark,
                name=f"side_wall_{side}",
            )
        drawer.visual(
            Box((0.580, 0.014, pan_wall)),
            origin=Origin(xyz=(0.0, 0.136, -front_height / 2 + pan_wall / 2)),
            material=drawer_dark,
            name="rear_wall",
        )
        drawer.visual(
            Box((0.530, 0.010, 0.010)),
            origin=Origin(xyz=(0.0, -0.171, -front_height / 2 + pan_wall - 0.004)),
            material=steel,
            name="front_inner_flange",
        )
        for x in (-0.190, 0.190):
            drawer.visual(
                Box((0.030, 0.042, 0.020)),
                origin=Origin(xyz=(x, -0.207, 0.004)),
                material=steel,
                name=f"handle_standoff_{x:+.2f}",
            )
        drawer.visual(
            Cylinder(radius=0.009, length=0.440),
            origin=Origin(xyz=(0.0, -0.230, 0.004), rpy=(0.0, pi / 2, 0.0)),
            material=steel,
            name="handle_bar",
        )
        drawer.visual(
            Box((0.095, 0.004, 0.020)),
            origin=Origin(xyz=(0.0, -0.196, 0.020)),
            material=label,
            name="label_plate",
        )
        model.articulation(
            f"body_to_{part_name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=drawer,
            origin=Origin(xyz=(0.0, -0.055, zc)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=0.285),
            motion_properties=MotionProperties(damping=2.0, friction=8.0),
        )

    add_drawer("top_drawer", 0.680, 0.076, 0.050)
    add_drawer("middle_drawer", 0.590, 0.078, 0.052)
    add_drawer("bottom_drawer", 0.505, 0.100, 0.070)

    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.055,
            0.034,
            inner_radius=0.035,
            carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.03),
            tread=TireTread(style="block", depth=0.004, count=18, land_ratio=0.56),
            grooves=(TireGroove(center_offset=0.0, width=0.004, depth=0.002),),
            sidewall=TireSidewall(style="square", bulge=0.02),
            shoulder=TireShoulder(width=0.004, radius=0.002),
        ),
        "caster_tire",
    )
    rim_mesh = mesh_from_geometry(
        WheelGeometry(
            0.039,
            0.030,
            rim=WheelRim(
                inner_radius=0.026,
                flange_height=0.004,
                flange_thickness=0.0025,
                bead_seat_depth=0.002,
            ),
            hub=WheelHub(
                radius=0.014,
                width=0.024,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=4, circle_diameter=0.020, hole_diameter=0.003),
            ),
            face=WheelFace(dish_depth=0.003, front_inset=0.0015, rear_inset=0.0015),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.0025, window_radius=0.006),
            bore=WheelBore(style="round", diameter=0.008),
        ),
        "caster_rim",
    )

    caster_positions = (
        (-0.37, -0.21),
        (0.37, -0.21),
        (-0.37, 0.21),
        (0.37, 0.21),
    )
    for i, (x, y) in enumerate(caster_positions):
        yoke = model.part(f"caster_yoke_{i}")
        yoke.visual(
            Cylinder(radius=0.040, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=zinc,
            name="swivel_bearing",
        )
        yoke.visual(
            Cylinder(radius=0.015, length=0.032),
            origin=Origin(xyz=(0.0, 0.0, -0.016)),
            material=zinc,
            name="kingpin",
        )
        yoke.visual(
            Box((0.074, 0.046, 0.014)),
            origin=Origin(xyz=(0.0, -0.021, -0.031)),
            material=zinc,
            name="fork_bridge",
        )
        for side, xarm in enumerate((-0.031, 0.031)):
            yoke.visual(
                Box((0.008, 0.120, 0.086)),
                origin=Origin(xyz=(xarm, -0.055, -0.075)),
                material=zinc,
                name=f"fork_arm_{side}",
            )
        yoke.visual(
            Box((0.080, 0.016, 0.018)),
            origin=Origin(xyz=(0.0, -0.107, -0.070)),
            material=zinc,
            name="fork_nose_tie",
        )
        yoke.visual(
            Box((0.022, 0.040, 0.010)),
            origin=Origin(xyz=(0.0, 0.006, -0.034)),
            material=black,
            name="grease_seal",
        )
        if y < 0.0:
            yoke.visual(
                Box((0.018, 0.070, 0.008)),
                origin=Origin(xyz=(0.0, -0.055, -0.021)),
                material=red,
                name="brake_link",
            )
            yoke.visual(
                Box((0.052, 0.022, 0.010)),
                origin=Origin(xyz=(0.0, -0.087, -0.016)),
                material=red,
                name="brake_pedal",
            )

        model.articulation(
            f"body_to_caster_{i}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=yoke,
            origin=Origin(xyz=(x, y, 0.152)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=20.0, velocity=6.0),
            motion_properties=MotionProperties(damping=0.2, friction=0.4),
        )

        wheel = model.part(f"wheel_{i}")
        wheel.visual(tire_mesh, origin=Origin(), material=rubber, name="tire")
        wheel.visual(rim_mesh, origin=Origin(), material=steel, name="rim")
        wheel.visual(
            Cylinder(radius=0.011, length=0.046),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
            material=zinc,
            name="hub_cap",
        )
        model.articulation(
            f"caster_{i}_to_wheel",
            ArticulationType.CONTINUOUS,
            parent=yoke,
            child=wheel,
            origin=Origin(xyz=(0.0, -0.040, -0.092)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=10.0, velocity=18.0),
            motion_properties=MotionProperties(damping=0.05, friction=0.15),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    drawers = ["top_drawer", "middle_drawer", "bottom_drawer"]
    for drawer_name in drawers:
        joint = object_model.get_articulation(f"body_to_{drawer_name}")
        drawer = object_model.get_part(drawer_name)
        rest = ctx.part_world_position(drawer)
        with ctx.pose({joint: 0.24}):
            extended = ctx.part_world_position(drawer)
        ctx.check(
            f"{drawer_name} slides out the front",
            rest is not None and extended is not None and extended[1] < rest[1] - 0.20,
            details=f"rest={rest}, extended={extended}",
        )

    for i in range(4):
        caster = object_model.get_articulation(f"body_to_caster_{i}")
        wheel_joint = object_model.get_articulation(f"caster_{i}_to_wheel")
        wheel = object_model.get_part(f"wheel_{i}")
        ctx.check(
            f"caster_{i} swivels freely",
            caster.articulation_type == ArticulationType.CONTINUOUS and caster.axis == (0.0, 0.0, 1.0),
            details=f"type={caster.articulation_type}, axis={caster.axis}",
        )
        ctx.check(
            f"wheel_{i} rolls on axle",
            wheel_joint.articulation_type == ArticulationType.CONTINUOUS
            and wheel_joint.axis == (1.0, 0.0, 0.0),
            details=f"type={wheel_joint.articulation_type}, axis={wheel_joint.axis}",
        )
        aabb = ctx.part_world_aabb(wheel)
        ctx.check(
            f"wheel_{i} reaches floor",
            aabb is not None and 0.0 <= aabb[0][2] <= 0.012,
            details=f"aabb={aabb}",
        )

    return ctx.report()


object_model = build_object_model()
