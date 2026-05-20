from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
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


def _arch_points(cx: float, cz: float, radius: float, count: int = 16):
    """Concave wheel-arch boundary points from lower-front to lower-rear."""
    start = math.radians(194.0)
    end = math.radians(-14.0)
    return [
        (cx + radius * math.cos(start + (end - start) * i / (count - 1)),
         cz + radius * math.sin(start + (end - start) * i / (count - 1)))
        for i in range(count)
    ]


def _hatchback_body_geometry():
    width = 0.96
    front_arch = _arch_points(-0.55, 0.20, 0.25)
    rear_arch = _arch_points(0.65, 0.20, 0.25)
    side_profile = [
        (-0.98, 0.14),
        (-0.80, 0.14),
        *front_arch,
        (0.40, 0.14),
        *rear_arch,
        (0.98, 0.14),
        (0.98, 0.40),
        (0.91, 0.70),
        (0.78, 0.86),
        (0.50, 0.96),
        (-0.20, 0.96),
        (-0.36, 0.86),
        (-0.50, 0.55),
        (-0.78, 0.48),
        (-0.98, 0.34),
    ]
    # Extrude the side silhouette through local +Z, then rotate so the
    # extrusion becomes the car width along world Y and the profile's second
    # coordinate becomes world Z.
    return ExtrudeGeometry(side_profile, width, center=True, closed=True).rotate_x(math.pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toy_hatchback_car")

    body_blue = Material("painted_toy_blue", rgba=(0.05, 0.28, 0.82, 1.0))
    dark_glass = Material("smoked_plastic_glass", rgba=(0.02, 0.035, 0.05, 0.72))
    black = Material("soft_black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    silver = Material("molded_silver_plastic", rgba=(0.72, 0.72, 0.68, 1.0))
    warm_white = Material("clear_headlamp", rgba=(1.0, 0.92, 0.64, 1.0))
    red = Material("red_tail_lamp", rgba=(0.85, 0.02, 0.02, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(_hatchback_body_geometry(), "hatchback_body_shell"),
        material=body_blue,
        name="body_shell",
    )
    # Toy-like bumpers, glass, hinge leaves, and axle beams are static
    # body-mounted details, each seated into the main shell.
    body.visual(Box((0.09, 0.82, 0.10)), origin=Origin(xyz=(-0.99, 0.0, 0.25)), material=black, name="front_bumper")
    body.visual(Box((0.07, 0.82, 0.12)), origin=Origin(xyz=(0.99, 0.0, 0.24)), material=black, name="rear_bumper")
    body.visual(Box((0.018, 0.66, 0.30)), origin=Origin(xyz=(-0.44, 0.0, 0.69), rpy=(0.0, -0.42, 0.0)), material=dark_glass, name="windshield")
    body.visual(Box((0.24, 0.006, 0.21)), origin=Origin(xyz=(0.57, 0.481, 0.71)), material=dark_glass, name="left_quarter_window")
    body.visual(Box((0.24, 0.006, 0.21)), origin=Origin(xyz=(0.57, -0.481, 0.71)), material=dark_glass, name="right_quarter_window")
    body.visual(Box((0.012, 0.16, 0.07)), origin=Origin(xyz=(-0.982, 0.25, 0.36)), material=warm_white, name="left_headlamp")
    body.visual(Box((0.012, 0.16, 0.07)), origin=Origin(xyz=(-0.982, -0.25, 0.36)), material=warm_white, name="right_headlamp")

    for axle_name, x in (("front_axle", -0.55), ("rear_axle", 0.65)):
        body.visual(
            Cylinder(radius=0.025, length=1.04),
            origin=Origin(xyz=(x, 0.0, 0.20), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=silver,
            name=axle_name,
        )
        for side, y in (("left", 0.44), ("right", -0.44)):
            body.visual(
                Box((0.06, 0.07, 0.26)),
                origin=Origin(xyz=(x, y, 0.32)),
                material=black,
                name=f"{side}_{axle_name}_strut",
            )

    for side, y, sign in (("left", 0.470, 1.0), ("right", -0.470, -1.0)):
        for idx, z in enumerate((0.50, 0.74)):
            body.visual(
                Box((0.018, 0.026, 0.16)),
                origin=Origin(xyz=(-0.288, sign * 0.491, z)),
                material=silver,
                name=f"{side}_door_hinge_leaf_{idx}",
            )
            body.visual(
                Cylinder(radius=0.014, length=0.15),
                origin=Origin(xyz=(-0.288, y, z)),
                material=silver,
                name=f"{side}_door_hinge_barrel_{idx}",
            )

    body.visual(
        Box((0.24, 0.78, 0.024)),
        origin=Origin(xyz=(0.870, 0.0, 0.858)),
        material=silver,
        name="liftgate_hinge_leaf",
    )
    body.visual(
        Cylinder(radius=0.014, length=0.82),
        origin=Origin(xyz=(0.988, 0.0, 0.858), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="liftgate_hinge_barrel",
    )

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.118,
            0.070,
            rim=WheelRim(inner_radius=0.078, flange_height=0.008, flange_thickness=0.004, bead_seat_depth=0.004),
            hub=WheelHub(
                radius=0.030,
                width=0.070,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.036, hole_diameter=0.004),
            ),
            face=WheelFace(dish_depth=0.006, front_inset=0.003, rear_inset=0.002),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.004, window_radius=0.012),
            bore=WheelBore(style="round", diameter=0.012),
        ),
        "toy_wheel_rim",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.180,
            0.090,
            inner_radius=0.122,
            carcass=TireCarcass(belt_width_ratio=0.68, sidewall_bulge=0.06),
            tread=TireTread(style="block", depth=0.006, count=18, land_ratio=0.58),
            grooves=(TireGroove(center_offset=0.0, width=0.008, depth=0.003),),
            sidewall=TireSidewall(style="rounded", bulge=0.05),
            shoulder=TireShoulder(width=0.008, radius=0.004),
        ),
        "toy_tire",
    )

    wheel_locations = {
        "front_left_wheel": (-0.55, 0.55, 0.20),
        "front_right_wheel": (-0.55, -0.55, 0.20),
        "rear_left_wheel": (0.65, 0.55, 0.20),
        "rear_right_wheel": (0.65, -0.55, 0.20),
    }
    for wheel_name, (x, y, z) in wheel_locations.items():
        wheel = model.part(wheel_name)
        wheel.visual(tire_mesh, material=black, name="tire")
        wheel.visual(wheel_mesh, material=silver, name="rim")
        model.articulation(
            f"body_to_{wheel_name}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(x, y, z), rpy=(0.0, 0.0, math.pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.5, velocity=30.0),
        )

    def make_side_door(name: str, y: float, window_y: float, hinge_axis):
        door = model.part(name)
        door.visual(Box((0.58, 0.032, 0.62)), origin=Origin(xyz=(0.305, 0.0, 0.0)), material=body_blue, name="door_panel")
        door.visual(Box((0.024, 0.022, 0.54)), origin=Origin(xyz=(0.006, 0.0, 0.0)), material=silver, name="hinge_leaf")
        door.visual(Box((0.36, 0.006, 0.22)), origin=Origin(xyz=(0.29, window_y, 0.13)), material=dark_glass, name="door_window")
        door.visual(Box((0.10, 0.010, 0.025)), origin=Origin(xyz=(0.46, window_y, -0.07)), material=black, name="door_handle")
        model.articulation(
            f"body_to_{name}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=door,
            origin=Origin(xyz=(-0.28, y, 0.575)),
            axis=hinge_axis,
            motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.15),
        )
        return door

    make_side_door("left_door", 0.515, 0.017, (0.0, 0.0, 1.0))
    make_side_door("right_door", -0.515, -0.017, (0.0, 0.0, -1.0))

    liftgate = model.part("liftgate")
    liftgate.visual(Box((0.035, 0.82, 0.52)), origin=Origin(xyz=(0.0, 0.0, -0.26)), material=body_blue, name="liftgate_panel")
    liftgate.visual(Box((0.006, 0.62, 0.20)), origin=Origin(xyz=(0.019, 0.0, -0.15)), material=dark_glass, name="rear_window")
    liftgate.visual(Box((0.010, 0.18, 0.035)), origin=Origin(xyz=(0.019, 0.0, -0.35)), material=black, name="liftgate_handle")
    liftgate.visual(Box((0.008, 0.08, 0.10)), origin=Origin(xyz=(0.018, 0.34, -0.30)), material=red, name="left_tail_lamp")
    liftgate.visual(Box((0.008, 0.08, 0.10)), origin=Origin(xyz=(0.018, -0.34, -0.30)), material=red, name="right_tail_lamp")
    model.articulation(
        "body_to_liftgate",
        ArticulationType.REVOLUTE,
        parent=body,
        child=liftgate,
        origin=Origin(xyz=(1.0195, 0.0, 0.86)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.5, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    liftgate = object_model.get_part("liftgate")

    wheel_names = ("front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel")
    ctx.check(
        "four wheels have continuous axle joints",
        all(
            object_model.get_articulation(f"body_to_{name}").articulation_type == ArticulationType.CONTINUOUS
            for name in wheel_names
        ),
        details="Each small wheel should spin continuously on its axle.",
    )
    ctx.check(
        "doors and liftgate are hinged panels",
        object_model.get_articulation("body_to_left_door").articulation_type == ArticulationType.REVOLUTE
        and object_model.get_articulation("body_to_right_door").articulation_type == ArticulationType.REVOLUTE
        and object_model.get_articulation("body_to_liftgate").articulation_type == ArticulationType.REVOLUTE,
        details="The two side doors and rear liftgate should be revolute hinged parts.",
    )

    for wheel_name, axle_name in (
        ("front_left_wheel", "front_axle"),
        ("front_right_wheel", "front_axle"),
        ("rear_left_wheel", "rear_axle"),
        ("rear_right_wheel", "rear_axle"),
    ):
        wheel = object_model.get_part(wheel_name)
        ctx.allow_overlap(
            body,
            wheel,
            elem_a=axle_name,
            elem_b="rim",
            reason="The toy axle is intentionally captured inside the wheel hub so the wheel can spin on it.",
        )
        ctx.expect_overlap(
            body,
            wheel,
            axes="y",
            elem_a=axle_name,
            elem_b="rim",
            min_overlap=0.003,
            name=f"{wheel_name} hub remains on axle",
        )
        ctx.expect_overlap(
            body,
            wheel,
            axes="xz",
            elem_a=axle_name,
            elem_b="rim",
            min_overlap=0.040,
            name=f"{wheel_name} axle is centered in hub",
        )

    ctx.expect_gap(
        left_door,
        body,
        axis="y",
        positive_elem="door_panel",
        negative_elem="body_shell",
        min_gap=0.010,
        name="left door stands just outside body side",
    )
    ctx.expect_gap(
        body,
        right_door,
        axis="y",
        positive_elem="body_shell",
        negative_elem="door_panel",
        min_gap=0.010,
        name="right door stands just outside body side",
    )
    ctx.expect_gap(
        liftgate,
        body,
        axis="x",
        positive_elem="liftgate_panel",
        negative_elem="body_shell",
        min_gap=0.010,
        name="liftgate closes behind rear body opening",
    )
    ctx.expect_overlap(left_door, body, axes="xz", elem_a="door_panel", elem_b="body_shell", min_overlap=0.40, name="left door covers cabin side opening")
    ctx.expect_overlap(right_door, body, axes="xz", elem_a="door_panel", elem_b="body_shell", min_overlap=0.40, name="right door covers cabin side opening")

    left_rest = ctx.part_element_world_aabb(left_door, elem="door_panel")
    right_rest = ctx.part_element_world_aabb(right_door, elem="door_panel")
    lift_rest = ctx.part_element_world_aabb(liftgate, elem="liftgate_panel")
    with ctx.pose({"body_to_left_door": 0.90, "body_to_right_door": 0.90, "body_to_liftgate": 0.90}):
        left_open = ctx.part_element_world_aabb(left_door, elem="door_panel")
        right_open = ctx.part_element_world_aabb(right_door, elem="door_panel")
        lift_open = ctx.part_element_world_aabb(liftgate, elem="liftgate_panel")
    ctx.check(
        "side doors swing outward",
        left_rest is not None
        and right_rest is not None
        and left_open is not None
        and right_open is not None
        and left_open[1][1] > left_rest[1][1] + 0.20
        and right_open[0][1] < right_rest[0][1] - 0.20,
        details=f"left_rest={left_rest}, left_open={left_open}, right_rest={right_rest}, right_open={right_open}",
    )
    ctx.check(
        "liftgate swings upward and rearward",
        lift_rest is not None
        and lift_open is not None
        and lift_open[1][0] > lift_rest[1][0] + 0.15
        and lift_open[0][2] > lift_rest[0][2] + 0.10,
        details=f"lift_rest={lift_rest}, lift_open={lift_open}",
    )

    return ctx.report()


object_model = build_object_model()
