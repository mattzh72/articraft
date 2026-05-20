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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


FRAME_RAIL_X = 0.46
FRAME_RAIL_Y = 2.35
FRAME_RAIL_Z = 0.95
FRAME_RAIL_SIZE = (0.12, FRAME_RAIL_Y, 0.24)
FRAME_CROSSMEMBER_X = 0.80

AXLE_CENTER_Z = 0.34
FORWARD_AXLE_Y = 0.42
REAR_AXLE_Y = -0.42
AXLE_TRACK = 1.96
AXLE_HALF_TRACK = AXLE_TRACK * 0.5
AXLE_TUBE_RADIUS = 0.085

ROD_PIVOT_Z = 0.72
ROD_EYE_RADIUS = 0.038
ROD_EYE_LENGTH = 0.068
ROD_PLATE_THICK = 0.016
ROD_PLATE_OFFSET = ROD_EYE_LENGTH * 0.5 + ROD_PLATE_THICK * 0.5
FORWARD_ROD_PIVOT_Y = 0.86
REAR_ROD_PIVOT_Y = -0.86
AXLE_ROD_BRACKET_X = 0.46
AXLE_ROD_BRACKET_Z = 0.14


def _tube_mesh(name: str, points: list[tuple[float, float, float]], radius: float):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=8,
            radial_segments=18,
            cap_ends=True,
        ),
        name,
    )


def _add_hub(part, *, side_sign: float, steel, dark_steel) -> None:
    spin_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))
    drum_center_x = 0.07 * side_sign
    flange_center_x = 0.145 * side_sign
    cap_center_x = 0.185 * side_sign

    part.visual(
        Cylinder(radius=0.18, length=0.14),
        origin=Origin(xyz=(drum_center_x, 0.0, 0.0), rpy=spin_origin.rpy),
        material=dark_steel,
        name="hub_drum",
    )
    part.visual(
        Cylinder(radius=0.205, length=0.026),
        origin=Origin(xyz=(flange_center_x, 0.0, 0.0), rpy=spin_origin.rpy),
        material=steel,
        name="hub_flange",
    )
    part.visual(
        Cylinder(radius=0.085, length=0.058),
        origin=Origin(xyz=(cap_center_x, 0.0, 0.0), rpy=spin_origin.rpy),
        material=steel,
        name="hub_cap",
    )

    stud_x = 0.168 * side_sign
    for index, (y, z) in enumerate(
        [
            (0.110, 0.000),
            (0.055, 0.095),
            (-0.055, 0.095),
            (-0.110, 0.000),
            (-0.055, -0.095),
            (0.055, -0.095),
        ]
    ):
        part.visual(
            Cylinder(radius=0.010, length=0.040),
            origin=Origin(xyz=(stud_x, y, z), rpy=spin_origin.rpy),
            material=dark_steel,
            name=f"stud_{index + 1}",
        )


def _add_axle_beam(part, *, steel, dark_steel) -> None:
    spin_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))
    part.visual(
        Cylinder(radius=AXLE_TUBE_RADIUS, length=1.72),
        origin=spin_origin,
        material=dark_steel,
        name="beam_tube",
    )
    part.visual(
        Sphere(radius=0.19),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=dark_steel,
        name="carrier_center",
    )
    part.visual(
        Cylinder(radius=0.155, length=0.14),
        origin=Origin(xyz=(0.0, -0.11, 0.00), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="carrier_cover",
    )
    part.visual(
        Box((0.40, 0.22, 0.11)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=dark_steel,
        name="center_web",
    )

    for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
        spindle_center_x = side_sign * 0.92
        bridge_x = side_sign * AXLE_ROD_BRACKET_X
        if side_name == "left":
            spindle_name = "left_spindle"
            backing_name = "left_brake_backing"
            bridge_name = "left_rod_bridge"
            inner_plate_name = "left_axle_plate_inner"
            outer_plate_name = "left_axle_plate_outer"
        else:
            spindle_name = "right_spindle"
            backing_name = "right_brake_backing"
            bridge_name = "right_rod_bridge"
            inner_plate_name = "right_axle_plate_inner"
            outer_plate_name = "right_axle_plate_outer"

        part.visual(
            Cylinder(radius=0.055, length=0.12),
            origin=Origin(xyz=(spindle_center_x, 0.0, 0.0), rpy=spin_origin.rpy),
            material=steel,
            name=spindle_name,
        )
        part.visual(
            Cylinder(radius=0.125, length=0.034),
            origin=Origin(xyz=(side_sign * 0.84, 0.0, 0.0), rpy=spin_origin.rpy),
            material=steel,
            name=backing_name,
        )
        part.visual(
            Box((0.100, 0.085, 0.052)),
            origin=Origin(xyz=(bridge_x, 0.0, 0.045)),
            material=steel,
            name=bridge_name,
        )
        for plate_name, x_offset in (
            (inner_plate_name, -ROD_PLATE_OFFSET),
            (outer_plate_name, ROD_PLATE_OFFSET),
        ):
            part.visual(
                Box((ROD_PLATE_THICK, 0.070, 0.120)),
                origin=Origin(
                    xyz=(bridge_x + side_sign * x_offset, 0.0, 0.140),
                ),
                material=steel,
                name=plate_name,
            )


def _add_frame_bracket(part, *, x_center: float, y_center: float, prefix: str, steel) -> None:
    for plate_name, x_offset in (("inner", -ROD_PLATE_OFFSET), ("outer", ROD_PLATE_OFFSET)):
        part.visual(
            Box((ROD_PLATE_THICK, 0.085, 0.220)),
            origin=Origin(xyz=(x_center + x_offset, y_center, 0.720)),
            material=steel,
            name=f"{prefix}_{plate_name}_plate",
        )


def _add_torque_rod(part, *, end_local: tuple[float, float, float], rod_name: str, steel, dark_steel) -> None:
    spin_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))
    part.visual(
        Cylinder(radius=ROD_EYE_RADIUS, length=ROD_EYE_LENGTH),
        origin=spin_origin,
        material=steel,
        name="chassis_eye",
    )
    part.visual(
        _tube_mesh(
            f"{rod_name}_shaft",
            [
                (0.0, 0.0, 0.0),
                (0.0, end_local[1] * 0.52, end_local[2] * 0.42),
                end_local,
            ],
            radius=0.019,
        ),
        material=dark_steel,
        name="rod_shaft",
    )
    part.visual(
        Cylinder(radius=ROD_EYE_RADIUS, length=ROD_EYE_LENGTH),
        origin=Origin(xyz=end_local, rpy=spin_origin.rpy),
        material=steel,
        name="axle_eye",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_truck_tandem_rear_axle")

    frame_steel = model.material("frame_steel", rgba=(0.20, 0.21, 0.23, 1.0))
    cast_steel = model.material("cast_steel", rgba=(0.28, 0.29, 0.30, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.60, 0.62, 0.65, 1.0))
    hub_steel = model.material("hub_steel", rgba=(0.70, 0.71, 0.73, 1.0))

    chassis = model.part("chassis_frame")
    chassis.inertial = Inertial.from_geometry(
        Box((1.10, 2.45, 0.40)),
        mass=820.0,
        origin=Origin(xyz=(0.0, 0.0, FRAME_RAIL_Z)),
    )
    chassis.visual(
        Box(FRAME_RAIL_SIZE),
        origin=Origin(xyz=(FRAME_RAIL_X, 0.0, FRAME_RAIL_Z)),
        material=frame_steel,
        name="left_rail",
    )
    chassis.visual(
        Box(FRAME_RAIL_SIZE),
        origin=Origin(xyz=(-FRAME_RAIL_X, 0.0, FRAME_RAIL_Z)),
        material=frame_steel,
        name="right_rail",
    )
    for cross_name, y_center in (
        ("front_crossmember", 0.98),
        ("center_crossmember", 0.00),
        ("rear_crossmember", -0.98),
    ):
        chassis.visual(
            Box((FRAME_CROSSMEMBER_X, 0.10, 0.12)),
            origin=Origin(xyz=(0.0, y_center, FRAME_RAIL_Z)),
            material=frame_steel,
            name=cross_name,
        )
    for prefix, x_center, y_center in (
        ("forward_left_bracket", FRAME_RAIL_X, FORWARD_ROD_PIVOT_Y),
        ("forward_right_bracket", -FRAME_RAIL_X, FORWARD_ROD_PIVOT_Y),
        ("rear_left_bracket", FRAME_RAIL_X, REAR_ROD_PIVOT_Y),
        ("rear_right_bracket", -FRAME_RAIL_X, REAR_ROD_PIVOT_Y),
    ):
        _add_frame_bracket(
            chassis,
            x_center=x_center,
            y_center=y_center,
            prefix=prefix,
            steel=machined_steel,
        )

    forward_axle = model.part("forward_axle_beam")
    forward_axle.inertial = Inertial.from_geometry(
        Box((2.00, 0.45, 0.42)),
        mass=430.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    _add_axle_beam(
        forward_axle,
        steel=machined_steel,
        dark_steel=cast_steel,
    )

    rear_axle = model.part("rear_axle_beam")
    rear_axle.inertial = Inertial.from_geometry(
        Box((2.00, 0.45, 0.42)),
        mass=445.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    _add_axle_beam(
        rear_axle,
        steel=machined_steel,
        dark_steel=cast_steel,
    )
    rear_axle.visual(
        Cylinder(radius=0.11, length=0.34),
        origin=Origin(xyz=(0.0, 0.16, 0.02), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hub_steel,
        name="inter_axle_input_housing",
    )

    forward_left_hub = model.part("forward_left_hub")
    forward_left_hub.inertial = Inertial.from_geometry(
        Cylinder(radius=0.205, length=0.22),
        mass=75.0,
        origin=Origin(xyz=(0.11, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_hub(forward_left_hub, side_sign=1.0, steel=hub_steel, dark_steel=cast_steel)

    forward_right_hub = model.part("forward_right_hub")
    forward_right_hub.inertial = Inertial.from_geometry(
        Cylinder(radius=0.205, length=0.22),
        mass=75.0,
        origin=Origin(xyz=(-0.11, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_hub(forward_right_hub, side_sign=-1.0, steel=hub_steel, dark_steel=cast_steel)

    rear_left_hub = model.part("rear_left_hub")
    rear_left_hub.inertial = Inertial.from_geometry(
        Cylinder(radius=0.205, length=0.22),
        mass=75.0,
        origin=Origin(xyz=(0.11, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_hub(rear_left_hub, side_sign=1.0, steel=hub_steel, dark_steel=cast_steel)

    rear_right_hub = model.part("rear_right_hub")
    rear_right_hub.inertial = Inertial.from_geometry(
        Cylinder(radius=0.205, length=0.22),
        mass=75.0,
        origin=Origin(xyz=(-0.11, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_hub(rear_right_hub, side_sign=-1.0, steel=hub_steel, dark_steel=cast_steel)

    rod_specs = [
        (
            "forward_left_torque_rod",
            (FRAME_RAIL_X, FORWARD_ROD_PIVOT_Y, ROD_PIVOT_Z),
            (0.0, FORWARD_AXLE_Y - FORWARD_ROD_PIVOT_Y, AXLE_ROD_BRACKET_Z - (ROD_PIVOT_Z - AXLE_CENTER_Z)),
        ),
        (
            "forward_right_torque_rod",
            (-FRAME_RAIL_X, FORWARD_ROD_PIVOT_Y, ROD_PIVOT_Z),
            (0.0, FORWARD_AXLE_Y - FORWARD_ROD_PIVOT_Y, AXLE_ROD_BRACKET_Z - (ROD_PIVOT_Z - AXLE_CENTER_Z)),
        ),
        (
            "rear_left_torque_rod",
            (FRAME_RAIL_X, REAR_ROD_PIVOT_Y, ROD_PIVOT_Z),
            (0.0, REAR_AXLE_Y - REAR_ROD_PIVOT_Y, AXLE_ROD_BRACKET_Z - (ROD_PIVOT_Z - AXLE_CENTER_Z)),
        ),
        (
            "rear_right_torque_rod",
            (-FRAME_RAIL_X, REAR_ROD_PIVOT_Y, ROD_PIVOT_Z),
            (0.0, REAR_AXLE_Y - REAR_ROD_PIVOT_Y, AXLE_ROD_BRACKET_Z - (ROD_PIVOT_Z - AXLE_CENTER_Z)),
        ),
    ]
    for rod_name, _, end_local in rod_specs:
        rod = model.part(rod_name)
        rod.inertial = Inertial.from_geometry(
            Box((0.10, 0.55, 0.16)),
            mass=24.0,
            origin=Origin(xyz=(0.0, end_local[1] * 0.5, end_local[2] * 0.5)),
        )
        _add_torque_rod(
            rod,
            end_local=end_local,
            rod_name=rod_name,
            steel=hub_steel,
            dark_steel=cast_steel,
        )

    model.articulation(
        "chassis_to_forward_axle",
        ArticulationType.FIXED,
        parent=chassis,
        child=forward_axle,
        origin=Origin(xyz=(0.0, FORWARD_AXLE_Y, AXLE_CENTER_Z)),
    )
    model.articulation(
        "chassis_to_rear_axle",
        ArticulationType.FIXED,
        parent=chassis,
        child=rear_axle,
        origin=Origin(xyz=(0.0, REAR_AXLE_Y, AXLE_CENTER_Z)),
    )

    for rod_name, pivot_origin, _ in rod_specs:
        model.articulation(
            f"chassis_to_{rod_name}",
            ArticulationType.REVOLUTE,
            parent=chassis,
            child=rod_name,
            origin=Origin(xyz=pivot_origin),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=90.0,
                velocity=1.2,
                lower=-0.22,
                upper=0.22,
            ),
        )

    for axle_name, hub_name, x_origin in (
        ("forward_axle_beam", "forward_left_hub", AXLE_HALF_TRACK),
        ("forward_axle_beam", "forward_right_hub", -AXLE_HALF_TRACK),
        ("rear_axle_beam", "rear_left_hub", AXLE_HALF_TRACK),
        ("rear_axle_beam", "rear_right_hub", -AXLE_HALF_TRACK),
    ):
        side = "left" if x_origin > 0.0 else "right"
        model.articulation(
            f"{axle_name}_to_{side}_hub",
            ArticulationType.CONTINUOUS,
            parent=axle_name,
            child=hub_name,
            origin=Origin(xyz=(x_origin, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=220.0, velocity=30.0),
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

    chassis = object_model.get_part("chassis_frame")
    forward_axle = object_model.get_part("forward_axle_beam")
    rear_axle = object_model.get_part("rear_axle_beam")
    forward_left_hub = object_model.get_part("forward_left_hub")
    forward_right_hub = object_model.get_part("forward_right_hub")
    rear_left_hub = object_model.get_part("rear_left_hub")
    rear_right_hub = object_model.get_part("rear_right_hub")
    forward_left_rod = object_model.get_part("forward_left_torque_rod")
    forward_right_rod = object_model.get_part("forward_right_torque_rod")
    rear_left_rod = object_model.get_part("rear_left_torque_rod")
    rear_right_rod = object_model.get_part("rear_right_torque_rod")

    wheel_joints = [
        object_model.get_articulation("forward_axle_beam_to_left_hub"),
        object_model.get_articulation("forward_axle_beam_to_right_hub"),
        object_model.get_articulation("rear_axle_beam_to_left_hub"),
        object_model.get_articulation("rear_axle_beam_to_right_hub"),
    ]
    rod_joints = [
        object_model.get_articulation("chassis_to_forward_left_torque_rod"),
        object_model.get_articulation("chassis_to_forward_right_torque_rod"),
        object_model.get_articulation("chassis_to_rear_left_torque_rod"),
        object_model.get_articulation("chassis_to_rear_right_torque_rod"),
    ]

    ctx.check(
        "all prompt parts exist",
        all(
            part is not None
            for part in (
                chassis,
                forward_axle,
                rear_axle,
                forward_left_hub,
                forward_right_hub,
                rear_left_hub,
                rear_right_hub,
                forward_left_rod,
                forward_right_rod,
                rear_left_rod,
                rear_right_rod,
            )
        ),
        details="Expected chassis, two axles, four torque rods, and four hubs.",
    )

    ctx.check(
        "wheel hubs use continuous x-axis rotation",
        all(
            joint.axis == (1.0, 0.0, 0.0)
            and joint.motion_limits is not None
            and joint.motion_limits.lower is None
            and joint.motion_limits.upper is None
            for joint in wheel_joints
        ),
        details=str([(joint.name, joint.axis, joint.motion_limits) for joint in wheel_joints]),
    )
    ctx.check(
        "torque rods pivot at chassis brackets about x",
        all(
            joint.axis == (1.0, 0.0, 0.0)
            and joint.motion_limits is not None
            and joint.motion_limits.lower is not None
            and joint.motion_limits.upper is not None
            for joint in rod_joints
        ),
        details=str([(joint.name, joint.axis, joint.motion_limits) for joint in rod_joints]),
    )

    ctx.expect_contact(
        forward_axle,
        forward_left_hub,
        elem_a="left_spindle",
        elem_b="hub_drum",
        name="forward left hub seats on forward axle spindle",
    )
    ctx.expect_contact(
        forward_axle,
        forward_right_hub,
        elem_a="right_spindle",
        elem_b="hub_drum",
        name="forward right hub seats on forward axle spindle",
    )
    ctx.expect_contact(
        rear_axle,
        rear_left_hub,
        elem_a="left_spindle",
        elem_b="hub_drum",
        name="rear left hub seats on rear axle spindle",
    )
    ctx.expect_contact(
        rear_axle,
        rear_right_hub,
        elem_a="right_spindle",
        elem_b="hub_drum",
        name="rear right hub seats on rear axle spindle",
    )

    for rod, axle, check_name in (
        (forward_left_rod, chassis, "forward left rod touches chassis bracket"),
        (forward_right_rod, chassis, "forward right rod touches chassis bracket"),
        (rear_left_rod, chassis, "rear left rod touches chassis bracket"),
        (rear_right_rod, chassis, "rear right rod touches chassis bracket"),
    ):
        ctx.expect_contact(rod, axle, name=check_name)

    for rod, axle, check_name in (
        (forward_left_rod, forward_axle, "forward left rod reaches forward axle"),
        (forward_right_rod, forward_axle, "forward right rod reaches forward axle"),
        (rear_left_rod, rear_axle, "rear left rod reaches rear axle"),
        (rear_right_rod, rear_axle, "rear right rod reaches rear axle"),
    ):
        ctx.expect_contact(rod, axle, name=check_name)

    rest_aabb = ctx.part_element_world_aabb(forward_left_rod, elem="axle_eye")
    with ctx.pose(chassis_to_forward_left_torque_rod=0.18):
        posed_aabb = ctx.part_element_world_aabb(forward_left_rod, elem="axle_eye")
    ctx.check(
        "forward left torque rod pitches downward at positive angle",
        rest_aabb is not None
        and posed_aabb is not None
        and posed_aabb[0][2] < rest_aabb[0][2] - 0.02,
        details=f"rest={rest_aabb}, posed={posed_aabb}",
    )

    ctx.expect_gap(
        chassis,
        forward_axle,
        axis="z",
        positive_elem="left_rail",
        negative_elem="beam_tube",
        min_gap=0.30,
        name="chassis sits well above forward axle",
    )
    ctx.expect_gap(
        chassis,
        rear_axle,
        axis="z",
        positive_elem="left_rail",
        negative_elem="beam_tube",
        min_gap=0.30,
        name="chassis sits well above rear axle",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
