from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_vacuum_with_articulated_wand", assets=ASSETS)

    body_paint = model.material("body_paint", rgba=(0.73, 0.30, 0.10, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.14, 0.15, 0.16, 1.0))
    nozzle_polymer = model.material("nozzle_polymer", rgba=(0.23, 0.24, 0.25, 1.0))
    steel = model.material("steel", rgba=(0.55, 0.57, 0.60, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.30, 0.31, 0.33, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    fastener = model.material("fastener", rgba=(0.76, 0.78, 0.80, 1.0))

    x_axis = Origin(rpy=(0.0, pi / 2.0, 0.0))
    y_axis = Origin(rpy=(pi / 2.0, 0.0, 0.0))

    def x_cyl(
        part,
        radius: float,
        length: float,
        *,
        xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
        material=None,
        name: str | None = None,
    ) -> None:
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=x_axis.rpy),
            material=material,
            name=name,
        )

    def y_cyl(
        part,
        radius: float,
        length: float,
        *,
        xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
        rpy: tuple[float, float, float] | None = None,
        material=None,
        name: str | None = None,
    ) -> None:
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=y_axis.rpy if rpy is None else rpy),
            material=material,
            name=name,
        )

    def _save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(name))

    def rounded_loop(
        width: float,
        depth: float,
        radius: float,
        *,
        z: float,
        center_xy: tuple[float, float] = (0.0, 0.0),
    ) -> list[tuple[float, float, float]]:
        cx, cy = center_xy
        return [(x + cx, y + cy, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8)]

    def add_x_fasteners(part, xs: tuple[float, ...], y: float, z: float, *, prefix: str) -> None:
        for index, x in enumerate(xs):
            x_cyl(
                part,
                0.006,
                0.004,
                xyz=(x, y, z),
                material=fastener,
                name=f"{prefix}_{index}",
            )

    def add_wheel_visuals(part, *, side_sign: float, radius: float, width: float) -> None:
        x_cyl(part, radius, width, material=rubber, name="tire")
        x_cyl(
            part,
            radius * 0.68,
            width * 0.44,
            xyz=(0.0, 0.0, 0.0),
            material=dark_plastic,
            name="rim",
        )
        x_cyl(
            part,
            radius * 0.40,
            0.006,
            xyz=(-side_sign * (width * 0.35), 0.0, 0.0),
            material=steel,
            name="inner_hub",
        )
        x_cyl(
            part,
            radius * 0.20,
            0.010,
            xyz=(side_sign * (width * 0.36), 0.0, 0.0),
            material=fastener,
            name="outer_cap",
        )
        for fastener_index, angle_scale in enumerate((-0.028, 0.0, 0.028)):
            x_cyl(
                part,
                0.0048,
                0.004,
                xyz=(side_sign * (width * 0.37), angle_scale, radius * 0.18),
                material=fastener,
                name=f"outer_fastener_{fastener_index}",
            )

    def add_roller_visuals(part) -> None:
        x_cyl(part, 0.015, 0.020, material=rubber, name="roller_tire")
        x_cyl(part, 0.010, 0.014, material=dark_plastic, name="roller_core")
        x_cyl(part, 0.006, 0.006, xyz=(0.0, 0.0, 0.0), material=fastener, name="roller_cap")

    main_body = model.part("main_body")
    main_body.inertial = Inertial.from_geometry(
        Box((0.40, 0.50, 0.30)),
        mass=12.5,
        origin=Origin(xyz=(0.0, 0.02, 0.15)),
    )
    main_body.visual(
        _save_mesh(
            "utility_vacuum_lower_shell.obj",
            LoftGeometry(
                [
                    rounded_loop(0.34, 0.42, 0.038, z=0.0, center_xy=(0.0, 0.010)),
                    rounded_loop(0.32, 0.38, 0.034, z=0.070, center_xy=(0.0, 0.000)),
                    rounded_loop(0.28, 0.28, 0.028, z=0.120, center_xy=(0.0, -0.020)),
                ],
                cap=True,
                closed=True,
            ),
        ),
        material=body_paint,
        name="lower_shell",
    )
    main_body.visual(
        _save_mesh(
            "utility_vacuum_upper_shell.obj",
            LoftGeometry(
                [
                    rounded_loop(0.24, 0.20, 0.030, z=0.120, center_xy=(0.0, -0.030)),
                    rounded_loop(0.20, 0.16, 0.024, z=0.205, center_xy=(0.0, -0.040)),
                    rounded_loop(0.16, 0.11, 0.018, z=0.255, center_xy=(0.0, -0.035)),
                ],
                cap=True,
                closed=True,
            ),
        ),
        material=body_paint,
        name="upper_shell",
    )
    main_body.visual(
        Cylinder(radius=0.075, length=0.06),
        origin=Origin(xyz=(0.0, -0.03, 0.27)),
        material=dark_plastic,
        name="motor_cap",
    )
    main_body.visual(
        Box((0.28, 0.06, 0.08)),
        origin=Origin(xyz=(0.0, 0.23, 0.045)),
        material=dark_plastic,
        name="front_bumper",
    )
    y_cyl(
        main_body,
        0.024,
        0.048,
        xyz=(0.0, 0.228, 0.088),
        material=dark_steel,
        name="inlet_tube",
    )
    main_body.visual(
        Box((0.012, 0.30, 0.10)),
        origin=Origin(xyz=(0.154, 0.00, 0.08)),
        material=dark_plastic,
        name="left_side_guard",
    )
    main_body.visual(
        Box((0.012, 0.30, 0.10)),
        origin=Origin(xyz=(-0.154, 0.00, 0.08)),
        material=dark_plastic,
        name="right_side_guard",
    )
    x_cyl(main_body, 0.012, 0.18, xyz=(0.0, -0.16, 0.30), material=dark_steel, name="carry_handle")
    main_body.visual(
        Cylinder(radius=0.012, length=0.11),
        origin=Origin(xyz=(0.08, -0.16, 0.245)),
        material=dark_steel,
        name="left_handle_post",
    )
    main_body.visual(
        Cylinder(radius=0.012, length=0.11),
        origin=Origin(xyz=(-0.08, -0.16, 0.245)),
        material=dark_steel,
        name="right_handle_post",
    )
    main_body.visual(
        Box((0.07, 0.05, 0.03)),
        origin=Origin(xyz=(0.0, -0.03, 0.315)),
        material=dark_plastic,
        name="switch_housing",
    )
    main_body.visual(
        Box((0.09, 0.03, 0.008)),
        origin=Origin(xyz=(0.0, -0.03, 0.334)),
        material=steel,
        name="switch_plate",
    )
    main_body.visual(
        Box((0.05, 0.05, 0.02)),
        origin=Origin(xyz=(0.0, 0.105, 0.130)),
        material=dark_plastic,
        name="wand_reinforcement_block",
    )
    main_body.visual(
        Box((0.21, 0.09, 0.05)),
        origin=Origin(xyz=(0.0, -0.145, 0.215)),
        material=dark_plastic,
        name="rear_handle_bridge",
    )
    main_body.visual(
        Box((0.012, 0.042, 0.060)),
        origin=Origin(xyz=(0.037, 0.158, 0.145)),
        material=dark_steel,
        name="left_wand_pivot_ear",
    )
    main_body.visual(
        Box((0.012, 0.042, 0.060)),
        origin=Origin(xyz=(-0.037, 0.158, 0.145)),
        material=dark_steel,
        name="right_wand_pivot_ear",
    )
    main_body.visual(
        Box((0.020, 0.034, 0.034)),
        origin=Origin(xyz=(0.050, 0.132, 0.128)),
        material=dark_steel,
        name="left_wand_pivot_base",
    )
    main_body.visual(
        Box((0.020, 0.034, 0.034)),
        origin=Origin(xyz=(-0.050, 0.132, 0.128)),
        material=dark_steel,
        name="right_wand_pivot_base",
    )
    x_cyl(
        main_body,
        0.024,
        0.006,
        xyz=(0.031, 0.158, 0.145),
        material=steel,
        name="left_wand_pivot_washer",
    )
    x_cyl(
        main_body,
        0.024,
        0.006,
        xyz=(-0.031, 0.158, 0.145),
        material=steel,
        name="right_wand_pivot_washer",
    )
    x_cyl(
        main_body,
        0.030,
        0.008,
        xyz=(0.166, -0.08, 0.07),
        material=dark_steel,
        name="left_axle_boss",
    )
    x_cyl(
        main_body,
        0.030,
        0.008,
        xyz=(-0.166, -0.08, 0.07),
        material=dark_steel,
        name="right_axle_boss",
    )
    main_body.visual(
        Box((0.09, 0.05, 0.015)),
        origin=Origin(xyz=(0.0, 0.18, 0.015)),
        material=dark_plastic,
        name="front_skid",
    )
    main_body.visual(
        Box((0.16, 0.025, 0.010)),
        origin=Origin(xyz=(0.0, 0.20, 0.105)),
        material=steel,
        name="front_guard_rail",
    )
    add_x_fasteners(main_body, (0.102, -0.102), 0.045, 0.12, prefix="shell_fastener")
    add_x_fasteners(main_body, (0.037, -0.037), 0.158, 0.175, prefix="pivot_fastener")

    left_rear_wheel = model.part("left_rear_wheel")
    left_rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.070, length=0.040),
        mass=1.3,
        origin=Origin(rpy=x_axis.rpy),
    )
    add_wheel_visuals(left_rear_wheel, side_sign=1.0, radius=0.070, width=0.040)

    right_rear_wheel = model.part("right_rear_wheel")
    right_rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.070, length=0.040),
        mass=1.3,
        origin=Origin(rpy=x_axis.rpy),
    )
    add_wheel_visuals(right_rear_wheel, side_sign=-1.0, radius=0.070, width=0.040)

    upper_wand = model.part("upper_wand")
    upper_wand.inertial = Inertial.from_geometry(
        Box((0.10, 0.42, 0.10)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.18, 0.05)),
    )
    x_cyl(upper_wand, 0.022, 0.056, material=steel, name="rear_pivot_barrel")
    upper_wand.visual(
        Box((0.050, 0.026, 0.046)),
        origin=Origin(xyz=(0.0, 0.025, 0.008)),
        material=dark_steel,
        name="rear_hinge_block",
    )
    y_cyl(
        upper_wand,
        0.019,
        0.274,
        xyz=(0.0, 0.172, 0.048),
        rpy=(-1.30, 0.0, 0.0),
        material=steel,
        name="upper_tube",
    )
    y_cyl(
        upper_wand,
        0.027,
        0.100,
        xyz=(0.0, 0.096, 0.031),
        rpy=(-1.30, 0.0, 0.0),
        material=rubber,
        name="handle_grip",
    )
    upper_wand.visual(
        Box((0.038, 0.055, 0.035)),
        origin=Origin(xyz=(0.0, 0.100, 0.060)),
        material=dark_plastic,
        name="trigger_block",
    )
    upper_wand.visual(
        Box((0.062, 0.024, 0.030)),
        origin=Origin(xyz=(0.0, 0.115, 0.089)),
        material=dark_plastic,
        name="guard_bridge",
    )
    x_cyl(upper_wand, 0.010, 0.100, xyz=(0.0, 0.124, 0.095), material=dark_steel, name="top_guard")
    upper_wand.visual(
        Box((0.010, 0.022, 0.040)),
        origin=Origin(xyz=(0.030, 0.115, 0.074)),
        material=dark_steel,
        name="left_guard_post",
    )
    upper_wand.visual(
        Box((0.010, 0.022, 0.040)),
        origin=Origin(xyz=(-0.030, 0.115, 0.074)),
        material=dark_steel,
        name="right_guard_post",
    )
    upper_wand.visual(
        Box((0.012, 0.050, 0.054)),
        origin=Origin(xyz=(0.037, 0.335, 0.096)),
        material=dark_steel,
        name="left_elbow_ear",
    )
    upper_wand.visual(
        Box((0.012, 0.050, 0.054)),
        origin=Origin(xyz=(-0.037, 0.335, 0.096)),
        material=dark_steel,
        name="right_elbow_ear",
    )
    x_cyl(
        upper_wand,
        0.024,
        0.006,
        xyz=(0.031, 0.347, 0.096),
        material=steel,
        name="left_elbow_washer",
    )
    x_cyl(
        upper_wand,
        0.024,
        0.006,
        xyz=(-0.031, 0.347, 0.096),
        material=steel,
        name="right_elbow_washer",
    )
    upper_wand.visual(
        Box((0.062, 0.030, 0.044)),
        origin=Origin(xyz=(0.0, 0.296, 0.094)),
        material=dark_steel,
        name="front_joint_sleeve",
    )
    add_x_fasteners(upper_wand, (0.028, -0.028), 0.000, 0.000, prefix="rear_joint_fastener")
    add_x_fasteners(upper_wand, (0.037, -0.037), 0.347, 0.096, prefix="front_joint_fastener")

    lower_wand = model.part("lower_wand")
    lower_wand.inertial = Inertial.from_geometry(
        Box((0.10, 0.38, 0.12)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.15, -0.06)),
    )
    x_cyl(lower_wand, 0.021, 0.056, material=steel, name="rear_pivot_barrel")
    lower_wand.visual(
        Box((0.048, 0.026, 0.046)),
        origin=Origin(xyz=(0.0, 0.022, -0.004)),
        material=dark_steel,
        name="rear_hinge_block",
    )
    y_cyl(
        lower_wand,
        0.024,
        0.110,
        xyz=(0.0, 0.049, -0.025),
        rpy=(-2.05, 0.0, 0.0),
        material=dark_plastic,
        name="reinforcement_sleeve",
    )
    y_cyl(
        lower_wand,
        0.018,
        0.270,
        xyz=(0.0, 0.151, -0.079),
        rpy=(-2.05, 0.0, 0.0),
        material=steel,
        name="lower_tube",
    )
    lower_wand.visual(
        Box((0.036, 0.032, 0.032)),
        origin=Origin(xyz=(0.0, 0.120, -0.050)),
        material=dark_plastic,
        name="service_collar",
    )
    lower_wand.visual(
        Box((0.012, 0.050, 0.052)),
        origin=Origin(xyz=(0.038, 0.286, -0.155)),
        material=dark_steel,
        name="left_nozzle_ear",
    )
    lower_wand.visual(
        Box((0.012, 0.050, 0.052)),
        origin=Origin(xyz=(-0.038, 0.286, -0.155)),
        material=dark_steel,
        name="right_nozzle_ear",
    )
    x_cyl(
        lower_wand,
        0.024,
        0.006,
        xyz=(0.031, 0.302, -0.157),
        material=steel,
        name="left_nozzle_washer",
    )
    x_cyl(
        lower_wand,
        0.024,
        0.006,
        xyz=(-0.031, 0.302, -0.157),
        material=steel,
        name="right_nozzle_washer",
    )
    lower_wand.visual(
        Box((0.064, 0.032, 0.046)),
        origin=Origin(xyz=(0.0, 0.255, -0.140)),
        material=dark_steel,
        name="front_joint_sleeve",
    )
    add_x_fasteners(lower_wand, (0.028, -0.028), 0.000, 0.000, prefix="rear_joint_fastener")
    add_x_fasteners(lower_wand, (0.038, -0.038), 0.302, -0.157, prefix="front_joint_fastener")

    floor_nozzle = model.part("floor_nozzle")
    floor_nozzle.inertial = Inertial.from_geometry(
        Box((0.34, 0.13, 0.09)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.06, -0.05)),
    )
    x_cyl(floor_nozzle, 0.024, 0.056, material=steel, name="pivot_barrel")
    floor_nozzle.visual(
        Box((0.060, 0.040, 0.070)),
        origin=Origin(xyz=(0.0, 0.020, -0.040)),
        material=dark_steel,
        name="neck_block",
    )
    floor_nozzle.visual(
        Box((0.012, 0.070, 0.060)),
        origin=Origin(xyz=(0.034, 0.045, -0.055)),
        material=dark_steel,
        name="left_neck_brace",
    )
    floor_nozzle.visual(
        Box((0.012, 0.070, 0.060)),
        origin=Origin(xyz=(-0.034, 0.045, -0.055)),
        material=dark_steel,
        name="right_neck_brace",
    )
    floor_nozzle.visual(
        _save_mesh(
            "utility_vacuum_nozzle_cover.obj",
            LoftGeometry(
                [
                    rounded_loop(0.040, 0.026, 0.010, z=-0.010, center_xy=(0.0, 0.044)),
                    rounded_loop(0.200, 0.086, 0.016, z=-0.024, center_xy=(0.0, 0.056)),
                    rounded_loop(0.320, 0.118, 0.012, z=-0.042, center_xy=(0.0, 0.065)),
                ],
                cap=True,
                closed=True,
            ),
        ),
        material=nozzle_polymer,
        name="top_cover",
    )
    floor_nozzle.visual(
        Box((0.026, 0.118, 0.044)),
        origin=Origin(xyz=(0.147, 0.065, -0.062)),
        material=nozzle_polymer,
        name="left_side_skirt",
    )
    floor_nozzle.visual(
        Box((0.026, 0.118, 0.044)),
        origin=Origin(xyz=(-0.147, 0.065, -0.062)),
        material=nozzle_polymer,
        name="right_side_skirt",
    )
    floor_nozzle.visual(
        Box((0.286, 0.012, 0.032)),
        origin=Origin(xyz=(0.0, 0.118, -0.060)),
        material=dark_plastic,
        name="front_bumper_strip",
    )
    floor_nozzle.visual(
        Box((0.060, 0.014, 0.044)),
        origin=Origin(xyz=(0.075, 0.014, -0.064)),
        material=nozzle_polymer,
        name="rear_right_skirt",
    )
    floor_nozzle.visual(
        Box((0.060, 0.014, 0.044)),
        origin=Origin(xyz=(-0.075, 0.014, -0.064)),
        material=nozzle_polymer,
        name="rear_left_skirt",
    )
    floor_nozzle.visual(
        Box((0.250, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, 0.032, -0.079)),
        material=rubber,
        name="intake_lip",
    )
    floor_nozzle.visual(
        Box((0.150, 0.018, 0.015)),
        origin=Origin(xyz=(0.0, 0.055, -0.028)),
        material=dark_plastic,
        name="top_rib",
    )
    floor_nozzle.visual(
        Box((0.012, 0.024, 0.030)),
        origin=Origin(xyz=(0.104, 0.012, -0.064)),
        material=dark_steel,
        name="left_roller_inboard_fork",
    )
    floor_nozzle.visual(
        Box((0.008, 0.024, 0.030)),
        origin=Origin(xyz=(0.138, 0.012, -0.064)),
        material=dark_steel,
        name="left_roller_outboard_fork",
    )
    x_cyl(
        floor_nozzle,
        0.011,
        0.008,
        xyz=(0.109, 0.012, -0.069),
        material=steel,
        name="left_roller_inboard_washer",
    )
    x_cyl(
        floor_nozzle,
        0.011,
        0.008,
        xyz=(0.130, 0.012, -0.069),
        material=steel,
        name="left_roller_outboard_washer",
    )
    floor_nozzle.visual(
        Box((0.012, 0.024, 0.030)),
        origin=Origin(xyz=(-0.104, 0.012, -0.064)),
        material=dark_steel,
        name="right_roller_inboard_fork",
    )
    floor_nozzle.visual(
        Box((0.008, 0.024, 0.030)),
        origin=Origin(xyz=(-0.138, 0.012, -0.064)),
        material=dark_steel,
        name="right_roller_outboard_fork",
    )
    x_cyl(
        floor_nozzle,
        0.011,
        0.008,
        xyz=(-0.109, 0.012, -0.069),
        material=steel,
        name="right_roller_inboard_washer",
    )
    x_cyl(
        floor_nozzle,
        0.011,
        0.008,
        xyz=(-0.130, 0.012, -0.069),
        material=steel,
        name="right_roller_outboard_washer",
    )
    add_x_fasteners(floor_nozzle, (0.120, -0.120, 0.0), 0.060, -0.038, prefix="cover_fastener")

    left_nozzle_roller = model.part("left_nozzle_roller")
    left_nozzle_roller.inertial = Inertial.from_geometry(
        Cylinder(radius=0.015, length=0.020),
        mass=0.08,
        origin=Origin(rpy=x_axis.rpy),
    )
    add_roller_visuals(left_nozzle_roller)

    right_nozzle_roller = model.part("right_nozzle_roller")
    right_nozzle_roller.inertial = Inertial.from_geometry(
        Cylinder(radius=0.015, length=0.020),
        mass=0.08,
        origin=Origin(rpy=x_axis.rpy),
    )
    add_roller_visuals(right_nozzle_roller)

    model.articulation(
        "left_rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=main_body,
        child=left_rear_wheel,
        origin=Origin(xyz=(0.187, -0.08, 0.07)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=22.0),
    )
    model.articulation(
        "right_rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=main_body,
        child=right_rear_wheel,
        origin=Origin(xyz=(-0.187, -0.08, 0.07)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=22.0),
    )
    model.articulation(
        "body_to_upper_wand",
        ArticulationType.REVOLUTE,
        parent=main_body,
        child=upper_wand,
        origin=Origin(xyz=(0.0, 0.158, 0.145)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=1.8, lower=-0.20, upper=0.95),
    )
    model.articulation(
        "upper_to_lower_wand",
        ArticulationType.REVOLUTE,
        parent=upper_wand,
        child=lower_wand,
        origin=Origin(xyz=(0.0, 0.347, 0.096)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=-0.35, upper=0.95),
    )
    model.articulation(
        "lower_to_floor_nozzle",
        ArticulationType.REVOLUTE,
        parent=lower_wand,
        child=floor_nozzle,
        origin=Origin(xyz=(0.0, 0.302, -0.157)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.2, lower=-0.40, upper=0.55),
    )
    model.articulation(
        "left_nozzle_roller_spin",
        ArticulationType.CONTINUOUS,
        parent=floor_nozzle,
        child=left_nozzle_roller,
        origin=Origin(xyz=(0.120, 0.012, -0.069)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=20.0),
    )
    model.articulation(
        "right_nozzle_roller_spin",
        ArticulationType.CONTINUOUS,
        parent=floor_nozzle,
        child=right_nozzle_roller,
        origin=Origin(xyz=(-0.120, 0.012, -0.069)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    main_body = object_model.get_part("main_body")
    left_rear_wheel = object_model.get_part("left_rear_wheel")
    right_rear_wheel = object_model.get_part("right_rear_wheel")
    upper_wand = object_model.get_part("upper_wand")
    lower_wand = object_model.get_part("lower_wand")
    floor_nozzle = object_model.get_part("floor_nozzle")
    left_nozzle_roller = object_model.get_part("left_nozzle_roller")
    right_nozzle_roller = object_model.get_part("right_nozzle_roller")

    left_rear_wheel_spin = object_model.get_articulation("left_rear_wheel_spin")
    right_rear_wheel_spin = object_model.get_articulation("right_rear_wheel_spin")
    body_to_upper_wand = object_model.get_articulation("body_to_upper_wand")
    upper_to_lower_wand = object_model.get_articulation("upper_to_lower_wand")
    lower_to_floor_nozzle = object_model.get_articulation("lower_to_floor_nozzle")
    left_nozzle_roller_spin = object_model.get_articulation("left_nozzle_roller_spin")
    right_nozzle_roller_spin = object_model.get_articulation("right_nozzle_roller_spin")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    for part_name in (
        "main_body",
        "left_rear_wheel",
        "right_rear_wheel",
        "upper_wand",
        "lower_wand",
        "floor_nozzle",
        "left_nozzle_roller",
        "right_nozzle_roller",
    ):
        ctx.check(f"{part_name}_present", object_model.get_part(part_name) is not None, f"missing part {part_name}")

    ctx.check(
        "wand_joint_axes_are_x",
        body_to_upper_wand.axis == (1.0, 0.0, 0.0)
        and upper_to_lower_wand.axis == (1.0, 0.0, 0.0)
        and lower_to_floor_nozzle.axis == (1.0, 0.0, 0.0),
        "wand and nozzle pivots should pitch about the world/local x axis",
    )
    ctx.check(
        "wheel_joint_axes_are_x",
        left_rear_wheel_spin.axis == (1.0, 0.0, 0.0)
        and right_rear_wheel_spin.axis == (1.0, 0.0, 0.0)
        and left_nozzle_roller_spin.axis == (1.0, 0.0, 0.0)
        and right_nozzle_roller_spin.axis == (1.0, 0.0, 0.0),
        "rolling elements should spin around x-axis axles",
    )

    ctx.expect_contact(
        left_rear_wheel,
        main_body,
        elem_a="inner_hub",
        elem_b="left_axle_boss",
        name="left_rear_wheel_mounted_to_body",
    )
    ctx.expect_contact(
        right_rear_wheel,
        main_body,
        elem_a="inner_hub",
        elem_b="right_axle_boss",
        name="right_rear_wheel_mounted_to_body",
    )
    ctx.expect_contact(
        upper_wand,
        main_body,
        elem_a="rear_pivot_barrel",
        elem_b="left_wand_pivot_washer",
        name="upper_wand_left_pivot_contact",
    )
    ctx.expect_contact(
        upper_wand,
        main_body,
        elem_a="rear_pivot_barrel",
        elem_b="right_wand_pivot_washer",
        name="upper_wand_right_pivot_contact",
    )
    ctx.expect_contact(
        lower_wand,
        upper_wand,
        elem_a="rear_pivot_barrel",
        elem_b="left_elbow_washer",
        name="lower_wand_left_elbow_contact",
    )
    ctx.expect_contact(
        lower_wand,
        upper_wand,
        elem_a="rear_pivot_barrel",
        elem_b="right_elbow_washer",
        name="lower_wand_right_elbow_contact",
    )
    ctx.expect_contact(
        floor_nozzle,
        lower_wand,
        elem_a="pivot_barrel",
        elem_b="left_nozzle_washer",
        name="floor_nozzle_left_pivot_contact",
    )
    ctx.expect_contact(
        floor_nozzle,
        lower_wand,
        elem_a="pivot_barrel",
        elem_b="right_nozzle_washer",
        name="floor_nozzle_right_pivot_contact",
    )
    ctx.expect_contact(
        left_nozzle_roller,
        floor_nozzle,
        elem_a="roller_tire",
        elem_b="left_roller_inboard_washer",
        name="left_nozzle_roller_mounted",
    )
    ctx.expect_contact(
        right_nozzle_roller,
        floor_nozzle,
        elem_a="roller_tire",
        elem_b="right_roller_inboard_washer",
        name="right_nozzle_roller_mounted",
    )

    ctx.expect_origin_distance(
        left_rear_wheel,
        right_rear_wheel,
        axes="x",
        min_dist=0.34,
        max_dist=0.40,
        name="rear_track_width_realistic",
    )
    ctx.expect_origin_gap(
        floor_nozzle,
        main_body,
        axis="y",
        min_gap=0.70,
        name="floor_nozzle_reaches_ahead_of_body",
    )
    ctx.expect_overlap(
        floor_nozzle,
        lower_wand,
        axes="x",
        min_overlap=0.03,
        name="wand_and_nozzle_stay_centered",
    )

    def check_pose(name: str, pose_map: dict[object, float]) -> None:
        with ctx.pose(pose_map):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{name}_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{name}_no_floating")

    check_pose(
        "reach_pose",
        {
            body_to_upper_wand: 0.18,
            upper_to_lower_wand: -0.10,
            lower_to_floor_nozzle: -0.08,
        },
    )
    check_pose(
        "upright_service_pose",
        {
            body_to_upper_wand: 0.72,
            upper_to_lower_wand: 0.52,
            lower_to_floor_nozzle: 0.28,
        },
    )
    check_pose(
        "deep_reach_pose",
        {
            body_to_upper_wand: -0.10,
            upper_to_lower_wand: 0.22,
            lower_to_floor_nozzle: -0.22,
        },
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
