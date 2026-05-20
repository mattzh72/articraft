from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


X_CYLINDER_RPY = (0.0, math.pi / 2.0, 0.0)
Y_CYLINDER_RPY = (-math.pi / 2.0, 0.0, 0.0)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _lathe_shell(name: str, outer_profile, inner_profile, *, rotate_y: float = 0.0, translate=(0.0, 0.0, 0.0)):
    shell = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    if rotate_y:
        shell.rotate_y(rotate_y)
    if translate != (0.0, 0.0, 0.0):
        shell.translate(*translate)
    return _mesh(name, shell)


def _tripod_leg_mesh(name: str, radius: float, points: list[tuple[float, float, float]]):
    return _mesh(
        name,
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=18,
            radial_segments=20,
            cap_ends=True,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_telescope_tripod")

    tripod_metal = model.material("tripod_metal", rgba=(0.22, 0.23, 0.25, 1.0))
    alloy = model.material("alloy", rgba=(0.66, 0.69, 0.72, 1.0))
    telescope_paint = model.material("telescope_paint", rgba=(0.18, 0.24, 0.18, 1.0))
    black_polymer = model.material("black_polymer", rgba=(0.08, 0.09, 0.10, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.06, 1.0))
    optics = model.material("optics", rgba=(0.36, 0.50, 0.66, 0.55))

    tripod_body = model.part("tripod_body")
    tripod_body.visual(
        _lathe_shell(
            "tripod_crown_shell",
            [
                (0.036, -0.075),
                (0.078, -0.070),
                (0.088, -0.040),
                (0.086, -0.005),
                (0.066, 0.050),
                (0.046, 0.120),
                (0.041, 0.240),
            ],
            [
                (0.027, -0.073),
                (0.027, 0.050),
                (0.028, 0.135),
                (0.029, 0.240),
            ],
        ),
        material=tripod_metal,
        name="crown_shell",
    )
    tripod_body.visual(
        Cylinder(radius=0.007, length=0.032),
        origin=Origin(xyz=(0.051, 0.0, 0.118), rpy=X_CYLINDER_RPY),
        material=black_polymer,
        name="column_lock_stem",
    )
    tripod_body.visual(
        Cylinder(radius=0.016, length=0.022),
        origin=Origin(xyz=(0.078, 0.0, 0.118), rpy=X_CYLINDER_RPY),
        material=black_polymer,
        name="column_lock_knob",
    )

    leg_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    for index, angle in enumerate(leg_angles):
        c = math.cos(angle)
        s = math.sin(angle)
        tangent = (-s, c, 0.0)
        hinge_xyz = (0.104 * c, 0.104 * s, -0.050)
        tripod_body.visual(
            Box((0.032, 0.034, 0.028)),
            origin=Origin(xyz=(0.076 * c, 0.076 * s, -0.050), rpy=(0.0, 0.0, angle)),
            material=tripod_metal,
            name=f"hinge_block_{index}",
        )
        for ear_index, ear_sign in enumerate((-1.0, 1.0)):
            tripod_body.visual(
                Cylinder(radius=0.013, length=0.016),
                origin=Origin(
                    xyz=(
                        hinge_xyz[0] + tangent[0] * 0.014 * ear_sign,
                        hinge_xyz[1] + tangent[1] * 0.014 * ear_sign,
                        hinge_xyz[2],
                    ),
                    rpy=(-math.pi / 2.0, 0.0, angle),
                ),
                material=tripod_metal,
                name=f"hinge_ear_{index}_{ear_index}",
            )

    for index, angle in enumerate(leg_angles):
        leg = model.part(f"leg_{index}")
        leg.visual(
            Cylinder(radius=0.012, length=0.012),
            origin=Origin(rpy=Y_CYLINDER_RPY),
            material=tripod_metal,
            name="hinge_barrel",
        )
        leg.visual(
            Box((0.030, 0.010, 0.024)),
            origin=Origin(xyz=(0.015, 0.0, -0.004)),
            material=tripod_metal,
            name="hinge_shoulder",
        )
        leg.visual(
            _tripod_leg_mesh(
                f"tripod_leg_upper_{index}",
                0.016,
                [
                    (0.020, 0.0, -0.010),
                    (0.145, 0.0, -0.245),
                    (0.255, 0.0, -0.470),
                ],
            ),
            material=alloy,
            name="upper_tube",
        )
        leg.visual(
            _tripod_leg_mesh(
                f"tripod_leg_lower_{index}",
                0.012,
                [
                    (0.248, 0.0, -0.462),
                    (0.335, 0.0, -0.680),
                    (0.402, 0.0, -0.845),
                ],
            ),
            material=alloy,
            name="lower_tube",
        )
        leg.visual(
            Sphere(radius=0.020),
            origin=Origin(xyz=(0.402, 0.0, -0.845)),
            material=rubber,
            name="foot_pad",
        )
        model.articulation(
            f"tripod_body_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=tripod_body,
            child=leg,
            origin=Origin(xyz=(0.104 * math.cos(angle), 0.104 * math.sin(angle), -0.050), rpy=(0.0, 0.0, angle)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=35.0,
                velocity=1.2,
                lower=0.0,
                upper=1.05,
            ),
        )

    center_column = model.part("center_column")
    center_column.visual(
        Cylinder(radius=0.022, length=0.760),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=alloy,
        name="column_tube",
    )
    center_column.visual(
        Cylinder(radius=0.034, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=black_polymer,
        name="column_stop_collar",
    )
    center_column.visual(
        Cylinder(radius=0.029, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.411)),
        material=black_polymer,
        name="column_collar",
    )
    center_column.visual(
        Cylinder(radius=0.021, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.430)),
        material=tripod_metal,
        name="column_spigot",
    )
    model.articulation(
        "tripod_body_to_center_column",
        ArticulationType.PRISMATIC,
        parent=tripod_body,
        child=center_column,
        origin=Origin(xyz=(0.0, 0.0, 0.240)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.16,
            lower=0.0,
            upper=0.220,
        ),
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.046, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=tripod_metal,
        name="pan_base",
    )
    pan_head.visual(
        Cylinder(radius=0.024, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=tripod_metal,
        name="pan_neck",
    )
    pan_head.visual(
        Box((0.080, 0.100, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.076)),
        material=tripod_metal,
        name="tilt_bridge",
    )
    for side, side_name in ((-1.0, "left"), (1.0, "right")):
        pan_head.visual(
            Box((0.048, 0.012, 0.095)),
            origin=Origin(xyz=(0.0, 0.056 * side, 0.1315)),
            material=tripod_metal,
            name=f"{side_name}_cheek",
        )
        pan_head.visual(
            Cylinder(radius=0.020, length=0.010),
            origin=Origin(xyz=(0.0, 0.067 * side, 0.132), rpy=Y_CYLINDER_RPY),
            material=black_polymer,
            name=f"{side_name}_trunnion_cap",
        )
    pan_head.visual(
        _tripod_leg_mesh(
            "pan_handle_bar",
            0.006,
            [
                (-0.020, 0.0, 0.074),
                (-0.110, 0.0, 0.020),
                (-0.205, 0.0, -0.030),
                (-0.255, 0.0, -0.070),
            ],
        ),
        material=tripod_metal,
        name="pan_handle_bar",
    )
    pan_head.visual(
        Cylinder(radius=0.012, length=0.090),
        origin=Origin(xyz=(-0.245, 0.0, -0.070), rpy=X_CYLINDER_RPY),
        material=black_polymer,
        name="pan_handle_grip",
    )
    model.articulation(
        "center_column_to_pan_head",
        ArticulationType.CONTINUOUS,
        parent=center_column,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 0.440)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=2.5,
        ),
    )

    telescope = model.part("telescope")
    telescope.visual(
        Cylinder(radius=0.048, length=0.060),
        origin=Origin(xyz=(0.030, 0.0, 0.0), rpy=X_CYLINDER_RPY),
        material=tripod_metal,
        name="mount_ring",
    )
    telescope.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(xyz=(0.0, -0.040, 0.0), rpy=Y_CYLINDER_RPY),
        material=tripod_metal,
        name="trunnion_left",
    )
    telescope.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(xyz=(0.0, 0.040, 0.0), rpy=Y_CYLINDER_RPY),
        material=tripod_metal,
        name="trunnion_right",
    )
    telescope.visual(
        _lathe_shell(
            "telescope_tube_shell",
            [
                (0.046, -0.020),
                (0.046, 0.465),
                (0.054, 0.505),
                (0.054, 0.640),
            ],
            [
                (0.041, -0.020),
                (0.041, 0.465),
                (0.049, 0.505),
                (0.049, 0.640),
            ],
            rotate_y=math.pi / 2.0,
        ),
        material=telescope_paint,
        name="tube_shell",
    )
    telescope.visual(
        Cylinder(radius=0.044, length=0.045),
        origin=Origin(xyz=(-0.020, 0.0, 0.0), rpy=X_CYLINDER_RPY),
        material=telescope_paint,
        name="rear_adapter",
    )
    telescope.visual(
        Cylinder(radius=0.050, length=0.060),
        origin=Origin(xyz=(0.585, 0.0, 0.0), rpy=X_CYLINDER_RPY),
        material=black_polymer,
        name="objective_cell",
    )
    telescope.visual(
        Cylinder(radius=0.041, length=0.110),
        origin=Origin(xyz=(0.570, 0.0, 0.0), rpy=X_CYLINDER_RPY),
        material=black_polymer,
        name="dew_baffle",
    )
    telescope.visual(
        Cylinder(radius=0.043, length=0.004),
        origin=Origin(xyz=(0.620, 0.0, 0.0), rpy=X_CYLINDER_RPY),
        material=optics,
        name="objective_lens",
    )
    telescope.visual(
        Cylinder(radius=0.026, length=0.130),
        origin=Origin(xyz=(-0.070, 0.0, 0.0), rpy=X_CYLINDER_RPY),
        material=black_polymer,
        name="focuser_body",
    )
    telescope.visual(
        Box((0.060, 0.040, 0.040)),
        origin=Origin(xyz=(-0.125, 0.0, 0.030)),
        material=black_polymer,
        name="diagonal_housing",
    )
    telescope.visual(
        Cylinder(radius=0.014, length=0.100),
        origin=Origin(xyz=(-0.145, 0.0, 0.085)),
        material=black_polymer,
        name="eyepiece_barrel",
    )
    telescope.visual(
        Cylinder(radius=0.019, length=0.030),
        origin=Origin(xyz=(-0.145, 0.0, 0.140)),
        material=rubber,
        name="eyecup",
    )
    telescope.visual(
        Sphere(radius=0.012),
        origin=Origin(xyz=(-0.065, -0.036, 0.0)),
        material=black_polymer,
        name="focus_knob_left",
    )
    telescope.visual(
        Cylinder(radius=0.004, length=0.020),
        origin=Origin(xyz=(-0.065, -0.026, 0.0), rpy=Y_CYLINDER_RPY),
        material=black_polymer,
        name="focus_shaft_left",
    )
    telescope.visual(
        Sphere(radius=0.012),
        origin=Origin(xyz=(-0.065, 0.036, 0.0)),
        material=black_polymer,
        name="focus_knob_right",
    )
    telescope.visual(
        Cylinder(radius=0.004, length=0.020),
        origin=Origin(xyz=(-0.065, 0.026, 0.0), rpy=Y_CYLINDER_RPY),
        material=black_polymer,
        name="focus_shaft_right",
    )
    telescope.visual(
        Cylinder(radius=0.012, length=0.180),
        origin=Origin(xyz=(0.190, 0.0, 0.085), rpy=X_CYLINDER_RPY),
        material=black_polymer,
        name="finder_scope",
    )
    telescope.visual(
        Box((0.012, 0.028, 0.050)),
        origin=Origin(xyz=(0.110, 0.0, 0.060)),
        material=tripod_metal,
        name="finder_bracket_rear",
    )
    telescope.visual(
        Box((0.012, 0.028, 0.050)),
        origin=Origin(xyz=(0.255, 0.0, 0.060)),
        material=tripod_metal,
        name="finder_bracket_front",
    )
    model.articulation(
        "pan_head_to_telescope",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=telescope,
        origin=Origin(xyz=(0.0, 0.0, 0.132)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=-0.65,
            upper=1.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tripod_body = object_model.get_part("tripod_body")
    center_column = object_model.get_part("center_column")
    telescope = object_model.get_part("telescope")
    leg_0 = object_model.get_part("leg_0")

    column_slide = object_model.get_articulation("tripod_body_to_center_column")
    pan_joint = object_model.get_articulation("center_column_to_pan_head")
    tilt_joint = object_model.get_articulation("pan_head_to_telescope")
    leg_joint = object_model.get_articulation("tripod_body_to_leg_0")

    def aabb_center(aabb):
        if aabb is None:
            return None
        min_corner, max_corner = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(min_corner, max_corner))

    ctx.expect_within(
        center_column,
        tripod_body,
        axes="xy",
        inner_elem="column_tube",
        outer_elem="crown_shell",
        name="center column stays centered in the crown sleeve",
    )
    ctx.expect_overlap(
        center_column,
        tripod_body,
        axes="z",
        elem_a="column_tube",
        elem_b="crown_shell",
        min_overlap=0.18,
        name="collapsed center column remains inserted in the crown sleeve",
    )

    column_rest = ctx.part_world_position(center_column)
    with ctx.pose({column_slide: 0.22}):
        ctx.expect_within(
            center_column,
            tripod_body,
            axes="xy",
            inner_elem="column_tube",
            outer_elem="crown_shell",
            name="extended center column stays centered in the crown sleeve",
        )
        ctx.expect_overlap(
            center_column,
            tripod_body,
            axes="z",
            elem_a="column_tube",
            elem_b="crown_shell",
            min_overlap=0.10,
            name="extended center column still retains insertion in the crown sleeve",
        )
        column_extended = ctx.part_world_position(center_column)

    ctx.check(
        "center column extends upward",
        column_rest is not None and column_extended is not None and column_extended[2] > column_rest[2] + 0.18,
        details=f"rest={column_rest}, extended={column_extended}",
    )

    objective_rest = aabb_center(ctx.part_element_world_aabb(telescope, elem="objective_lens"))
    with ctx.pose({pan_joint: math.pi / 2.0}):
        objective_panned = aabb_center(ctx.part_element_world_aabb(telescope, elem="objective_lens"))
    ctx.check(
        "pan head rotates telescope around the vertical axis",
        objective_rest is not None
        and objective_panned is not None
        and objective_rest[0] > 0.45
        and abs(objective_rest[1]) < 0.05
        and objective_panned[1] > 0.45
        and abs(objective_panned[0]) < 0.06,
        details=f"rest={objective_rest}, panned={objective_panned}",
    )

    with ctx.pose({tilt_joint: 0.70}):
        objective_tilted = aabb_center(ctx.part_element_world_aabb(telescope, elem="objective_lens"))
    ctx.check(
        "telescope tilts upward in the cradle",
        objective_rest is not None
        and objective_tilted is not None
        and objective_tilted[2] > objective_rest[2] + 0.30
        and objective_tilted[0] < objective_rest[0] - 0.10,
        details=f"rest={objective_rest}, tilted={objective_tilted}",
    )

    foot_rest = aabb_center(ctx.part_element_world_aabb(leg_0, elem="foot_pad"))
    with ctx.pose({leg_joint: 0.95}):
        foot_folded = aabb_center(ctx.part_element_world_aabb(leg_0, elem="foot_pad"))
    ctx.check(
        "tripod leg can fold upward from the deployed stance",
        foot_rest is not None and foot_folded is not None and foot_folded[2] > foot_rest[2] + 0.55,
        details=f"rest={foot_rest}, folded={foot_folded}",
    )

    return ctx.report()


object_model = build_object_model()
