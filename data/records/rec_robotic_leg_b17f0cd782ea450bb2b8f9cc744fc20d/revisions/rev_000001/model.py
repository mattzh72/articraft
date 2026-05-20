from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LoftGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_profile,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _loft_z(
    sections: list[tuple[float, float, float, float, float]],
    *,
    segments: int = 44,
    exponent: float = 2.8,
):
    """Loft a sculpted oval housing along local Z.

    Each section is (z, width_x, depth_y, x_offset, y_offset).
    """
    loops = []
    for z, width_x, depth_y, x_offset, y_offset in sections:
        profile = superellipse_profile(width_x, depth_y, exponent=exponent, segments=segments)
        loops.append([(x + x_offset, y + y_offset, z) for x, y in profile])
    return LoftGeometry(loops, cap=True, closed=True)


def _loft_x(
    sections: list[tuple[float, float, float, float, float]],
    *,
    segments: int = 44,
    exponent: float = 2.6,
):
    """Loft an oval volume along local X.

    Each section is (x, width_y, height_z, y_offset, z_offset).
    """
    loops = []
    for x, width_y, height_z, y_offset, z_offset in sections:
        profile = superellipse_profile(width_y, height_z, exponent=exponent, segments=segments)
        # LoftGeometry expects every section loop to have area in XY at a
        # constant temporary Z.  Build that way, then permute axes so the loft
        # runs along the real local X axis.
        loops.append([(y + y_offset, z + z_offset, x) for y, z in profile])
    geom = LoftGeometry(loops, cap=True, closed=True)
    geom.vertices = [(z, x, y) for x, y, z in geom.vertices]
    return geom


def _add_bolt(part, *, x: float, y: float, z: float, material: Material, name: str) -> None:
    part.visual(
        Cylinder(radius=0.012, length=0.008),
        origin=Origin(xyz=(x, y, z)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="humanoid_robotic_leg")

    warm_shell = model.material("warm_ceramic_shell", rgba=(0.78, 0.80, 0.76, 1.0))
    graphite = model.material("graphite_hardcoat", rgba=(0.08, 0.09, 0.10, 1.0))
    dark_insert = model.material("dark_service_insert", rgba=(0.16, 0.18, 0.19, 1.0))
    satin_steel = model.material("satin_bearing_steel", rgba=(0.62, 0.64, 0.64, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.025, 0.025, 0.023, 1.0))
    accent = model.material("blue_status_glass", rgba=(0.10, 0.38, 0.75, 1.0))

    hip_mount = model.part("hip_mount")
    hip_mount.visual(
        _mesh(
            "hip_compact_socket",
            _loft_x(
                [
                    (-0.235, 0.145, 0.125, 0.0, 0.075),
                    (-0.165, 0.175, 0.155, 0.0, 0.055),
                    (-0.095, 0.150, 0.120, 0.0, 0.030),
                ],
                segments=48,
                exponent=2.6,
            ),
        ),
        material=graphite,
        name="pelvis_socket",
    )
    hip_mount.visual(
        Box((0.060, 0.160, 0.100)),
        origin=Origin(xyz=(-0.120, 0.0, 0.000)),
        material=graphite,
        name="rear_bridge",
    )
    for side, y in enumerate((0.064, -0.064)):
        hip_mount.visual(
            Cylinder(radius=0.105, length=0.018),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=graphite,
            name=("hip_cheek_0" if side == 0 else "hip_cheek_1"),
        )
        hip_mount.visual(
            Cylinder(radius=0.067, length=0.006),
            origin=Origin(xyz=(0.0, y + (0.012 if y > 0 else -0.012), 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=satin_steel,
            name=f"hip_outer_cap_{side}",
        )
    hip_mount.visual(
        Box((0.205, 0.160, 0.034)),
        origin=Origin(xyz=(-0.165, 0.0, 0.150)),
        material=graphite,
        name="mounting_flange",
    )
    for ix, x in enumerate((-0.225, -0.105)):
        for iy, y in enumerate((-0.052, 0.052)):
            _add_bolt(hip_mount, x=x, y=y, z=0.171, material=satin_steel, name=f"bolt_{ix}_{iy}")
    hip_mount.visual(
        Cylinder(radius=0.014, length=0.040),
        origin=Origin(xyz=(-0.236, 0.0, 0.088), rpy=(0.0, pi / 2.0, 0.0)),
        material=accent,
        name="status_lens",
    )

    upper_leg = model.part("upper_leg")
    upper_leg.visual(
        Cylinder(radius=0.064, length=0.110),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="hip_barrel",
    )
    upper_leg.visual(
        Box((0.095, 0.082, 0.050)),
        origin=Origin(xyz=(0.006, 0.0, -0.055)),
        material=graphite,
        name="hip_neck",
    )
    upper_leg.visual(
        _mesh(
            "upper_leg_sculpted_shell",
            _loft_z(
                [
                    (-0.078, 0.140, 0.084, 0.012, 0.0),
                    (-0.165, 0.174, 0.112, 0.018, 0.0),
                    (-0.292, 0.158, 0.104, 0.000, 0.0),
                    (-0.405, 0.122, 0.090, -0.012, 0.0),
                ],
                segments=56,
                exponent=2.7,
            ),
        ),
        material=warm_shell,
        name="upper_shell",
    )
    upper_leg.visual(
        Box((0.020, 0.010, 0.245)),
        origin=Origin(xyz=(0.074, 0.000, -0.258)),
        material=dark_insert,
        name="front_service_strip",
    )
    upper_leg.visual(
        Cylinder(radius=0.037, length=0.170),
        origin=Origin(xyz=(0.082, 0.000, -0.214)),
        material=graphite,
        name="thigh_actuator_can",
    )
    upper_leg.visual(
        Cylinder(radius=0.016, length=0.210),
        origin=Origin(xyz=(0.089, 0.000, -0.330)),
        material=satin_steel,
        name="actuator_rod",
    )
    for side, y in enumerate((0.056, -0.056)):
        upper_leg.visual(
            Box((0.047, 0.018, 0.150)),
            origin=Origin(xyz=(-0.006, y, -0.334)),
            material=graphite,
            name=f"knee_side_plate_{side}",
        )
        upper_leg.visual(
            Cylinder(radius=0.085, length=0.018),
            origin=Origin(xyz=(0.0, 0.064 if y > 0 else -0.064, -0.480), rpy=(pi / 2.0, 0.0, 0.0)),
            material=graphite,
            name=("knee_cheek_0" if side == 0 else "knee_cheek_1"),
        )
        upper_leg.visual(
            Cylinder(radius=0.048, length=0.006),
            origin=Origin(xyz=(0.0, 0.073 if y > 0 else -0.073, -0.480), rpy=(pi / 2.0, 0.0, 0.0)),
            material=satin_steel,
            name=f"knee_outer_cap_{side}",
        )
        upper_leg.visual(
            Box((0.020, 0.020, 0.230)),
            origin=Origin(xyz=(-0.060, 0.050 if y > 0 else -0.050, -0.275)),
            material=dark_insert,
            name=f"side_inset_{side}",
        )

    lower_leg = model.part("lower_leg")
    lower_leg.visual(
        Cylinder(radius=0.064, length=0.110),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="knee_barrel",
    )
    lower_leg.visual(
        _mesh(
            "lower_leg_tapered_shell",
            _loft_z(
                [
                    (-0.055, 0.126, 0.094, 0.000, 0.0),
                    (-0.160, 0.142, 0.096, -0.010, 0.0),
                    (-0.300, 0.118, 0.086, -0.006, 0.0),
                    (-0.402, 0.092, 0.074, 0.008, 0.0),
                ],
                segments=56,
                exponent=2.85,
            ),
        ),
        material=warm_shell,
        name="lower_shell",
    )
    lower_leg.visual(
        Box((0.018, 0.009, 0.275)),
        origin=Origin(xyz=(0.063, 0.0, -0.250)),
        material=dark_insert,
        name="shin_service_strip",
    )
    lower_leg.visual(
        Cylinder(radius=0.026, length=0.250),
        origin=Origin(xyz=(-0.055, 0.0, -0.210)),
        material=graphite,
        name="shin_power_module",
    )
    for side, y in enumerate((0.046, -0.046)):
        lower_leg.visual(
            Box((0.038, 0.014, 0.128)),
            origin=Origin(xyz=(0.004, y, -0.340)),
            material=graphite,
            name=f"ankle_side_plate_{side}",
        )
        lower_leg.visual(
            Box((0.038, 0.014, 0.040)),
            origin=Origin(xyz=(0.004, 0.057 if y > 0 else -0.057, -0.412)),
            material=graphite,
            name=f"ankle_bridge_{side}",
        )
        lower_leg.visual(
            Cylinder(radius=0.066, length=0.016),
            origin=Origin(xyz=(0.0, 0.053 if y > 0 else -0.053, -0.480), rpy=(pi / 2.0, 0.0, 0.0)),
            material=graphite,
            name=("ankle_cheek_0" if side == 0 else "ankle_cheek_1"),
        )
        lower_leg.visual(
            Cylinder(radius=0.038, length=0.006),
            origin=Origin(xyz=(0.0, 0.060 if y > 0 else -0.060, -0.480), rpy=(pi / 2.0, 0.0, 0.0)),
            material=satin_steel,
            name=f"ankle_outer_cap_{side}",
        )

    lower_leg.visual(
        _mesh(
            "shin_signal_cable",
            tube_from_spline_points(
                [
                    (-0.060, -0.030, -0.070),
                    (-0.078, -0.040, -0.180),
                    (-0.068, -0.034, -0.318),
                    (-0.030, -0.024, -0.415),
                ],
                radius=0.006,
                samples_per_segment=10,
                radial_segments=12,
            ),
        ),
        material=rubber,
        name="signal_cable",
    )

    foot = model.part("foot")
    foot.visual(
        Cylinder(radius=0.045, length=0.090),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="ankle_barrel",
    )
    foot.visual(
        Box((0.080, 0.084, 0.078)),
        origin=Origin(xyz=(0.000, 0.0, -0.041)),
        material=graphite,
        name="ankle_block",
    )
    foot.visual(
        _mesh(
            "robot_foot_shell",
            _loft_x(
                [
                    (-0.105, 0.102, 0.052, 0.0, -0.102),
                    (0.030, 0.132, 0.070, 0.0, -0.104),
                    (0.180, 0.116, 0.056, 0.0, -0.111),
                    (0.245, 0.082, 0.038, 0.0, -0.118),
                ],
                segments=48,
                exponent=2.5,
            ),
        ),
        material=warm_shell,
        name="foot_shell",
    )
    foot.visual(
        _mesh(
            "robot_foot_sole",
            _loft_x(
                [
                    (-0.120, 0.132, 0.026, 0.0, -0.150),
                    (0.055, 0.158, 0.030, 0.0, -0.150),
                    (0.255, 0.126, 0.024, 0.0, -0.150),
                ],
                segments=42,
                exponent=3.0,
            ),
        ),
        material=rubber,
        name="sole_pad",
    )
    foot.visual(
        Box((0.015, 0.118, 0.010)),
        origin=Origin(xyz=(0.165, 0.0, -0.122)),
        material=dark_insert,
        name="toe_flex_line",
    )

    model.articulation(
        "hip_pitch",
        ArticulationType.REVOLUTE,
        parent=hip_mount,
        child=upper_leg,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=2.6, lower=-0.85, upper=1.15),
    )
    model.articulation(
        "knee_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_leg,
        child=lower_leg,
        origin=Origin(xyz=(0.0, 0.0, -0.480)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=2.8, lower=0.0, upper=2.05),
    )
    model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent=lower_leg,
        child=foot,
        origin=Origin(xyz=(0.0, 0.0, -0.480)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=3.0, lower=-0.55, upper=0.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hip_mount = object_model.get_part("hip_mount")
    upper_leg = object_model.get_part("upper_leg")
    lower_leg = object_model.get_part("lower_leg")
    foot = object_model.get_part("foot")
    hip = object_model.get_articulation("hip_pitch")
    knee = object_model.get_articulation("knee_pitch")
    ankle = object_model.get_articulation("ankle_pitch")

    ctx.expect_gap(
        hip_mount,
        upper_leg,
        axis="y",
        positive_elem="hip_cheek_0",
        negative_elem="hip_barrel",
        min_gap=-0.0005,
        max_gap=0.002,
        name="hip cheek captures barrel side",
    )
    ctx.expect_gap(
        upper_leg,
        hip_mount,
        axis="y",
        positive_elem="hip_barrel",
        negative_elem="hip_cheek_1",
        min_gap=-0.0005,
        max_gap=0.002,
        name="hip barrel captured between cheeks",
    )
    ctx.expect_gap(
        upper_leg,
        lower_leg,
        axis="y",
        positive_elem="knee_cheek_0",
        negative_elem="knee_barrel",
        min_gap=-0.0005,
        max_gap=0.002,
        name="knee cheek captures barrel side",
    )
    ctx.expect_gap(
        lower_leg,
        upper_leg,
        axis="y",
        positive_elem="knee_barrel",
        negative_elem="knee_cheek_1",
        min_gap=-0.0005,
        max_gap=0.002,
        name="knee barrel captured between cheeks",
    )
    ctx.expect_gap(
        lower_leg,
        foot,
        axis="y",
        positive_elem="ankle_cheek_0",
        negative_elem="ankle_barrel",
        min_gap=-0.0005,
        max_gap=0.002,
        name="ankle cheek captures barrel side",
    )
    ctx.expect_gap(
        foot,
        lower_leg,
        axis="y",
        positive_elem="ankle_barrel",
        negative_elem="ankle_cheek_1",
        min_gap=-0.0005,
        max_gap=0.002,
        name="ankle barrel captured between cheeks",
    )

    straight_ankle = ctx.part_world_position(foot)
    with ctx.pose({knee: 1.05}):
        flexed_ankle = ctx.part_world_position(foot)
    ctx.check(
        "knee flexion moves ankle rearward",
        straight_ankle is not None
        and flexed_ankle is not None
        and flexed_ankle[0] < straight_ankle[0] - 0.20,
        details=f"straight={straight_ankle}, flexed={flexed_ankle}",
    )

    straight_knee = ctx.part_world_position(lower_leg)
    with ctx.pose({hip: 0.65}):
        flexed_knee = ctx.part_world_position(lower_leg)
    ctx.check(
        "hip flexion moves knee forward",
        straight_knee is not None
        and flexed_knee is not None
        and flexed_knee[0] > straight_knee[0] + 0.20,
        details=f"straight={straight_knee}, flexed={flexed_knee}",
    )

    rest_foot_box = ctx.part_world_aabb(foot)
    with ctx.pose({ankle: 0.65}):
        raised_toe_box = ctx.part_world_aabb(foot)
    ctx.check(
        "ankle pitch raises the toe",
        rest_foot_box is not None
        and raised_toe_box is not None
        and raised_toe_box[1][2] > rest_foot_box[1][2] + 0.020,
        details=f"rest={rest_foot_box}, pitched={raised_toe_box}",
    )

    return ctx.report()


object_model = build_object_model()
