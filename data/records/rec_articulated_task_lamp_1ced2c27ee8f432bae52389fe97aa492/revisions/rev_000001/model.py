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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _torchiere_bowl_mesh():
    return _mesh(
        "main_torchiere_bowl",
        LatheGeometry.from_shell_profiles(
            [
                (0.028, 0.000),
                (0.040, 0.012),
                (0.098, 0.044),
                (0.154, 0.100),
                (0.178, 0.145),
            ],
            [
                (0.021, 0.004),
                (0.032, 0.016),
                (0.090, 0.047),
                (0.146, 0.100),
                (0.170, 0.141),
            ],
            segments=72,
            start_cap="round",
            end_cap="flat",
            lip_samples=10,
        ),
    )


def _reading_shade_mesh():
    return _mesh(
        "reading_shade_shell",
        LatheGeometry.from_shell_profiles(
            [
                (0.014, 0.000),
                (0.020, 0.010),
                (0.037, 0.036),
                (0.055, 0.082),
                (0.061, 0.118),
            ],
            [
                (0.010, 0.004),
                (0.016, 0.013),
                (0.032, 0.038),
                (0.049, 0.082),
                (0.055, 0.113),
            ],
            segments=56,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="torchiere_floor_lamp")

    satin_black = model.material("satin_black", rgba=(0.16, 0.16, 0.17, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.24, 0.25, 0.27, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    warm_white = model.material("warm_white", rgba=(0.94, 0.92, 0.86, 1.0))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.165, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=satin_black,
        name="base_disc",
    )
    stand.visual(
        Cylinder(radius=0.126, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=dark_steel,
        name="base_foot_ring",
    )
    stand.visual(
        Cylinder(radius=0.012, length=1.560),
        origin=Origin(xyz=(0.0, 0.0, 0.820)),
        material=brushed_steel,
        name="post_shaft",
    )
    stand.visual(
        Cylinder(radius=0.028, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 1.080)),
        material=satin_black,
        name="shoulder_collar",
    )
    stand.visual(
        Cylinder(radius=0.018, length=0.040),
        origin=Origin(xyz=(0.020, 0.0, 1.080), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="shoulder_boss",
    )
    stand.visual(
        Cylinder(radius=0.030, length=0.065),
        origin=Origin(xyz=(0.0, 0.0, 1.6325)),
        material=satin_black,
        name="top_socket",
    )
    stand.inertial = Inertial.from_geometry(
        Cylinder(radius=0.165, length=1.70),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, 0.85)),
    )

    main_shade = model.part("main_shade")
    main_shade.visual(
        _torchiere_bowl_mesh(),
        material=warm_white,
        name="main_bowl_shell",
    )
    main_shade.visual(
        Cylinder(radius=0.023, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=satin_black,
        name="mount_collar",
    )
    main_shade.visual(
        Cylinder(radius=0.012, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=dark_steel,
        name="lamp_socket_stem",
    )
    main_shade.inertial = Inertial.from_geometry(
        Cylinder(radius=0.180, length=0.170),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
    )

    reading_arm = model.part("reading_arm")
    reading_arm.visual(
        Cylinder(radius=0.012, length=0.036),
        origin=Origin(xyz=(0.012, 0.0, 0.0)),
        material=satin_black,
        name="root_knuckle",
    )
    reading_arm.visual(
        _mesh(
            "reading_arm_tube",
            tube_from_spline_points(
                [
                    (0.018, 0.0, 0.000),
                    (0.120, 0.0, 0.018),
                    (0.255, 0.0, 0.052),
                    (0.360, 0.0, 0.082),
                    (0.402, 0.0, 0.090),
                ],
                radius=0.008,
                samples_per_segment=18,
                radial_segments=16,
                cap_ends=True,
            ),
        ),
        material=brushed_steel,
        name="arm_tube",
    )
    reading_arm.visual(
        Box((0.028, 0.040, 0.022)),
        origin=Origin(xyz=(0.406, 0.0, 0.090)),
        material=satin_black,
        name="tip_block",
    )
    reading_arm.inertial = Inertial.from_geometry(
        Box((0.430, 0.050, 0.120)),
        mass=0.8,
        origin=Origin(xyz=(0.215, 0.0, 0.050)),
    )

    reading_shade = model.part("reading_shade")
    reading_shade.visual(
        Cylinder(radius=0.010, length=0.030),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="hinge_barrel",
    )
    reading_shade.visual(
        _reading_shade_mesh(),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0 + 0.12, 0.0)),
        material=warm_white,
        name="shade_shell",
    )
    reading_shade.visual(
        Cylinder(radius=0.013, length=0.030),
        origin=Origin(xyz=(0.028, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="shade_neck",
    )
    reading_shade.inertial = Inertial.from_geometry(
        Cylinder(radius=0.062, length=0.125),
        mass=0.45,
        origin=Origin(xyz=(0.060, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "stand_to_main_shade",
        ArticulationType.FIXED,
        parent=stand,
        child=main_shade,
        origin=Origin(xyz=(0.0, 0.0, 1.665)),
    )
    model.articulation(
        "stand_to_reading_arm",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=reading_arm,
        origin=Origin(xyz=(0.040, 0.0, 1.080)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.4,
            lower=-2.15,
            upper=2.15,
        ),
    )
    model.articulation(
        "reading_arm_to_shade",
        ArticulationType.REVOLUTE,
        parent=reading_arm,
        child=reading_shade,
        origin=Origin(xyz=(0.420, 0.0, 0.090)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=1.8,
            lower=-0.55,
            upper=1.00,
        ),
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

    stand = object_model.get_part("stand")
    main_shade = object_model.get_part("main_shade")
    reading_arm = object_model.get_part("reading_arm")
    reading_shade = object_model.get_part("reading_shade")
    shoulder = object_model.get_articulation("stand_to_reading_arm")
    tilt = object_model.get_articulation("reading_arm_to_shade")

    ctx.check(
        "shoulder joint revolves around vertical axis",
        shoulder.axis == (0.0, 0.0, 1.0),
        details=f"axis={shoulder.axis}",
    )
    ctx.check(
        "reading shade tilts on lateral axis",
        tilt.axis == (0.0, 1.0, 0.0),
        details=f"axis={tilt.axis}",
    )

    ctx.expect_origin_distance(
        main_shade,
        stand,
        axes="xy",
        max_dist=0.001,
        name="main shade stays centered on the post",
    )
    ctx.expect_contact(
        main_shade,
        stand,
        elem_a="mount_collar",
        elem_b="top_socket",
        contact_tol=0.0015,
        name="main shade mounts to the top socket",
    )
    ctx.expect_contact(
        reading_arm,
        stand,
        elem_a="root_knuckle",
        elem_b="shoulder_boss",
        contact_tol=0.0015,
        name="reading arm seats against the shoulder boss",
    )
    ctx.expect_contact(
        reading_shade,
        reading_arm,
        elem_a="hinge_barrel",
        elem_b="tip_block",
        contact_tol=0.0015,
        name="reading shade seats at the arm tip",
    )

    with ctx.pose({shoulder: shoulder.motion_limits.lower}):
        lower_sweep_pos = ctx.part_world_position(reading_shade)
    with ctx.pose({shoulder: shoulder.motion_limits.upper}):
        upper_sweep_pos = ctx.part_world_position(reading_shade)

    ctx.check(
        "reading arm sweeps around the post",
        lower_sweep_pos is not None
        and upper_sweep_pos is not None
        and abs(upper_sweep_pos[1] - lower_sweep_pos[1]) > 0.55
        and upper_sweep_pos[0] < 0.0
        and lower_sweep_pos[0] < 0.0,
        details=f"lower={lower_sweep_pos}, upper={upper_sweep_pos}",
    )

    with ctx.pose({shoulder: 1.25}):
        ctx.expect_origin_distance(
            reading_shade,
            stand,
            axes="xy",
            min_dist=0.20,
            name="reading shade remains offset from the stand when swung aside",
        )

    def _center_from_aabb(aabb):
        if aabb is None:
            return None
        (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
        return (
            0.5 * (min_x + max_x),
            0.5 * (min_y + max_y),
            0.5 * (min_z + max_z),
        )

    with ctx.pose({tilt: tilt.motion_limits.lower}):
        shade_up_center = _center_from_aabb(
            ctx.part_element_world_aabb(reading_shade, elem="shade_shell")
        )
    with ctx.pose({tilt: tilt.motion_limits.upper}):
        shade_down_center = _center_from_aabb(
            ctx.part_element_world_aabb(reading_shade, elem="shade_shell")
        )

    ctx.check(
        "positive tilt lowers the reading shade",
        shade_up_center is not None
        and shade_down_center is not None
        and shade_down_center[2] < shade_up_center[2] - 0.03,
        details=f"up={shade_up_center}, down={shade_down_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
