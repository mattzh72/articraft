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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_with_pan_tilt_device")

    matte_black = model.material("matte_black", rgba=(0.14, 0.14, 0.15, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.22, 0.23, 0.25, 1.0))
    aluminum = model.material("aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.09, 0.10, 1.0))

    crown_arm_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.0, 0.0, -0.004),
                (0.030, 0.0, -0.010),
                (0.056, 0.0, -0.014),
            ],
            radius=0.008,
            samples_per_segment=18,
            radial_segments=16,
            cap_ends=True,
        ),
        "tripod_crown_arm",
    )
    leg_tube_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.0, 0.0, 0.0),
                (0.080, 0.0, -0.120),
                (0.185, 0.0, -0.430),
                (0.320, 0.0, -0.900),
            ],
            radius=0.011,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
        "tripod_leg_tube",
    )
    yoke_arm_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.0, 0.0, 0.018),
                (0.022, 0.0, 0.060),
                (0.050, 0.0, 0.100),
            ],
            radius=0.007,
            samples_per_segment=18,
            radial_segments=16,
            cap_ends=True,
        ),
        "tripod_head_yoke_arm",
    )

    crown = model.part("crown")
    crown.visual(
        Cylinder(radius=0.040, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=dark_graphite,
        name="hub_shell",
    )
    crown.visual(
        Cylinder(radius=0.018, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=dark_graphite,
        name="pan_spigot",
    )
    crown.visual(
        Cylinder(radius=0.028, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=matte_black,
        name="pan_seat",
    )

    hinge_radius = 0.082
    hinge_height = -0.014
    leg_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    for index, angle in enumerate(leg_angles):
        c = math.cos(angle)
        s = math.sin(angle)
        crown.visual(
            crown_arm_mesh,
            origin=Origin(rpy=(0.0, 0.0, angle)),
            material=matte_black,
            name=f"crown_arm_{index}",
        )
        crown.visual(
            Box((0.024, 0.028, 0.018)),
            origin=Origin(
                xyz=(0.060 * c, 0.060 * s, hinge_height),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_graphite,
            name=f"leg_hinge_block_{index}",
        )
    crown.inertial = Inertial.from_geometry(
        Box((0.18, 0.18, 0.12)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
    )

    for index, angle in enumerate(leg_angles):
        leg = model.part(f"leg_{index}")
        leg.visual(
            Cylinder(radius=0.010, length=0.030),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=aluminum,
            name="hinge_knuckle",
        )
        leg.visual(
            leg_tube_mesh,
            material=aluminum,
            name="leg_tube",
        )
        leg.visual(
            Cylinder(radius=0.015, length=0.032),
            origin=Origin(
                xyz=(0.320, 0.0, -0.900),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=rubber,
            name="foot",
        )
        leg.inertial = Inertial.from_geometry(
            Box((0.36, 0.05, 0.95)),
            mass=0.45,
            origin=Origin(xyz=(0.160, 0.0, -0.450)),
        )

        model.articulation(
            f"crown_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=crown,
            child=leg,
            origin=Origin(
                xyz=(hinge_radius * math.cos(angle), hinge_radius * math.sin(angle), hinge_height),
                rpy=(0.0, 0.0, angle),
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=35.0,
                velocity=1.2,
                lower=0.0,
                upper=math.radians(70.0),
            ),
        )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.046, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_graphite,
        name="rotary_stage",
    )
    pan_head.visual(
        Cylinder(radius=0.022, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=matte_black,
        name="center_column",
    )
    pan_head.visual(
        Cylinder(radius=0.028, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=dark_graphite,
        name="upper_hub",
    )
    pan_head.visual(
        yoke_arm_mesh,
        origin=Origin(xyz=(0.0, 0.040, 0.0)),
        material=matte_black,
        name="left_yoke_arm",
    )
    pan_head.visual(
        yoke_arm_mesh,
        origin=Origin(xyz=(0.0, -0.040, 0.0)),
        material=matte_black,
        name="right_yoke_arm",
    )
    pan_head.visual(
        Box((0.018, 0.080, 0.012)),
        origin=Origin(xyz=(0.008, 0.0, 0.058)),
        material=dark_graphite,
        name="yoke_bridge",
    )
    pan_head.visual(
        Box((0.018, 0.010, 0.024)),
        origin=Origin(xyz=(0.050, 0.048, 0.100)),
        material=dark_graphite,
        name="left_tilt_lug",
    )
    pan_head.visual(
        Box((0.018, 0.010, 0.024)),
        origin=Origin(xyz=(0.050, -0.048, 0.100)),
        material=dark_graphite,
        name="right_tilt_lug",
    )
    pan_head.inertial = Inertial.from_geometry(
        Box((0.12, 0.14, 0.14)),
        mass=0.55,
        origin=Origin(xyz=(0.025, 0.0, 0.050)),
    )

    model.articulation(
        "crown_to_pan_head",
        ArticulationType.CONTINUOUS,
        parent=crown,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 0.063)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
        ),
    )

    device_mount = model.part("device_mount")
    device_mount.visual(
        Cylinder(radius=0.008, length=0.074),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="tilt_shaft",
    )
    device_mount.visual(
        Box((0.024, 0.014, 0.046)),
        origin=Origin(xyz=(0.0, 0.024, 0.023)),
        material=matte_black,
        name="left_cheek",
    )
    device_mount.visual(
        Box((0.024, 0.014, 0.046)),
        origin=Origin(xyz=(0.0, -0.024, 0.023)),
        material=matte_black,
        name="right_cheek",
    )
    device_mount.visual(
        Box((0.084, 0.050, 0.012)),
        origin=Origin(xyz=(0.012, 0.0, 0.042)),
        material=dark_graphite,
        name="plate_support",
    )
    device_mount.visual(
        Box((0.092, 0.040, 0.012)),
        origin=Origin(xyz=(0.012, 0.0, 0.050)),
        material=aluminum,
        name="device_plate",
    )
    device_mount.visual(
        Box((0.110, 0.058, 0.070)),
        origin=Origin(xyz=(0.015, 0.0, 0.088)),
        material=matte_black,
        name="device_body",
    )
    device_mount.visual(
        Box((0.030, 0.020, 0.018)),
        origin=Origin(xyz=(-0.008, 0.0, 0.126)),
        material=dark_graphite,
        name="viewfinder",
    )
    device_mount.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(
            xyz=(0.079, 0.0, 0.088),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_graphite,
        name="lens_mount",
    )
    device_mount.visual(
        Cylinder(radius=0.023, length=0.050),
        origin=Origin(
            xyz=(0.104, 0.0, 0.088),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_graphite,
        name="lens_barrel",
    )
    device_mount.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(
            xyz=(0.138, 0.0, 0.088),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=rubber,
        name="lens_hood",
    )
    device_mount.inertial = Inertial.from_geometry(
        Box((0.18, 0.08, 0.16)),
        mass=1.1,
        origin=Origin(xyz=(0.040, 0.0, 0.080)),
    )

    model.articulation(
        "pan_head_to_device_mount",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=device_mount,
        origin=Origin(xyz=(0.050, 0.0, 0.100)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.8,
            lower=math.radians(-75.0),
            upper=math.radians(40.0),
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

    crown = object_model.get_part("crown")
    pan_head = object_model.get_part("pan_head")
    device_mount = object_model.get_part("device_mount")
    leg_0 = object_model.get_part("leg_0")
    pan_joint = object_model.get_articulation("crown_to_pan_head")
    tilt_joint = object_model.get_articulation("pan_head_to_device_mount")
    leg_joint = object_model.get_articulation("crown_to_leg_0")

    def _aabb_center(aabb):
        return (
            None
            if aabb is None
            else tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))
        )

    rest_foot = ctx.part_element_world_aabb(leg_0, elem="foot")
    with ctx.pose({leg_joint: math.radians(60.0)}):
        folded_foot = ctx.part_element_world_aabb(leg_0, elem="foot")
    ctx.check(
        "one leg folds upward from the crown",
        rest_foot is not None
        and folded_foot is not None
        and folded_foot[0][2] > rest_foot[0][2] + 0.30,
        details=f"rest_foot={rest_foot}, folded_foot={folded_foot}",
    )

    rest_pan_pos = ctx.part_world_position(device_mount)
    with ctx.pose({pan_joint: math.pi / 2.0}):
        panned_pos = ctx.part_world_position(device_mount)
    ctx.check(
        "device pans around the exposed vertical stage",
        rest_pan_pos is not None
        and panned_pos is not None
        and abs(panned_pos[2] - rest_pan_pos[2]) < 1e-6
        and abs(panned_pos[1] - rest_pan_pos[0]) < 0.01
        and abs(panned_pos[0]) < 0.01,
        details=f"rest_pan_pos={rest_pan_pos}, panned_pos={panned_pos}",
    )

    rest_lens = _aabb_center(ctx.part_element_world_aabb(device_mount, elem="lens_hood"))
    with ctx.pose({tilt_joint: math.radians(30.0)}):
        tilted_lens = _aabb_center(ctx.part_element_world_aabb(device_mount, elem="lens_hood"))
    ctx.check(
        "device tilts upward about the horizontal axis",
        rest_lens is not None
        and tilted_lens is not None
        and tilted_lens[2] > rest_lens[2] + 0.02,
        details=f"rest_lens={rest_lens}, tilted_lens={tilted_lens}",
    )

    ctx.expect_origin_gap(
        device_mount,
        crown,
        axis="z",
        min_gap=0.13,
        max_gap=0.18,
        name="device mount sits above the tripod crown",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
