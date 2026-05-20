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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_pan_tilt_mount")

    matte_black = model.material("matte_black", rgba=(0.16, 0.17, 0.18, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.23, 0.24, 0.26, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.09, 0.10, 1.0))

    crown = model.part("crown")
    crown.visual(
        Cylinder(radius=0.022, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, 0.210)),
        material=matte_black,
        name="center_column",
    )
    crown.visual(
        Cylinder(radius=0.060, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.375)),
        material=dark_graphite,
        name="crown_core",
    )
    crown.visual(
        Cylinder(radius=0.078, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.410)),
        material=dark_graphite,
        name="crown_shoulder",
    )
    crown.visual(
        Cylinder(radius=0.030, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.429)),
        material=matte_black,
        name="head_riser",
    )
    crown.visual(
        Box((0.120, 0.090, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.447)),
        material=dark_graphite,
        name="support_deck",
    )
    crown.inertial = Inertial.from_geometry(
        Box((0.220, 0.220, 0.500)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.250)),
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.028, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=matte_black,
        name="pan_rotor",
    )
    head.visual(
        Box((0.030, 0.058, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=matte_black,
        name="neck_block",
    )
    head.visual(
        Box((0.024, 0.012, 0.060)),
        origin=Origin(xyz=(0.0, 0.028, 0.076)),
        material=matte_black,
        name="left_yoke_arm",
    )
    head.visual(
        Box((0.024, 0.012, 0.060)),
        origin=Origin(xyz=(0.0, -0.028, 0.076)),
        material=matte_black,
        name="right_yoke_arm",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.070, 0.090, 0.110)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )

    device_plate = model.part("device_plate")
    device_plate.visual(
        Box((0.020, 0.044, 0.024)),
        material=matte_black,
        name="pivot_block",
    )
    device_plate.visual(
        Box((0.026, 0.032, 0.050)),
        origin=Origin(xyz=(0.020, 0.0, 0.037)),
        material=matte_black,
        name="tilt_upright",
    )
    device_plate.visual(
        Box((0.140, 0.070, 0.008)),
        origin=Origin(xyz=(0.050, 0.0, 0.066)),
        material=dark_graphite,
        name="mount_plate",
    )
    device_plate.visual(
        Box((0.118, 0.048, 0.003)),
        origin=Origin(xyz=(0.050, 0.0, 0.0715)),
        material=rubber,
        name="plate_pad",
    )
    device_plate.visual(
        Box((0.008, 0.040, 0.018)),
        origin=Origin(xyz=(-0.018, 0.0, 0.075)),
        material=matte_black,
        name="rear_fence",
    )
    device_plate.inertial = Inertial.from_geometry(
        Box((0.150, 0.080, 0.100)),
        mass=0.30,
        origin=Origin(xyz=(0.020, 0.0, 0.050)),
    )

    model.articulation(
        "crown_to_head_pan",
        ArticulationType.CONTINUOUS,
        parent=crown,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.460)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0),
    )
    model.articulation(
        "head_to_device_tilt",
        ArticulationType.REVOLUTE,
        parent=head,
        child=device_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.086)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.6,
            lower=-1.0,
            upper=1.2,
        ),
    )

    leg_strut = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.008, 0.0, -0.006),
                (0.165, 0.0, -0.080),
                (0.340, 0.0, -0.360),
            ],
            radius=0.012,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
        "tripod_leg_strut",
    )

    leg_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    hinge_radius = 0.072
    hinge_block_radius = 0.060
    for index, angle in enumerate(leg_angles):
        c = math.cos(angle)
        s = math.sin(angle)

        crown.visual(
            Box((0.024, 0.032, 0.026)),
            origin=Origin(
                xyz=(hinge_block_radius * c, hinge_block_radius * s, 0.378),
                rpy=(0.0, 0.0, angle),
            ),
            material=matte_black,
            name=f"hinge_block_{index}",
        )

        leg = model.part(f"leg_{index}")
        leg.visual(
            Cylinder(radius=0.010, length=0.028),
            origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=matte_black,
            name="leg_mount",
        )
        leg.visual(
            leg_strut,
            material=matte_black,
            name="leg_strut",
        )
        leg.visual(
            Sphere(radius=0.016),
            origin=Origin(xyz=(0.340, 0.0, -0.360)),
            material=rubber,
            name="foot_pad",
        )
        leg.inertial = Inertial.from_geometry(
            Box((0.380, 0.050, 0.390)),
            mass=0.48,
            origin=Origin(xyz=(0.170, 0.0, -0.180)),
        )

        model.articulation(
            f"crown_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=crown,
            child=leg,
            origin=Origin(
                xyz=(hinge_radius * c, hinge_radius * s, 0.378),
                rpy=(0.0, 0.0, angle),
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=18.0,
                velocity=1.5,
                lower=0.0,
                upper=1.05,
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
    head = object_model.get_part("head")
    device_plate = object_model.get_part("device_plate")
    leg_0 = object_model.get_part("leg_0")
    leg_1 = object_model.get_part("leg_1")
    leg_2 = object_model.get_part("leg_2")
    pan = object_model.get_articulation("crown_to_head_pan")
    tilt = object_model.get_articulation("head_to_device_tilt")
    leg_0_hinge = object_model.get_articulation("crown_to_leg_0")

    def aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))

    support_aabb = ctx.part_element_world_aabb(crown, elem="support_deck")
    rotor_aabb = ctx.part_element_world_aabb(head, elem="pan_rotor")
    if support_aabb is not None and rotor_aabb is not None:
        support_dims = tuple(hi - lo for lo, hi in zip(*support_aabb))
        rotor_dims = tuple(hi - lo for lo, hi in zip(*rotor_aabb))
        ctx.check(
            "fixed support stays broader than rotating pan member",
            support_dims[0] > rotor_dims[0] * 1.8 and support_dims[1] > rotor_dims[1] * 1.6,
            details=f"support_dims={support_dims}, rotor_dims={rotor_dims}",
        )
    else:
        ctx.fail("fixed support stays broader than rotating pan member", "missing support or rotor AABB")

    ctx.expect_gap(
        device_plate,
        crown,
        axis="z",
        positive_elem="mount_plate",
        negative_elem="support_deck",
        min_gap=0.12,
        name="device plate clears the crown support deck",
    )

    rest_plate_center = aabb_center(ctx.part_element_world_aabb(device_plate, elem="mount_plate"))
    with ctx.pose({pan: math.pi / 2.0}):
        panned_plate_center = aabb_center(ctx.part_element_world_aabb(device_plate, elem="mount_plate"))
    ctx.check(
        "head pan swings the plate around vertical",
        rest_plate_center is not None
        and panned_plate_center is not None
        and rest_plate_center[0] > 0.015
        and abs(rest_plate_center[1]) < 0.010
        and panned_plate_center[1] > 0.015
        and abs(panned_plate_center[0]) < 0.010,
        details=f"rest_plate_center={rest_plate_center}, panned_plate_center={panned_plate_center}",
    )

    neutral_plate_center = aabb_center(ctx.part_element_world_aabb(device_plate, elem="mount_plate"))
    with ctx.pose({tilt: 0.5}):
        tilted_plate_center = aabb_center(ctx.part_element_world_aabb(device_plate, elem="mount_plate"))
    ctx.check(
        "device plate tilts upward on the horizontal axis",
        neutral_plate_center is not None
        and tilted_plate_center is not None
        and tilted_plate_center[2] > neutral_plate_center[2] + 0.015,
        details=f"neutral_plate_center={neutral_plate_center}, tilted_plate_center={tilted_plate_center}",
    )

    foot_centers = [
        aabb_center(ctx.part_element_world_aabb(leg, elem="foot_pad"))
        for leg in (leg_0, leg_1, leg_2)
    ]
    foot_radii = [
        math.hypot(center[0], center[1])
        for center in foot_centers
        if center is not None
    ]
    ctx.check(
        "tripod feet land in a broad three-point stance",
        len(foot_radii) == 3
        and min(foot_radii) > 0.32
        and max(abs(center[2] - 0.018) for center in foot_centers if center is not None) < 0.015,
        details=f"foot_centers={foot_centers}, foot_radii={foot_radii}",
    )

    rest_leg_aabb = ctx.part_world_aabb(leg_0)
    with ctx.pose({leg_0_hinge: leg_0_hinge.motion_limits.upper}):
        folded_leg_aabb = ctx.part_world_aabb(leg_0)
    rest_leg_min_z = rest_leg_aabb[0][2] if rest_leg_aabb is not None else None
    folded_leg_min_z = folded_leg_aabb[0][2] if folded_leg_aabb is not None else None
    ctx.check(
        "a tripod leg folds upward toward the crown",
        rest_leg_min_z is not None
        and folded_leg_min_z is not None
        and folded_leg_min_z > rest_leg_min_z + 0.18,
        details=f"rest_leg_min_z={rest_leg_min_z}, folded_leg_min_z={folded_leg_min_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
