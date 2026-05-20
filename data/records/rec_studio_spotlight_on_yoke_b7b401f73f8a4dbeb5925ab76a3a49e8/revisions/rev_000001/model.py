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
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _radial_bar_origin(angle: float, inner_radius: float, outer_radius: float, z: float) -> Origin:
    length = outer_radius - inner_radius
    mid_radius = inner_radius + 0.5 * length
    return Origin(
        xyz=(mid_radius * math.cos(angle), mid_radius * math.sin(angle), z),
        rpy=(0.0, math.pi / 2.0, angle),
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_spotlight_yoke_stand")

    powder_black = model.material("powder_black", rgba=(0.15, 0.15, 0.16, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.68, 0.70, 0.73, 1.0))
    lamp_gray = model.material("lamp_gray", rgba=(0.24, 0.26, 0.29, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.74, 0.84, 0.92, 0.68))
    handle_black = model.material("handle_black", rgba=(0.09, 0.09, 0.10, 1.0))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.24, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=powder_black,
        name="base_plate",
    )
    stand.visual(
        Cylinder(radius=0.12, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=powder_black,
        name="base_skirt",
    )
    stand.visual(
        Cylinder(radius=0.028, length=0.70),
        origin=Origin(xyz=(0.0, 0.0, 0.405)),
        material=powder_black,
        name="mast",
    )
    stand.visual(
        Cylinder(radius=0.072, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, 0.785)),
        material=lamp_gray,
        name="bearing_housing",
    )

    guard_ring = _mesh(
        "spotlight_guard_ring",
        TorusGeometry(radius=0.25, tube=0.009, radial_segments=18, tubular_segments=72),
    )
    stand.visual(
        guard_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.915)),
        material=satin_steel,
        name="guard_ring",
    )

    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        stand.visual(
            Cylinder(radius=0.007, length=0.058),
            origin=_radial_bar_origin(angle, 0.072, 0.13, 0.81),
            material=satin_steel,
            name=f"guard_lower_support_{index}",
        )
        stand.visual(
            Cylinder(radius=0.007, length=0.11),
            origin=Origin(xyz=(0.13 * math.cos(angle), 0.13 * math.sin(angle), 0.865)),
            material=satin_steel,
            name=f"guard_post_{index}",
        )
        stand.visual(
            Cylinder(radius=0.007, length=0.12),
            origin=_radial_bar_origin(angle, 0.13, 0.25, 0.915),
            material=satin_steel,
            name=f"guard_upper_support_{index}",
        )

    stand.inertial = Inertial.from_geometry(
        Box((0.56, 0.56, 0.86)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.43)),
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.105, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=powder_black,
        name="pan_turntable",
    )
    yoke.visual(
        Cylinder(radius=0.055, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, 0.0875)),
        material=lamp_gray,
        name="center_spine",
    )

    yoke_hoop = _mesh(
        "spotlight_yoke_hoop",
        tube_from_spline_points(
            [
                (0.03, 0.150, 0.44),
                (0.043, 0.165, 0.52),
                (0.050, 0.135, 0.60),
                (0.050, 0.000, 0.64),
                (0.050, -0.135, 0.60),
                (0.043, -0.165, 0.52),
                (0.03, -0.150, 0.44),
            ],
            radius=0.016,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
    )
    yoke.visual(yoke_hoop, material=satin_steel, name="yoke_hoop")

    left_brace = _mesh(
        "spotlight_left_yoke_brace",
        tube_from_spline_points(
            [
                (0.0, 0.0, 0.12),
                (0.010, 0.075, 0.24),
                (0.022, 0.126, 0.35),
                (0.03, 0.150, 0.44),
            ],
            radius=0.012,
            samples_per_segment=14,
            radial_segments=16,
            cap_ends=True,
        ),
    )
    right_brace = _mesh(
        "spotlight_right_yoke_brace",
        tube_from_spline_points(
            [
                (0.0, 0.0, 0.12),
                (0.010, -0.075, 0.24),
                (0.022, -0.126, 0.35),
                (0.03, -0.150, 0.44),
            ],
            radius=0.012,
            samples_per_segment=14,
            radial_segments=16,
            cap_ends=True,
        ),
    )
    yoke.visual(left_brace, material=satin_steel, name="left_brace")
    yoke.visual(right_brace, material=satin_steel, name="right_brace")
    yoke.visual(
        Cylinder(radius=0.022, length=0.026),
        origin=Origin(xyz=(0.03, 0.135, 0.44), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lamp_gray,
        name="left_trunnion",
    )
    yoke.visual(
        Cylinder(radius=0.022, length=0.026),
        origin=Origin(xyz=(0.03, -0.135, 0.44), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lamp_gray,
        name="right_trunnion",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.25, 0.38, 0.70)),
        mass=4.5,
        origin=Origin(xyz=(0.03, 0.0, 0.33)),
    )

    lamp = model.part("lamp")
    lamp.visual(
        Cylinder(radius=0.088, length=0.26),
        origin=Origin(xyz=(0.02, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lamp_gray,
        name="can_shell",
    )
    lamp.visual(
        Cylinder(radius=0.064, length=0.09),
        origin=Origin(xyz=(-0.115, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_black,
        name="rear_cap",
    )
    lamp.visual(
        Cylinder(radius=0.103, length=0.04),
        origin=Origin(xyz=(0.16, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_black,
        name="front_bezel",
    )

    front_ring = _mesh(
        "spotlight_front_ring",
        TorusGeometry(radius=0.103, tube=0.007, radial_segments=14, tubular_segments=48).rotate_y(
            math.pi / 2.0
        ),
    )
    lamp.visual(
        front_ring,
        origin=Origin(xyz=(0.18, 0.0, 0.0)),
        material=satin_steel,
        name="front_ring",
    )
    lamp.visual(
        Cylinder(radius=0.094, length=0.004),
        origin=Origin(xyz=(0.178, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="lens",
    )
    lamp.visual(
        Cylinder(radius=0.016, length=0.034),
        origin=Origin(xyz=(0.03, 0.105, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lamp_gray,
        name="left_hub",
    )
    lamp.visual(
        Cylinder(radius=0.016, length=0.034),
        origin=Origin(xyz=(0.03, -0.105, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lamp_gray,
        name="right_hub",
    )
    lamp.visual(
        Box((0.14, 0.018, 0.020)),
        origin=Origin(xyz=(-0.005, 0.0, 0.094)),
        material=handle_black,
        name="top_handle",
    )
    lamp.inertial = Inertial.from_geometry(
        Box((0.36, 0.24, 0.24)),
        mass=5.0,
        origin=Origin(xyz=(0.03, 0.0, 0.0)),
    )

    model.articulation(
        "stand_to_yoke_pan",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.83)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.4,
            lower=-2.8,
            upper=2.8,
        ),
    )
    model.articulation(
        "yoke_to_lamp_tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=lamp,
        origin=Origin(xyz=(0.03, 0.0, 0.44)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=1.2,
            lower=-0.9,
            upper=1.15,
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
    yoke = object_model.get_part("yoke")
    lamp = object_model.get_part("lamp")
    pan = object_model.get_articulation("stand_to_yoke_pan")
    tilt = object_model.get_articulation("yoke_to_lamp_tilt")

    ctx.expect_contact(
        yoke,
        lamp,
        elem_a="left_trunnion",
        elem_b="left_hub",
        name="left tilt hub seats against left yoke trunnion",
    )
    ctx.expect_contact(
        yoke,
        lamp,
        elem_a="right_trunnion",
        elem_b="right_hub",
        name="right tilt hub seats against right yoke trunnion",
    )
    ctx.expect_within(
        yoke,
        stand,
        axes="xy",
        inner_elem="pan_turntable",
        outer_elem="guard_ring",
        margin=0.0,
        name="pan turntable stays inside fixed guard ring footprint",
    )

    rest_front = _aabb_center(ctx.part_element_world_aabb(lamp, elem="front_ring"))
    with ctx.pose({pan: 1.2}):
        panned_front = _aabb_center(ctx.part_element_world_aabb(lamp, elem="front_ring"))
    ctx.check(
        "positive pan swings the lamp head around the stand",
        rest_front is not None
        and panned_front is not None
        and abs(panned_front[1] - rest_front[1]) > 0.15,
        details=f"rest_front={rest_front}, panned_front={panned_front}",
    )

    with ctx.pose({tilt: 0.9}):
        tilted_front = _aabb_center(ctx.part_element_world_aabb(lamp, elem="front_ring"))
    ctx.check(
        "positive tilt lifts the lamp nose upward",
        rest_front is not None
        and tilted_front is not None
        and tilted_front[2] > rest_front[2] + 0.10,
        details=f"rest_front={rest_front}, tilted_front={tilted_front}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
