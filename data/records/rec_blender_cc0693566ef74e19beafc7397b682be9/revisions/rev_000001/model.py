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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vacuum_blender")

    base_black = model.material("base_black", rgba=(0.12, 0.12, 0.13, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.20, 0.21, 0.23, 1.0))
    clear_smoke = model.material("clear_smoke", rgba=(0.78, 0.84, 0.88, 0.30))
    soft_black = model.material("soft_black", rgba=(0.16, 0.16, 0.17, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.74, 1.0))

    base = model.part("base")
    base_shell = LatheGeometry(
        [
            (0.0, 0.0),
            (0.086, 0.0),
            (0.108, 0.006),
            (0.121, 0.022),
            (0.126, 0.055),
            (0.124, 0.096),
            (0.112, 0.118),
            (0.099, 0.128),
            (0.0, 0.128),
        ],
        segments=72,
    )
    base.visual(
        _mesh("vacuum_blender_base_shell", base_shell),
        material=base_black,
        name="base_shell",
    )
    base.visual(
        Cylinder(radius=0.095, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.136)),
        material=dark_gray,
        name="base_lock_ring",
    )
    base.visual(
        Cylinder(radius=0.021, length=0.020),
        origin=Origin(xyz=(0.108, 0.0, 0.058), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="control_dial",
    )
    base.visual(
        Cylinder(radius=0.011, length=0.028),
        origin=Origin(xyz=(0.096, 0.0, 0.058), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="control_dial_stem",
    )
    base.visual(
        Box((0.030, 0.072, 0.046)),
        origin=Origin(xyz=(0.094, 0.0, 0.066)),
        material=dark_gray,
        name="control_panel",
    )
    base.visual(
        Box((0.020, 0.052, 0.010)),
        origin=Origin(xyz=(0.112, 0.0, 0.088)),
        material=soft_black,
        name="vacuum_toggle",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.260, 0.260, 0.150)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
    )

    jug = model.part("jug")
    jug_shell = LatheGeometry.from_shell_profiles(
        [
            (0.070, 0.0),
            (0.078, 0.018),
            (0.082, 0.060),
            (0.082, 0.220),
            (0.078, 0.285),
            (0.074, 0.310),
        ],
        [
            (0.0, 0.018),
            (0.060, 0.030),
            (0.072, 0.060),
            (0.072, 0.268),
            (0.068, 0.302),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    jug.visual(
        _mesh("vacuum_blender_jug_shell", jug_shell),
        material=clear_smoke,
        name="jug_shell",
    )
    jug.visual(
        Cylinder(radius=0.091, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=soft_black,
        name="jug_lock_collar",
    )
    jug.visual(
        Box((0.018, 0.024, 0.010)),
        origin=Origin(xyz=(0.0, 0.078, 0.018)),
        material=soft_black,
        name="lock_lug_left",
    )
    jug.visual(
        Box((0.018, 0.024, 0.010)),
        origin=Origin(xyz=(0.0, -0.078, 0.018)),
        material=soft_black,
        name="lock_lug_right",
    )
    jug_handle = tube_from_spline_points(
        [
            (0.078, 0.0, 0.246),
            (0.116, 0.0, 0.248),
            (0.145, 0.0, 0.205),
            (0.153, 0.0, 0.154),
            (0.142, 0.0, 0.099),
            (0.116, 0.0, 0.062),
            (0.078, 0.0, 0.074),
        ],
        radius=0.010,
        samples_per_segment=20,
        radial_segments=18,
        cap_ends=True,
    )
    jug.visual(
        _mesh("vacuum_blender_jug_handle", jug_handle),
        material=soft_black,
        name="jug_handle",
    )
    jug.visual(
        Cylinder(radius=0.012, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=soft_black,
        name="blade_bearing_boss",
    )
    jug.inertial = Inertial.from_geometry(
        Box((0.310, 0.200, 0.330)),
        mass=1.3,
        origin=Origin(xyz=(0.030, 0.0, 0.165)),
    )

    cap = model.part("seal_cap")
    cap.visual(
        Cylinder(radius=0.067, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=soft_black,
        name="cap_plug",
    )
    cap.visual(
        Cylinder(radius=0.090, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_gray,
        name="cap_top",
    )
    cap.visual(
        Cylinder(radius=0.018, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=soft_black,
        name="suction_connector_body",
    )
    cap.visual(
        Cylinder(radius=0.010, length=0.032),
        origin=Origin(xyz=(0.016, 0.0, 0.041), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=soft_black,
        name="suction_connector_cap",
    )
    cap.inertial = Inertial.from_geometry(
        Box((0.190, 0.180, 0.070)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    blade = model.part("blade")
    blade.visual(
        Cylinder(radius=0.007, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=steel,
        name="blade_spindle",
    )
    blade.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=steel,
        name="blade_hub",
    )
    blade.visual(
        Box((0.092, 0.010, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=steel,
        name="blade_long",
    )
    blade.visual(
        Box((0.066, 0.009, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.010), rpy=(0.0, 0.0, math.pi / 2.0)),
        material=steel,
        name="blade_cross",
    )
    blade.inertial = Inertial.from_geometry(
        Box((0.100, 0.080, 0.040)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
    )

    model.articulation(
        "base_to_jug",
        ArticulationType.REVOLUTE,
        parent=base,
        child=jug,
        origin=Origin(xyz=(0.0, 0.0, 0.147)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.0,
            lower=0.0,
            upper=0.52,
        ),
    )
    model.articulation(
        "jug_to_seal_cap",
        ArticulationType.PRISMATIC,
        parent=jug,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.310)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.06,
            lower=0.0,
            upper=0.018,
        ),
    )
    model.articulation(
        "jug_to_blade",
        ArticulationType.CONTINUOUS,
        parent=jug,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=30.0,
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

    base = object_model.get_part("base")
    jug = object_model.get_part("jug")
    cap = object_model.get_part("seal_cap")
    blade = object_model.get_part("blade")

    jug_lock = object_model.get_articulation("base_to_jug")
    cap_slide = object_model.get_articulation("jug_to_seal_cap")
    blade_spin = object_model.get_articulation("jug_to_blade")

    ctx.expect_gap(
        jug,
        base,
        axis="z",
        positive_elem="jug_lock_collar",
        negative_elem="base_lock_ring",
        max_gap=0.0015,
        max_penetration=1e-6,
        name="jug collar seats on base ring",
    )
    ctx.expect_overlap(
        jug,
        base,
        axes="xy",
        elem_a="jug_lock_collar",
        elem_b="base_lock_ring",
        min_overlap=0.16,
        name="jug collar stays centered over the base lock ring",
    )
    ctx.expect_gap(
        cap,
        jug,
        axis="z",
        positive_elem="cap_top",
        negative_elem="jug_shell",
        max_gap=0.0015,
        max_penetration=0.0,
        name="seal cap seats on the jug rim",
    )

    jug_limits = jug_lock.motion_limits
    ctx.check(
        "jug twist-lock uses a short vertical revolute joint",
        jug_lock.articulation_type == ArticulationType.REVOLUTE
        and jug_lock.axis == (0.0, 0.0, 1.0)
        and jug_limits is not None
        and jug_limits.lower == 0.0
        and jug_limits.upper is not None
        and jug_limits.upper <= 0.65,
        details=f"type={jug_lock.articulation_type}, axis={jug_lock.axis}, limits={jug_limits}",
    )

    handle_rest_aabb = ctx.part_element_world_aabb(jug, elem="jug_handle")
    handle_twisted_aabb = None
    with ctx.pose({jug_lock: jug_limits.upper if jug_limits and jug_limits.upper is not None else 0.0}):
        handle_twisted_aabb = ctx.part_element_world_aabb(jug, elem="jug_handle")
    handle_rest_y = None
    handle_twisted_y = None
    if handle_rest_aabb is not None:
        handle_rest_y = 0.5 * (handle_rest_aabb[0][1] + handle_rest_aabb[1][1])
    if handle_twisted_aabb is not None:
        handle_twisted_y = 0.5 * (handle_twisted_aabb[0][1] + handle_twisted_aabb[1][1])
    ctx.check(
        "jug handle rotates around the twist lock",
        handle_rest_y is not None
        and handle_twisted_y is not None
        and handle_twisted_y > handle_rest_y + 0.025,
        details=f"rest_y={handle_rest_y}, twisted_y={handle_twisted_y}",
    )

    cap_rest = ctx.part_world_position(cap)
    cap_open = None
    with ctx.pose({cap_slide: cap_slide.motion_limits.upper if cap_slide.motion_limits and cap_slide.motion_limits.upper is not None else 0.0}):
        cap_open = ctx.part_world_position(cap)
    ctx.check(
        "seal cap lifts upward on its prismatic fit",
        cap_rest is not None and cap_open is not None and cap_open[2] > cap_rest[2] + 0.015,
        details=f"rest={cap_rest}, open={cap_open}",
    )

    blade_limits = blade_spin.motion_limits
    blade_rest_aabb = ctx.part_element_world_aabb(blade, elem="blade_long")
    blade_turned_aabb = None
    with ctx.pose({blade_spin: math.pi / 2.0}):
        blade_turned_aabb = ctx.part_element_world_aabb(blade, elem="blade_long")

    rest_dx = rest_dy = turned_dx = turned_dy = None
    if blade_rest_aabb is not None:
        rest_dx = blade_rest_aabb[1][0] - blade_rest_aabb[0][0]
        rest_dy = blade_rest_aabb[1][1] - blade_rest_aabb[0][1]
    if blade_turned_aabb is not None:
        turned_dx = blade_turned_aabb[1][0] - blade_turned_aabb[0][0]
        turned_dy = blade_turned_aabb[1][1] - blade_turned_aabb[0][1]

    ctx.check(
        "blade is configured as a continuous vertical spin",
        blade_spin.articulation_type == ArticulationType.CONTINUOUS
        and blade_spin.axis == (0.0, 0.0, 1.0)
        and blade_limits is not None
        and blade_limits.lower is None
        and blade_limits.upper is None
        and rest_dx is not None
        and rest_dy is not None
        and turned_dx is not None
        and turned_dy is not None
        and rest_dx > rest_dy
        and turned_dy > turned_dx,
        details=(
            f"type={blade_spin.articulation_type}, axis={blade_spin.axis}, "
            f"rest=({rest_dx}, {rest_dy}), turned=({turned_dx}, {turned_dy})"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
